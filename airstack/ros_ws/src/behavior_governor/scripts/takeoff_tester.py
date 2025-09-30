#!/usr/bin/env python3
import argparse
import math
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Float64
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, HomePosition
from mavros_msgs.srv import CommandBool, SetMode, ParamGet


class TakeoffTester(Node):
    def __init__(self, governor_ns: str, mavros_ns: str, use_direct_mavros: bool,
                 arm_timeout: float, ascend_timeout: float, wait_gps_timeout: float,
                 disarm_on_exit: bool):
        super().__init__('takeoff_tester')

        self.gov_ns = governor_ns.rstrip('/')
        self.mav_ns = mavros_ns.rstrip('/')
        self.use_direct = use_direct_mavros
        self.arm_timeout = arm_timeout
        self.ascend_timeout = ascend_timeout
        self.wait_gps_timeout = wait_gps_timeout
        self.disarm_on_exit = disarm_on_exit

        # State
        self._state = State()
        self._home_ok = False
        self._navfix_ok = False
        self._rel_alt = None
        self._gov_status = None

        # QoS
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        be_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.create_subscription(State, f'{self.mav_ns}/state', self._on_state, reliable_qos)
        self.create_subscription(HomePosition, f'{self.mav_ns}/home_position/home', self._on_home, reliable_qos)
        self.create_subscription(NavSatFix, f'{self.mav_ns}/global_position/global', self._on_navfix, be_qos)
        self.create_subscription(Float64, f'{self.mav_ns}/global_position/rel_alt', self._on_rel_alt, be_qos)
        self.create_subscription(String, f'{self.gov_ns}/status', self._on_gov_status, reliable_qos)

        # Publisher to governor command topic
        self.cmd_pub = self.create_publisher(String, f'{self.gov_ns}/command', 10)

        # MAVROS service clients (for direct path or param)
        self.cli_arm = self.create_client(CommandBool, f'{self.mav_ns}/cmd/arming')
        self.cli_mode = self.create_client(SetMode, f'{self.mav_ns}/set_mode')
        self.cli_param_get = self.create_client(ParamGet, f'{self.mav_ns}/param/get')

        # Signal handler for clean exit
        signal.signal(signal.SIGINT, self._sigint)

    # ---------- Callbacks ----------
    def _on_state(self, msg: State):
        self._state = msg

    def _on_home(self, msg: HomePosition):
        self._home_ok = True

    def _on_navfix(self, msg: NavSatFix):
        # NavSatStatus: -1 = no fix, 0 = fix, 1 = SBAS, 2 = GBAS
        self._navfix_ok = (msg.status.status is not None) and (msg.status.status >= 0)

    def _on_rel_alt(self, msg: Float64):
        self._rel_alt = msg.data

    def _on_gov_status(self, msg: String):
        self._gov_status = msg.data
        self.get_logger().info(f'[governor/status] {msg.data}')

    # ---------- Helpers ----------
    def _spin_wait(self, pred, timeout_s: float, desc: str, poll: float = 0.1) -> bool:
        start = time.time()
        while (time.time() - start) < timeout_s:
            rclpy.spin_once(self, timeout_sec=poll)
            if pred():
                return True
        self.get_logger().error(f'Timeout waiting for: {desc}')
        return False

    def _publish_cmd(self, text: str):
        msg = String()
        msg.data = text
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'[governor/command] -> {text}')

    def _get_mis_takeoff_alt(self, default_alt: float = 2.5) -> float:
        # Try to read PX4 param MIS_TAKEOFF_ALT (meters above home)
        if not self.cli_param_get.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('param/get not available, fallback MIS_TAKEOFF_ALT default')
            return default_alt
        req = ParamGet.Request()
        req.param_id = 'MIS_TAKEOFF_ALT'
        fut = self.cli_param_get.call_async(req)
        if not self._spin_wait(lambda: fut.done(), 3.0, 'ParamGet(MIS_TAKEOFF_ALT)'):
            return default_alt
        resp = fut.result()
        if resp and resp.success:
            # ParamValue has .integer and .real; take .real if non-zero else cast integer
            val = resp.value.real if (not math.isclose(resp.value.real, 0.0)) else float(resp.value.integer)
            if val <= 0.0:
                self.get_logger().warn(f'MIS_TAKEOFF_ALT returned {val}, using default {default_alt}')
                return default_alt
            self.get_logger().info(f'MIS_TAKEOFF_ALT = {val:.2f} m')
            return float(val)
        self.get_logger().warn('ParamGet failed, using default')
        return default_alt

    # ---------- MAVROS direct path (optional) ----------
    def _direct_arm(self, arm: bool) -> bool:
        if not self.cli_arm.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('cmd/arming service not available')
            return False
        req = CommandBool.Request()
        req.value = arm
        fut = self.cli_arm.call_async(req)
        ok = self._spin_wait(lambda: fut.done(), self.arm_timeout, 'direct arming')
        return bool(ok and fut.result() and fut.result().success)

    def _direct_set_mode(self, mode: str) -> bool:
        if not self.cli_mode.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('set_mode service not available')
            return False
        req = SetMode.Request()
        req.custom_mode = mode  # e.g., 'AUTO.TAKEOFF'
        fut = self.cli_mode.call_async(req)
        ok = self._spin_wait(lambda: fut.done(), 3.0, f'set_mode {mode}')
        return bool(ok and fut.result() and fut.result().mode_sent)

    # ---------- Main flow ----------
    def run_takeoff(self):
        self.get_logger().info('--- TAKEOFF TEST START ---')

        # 1) Wait FCU connection
        ok = self._spin_wait(lambda: getattr(self._state, 'connected', False), 15.0, 'MAVROS FCU connection')
        if not ok:
            return False
        self.get_logger().info('MAVROS connected.')

        # 2) GPS/home readiness (PX4 auto takeoff requires global position + home set)
        ok = self._spin_wait(lambda: self._navfix_ok, self.wait_gps_timeout, 'GPS fix')
        if not ok:
            return False
        ok = self._spin_wait(lambda: self._home_ok, self.wait_gps_timeout, 'home position')
        if not ok:
            return False
        self.get_logger().info('GPS + Home ready.')

        # 3) Arm
        if not self._state.armed:
            if self.use_direct:
                self.get_logger().info('Arming (direct MAVROS)...')
                if not self._direct_arm(True):
                    self.get_logger().error('Direct arming failed.')
                    return False
            else:
                self._publish_cmd('arm')
                ok = self._spin_wait(lambda: self._state.armed, self.arm_timeout, 'armed')
                if not ok:
                    self.get_logger().error('Arming via governor timed out.')
                    return False
        self.get_logger().info('Armed.')

        # 4) Expected takeoff altitude (relative)
        expected_alt = self._get_mis_takeoff_alt(default_alt=2.5)
        target_reached_threshold = max(1.0, 0.8 * expected_alt)  # allow margin

        # 5) Trigger takeoff
        if self.use_direct:
            self.get_logger().info("Setting mode 'AUTO.TAKEOFF' (direct MAVROS)...")
            if not self._direct_set_mode('AUTO.TAKEOFF'):
                self.get_logger().error('Failed to set AUTO.TAKEOFF')
                return False
        else:
            self._publish_cmd('takeoff')

        # 6) Watch climb
        start_alt = self._rel_alt if self._rel_alt is not None else 0.0
        self.get_logger().info(f'Start rel_alt = {start_alt:.2f} m; waiting to exceed {target_reached_threshold:.2f} m')

        # First: ensure positive climb within 10s
        def climbing():
            return (self._rel_alt is not None) and (self._rel_alt > start_alt + 0.5)

        if not self._spin_wait(climbing, 10.0, 'initial climb (Δalt > 0.5m)'):
            self.get_logger().error('No initial climb detected; takeoff likely not engaged.')
            return False

        # Then: wait to reach target threshold
        ok = self._spin_wait(lambda: (self._rel_alt or 0.0) >= target_reached_threshold,
                             self.ascend_timeout, 'reach target altitude')
        if not ok:
            self.get_logger().error('Takeoff altitude not reached in time.')
            return False

        self.get_logger().info(f'✅ Takeoff success: rel_alt = {self._rel_alt:.2f} m (target≈{expected_alt:.2f} m)')
        return True

    def _sigint(self, *_):
        self.get_logger().warn('SIGINT — exiting…')
        if self.disarm_on_exit and self._state.armed:
            if self.use_direct:
                self._direct_arm(False)
            else:
                self._publish_cmd('land')  # safer than instant disarm in air; use 'disarm' on ground.
        rclpy.shutdown()
        sys.exit(0)


def main():
    parser = argparse.ArgumentParser(description='Behavior governor TAKEOFF tester')
    parser.add_argument('--governor-ns', default='/behavior_governor', help='Governor namespace')
    parser.add_argument('--mavros-ns', default='/mavros', help='MAVROS namespace')
    parser.add_argument('--direct-mavros', action='store_true',
                        help='Bypass governor: arm + AUTO.TAKEOFF via MAVROS directly')
    parser.add_argument('--arm-timeout', type=float, default=10.0)
    parser.add_argument('--ascend-timeout', type=float, default=60.0)
    parser.add_argument('--wait-gps-timeout', type=float, default=30.0)
    parser.add_argument('--disarm-on-exit', action='store_true', help='Disarm/Land on Ctrl+C')
    args = parser.parse_args()

    rclpy.init()
    node = TakeoffTester(args.governor_ns, args.mavros_ns, args.direct_mavros,
                         args.arm_timeout, args.ascend_timeout, args.wait_gps_timeout,
                         args.disarm_on_exit)
    ok = node.run_takeoff()
    node.get_logger().info('DONE' if ok else 'FAILED')
    time.sleep(0.5)  # flush logs
    rclpy.shutdown()
    sys.exit(0 if ok else 2)


if __name__ == '__main__':
    main()
