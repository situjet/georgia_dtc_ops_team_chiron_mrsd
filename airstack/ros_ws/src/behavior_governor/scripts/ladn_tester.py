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
from mavros_msgs.msg import State, ExtendedState, HomePosition
from mavros_msgs.srv import CommandBool, SetMode, ParamGet
# Optional alternative landing path:
# from mavros_msgs.srv import CommandTOL  # if you want to use /mavros/cmd/land

LANDED_STATE_UNDEFINED = 0
LANDED_STATE_ON_GROUND = 1
LANDED_STATE_IN_AIR = 2
LANDED_STATE_TAKEOFF = 3
LANDED_STATE_LANDING = 4


class LandTester(Node):
    def __init__(self, governor_ns: str, mavros_ns: str, use_direct_mavros: bool,
                 descend_timeout: float, initial_descent_timeout: float,
                 ground_dwell: float, wait_connect_timeout: float,
                 check_auto_disarm: bool):
        super().__init__('land_tester')

        self.gov_ns = governor_ns.rstrip('/')
        self.mav_ns = mavros_ns.rstrip('/')
        self.use_direct = use_direct_mavros
        self.descend_timeout = descend_timeout
        self.initial_descent_timeout = initial_descent_timeout
        self.ground_dwell = ground_dwell
        self.wait_connect_timeout = wait_connect_timeout
        self.check_auto_disarm = check_auto_disarm

        # State
        self._state = State()
        self._ext_state = ExtendedState()
        self._rel_alt = None
        self._home_ok = False
        self._navfix_ok = False
        self._navfix = None
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
        self.create_subscription(ExtendedState, f'{self.mav_ns}/extended_state', self._on_ext_state, reliable_qos)
        self.create_subscription(Float64, f'{self.mav_ns}/global_position/rel_alt', self._on_rel_alt, be_qos)
        self.create_subscription(HomePosition, f'{self.mav_ns}/home_position/home', self._on_home, reliable_qos)
        self.create_subscription(NavSatFix, f'{self.mav_ns}/global_position/global', self._on_navfix, be_qos)
        self.create_subscription(String, f'{self.gov_ns}/status', self._on_gov_status, reliable_qos)

        # Governor command publisher
        self.cmd_pub = self.create_publisher(String, f'{self.gov_ns}/command', 10)

        # MAVROS service clients (direct path + params)
        self.cli_mode = self.create_client(SetMode, f'{self.mav_ns}/set_mode')
        self.cli_param_get = self.create_client(ParamGet, f'{self.mav_ns}/param/get')
        self.cli_arm = self.create_client(CommandBool, f'{self.mav_ns}/cmd/arming')
        # self.cli_land = self.create_client(CommandTOL, f'{self.mav_ns}/cmd/land')  # optional

        # Signal handler
        signal.signal(signal.SIGINT, self._sigint)

    # ---------- Callbacks ----------
    def _on_state(self, msg: State):
        self._state = msg

    def _on_ext_state(self, msg: ExtendedState):
        self._ext_state = msg

    def _on_rel_alt(self, msg: Float64):
        self._rel_alt = msg.data

    def _on_home(self, msg: HomePosition):
        self._home_ok = True

    def _on_navfix(self, msg: NavSatFix):
        self._navfix = msg
        self._navfix_ok = (msg.status.status is not None) and (msg.status.status >= 0)

    def _on_gov_status(self, msg: String):
        self._gov_status = msg.data
        self.get_logger().info(f'[governor/status] {msg.data}')

    # ---------- Helpers ----------
    def _spin_once(self, dt=0.1):
        rclpy.spin_once(self, timeout_sec=dt)

    def _spin_wait(self, pred, timeout_s: float, desc: str, poll: float = 0.1) -> bool:
        start = time.time()
        while (time.time() - start) < timeout_s:
            self._spin_once(poll)
            if pred():
                return True
        self.get_logger().error(f'Timeout waiting for: {desc}')
        return False

    def _publish_cmd(self, text: str):
        msg = String()
        msg.data = text
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'[governor/command] -> {text}')

    def _get_com_disarm_land(self) -> float:
        """PX4 param: COM_DISARM_LAND (s) >0 means auto-disarm after landing."""
        if not self.check_auto_disarm:
            return 0.0
        if not self.cli_param_get.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('param/get not available; skip COM_DISARM_LAND check')
            return 0.0
        req = ParamGet.Request()
        req.param_id = 'COM_DISARM_LAND'
        fut = self.cli_param_get.call_async(req)
        if not self._spin_wait(lambda: fut.done(), 3.0, 'ParamGet(COM_DISARM_LAND)'):
            return 0.0
        resp = fut.result()
        if not resp or not resp.success:
            return 0.0
        val = resp.value.real if (not math.isclose(resp.value.real, 0.0)) else float(resp.value.integer)
        self.get_logger().info(f'COM_DISARM_LAND = {val:.2f} s')
        return max(0.0, float(val))

    def _direct_set_mode(self, mode: str) -> bool:
        if not self.cli_mode.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('set_mode service not available')
            return False
        req = SetMode.Request()
        req.custom_mode = mode
        fut = self.cli_mode.call_async(req)
        ok = self._spin_wait(lambda: fut.done(), 3.0, f'set_mode {mode}')
        return bool(ok and fut.result() and fut.result().mode_sent)

    # def _direct_cmd_tol_land(self) -> bool:
    #     # Optional alternative path via /mavros/cmd/land
    #     if not self.cli_land.wait_for_service(timeout_sec=2.0):
    #         self.get_logger().error('cmd/land not available')
    #         return False
    #     from mavros_msgs.srv import CommandTOL
    #     req = CommandTOL.Request()
    #     req.min_pitch = 0.0
    #     req.yaw = 0.0
    #     # Use current position if available; otherwise zeros
    #     if self._navfix is not None:
    #         req.latitude = float(self._navfix.latitude)
    #         req.longitude = float(self._navfix.longitude)
    #     else:
    #         req.latitude = 0.0
    #         req.longitude = 0.0
    #     req.altitude = 0.0
    #     fut = self.cli_land.call_async(req)
    #     ok = self._spin_wait(lambda: fut.done(), 3.0, 'cmd/land')
    #     return bool(ok and fut.result() and fut.result().success)

    def _is_airborne(self) -> bool:
        # Prefer ExtendedState; fall back to rel_alt heuristic
        if hasattr(self._ext_state, 'landed_state') and self._ext_state.landed_state != LANDED_STATE_ON_GROUND:
            return True
        if self._rel_alt is not None and self._rel_alt > 0.5:
            return True
        return False

    # ---------- Main flow ----------
    def run_land(self):
        self.get_logger().info('--- LAND TEST START ---')

        # 1) Wait MAVROS connected
        ok = self._spin_wait(lambda: getattr(self._state, 'connected', False), self.wait_connect_timeout, 'MAVROS FCU connection')
        if not ok:
            return False
        self.get_logger().info('MAVROS connected.')

        # 2) Check airborne
        self._spin_once(0.1)  # get latest rel_alt/extended_state
        if not self._is_airborne():
            self.get_logger().warn('Vehicle is not airborne (already on ground). Nothing to land.')
            return True

        # 3) Trigger LAND
        if self.use_direct:
            self.get_logger().info("Setting mode 'AUTO.LAND' (direct MAVROS)...")
            if not self._direct_set_mode('AUTO.LAND'):
                self.get_logger().error('Failed to set AUTO.LAND')
                return False
        else:
            self._publish_cmd('land')

        # 4) Confirm initial descent
        start_alt = self._rel_alt if self._rel_alt is not None else 0.0
        self.get_logger().info(f'Start rel_alt = {start_alt:.2f} m; waiting for descent…')

        def descending():
            return (self._rel_alt is not None) and (self._rel_alt < start_alt - 0.3)

        if not self._spin_wait(descending, self.initial_descent_timeout, 'initial descent (Δalt < -0.3m)'):
            self.get_logger().error('No initial descent detected; LAND might not be engaged.')
            return False

        # 5) Wait until landed (extended_state + rel_alt threshold)
        def on_ground():
            ls = getattr(self._ext_state, 'landed_state', LANDED_STATE_UNDEFINED)
            return (ls == LANDED_STATE_ON_GROUND) and (self._rel_alt is not None) and (self._rel_alt <= 0.3)

        if not self._spin_wait(on_ground, self.descend_timeout, 'touchdown (landed_state=ON_GROUND & rel_alt<=0.3m)'):
            self.get_logger().error('Touchdown not confirmed in time.')
            return False

        # Dwell to ensure stable ground contact
        t0 = time.time()
        while time.time() - t0 < self.ground_dwell:
            self._spin_once(0.1)

        self.get_logger().info('✅ Touchdown confirmed.')

        # 6) Optional: verify/discuss auto-disarm
        if self.check_auto_disarm:
            delay = self._get_com_disarm_land()
            if delay > 0.0:
                self.get_logger().info('Waiting for auto-disarm after landing…')
                ok = self._spin_wait(lambda: not self._state.armed, max(1.0, delay + 3.0), 'auto-disarm on ground')
                if ok:
                    self.get_logger().info('✅ Auto-disarmed on ground.')
                else:
                    self.get_logger().warn('Still armed on ground; consider manual disarm or check COM_DISARM_LAND.')

        return True

    def _sigint(self, *_):
        self.get_logger().warn('SIGINT — exiting…')
        # If still airborne, prefer LAND rather than hard disarm
        if self._is_airborne():
            if self.use_direct:
                self._direct_set_mode('AUTO.LAND')
            else:
                self._publish_cmd('land')
        rclpy.shutdown()
        sys.exit(0)


def main():
    parser = argparse.ArgumentParser(description='Behavior governor LAND tester')
    parser.add_argument('--governor-ns', default='/behavior_governor', help='Governor namespace')
    parser.add_argument('--mavros-ns', default='/mavros', help='MAVROS namespace')
    parser.add_argument('--direct-mavros', action='store_true',
                        help='Bypass governor: set AUTO.LAND via MAVROS directly')
    parser.add_argument('--descend-timeout', type=float, default=120.0,
                        help='Total time allowed to reach ground')
    parser.add_argument('--initial-descent-timeout', type=float, default=10.0,
                        help='Time allowed to detect initial descent')
    parser.add_argument('--ground-dwell', type=float, default=2.0,
                        help='Extra time to confirm stable ground contact')
    parser.add_argument('--wait-connect-timeout', type=float, default=15.0)
    parser.add_argument('--check-auto-disarm', action='store_true',
                        help='Read COM_DISARM_LAND and observe disarm after landing')
    args = parser.parse_args()

    rclpy.init()
    node = LandTester(args.governor_ns, args.mavros_ns, args.direct_mavros,
                      args.descend_timeout, args.initial_descent_timeout,
                      args.ground_dwell, args.wait_connect_timeout,
                      args.check_auto_disarm)
    ok = node.run_land()
    node.get_logger().info('DONE' if ok else 'FAILED')
    time.sleep(0.4)
    rclpy.shutdown()
    sys.exit(0 if ok else 2)


if __name__ == '__main__':
    main()