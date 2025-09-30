#!/usr/bin/env python3
"""NavSatFix -> WinTAK CoT UDP bridge node"""
import socket
import uuid
from datetime import datetime, timedelta
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import NavSatFix


def _iso_utc(dt=None):
    if dt is None:
        dt = datetime.utcnow()
    return dt.strftime("%Y-%m-%dT%H:%M:%SZ")


def _cot_xml(uid, callsign, lat, lon, hae, stale_s):
    now = datetime.utcnow()
    t = _iso_utc(now)
    stale = _iso_utc(now + timedelta(seconds=stale_s))
    return (
        f'<event version="2.0" uid="{uid}" type="a-f-G-U-C" how="m-g" '
        f'time="{t}" start="{t}" stale="{stale}">'
        f'<point lat="{lat}" lon="{lon}" hae="{hae}" ce="9999999.0" le="9999999.0" />'
        f'<detail><contact callsign="{callsign}"/></detail>'
        f'</event>'
    )


class NavSatFixToWinTAK(Node):
    def __init__(self):
        super().__init__('navsatfix_to_wintak')
        self.declare_parameter('tak_host', '127.0.0.1')
        self.declare_parameter('tak_port', 6969)
        self.declare_parameter('callsign', 'GPS')
        self.declare_parameter('stale_seconds', 60)
        self.declare_parameter('send_rate_hz', 2.0)
        self.declare_parameter('fix_topic', '/gps/fix')
        self.declare_parameter('stable_uid', True)
        self.declare_parameter('uid_namespace', 'tak-gps-namespace')

        p = self.get_parameter
        self.tak_host = p('tak_host').get_parameter_value().string_value
        self.tak_port = p('tak_port').get_parameter_value().integer_value
        self.callsign = p('callsign').get_parameter_value().string_value
        self.stale_seconds = p('stale_seconds').value
        self.send_rate_hz = p('send_rate_hz').value
        self.fix_topic = p('fix_topic').get_parameter_value().string_value
        self.stable_uid = p('stable_uid').value
        self.uid_namespace = p('uid_namespace').get_parameter_value().string_value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.queue = deque()
        self.uid_fix = str(uuid.uuid5(uuid.NAMESPACE_DNS, self.uid_namespace)) if self.stable_uid else None

        qos = QoSProfile(depth=50)
        self.create_subscription(NavSatFix, self.fix_topic, self._on_fix, qos)

        period = 1.0 / max(self.send_rate_hz, 1e-6)
        self.create_timer(period, self._drain_queue)

        self.get_logger().info(
            f"NavSatFix->WinTAK UDP {self.tak_host}:{self.tak_port}, "
            f"rate={self.send_rate_hz:.1f}Hz callsign={self.callsign}"
        )

    def _on_fix(self, msg: NavSatFix):
        try:
            lat, lon = float(msg.latitude), float(msg.longitude)
        except Exception:
            return
        alt = float(msg.altitude) if msg.altitude == msg.altitude else 0.0
        self.queue.append((lat, lon, alt))

    def _drain_queue(self):
        if not self.queue:
            return
        lat, lon, alt = self.queue.popleft()
        uid = self.uid_fix or str(uuid.uuid4())
        xml = _cot_xml(uid, self.callsign, lat, lon, alt, self.stale_seconds)
        try:
            self.sock.sendto(xml.encode('utf-8'), (self.tak_host, self.tak_port))
            self.get_logger().info(f"Sent CoT {self.callsign}: {lat:.6f},{lon:.6f},{alt:.1f}")
        except Exception as e:
            self.get_logger().error(f"UDP send failed: {e}")


def main():
    rclpy.init()
    node = NavSatFixToWinTAK()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
