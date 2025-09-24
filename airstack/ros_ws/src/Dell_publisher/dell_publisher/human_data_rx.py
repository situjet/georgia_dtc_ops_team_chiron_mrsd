#!/usr/bin/env python3
import socket
import importlib
from typing import Tuple

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.serialization import deserialize_message


def _import_msg_class(type_str: str):
    pkg, _, msg = type_str.partition('/msg/')
    mod = importlib.import_module(f'{pkg}.msg')
    return getattr(mod, msg)


class HumanDataRX(Node):
    """
    RX bridge (ground side):
    - Listens on UDP for serialized HumanDataMsg chunks between __start__/__end__.
    - Reassembles and deserializes back into msg object.
    - Validates basic content rules.
    - Publishes to ROS topic for downstream consumers.
    """

    def __init__(self):
        super().__init__('human_data_rx')

        # ---------- Parameters ----------
        self.declare_parameter('udp_ip', '0.0.0.0', ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('udp_port', 5005, ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('topic_name', '/uav1/blusdr/human_data', ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('topic_type', 'dtc_network_msgs/msg/HumanDataMsg', ParameterDescriptor(type=ParameterType.PARAMETER_STRING))

        self.udp_ip = self.get_parameter('udp_ip').value
        self.udp_port = int(self.get_parameter('udp_port').value)
        self.topic_name = self.get_parameter('topic_name').value
        self.topic_type = self.get_parameter('topic_type').value

        self.MsgClass = _import_msg_class(self.topic_type)
        self.pub = self.create_publisher(self.MsgClass, self.topic_name, 10)

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.settimeout(0.5)

        self.buffer = bytearray()
        self.receiving = False

        self.get_logger().info(f"[RX] UDP {self.udp_ip}:{self.udp_port} -> {self.topic_name} ({self.topic_type})")
        self.timer = self.create_timer(0.001, self._poll)

    # ---------- Validation ----------
    def _valid_human_data(self, m) -> Tuple[bool, str]:
        if not getattr(m, 'system', ''):
            return False, "system is empty"
        gps = getattr(m, 'gps_data', None)
        if gps is None:
            return False, "gps_data is None"
        if not (-90.0 <= gps.latitude <= 90.0 and -180.0 <= gps.longitude <= 180.0):
            return False, "gps lat/lon out of range"
        has_raw = bool(m.raw_images)
        has_comp = bool(m.compressed_images)
        if not (has_raw ^ has_comp):
            return False, "must have exactly one of raw_images or compressed_images"
        if has_raw:
            img = m.raw_images[0]
            if img.height == 0 or img.width == 0 or not img.encoding or len(img.data) == 0:
                return False, "raw image invalid (shape/encoding/data)"
        if m.header.stamp.sec == 0 and m.header.stamp.nanosec == 0:
            return False, "header.stamp is zero"
        return True, ""

    def _poll(self):
        try:
            data, _ = self.sock.recvfrom(2048)
        except socket.timeout:
            return
        except Exception as e:
            self.get_logger().warn(f"UDP recv error: {e}")
            return

        if data == b'__start__':
            self.buffer.clear()
            self.receiving = True
            return

        if data == b'__end__':
            if self.receiving and self.buffer:
                try:
                    msg = deserialize_message(bytes(self.buffer), self.MsgClass)
                    ok, why = self._valid_human_data(msg)
                    if not ok:
                        self.get_logger().warn(f"Dropping invalid HumanDataMsg: {why}")
                    else:
                        self.pub.publish(msg)
                except Exception as e:
                    self.get_logger().warn(f"Deserialize failed: {e}")
            self.receiving = False
            self.buffer.clear()
            return

        if self.receiving:
            self.buffer.extend(data)


def main(args=None):
    rclpy.init(args=args)
    node = HumanDataRX()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.sock.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
