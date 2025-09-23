#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import socket
import time
import json
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from rclpy.serialization import serialize_message
from geometry_msgs.msg import Point
import random
import string
from geometry_msgs.msg import Point, PoseStamped, Pose, Quaternion, Vector3
from builtin_interfaces.msg import Time
import importlib
import time

    
class MessagePublisher(Node):
    def __init__(self):
        super().__init__('message_publisher')
        
        # Declare parameters
        self.declare_parameter('topic_name', "drone/data",
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('topic_type', "dtc_network_msgs/msg/HumanDataMsg",
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('udp_ip', "10.3.1.106",
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('udp_port', 5005,
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        
        # Get parameters
        self.topic_name = self.get_parameter('topic_name').value
        self.topic_type = self.get_parameter('topic_type').value
        self.udp_ip = self.get_parameter('udp_ip').value
        self.UDP_PORT = self.get_parameter('udp_port').value
        self.framecount=0
        self.timer = time.time()
        try:
            # Handle both formats: std_msgs/String and std_msgs.msg.String
            if '/msg/' in self.topic_type:
                pkg, _, msg = self.topic_type.partition('/msg/')
            elif '/' in self.topic_type:
                pkg, msg = self.topic_type.split('/', 1)
            else:
                # Assume format like std_msgs.msg.String
                parts = self.topic_type.split('.')
                if len(parts) >= 3:
                    pkg = parts[0]
                    msg = parts[2]
                else:
                    raise ValueError(f"Invalid message type format: {self.topic_type}")
            
            msg_module = importlib.import_module(f'{pkg}.msg')
            self.MsgClass = getattr(msg_module, msg)
        except (ImportError, AttributeError) as e:
            self.get_logger().error(f"Failed to import message type '{self.topic_type}': {e}")
            exit()
        
        # create Subscription
        self.subscription = self.create_subscription(
            self.MsgClass,
            self.topic_name,
            self.topic_callback,
            10)
        
        # Create CV bridge
        self.bridge = CvBridge()

    def topic_callback(self, msg):
        try:
            serialized_message = serialize_message(msg)
            self.publish_data(serialized_message)
            self.framecount=self.framecount+1
            if self.framecount%10==0:
                self.get_logger().info(f'message count: {self.framecount}, Speed: {10/(time.time() - self.timer)} MPS')
                self.timer = time.time()
            
        except Exception as e:
            self.get_logger().error(f'Error processing message: {str(e)}')
    
    def publish_data(self,data):
        UDP_IP = str(self.udp_ip)  
        # Replace with receiver's IP
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #data = bytearray(msg, "utf-8")
        self.get_logger().info(f'Sending {len(data)} Bytes')
        sock.sendto(b'__start__', (UDP_IP, self.UDP_PORT))
        # Send in chunks because UDP packet size is limited
        CHUNK_SIZE = 512
        for i in range(0, len(data), CHUNK_SIZE):
            chunk = data[i:i+CHUNK_SIZE]
            sock.sendto(chunk, (UDP_IP, self.UDP_PORT))
            time.sleep(0.01) #sleep to ensure no data loss
        # Send a special message to signal end of image
        sock.sendto(b'__end__', (UDP_IP, self.UDP_PORT))


def main(args=None):
    rclpy.init(args=args)
    node = MessagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
