import rclpy
from rclpy.node import Node
from std_msgs.msg import String

rclpy.init()
n = Node('quick_takeoff_sender')
pub = n.create_publisher(String, '/behavior_governor/command', 10)
msg = String(); msg.data = 'takeoff'
pub.publish(msg)
print('sent: takeoff')
rclpy.shutdown()
