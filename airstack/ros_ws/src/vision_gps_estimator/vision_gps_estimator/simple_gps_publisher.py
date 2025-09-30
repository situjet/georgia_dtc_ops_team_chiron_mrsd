#!/usr/bin/env python3
"""
简化的GPS发布器节点
从MAVROS数据读取GPS信息并重新发布到vision_gps主题
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import NavSatFix, CompressedImage
from std_msgs.msg import Float64
import time

class SimpleGPSPublisher(Node):
    def __init__(self):
        super().__init__('simple_gps_publisher')
        
        # 创建兼容MAVROS的QoS配置
        mavros_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 订阅MAVROS GPS数据
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/dtc_mrsd_/mavros/global_position/global',
            self.gps_callback,
            mavros_qos
        )
        
        # 订阅高度数据
        self.altitude_sub = self.create_subscription(
            Float64,
            '/dtc_mrsd_/mavros/global_position/rel_alt',
            self.altitude_callback,
            mavros_qos
        )
        
        # 订阅图像流以确保系统运行
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/image_raw_compressed',
            self.image_callback,
            10
        )
        
        # 发布目标GPS位置
        self.target_gps_pub = self.create_publisher(
            NavSatFix,
            '/vision_gps/target_gps',
            10
        )
        
        # 状态变量
        self.latest_gps = None
        self.latest_altitude = None
        self.image_count = 0
        
        # 定时器：每5秒发布一次GPS数据（模拟目标检测）
        self.timer = self.create_timer(5.0, self.publish_target_gps)
        
        self.get_logger().info("SimpleGPSPublisher节点已启动")
    
    def gps_callback(self, msg):
        """GPS数据回调"""
        self.latest_gps = msg
        self.get_logger().info(f"收到GPS数据: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}")
    
    def altitude_callback(self, msg):
        """高度数据回调"""
        self.latest_altitude = msg.data
        self.get_logger().info(f"收到高度数据: {msg.data:.2f}m")
    
    def image_callback(self, msg):
        """图像数据回调"""
        self.image_count += 1
        if self.image_count % 25 == 0:  # 每25帧记录一次
            self.get_logger().info(f"收到图像帧: {self.image_count}")
    
    def publish_target_gps(self):
        """发布目标GPS位置（模拟检测到的目标）"""
        if self.latest_gps is None:
            self.get_logger().warning("没有GPS数据，无法发布目标位置")
            return
        
        # 创建目标GPS消息（在当前位置附近添加小偏移模拟目标）
        target_msg = NavSatFix()
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.header.frame_id = "map"
        
        # 添加小偏移（约10米）模拟检测到的目标
        lat_offset = 0.0001  # 约11米
        lon_offset = 0.0001  # 约11米
        
        target_msg.latitude = self.latest_gps.latitude + lat_offset
        target_msg.longitude = self.latest_gps.longitude + lon_offset
        
        if self.latest_altitude is not None:
            target_msg.altitude = self.latest_altitude
        else:
            target_msg.altitude = self.latest_gps.altitude
        
        # 设置协方差（表示估计精度）
        target_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        target_msg.position_covariance = [
            25.0, 0.0, 0.0,    # 5m标准差的平方
            0.0, 25.0, 0.0,
            0.0, 0.0, 25.0
        ]
        
        target_msg.status.status = NavSatFix.STATUS_FIX
        target_msg.status.service = NavSatFix.SERVICE_GPS
        
        # 发布目标GPS
        self.target_gps_pub.publish(target_msg)
        
        self.get_logger().info(
            f"发布目标GPS: lat={target_msg.latitude:.6f}, "
            f"lon={target_msg.longitude:.6f}, alt={target_msg.altitude:.2f}m"
        )

def main(args=None):
    rclpy.init(args=args)
    node = SimpleGPSPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
