#!/usr/bin/env python3
"""
中心像素GPS节点测试脚本
======================

用于测试中心像素GPS估计节点功能的简单测试脚本。
"""

import rclpy
from rclpy.node import Node
import time
import numpy as np
from sensor_msgs.msg import NavSatFix, CompressedImage
from geometry_msgs.msg import Vector3, PointStamped
from std_msgs.msg import Float64, String


class CenterPixelGPSNodeTester(Node):
    """中心像素GPS节点测试器"""
    
    def __init__(self):
        super().__init__('center_pixel_gps_tester')
        
        # 订阅输出话题
        self.target_gps_sub = self.create_subscription(
            NavSatFix,
            '/center_gps/target_gps',
            self.target_gps_callback,
            10
        )
        
        self.center_marker_sub = self.create_subscription(
            PointStamped,
            '/center_gps/center_marker',
            self.center_marker_callback,
            10
        )
        
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/center_gps/image_compressed',
            self.image_callback,
            10
        )
        
        # 发布模拟传感器数据
        self.gps_pub = self.create_publisher(NavSatFix, '/dtc_mrsd/mavros/global_position/global', 10)
        self.altitude_pub = self.create_publisher(Float64, '/dtc_mrsd/mavros/global_position/rel_alt', 10)
        self.heading_pub = self.create_publisher(Float64, '/dtc_mrsd/mavros/global_position/compass_hdg', 10)
        self.gimbal_pub = self.create_publisher(Vector3, '/gimbal_attitude', 10)
        self.camera_mode_pub = self.create_publisher(String, '/camera_mode', 10)
        self.image_pub = self.create_publisher(CompressedImage, '/image_raw_compressed', 10)
        
        # 测试统计
        self.stats = {
            'gps_estimates_received': 0,
            'center_markers_received': 0,
            'images_received': 0,
            'test_start_time': time.time()
        }
        
        # 创建定时器发布模拟数据
        self.sensor_timer = self.create_timer(0.1, self.publish_sensor_data)  # 10Hz
        self.image_timer = self.create_timer(0.2, self.publish_test_image)    # 5Hz
        self.stats_timer = self.create_timer(5.0, self.print_stats)           # 每5秒
        
        self.get_logger().info("中心像素GPS节点测试器已启动")

    def publish_sensor_data(self):
        """发布模拟传感器数据"""
        current_time = self.get_clock().now()
        
        # 模拟GPS数据 (固定位置)
        gps_msg = NavSatFix()
        gps_msg.header.stamp = current_time.to_msg()
        gps_msg.header.frame_id = "gps"
        gps_msg.latitude = 33.7756  # 亚特兰大附近
        gps_msg.longitude = -84.3963
        gps_msg.altitude = 300.0
        gps_msg.status.status = 0  # GPS_FIX
        self.gps_pub.publish(gps_msg)
        
        # 模拟高度数据
        altitude_msg = Float64()
        altitude_msg.data = 50.0  # 50米高度
        self.altitude_pub.publish(altitude_msg)
        
        # 模拟航向数据 (缓慢旋转)
        heading_msg = Float64()
        heading_msg.data = (time.time() * 10) % 360  # 缓慢旋转
        self.heading_pub.publish(heading_msg)
        
        # 模拟云台姿态 (轻微摆动)
        gimbal_msg = Vector3()
        gimbal_msg.x = np.sin(time.time() * 0.5) * 0.1  # pitch
        gimbal_msg.y = np.cos(time.time() * 0.3) * 0.05  # roll
        gimbal_msg.z = np.sin(time.time() * 0.2) * 0.2   # yaw
        self.gimbal_pub.publish(gimbal_msg)
        
        # 偶尔切换相机模式
        if int(time.time()) % 30 == 0:  # 每30秒切换一次
            mode_msg = String()
            mode_msg.data = "IR" if int(time.time()) % 60 < 30 else "EO"
            self.camera_mode_pub.publish(mode_msg)

    def publish_test_image(self):
        """发布测试图像"""
        # 创建简单的测试图像 (640x512, 彩色渐变)
        import cv2
        
        width, height = 640, 512
        image = np.zeros((height, width, 3), dtype=np.uint8)
        
        # 创建彩色渐变
        for y in range(height):
            for x in range(width):
                image[y, x] = [
                    int(255 * x / width),      # 红色渐变
                    int(255 * y / height),     # 绿色渐变
                    128                        # 固定蓝色
                ]
        
        # 在中心绘制一个目标
        center_x, center_y = width // 2, height // 2
        cv2.circle(image, (center_x, center_y), 20, (255, 255, 255), 3)
        cv2.circle(image, (center_x, center_y), 5, (0, 0, 0), -1)
        
        # 添加时间戳文本
        timestamp_text = f"Time: {time.time():.1f}"
        cv2.putText(image, timestamp_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # 压缩并发布
        try:
            _, buffer = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = self.get_clock().now().to_msg()
            compressed_msg.header.frame_id = "camera_optical_frame"
            compressed_msg.format = "jpeg"
            compressed_msg.data = np.array(buffer).tobytes()
            
            self.image_pub.publish(compressed_msg)
            
        except Exception as e:
            self.get_logger().error(f"发布测试图像失败: {e}")

    def target_gps_callback(self, msg: NavSatFix):
        """处理GPS估计结果"""
        self.stats['gps_estimates_received'] += 1
        self.get_logger().info(
            f"接收到GPS估计: ({msg.latitude:.6f}, {msg.longitude:.6f}), "
            f"框架: {msg.header.frame_id}"
        )

    def center_marker_callback(self, msg: PointStamped):
        """处理中心标记"""
        self.stats['center_markers_received'] += 1
        self.get_logger().debug(
            f"接收到中心标记: ({msg.point.x:.1f}, {msg.point.y:.1f})"
        )

    def image_callback(self, msg: CompressedImage):
        """处理输出图像"""
        self.stats['images_received'] += 1
        self.get_logger().debug(f"接收到输出图像，大小: {len(msg.data)} 字节")

    def print_stats(self):
        """打印测试统计"""
        elapsed_time = time.time() - self.stats['test_start_time']
        
        self.get_logger().info(
            f"测试统计 (运行时间: {elapsed_time:.1f}s):\n"
            f"  GPS估计: {self.stats['gps_estimates_received']}\n"
            f"  中心标记: {self.stats['center_markers_received']}\n"
            f"  输出图像: {self.stats['images_received']}\n"
            f"  GPS估计频率: {self.stats['gps_estimates_received'] / elapsed_time:.2f} Hz\n"
            f"  图像频率: {self.stats['images_received'] / elapsed_time:.2f} Hz"
        )


def main(args=None):
    """主入口点"""
    rclpy.init(args=args)
    
    try:
        tester = CenterPixelGPSNodeTester()
        
        print("中心像素GPS节点测试器启动")
        print("发布模拟传感器数据到以下话题:")
        print("  - /dtc_mrsd/mavros/global_position/global")
        print("  - /dtc_mrsd/mavros/global_position/rel_alt")
        print("  - /dtc_mrsd/mavros/global_position/compass_hdg")
        print("  - /gimbal_attitude")
        print("  - /camera_mode")
        print("  - /image_raw_compressed")
        print()
        print("监控以下输出话题:")
        print("  - /center_gps/target_gps")
        print("  - /center_gps/center_marker")
        print("  - /center_gps/image_compressed")
        print()
        print("按 Ctrl+C 停止测试")
        
        rclpy.spin(tester)
        
    except KeyboardInterrupt:
        print("\n测试被用户中断")
    except Exception as e:
        print(f"测试失败: {e}")
    finally:
        if rclpy.ok():
            tester.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

