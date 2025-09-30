#!/usr/bin/env python3
"""
简化的YOLO测试器
测试YOLO检测器是否可以在图像流上工作
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import numpy as np
import time
from cv_bridge import CvBridge

# 尝试导入YOLO
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("警告: ultralytics YOLO不可用，将使用模拟检测")

class SimpleYOLOTester(Node):
    def __init__(self):
        super().__init__('simple_yolo_tester')
        
        # OpenCV bridge
        self.bridge = CvBridge()
        
        # 订阅压缩图像流
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/image_raw_compressed',
            self.image_callback,
            10
        )
        
        # 发布检测结果
        self.detection_pub = self.create_publisher(
            String,
            '/yolo_detections',
            10
        )
        
        # YOLO模型
        self.model = None
        self.yolo_loaded = False
        
        # 统计
        self.frame_count = 0
        self.detection_count = 0
        self.last_detection_time = 0
        
        # 尝试加载YOLO模型
        if YOLO_AVAILABLE:
            self._load_yolo_model()
        
        self.get_logger().info("SimpleYOLOTester节点已启动")
        if not YOLO_AVAILABLE:
            self.get_logger().warning("YOLO不可用，将使用模拟检测")
    
    def _load_yolo_model(self):
        """加载YOLO模型"""
        try:
            # 尝试不同的模型路径
            model_paths = [
                'yolo11n.pt',  # 最小模型
                'yolov8n.pt',  # 备选模型
                '/root/ros_ws/src/vision_gps_estimator/models/yolo12s.pt',  # 配置中的模型
            ]
            
            for model_path in model_paths:
                try:
                    self.get_logger().info(f"尝试加载YOLO模型: {model_path}")
                    self.model = YOLO(model_path)
                    self.yolo_loaded = True
                    self.get_logger().info(f"成功加载YOLO模型: {model_path}")
                    break
                except Exception as e:
                    self.get_logger().warning(f"无法加载模型 {model_path}: {e}")
                    continue
            
            if not self.yolo_loaded:
                self.get_logger().error("无法加载任何YOLO模型，将使用模拟检测")
                
        except Exception as e:
            self.get_logger().error(f"YOLO模型加载失败: {e}")
    
    def image_callback(self, msg):
        """图像回调函数"""
        try:
            self.frame_count += 1
            
            # 限制处理频率（每5帧处理一次）
            if self.frame_count % 5 != 0:
                return
            
            # 解压缩图像
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if frame is None:
                self.get_logger().warning("无法解码图像")
                return
            
            # 运行检测
            detections = self._run_detection(frame)
            
            # 发布检测结果
            if detections:
                self._publish_detections(detections, frame.shape)
            
            # 定期记录统计信息
            if self.frame_count % 50 == 0:
                self.get_logger().info(
                    f"处理了 {self.frame_count} 帧，检测到 {self.detection_count} 个目标"
                )
                
        except Exception as e:
            self.get_logger().error(f"图像处理错误: {e}")
    
    def _run_detection(self, frame):
        """运行目标检测"""
        current_time = time.time()
        
        if self.yolo_loaded and self.model is not None:
            # 使用真实YOLO检测
            try:
                results = self.model(
                    frame,
                    conf=0.5,
                    iou=0.45,
                    classes=[0],  # 只检测人
                    verbose=False
                )
                
                detections = []
                if results and len(results) > 0:
                    result = results[0]
                    if result.boxes is not None and len(result.boxes) > 0:
                        boxes = result.boxes.xyxy.cpu().numpy()
                        scores = result.boxes.conf.cpu().numpy()
                        classes = result.boxes.cls.cpu().numpy()
                        
                        for i in range(len(boxes)):
                            x1, y1, x2, y2 = boxes[i]
                            center_x = (x1 + x2) / 2
                            center_y = (y1 + y2) / 2
                            
                            detections.append({
                                'bbox': [x1, y1, x2, y2],
                                'center': [center_x, center_y],
                                'confidence': float(scores[i]),
                                'class': int(classes[i])
                            })
                
                self.last_detection_time = current_time
                return detections
                
            except Exception as e:
                self.get_logger().error(f"YOLO检测失败: {e}")
                return []
        else:
            # 使用模拟检测（每10秒生成一个假检测）
            if current_time - self.last_detection_time > 10.0:
                height, width = frame.shape[:2]
                # 在图像中心附近生成假检测
                center_x = width // 2 + np.random.randint(-100, 100)
                center_y = height // 2 + np.random.randint(-50, 50)
                
                detections = [{
                    'bbox': [center_x-50, center_y-50, center_x+50, center_y+50],
                    'center': [center_x, center_y],
                    'confidence': 0.85,
                    'class': 0,
                    'simulated': True
                }]
                
                self.last_detection_time = current_time
                return detections
            
            return []
    
    def _publish_detections(self, detections, image_shape):
        """发布检测结果"""
        if not detections:
            return
        
        self.detection_count += len(detections)
        
        # 创建检测结果消息
        detection_msg = String()
        
        result_text = f"检测到 {len(detections)} 个目标:\n"
        for i, det in enumerate(detections):
            center_x, center_y = det['center']
            confidence = det['confidence']
            is_sim = det.get('simulated', False)
            sim_text = " (模拟)" if is_sim else ""
            
            result_text += f"目标{i+1}: 中心({center_x:.1f}, {center_y:.1f}), 置信度{confidence:.2f}{sim_text}\n"
        
        result_text += f"图像尺寸: {image_shape[1]}x{image_shape[0]}"
        
        detection_msg.data = result_text
        self.detection_pub.publish(detection_msg)
        
        self.get_logger().info(f"发布检测结果: {len(detections)} 个目标")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleYOLOTester()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
