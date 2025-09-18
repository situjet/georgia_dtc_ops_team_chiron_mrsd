import os
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

# Try to import OpenCV
CV_AVAILABLE = False
try:
    import cv2
    import numpy as np
    CV_AVAILABLE = True
except Exception as e:
    print(f"OpenCV import failed: {e}")
    CV_AVAILABLE = False


class RtspStreamer(Node):
    def __init__(self):
        super().__init__('rtsp_streamer_node')

        self.declare_parameter('rtsp_url', 'rtsp://10.3.1.124:8556/ghadron')
        self.declare_parameter('topic', '/image_raw_compressed')
        self.declare_parameter('fps', 5.0)
        self.declare_parameter('width', 0)
        self.declare_parameter('height', 0)
        self.declare_parameter('jpeg_quality', 60)
        self.declare_parameter('rtsp_latency_ms', 200)
        self.declare_parameter('prefer_tcp', True)
        self.declare_parameter('test_mode', False)

        self.rtsp_url: str = self.get_parameter('rtsp_url').get_parameter_value().string_value
        self.topic: str = self.get_parameter('topic').get_parameter_value().string_value
        self.fps: float = self.get_parameter('fps').get_parameter_value().double_value or 5.0
        self.width: int = self.get_parameter('width').get_parameter_value().integer_value
        self.height: int = self.get_parameter('height').get_parameter_value().integer_value
        self.jpeg_quality: int = self.get_parameter('jpeg_quality').get_parameter_value().integer_value or 60
        self.rtsp_latency_ms: int = self.get_parameter('rtsp_latency_ms').get_parameter_value().integer_value or 200
        self.prefer_tcp: bool = self.get_parameter('prefer_tcp').get_parameter_value().bool_value
        self.test_mode: bool = self.get_parameter('test_mode').get_parameter_value().bool_value

        if not CV_AVAILABLE:
            self.get_logger().fatal('OpenCV is required but not available. Exiting.')
            raise RuntimeError('OpenCV unavailable')

        # Prefer FFmpeg backend and TCP transport for stability over lossy UDP
        os.environ['OPENCV_VIDEOIO_PRIORITY_FFMPEG'] = '9999'
        if self.prefer_tcp:
            os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'rtsp_transport;tcp|stimeout;2000000'

        self.publisher = self.create_publisher(CompressedImage, self.topic, 10)

        self.cap: Optional['cv2.VideoCapture'] = None
        self.connection_retry_count = 0
        self.max_retries = 10

        # Timer for frame grabbing at target FPS
        self.period = 1.0 / max(self.fps, 0.1)
        self.timer = self.create_timer(self.period, self._on_timer)

        # Connect to RTSP or test mode
        if self.test_mode:
            self.get_logger().info('Running in test mode - generating synthetic frames')
            self._setup_test_mode()
        else:
            self._connect()

    def _connect(self):
        if self.cap:
            try:
                self.cap.release()
            except Exception:
                pass

        if self.connection_retry_count >= self.max_retries:
            self.get_logger().error(f'Max RTSP connection retries ({self.max_retries}) exceeded. Switching to test mode.')
            self._setup_test_mode()
            return

        url = self.rtsp_url

        # Append latency parameter for GStreamer-like servers if not present
        if 'latency=' not in url and self.rtsp_latency_ms > 0:
            sep = '&' if '?' in url else '?'
            url = f"{url}{sep}latency={self.rtsp_latency_ms}"

        self.get_logger().info(f"Connecting RTSP: {url} (attempt {self.connection_retry_count + 1}/{self.max_retries})")
        self.cap = cv2.VideoCapture(url)
        if not self.cap.isOpened():
            self.connection_retry_count += 1
            self.get_logger().warning(f'Failed to open RTSP stream, will retry. Attempt {self.connection_retry_count}/{self.max_retries}')
            self.cap = None
            return

        # Set desired width/height if provided (>0)
        if self.width > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
        if self.height > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))

        self.connection_retry_count = 0  # Reset on successful connection
        self.get_logger().info('RTSP stream connected successfully.')

    def _setup_test_mode(self):
        """Generate synthetic frames for testing when RTSP is unavailable"""
        self.test_mode = True
        self.test_frame_counter = 0
        self.get_logger().info('Test mode enabled - will generate synthetic frames')

    def _on_timer(self):
        if self.test_mode:
            self._generate_test_frame()
            return

        if self.cap is None:
            self._connect()
            return

        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warning('Frame grab failed; reconnecting...')
            self._connect()
            return

        self._publish_frame(frame)

    def _generate_test_frame(self):
        """Generate a synthetic frame for testing"""
        import numpy as np
        
        # Create a test pattern
        width = self.width or 640
        height = self.height or 360
        
        # Create gradient background
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        for y in range(height):
            for x in range(width):
                frame[y, x] = [
                    int(255 * x / width),  # Red gradient
                    int(255 * y / height),  # Green gradient
                    128  # Blue constant
                ]
        
        # Add frame counter text
        self.test_frame_counter += 1
        text = f"Test Frame {self.test_frame_counter}"
        cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(frame, f"ROS_DOMAIN_ID: {os.environ.get('ROS_DOMAIN_ID', 'unset')}", 
                   (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        self._publish_frame(frame)

    def _publish_frame(self, frame):
        """Encode and publish a frame"""
        # Encode to JPEG (CompressedImage expects JPEG/PNG)
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(max(1, min(self.jpeg_quality, 100)))]
        ok, buf = cv2.imencode('.jpg', frame, encode_params)
        if not ok:
            self.get_logger().warning('JPEG encode failed for frame')
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.format = 'jpeg'
        msg.data = buf.tobytes()
        self.publisher.publish(msg)

    def destroy_node(self):
        try:
            if self.cap:
                self.cap.release()
        finally:
            return super().destroy_node()


def main():
    rclpy.init()
    node = RtspStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
