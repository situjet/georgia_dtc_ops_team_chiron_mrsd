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
        self.declare_parameter('force_test_mode', False)  # Force test mode for debugging

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
        self.max_retries = 5  # Reduced from 10 to switch to test mode faster
        self.consecutive_frame_failures = 0
        self.max_consecutive_failures = 3  # Switch to test mode after 3 consecutive frame failures

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
            self.cap = None

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
        
        try:
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

            # Test the connection by reading a frame
            ok, test_frame = self.cap.read()
            if not ok or test_frame is None:
                self.connection_retry_count += 1
                self.get_logger().warning(f'RTSP stream opened but failed to read test frame. Attempt {self.connection_retry_count}/{self.max_retries}')
                self.cap.release()
                self.cap = None
                return

            self.connection_retry_count = 0  # Reset on successful connection
            self.consecutive_frame_failures = 0  # Reset frame failure counter
            self.get_logger().info(f'RTSP stream connected successfully. Frame size: {test_frame.shape}')
            
        except Exception as e:
            self.connection_retry_count += 1
            self.get_logger().error(f'Exception during RTSP connection: {e}. Attempt {self.connection_retry_count}/{self.max_retries}')
            if self.cap:
                try:
                    self.cap.release()
                except Exception:
                    pass
                self.cap = None

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

        try:
            ok, frame = self.cap.read()
            if not ok or frame is None:
                self.consecutive_frame_failures += 1
                self.get_logger().warning(f'Frame grab failed ({self.consecutive_frame_failures}/{self.max_consecutive_failures}); will reconnect or switch to test mode')
                
                if self.consecutive_frame_failures >= self.max_consecutive_failures:
                    self.get_logger().warning(f'Too many consecutive frame failures ({self.consecutive_frame_failures}), switching to test mode')
                    self._setup_test_mode()
                else:
                    self._connect()
                return

            # Reset failure counter on successful frame grab
            self.consecutive_frame_failures = 0
            self._publish_frame(frame)
            
        except Exception as e:
            self.get_logger().error(f'Exception during frame processing: {e}')
            self.consecutive_frame_failures += 1
            if self.consecutive_frame_failures >= self.max_consecutive_failures:
                self.get_logger().warning('Too many exceptions, switching to test mode')
                self._setup_test_mode()

    def _generate_test_frame(self):
        """Generate a synthetic frame for testing"""
        # Create a test pattern
        width = self.width or 640
        height = self.height or 360
        
        # Create gradient background using vectorized operations for efficiency
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Create meshgrid for efficient gradient calculation
        x_indices, y_indices = np.meshgrid(np.arange(width), np.arange(height))
        
        frame[:, :, 0] = (255 * x_indices / width).astype(np.uint8)  # Red gradient
        frame[:, :, 1] = (255 * y_indices / height).astype(np.uint8)  # Green gradient
        frame[:, :, 2] = 128  # Blue constant
        
        # Add frame counter text
        self.test_frame_counter += 1
        text = f"Test Frame {self.test_frame_counter}"
        cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(frame, f"ROS_DOMAIN_ID: {os.environ.get('ROS_DOMAIN_ID', 'unset')}", 
                   (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f"Topic: {self.topic}", 
                   (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        self._publish_frame(frame)

    def _publish_frame(self, frame):
        """Encode and publish a frame"""
        try:
            # Encode to JPEG (CompressedImage expects JPEG/PNG)
            encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(max(1, min(self.jpeg_quality, 100)))]
            ok, buf = cv2.imencode('.jpg', frame, encode_params)
            if not ok or buf is None:
                self.get_logger().warning('JPEG encode failed for frame')
                return

            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'
            msg.format = 'jpeg'
            msg.data = buf.tobytes()
            
            self.publisher.publish(msg)
            
            # Log publishing success periodically
            if hasattr(self, 'frame_count'):
                self.frame_count += 1
            else:
                self.frame_count = 1
                
            if self.frame_count % 50 == 0:  # Log every 50 frames (~10 seconds at 5 FPS)
                mode = "test" if self.test_mode else "RTSP"
                self.get_logger().info(f'Published {self.frame_count} frames to {self.topic} ({mode} mode)')
                
        except Exception as e:
            self.get_logger().error(f'Exception during frame publishing: {e}')

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
