import os
import time
import threading
from typing import Optional
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2

# Try to import GStreamer
GSTREAMER_AVAILABLE = False
try:
    import gi
    gi.require_version('Gst', '1.0')
    gi.require_version('GstApp', '1.0')
    from gi.repository import Gst, GLib, GstApp
    GSTREAMER_AVAILABLE = True
except Exception as e:
    print(f"GStreamer import failed: {e}")
    GSTREAMER_AVAILABLE = False

class GStreamerStreaming:
    def __init__(self, rtsp_url, width=640, height=360):
        self.rtsp_url = rtsp_url
        self.WIDTH = width
        self.HEIGHT = height
        self.fps = 5
        self.is_running = True
        self.frame_count = 0
        self.last_log_time = time.time()
        
        # Initialize GStreamer
        Gst.init(None)
        
        # Create pipeline
        self.pipeline = None
        self.appsink = None
        
        # Create main loop and thread
        self.loop = GLib.MainLoop()
        self.loop_thread = None
        
        # Frame buffer
        self.frame_buffer = deque(maxlen=10)
        self.buffer_lock = threading.Lock()
        
    def get_pipeline_string(self):
        """Get GStreamer pipeline string"""
        # For now, force software decoding to avoid hardware-specific issues
        # This is more reliable in Docker containers
        software_pipeline = (
            f"rtspsrc location={self.rtsp_url} latency=200 buffer-mode=auto ! queue ! "
            "rtph265depay ! queue ! h265parse ! queue ! "
            "avdec_h265 ! queue ! "
            "videorate ! video/x-raw,framerate=5/1 ! queue ! "
            f"videoscale ! video/x-raw,width={self.WIDTH},height={self.HEIGHT} ! queue ! "
            "videoconvert ! video/x-raw,format=BGR ! queue ! "
            "appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true"
        )
        print(f"[GStreamer] Using software decoder pipeline for RTSP: {self.rtsp_url}")
        return software_pipeline
        
    def start_pipeline(self):
        """Start GStreamer pipeline"""
        try:
            pipeline_str = self.get_pipeline_string()
            print(f"Using GStreamer pipeline: {pipeline_str}")
            
            # Create Pipeline
            self.pipeline = Gst.parse_launch(pipeline_str)
            
            # Get appsink element
            self.appsink = self.pipeline.get_by_name("sink")
            if not self.appsink:
                print("Error: Could not get appsink element")
                return False
                
            # Set new frame callback
            self.appsink.connect("new-sample", self.on_new_sample, None)
            
            # Start pipeline
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                print("Error: Could not start GStreamer pipeline")
                return False
                
            # Run GLib main loop in a separate thread
            self.loop_thread = threading.Thread(target=self.run_loop)
            self.loop_thread.daemon = True
            self.loop_thread.start()
            
            print("GStreamer streaming started")
            return True
            
        except Exception as e:
            print(f"Error starting GStreamer: {str(e)}")
            return False
            
    def on_new_sample(self, sink, data):
        """Process new GStreamer sample"""
        try:
            # Get sample
            sample = sink.emit("pull-sample")
            if not sample:
                print("[GStreamer] Warning: No sample received")
                return Gst.FlowReturn.ERROR
                
            # Get buffer from sample
            buffer = sample.get_buffer()
            if not buffer:
                print("[GStreamer] Warning: No buffer in sample")
                return Gst.FlowReturn.ERROR
            
            # Get timestamp
            timestamp = time.time()
            
            # Get memory from buffer
            success, map_info = buffer.map(Gst.MapFlags.READ)
            if not success:
                print("[GStreamer] Error: Failed to map buffer")
                return Gst.FlowReturn.ERROR
            
            try:
                # Get buffer size and validate
                buffer_size = map_info.size
                expected_size = self.HEIGHT * self.WIDTH * 3
                if buffer_size != expected_size:
                    print(f"[GStreamer] Warning: Buffer size mismatch. Got {buffer_size}, expected {expected_size}")
                    # Try to proceed anyway if buffer is large enough
                    if buffer_size < expected_size:
                        return Gst.FlowReturn.ERROR
                
                # Copy data and convert to NumPy array
                frame_data = np.ndarray(
                    shape=(self.HEIGHT, self.WIDTH, 3),
                    dtype=np.uint8,
                    buffer=map_info.data
                ).copy()
                
                # Add frame and timestamp to buffer
                with self.buffer_lock:
                    self.frame_buffer.append((frame_data, timestamp))
                    self.frame_count += 1
                    
                # Periodically log frame rate
                current_time = time.time()
                if current_time - self.last_log_time > 5.0:
                    elapsed = current_time - self.last_log_time
                    fps = self.frame_count / elapsed if elapsed > 0 else 0
                    print(f"[GStreamer] Frame rate: {fps:.2f} FPS, buffer size: {len(self.frame_buffer)}")
                    self.frame_count = 0
                    self.last_log_time = current_time
            finally:
                # Always release buffer
                buffer.unmap(map_info)
                
            return Gst.FlowReturn.OK
            
        except Exception as e:
            print(f"Error processing frame: {str(e)}")
            return Gst.FlowReturn.ERROR
            
    def run_loop(self):
        """Run GLib main loop"""
        try:
            self.loop.run()
        except Exception as e:
            print(f"GStreamer loop error: {str(e)}")
            
    def retrieve_image(self):
        """Get the latest image frame and timestamp"""
        with self.buffer_lock:
            if not self.frame_buffer:
                return None, 0
            # Get the latest frame without clearing the entire buffer
            # This matches the behavior of the working streaming.py
            frame, timestamp = self.frame_buffer[-1]
            return frame, timestamp
            
    def shutdown(self):
        """Close GStreamer resources"""
        self.is_running = False
        
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
            
        if self.loop and self.loop.is_running():
            self.loop.quit()
            
        if self.loop_thread and self.loop_thread.is_alive():
            self.loop_thread.join(timeout=3)

class RtspStreamerGStreamer(Node):
    def __init__(self):
        # 强制使用系统时间，忽略use_sim_time参数
        # 这是必要的，因为视频流需要实时处理
        super().__init__('rtsp_streamer_node')

        # 声明参数
        self.declare_parameter('rtsp_url', 'rtsp://10.3.1.124:8556/ghadron')
        self.declare_parameter('topic', '/image_raw_compressed')
        self.declare_parameter('fps', 1)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 360)
        self.declare_parameter('jpeg_quality', 60)

        # 获取参数
        self.rtsp_url = self.get_parameter('rtsp_url').get_parameter_value().string_value
        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.fps = self.get_parameter('fps').get_parameter_value().double_value or 1.0
        self.width = self.get_parameter('width').get_parameter_value().integer_value or 640
        self.height = self.get_parameter('height').get_parameter_value().integer_value or 360
        self.jpeg_quality = self.get_parameter('jpeg_quality').get_parameter_value().integer_value or 60

        # 创建发布器
        self.publisher = self.create_publisher(CompressedImage, self.topic, 2)

        # 尝试使用GStreamer
        self.use_gstreamer = GSTREAMER_AVAILABLE
        self.streamer = None
        self.cap = None

        if self.use_gstreamer:
            self.get_logger().info('✅ 使用GStreamer后端')
            try:
                self.streamer = GStreamerStreaming(self.rtsp_url, self.width, self.height)
                if not self.streamer.start_pipeline():
                    self.get_logger().warning('GStreamer启动失败，降级到OpenCV')
                    self.use_gstreamer = False
                    self._setup_opencv()
            except Exception as e:
                self.get_logger().warning(f'GStreamer初始化失败: {e}，降级到OpenCV')
                self.use_gstreamer = False
                self._setup_opencv()
        else:
            self.get_logger().info('⚠️ 使用OpenCV后端')
            self._setup_opencv()

        # 定时器
        self.period = 1.0 / max(self.fps, 0.1)
        self.get_logger().info(f'Creating timer with period {self.period:.3f}s (FPS: {self.fps})')
        self.timer = self.create_timer(self.period, self._on_timer)
        self.frame_count = 0
        self.get_logger().info('Timer created successfully')

    def _setup_opencv(self):
        """设置OpenCV后端"""
        try:
            self.cap = cv2.VideoCapture(self.rtsp_url)
            if self.width > 0:
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
            if self.height > 0:
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))
        except Exception as e:
            self.get_logger().error(f'OpenCV设置失败: {e}')

    def _on_timer(self):
        """定时器回调"""
        try:
            if self.use_gstreamer and self.streamer:
                frame, timestamp = self.streamer.retrieve_image()
                if frame is not None:
                    self._publish_frame(frame)
            elif self.cap:
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    self._publish_frame(frame)
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {e}')

    def _publish_frame(self, frame):
        """发布帧"""
        try:
            # 编码为JPEG
            encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(max(1, min(self.jpeg_quality, 100)))]
            ok, buf = cv2.imencode('.jpg', frame, encode_params)
            if not ok or buf is None:
                return

            # 创建ROS消息
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'
            msg.format = 'jpeg'
            msg.data = buf.tobytes()
            
            self.publisher.publish(msg)
            
            # 定期日志
            self.frame_count += 1
            if self.frame_count % 50 == 0:
                backend = "GStreamer" if self.use_gstreamer else "OpenCV"
                self.get_logger().info(f'Published {self.frame_count} frames using {backend}')
                
        except Exception as e:
            self.get_logger().error(f'Frame publishing error: {e}')

    def destroy_node(self):
        """清理资源"""
        try:
            if self.streamer:
                self.streamer.shutdown()
            if self.cap:
                self.cap.release()
        finally:
            return super().destroy_node()

def main():
    rclpy.init()
    node = RtspStreamerGStreamer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
