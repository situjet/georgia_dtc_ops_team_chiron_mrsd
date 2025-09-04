# from cv_bridge import CvBridge
# import cv2
# import subprocess
# import numpy as np
# import time
# import threading
# import os
# from datetime import datetime
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, NavSatFix
# from std_msgs.msg import Float32
# from geometry_msgs.msg import PoseStamped
# from sensor_msgs.msg import Imu
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
# from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# #class to handle all the rtsp retrieval functions and stuff
# class Streaming:
#     rtsp_url = 'rtsp://10.3.1.124:8554/ghadron'
#     WIDTH = 1280  # 1k resolution for adi recording
#     HEIGHT = 720
#     retry_max = 10
#     retry_delay = 2.0
#     fps = 15
#     is_running = True
#     retry_count = 0
#     process = None
#     frame_count = 0
#     last_log_time = time.time()
#     last_frame_time = 0
#     bridge = CvBridge()
    
#     def get_ffmpeg_cmd(self):
#         return [
#             "ffmpeg",
#             "-fflags", "nobuffer",
#             "-flags", "low_delay",
#             "-rtsp_transport", "tcp",
#             "-stimeout", "5000000",    
#             "-use_wallclock_as_timestamps", "1",
#             "-avioflags", "direct",
#             "-flush_packets", "1",
#             "-probesize", "32",        # Reduce initial buffering
#             "-analyzeduration", "0",   # Skip detailed stream analysis
#             "-thread_queue_size", "512", # Increase thread queue size
#             "-hwaccel", "auto",        # Use hardware acceleration if available
#             "-i", self.rtsp_url,
#             "-vsync", "0",
#             "-copyts",
#             "-vf", f"fps={self.fps},scale={self.WIDTH}:{self.HEIGHT}",
#             "-pix_fmt", "bgr24",
#             "-f", "rawvideo",
#             "-"
#         ]

#     def start_ffmpeg(self):
#         try:
#             cmd = self.get_ffmpeg_cmd()
            
#             self.process = subprocess.Popen(
#                 cmd,
#                 stdout=subprocess.PIPE,
#                 stderr=subprocess.PIPE,
#                 bufsize=10**8
#             )
            
#             if self.process.poll() is None:
#                 return True
#             else:
#                 return False
                
#         except Exception as e:
#             return False

#     def retrieve_image(self):
#         frame_size = self.WIDTH * self.HEIGHT * 3
        
#         # Pre-allocate the buffer for better performance
#         frame_buffer = bytearray(frame_size)
#         view = memoryview(frame_buffer)
        
#         #checking if the ffmpeg is running
#         if not self.process or self.process.poll() is not None:
#             retry_wait = self.retry_delay * (1.5 ** min(self.retry_count, 10))
#             self.retry_count += 1
            
#             if self.retry_count > 1:
#                 time.sleep(retry_wait)
            
#             if not self.start_ffmpeg():
#                 return
#             else:
#                 self.retry_count = 0

        
#         try:
#             # More efficient reading using pre-allocated buffer
#             bytes_read = 0
#             while bytes_read < frame_size and self.is_running:
#                 chunk = self.process.stdout.read(frame_size - bytes_read)
#                 if not chunk:
#                     break
#                 view[bytes_read:bytes_read+len(chunk)] = chunk
#                 bytes_read += len(chunk)
            
#             if bytes_read != frame_size:
#                 print("Misalignment in bytes!")
#                 return

#             #return the image frame
#             frame = np.frombuffer(frame_buffer, np.uint8).reshape((self.HEIGHT, self.WIDTH, 3))
#             return frame
            
#         except Exception as e:
#             pass

#     def shutdown(self):
#         self.is_running = False
        
#         if self.process and self.process.poll() is None:
#             self.process.terminate()
#             try:
#                 self.process.wait(timeout=3)
#             except subprocess.TimeoutExpired:
#                 self.process.kill()


# # ROS2 node for RTSP streaming and recording
# class RTSPStreamNode(Node):
#     def __init__(self):
#         super().__init__('rtsp_stream_node')
        
#         # Declare parameters
#         self.declare_parameter('rtsp_url', 'rtsp://10.3.1.124:8555/ghadron',
#             ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
#         self.declare_parameter('width', 1280,
#             ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
#         self.declare_parameter('height', 720,
#             ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
#         self.declare_parameter('fps', 15,
#             ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
#         self.declare_parameter('record_stream', True,
#             ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
#         self.declare_parameter('mcap_base_dir', '/home/dtc/humanflow/ros2_ghadron_gimbal/adi_recording',
#             ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
#         self.declare_parameter('shutdown_timeout', 10.0,
#             ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        
#         # Get parameters
#         self.rtsp_url = self.get_parameter('rtsp_url').value
#         self.width = self.get_parameter('width').value
#         self.height = self.get_parameter('height').value
#         self.fps = self.get_parameter('fps').value
#         self.record_stream = self.get_parameter('record_stream').value
#         self.mcap_base_dir = self.get_parameter('mcap_base_dir').value
#         self.shutdown_timeout = self.get_parameter('shutdown_timeout').value
        
#         # Initialize Streaming class
#         self.streaming = Streaming()
#         self.streaming.rtsp_url = self.rtsp_url
#         self.streaming.WIDTH = self.width
#         self.streaming.HEIGHT = self.height
#         self.streaming.fps = self.fps
        
#         # Bridge for converting between OpenCV and ROS images
#         self.bridge = CvBridge()
        
#         # Setup QoS profile for real-time video streaming
#         qos = QoSProfile(
#             depth=5,
#             reliability=QoSReliabilityPolicy.BEST_EFFORT,
#             history=QoSHistoryPolicy.KEEP_LAST,
#             durability=QoSDurabilityPolicy.VOLATILE
#         )
        
#         # Create publisher for image topic
#         self.image_publisher = self.create_publisher(Image, 'image_raw', qos)
        
#         # Create subscribers for drone telemetry data
#         self.altitude_subscription = self.create_subscription(
#             Float32,
#             '/robot_1/mavros/altitude',
#             self.altitude_callback,
#             10)
        
#         self.rel_alt_subscription = self.create_subscription(
#             Float32,
#             '/robot_1/mavros/global_position/rel_alt',
#             self.rel_alt_callback,
#             10)
            
#         mavros_qos = QoSProfile(
#             depth=10,
#             reliability=QoSReliabilityPolicy.BEST_EFFORT,
#             history=QoSHistoryPolicy.KEEP_LAST,
#             durability=QoSDurabilityPolicy.VOLATILE
#         )
        
#         self.gps_subscription = self.create_subscription(
#             NavSatFix,
#             '/robot_1/mavros/global_position/global',
#             self.gps_callback,
#             qos_profile=mavros_qos)
            
#         self.imu_subscription = self.create_subscription(
#             Imu,
#             '/robot_1/mavros/imu/data',
#             self.imu_callback,
#             qos_profile=mavros_qos)
            
#         # 添加罗盘航向订阅
#         self.compass_hdg_subscription = self.create_subscription(
#             Float32,
#             '/robot_1/mavros/global_position/compass_hdg',
#             self.compass_hdg_callback,
#             qos_profile=mavros_qos)
        
#         # Initialize ROS2 bag recording
#         self.mcap_recording_process = None
#         if self.record_stream:
#             self.start_mcap_recording()
        
#         # Start streaming thread
#         self.streaming_thread = threading.Thread(target=self.stream_and_publish)
#         self.streaming_thread.daemon = True
#         self.streaming_thread.start()
        
#         self.get_logger().info(f"RTSP stream started from: {self.rtsp_url}")
#         self.get_logger().info(f"Publishing to topic: image_raw")
#         self.get_logger().info(f"Subscribed to altitude, GPS and IMU data")
#         if self.record_stream:
#             self.get_logger().info(f"Recording MCAP to: {self.mcap_base_dir}")
    
#     def altitude_callback(self, msg):
#         # Process altitude data
#         pass
        
#     def rel_alt_callback(self, msg):
#         # Process relative altitude data
#         pass
        
#     def gps_callback(self, msg):
#         # Process GPS data
#         pass
        
#     def imu_callback(self, msg):
#         # Process IMU data
#         pass
    
#     def compass_hdg_callback(self, msg):
#         # Process compass heading data
#         pass
    
#     def start_mcap_recording(self):
#         """Start recording MCAP file using ros2 bag command"""
#         try:
#             # Create directory structure with date
#             now = datetime.now()
#             day_folder = now.strftime("%Y-%m-%d")
#             timestamp = now.strftime("%Y%m%d_%H%M%S")
            
#             day_dir = os.path.join(self.mcap_base_dir, day_folder)
#             os.makedirs(day_dir, exist_ok=True)
            
#             recording_path = os.path.join(day_dir, f"recording_{timestamp}")
#             metadata_path = os.path.join(day_dir, f"metadata_{timestamp}.yaml")
            
#             # Prepare ros2 bag record command with additional topics
#             cmd = [
#                 "ros2", "bag", "record",
#                 "-o", recording_path,
#                 "--storage", "mcap",
#                 "--max-bag-duration", "60",  # Create new file every 60 seconds
#                 "--metadata-output-file", metadata_path,  
#                 "/image_raw",
#                 "/robot_1/mavros/altitude",
#                 "/robot_1/mavros/global_position/rel_alt",
#                 "/robot_1/mavros/global_position/global",
#                 "/robot_1/mavros/imu/data",
#                 "/robot_1/mavros/global_position/compass_hdg"
#             ]
            
#             # Start recording process
#             self.mcap_recording_process = subprocess.Popen(
#                 cmd,
#                 stdout=subprocess.PIPE,
#                 stderr=subprocess.PIPE
#             )
            
#             self.get_logger().info(f"Started MCAP recording to: {recording_path}")
            
#         except Exception as e:
#             self.get_logger().error(f"Failed to start MCAP recording: {str(e)}")
    
#     def stream_and_publish(self):
#         """Get frames from RTSP stream and publish to ROS topic"""
#         fps_limit = self.fps  # Maximum frames per second to publish
#         frame_time = 1.0 / fps_limit
        
#         # Add timer for logging messages
#         last_log_time = time.time()
#         log_interval = 5.0  # Log message every 5 seconds
#         frame_count = 0
        
#         while rclpy.ok():
#             start_time = time.time()
            
#             # Get frame from RTSP stream
#             frame = self.streaming.retrieve_image()
            
#             if frame is not None:
#                 try:
#                     # Convert OpenCV image to ROS Image message
#                     img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    
#                     # Add timestamp and frame ID
#                     img_msg.header.stamp = self.get_clock().now().to_msg()
#                     img_msg.header.frame_id = 'camera'
                    
#                     # Publish image
#                     self.image_publisher.publish(img_msg)
                    
#                     # Log message about publishing (once per interval)
#                     frame_count += 1
#                     current_time = time.time()
#                     if current_time - last_log_time >= log_interval:
#                         fps = frame_count / (current_time - last_log_time)
#                         self.get_logger().info(f"Publishing images at {fps:.2f} FPS")
#                         frame_count = 0
#                         last_log_time = current_time
                        
#                 except Exception as e:
#                     self.get_logger().error(f"Error publishing image: {str(e)}")
            
#             # Maintain target FPS
#             elapsed = time.time() - start_time
#             sleep_time = max(0, frame_time - elapsed)
#             time.sleep(sleep_time)
    
#     def shutdown(self):
#         """optimize shutdown to ensure mcap recording is complete"""
#         # stop streaming first
#         if hasattr(self, 'streaming'):
#             self.streaming.shutdown()
        
#         # stop mcap recording process gracefully
#         if self.mcap_recording_process and self.mcap_recording_process.poll() is None:
#             try:
#                 self.get_logger().info(f"Stopping MCAP recording process, will wait up to {self.shutdown_timeout} seconds...")
                
#                 # send SIGTERM signal first
#                 self.mcap_recording_process.terminate()
                
#                 # calculate timeout
#                 shutdown_start = time.time()
#                 check_interval = 0.5  # check every 0.5 seconds
                
#                 # wait for process to end or timeout
#                 while time.time() - shutdown_start < self.shutdown_timeout:
#                     if self.mcap_recording_process.poll() is not None:
#                         elapsed = time.time() - shutdown_start
#                         self.get_logger().info(f"MCAP recording process terminated gracefully, took {elapsed:.2f} seconds, file saved")
#                         break
#                     time.sleep(check_interval)
                
#                 # if process is still running, force kill
#                 if self.mcap_recording_process.poll() is None:
#                     self.get_logger().warn(f"MCAP recording process did not terminate within {self.shutdown_timeout} seconds, will force kill")
#                     self.mcap_recording_process.kill()
#                     self.get_logger().warn("forced to kill recording process, last mcap file may be incomplete")
#                 else:
#                     self.get_logger().info("MCAP recording process terminated gracefully, all data saved")
#             except Exception as e:
#                 self.get_logger().error(f"error during recording shutdown: {str(e)}")
        
#         # log shutdown info
#         try:
#             self.get_logger().info("RTSP stream node shutting down")
#         except Exception:
#             print("RTSP stream node shutting down (unable to log)")

# def main(args=None):
#     rclpy.init(args=args)
#     node = RTSPStreamNode()
    
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.shutdown()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
        
        