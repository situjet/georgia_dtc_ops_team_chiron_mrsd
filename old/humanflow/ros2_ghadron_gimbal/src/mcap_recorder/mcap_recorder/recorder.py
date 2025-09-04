#!/usr/bin/env python3

import cv2
import rclpy
import os
import time
import threading
import numpy as np
from datetime import datetime
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Float32, Float64
from sensor_msgs.msg import NavSatFix, Imu, Image
from cv_bridge import CvBridge
import subprocess
import signal  # Import signal module
import shutil  # Import shutil module
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from mavros_msgs.msg import Altitude
import traceback # Import traceback for logging
import yaml    # Import yaml module
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

class RecorderNode(Node):
    def __init__(self):
        super().__init__('recorder_node')
        
        # Declare parameters with defaults and descriptions
        self.declare_parameter('output_base_dir', "/home/dtc/humanflow/ros2_ghadron_gimbal/mcap_recordings",
                               ParameterDescriptor(description='Base directory for saving recordings.'))
        self.declare_parameter('rtsp_url', 'rtsp://10.3.1.124:8554/ghadron',
                               ParameterDescriptor(description='RTSP stream URL.'))
        self.declare_parameter('topics_to_record', [
            '/camera/image_raw',
            '/robot_1/mavros/global_position/rel_alt',
            '/robot_1/mavros/global_position/global',
            '/robot_1/mavros/imu/data',
            '/robot_1/mavros/global_position/compass_hdg',
            '/gimbal_attitude'
            ], ParameterDescriptor(description='List of topics to record.'))
        self.declare_parameter('monitor_interval_sec', 5.0,
                               ParameterDescriptor(description='Interval in seconds for checking process and disk space.'))
        self.declare_parameter('disk_threshold_gb', 2.0,
                                ParameterDescriptor(description='Minimum free disk space in GB before stopping recording.'))
        self.declare_parameter('rtsp_reconnect_delay_sec', 2.0,
                                ParameterDescriptor(description='Delay in seconds before attempting RTSP reconnection.'))
        self.declare_parameter('shutdown_timeout_sec', 15.0,
                                ParameterDescriptor(description='Timeout in seconds for graceful shutdown of ros2 bag record.'))
        self.declare_parameter('use_rtsp_tcp', True, # Default to TCP for potentially better reliability
                               ParameterDescriptor(description='Use TCP transport for RTSP stream (True) or default (usually UDP) (False).'))

        # Get parameters
        self.output_base_dir = self.get_parameter('output_base_dir').get_parameter_value().string_value
        self.rtsp_url = self.get_parameter('rtsp_url').get_parameter_value().string_value
        self.topics_to_record = self.get_parameter('topics_to_record').get_parameter_value().string_array_value
        self.monitor_interval = self.get_parameter('monitor_interval_sec').get_parameter_value().double_value
        self.disk_threshold_bytes = self.get_parameter('disk_threshold_gb').get_parameter_value().double_value * (1024**3)
        self.reconnect_delay = self.get_parameter('rtsp_reconnect_delay_sec').get_parameter_value().double_value
        self.shutdown_timeout = self.get_parameter('shutdown_timeout_sec').get_parameter_value().double_value
        self.use_rtsp_tcp = self.get_parameter('use_rtsp_tcp').get_parameter_value().bool_value # Get TCP parameter

        # Log parameters used
        self.get_logger().info("--- Recorder Parameters ---")
        self.get_logger().info(f"Output Base Dir: {self.output_base_dir}")
        self.get_logger().info(f"RTSP URL: {self.rtsp_url}")
        self.get_logger().info(f"Topics to Record: {list(self.topics_to_record)}")
        self.get_logger().info(f"Monitor Interval (sec): {self.monitor_interval}")
        self.get_logger().info(f"Disk Threshold (GB): {self.get_parameter('disk_threshold_gb').get_parameter_value().double_value}")
        self.get_logger().info(f"RTSP Reconnect Delay (sec): {self.reconnect_delay}")
        self.get_logger().info(f"Shutdown Timeout (sec): {self.shutdown_timeout}")
        self.get_logger().info(f"Use RTSP TCP: {self.use_rtsp_tcp}") # Log TCP parameter
        self.get_logger().info("---------------------------")

        # Initialize shutdown flag
        self._is_shutdown = False

        # Create unique output directory for this run
        self.output_dir = os.path.join(self.output_base_dir, f'{datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}')
        self.start_time_str = os.path.basename(self.output_dir) # Store start time string
        try:
            os.makedirs(self.output_dir, exist_ok=True)
            self.get_logger().info(f'Created recording directory: {self.output_dir}')
        except OSError as e:
            self.get_logger().fatal(f"CRITICAL: Could not create output directory {self.output_dir}: {e}. Node will not start correctly.")
            # Let the initialization continue so node can be destroyed, but it won't function.
            # A better approach might be to raise an exception here handled in main.
            return # Stop initialization if directory creation fails

        # Save metadata
        self.save_metadata() # Logs success/failure inside

        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Publisher for the camera images from RTSP
        # Use default reliable QoS for camera image publisher
        self.image_publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # Define RELIABLE mavros QoS configuration for critical data
        # MAVROS publishers often use BEST_EFFORT, so match that.
        mavros_qos_best_effort = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT, # Use BEST_EFFORT to match publishers
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE # MAVROS topics are typically volatile
        )
        
        # Create subscriptions with RELIABLE QoS
        # Create subscriptions with matching BEST_EFFORT QoS
        self.rel_alt_subscription = self.create_subscription(
            Float64,
            '/robot_1/mavros/global_position/rel_alt',
            self.rel_alt_callback,
            qos_profile=mavros_qos_best_effort) # Use best_effort QoS
            
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/robot_1/mavros/global_position/global',
            self.gps_callback,
            qos_profile=mavros_qos_best_effort) # Use best_effort QoS
            
        self.imu_subscription = self.create_subscription(
            Imu,
            '/robot_1/mavros/imu/data',
            self.imu_callback,
            qos_profile=mavros_qos_best_effort) # Use best_effort QoS
            
        self.compass_hdg_subscription = self.create_subscription(
            Float64,
            '/robot_1/mavros/global_position/compass_hdg',
            self.compass_hdg_callback,
            qos_profile=mavros_qos_best_effort) # Use best_effort QoS
        
        # Start RTSP stream receiver thread
        self.rtsp_thread = threading.Thread(target=self.rtsp_receiver)
        self.rtsp_thread.daemon = True
        self.rtsp_active = threading.Event() # Event to signal RTSP is active
        self.rtsp_thread.start()
        
        # Wait briefly for RTSP to potentially connect before starting bag recording
        self.get_logger().info("Waiting for RTSP stream to become active before starting MCAP...")
        if not self.rtsp_active.wait(timeout=10.0):
             self.get_logger().warn('RTSP stream did not become active within timeout. MCAP recording will start, but might miss initial frames or fail if stream never connects.')
        else:
            self.get_logger().info('RTSP stream active.')
        
        # Start MCAP recording process
        self.mcap_process = None
        self.recording_active = False
        self.start_mcap_recording() # Logs info/errors inside
        
        # Start monitoring timer only if recording started successfully
        if self.recording_active:
            self.get_logger().info(f"Starting monitoring timer with interval {self.monitor_interval} seconds.")
            self.monitor_timer = self.create_timer(self.monitor_interval, self.monitor_recording)
        else:
            self.get_logger().warn("MCAP recording did not start. Monitoring timer will not be created.")
            self.monitor_timer = None # Ensure timer is not accessed if recording failed to start
        
        self.get_logger().info(f'Recorder node initialization complete.')
    
    def altitude_callback(self, msg):
        # Keep callbacks lightweight
        pass
        
    def rel_alt_callback(self, msg):
        pass
        
    def gps_callback(self, msg):
        pass
        
    def imu_callback(self, msg):
        pass
        
    def compass_hdg_callback(self, msg):
        pass
    
    @profile
    def rtsp_receiver(self):
        """Receive RTSP stream using GStreamer with hardware acceleration and publish to ROS topic"""
        self.get_logger().info(f'RTSP: Attempting to connect to stream: {self.rtsp_url} with GStreamer (HW acceleration)...')
        
        # Initialize GStreamer
        Gst.init(None)
        
        frame_count = 0
        log_interval_frames = 100
        
        try:
            while rclpy.ok() and not self._is_shutdown:
                # 构建支持 Jetson Orin NX 的自适应 GStreamer 管道
                transport_param = "protocols=tcp " if self.use_rtsp_tcp else ""
                
                # 尝试 Jetson Orin NX 的硬件加速
                try:
                    pipeline_str = (
                        f"rtspsrc location={self.rtsp_url} {transport_param}latency=0 ! "
                        "rtph265depay ! h265parse ! nvv4l2decoder enable-max-performance=1 ! "
                        "nvvidconv ! video/x-raw,format=BGRx ! "
                        "videorate ! video/x-raw,framerate=10/1 ! " # 限制为 10 帧/秒
                        "videoscale ! video/x-raw,width=640,height=512 ! " # 缩放到 640x512
                        "videoconvert ! video/x-raw,format=BGR ! "
                        "appsink name=sink emit-signals=true max-buffers=2 drop=true sync=false"
                    )
                    
                    # 测试管道可行性
                    test_pipeline = Gst.parse_launch(pipeline_str)
                    test_pipeline.set_state(Gst.State.NULL)
                    self.get_logger().info("RTSP: Using Jetson Orin NX hardware accelerated decoder")
                except Exception as e:
                    self.get_logger().warn(f"RTSP: Jetson decoder error: {str(e)}, falling back to software decoder")
                    # 降级到软件解码
                    pipeline_str = (
                        f"rtspsrc location={self.rtsp_url} {transport_param}latency=0 buffer-mode=auto ! "
                        "rtph265depay ! h265parse ! avdec_h265 max-threads=4 ! "
                        "videorate ! video/x-raw,framerate=10/1 ! " # 限制为 10 帧/秒
                        "videoscale ! video/x-raw,width=640,height=512 ! " # 缩放到 640x512
                        "videoconvert ! video/x-raw,format=BGR ! "
                        "appsink name=sink emit-signals=true max-buffers=2 drop=true sync=false"
                    )
                
                self.get_logger().info(f"RTSP: Creating GStreamer pipeline: {pipeline_str}")
                
                pipeline = Gst.parse_launch(pipeline_str)
                appsink = pipeline.get_by_name("sink")
                
                # 设置新样本回调
                @profile
                def on_new_sample(sink):
                    nonlocal frame_count
                    sample = sink.emit("pull-sample")
                    if not sample:
                        return Gst.FlowReturn.ERROR
                    
                    # 帧率控制：只处理每 X 帧中的一帧（适应于原始帧率）
                    # 假设原始是 30fps，要降到 10fps，则每 3 帧处理 1 帧
                    # if frame_count % 3 != 0:  # 调整分母以达到大约 10fps
                    #     frame_count += 1
                    #     return Gst.FlowReturn.OK  # 跳过该帧，但告诉 GStreamer 一切正常
                    
                    buf = sample.get_buffer()
                    caps = sample.get_caps()
                    
                    success, map_info = buf.map(Gst.MapFlags.READ)
                    if not success:
                        return Gst.FlowReturn.ERROR
                    
                    # 提取帧尺寸
                    structure = caps.get_structure(0)
                    width = structure.get_value("width")
                    height = structure.get_value("height")
                    
                    # 创建 numpy 数组
                    frame_data = np.ndarray(
                        shape=(height, width, 3),
                        dtype=np.uint8,
                        buffer=map_info.data
                    ).copy()
                    
                    buf.unmap(map_info)
                    
                    # 转换并发布 ROS 图像消息
                    img_msg = self.bridge.cv2_to_imgmsg(frame_data, 'bgr8')
                    img_msg.header.stamp = self.get_clock().now().to_msg()
                    img_msg.header.frame_id = 'camera'
                    self.image_publisher.publish(img_msg)
                    
                    if not self.rtsp_active.is_set():
                        self.get_logger().info("RTSP: Stream became active.")
                        self.rtsp_active.set()
                    
                    frame_count += 1
                    if frame_count % log_interval_frames == 0:
                        self.get_logger().debug(f"RTSP: Published frame {frame_count}")
                    
                    return Gst.FlowReturn.OK
                
                appsink.connect("new-sample", on_new_sample)
                
                ret = pipeline.set_state(Gst.State.PLAYING)
                if ret == Gst.StateChangeReturn.FAILURE:
                    self.get_logger().error("RTSP: Failed to set pipeline to PLAYING state")
                    time.sleep(self.reconnect_delay)
                    continue
                
                bus = pipeline.get_bus()
                self.get_logger().info("RTSP: GStreamer pipeline started, waiting for frames...")
                
                while rclpy.ok() and not self._is_shutdown:
                    message = bus.timed_pop_filtered(
                        100 * Gst.MSECOND,
                        Gst.MessageType.ERROR | Gst.MessageType.EOS | Gst.MessageType.STATE_CHANGED
                    )
                    
                    if message:
                        if message.type == Gst.MessageType.ERROR:
                            err, debug = message.parse_error()
                            self.get_logger().error(f"RTSP: GStreamer error: {err}, {debug}")
                            break
                        elif message.type == Gst.MessageType.EOS:
                            self.get_logger().info("RTSP: End of stream received")
                            break
                
                self.get_logger().info("RTSP: Stopping pipeline for reconnection")
                self.rtsp_active.clear()
                pipeline.set_state(Gst.State.NULL)
                
                if self._is_shutdown:
                    self.get_logger().info("RTSP: Shutdown requested, not attempting reconnection")
                    break
                
                self.get_logger().info(f"RTSP: Reconnecting in {self.reconnect_delay} seconds...")
                time.sleep(self.reconnect_delay)
                
        except Exception as e:
            if not self._is_shutdown:
                self.get_logger().error(f'RTSP: Unhandled exception in GStreamer loop: {str(e)}')
                tb_str = traceback.format_exc()
                self.get_logger().error(f"RTSP: Traceback:\n{tb_str}")
        finally:
            if 'pipeline' in locals():
                pipeline.set_state(Gst.State.NULL)
            self.rtsp_active.clear()
            self.get_logger().info('RTSP: GStreamer receiver thread finished.')
    
    def start_mcap_recording(self):
        """Start MCAP recording process"""
        if not self.topics_to_record:
             self.get_logger().warn("MCAP: No topics specified for recording ('topics_to_record' parameter is empty). Skipping MCAP recording start.")
             self.recording_active = False
             return

        # Build ros2 bag record command
        mcap_filename = 'recording' # Base name, ros2 bag adds timestamp etc. automatically
        mcap_file_path = os.path.join(self.output_dir, mcap_filename)
        # Ensure the base filename itself doesn't have problematic characters if changed
        safe_mcap_file_path = os.path.abspath(mcap_file_path) # Use absolute path

        cmd = ['ros2', 'bag', 'record', '-o', safe_mcap_file_path, '-s', 'mcap', '-d', '60']
        cmd.extend(self.topics_to_record)
        cmd_str = " ".join(cmd) # For logging

        # Start recording process
        try:
            self.get_logger().info(f'MCAP: Starting recording with command: {cmd_str}')
            # Use preexec_fn to make the process group leader, easier to send signals
            # Redirect stdout/stderr to PIPE to potentially capture output if needed, or DEVNULL to ignore
            self.mcap_process = subprocess.Popen(
                cmd,
                preexec_fn=os.setsid,
                stdout=subprocess.DEVNULL, # Ignore stdout
                stderr=subprocess.PIPE     # Capture stderr
            )
            self.recording_active = True
            self.get_logger().info(f'MCAP: Recording process started successfully with PID: {self.mcap_process.pid}. Output directory: {safe_mcap_file_path}')
        except FileNotFoundError:
             self.get_logger().error(f"MCAP: 'ros2' command not found. Is the ROS 2 environment sourced correctly? Failed to start recording.")
             self.mcap_process = None
             self.recording_active = False
        except Exception as e:
            self.get_logger().error(f'MCAP: Failed to start recording process: {str(e)}')
            tb_str = traceback.format_exc()
            self.get_logger().error(f"MCAP: Traceback:\n{tb_str}")
            self.mcap_process = None
            self.recording_active = False
    @profile
    def monitor_recording(self):
        """Periodically check MCAP process status and disk space."""
        if self._is_shutdown: # Don't monitor if shutting down
             self.get_logger().info("Monitor: Shutdown in progress, skipping monitoring check.")
             return
             
        if not self.recording_active or self.mcap_process is None:
            # This case should ideally not happen if timer is cancelled properly, but good safety check
            self.get_logger().warn("Monitor: Monitoring called but recording is not active or process is None. Stopping monitor.")
            if self.monitor_timer:
                self.monitor_timer.cancel()
            return

        # Check process status
        proc_status = self.mcap_process.poll()
        if proc_status is not None:
            # Process terminated - could be due to 60-second limit or an error
            if proc_status == 0:
                # Normal termination (likely due to 60-second limit)
                self.get_logger().info('MONITOR: MCAP segment completed. Starting new recording segment.')
                self.start_mcap_recording()  # Start a new recording segment
            else:
                # Abnormal termination
                self.get_logger().error(f'MONITOR: MCAP recording process (PID: {self.mcap_process.pid}) terminated unexpectedly with exit code: {proc_status}.')
                
                # Attempt to read stderr from the process
                try:
                    stderr_output = self.mcap_process.stderr.read().decode('utf-8', errors='ignore')
                    if stderr_output:
                        self.get_logger().error(f"MONITOR: MCAP process stderr output:\n{stderr_output}")
                    else:
                        self.get_logger().info("MONITOR: No stderr output captured from MCAP process.")
                except Exception as e:
                    self.get_logger().warn(f"MONITOR: Could not read stderr from MCAP process: {e}")

                self.recording_active = False
                # Trigger node shutdown if the recording process dies abnormally
                if rclpy.ok() and not self._is_shutdown:
                    self.get_logger().info("MONITOR: Initiating node shutdown due to MCAP process termination.")
                    # Schedule shutdown in the main ROS thread to avoid issues
                    self.create_timer(0.1, self.shutdown, oneshot=True) # Call shutdown slightly later
            return

        # Check disk space
        try:
            disk_usage = shutil.disk_usage(self.output_dir)
            free_space_gb = disk_usage.free / (1024**3)
            threshold_gb = self.disk_threshold_bytes / (1024**3) # For logging
            if disk_usage.free < self.disk_threshold_bytes:
                self.get_logger().error(f"MONITOR: Low disk space detected! Free space: {free_space_gb:.2f} GB. Threshold is {threshold_gb:.2f} GB.")
                self.get_logger().error("MONITOR: Stopping recording and initiating node shutdown.")
                self.recording_active = False
                if rclpy.ok() and not self._is_shutdown:
                    # Schedule shutdown in the main ROS thread
                    self.create_timer(0.1, self.shutdown, oneshot=True)
            # else: # Optional: Info log if check passes
            #     self.get_logger().debug(f"Monitor: Disk space OK ({free_space_gb:.2f} GB free). MCAP process (PID: {self.mcap_process.pid}) is running.")
        except FileNotFoundError:
             # This is critical, as the recording directory is gone.
             self.get_logger().error(f"MONITOR: CRITICAL - Output directory '{self.output_dir}' not found during disk usage check!")
             self.get_logger().error("MONITOR: Cannot monitor disk space. Stopping recording and initiating node shutdown.")
             self.recording_active = False
             if self.monitor_timer: self.monitor_timer.cancel() # Stop further checks immediately
             if rclpy.ok() and not self._is_shutdown:
                 # Schedule shutdown in the main ROS thread
                 self.create_timer(0.1, self.shutdown, oneshot=True)
        except Exception as e:
            self.get_logger().warn(f"Monitor: Could not check disk space for '{self.output_dir}'. Error: {e}")
            # Decide if this warrants stopping: maybe allow a few failures? For now, just warn.

    def save_metadata(self):
        """Saves recording parameters and context to a YAML file."""
        if not hasattr(self, 'output_dir') or not self.output_dir:
             self.get_logger().error("Metadata: Cannot save metadata, output directory not set.")
             return
             
        metadata = {
            'recording_start_time_str': self.start_time_str,
            'output_directory': self.output_dir,
            'parameters': {
                'output_base_dir': self.output_base_dir,
                'rtsp_url': self.rtsp_url,
                'topics_to_record': list(self.topics_to_record), # Convert tuple to list for YAML
                'monitor_interval_sec': self.monitor_interval,
                'disk_threshold_gb': self.get_parameter('disk_threshold_gb').get_parameter_value().double_value,
                'rtsp_reconnect_delay_sec': self.reconnect_delay,
                'shutdown_timeout_sec': self.shutdown_timeout,
            },
            'ros_details': { # Add more runtime context
                 'ros_distribution': os.environ.get('ROS_DISTRO', 'unknown'),
                 # Add git commit hash if available (requires git command execution)
                 # 'git_commit': self.get_git_commit(),
            }
        }

        metadata_path = os.path.join(self.output_dir, 'metadata.yaml')
        temp_metadata_path = metadata_path + ".tmp" # Write to temp file first

        try:
            # Write to a temporary file first
            with open(temp_metadata_path, 'w') as f:
                yaml.dump(metadata, f, default_flow_style=False, sort_keys=False)
            
            # If write successful, rename to the final filename
            os.rename(temp_metadata_path, metadata_path)
            self.get_logger().info(f'Metadata: Saved successfully to {metadata_path}')
        except Exception as e:
            self.get_logger().error(f'Metadata: Failed to save to {metadata_path}: {e}')
            # Clean up temp file if it exists
            if os.path.exists(temp_metadata_path):
                try:
                    os.remove(temp_metadata_path)
                except OSError as rm_e:
                    self.get_logger().warn(f"Metadata: Could not remove temporary file {temp_metadata_path}: {rm_e}")


    def shutdown(self):
        """Shutdown recorder gracefully."""
        # Use a flag to prevent recursive shutdown calls
        if hasattr(self, '_is_shutdown') and self._is_shutdown:
             # self.get_logger().debug('Shutdown: Already in progress.') # Optional debug
             return
        self._is_shutdown = True # Set flag immediately

        self.get_logger().info('===== INITIATING RECORDER SHUTDOWN =====')

        # Stop the monitoring timer
        if self.monitor_timer:
            self.get_logger().info('Shutdown: Cancelling monitoring timer...')
            self.monitor_timer.cancel()
            self.get_logger().info('Shutdown: Monitoring timer cancelled.')
            self.monitor_timer = None # Ensure it's cleared

        # --- Stop MCAP Recording ---
        if self.recording_active and self.mcap_process and self.mcap_process.poll() is None:
            mcap_pid = self.mcap_process.pid
            self.get_logger().info(f'Shutdown: Attempting graceful shutdown of MCAP process (PID: {mcap_pid})...')
            
            # 1. Send SIGINT (Ctrl+C)
            self.get_logger().info(f'Shutdown: Sending SIGINT to MCAP process group (PID: {mcap_pid})...')
            try:
                 pgid = os.getpgid(mcap_pid)
                 os.killpg(pgid, signal.SIGINT)
                 self.get_logger().info(f'Shutdown: Waiting up to {self.shutdown_timeout} seconds for MCAP process to finish after SIGINT...')
                 try:
                     # Wait for process to terminate
                     self.mcap_process.wait(timeout=self.shutdown_timeout)
                     exit_code = self.mcap_process.returncode
                     self.get_logger().info(f'Shutdown: MCAP process (PID: {mcap_pid}) terminated gracefully after SIGINT (Exit Code: {exit_code}).')
                 except subprocess.TimeoutExpired:
                     # 2. Send SIGTERM if SIGINT timed out
                     self.get_logger().warn(f'Shutdown: MCAP process (PID: {mcap_pid}) did not terminate within {self.shutdown_timeout}s after SIGINT. Sending SIGTERM...')
                     try:
                         # Try SIGTERM on the process group first
                         os.killpg(pgid, signal.SIGTERM)
                     except ProcessLookupError:
                          self.get_logger().warn(f"Shutdown: Process group for PID {mcap_pid} already gone before SIGTERM.")
                     except Exception as term_e:
                          self.get_logger().warn(f"Shutdown: Error sending SIGTERM to process group {pgid}: {term_e}. Trying parent PID {mcap_pid} directly.")
                          try:
                              self.mcap_process.terminate() # Send SIGTERM to parent
                          except ProcessLookupError:
                               self.get_logger().warn(f"Shutdown: Process PID {mcap_pid} already gone before direct SIGTERM.")

                     term_wait_timeout = 5 # Shorter timeout after SIGTERM
                     self.get_logger().info(f'Shutdown: Waiting up to {term_wait_timeout} seconds for MCAP process to finish after SIGTERM...')
                     try:
                         self.mcap_process.wait(timeout=term_wait_timeout)
                         exit_code = self.mcap_process.returncode
                         self.get_logger().info(f'Shutdown: MCAP process (PID: {mcap_pid}) terminated after SIGTERM (Exit Code: {exit_code}).')
                     except subprocess.TimeoutExpired:
                          # 3. Send SIGKILL if SIGTERM timed out
                          self.get_logger().error(f'Shutdown: MCAP process (PID: {mcap_pid}) did not terminate after SIGTERM. Sending SIGKILL (Forcing termination)...')
                          try:
                              # Try SIGKILL on the process group first
                              os.killpg(pgid, signal.SIGKILL)
                          except ProcessLookupError:
                              self.get_logger().warn(f"Shutdown: Process group for PID {mcap_pid} already gone before SIGKILL.")
                          except Exception as kill_e:
                              self.get_logger().warn(f"Shutdown: Error sending SIGKILL to process group {pgid}: {kill_e}. Trying parent PID {mcap_pid} directly.")
                              try:
                                   self.mcap_process.kill() # Send SIGKILL to parent
                              except ProcessLookupError:
                                   self.get_logger().warn(f"Shutdown: Process PID {mcap_pid} already gone before direct SIGKILL.")

                          try:
                              self.mcap_process.wait(timeout=2) # Wait briefly after kill
                              exit_code = self.mcap_process.returncode
                              self.get_logger().info(f'Shutdown: MCAP process (PID: {mcap_pid}) terminated after SIGKILL (Exit Code: {exit_code}).')
                          except Exception as final_wait_e:
                               self.get_logger().error(f"Shutdown: Error or timeout waiting after SIGKILL for PID {mcap_pid}: {final_wait_e}")
            except ProcessLookupError:
                 self.get_logger().warn(f"Shutdown: MCAP process (PID: {mcap_pid}) was already gone before sending SIGINT.")
            except Exception as e:
                 self.get_logger().error(f"Shutdown: Error during MCAP process shutdown sequence for PID {mcap_pid}: {e}")
                 tb_str = traceback.format_exc()
                 self.get_logger().error(f"Shutdown: Traceback:\n{tb_str}")

        elif self.mcap_process and self.mcap_process.poll() is not None:
             self.get_logger().info(f"Shutdown: MCAP process (PID: {self.mcap_process.pid}) had already terminated before shutdown sequence.")
        elif not self.recording_active:
             self.get_logger().info("Shutdown: MCAP recording was not active.")

        self.recording_active = False
        self.mcap_process = None # Clear the process handle

        # --- Stop RTSP Thread ---
        # The RTSP thread checks rclpy.ok() and self._is_shutdown in its loop.
        # Setting self._is_shutdown should signal it to stop.
        # Wait briefly for RTSP thread to finish.
        if hasattr(self, 'rtsp_thread') and self.rtsp_thread.is_alive():
             self.get_logger().info("Shutdown: Waiting for RTSP receiver thread to finish...")
             self.rtsp_thread.join(timeout=3.0) # Wait a few seconds
             if self.rtsp_thread.is_alive():
                 self.get_logger().warn("Shutdown: RTSP receiver thread did not finish within timeout.")
             else:
                 self.get_logger().info("Shutdown: RTSP receiver thread finished.")

        self.get_logger().info('===== RECORDER SHUTDOWN COMPLETE =====')

def main(args=None):
    rclpy.init(args=args)
    recorder = None
    exit_code = 0
    try:
        recorder = RecorderNode()
        # Check if initialization failed (e.g., directory creation)
        if not hasattr(recorder, 'output_dir') or not recorder.output_dir:
             # Error logged in __init__, node will likely fail anyway
             raise RuntimeError("RecorderNode initialization failed critically (e.g., output directory).")

        recorder.get_logger().info("Recorder node spinning...")
        rclpy.spin(recorder)

    except KeyboardInterrupt:
        if recorder: recorder.get_logger().info('KeyboardInterrupt received. Initiating shutdown...')
        else: print('KeyboardInterrupt received before node fully initialized.')
        # Shutdown is handled in finally
    except Exception as e:
        exit_code = 1 # Indicate error exit
        if recorder:
            tb_str = traceback.format_exc()
            recorder.get_logger().fatal(f"FATAL: Unhandled exception in main loop: {e}\n{tb_str}")
        else:
            tb_str = traceback.format_exc()
            print(f"FATAL: Unhandled exception before or during node initialization: {e}\n{tb_str}")
    finally:
        print("Executing finally block...") # Use print as logger might be gone
        if recorder:
            print("Calling recorder.shutdown()...") # Use print
            recorder.shutdown() # Ensure shutdown is called
            print("Calling recorder.destroy_node()...") # Use print
            recorder.destroy_node()
        else:
             print("Recorder object not available in finally block.")

        # Use try_shutdown for better handling if already shut down
        print("Calling rclpy.try_shutdown()...") # Use print
        rclpy.try_shutdown()
        print("rclpy shutdown sequence complete.")
        # Consider exiting with the captured exit code
        # import sys
        # sys.exit(exit_code)


if __name__ == '__main__':
    main()