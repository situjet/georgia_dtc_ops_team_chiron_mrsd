#!/usr/bin/env python3
"""
Enhanced Streaming Module for AirStack Integration
=================================================

Refactored from original humanflow streaming.py with:
- ROS2 parameter integration
- Improved PTS timestamp handling
- Enhanced error recovery
- Diagnostic reporting
"""

import sys
import gi
import numpy as np
import cv2
import time
import threading
from collections import deque
from typing import Optional, Tuple, Dict, Any
import logging

# Require GStreamer 1.0
gi.require_version('Gst', '1.0')
gi.require_version('GstApp', '1.0')
from gi.repository import Gst, GLib, GstApp

# Configure logging
logger = logging.getLogger(__name__)


class StreamingModule:
    """Enhanced streaming module with ROS2 integration and improved error handling."""
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize streaming module with configuration.
        
        Args:
            config: Configuration dictionary with streaming parameters
        """
        # Configuration from ROS2 parameters
        self.rtsp_url = config.get('rtsp_url', 'rtsp://10.3.1.124:8554/ghadron')
        self.width = config.get('width', 640)
        self.height = config.get('height', 512)
        self.fps = config.get('fps', 10)
        self.retry_max = config.get('retry_max', 10)
        self.retry_delay = config.get('retry_delay', 2.0)
        self.buffer_size = config.get('buffer_size', 10)
        
        # State management
        self.is_running = True
        self.retry_count = 0
        self.frame_count = 0
        self.last_log_time = time.time()
        
        # Timestamp alignment for ROS2
        self.ros_time_offset = None
        self.first_pts = None
        
        # Initialize GStreamer
        if not Gst.is_initialized():
            Gst.init(None)
        
        # Pipeline components
        self.pipeline = None
        self.appsink = None
        
        # Main loop and thread
        self.loop = GLib.MainLoop()
        self.loop_thread = None
        
        # Frame buffer with thread safety
        self.frame_buffer = deque(maxlen=self.buffer_size)
        self.buffer_lock = threading.Lock()
        
        # Diagnostics
        self.stats = {
            'frames_received': 0,
            'frames_dropped': 0,
            'reconnections': 0,
            'last_frame_time': 0,
            'average_fps': 0.0
        }
        
        logger.info(f"StreamingModule initialized: {self.width}x{self.height}@{self.fps}fps")

    def get_pipeline_string(self) -> str:
        """
        Get optimized GStreamer pipeline string with hardware acceleration fallback.
        
        Returns:
            Pipeline string for GStreamer
        """
        base_params = f"location={self.rtsp_url} latency=0"
        common_elements = (
            f"videorate ! video/x-raw,framerate={self.fps}/1 ! queue ! "
            f"videoscale ! video/x-raw,width={self.width},height={self.height} ! queue ! "
            "videoconvert ! video/x-raw,format=BGR ! queue ! "
            "appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true"
        )
        
        # Hardware acceleration options in order of preference
        hw_pipelines = [
            # Jetson Orin NX hardware decoder
            (
                f"rtspsrc {base_params} ! queue ! "
                "rtph265depay ! queue ! h265parse ! queue ! nvv4l2decoder enable-max-performance=1 ! queue ! "
                f"nvvidconv ! video/x-raw(memory:NVMM),width={self.width},height={self.height} ! queue ! "
                f"videorate ! video/x-raw(memory:NVMM),framerate={self.fps}/1 ! queue ! "
                "nvvidconv ! video/x-raw ! queue ! "
                "videoconvert ! video/x-raw,format=BGR ! queue ! "
                "appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true"
            ),
            # NVIDIA GPU decoder
            (
                f"rtspsrc {base_params} ! queue ! "
                "rtph265depay ! queue ! h265parse ! queue ! nvh265dec ! queue ! "
                + common_elements
            ),
            # VA-API (Intel/AMD GPU)
            (
                f"rtspsrc {base_params} ! queue ! "
                "rtph265depay ! queue ! h265parse ! queue ! vaapih265dec ! queue ! "
                + common_elements
            ),
            # Software fallback
            (
                f"rtspsrc {base_params} buffer-mode=auto ! queue ! "
                "rtph265depay ! queue ! h265parse ! queue ! avdec_h265 max-threads=4 ! queue ! "
                + common_elements
            )
        ]
        
        # Try each pipeline until one works
        for i, pipeline_str in enumerate(hw_pipelines):
            try:
                test_pipeline = Gst.parse_launch(pipeline_str)
                test_pipeline.set_state(Gst.State.NULL)
                
                hw_types = ["Jetson Orin NX", "NVIDIA GPU", "VA-API", "Software"]
                logger.info(f"Using {hw_types[i]} decoder")
                return pipeline_str
                
            except Exception as e:
                logger.debug(f"Failed to create pipeline {i}: {e}")
                continue
        
        # Should never reach here, but return software fallback
        logger.warning("All hardware acceleration failed, using software decoder")
        return hw_pipelines[-1]

    def start_pipeline(self) -> bool:
        """
        Start GStreamer pipeline with enhanced error handling.
        
        Returns:
            True if pipeline started successfully, False otherwise
        """
        try:
            pipeline_str = self.get_pipeline_string()
            logger.info(f"Starting pipeline: {pipeline_str[:100]}...")
            
            # Create pipeline
            self.pipeline = Gst.parse_launch(pipeline_str)
            
            # Get appsink element
            self.appsink = self.pipeline.get_by_name("sink")
            if not self.appsink:
                logger.error("Could not get appsink element")
                return False
            
            # Set callback for new frames
            self.appsink.connect("new-sample", self._on_new_sample, None)
            
            # Set up bus for error handling
            bus = self.pipeline.get_bus()
            bus.add_signal_watch()
            bus.connect("message", self._on_bus_message)
            
            # Start pipeline
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                logger.error("Could not start GStreamer pipeline")
                return False
            
            # Start GLib main loop in separate thread
            self.loop_thread = threading.Thread(target=self._run_loop, daemon=True)
            self.loop_thread.start()
            
            # Initialize timestamp alignment
            self.ros_time_offset = time.time()
            
            logger.info("GStreamer streaming started successfully")
            return True
            
        except Exception as e:
            logger.error(f"Error starting GStreamer: {e}")
            return False

    def _on_new_sample(self, sink, data) -> Gst.FlowReturn:
        """
        Process new GStreamer sample with improved timestamp handling.
        
        Args:
            sink: GStreamer appsink element
            data: User data (unused)
            
        Returns:
            GStreamer flow return status
        """
        try:
            # Get sample
            sample = sink.emit("pull-sample")
            if not sample:
                return Gst.FlowReturn.ERROR
            
            # Get buffer and timestamp
            buffer = sample.get_buffer()
            
            # Extract PTS timestamp and align with ROS time
            frame_timestamp = self._get_aligned_timestamp(buffer)
            
            # Map buffer to access data
            success, map_info = buffer.map(Gst.MapFlags.READ)
            if not success:
                return Gst.FlowReturn.ERROR
            
            # Convert to numpy array
            frame_data = np.ndarray(
                shape=(self.height, self.width, 3),
                dtype=np.uint8,
                buffer=map_info.data
            ).copy()  # Important: copy data before unmapping
            
            # Unmap buffer
            buffer.unmap(map_info)
            
            # Add to buffer with thread safety
            with self.buffer_lock:
                self.frame_buffer.append((frame_data, frame_timestamp))
                self.frame_count += 1
                self.stats['frames_received'] += 1
                self.stats['last_frame_time'] = frame_timestamp
            
            # Update statistics
            self._update_fps_stats()
            
            return Gst.FlowReturn.OK
            
        except Exception as e:
            logger.error(f"Error processing frame: {e}")
            self.stats['frames_dropped'] += 1
            return Gst.FlowReturn.ERROR

    def _get_aligned_timestamp(self, buffer: Gst.Buffer) -> float:
        """
        Get timestamp aligned with ROS time base.
        
        Args:
            buffer: GStreamer buffer with PTS
            
        Returns:
            Timestamp in seconds (float) aligned with ROS time
        """
        try:
            # Try to get PTS timestamp
            pts = buffer.pts
            if pts != Gst.CLOCK_TIME_NONE:
                # Convert nanoseconds to seconds
                pts_sec = pts / 1e9
                
                # Initialize offset on first frame
                if self.first_pts is None:
                    self.first_pts = pts_sec
                    return self.ros_time_offset
                
                # Return aligned timestamp
                return self.ros_time_offset + (pts_sec - self.first_pts)
            else:
                # Fallback to system time
                return time.time()
                
        except Exception:
            # Final fallback
            return time.time()

    def _on_bus_message(self, bus, message) -> bool:
        """
        Handle GStreamer bus messages for error recovery.
        
        Args:
            bus: GStreamer bus
            message: Bus message
            
        Returns:
            True to continue receiving messages
        """
        msg_type = message.type
        
        if msg_type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            logger.error(f"GStreamer error: {err.message}")
            logger.debug(f"Debug info: {debug}")
            
            # Trigger reconnection
            self._schedule_reconnection()
            
        elif msg_type == Gst.MessageType.WARNING:
            warn, debug = message.parse_warning()
            logger.warning(f"GStreamer warning: {warn.message}")
            
        elif msg_type == Gst.MessageType.EOS:
            logger.info("End of stream reached")
            self._schedule_reconnection()
        
        return True

    def _schedule_reconnection(self):
        """Schedule pipeline reconnection with exponential backoff."""
        if self.retry_count < self.retry_max:
            self.retry_count += 1
            delay = min(self.retry_delay * (2 ** (self.retry_count - 1)), 60.0)
            
            logger.info(f"Scheduling reconnection #{self.retry_count} in {delay:.1f}s")
            
            # Stop current pipeline
            if self.pipeline:
                self.pipeline.set_state(Gst.State.NULL)
            
            # Schedule restart
            threading.Timer(delay, self._attempt_reconnection).start()
        else:
            logger.error(f"Max reconnection attempts ({self.retry_max}) reached")

    def _attempt_reconnection(self):
        """Attempt to reconnect the pipeline."""
        logger.info("Attempting pipeline reconnection...")
        
        if self.start_pipeline():
            logger.info("Pipeline reconnection successful")
            self.retry_count = 0  # Reset retry counter
            self.stats['reconnections'] += 1
        else:
            logger.error("Pipeline reconnection failed")
            self._schedule_reconnection()

    def _run_loop(self):
        """Run GLib main loop with error handling."""
        try:
            self.loop.run()
        except Exception as e:
            logger.error(f"GStreamer loop error: {e}")

    def _update_fps_stats(self):
        """Update FPS statistics for monitoring."""
        current_time = time.time()
        if current_time - self.last_log_time > 5.0:
            elapsed = current_time - self.last_log_time
            fps = self.frame_count / elapsed if elapsed > 0 else 0
            self.stats['average_fps'] = fps
            
            logger.debug(f"Streaming FPS: {fps:.2f} (received {self.frame_count} frames)")
            
            self.frame_count = 0
            self.last_log_time = current_time

    def retrieve_image(self) -> Optional[Tuple[np.ndarray, float]]:
        """
        Get the latest image frame and timestamp.
        
        Returns:
            Tuple of (frame, timestamp) or None if no frame available
        """
        with self.buffer_lock:
            if not self.frame_buffer:
                return None
            return self.frame_buffer[-1]  # Return latest frame

    def get_stats(self) -> Dict[str, Any]:
        """
        Get streaming statistics for diagnostics.
        
        Returns:
            Dictionary with streaming statistics
        """
        return self.stats.copy()

    def shutdown(self):
        """Clean shutdown of streaming resources."""
        logger.info("Shutting down streaming module...")
        
        self.is_running = False
        
        # Stop pipeline
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None
        
        # Stop main loop
        if self.loop and self.loop.is_running():
            self.loop.quit()
        
        # Wait for loop thread
        if self.loop_thread and self.loop_thread.is_alive():
            self.loop_thread.join(timeout=3)
        
        logger.info("Streaming module shutdown complete")


# Backwards compatibility alias
Streaming = StreamingModule
