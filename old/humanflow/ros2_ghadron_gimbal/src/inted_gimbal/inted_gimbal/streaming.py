import sys
import gi
import numpy as np
import cv2
import time
import threading
from collections import deque

# Require GStreamer 1.0
gi.require_version('Gst', '1.0')
gi.require_version('GstApp', '1.0')
from gi.repository import Gst, GLib, GstApp

class Streaming:
    def __init__(self):
        # Configuration parameters
        self.rtsp_url = 'rtsp://10.3.1.124:8554/ghadron'
        self.WIDTH = 640
        self.HEIGHT = 512
        self.fps = 10
        self.is_running = True
        self.retry_count = 0
        self.retry_max = 10
        self.retry_delay = 2.0
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
        # Try different GPU accelerated decoders based on system capabilities
        try:
            # First try Jetson-specific hardware decoder (Orin NX)
            jetson_pipeline = (
                f"rtspsrc location={self.rtsp_url} latency=0 ! queue ! "
                "rtph265depay ! queue ! h265parse ! queue ! nvv4l2decoder enable-max-performance=1 ! queue ! "
                "nvvidconv ! "
                f"video/x-raw(memory:NVMM),width={self.WIDTH},height={self.HEIGHT} ! queue ! "
                "videorate ! video/x-raw(memory:NVMM),framerate=5/1 ! queue ! "
                "nvvidconv ! video/x-raw ! queue ! " 
                "videoconvert ! video/x-raw,format=BGR ! queue ! "
                "appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true"
            )
            
            # Try to create Jetson Pipeline
            test_pipeline = Gst.parse_launch(jetson_pipeline)
            test_pipeline.set_state(Gst.State.NULL)  # Release resources immediately
            print("Using Jetson Orin NX hardware accelerated decoder")
            return jetson_pipeline
            
        except Exception as e:
            print(f"Error when trying Jetson decoder: {str(e)}")
            try:
                # Standard NVIDIA GPU decoder
                nvidia_pipeline = (
                    f"rtspsrc location={self.rtsp_url} latency=0 ! queue ! "
                    "rtph265depay ! queue ! h265parse ! queue ! nvh265dec ! queue ! "
                    "videorate ! video/x-raw,framerate=5/1 ! queue ! "
                    f"videoscale ! video/x-raw,width={self.WIDTH},height={self.HEIGHT} ! queue ! "
                    "videoconvert ! video/x-raw,format=BGR ! queue ! "
                    "appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true"
                )
                
                # Try to create NVIDIA Pipeline
                test_pipeline = Gst.parse_launch(nvidia_pipeline)
                test_pipeline.set_state(Gst.State.NULL)  # Release resources immediately
                print("Using NVIDIA GPU accelerated decoder")
                return nvidia_pipeline
                
            except Exception:
                try:
                    # Try VA-API (Intel/AMD GPU)
                    vaapi_pipeline = (
                        f"rtspsrc location={self.rtsp_url} latency=0 ! queue ! "
                        "rtph265depay ! queue ! h265parse ! queue ! vaapih265dec ! queue ! "
                        "videorate ! video/x-raw,framerate=5/1 ! queue ! "
                        f"videoscale ! video/x-raw,width={self.WIDTH},height={self.HEIGHT} ! queue ! "
                        "videoconvert ! video/x-raw,format=BGR ! queue ! "
                        "appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true"
                    )
                    
                    # Try to create VA-API Pipeline
                    test_pipeline = Gst.parse_launch(vaapi_pipeline)
                    test_pipeline.set_state(Gst.State.NULL)  # Release resources immediately
                    print("Using VA-API GPU accelerated decoder")
                    return vaapi_pipeline
                    
                except Exception:
                    # Fallback to software decoding
                    print("No GPU accelerated decoder found, using software decoder")
                    return (
                        f"rtspsrc location={self.rtsp_url} latency=0 buffer-mode=auto ! queue ! "
                        "rtph265depay ! queue ! h265parse ! queue ! avdec_h265 max-threads=4 ! queue ! "
                        "videorate ! video/x-raw,framerate=5/1 ! queue ! "
                        f"videoscale ! video/x-raw,width={self.WIDTH},height={self.HEIGHT} ! queue ! "
                        "videoconvert ! video/x-raw,format=BGR ! queue ! "
                        "appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true"
                    )
        
    def start_pipeline(self):
        """Start GStreamer media streaming (compatible with old function name)"""
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
            
    # For backward compatibility, add an alias
    # start_ffmpeg = start_pipeline
            
    def on_new_sample(self, sink, data):
        """Process new GStreamer sample"""
        try:
            # Get sample
            sample = sink.emit("pull-sample")
            if not sample:
                return Gst.FlowReturn.ERROR
                
            # Get buffer from sample
            buffer = sample.get_buffer()
            
            # Get timestamp
            timestamp = time.time()
            
            # Get memory from buffer
            success, map_info = buffer.map(Gst.MapFlags.READ)
            if not success:
                return Gst.FlowReturn.ERROR
                
            # Copy data and convert to NumPy array
            frame_data = np.ndarray(
                shape=(self.HEIGHT, self.WIDTH, 3),
                dtype=np.uint8,
                buffer=map_info.data
            ).copy()  # Create a copy, can't use same memory after unlocking
            
            # Release buffer
            buffer.unmap(map_info)
            
            # Add frame and timestamp to buffer
            with self.buffer_lock:
                self.frame_buffer.append((frame_data, timestamp))
                self.frame_count += 1
                
            # Periodically log frame rate
            current_time = time.time()
            if current_time - self.last_log_time > 5.0:
                elapsed = current_time - self.last_log_time
                fps = self.frame_count / elapsed if elapsed > 0 else 0
                print(f"GStreamer frame rate: {fps:.2f} FPS (received {self.frame_count} frames)")
                self.frame_count = 0
                self.last_log_time = current_time
                
            return Gst.FlowReturn.OK
            
        except Exception as e:
            print(f"Error processing frame: {str(e)}")
            return Gst.FlowReturn.ERROR
            
    def run_loop(self):
        """Run GLib main loop in a separate thread"""
        try:
            self.loop.run()
        except Exception as e:
            print(f"GStreamer loop error: {str(e)}")
            
    def retrieve_image(self):
        """Get the latest image frame and timestamp (compatible with old interface)"""
        with self.buffer_lock:
            if not self.frame_buffer:
                return None, 0
            frame, timestamp = self.frame_buffer[-1]  # Get the latest frame
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
            
        print("GStreamer streaming closed")


def main():
    """Main function to set up and run the GStreamer pipeline."""

    # --- Configuration ---
    # Replace with your actual RTSP stream URL
    rtsp_url = "rtsp://10.3.1.124:8554/ghadron"
    # Example: rtsp://wowzaec2demo.streamlock.net/vod/mp4:BigBuckBunny_115k.mov

    # --- Initialization ---
    Gst.init(sys.argv[1:]) # Initialize GStreamer, passing command line args

    # --- Create Pipeline ---
    # Build the pipeline string. This is a common pipeline for H.264 RTSP streams.
    # You might need to adjust rtph264depay/h264parse/avdec_h264 based on the
    # actual codec used by your stream (e.g., H.265 -> rtph265depay, h265parse, avdec_h265).
    #
    # Breakdown:
    # rtspsrc: Connects to the RTSP URL and receives RTP packets.
    #          'latency=0' tries to minimize buffering delay.
    #          'location': The URL of the stream.
    # rtph264depay: Extracts H.264 video data from RTP packets.
    # h264parse: Parses the H.264 stream for downstream elements.
    # avdec_h264: Decodes the H.264 video (requires gstreamer1.0-libav).
    #             Alternatively, use hardware decoders if available (e.g., nvdec_h264).
    # videoconvert: Converts the decoded video format if needed by the sink.
    # autovideosink: Automatically selects the best available video sink for display.
    pipeline_str = (
        f"rtspsrc location={rtsp_url} latency=0 ! "
        "rtph265depay ! h265parse ! avdec_h265 ! "
        "videorate ! video/x-raw,framerate=5/1 ! "  # 限制帧率到10fps
        "videoconvert ! autovideosink"
    )

    print(f"Using pipeline: {pipeline_str}")

    try:
        pipeline = Gst.parse_launch(pipeline_str)
    except GLib.Error as e:
        print(f"Error creating pipeline: {e}")
        print("Check if all necessary GStreamer plugins are installed:")
        print("  gstreamer1.0-plugins-base, gstreamer1.0-plugins-good,")
        print("  gstreamer1.0-plugins-bad, gstreamer1.0-plugins-ugly,")
        print("  gstreamer1.0-libav (for avdec_h264)")
        sys.exit(1)

    # --- Create Main Loop ---
    # A GLib Main Loop is needed to run the pipeline and handle messages.
    loop = GLib.MainLoop()

    # --- Bus Message Handling ---
    # The bus listens for messages from the pipeline elements (errors, EOS, etc.)
    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect("message", on_message, loop) # Call on_message when a message arrives

    # --- Start Pipeline ---
    print("Starting pipeline...")
    ret = pipeline.set_state(Gst.State.PLAYING)
    if ret == Gst.StateChangeReturn.FAILURE:
        print("Error: Unable to set the pipeline to the playing state.")
        pipeline.set_state(Gst.State.NULL)
        sys.exit(1)
    elif ret == Gst.StateChangeReturn.NO_PREROLL:
         print("Pipeline is live and does not preroll.")
    elif ret == Gst.StateChangeReturn.ASYNC:
         print("Pipeline is starting asynchronously.")
    else:
        print("Pipeline state set to PLAYING.")


    # --- Run Main Loop ---
    try:
        print("Running main loop (Press Ctrl+C to quit)...")
        loop.run()
    except KeyboardInterrupt:
        print("Ctrl+C pressed, shutting down.")
    finally:
        # --- Cleanup ---
        print("Stopping pipeline...")
        pipeline.set_state(Gst.State.NULL)
        print("Pipeline stopped.")
        loop.quit() # Ensure loop quits if not already done

def on_message(bus, message, loop):
    """Callback function to handle messages from the GStreamer bus."""
    mtype = message.type

    if mtype == Gst.MessageType.EOS:
        # End-of-stream message
        print("End-of-Stream reached.")
        loop.quit()
    elif mtype == Gst.MessageType.ERROR:
        # Error message
        err, debug = message.parse_error()
        print(f"Error received from element {message.src.get_name()}: {err.message}")
        print(f"Debugging information: {debug if debug else 'none'}")
        loop.quit()
    elif mtype == Gst.MessageType.WARNING:
        # Warning message
        warn, debug = message.parse_warning()
        print(f"Warning received from element {message.src.get_name()}: {warn.message}")
        print(f"Debugging information: {debug if debug else 'none'}")
    elif mtype == Gst.MessageType.STATE_CHANGED:
        # Optional: Handle state changes if needed (useful for debugging)
        if message.src == pipeline: # Filter messages from the pipeline itself
            old_state, new_state, pending_state = message.parse_state_changed()
            # print(f"Pipeline state changed from {Gst.Element.state_get_name(old_state)} to {Gst.Element.state_get_name(new_state)}")
            pass # Usually not needed for basic playback
    else:
        # Handle other message types if necessary
        # print(f"Received message type: {mtype}")
        pass

    return True # Important: Return True to keep the signal watch active


if __name__ == '__main__':
    # Need to assign pipeline to a global or pass it correctly if on_message needs it
    # For this simple example, we only need the loop in on_message
    pipeline = None
    main()

        
        
