#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
import time

def main():
    # Kill the current rtsp_streamer_node
    subprocess.run(['pkill', '-f', 'rtsp_streamer_node'], capture_output=True)
    time.sleep(2)
    
    # Start new instance with test_mode enabled
    rclpy.init()
    
    from rtsp_streamer.rtsp_streamer_node import RtspStreamer
    
    # Create node directly with test_mode parameter
    node = RtspStreamer()
    node.declare_parameter_if_not_declared('test_mode', True)
    node.set_parameters([rclpy.Parameter('test_mode', rclpy.Parameter.Type.BOOL, True)])
    
    try:
        print("Starting rtsp_streamer in test mode...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()