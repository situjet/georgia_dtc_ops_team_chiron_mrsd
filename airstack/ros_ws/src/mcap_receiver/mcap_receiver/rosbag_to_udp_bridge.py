#!/usr/bin/env python3
"""
rosbag_to_udp_bridge.py - ROS 2 rosbag to UDP bridge
Replays ROS 2 rosbag messages and forwards them via UDP
"""

import subprocess
import sys
import os
import time
from pathlib import Path

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='ROS 2 Rosbag to UDP Bridge')
    parser.add_argument('rosbag_path', help='Path to ROS 2 rosbag directory or .db3 file')
    parser.add_argument('--udp-ip', default='10.3.1.106', 
                       help='Target UDP IP (default: 10.3.1.106)')
    parser.add_argument('--udp-port', type=int, default=5005,
                       help='Target UDP port (default: 5005)')
    parser.add_argument('--topic-name', default='/human_data',
                       help='Topic to bridge (default: /human_data)')
    parser.add_argument('--topic-type', default='dtc_network_msgs/msg/HumanDataMsg',
                       help='Topic message type (default: dtc_network_msgs/msg/HumanDataMsg)')
    
    args = parser.parse_args()
    
    try:
        # Get the directory containing the .db3 file
        rosbag_path = Path(args.rosbag_path)
        if rosbag_path.is_file() and rosbag_path.suffix == '.db3':
            # If it's a .db3 file, use its parent directory
            rosbag_dir = rosbag_path.parent
        else:
            rosbag_dir = rosbag_path
        
        print(f"Processing rosbag: {rosbag_dir}")
        print(f"Target UDP: {args.udp_ip}:{args.udp_port}")
        print(f"Topic: {args.topic_name} ({args.topic_type})")
        
        # Start UDP publisher in background
        publisher_cmd = [
            'ros2', 'run', 'dell_publisher', 'rostopic_publisher',
            '--ros-args',
            '-p', f'topic_name:={args.topic_name}',
            '-p', f'topic_type:={args.topic_type}',
            '-p', f'udp_ip:={args.udp_ip}',
            '-p', f'udp_port:={args.udp_port}'
        ]
        
        print("Starting UDP publisher...")
        publisher_process = subprocess.Popen(publisher_cmd)
        
        # Wait a moment for publisher to start
        time.sleep(2)
        
        # Play the rosbag
        play_cmd = [
            'ros2', 'bag', 'play', str(rosbag_dir),
            '--topics', args.topic_name
        ]
        
        print("Starting rosbag playback...")
        play_result = subprocess.run(play_cmd)
        
        # Stop publisher
        publisher_process.terminate()
        publisher_process.wait(timeout=5)
        
        if play_result.returncode == 0:
            print("Rosbag playback completed successfully")
            return 0
        else:
            print(f"Rosbag playback failed with exit code: {play_result.returncode}")
            return 1
            
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        return 1
    except Exception as e:
        print(f"Error: {e}")
        return 1

if __name__ == '__main__':
    sys.exit(main())
