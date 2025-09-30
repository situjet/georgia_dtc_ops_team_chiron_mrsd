#!/usr/bin/env python3
"""
simple_mcap_processor.py - Simplified bag processor
Supports both MCAP and ROS 2 rosbag (.db3) files
Designed for single-file single-target scenarios, removing unnecessary complexity
"""

import sys
import subprocess
import os
import gzip
from pathlib import Path

def process_bag_file(filepath):
    """Process both MCAP and ROS 2 rosbag files"""
    filepath = Path(filepath)
    
    # Unified processing completion marker directory
    processed_dir = filepath.parent / '.processed'
    processed_dir.mkdir(exist_ok=True)
    
    # Marker file name based on original file
    marker_file = processed_dir / f"{filepath.name}.done"
    
    # Check if already processed
    if marker_file.exists():
        print(f"Already processed: {filepath}")
        return True
    
    try:
        print(f"Processing: {filepath}")
        
        # Determine file type and processing method
        file_extension = filepath.suffix.lower()
        
        if file_extension == '.gz':
            # Handle compressed files
            result = subprocess.run(['gzip', '-t', str(filepath)], 
                                  capture_output=True, text=True)
            if result.returncode != 0:
                print(f"ERROR: Corrupted gzip file: {filepath}")
                return False
            
            # Decompress
            output_path = filepath.with_suffix('')
            print(f"Decompressing to: {output_path}")
            
            with gzip.open(filepath, 'rb') as f_in:
                with open(output_path, 'wb') as f_out:
                    f_out.write(f_in.read())
            
            processed_file = output_path
            file_extension = processed_file.suffix.lower()
        else:
            processed_file = filepath
        
        # Get UDP target from environment variables
        udp_ip = os.environ.get('UDP_TARGET_IP', '10.3.1.106')
        udp_port = os.environ.get('UDP_TARGET_PORT', '5005')
        
        # Process based on file type
        if file_extension == '.mcap':
            # Process MCAP file using existing bridge
            print(f"Processing MCAP file: {processed_file}")
            success = process_mcap_file(processed_file, udp_ip, udp_port)
        elif file_extension == '.db3':
            # Process ROS 2 rosbag file
            print(f"Processing ROS 2 rosbag file: {processed_file}")
            success = process_rosbag_file(processed_file, udp_ip, udp_port)
        else:
            print(f"ERROR: Unsupported file type: {file_extension}")
            return False
        
        if success:
            # Mark as processed
            marker_file.touch()
            print(f"Processing completed: {filepath}")
            return True
        else:
            print(f"Processing failed: {filepath}")
            return False
        
    except subprocess.CalledProcessError as e:
        print(f"Command failed: {e}")
        return False
    except Exception as e:
        print(f"ERROR processing {filepath}: {e}")
        return False

def process_mcap_file(mcap_file, udp_ip, udp_port):
    """Process MCAP file using existing bridge"""
    try:
        # Get script directory
        script_dir = Path(__file__).parent
        bridge_script = script_dir / "mcap_to_udp_bridge.py"
        
        # Get workspace root (from mcap_receiver/mcap_receiver to ros_ws)
        workspace_root = script_dir.parent.parent.parent
        install_setup = workspace_root / "install" / "setup.bash"
        
        cmd = [
            'bash', '-c',
            f'set +u && source /opt/ros/humble/setup.bash && '
            f'source {install_setup} && '
            f'python3 {bridge_script} {mcap_file} --udp-ip {udp_ip} --udp-port {udp_port}'
        ]
        
        # Execute synchronously, wait for bridge completion
        result = subprocess.run(cmd)
        
        if result.returncode == 0:
            print(f"MCAP processing completed successfully: {mcap_file}")
            return True
        else:
            print(f"MCAP processing failed with exit code: {result.returncode}")
            return False
            
    except Exception as e:
        print(f"Error processing MCAP file: {e}")
        return False

def process_rosbag_file(db3_file, udp_ip, udp_port):
    """Process ROS 2 rosbag (.db3) file"""
    try:
        # Get script directory
        script_dir = Path(__file__).parent
        rosbag_bridge_script = script_dir / "rosbag_to_udp_bridge.py"
        
        # Check if rosbag bridge exists, if not create it
        if not rosbag_bridge_script.exists():
            print("Creating rosbag bridge script...")
            create_rosbag_bridge_script(rosbag_bridge_script)
        
        # Get workspace root (from mcap_receiver/mcap_receiver to ros_ws)
        workspace_root = script_dir.parent.parent.parent
        install_setup = workspace_root / "install" / "setup.bash"
        
        cmd = [
            'bash', '-c',
            f'set +u && source /opt/ros/humble/setup.bash && '
            f'source {install_setup} && '
            f'python3 {rosbag_bridge_script} {db3_file} --udp-ip {udp_ip} --udp-port {udp_port}'
        ]
        
        # Execute synchronously, wait for bridge completion
        result = subprocess.run(cmd)
        
        if result.returncode == 0:
            print(f"Rosbag processing completed successfully: {db3_file}")
            return True
        else:
            print(f"Rosbag processing failed with exit code: {result.returncode}")
            return False
            
    except Exception as e:
        print(f"Error processing rosbag file: {e}")
        return False

def create_rosbag_bridge_script(script_path):
    """Create a bridge script for ROS 2 rosbag files"""
    script_content = '''#!/usr/bin/env python3
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
        # Handle rosbag path - can be directory or .db3 file
        rosbag_path = Path(args.rosbag_path)
        if rosbag_path.is_file() and rosbag_path.suffix == '.db3':
            # For single .db3 file, use the file directly
            rosbag_target = str(rosbag_path)
        else:
            # For directory, use directory
            rosbag_target = str(rosbag_path)
        
        print(f"Processing rosbag: {rosbag_target}")
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
            'ros2', 'bag', 'play', rosbag_target,
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
        print("\\nInterrupted by user")
        return 1
    except Exception as e:
        print(f"Error: {e}")
        return 1

if __name__ == '__main__':
    sys.exit(main())
'''
    
    with open(script_path, 'w') as f:
        f.write(script_content)
    
    # Make script executable
    os.chmod(script_path, 0o755)
    print(f"Created rosbag bridge script: {script_path}")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Simple Bag Processor with Bridge')
    parser.add_argument('bag_file', nargs='?', help='Path to bag file (.mcap, .mcap.gz, or .db3)')
    parser.add_argument('--version', action='version', version='Simple Bag Processor 1.0')
    
    args = parser.parse_args()
    
    if not args.bag_file:
        parser.print_help()
        sys.exit(1)
    
    filepath = args.bag_file
    if not os.path.exists(filepath):
        print(f"File not found: {filepath}")
        sys.exit(1)
    
    success = process_bag_file(filepath)
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()
