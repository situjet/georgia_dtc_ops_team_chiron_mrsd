#!/usr/bin/env python3
"""
mcap_to_udp_bridge.py - MCAP to UDP bridge
coordinates MCAP replayer and rostopic_publisher, implements end-to-end message transmission
"""

import subprocess
import time
import signal
import sys
import os
import threading
from pathlib import Path
import json
import tempfile

class McapToUdpBridge:
    """MCAP to UDP bridge"""
    
    def __init__(self, mcap_file: str, udp_ip: str = "10.3.1.106", udp_port: int = 5005):
        self.mcap_file = Path(mcap_file)
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.topic_prefix = "/mcap_replay"
        
        self.replayer_process = None
        self.publisher_processes = []
        self.is_running = False
        
        # verify MCAP file
        if not self.mcap_file.exists():
            raise FileNotFoundError(f"MCAP file not found: {mcap_file}")
        
        print(f"Initialized bridge for: {mcap_file}")
        print(f"Target UDP: {udp_ip}:{udp_port}")
        
        # register signal handler
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def signal_handler(self, signum, frame):
        """signal handler"""
        print(f"\nReceived signal {signum}, shutting down...")
        self.stop()
        sys.exit(0)
    
    def analyze_mcap_topics(self):
        """analyze topics in MCAP file"""
        try:
            # analyze file using replayer
            cmd = [
                'python3', 
                str(Path(__file__).parent / 'mcap_to_rostopic_replayer.py'),
                str(self.mcap_file),
                '--analyze-only'
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            
            if result.returncode == 0:
                print("MCAP Analysis:")
                print(result.stdout)
                return True
            else:
                print(f"Failed to analyze MCAP: {result.stderr}")
                return False
                
        except Exception as e:
            print(f"Error analyzing MCAP: {e}")
            return False
    
    def start_replayer(self):
        """start MCAP replayer"""
        try:
            cmd = [
                'python3',
                str(Path(__file__).parent / 'mcap_to_rostopic_replayer.py'),
                str(self.mcap_file),
                '--topic-prefix', self.topic_prefix
            ]
            
            print("Starting MCAP replayer...")
            self.replayer_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            # wait for replayer to start
            time.sleep(3)
            
            if self.replayer_process.poll() is None:
                print("✅ MCAP replayer started successfully")
                return True
            else:
                stdout, stderr = self.replayer_process.communicate()
                print(f"❌ MCAP replayer failed to start:")
                print(f"STDOUT: {stdout.decode()}")
                print(f"STDERR: {stderr.decode()}")
                return False
                
        except Exception as e:
            print(f"Error starting replayer: {e}")
            return False
    
    def start_publisher_for_topic(self, original_topic: str, topic_type: str):
        """start rostopic_publisher for specific topic"""
        try:
            # construct replay topic name
            replay_topic = f"{self.topic_prefix}{original_topic}"
            
            # start rostopic_publisher
            cmd = [
                'ros2', 'run', 'one_way_comms_udp', 'rostopic_publisher',
                '--ros-args',
                '-p', f'topic_name:={replay_topic}',
                '-p', f'topic_type:={topic_type}',
                '-p', f'udp_ip:={self.udp_ip}',
                '-p', f'udp_port:={self.udp_port}'
            ]
            
            print(f"Starting publisher for {replay_topic} -> {self.udp_ip}:{self.udp_port}")
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            self.publisher_processes.append({
                'process': process,
                'topic': replay_topic,
                'type': topic_type
            })
            
            return True
            
        except Exception as e:
            print(f"Error starting publisher for {original_topic}: {e}")
            return False
    
    def start_auto_publishers(self):
        """automatically start all required publishers"""
        # Here we use a generic method
        # In actual use, you may need to configure based on specific MCAP content
        
        # example: start publisher for common topic types
        common_topics = [
            ("/sensor_data", "sensor_msgs/msg/PointCloud2"),
            ("/camera/image", "sensor_msgs/msg/Image"),
            ("/drone/data", "dtc_network_msgs/msg/HumanDataMsg"),
            ("/status", "std_msgs/msg/String")
        ]
        
        print("Starting publishers for common topics...")
        for topic, topic_type in common_topics:
            self.start_publisher_for_topic(topic, topic_type)
            time.sleep(1)  # avoid starting too many processes simultaneously
    
    def start_single_publisher(self, topic_name: str, topic_type: str):
        """start single publisher (simplified version)"""
        try:
            replay_topic = f"{self.topic_prefix}{topic_name}"
            
            cmd = [
                'ros2', 'run', 'one_way_comms_udp', 'rostopic_publisher',
                '--ros-args',
                '-p', f'topic_name:={replay_topic}',
                '-p', f'topic_type:={topic_type}',
                '-p', f'udp_ip:={self.udp_ip}',
                '-p', f'udp_port:={self.udp_port}'
            ]
            
            print(f"Starting single publisher: {replay_topic} ({topic_type})")
            process = subprocess.Popen(cmd)
            
            self.publisher_processes.append({
                'process': process,
                'topic': replay_topic,
                'type': topic_type
            })
            
            return True
            
        except Exception as e:
            print(f"Error starting single publisher: {e}")
            return False
    
    def start(self, topic_name: str = None, topic_type: str = None):
        """start bridge system"""
        if self.is_running:
            print("Bridge already running")
            return
        
        print("🚀 Starting MCAP to UDP Bridge...")
        
        # 1. analyze MCAP file
        if not self.analyze_mcap_topics():
            print("❌ Failed to analyze MCAP file")
            return False
        
        # 2. start replayer
        if not self.start_replayer():
            print("❌ Failed to start replayer")
            return False
        
        # 3. start publisher(s)
        if topic_name and topic_type:
            # start single specified publisher
            if not self.start_single_publisher(topic_name, topic_type):
                print("❌ Failed to start publisher")
                self.stop()
                return False
        else:
            # start auto publishers
            self.start_auto_publishers()
        
        self.is_running = True
        print("✅ Bridge started successfully!")
        print(f"📡 MCAP messages will be replayed and forwarded to {self.udp_ip}:{self.udp_port}")
        
        return True
    
    def stop(self):
        """stop bridge system"""
        if not self.is_running:
            return
        
        print("🛑 Stopping MCAP to UDP Bridge...")
        
        # stop all publisher processes
        for pub_info in self.publisher_processes:
            try:
                process = pub_info['process']
                if process.poll() is None:
                    process.terminate()
                    process.wait(timeout=5)
            except Exception as e:
                print(f"Error stopping publisher: {e}")
        
        # stop replayer
        if self.replayer_process and self.replayer_process.poll() is None:
            try:
                self.replayer_process.terminate()
                self.replayer_process.wait(timeout=5)
            except Exception as e:
                print(f"Error stopping replayer: {e}")
        
        self.is_running = False
        print("✅ Bridge stopped")
    
    def wait(self):
        """wait for bridge system completion"""
        try:
            if self.replayer_process:
                self.replayer_process.wait()
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

def main():
    """main function"""
    import argparse
    
    parser = argparse.ArgumentParser(description='MCAP to UDP Bridge')
    parser.add_argument('mcap_file', help='Path to MCAP file')
    parser.add_argument('--udp-ip', default='10.3.1.106', 
                       help='Target UDP IP (default: 10.3.1.106)')
    parser.add_argument('--udp-port', type=int, default=5005,
                       help='Target UDP port (default: 5005)')
    parser.add_argument('--topic-name', 
                       help='Specific topic to bridge (e.g., /sensor_data)')
    parser.add_argument('--topic-type',
                       help='Topic message type (e.g., sensor_msgs/msg/PointCloud2)')
    parser.add_argument('--analyze-only', action='store_true',
                       help='Only analyze MCAP file, do not start bridge')
    
    args = parser.parse_args()
    
    try:
        bridge = McapToUdpBridge(args.mcap_file, args.udp_ip, args.udp_port)
        
        if args.analyze_only:
            bridge.analyze_mcap_topics()
        else:
            if bridge.start(args.topic_name, args.topic_type):
                print("\n🎮 Bridge is running. Press Ctrl+C to stop.")
                bridge.wait()
            else:
                print("❌ Failed to start bridge")
                sys.exit(1)
                
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()

