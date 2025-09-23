#!/usr/bin/env python3
"""
mcap_to_rostopic_replayer.py - MCAP file to ROS Topic replayer
replay messages from MCAP file to local ROS topics in real-time，for rostopic_publisher subscription
"""

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
import time
import importlib
from pathlib import Path
from mcap import reader
from mcap_ros2.reader import read_ros2_messages
import threading
import signal
import sys

class McapToRosTopicReplayer(Node):
    """MCAP to ROS Topic replayer"""
    
    def __init__(self, mcap_file: str, topic_prefix: str = "/mcap_replay"):
        super().__init__('mcap_to_rostopic_replayer')
        
        self.mcap_file = Path(mcap_file)
        self.topic_prefix = topic_prefix
        self.publishers = {}  # {topic_name: publisher}
        self.msg_type_cache = {}  # {schema_name: msg_class}
        self.is_playing = False
        self.replay_thread = None
        
        # verify MCAP file
        if not self.mcap_file.exists():
            self.get_logger().error(f"MCAP file not found: {mcap_file}")
            raise FileNotFoundError(f"MCAP file not found: {mcap_file}")
        
        self.get_logger().info(f"Initialized MCAP replayer for: {mcap_file}")
        self.get_logger().info(f"Topic prefix: {topic_prefix}")
        
        # register signal handler
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def signal_handler(self, signum, frame):
        """signal handler"""
        self.get_logger().info(f"Received signal {signum}, stopping replay...")
        self.stop_replay()
        rclpy.shutdown()
    
    def get_message_type(self, schema_name: str):
        """get message type"""
        if schema_name in self.msg_type_cache:
            return self.msg_type_cache[schema_name]
        
        try:
            # parse schema name (e.g., "sensor_msgs/msg/Image")
            parts = schema_name.split('/')
            if len(parts) == 3 and parts[1] == 'msg':
                pkg_name = parts[0]
                msg_name = parts[2]
                
                # import message module
                msg_module = importlib.import_module(f'{pkg_name}.msg')
                msg_class = getattr(msg_module, msg_name)
                
                self.msg_type_cache[schema_name] = msg_class
                self.get_logger().info(f"Loaded message type: {schema_name}")
                return msg_class
            else:
                self.get_logger().warning(f"Unknown schema format: {schema_name}")
                return None
                
        except (ImportError, AttributeError) as e:
            self.get_logger().error(f"Failed to import message type '{schema_name}': {e}")
            return None
    
    def create_publisher_for_topic(self, original_topic: str, msg_class):
        """create publisher for topic"""
        # create replay topic name
        replay_topic = f"{self.topic_prefix}{original_topic}"
        
        if replay_topic not in self.publishers:
            publisher = self.create_publisher(msg_class, replay_topic, 10)
            self.publishers[replay_topic] = publisher
            self.get_logger().info(f"Created publisher for: {replay_topic} ({msg_class.__name__})")
        
        return self.publishers[replay_topic]
    
    def analyze_mcap_file(self):
        """analyze MCAP file content"""
        try:
            with open(self.mcap_file, 'rb') as f:
                mcap_reader = reader.make_reader(f)
                summary = mcap_reader.get_summary()
                
                if summary:
                    self.get_logger().info(f"MCAP contains {summary.message_count} messages")
                    duration_ns = summary.message_end_time - summary.message_start_time
                    duration_s = duration_ns / 1e9
                    self.get_logger().info(f"Duration: {duration_s:.2f} seconds")
                
                # display all topics
                for channel_id, channel in summary.channels.items():
                    replay_topic = f"{self.topic_prefix}{channel.topic}"
                    self.get_logger().info(f"Found topic: {channel.topic} -> {replay_topic} [{channel.schema.name}]")
                
                return True
                
        except Exception as e:
            self.get_logger().error(f"Failed to analyze MCAP file: {e}")
            return False
    
    def start_replay(self):
        """start replay"""
        if self.is_playing:
            self.get_logger().warning("Replay already in progress")
            return
        
        # analyze file first
        if not self.analyze_mcap_file():
            return
        
        self.is_playing = True
        self.replay_thread = threading.Thread(target=self._replay_loop, daemon=True)
        self.replay_thread.start()
        self.get_logger().info("Started MCAP replay")
    
    def stop_replay(self):
        """stop replay"""
        self.is_playing = False
        if self.replay_thread and self.replay_thread.is_alive():
            self.replay_thread.join(timeout=5)
        self.get_logger().info("Stopped MCAP replay")
    
    def _replay_loop(self):
        """replay loop"""
        try:
            with open(self.mcap_file, 'rb') as f:
                mcap_reader = reader.make_reader(f)
                
                # get channels information
                channels = {}
                for channel_id, channel in mcap_reader.get_summary().channels.items():
                    channels[channel_id] = channel
                
                # Real-time message replay
                last_timestamp = None
                first_timestamp = None
                message_count = 0
                
                for message in mcap_reader.iter_messages(channels=list(channels.keys())):
                    if not self.is_playing or not rclpy.ok():
                        break
                    
                    # Get channel and schema information
                    channel = channels[message.channel_id]
                    schema = channel.schema
                    
                    # Handle timing - real-time playback
                    if first_timestamp is None:
                        first_timestamp = message.publish_time
                        last_timestamp = message.publish_time
                    
                    # Calculate delay for real-time playback
                    if last_timestamp is not None:
                        time_diff = (message.publish_time - last_timestamp) / 1e9  # Convert to seconds
                        if time_diff > 0:
                            time.sleep(time_diff)  # Real-time playback delay
                    
                    last_timestamp = message.publish_time
                    
                    try:
                        # get message type
                        msg_class = self.get_message_type(schema.name)
                        if not msg_class:
                            continue
                        
                        # Deserialize message
                        ros_msg = deserialize_message(message.data, msg_class)
                        
                        # Create publisher and publish
                        publisher = self.create_publisher_for_topic(channel.topic, msg_class)
                        publisher.publish(ros_msg)
                        
                        message_count += 1
                        if message_count % 100 == 0:
                            self.get_logger().info(f"Replayed {message_count} messages")
                        
                    except Exception as e:
                        self.get_logger().error(f"Error processing message: {e}")
                        continue
                
                self.get_logger().info(f"Replay completed. Total messages: {message_count}")
                
        except Exception as e:
            self.get_logger().error(f"Error in replay loop: {e}")
        finally:
            self.is_playing = False

def main():
    """main function"""
    import argparse
    
    parser = argparse.ArgumentParser(description='MCAP to ROS Topic Replayer')
    parser.add_argument('mcap_file', help='Path to MCAP file')
    parser.add_argument('--topic-prefix', default='/mcap_replay', 
                       help='Prefix for replayed topics (default: /mcap_replay)')
    parser.add_argument('--analyze-only', action='store_true',
                       help='Only analyze the MCAP file, do not replay')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        replayer = McapToRosTopicReplayer(args.mcap_file, args.topic_prefix)
        
        if args.analyze_only:
            # Only analyze file
            replayer.analyze_mcap_file()
        else:
            # start replay
            replayer.start_replay()
            
            # Keep nodes running
            rclpy.spin(replayer)
            
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'replayer' in locals():
            replayer.stop_replay()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

