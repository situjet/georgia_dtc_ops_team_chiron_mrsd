from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
import os

def generate_launch_description():
    """Launch MCAP to UDP bridge system"""
    
    # Declare launch arguments
    mcap_file_arg = DeclareLaunchArgument(
        'mcap_file',
        default_value='',
        description='Path to MCAP file to replay'
    )
    
    udp_ip_arg = DeclareLaunchArgument(
        'udp_ip',
        default_value='10.3.1.106',
        description='UDP target IP address'
    )
    
    udp_port_arg = DeclareLaunchArgument(
        'udp_port',
        default_value='5005',
        description='UDP target port'
    )
    
    playback_rate_arg = DeclareLaunchArgument(
        'playback_rate',
        default_value='1.0',
        description='MCAP playback rate (1.0 = realtime)'
    )
    
    # MCAP replayer node
    mcap_replayer_node = Node(
        package='mcap_receiver',
        executable='mcap_to_rostopic_replayer',
        name='mcap_replayer',
        parameters=[{
            'mcap_file': LaunchConfiguration('mcap_file'),
            'playback_rate': LaunchConfiguration('playback_rate'),
            'topic_prefix': '/mcap_replay'
        }],
        output='screen'
    )
    
    # UDP publisher node (from dell_publisher)
    udp_publisher_node = Node(
        package='dell_publisher',
        executable='rostopic_publisher',
        name='udp_publisher',
        parameters=[{
            'topic_name': '/mcap_replay/sensor_data',
            'topic_type': 'std_msgs/String',
            'udp_ip': LaunchConfiguration('udp_ip'),
            'udp_port': LaunchConfiguration('udp_port')
        }],
        output='screen'
    )
    
    # Group all nodes
    mcap_bridge_group = GroupAction([
        PushRosNamespace('mcap_bridge'),
        mcap_replayer_node,
        udp_publisher_node,
    ])
    
    return LaunchDescription([
        mcap_file_arg,
        udp_ip_arg,
        udp_port_arg,
        playback_rate_arg,
        mcap_bridge_group
    ])
