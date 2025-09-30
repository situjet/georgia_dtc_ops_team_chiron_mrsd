from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('rtsp_streamer')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'config.yaml'),
        description='Path to the configuration file'
    )
    
    # Check if external config file exists
    external_config = '/ros2_ws/config/config.yaml'
    config_file = external_config if os.path.exists(external_config) else LaunchConfiguration('config_file')
    
    # Create the node
    rtsp_streamer_node = Node(
        package='rtsp_streamer',
        executable='rtsp_streamer_node',
        name='rtsp_streamer_node',
        output='screen',
        parameters=[config_file],
        respawn=True,
        respawn_delay=5.0
    )
    
    return LaunchDescription([
        config_file_arg,
        rtsp_streamer_node
    ])