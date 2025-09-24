from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtsp_forwarder',
            executable='rtsp_forwarder_node',
            name='rtsp_forwarder',
            output='screen',
            parameters=[
                {
                    'listen_host': '0.0.0.0',
                    'listen_port': 8556,
                    'target_host': '10.3.1.124',
                    'target_port': 8556,
                }
            ],
            respawn=True,
            respawn_delay=2.0,
        )
    ])
