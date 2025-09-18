from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtsp_streamer',
            executable='rtsp_streamer_node',
            output='screen',
            parameters=[{
                'rtsp_url': 'rtsp://10.3.1.124:8556/ghadron',
                'topic': '/image_raw_compressed',
                'fps': 5.0,
                'width': 640,
                'height': 360,
                'jpeg_quality': 50,
                'rtsp_latency_ms': 200,
                'prefer_tcp': True,
            }]
        )
    ])
