from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gimbal_status',
            executable='gimbal_status_node',
            name='gimbal_status',
            output='screen'
        ),
        Node(
            package='adi_recorder',
            executable='rtsp_node',
            name='recorder',
            output='screen'
        )
    ])