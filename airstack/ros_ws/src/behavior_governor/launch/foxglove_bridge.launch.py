#!/usr/bin/env python3
"""
Launch file for Foxglove to BehaviorGovernor bridge
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    default_altitude_arg = DeclareLaunchArgument(
        'default_altitude',
        default_value='10.0',
        description='Default altitude for waypoints in meters'
    )
    
    default_hold_time_arg = DeclareLaunchArgument(
        'default_hold_time',
        default_value='0.0',
        description='Default hold time at waypoints in seconds'
    )
    
    default_acceptance_radius_arg = DeclareLaunchArgument(
        'default_acceptance_radius',
        default_value='0.5',
        description='Default acceptance radius for waypoints in meters'
    )
    
    # Foxglove bridge node
    foxglove_bridge_node = Node(
        package='behavior_governor',
        executable='foxglove_bridge.py',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'default_altitude': LaunchConfiguration('default_altitude'),
            'default_hold_time': LaunchConfiguration('default_hold_time'),
            'default_acceptance_radius': LaunchConfiguration('default_acceptance_radius'),
        }],
        remappings=[
            # Add any topic remappings here if needed
        ]
    )
    
    return LaunchDescription([
        default_altitude_arg,
        default_hold_time_arg,
        default_acceptance_radius_arg,
        foxglove_bridge_node,
    ])
