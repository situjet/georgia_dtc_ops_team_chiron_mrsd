#!/usr/bin/env python3
"""
Launch file for complete behavior system including:
- behavior_governor (C++ core)
- foxglove_bridge (middleware)
- Optional: behavior extensions (Python)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Find package share directory
    pkg_behavior_governor = FindPackageShare('behavior_governor')
    
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
    
    enable_extensions_arg = DeclareLaunchArgument(
        'enable_extensions',
        default_value='false',
        description='Enable Python behavior extensions'
    )
    
    # Core behavior_governor node (C++)
    behavior_governor_node = Node(
        package='behavior_governor',
        executable='behavior_governor',
        name='behavior_governor',
        output='screen',
        parameters=[{
            # Add any C++ node parameters here if needed
        }],
        remappings=[
            # Standard MAVROS topics should already be correct
        ]
    )
    
    # Foxglove bridge node (Python middleware)
    foxglove_bridge_node = Node(
        package='behavior_governor',
        executable='foxglove_bridge.py',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'default_altitude': LaunchConfiguration('default_altitude'),
            'default_hold_time': LaunchConfiguration('default_hold_time'),
            'default_acceptance_radius': LaunchConfiguration('default_acceptance_radius'),
        }]
    )
    
    # Optional: Python behavior extensions
    # Uncomment if you have behavior_extensions.py
    # behavior_extensions_node = Node(
    #     package='behavior_governor',
    #     executable='behavior_extensions.py',
    #     name='behavior_extensions',
    #     output='screen',
    #     condition=IfCondition(LaunchConfiguration('enable_extensions'))
    # )
    
    return LaunchDescription([
        # Launch arguments
        default_altitude_arg,
        default_hold_time_arg,
        default_acceptance_radius_arg,
        enable_extensions_arg,
        
        # Nodes
        behavior_governor_node,
        foxglove_bridge_node,
        # behavior_extensions_node,  # Uncomment if using
    ])
