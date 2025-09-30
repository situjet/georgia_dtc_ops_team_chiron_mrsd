#!/usr/bin/env python3
"""
Launch file for Vision GPS Estimator Node
=========================================

Enhanced launch configuration for AirStack integration with:
- Parameter loading from YAML
- Topic remapping support  
- Static TF publishing
- Optional MCAP recording
- Diagnostic monitoring
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the launch description for vision GPS estimator."""
    
    # Package directory
    pkg_dir = get_package_share_directory('vision_gps_estimator')
    
    # Launch arguments
    declare_config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('vision_gps_estimator'),
            'config',
            'params.yaml'
        ]),
        description='Path to the configuration YAML file'
    )
    
    declare_robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='dtc_mrsd',
        description='Robot namespace for MAVROS topics'
    )
    
    declare_camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera_optical_frame',
        description='Camera optical frame ID'
    )
    
    declare_base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Robot base frame ID'
    )
    
    declare_record_mcap_arg = DeclareLaunchArgument(
        'record_mcap',
        default_value='false',
        description='Enable MCAP recording of relevant topics'
    )
    
    declare_mcap_output_dir_arg = DeclareLaunchArgument(
        'mcap_output_dir',
        default_value='/tmp/vision_gps_logs',
        description='Output directory for MCAP files'
    )
    
    declare_log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='Logging level (DEBUG, INFO, WARN, ERROR)'
    )
    
    declare_enable_diagnostics_arg = DeclareLaunchArgument(
        'enable_diagnostics',
        default_value='true',
        description='Enable diagnostic monitoring'
    )
    
    # Vision GPS Estimator Node
    vision_gps_node = Node(
        package='vision_gps_estimator',
        executable='integrated_node',
        name='vision_gps_estimator',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            # MAVROS topic remappings
            ('/dtc_mrsd/mavros/global_position/global', 
             [LaunchConfiguration('robot_namespace'), '/mavros/global_position/global']),
            ('/dtc_mrsd/mavros/global_position/rel_alt',
             [LaunchConfiguration('robot_namespace'), '/mavros/global_position/rel_alt']),
            ('/dtc_mrsd/mavros/global_position/compass_hdg',
             [LaunchConfiguration('robot_namespace'), '/mavros/global_position/compass_hdg']),
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        respawn=True,
        respawn_delay=2
    )
    
    # Static transform from base_link to camera_optical_frame
    # This should be calibrated for the actual robot setup
    static_tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_camera',
        arguments=[
            '0.1', '0.0', '0.05',  # x, y, z translation (meters)
            '0.0', '0.0', '0.0',   # roll, pitch, yaw rotation (radians)
            LaunchConfiguration('base_frame'),
            LaunchConfiguration('camera_frame')
        ],
        output='screen'
    )
    
    # Optional: Static transform for map/world frame
    static_tf_map_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher', 
        name='static_tf_map_to_base',
        arguments=[
            '0.0', '0.0', '0.0',   # x, y, z
            '0.0', '0.0', '0.0',   # roll, pitch, yaw
            'map',
            LaunchConfiguration('base_frame')
        ],
        output='screen'
    )
    
    # Diagnostic aggregator (optional)
    diagnostic_aggregator = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        name='diagnostic_aggregator',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_diagnostics')),
        parameters=[{
            'analyzers': {
                'vision_gps': {
                    'type': 'diagnostic_aggregator/GenericAnalyzer',
                    'path': 'Vision GPS',
                    'contains': ['vision_gps_estimator']
                }
            }
        }]
    )
    
    # MCAP recording node (optional)
    def create_mcap_recorder(context, *args, **kwargs):
        """Create MCAP recorder with dynamic topic list."""
        robot_ns = LaunchConfiguration('robot_namespace').perform(context)
        output_dir = LaunchConfiguration('mcap_output_dir').perform(context)
        
        # Topic list for recording
        input_topics = [
            f'/{robot_ns}/mavros/global_position/global',
            f'/{robot_ns}/mavros/global_position/rel_alt', 
            f'/{robot_ns}/mavros/global_position/compass_hdg',
            '/gimbal_attitude',
            '/camera_mode',
            '/tf',
            '/tf_static'
        ]
        
        output_topics = [
            '/vision_gps/image_compressed',
            '/vision_gps/camera_info',
            '/vision_gps/detections',
            '/vision_gps/target_gps',
            '/vision_gps/target_gps_list'
        ]
        
        all_topics = input_topics + output_topics
        
        return [Node(
            package='rosbag2_py',
            executable='record',
            name='mcap_recorder',
            output='screen',
            condition=IfCondition(LaunchConfiguration('record_mcap')),
            arguments=[
                '--storage', 'mcap',
                '--output', f'{output_dir}/vision_gps_$(date +%Y%m%d_%H%M%S)',
                '--topics'
            ] + all_topics,
            respawn=False
        )]
    
    mcap_recorder = OpaqueFunction(function=create_mcap_recorder)
    
    # Launch description
    return LaunchDescription([
        # Arguments
        declare_config_file_arg,
        declare_robot_namespace_arg,
        declare_camera_frame_arg,
        declare_base_frame_arg,
        declare_record_mcap_arg,
        declare_mcap_output_dir_arg,
        declare_log_level_arg,
        declare_enable_diagnostics_arg,
        
        # Nodes
        vision_gps_node,
        static_tf_base_to_camera,
        static_tf_map_to_base,
        diagnostic_aggregator,
        mcap_recorder,
    ])
