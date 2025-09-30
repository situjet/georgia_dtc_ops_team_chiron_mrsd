#!/usr/bin/env python3
"""
中心像素GPS估计节点启动文件
============================

用于启动中心像素GPS估计节点的启动配置，假设目标始终位于图像中心。
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
    """生成中心像素GPS估计器的启动描述"""
    
    # 包目录
    pkg_dir = get_package_share_directory('vision_gps_estimator')
    
    # 启动参数
    declare_config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('vision_gps_estimator'),
            'config',
            'center_pixel_params.yaml'
        ]),
        description='配置YAML文件的路径'
    )
    
    declare_robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='dtc_mrsd_',
        description='MAVROS话题的机器人命名空间'
    )
    
    declare_camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera_optical_frame',
        description='相机光学框架ID'
    )
    
    declare_base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='机器人基础框架ID'
    )
    
    declare_record_mcap_arg = DeclareLaunchArgument(
        'record_mcap',
        default_value='false',
        description='启用相关话题的MCAP记录'
    )
    
    declare_mcap_output_dir_arg = DeclareLaunchArgument(
        'mcap_output_dir',
        default_value='/tmp/center_pixel_gps_logs',
        description='MCAP文件的输出目录'
    )
    
    declare_log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='日志级别 (DEBUG, INFO, WARN, ERROR)'
    )
    
    declare_enable_diagnostics_arg = DeclareLaunchArgument(
        'enable_diagnostics',
        default_value='true',
        description='启用诊断监控'
    )
    
    declare_estimation_frequency_arg = DeclareLaunchArgument(
        'estimation_frequency',
        default_value='5.0',
        description='GPS估计频率 (Hz)'
    )
    
    # 中心像素GPS估计节点
    center_pixel_gps_node = Node(
        package='vision_gps_estimator',
        executable='center_pixel_gps_node',
        name='center_pixel_gps_estimator',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'estimation.frequency': LaunchConfiguration('estimation_frequency'),
            }
        ],
        remappings=[
            # MAVROS话题重映射
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
    
    # 从base_link到camera_optical_frame的静态变换
    # 这应该根据实际的机器人设置进行校准
    static_tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_camera',
        arguments=[
            '0.1', '0.0', '0.05',  # x, y, z 平移（米）
            '0.0', '0.0', '0.0',   # roll, pitch, yaw 旋转（弧度）
            LaunchConfiguration('base_frame'),
            LaunchConfiguration('camera_frame')
        ],
        output='screen'
    )
    
    # 可选：map/world框架的静态变换
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
    
    # 诊断聚合器（可选）
    diagnostic_aggregator = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        name='diagnostic_aggregator',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_diagnostics')),
        parameters=[{
            'analyzers': {
                'center_pixel_gps': {
                    'type': 'diagnostic_aggregator/GenericAnalyzer',
                    'path': 'Center Pixel GPS',
                    'contains': ['center_pixel_gps_estimator']
                }
            }
        }]
    )
    
    # MCAP记录节点（可选）
    def create_mcap_recorder(context, *args, **kwargs):
        """创建带动态话题列表的MCAP记录器"""
        robot_ns = LaunchConfiguration('robot_namespace').perform(context)
        output_dir = LaunchConfiguration('mcap_output_dir').perform(context)
        
        # 记录的话题列表
        input_topics = [
            f'/{robot_ns}/mavros/global_position/global',
            f'/{robot_ns}/mavros/global_position/rel_alt', 
            f'/{robot_ns}/mavros/global_position/compass_hdg',
            '/gimbal_attitude',
            '/camera_mode',
            '/image_raw_compressed',
            '/tf',
            '/tf_static'
        ]
        
        output_topics = [
            '/center_gps/image_compressed',
            '/center_gps/camera_info',
            '/center_gps/target_gps',
            '/center_gps/target_gps_list',
            '/center_gps/target_local_enu',
            '/center_gps/center_marker'
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
                '--output', f'{output_dir}/center_pixel_gps_$(date +%Y%m%d_%H%M%S)',
                '--topics'
            ] + all_topics,
            respawn=False
        )]
    
    mcap_recorder = OpaqueFunction(function=create_mcap_recorder)
    
    # 启动描述
    return LaunchDescription([
        # 参数
        declare_config_file_arg,
        declare_robot_namespace_arg,
        declare_camera_frame_arg,
        declare_base_frame_arg,
        declare_record_mcap_arg,
        declare_mcap_output_dir_arg,
        declare_log_level_arg,
        declare_enable_diagnostics_arg,
        declare_estimation_frequency_arg,
        
        # 节点
        center_pixel_gps_node,
        static_tf_base_to_camera,
        static_tf_map_to_base,
        diagnostic_aggregator,
        mcap_recorder,
    ])

