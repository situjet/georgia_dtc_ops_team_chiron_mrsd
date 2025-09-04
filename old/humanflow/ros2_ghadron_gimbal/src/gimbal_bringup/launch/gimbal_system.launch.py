from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from datetime import datetime

def generate_launch_description():
    # get current date and time
    now = datetime.now()
    
    # create folder by day
    day_folder = now.strftime("%Y-%m-%d")
    
    # create detailed timestamp file name (year-month-day_hour-minute-second)
    timestamp = now.strftime("%Y%m%d_%H%M%S")
    
    # build full path: base path/date/year-month-day_hour-minute-second
    base_dir = "/home/dtc/humanflow/ros2_ghadron_gimbal/mcap_recording"
    day_dir = os.path.join(base_dir, day_folder)
    
    # ensure date folder exists
    os.makedirs(day_dir, exist_ok=True)
    
    # final recording path
    recording_path = os.path.join(day_dir, f"recording_{timestamp}")
    
    return LaunchDescription([
        LogInfo(msg=['Launching gimbal system...']),
        
        # gimbal control node
        Node(
            package='gimbal_angle_control',
            executable='gimbal_angle_control_node',
            name='gimbal_angle_control_node',
            output='screen'
        ),
        
        # gimbal status node
        Node(
            package='gimbal_status',
            executable='gimbal_status_node',
            name='gimbal_status_node',
            output='screen'
        ),
        
        # video stream publish node
        # Node(
        #     package='stream_publisher',
        #     executable='stream_node',
        #     name='stream_node',
        #     parameters=[{
        #         'rtsp_url': 'rtsp://10.3.1.124:8554/ghadron',
        #         'width': 640,
        #         'height': 360
        #     }],
        #     output='screen'
        # ),
        # Node(
        #     package='detect_and_track',
        #     executable='detect_track_node',
        #     name='detect_track_node',
        #     output='screen'
        # ),

        Node(package='inted_gimbal',
             executable='integrated_node',
             name='integrated_node',
             output='screen'),

        Node(package='payload',
             executable='payload_node',
             name='payload_node',
             output='screen'),
        
        # # image view node - not save separate image file
        # Node(
        #     package='image_viewer',
        #     executable='image_viewer_node',
        #     name='image_viewer_node',
        #     output='screen'
        # ),

        # # YOLO detection node
        # Node(
        #     package='yolo_detection',
        #     executable='yolo_detection_node',
        #     name='yolo_detection_node',
        #     output='screen'
        # ),
        
        # # human tracking node
        # Node(
        #     package='human_tracking',
        #     executable='tracking_node',
        #     name='tracking_node',
        #     output='screen'
        # ),
        
        # use bash -c command line format for recording
        # ExecuteProcess(
        #     cmd=['bash', '-c', 
        #          f"mkdir -p $(dirname {recording_path}) && " +
        #          f"chmod 777 $(dirname {recording_path}) && " +
        #          f"ros2 bag record " +
        #          f"-o {recording_path} " +
        #          f"--storage mcap " +
        #          f"--max-bag-duration 60 " +
        #          f"/image_raw " + 
        #          f"/robot_1/mavros/imu/data " +
        #          f"/gimbal_attitude " +
        #          f"/robot_1/mavros/global_position/global"],
        #     output='screen'
        # ),

        # LogInfo(msg=['All nodes started. Data is being recorded to: ' + recording_path])
    ]) 
