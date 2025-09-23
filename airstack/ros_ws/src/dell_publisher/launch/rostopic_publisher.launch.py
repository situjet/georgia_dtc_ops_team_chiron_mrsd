from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='drone/data',
        description='Name of the ROS topic to subscribe to'
    )
    
    topic_type_arg = DeclareLaunchArgument(
        'topic_type',
        default_value='dtc_network_msgs/msg/HumanDataMsg',
        description='Type of the ROS message'
    )
    
    udp_ip_arg = DeclareLaunchArgument(
        'udp_ip',
        default_value='10.3.1.106',
        description='UDP destination IP address'
    )
    
    udp_port_arg = DeclareLaunchArgument(
        'udp_port',
        default_value='5005',
        description='UDP destination port'
    )

    # Create the node
    rostopic_publisher_node = Node(
        package='dell_publisher',
        executable='rostopic_publisher',
        name='message_publisher',
        parameters=[{
            'topic_name': LaunchConfiguration('topic_name'),
            'topic_type': LaunchConfiguration('topic_type'),
            'udp_ip': LaunchConfiguration('udp_ip'),
            'udp_port': LaunchConfiguration('udp_port'),
        }],
        output='screen'
    )

    return LaunchDescription([
        topic_name_arg,
        topic_type_arg,
        udp_ip_arg,
        udp_port_arg,
        rostopic_publisher_node
    ])
