import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from humanflow_msgs.msg import HumanPoseArray24  
from geometry_msgs.msg import Point

class PoseVisualizer(Node):
    def __init__(self):
        super().__init__('pose_visualizer')
        self.marker_pub = self.create_publisher(MarkerArray, 'human_pose_markers', 10)
        self.pose_sub = self.create_subscription(HumanPoseArray24, 'human_poses_24', self.pose_callback, 10)
        self.last_marker_count = 0 

    def pose_callback(self, msg):
        marker_array = MarkerArray()
 
        if self.last_marker_count > 0:
            delete_marker = Marker()
            delete_marker.header.frame_id = "map"
            delete_marker.action = Marker.DELETEALL
            marker_array.markers.append(delete_marker)

        joint_names = [
            'pelvis', 'spine1', 'spine2', 'spine3',  # 0-3
            'left_hip', 'left_knee', 'left_ankle', 'left_foot',  # 4-7
            'right_hip', 'right_knee', 'right_ankle', 'right_foot',  # 8-11
            'neck', 'head',  # 12-13
            'left_shoulder', 'left_elbow', 'left_wrist',  # 14-16
            'right_shoulder', 'right_elbow', 'right_wrist',  # 17-19
            'left_hand', 'right_hand'  # 20-21
        ]

        connections = {
            'right_leg': [(0, 1), (1, 4), (4, 7), (7, 10)],
            'left_leg': [(0, 2), (2, 5), (5, 8), (8, 11)],
            'spine': [(0, 3), (3, 6), (6, 9), (9, 12)],
            'left_arm': [(9, 13), (13, 16), (16, 18), (18, 20), (20, 22)],
            'right_arm': [(9, 14), (14, 17), (17, 19), (19, 21), (21, 23)],
            'head': [(12, 15)]
        }

        joint_colors = {
            'spine': (1.0, 0.0, 0.0),
            'left_leg': (0.0, 1.0, 0.0),
            'right_leg': (0.0, 1.0, 0.0),
            'left_arm': (1.0, 1.0, 0.0),
            'right_arm': (1.0, 1.0, 0.0),
            'head': (0.0, 1.0, 1.0)
        }

        for person_idx, human_pose in enumerate(msg.poses):
            for joint_idx, joint in enumerate(human_pose.keypoints):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.ns = f"person_{person_idx}_joints"
                marker.id = person_idx * 100 + joint_idx
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = joint.x / 10000
                marker.pose.position.y = joint.y / 10000
                marker.pose.position.z = joint.z / 10000
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.01
                marker.scale.y = 0.01
                marker.scale.z = 0.01

                for connection_name, connection_pairs in connections.items():
                    if any(joint_idx in pair for pair in connection_pairs):
                        color = joint_colors[connection_name]
                        marker.color.r = color[0]
                        marker.color.g = color[1]
                        marker.color.b = color[2]
                        marker.color.a = 1.0
                        break

                marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()
                marker_array.markers.append(marker)

            for connection_name, connection_pairs in connections.items():
                line_marker = Marker()
                line_marker.header = msg.header
                line_marker.header.frame_id = "map"
                line_marker.ns = f"person_{person_idx}_{connection_name}"
                line_marker.id = person_idx * 1000 + len(marker_array.markers)
                line_marker.type = Marker.LINE_STRIP
                line_marker.action = Marker.ADD
                line_marker.scale.x = 0.01
                line_marker.color.r = 0.8
                line_marker.color.g = 0.8
                line_marker.color.b = 0.8
                line_marker.color.a = 1.0
                
         
                line_marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()
                
                for start, end in connection_pairs:
                    if start < len(human_pose.keypoints) and end < len(human_pose.keypoints):
                        start_joint = human_pose.keypoints[start]
                        end_joint = human_pose.keypoints[end]
                        
                        start_point = Point(x=start_joint.x / 10000, y=start_joint.y / 10000, z=start_joint.z / 10000)
                        end_point = Point(x=end_joint.x / 10000, y=end_joint.y / 10000, z=end_joint.z / 10000)
                        
                        line_marker.points.append(start_point)
                        line_marker.points.append(end_point)
                marker_array.markers.append(line_marker)
                
            # Draw chest orientation arrow
            chest_arrow = Marker()
            chest_arrow.header.frame_id = "map"
            chest_arrow.ns = f"person_{person_idx}_chest_direction"
            chest_arrow.id = person_idx * 2000
            chest_arrow.type = Marker.ARROW
            chest_arrow.action = Marker.ADD

            # Modify arrow dimensions
            chest_arrow.scale.x = 0.005  # shaft diameter
            chest_arrow.scale.y = 0.01   # head diameter
            chest_arrow.scale.z = 0.1   # head length

            # Set arrow color
            chest_arrow.color.r = 1.0
            chest_arrow.color.g = 0.0
            chest_arrow.color.b = 0.0
            chest_arrow.color.a = 1.0

            # Set arrow points (start and end)
            start_point = Point()
            start_point.x = human_pose.chest_orientation[0].x / 10000
            start_point.y = human_pose.chest_orientation[0].y / 10000
            start_point.z = human_pose.chest_orientation[0].z / 10000

            end_point = Point()
            end_point.x = human_pose.chest_orientation[1].x / 10000
            end_point.y = human_pose.chest_orientation[1].y / 10000
            end_point.z = human_pose.chest_orientation[1].z / 10000

            chest_arrow.points = [start_point, end_point]
            chest_arrow.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()

            marker_array.markers.append(chest_arrow)

            # Draw head orientation arrow
            head_arrow = Marker()
            head_arrow.header.frame_id = "map"
            head_arrow.ns = f"person_{person_idx}_head_direction"
            head_arrow.id = person_idx * 2001
            head_arrow.type = Marker.ARROW
            head_arrow.action = Marker.ADD
            head_arrow.scale.x = 0.005  # shaft diameter
            head_arrow.scale.y = 0.01   # head diameter
            head_arrow.scale.z = 0.1   # head length

            # Set head arrow color (blue)
            head_arrow.color.r = 0.0
            head_arrow.color.g = 0.0
            head_arrow.color.b = 1.0
            head_arrow.color.a = 1.0

            # Set arrow points
            start_point = Point()
            start_point.x = human_pose.head_orientation[0].x / 10000
            start_point.y = human_pose.head_orientation[0].y / 10000
            start_point.z = human_pose.head_orientation[0].z / 10000

            end_point = Point()
            end_point.x = human_pose.head_orientation[1].x / 10000
            end_point.y = human_pose.head_orientation[1].y / 10000
            end_point.z = human_pose.head_orientation[1].z / 10000

            head_arrow.points = [start_point, end_point]
            head_arrow.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()

            marker_array.markers.append(head_arrow)

        self.last_marker_count = len(marker_array.markers)
        self.marker_pub.publish(marker_array)
        self.get_logger().info("Published HumanPose markers")

def main(args=None):
    rclpy.init(args=args)
    visualizer = PoseVisualizer()
    rclpy.spin(visualizer)
    visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()