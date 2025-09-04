#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from humanflow_msgs.msg import HumanPose24, HumanPoseArray24
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from sensor_msgs.msg import Image
import torch
import torchvision
import os
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import cv2
import numpy as np  
class PoseEstimatorNode(Node):
    def __init__(self):
        super().__init__('pose_estimator')
        self.video_show = self.declare_parameter('video_show', True)
        # create subscriber to subscribe camera topic
        self.subscription = self.create_subscription(
            Image,
            '/drone1/camera',
            self.image_callback,
            10)
        
        self.pose_publisher = self.create_publisher(
            HumanPoseArray24,
            'human_poses_24',
            10)
            
        # initialize CvBridge
        self.bridge = CvBridge()
            
        # load model
        self.model_path = os.path.join(get_package_share_directory('pose_estimator'), 'models', 'nlf_l_multi.torchscript')
        self.get_logger().info('Loading model...')
        self.model = torch.jit.load(self.model_path).cuda().eval()
        self.get_logger().info('Model loaded successfully')

    def cv2_to_torch(self, cv_image):
        """Convert OpenCV image to torch tensor"""
        tensor = torch.from_numpy(cv_image).permute(2, 0, 1)
        return tensor.float()

    def image_callback(self, msg):
        try:
            # convert ros image to opencv format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            
            # convert to torch tensor
            image = self.cv2_to_torch(cv_image).cuda()
            frame_batch = image.unsqueeze(0)

            # detect with model
            with torch.inference_mode(), torch.device('cuda'):
                pred = self.model.detect_smpl_batched(frame_batch)

            if 'joints3d' in pred:
                joints3d_data = pred['joints3d']
                joints2d_data = pred['joints2d']
                print(joints2d_data[0].shape,joints3d_data[0].shape)
                
                if isinstance(joints3d_data, list):
                    if len(joints3d_data) > 0:
                        joints3d = joints3d_data[0].cpu().numpy()
                    else:
                        return
                else:
                    joints3d = joints3d_data[0].cpu().numpy()

                joints_tensor = joints3d_data[0]

                # create pose message
                pose_array = HumanPoseArray24()
                pose_array.header = Header()
                pose_array.header.stamp = self.get_clock().now().to_msg()
                pose_array.header.frame_id = "map"

                num_people = joints_tensor.shape[0]
                num_joints = joints_tensor.shape[1]
                
                for person_idx in range(num_people):
                    human_pose = HumanPose24()
                    human_pose.header = pose_array.header
                    
                    keypoints = []
                    chest_orientation = []
                    head_orientation = []
                    for joint_idx in range(num_joints):
                        point = Point()
                        coords = joints_tensor[person_idx, joint_idx].cpu()
                        point.x = float(coords[0])
                        point.y = float(coords[1])
                        point.z = float(coords[2])
                        keypoints.append(point)
                        if joint_idx in [9,16,17]:
                            chest_orientation.append(point)
                        if joint_idx in [12,15]:
                            head_orientation.append(point)
                    orientation_scale = 500.0

                    # Calculate head direction first
                    head_direction = np.array([
                        head_orientation[1].x - head_orientation[0].x,
                        head_orientation[1].y - head_orientation[0].y,
                        head_orientation[1].z - head_orientation[0].z
                    ])
                    head_direction = head_direction / np.linalg.norm(head_direction)

                    # Calculate chest normal vector
                    v1 = np.array([
                        chest_orientation[1].x - chest_orientation[0].x,
                        chest_orientation[1].y - chest_orientation[0].y,
                        chest_orientation[1].z - chest_orientation[0].z
                    ])

                    v2 = np.array([
                        chest_orientation[2].x - chest_orientation[0].x,
                        chest_orientation[2].y - chest_orientation[0].y,
                        chest_orientation[2].z - chest_orientation[0].z
                    ])

                    chest_normal_vector = np.cross(v1, v2)
                    chest_normal_vector = chest_normal_vector / np.linalg.norm(chest_normal_vector)

                    # Calculate angle between chest and head direction
                    dot_product = np.dot(chest_normal_vector, head_direction)
                    # If angle is greater than 90 degrees (dot product < 0), reverse the chest direction
                    if dot_product < 0:
                        chest_normal_vector = -chest_normal_vector

                    # Apply scale after direction correction
                    chest_normal_vector = chest_normal_vector * orientation_scale

                    # Calculate chest center and direction points
                    chest_center = Point(
                        x=(chest_orientation[0].x + chest_orientation[1].x + chest_orientation[2].x) / 3,
                        y=(chest_orientation[0].y + chest_orientation[1].y + chest_orientation[2].y) / 3,
                        z=(chest_orientation[0].z + chest_orientation[1].z + chest_orientation[2].z) / 3
                    )

                    chest_direction = Point(
                        x=chest_center.x + chest_normal_vector[0],
                        y=chest_center.y + chest_normal_vector[1],
                        z=chest_center.z + chest_normal_vector[2]
                    )

                    human_pose.chest_orientation = [chest_center, chest_direction]

                    # Scale head direction and set orientation
                    head_direction = head_direction * orientation_scale
                    human_pose.head_orientation = [
                        Point(
                            x=head_orientation[0].x,
                            y=head_orientation[0].y,
                            z=head_orientation[0].z
                        ),
                        Point(
                            x=head_orientation[0].x + head_direction[0],
                            y=head_orientation[0].y + head_direction[1],
                            z=head_orientation[0].z + head_direction[2]
                        )
                    ]

                    human_pose.keypoints = keypoints
                    human_pose.bounding_box_id = person_idx
                    pose_array.poses.append(human_pose)

                # publish pose
                self.pose_publisher.publish(pose_array)
                self.get_logger().info(f'Published poses for {num_people} people')
                # show image
                if self.video_show.value:
                    joints2d = pred['joints2d'][0]
                    display_image = draw_skeleton_2d(cv_image, joints2d,timestamp=None,
                                              detection_time=None)
                    cv2.imshow('image', display_image)
                    cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')



def create_connection_list(num_people):
    """Create a list of connections for multiple people, with 24 joints each"""
    base_connections = {
        'spine': [(0, 1), (1, 4), (4, 7), (7, 10)],
        'left_leg': [(0, 2), (2, 5), (5, 8), (8, 11)],
        'right_leg': [(0, 3), (3, 6), (6, 9), (9, 12)],
        'left_arm': [(9, 13), (13, 16), (16, 18), (18, 20)],
        'right_arm': [(9, 14), (14, 17), (17, 19), (19, 21)],
        'head': [(12, 15)]
    }
    
    connections_list = []
    for i in range(num_people):
        offset = i * 24  
        person_connections = {}
        for part, conns in base_connections.items():
            person_connections[part] = [(start + offset, end + offset) for start, end in conns]
        connections_list.append(person_connections)
    
    return connections_list

def draw_skeleton_2d(img, joints2d, timestamp=None, detection_time=None):
    """Draw 2D skeleton on image using OpenCV"""
    if torch.is_tensor(joints2d):
        joints2d = joints2d.detach().cpu().numpy()
    joints2d = joints2d.reshape(-1, 2)
    
    # Get image resolution
    height, width = img.shape[:2]
    resolution_text = f"Resolution: {width}x{height}"
    
    # Calculate number of people based on joints
    num_people = len(joints2d) // 24
    
    # Get connections for all people
    connections_list = create_connection_list(num_people)
    
    colors = {
        'spine': (255, 0, 0),      # Blue in BGR
        'left_leg': (0, 0, 255),   # Red in BGR
        'right_leg': (0, 255, 0),  # Green in BGR
        'left_arm': (0, 165, 255), # Orange in BGR
        'right_arm': (255, 0, 255),# Purple in BGR
        'head': (42, 42, 165)      # Brown in BGR
    }
    
    # Draw joints for all people
    for i, (x, y) in enumerate(joints2d):
        joint_point = (int(x), int(y))
        cv2.circle(img, joint_point, 3, (0, 0, 0), -1)  # Black dots
        # Add joint index 
        relative_index = i % 24  
        cv2.putText(img, str(relative_index), (int(x)+5, int(y)+5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    
    # Draw lines for each person
    for person_idx, connections in enumerate(connections_list):
        for part, conns in connections.items():
            for start, end in conns:
                if start < len(joints2d) and end < len(joints2d):
                    start_point = tuple(map(int, joints2d[start]))
                    end_point = tuple(map(int, joints2d[end]))
                    cv2.line(img, start_point, end_point, colors[part], 2)

    
    # Add timestamp and resolution information
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.6
    thickness = 2
    padding = 10
    
    # Draw black background for better text visibility
    def draw_text_with_background(text, position):
        (text_width, text_height), _ = cv2.getTextSize(text, font, font_scale, thickness)
        cv2.rectangle(img, 
                     (position[0] - padding, position[1] - text_height - padding),
                     (position[0] + text_width + padding, position[1] + padding),
                     (0, 0, 0), -1)
        cv2.putText(img, text, position, font, font_scale, (255, 255, 255), thickness)
    
    # Add timestamp at top-left
    if timestamp:
        timestamp_text = f"Time: {timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')[:-4]}"
        draw_text_with_background(timestamp_text, (20, 30))
    
    # Add resolution at top-right
    text_size = cv2.getTextSize(resolution_text, font, font_scale, thickness)[0]
    resolution_pos = (width - text_size[0] - 40, 30)
    draw_text_with_background(resolution_text, resolution_pos)
    
    # Add detection time if available
    if detection_time is not None:
        detection_text = f"Pose Estimation Time: {detection_time*1000:.1f}ms"
        draw_text_with_background(detection_text, (20, 70))
    
    return img

def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()