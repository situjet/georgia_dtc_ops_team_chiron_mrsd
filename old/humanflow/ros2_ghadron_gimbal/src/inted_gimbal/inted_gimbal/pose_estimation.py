#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from humanflow_msgs.msg import HumanPose24, HumanPoseArray24
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import os
from ament_index_python.packages import get_package_share_directory
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class PoseEstimation:
    model = None

    def load_model(self):
        print('Loading NLF model...')
        # model_path = "models/yolo11x-pose.pt"
        model_path = "models/nlf_l_multi_0.2.2.torchscript"
        if not os.path.exists(model_path):
            print("Model not found!")
            return False
        self.model = YOLO(model_path)
        return True

    def pose_detect(self, image):
        # Process at 1280x720 resolution
        cv_image = cv2.resize(image, (1280, 720)) if image.shape[0] != 720 or image.shape[1] != 1280 else image
        
        # Convert to torch tensor
        tensor = torch.from_numpy(cv_image).permute(2, 0, 1)
        frame_batch = tensor.float().cuda().unsqueeze(0)

        # Pose array
        pose_array = HumanPoseArray24()

        try:
            # Run the model
            with torch.inference_mode(), torch.device('cuda'):
                result = self.model.detect_smpl_batched(frame_batch)

            if 'joints3d' in result and len(result['joints3d']) > 0:
                joints3d = result['joints3d'][0]
                
                # Take top 5 persons (or fewer if less than 5 detected)
                num_people = min(5, joints3d.shape[0])
                
                # Process each person
                for person_idx in range(num_people):
                    human_pose = HumanPose24()
                    keypoints = []
                    chest_points = []
                    head_points = []

                    # Process each joint
                    for joint_idx in range(joints3d.shape[1]):
                        coords = joints3d[person_idx, joint_idx].cpu()
                        
                        point = Point()
                        point.x = float(coords[0])
                        point.y = float(coords[1])
                        point.z = float(coords[2])
                        keypoints.append(point)

                        # Collect orientation points
                        if joint_idx in [9, 16, 17]:
                            chest_points.append(point)
                        if joint_idx in [12, 15]:
                            head_points.append(point)

                    # Calculate orientations if we have enough points
                    if len(chest_points) == 3:
                        center, direction = self._calculate_chest_orientation(chest_points)
                        human_pose.chest_orientation = [center, direction]

                    if len(head_points) == 2:
                        human_pose.head_orientation = self._calculate_head_orientation(head_points)

                    human_pose.keypoints = keypoints
                    human_pose.bounding_box_id = person_idx
                    pose_array.poses.append(human_pose)

        except Exception as e:
            print(f"Error in pose detection: {str(e)}")
            
        return pose_array

    def _calculate_chest_orientation(self, points, scale=500.0):
        """Calculate chest orientation from three points"""
        v1 = np.array([
            points[1].x - points[0].x,
            points[1].y - points[0].y,
            points[1].z - points[0].z
        ])

        v2 = np.array([
            points[2].x - points[0].x,
            points[2].y - points[0].y,
            points[2].z - points[0].z
        ])

        normal = np.cross(v1, v2)
        normal = normal / np.linalg.norm(normal) * scale

        center = Point(
            x=sum(p.x for p in points) / 3,
            y=sum(p.y for p in points) / 3,
            z=sum(p.z for p in points) / 3
        )

        direction = Point(
            x=center.x + normal[0],
            y=center.y + normal[1],
            z=center.z + normal[2]
        )

        return center, direction

    def _calculate_head_orientation(self, points, scale=500.0):
        """Calculate head orientation from two points"""
        direction = np.array([
            points[1].x - points[0].x,
            points[1].y - points[0].y,
            points[1].z - points[0].z
        ])
        direction = direction / np.linalg.norm(direction) * scale

        return [
            Point(x=points[0].x, y=points[0].y, z=points[0].z),
            Point(
                x=points[0].x + direction[0],
                y=points[0].y + direction[1],
                z=points[0].z + direction[2]
            )
        ] 