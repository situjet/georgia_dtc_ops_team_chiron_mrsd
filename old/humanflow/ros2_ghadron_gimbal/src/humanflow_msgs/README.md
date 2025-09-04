# Bounding box messages
# Human pose messages
## Overview
This package defines ROS message types for human pose estimation and tracking. The keypoint definitions are based on a standard human skeleton model, compatible with SMPL (Skinned Multi-Person Linear Model, ref: https://smpl.is.tue.mpg.de/).

## HumanPosseConstants.msg
Defines a comprehensive set of human body keypoints (24 joints), as well as the links between them. 


**Joint indexes**: 

{'pelvis': 0, 'spine1': 1, 'spine2': 2, 'spine3': 3, 'l_hip': 4, 'l_knee': 5, 'l_ankle': 6, 'l_foot': 7, 'r_hip': 8, 'r_knee': 9, 'r_ankle': 10, 'r_foot': 11, 'neck': 12, 'head': 13, 'l_shoulder': 14, 'l_elbow': 15, 'l_wrist': 16, 'r_shoulder': 17, 'r_elbow': 18, 'r_wrist': 19, 'l_hand': 20, 'r_hand': 21, 'l_fingers': 22, 'r_fingers': 23}

![Human Pose Keypoints](assets/joint_indexes.png)


**Joint links**:

Right leg: (0,1) -> (1,4) -> (4,7) -> (7,10)

Left leg:  (0,2) -> (2,5) -> (5,8) -> (8,11)

Spine:     (0,3) -> (3,6) -> (6,9) -> (9,12)

Left arm:  (9,13) -> (13,16) -> (16,18) -> (18,20) -> (20,22)

Right arm: (9,14) -> (14,17) -> (17,19) -> (19,21) -> (21,23)

Head:      (12,15)

![Human Pose Keypoints](assets/joint_links.png)







## HumanPose.msg
This message package defines the human pose data structure for each detected person in the scene. Each detection includes:
- A unique bounding box identifier
- A set of 24 3D joint positions representing the full human skeleton
- A standard ROS header (std_msgs/Header) containing timestamp and frame information

## HumanPoseArray.msg
This message package defines a collection of human pose detections within a single frame. It contains:
- A standard ROS header (timestamp and frame information)
- An array of HumanPose messages, where each element represents a complete pose detection for one person
