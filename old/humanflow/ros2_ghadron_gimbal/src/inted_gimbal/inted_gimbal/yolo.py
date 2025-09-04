#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
# Import custom pose message type
from humanflow_msgs.msg import HumanPose17, HumanPoseArray17

class Detection:
    model = None

    def load_model(self):
        print('Loading YOLOv11s-pose.engine...')
        # Change the path to the generated .engine file
        # model_path = "models/yolo11x_pose.pt" 
        model_path = "models/yolo11s-pose.engine" # Load the TensorRT engine

        if not os.path.exists(model_path):
            print(f"Model not found at {model_path}!")
            return False
        
        try:
            # Explicitly specify the task for the engine
            self.model = YOLO(model_path, task='pose') # Add task='pose' here
            # You might need to check if the first inference takes longer due to engine warmup
            print("TensorRT engine loaded successfully via Ultralytics (Task: Pose).") # Updated log
            return True
        except Exception as e:
            print(f"Error loading TensorRT engine via Ultralytics: {e}")

            return False # Return False if engine loading fails and no fallback

    def preprocess_image(self, image):
        """
        Reduce overexposure in images taken under bright sunlight.
        
        Args:
            image: Input image (numpy array)
            
        Returns:
            Preprocessed image with reduced overexposure
        """
        # Convert to LAB color space (L for lightness, A and B for color)
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        
        # Apply CLAHE (Contrast Limited Adaptive Histogram Equalization) to the L channel
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        l_clahe = clahe.apply(l)
        
        # Detect overexposed regions (high brightness)
        _, bright_mask = cv2.threshold(l, 180, 255, cv2.THRESH_BINARY)
        
        # Reduce brightness of overexposed regions
        if np.any(bright_mask):
            # Get mean of L channel in non-overexposed regions
            mean_l = np.mean(l[bright_mask == 0]) if np.any(bright_mask == 0) else 128
            # Adjust overexposed regions to be closer to the mean
            l_clahe[bright_mask == 255] = np.clip(l_clahe[bright_mask == 255] * 0.5, mean_l, 235)
        
        # Merge channels and convert back to BGR
        merged = cv2.merge([l_clahe, a, b])
        result = cv2.cvtColor(merged, cv2.COLOR_LAB2BGR)
        
        # Apply slight gamma correction to further reduce brightness if needed
        if np.mean(l) > 180:  # If image is generally very bright
            gamma = 1.5
            inv_gamma = 1.0 / gamma
            table = np.array([((i / 255.0) ** inv_gamma) * 255 
                              for i in np.arange(0, 256)]).astype("uint8")
            result = cv2.LUT(result, table)
            
        return result

    def yolo_detect(self, image):
        # Remove manual resizing:
        # cv_image = cv2.resize(image, (1280, 720)) if image.shape[0] != 720 or image.shape[1] != 1280 else image
        
        if not self.model:
            print("Model not loaded.")
            return None #Detection2DArray(), HumanPoseArray17()

        # We no longer do preprocessing here as it's expected to be done externally
        # processed_image = self.preprocess_image(image)
        
        inference_start_time = time.time()
        
        # Get image size from input image
        imgsz = max(image.shape[0], image.shape[1])
        # print(f"Input image size: {image.shape[0]}x{image.shape[1]}, using imgsz={imgsz}")
        
        # Run the model specifying input size from the input image
        # The library handles resizing internally
        # No need to specify half=True here as it's baked into the engine
        result = self.model(image, # Use the image directly (assumed to be preprocessed)
                          imgsz=640,#imgsz,       # Use image size from input image
                          conf=0.5,        
                          iou=0.45,        
                          max_det=10,      
                          classes=[0],     
                          verbose=False)   # Keep verbose=False for cleaner logs
        
        inference_end_time = time.time()
        inference_time = (inference_end_time - inference_start_time) * 1000
        # Indicate that TensorRT is being used (or should be)
        # print(f"Inference time (TensorRT engine, imgsz={imgsz:.0f}): {inference_time:.2f} ms")

        # Removed the redundant model.names printout, as it might not be available/relevant for engines

        # Detection array
        detection_array = Detection2DArray()
        

        if len(result) > 0 and result[0].boxes is not None: # Check if boxes exist
            boxes = result[0].boxes 
            
            # Filter based on the model's output directly
            # Since classes=[0] and conf=0.5 were likely used in export/engine,
            # the results should already reflect this.
            person_boxes = list(boxes) 

            # # Check if keypoints are available
            # keypoints_data = None
            # if hasattr(result[0], 'keypoints') and hasattr(result[0].keypoints, 'data') and result[0].keypoints.data is not None:
            #     keypoints_data = result[0].keypoints.data

            if person_boxes:
                # Sort by confidence (descending)
                person_boxes.sort(key=lambda box: float(box.conf[0].cpu().numpy()), reverse=True)
                # Limit to top 5 detections
                top_persons = person_boxes[:min(3, len(person_boxes))]
                
                for i, box in enumerate(top_persons):
                    # Coordinates are scaled to the original image size by Ultralytics
                    x1, y1, x2, y2 = map(float, box.xyxy[0].cpu().numpy()) 
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    confidence = float(box.conf[0].cpu().numpy())
                    # Ensure class is correct if needed, though filtered by classes=[0]
                    # class_id = int(box.cls[0].item()) 
                    
                    detection = Detection2D()
                    detection.bbox.center.position.x = center_x
                    detection.bbox.center.position.y = center_y
                    detection.bbox.size_x = float(x2 - x1)
                    detection.bbox.size_y = float(y2 - y1)
                    # Store confidence in theta field as before
                    detection.bbox.center.theta = confidence 
                    # Assuming class ID 0 for person
                    detection.id = "0" 
                    detection_array.detections.append(detection)
                    
                    # 在检测到目标后打印边界框中心坐标
                    # print(f"Bounding box center: x={center_x}, y={center_y} (width, height)")
                    
                    # # Process keypoints if available
                    # if keypoints_data is not None and i < len(keypoints_data):
                    #     person_keypoints = keypoints_data[i]
                    #     human_pose = HumanPose17()
                    #     human_pose.bounding_box_id = i
                        
                    #     # Extract the 17 keypoints
                    #     keypoints_list = []
                    #     for kp_idx in range(person_keypoints.shape[0]):
                    #         point = Point()
                    #         # Keypoints structure: [x, y, confidence]
                    #         kp = person_keypoints[kp_idx].cpu().numpy()
                    #         point.x = float(kp[0])
                    #         point.y = float(kp[1])
                    #         # Confidence value stored in z field
                    #         point.z = float(kp[2]) if len(kp) > 2 else 0.0
                    #         keypoints_list.append(point)
                            
                    #     human_pose.keypoints = keypoints_list
                    #     pose_array.poses.append(human_pose)
                        
                    #     print(f"Processed keypoints for person {i}")
                    
        # Handle cases where no detections are found or result structure is unexpected
        elif len(result) == 0:
             print("No results returned by the model.")
        else:
             print(f"Model returned result, but no boxes found. Result structure: {result[0]}")

        return detection_array
    

    # def Reid(self, detection_array: Detection2DArray):
    # subscribe to GPS topic
    #     pass

