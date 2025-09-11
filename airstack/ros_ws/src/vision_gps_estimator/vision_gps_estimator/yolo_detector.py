#!/usr/bin/env python3
"""
Enhanced YOLO Detector Module for AirStack Integration
=====================================================

Refactored from original humanflow yolo.py with:
- Pure structure output (no ROS message construction)
- Parameterized inference settings
- Enhanced error handling and logging
- Performance monitoring
"""

import os
import time
import logging
import numpy as np
import cv2
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None
    logging.warning("Ultralytics not available, YOLO detection will be disabled")

logger = logging.getLogger(__name__)


@dataclass
class DetectionResult:
    """Structure for detection results."""
    boxes: np.ndarray      # [N, 4] - (x1, y1, x2, y2)
    scores: np.ndarray     # [N] - confidence scores
    classes: np.ndarray    # [N] - class IDs
    centers: np.ndarray    # [N, 2] - (center_x, center_y)
    inference_time_ms: float
    image_shape: Tuple[int, int]  # (height, width)


class YOLODetector:
    """Enhanced YOLO detector with parameterized settings and pure output."""
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize YOLO detector with configuration.
        
        Args:
            config: Configuration dictionary with detector parameters
        """
        # Configuration from parameters
        self.model_path = config.get('model_path', 'models/yolo11s-pose.engine')
        self.conf_threshold = config.get('conf_threshold', 0.5)
        self.iou_threshold = config.get('iou_threshold', 0.45)
        self.max_detections = config.get('max_detections', 10)
        self.input_size = config.get('input_size', 640)
        self.target_classes = config.get('target_classes', [0])  # Person class
        self.enable_preprocessing = config.get('enable_preprocessing', False)
        
        # Model and state
        self.model = None
        self.is_loaded = False
        
        # Performance tracking
        self.stats = {
            'total_inferences': 0,
            'total_detections': 0,
            'average_inference_time': 0.0,
            'last_inference_time': 0.0,
            'model_warmup_complete': False
        }
        
        logger.info(f"YOLODetector initialized with model: {self.model_path}")
        logger.info(f"Detection parameters - conf: {self.conf_threshold}, "
                   f"iou: {self.iou_threshold}, max_det: {self.max_detections}")

    def load_model(self) -> bool:
        """
        Load YOLO model with error handling.
        
        Returns:
            True if model loaded successfully, False otherwise
        """
        if YOLO is None:
            logger.error("Ultralytics YOLO not available")
            return False
            
        if not os.path.exists(self.model_path):
            logger.error(f"Model not found at {self.model_path}")
            return False
        
        try:
            logger.info(f"Loading YOLO model: {self.model_path}")
            
            # Determine task based on model path
            task = 'pose' if 'pose' in self.model_path.lower() else 'detect'
            
            # Load model
            self.model = YOLO(self.model_path, task=task)
            
            # Warm up model with dummy inference
            self._warmup_model()
            
            self.is_loaded = True
            logger.info(f"YOLO model loaded successfully (task: {task})")
            return True
            
        except Exception as e:
            logger.error(f"Error loading YOLO model: {e}")
            self.model = None
            self.is_loaded = False
            return False

    def _warmup_model(self):
        """Warm up model with dummy inference to avoid first-inference delay."""
        try:
            dummy_image = np.zeros((640, 640, 3), dtype=np.uint8)
            logger.info("Warming up YOLO model...")
            
            # Run dummy inference
            _ = self.model(
                dummy_image,
                imgsz=self.input_size,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                max_det=1,
                classes=self.target_classes,
                verbose=False
            )
            
            self.stats['model_warmup_complete'] = True
            logger.info("YOLO model warmup complete")
            
        except Exception as e:
            logger.warning(f"Model warmup failed: {e}")

    def preprocess_image(self, image: np.ndarray) -> np.ndarray:
        """
        Preprocess image to reduce overexposure (optional enhancement).
        
        Args:
            image: Input image (BGR format)
            
        Returns:
            Preprocessed image
        """
        if not self.enable_preprocessing:
            return image
            
        try:
            # Convert to LAB color space
            lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            
            # Apply CLAHE to L channel
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            l_clahe = clahe.apply(l)
            
            # Handle overexposed regions
            _, bright_mask = cv2.threshold(l, 180, 255, cv2.THRESH_BINARY)
            
            if np.any(bright_mask):
                mean_l = np.mean(l[bright_mask == 0]) if np.any(bright_mask == 0) else 128
                l_clahe[bright_mask == 255] = np.clip(
                    l_clahe[bright_mask == 255] * 0.5, mean_l, 235
                )
            
            # Merge channels and convert back
            merged = cv2.merge([l_clahe, a, b])
            result = cv2.cvtColor(merged, cv2.COLOR_LAB2BGR)
            
            # Apply gamma correction for very bright images
            if np.mean(l) > 180:
                gamma = 1.5
                inv_gamma = 1.0 / gamma
                table = np.array([
                    ((i / 255.0) ** inv_gamma) * 255 
                    for i in np.arange(0, 256)
                ]).astype("uint8")
                result = cv2.LUT(result, table)
            
            return result
            
        except Exception as e:
            logger.warning(f"Image preprocessing failed: {e}")
            return image

    def infer(self, image: np.ndarray) -> Optional[DetectionResult]:
        """
        Run YOLO inference and return pure detection results.
        
        Args:
            image: Input image (BGR format)
            
        Returns:
            DetectionResult object or None if inference failed
        """
        if not self.is_loaded or self.model is None:
            logger.error("Model not loaded, cannot perform inference")
            return None
        
        try:
            # Preprocess image if enabled
            processed_image = self.preprocess_image(image)
            
            # Record inference start time
            inference_start = time.time()
            
            # Run inference
            results = self.model(
                processed_image,
                imgsz=self.input_size,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                max_det=self.max_detections,
                classes=self.target_classes,
                verbose=False
            )
            
            # Calculate inference time
            inference_time = (time.time() - inference_start) * 1000
            
            # Process results
            detection_result = self._process_results(
                results, image.shape, inference_time
            )
            
            # Update statistics
            self._update_stats(inference_time, detection_result)
            
            return detection_result
            
        except Exception as e:
            logger.error(f"YOLO inference failed: {e}")
            return None

    def _process_results(self, results, image_shape: Tuple[int, int, int], 
                        inference_time: float) -> DetectionResult:
        """
        Process YOLO results into standardized format.
        
        Args:
            results: YOLO inference results
            image_shape: Original image shape (H, W, C)
            inference_time: Inference time in milliseconds
            
        Returns:
            DetectionResult object
        """
        height, width = image_shape[:2]
        
        # Initialize empty arrays
        boxes = np.array([]).reshape(0, 4)
        scores = np.array([])
        classes = np.array([])
        centers = np.array([]).reshape(0, 2)
        
        # Process results if available
        if len(results) > 0 and results[0].boxes is not None:
            result = results[0]
            
            # Extract detection data
            if len(result.boxes) > 0:
                # Get boxes in xyxy format
                boxes_tensor = result.boxes.xyxy.cpu().numpy()
                scores_tensor = result.boxes.conf.cpu().numpy()
                classes_tensor = result.boxes.cls.cpu().numpy().astype(int)
                
                # Filter valid detections
                valid_indices = scores_tensor >= self.conf_threshold
                
                if np.any(valid_indices):
                    boxes = boxes_tensor[valid_indices]
                    scores = scores_tensor[valid_indices]
                    classes = classes_tensor[valid_indices]
                    
                    # Calculate centers
                    centers = np.column_stack([
                        (boxes[:, 0] + boxes[:, 2]) / 2,  # center_x
                        (boxes[:, 1] + boxes[:, 3]) / 2   # center_y
                    ])
                    
                    # Sort by confidence (highest first)
                    sort_indices = np.argsort(scores)[::-1]
                    boxes = boxes[sort_indices]
                    scores = scores[sort_indices]
                    classes = classes[sort_indices]
                    centers = centers[sort_indices]
        
        return DetectionResult(
            boxes=boxes,
            scores=scores,
            classes=classes,
            centers=centers,
            inference_time_ms=inference_time,
            image_shape=(height, width)
        )

    def _update_stats(self, inference_time: float, result: DetectionResult):
        """Update performance statistics."""
        self.stats['total_inferences'] += 1
        self.stats['total_detections'] += len(result.boxes)
        self.stats['last_inference_time'] = inference_time
        
        # Update running average
        n = self.stats['total_inferences']
        old_avg = self.stats['average_inference_time']
        self.stats['average_inference_time'] = (
            (old_avg * (n - 1) + inference_time) / n
        )

    def get_stats(self) -> Dict[str, Any]:
        """
        Get detector statistics for monitoring.
        
        Returns:
            Dictionary with detector statistics
        """
        return self.stats.copy()

    def update_parameters(self, **kwargs):
        """
        Update detector parameters at runtime.
        
        Args:
            **kwargs: Parameter updates
        """
        updated = []
        
        if 'conf_threshold' in kwargs:
            self.conf_threshold = kwargs['conf_threshold']
            updated.append(f"conf_threshold={self.conf_threshold}")
            
        if 'iou_threshold' in kwargs:
            self.iou_threshold = kwargs['iou_threshold']
            updated.append(f"iou_threshold={self.iou_threshold}")
            
        if 'max_detections' in kwargs:
            self.max_detections = kwargs['max_detections']
            updated.append(f"max_detections={self.max_detections}")
        
        if updated:
            logger.info(f"Updated detector parameters: {', '.join(updated)}")

    def shutdown(self):
        """Clean shutdown of detector resources."""
        logger.info("Shutting down YOLO detector...")
        
        if self.model is not None:
            # Clear model from memory
            del self.model
            self.model = None
        
        self.is_loaded = False
        logger.info("YOLO detector shutdown complete")


# Backwards compatibility alias
Detection = YOLODetector
