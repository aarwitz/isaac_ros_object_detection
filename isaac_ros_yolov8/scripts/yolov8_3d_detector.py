#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header

import message_filters
from message_filters import ApproximateTimeSynchronizer, Subscriber


class YOLOv8_3D_Detector(Node):
    def __init__(self):
        super().__init__('yolov8_3d_detector')
        
        self.bridge = CvBridge()
        
        # Publishers
        self.point_pub = self.create_publisher(
            PointStamped, 
            '/detected_objects_3d', 
            10
        )
        
        # Depth filtering parameters
        self.min_depth_mm = 200    # 20cm minimum
        self.max_depth_mm = 5000   # 5m maximum (filter out 65535mm errors)
        
        # Subscribers using message_filters for synchronization
        self.detection_sub = message_filters.Subscriber(
            self, 
            Detection2DArray, 
            '/detections_output'
        )
        
        self.depth_sub = message_filters.Subscriber(
            self, 
            Image, 
            '/depth/image_rect_raw'
        )
        
        self.depth_info_sub = message_filters.Subscriber(
            self, 
            CameraInfo, 
            '/depth/camera_info'
        )
        
        self.yolov8_info_sub = message_filters.Subscriber(
            self, 
            CameraInfo, 
            '/yolov8_encoder/resize/camera_info'
        )
        
        # Time synchronizer
        self.sync = ApproximateTimeSynchronizer(
            [self.detection_sub, self.depth_sub, self.depth_info_sub, self.yolov8_info_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.synchronized_callback)
        
        self.get_logger().info('YOLOv8 3D Detector node started')
        self.get_logger().info(f'Depth filtering: {self.min_depth_mm}mm - {self.max_depth_mm}mm')
        
        # COCO class names
        self.coco_classes = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat',
            'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat',
            'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack',
            'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
            'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
            'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
            'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake',
            'chair', 'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop',
            'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
            'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier',
            'toothbrush'
        ]

    def get_robust_depth(self, depth_image, center_x, center_y, search_radius=15):
        """
        Get a robust depth value by sampling around the center point.
        Filters out invalid depth readings.
        """
        height, width = depth_image.shape
        
        # Define search area
        min_x = max(0, center_x - search_radius)
        max_x = min(width, center_x + search_radius + 1)
        min_y = max(0, center_y - search_radius)
        max_y = min(height, center_y + search_radius + 1)
        
        # Extract depth values in the search area
        depth_patch = depth_image[min_y:max_y, min_x:max_x]
        
        # Filter valid depth values (non-zero and within reasonable range)
        valid_mask = (depth_patch > 0) & (depth_patch >= self.min_depth_mm) & (depth_patch <= self.max_depth_mm)
        valid_depths = depth_patch[valid_mask]
        
        if len(valid_depths) >= 3:  # Need at least 3 valid points for robust estimate
            # Use median for robustness
            median_depth = np.median(valid_depths)
            std_depth = np.std(valid_depths)
            num_valid = len(valid_depths)
            total_pixels = depth_patch.size
            
            # Additional filtering: reject if standard deviation is too high (inconsistent depth)
            if std_depth < 100:  # Less than 10cm variation
                return median_depth, num_valid, total_pixels, std_depth
            else:
                # Try with stricter filtering - only keep values close to median
                close_to_median = valid_depths[np.abs(valid_depths - median_depth) < 50]  # 5cm tolerance
                if len(close_to_median) >= 2:
                    return np.median(close_to_median), len(close_to_median), total_pixels, np.std(close_to_median)
        
        return 0, len(valid_depths) if len(valid_depths) > 0 else 0, depth_patch.size, 0

    def sample_bounding_box_depth(self, depth_image, bbox_center_x, bbox_center_y, bbox_width, bbox_height):
        """
        Sample depth from multiple points within the bounding box
        """
        samples = []
        
        # Sample from a grid within the bounding box
        sample_points = [
            (bbox_center_x, bbox_center_y),  # Center
            (bbox_center_x - bbox_width//4, bbox_center_y - bbox_height//4),  # Top-left
            (bbox_center_x + bbox_width//4, bbox_center_y - bbox_height//4),  # Top-right
            (bbox_center_x - bbox_width//4, bbox_center_y + bbox_height//4),  # Bottom-left
            (bbox_center_x + bbox_width//4, bbox_center_y + bbox_height//4),  # Bottom-right
            (bbox_center_x, bbox_center_y - bbox_height//3),  # Top-center
            (bbox_center_x, bbox_center_y + bbox_height//3),  # Bottom-center
            (bbox_center_x - bbox_width//3, bbox_center_y),  # Left-center
            (bbox_center_x + bbox_width//3, bbox_center_y),  # Right-center
        ]
        
        height, width = depth_image.shape
        
        for x, y in sample_points:
            x, y = int(x), int(y)
            if 0 <= x < width and 0 <= y < height:
                depth_mm, num_valid, total_pixels, std_dev = self.get_robust_depth(depth_image, x, y, search_radius=8)
                if depth_mm > 0:
                    samples.append(depth_mm)
        
        if samples:
            # Return median of all valid samples
            return np.median(samples), len(samples)
        
        return 0, 0

    def synchronized_callback(self, detection_msg, depth_msg, depth_info_msg, yolov8_info_msg):
        """Callback when detection, depth image, and camera infos are synchronized"""
        try:
            # Convert depth image to OpenCV format
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
            
            # Extract depth camera intrinsics
            fx_depth = depth_info_msg.k[0]
            fy_depth = depth_info_msg.k[4]
            cx_depth = depth_info_msg.k[2]
            cy_depth = depth_info_msg.k[5]
            
            # Get image dimensions
            depth_height = depth_info_msg.height
            depth_width = depth_info_msg.width
            yolov8_height = yolov8_info_msg.height
            yolov8_width = yolov8_info_msg.width
            
            num_detections = len(detection_msg.detections)
            if num_detections > 0:
                self.get_logger().info(f"📊 Processing {num_detections} detections")
            
            # Process each detection
            for i, detection in enumerate(detection_msg.detections):
                # Get bounding box info
                bbox_center_x_yolo = detection.bbox.center.position.x
                bbox_center_y_yolo = detection.bbox.center.position.y
                bbox_width_yolo = detection.bbox.size_x
                bbox_height_yolo = detection.bbox.size_y
                
                # Get class info
                if detection.results:
                    class_id = int(detection.results[0].hypothesis.class_id)
                    confidence = detection.results[0].hypothesis.score
                    class_name = self.coco_classes[class_id] if class_id < len(self.coco_classes) else f"class_{class_id}"
                else:
                    class_id = -1
                    confidence = 0.0
                    class_name = "unknown"
                
                # Transform coordinates from YOLOv8 space (640x640) to depth space (848x480)
                scale_x = 640.0 / yolov8_width
                scale_y = 480.0 / yolov8_height
                
                color_x = bbox_center_x_yolo * scale_x
                color_y = bbox_center_y_yolo * scale_y
                color_width = bbox_width_yolo * scale_x
                color_height = bbox_height_yolo * scale_y
                
                # Transform to depth image coordinates
                depth_x = color_x * (depth_width / 640.0)
                depth_y = color_y
                depth_bbox_width = color_width * (depth_width / 640.0)
                depth_bbox_height = color_height
                
                pixel_x = int(depth_x)
                pixel_y = int(depth_y)
                
                # Check bounds
                if (0 <= pixel_x < depth_width and 0 <= pixel_y < depth_height):
                    
                    # Try multiple depth sampling strategies
                    depth_mm = 0
                    method_used = ""
                    
                    # Strategy 1: Sample from bounding box area
                    depth_mm, num_samples = self.sample_bounding_box_depth(
                        depth_image, pixel_x, pixel_y, int(depth_bbox_width), int(depth_bbox_height)
                    )
                    
                    if depth_mm > 0:
                        method_used = f"bbox_sampling_{num_samples}_points"
                    else:
                        # Strategy 2: Try center point with larger radius
                        depth_mm, num_valid, total_pixels, std_dev = self.get_robust_depth(
                            depth_image, pixel_x, pixel_y, search_radius=20
                        )
                        if depth_mm > 0:
                            method_used = f"center_sampling_{num_valid}/{total_pixels}_pixels"
                    
                    if depth_mm > 0:  # Valid depth found
                        depth_m = depth_mm / 1000.0
                        
                        # Convert to 3D coordinates
                        world_x = (pixel_x - cx_depth) * depth_m / fx_depth
                        world_y = (pixel_y - cy_depth) * depth_m / fy_depth
                        world_z = depth_m
                        
                        # Create and publish 3D point
                        point_msg = PointStamped()
                        point_msg.header = depth_msg.header
                        point_msg.point.x = world_x
                        point_msg.point.y = world_y
                        point_msg.point.z = world_z
                        
                        self.point_pub.publish(point_msg)
                        
                        self.get_logger().info(
                            f"🎯 {class_name} (conf: {confidence:.3f}) "
                            f"3D: ({world_x:.3f}, {world_y:.3f}, {world_z:.3f})m "
                            f"[{method_used}] depth: {depth_mm:.0f}mm"
                        )
                        
                    else:
                        self.get_logger().warn(
                            f"❌ {class_name}: No valid depth found at ({pixel_x}, {pixel_y}) "
                            f"in {int(depth_bbox_width)}x{int(depth_bbox_height)}px area"
                        )
                        
        except Exception as e:
            self.get_logger().error(f'❌ Error: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    detector = YOLOv8_3D_Detector()
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()