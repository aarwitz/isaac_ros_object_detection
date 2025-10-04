#!/usr/bin/env python3

"""
Complete 3D Object Detection Script for Isaac ROS YOLOv8 + RealSense
This script subscribes to YOLOv8 detections and depth data to compute 3D object positions.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped
import message_filters
from message_filters import ApproximateTimeSynchronizer

class ObjectDetection3D(Node):
    def __init__(self):
        super().__init__('object_detection_3d')
        
        self.bridge = CvBridge()
        
        # Publishers for 3D coordinates
        self.point_pub = self.create_publisher(
            PointStamped,
            'detected_objects_3d',
            10
        )
        
        # Message filter subscribers for synchronized data
        self.detection_sub = message_filters.Subscriber(
            self,
            Detection2DArray,
            '/detections_output'
        )
        
        # Use ALIGNED depth for easier coordinate transformation
        self.depth_sub = message_filters.Subscriber(
            self,
            Image,
            '/aligned_depth_to_color/image_raw'
        )
        
        self.depth_info_sub = message_filters.Subscriber(
            self,
            CameraInfo,
            '/aligned_depth_to_color/camera_info'
        )
        
        self.yolo_info_sub = message_filters.Subscriber(
            self,
            CameraInfo,
            '/yolov8_encoder/resize/camera_info'
        )
        
        # Synchronizer to align all data streams
        self.sync = ApproximateTimeSynchronizer(
            [self.detection_sub, self.depth_sub, self.depth_info_sub, self.yolo_info_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.process_detections)
        
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
        
        self.get_logger().info('🚀 3D Object Detection Node Started!')
        self.get_logger().info('📡 Waiting for synchronized detection and aligned depth data...')
    
    def transform_coordinates(self, yolo_x, yolo_y, yolo_width, yolo_height, color_width, color_height):
        """Transform from YOLOv8 coordinates to color image coordinates (now same as aligned depth)"""
        # Scale from YOLOv8 (640x640) to color camera resolution (640x480)
        scale_x = color_width / yolo_width
        scale_y = color_height / yolo_height
        
        color_x = yolo_x * scale_x
        color_y = yolo_y * scale_y
        
        return int(color_x), int(color_y)
    
    def pixel_to_world(self, pixel_x, pixel_y, depth_m, fx, fy, cx, cy):
        """Convert pixel coordinates + depth to 3D world coordinates"""
        world_x = (pixel_x - cx) * depth_m / fx
        world_y = (pixel_y - cy) * depth_m / fy
        world_z = depth_m
        return world_x, world_y, world_z
    
    def process_detections(self, detection_msg, depth_msg, depth_info_msg, yolo_info_msg):
        """Process synchronized detection and aligned depth data"""
        try:
            # Convert depth image (aligned depth is in same coordinate frame as color)
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
            
            # Camera parameters (aligned depth uses color camera intrinsics)
            fx = depth_info_msg.k[0]
            fy = depth_info_msg.k[4] 
            cx = depth_info_msg.k[2]
            cy = depth_info_msg.k[5]
            
            color_width = depth_info_msg.width  # Should be 640
            color_height = depth_info_msg.height  # Should be 480
            yolo_width = yolo_info_msg.width      # Should be 640
            yolo_height = yolo_info_msg.height    # Should be 640
            
            if not detection_msg.detections:
                return
                
            self.get_logger().info(f'📊 Processing {len(detection_msg.detections)} detections')
            self.get_logger().info(f'📐 Image sizes: YOLOv8={yolo_width}x{yolo_height}, Aligned Depth={color_width}x{color_height}')
            
            for i, detection in enumerate(detection_msg.detections):
                # Get detection info
                bbox_x = detection.bbox.center.position.x
                bbox_y = detection.bbox.center.position.y
                bbox_w = detection.bbox.size_x
                bbox_h = detection.bbox.size_y
                
                if detection.results:
                    class_id = int(detection.results[0].hypothesis.class_id)
                    confidence = detection.results[0].hypothesis.score
                    class_name = self.coco_classes[class_id] if class_id < len(self.coco_classes) else f"class_{class_id}"
                else:
                    class_name = "unknown"
                    confidence = 0.0
                
                # Transform coordinates (much simpler with aligned depth)
                depth_x, depth_y = self.transform_coordinates(
                    bbox_x, bbox_y, yolo_width, yolo_height, color_width, color_height
                )
                
                # Check bounds
                if 0 <= depth_x < color_width and 0 <= depth_y < color_height:
                    # Get depth value (convert from mm to meters)
                    depth_mm = depth_image[depth_y, depth_x]
                    depth_m = depth_mm / 1000.0
                    
                    if depth_m > 0.1 and depth_m < 10.0:  # Reasonable depth range
                        # Convert to 3D coordinates
                        world_x, world_y, world_z = self.pixel_to_world(
                            depth_x, depth_y, depth_m, fx, fy, cx, cy
                        )
                        
                        # Publish 3D point
                        point_msg = PointStamped()
                        point_msg.header = depth_msg.header
                        point_msg.point.x = world_x
                        point_msg.point.y = world_y  
                        point_msg.point.z = world_z
                        
                        self.point_pub.publish(point_msg)
                        
                        # Log result
                        self.get_logger().info(
                            f'🎯 {class_name} (conf: {confidence:.2f}) '
                            f'YOLO: ({bbox_x:.0f},{bbox_y:.0f}) -> '
                            f'Aligned Depth: ({depth_x},{depth_y}) -> '
                            f'3D: ({world_x:.3f}, {world_y:.3f}, {world_z:.3f})m'
                        )
                        
                        # Also log bounding box info for context
                        self.get_logger().info(
                            f'   📦 BBox: {bbox_w:.0f}x{bbox_h:.0f}px, Depth: {depth_mm}mm'
                        )
                        
                    else:
                        self.get_logger().warn(
                            f'⚠️  {class_name}: Invalid depth {depth_mm}mm at ({depth_x},{depth_y})'
                        )
                else:
                    self.get_logger().warn(
                        f'⚠️  {class_name}: Coordinates ({depth_x},{depth_y}) outside image bounds'
                    )
                    
        except Exception as e:
            self.get_logger().error(f'❌ Error processing detections: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    node = ObjectDetection3D()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutting down 3D Object Detection Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()