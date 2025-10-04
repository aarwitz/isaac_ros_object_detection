#!/usr/bin/env python3
# filepath: /home/taylor/workspaces/isaac_ros-dev/src/isaac_ros_object_detection/isaac_ros_yolov8/scripts/yolov8_3d_detector.py

"""
Enhanced 3D Object Detection Script for Isaac ROS YOLOv8 + RealSense
This script subscribes to YOLOv8 detections and depth data to compute 3D object positions
and generates cropped point clouds for each detected object.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
import message_filters
from message_filters import ApproximateTimeSynchronizer
import struct
import json

class ObjectDetection3D(Node):
    def __init__(self):
        super().__init__('object_detection_3d')
        
        self.bridge = CvBridge()
        
        # Publishers for 3D coordinates and point clouds
        self.point_pub = self.create_publisher(
            PointStamped,
            'detected_objects_3d',
            10
        )
        
        # Publisher for cropped object point clouds
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            'detected_objects_pointcloud',
            10
        )
        
        # Publisher for 3D detection metadata (JSON)
        self.metadata_pub = self.create_publisher(
            Image,  # We'll encode JSON as a string in image message for web streaming
            'detection_metadata',
            10
        )
        
        # Message filter subscribers for synchronized data
        self.detection_sub = message_filters.Subscriber(
            self,
            Detection2DArray,
            '/detections_output'
        )
        
        self.depth_sub = message_filters.Subscriber(
            self,
            Image,
            '/aligned_depth_to_color/image_raw'
        )
        
        self.color_sub = message_filters.Subscriber(
            self,
            Image,
            '/image_rect'  # RGB image for point cloud coloring
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
            [self.detection_sub, self.depth_sub, self.color_sub, self.depth_info_sub, self.yolo_info_sub],
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
        
        self.get_logger().info('🚀 Enhanced 3D Object Detection Node Started!')
        self.get_logger().info('📡 Waiting for synchronized detection, depth, and color data...')
    
    def transform_coordinates(self, yolo_x, yolo_y, yolo_width, yolo_height, color_width, color_height):
        """Transform from YOLOv8 coordinates to color image coordinates"""
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
    
    def create_object_pointcloud(self, depth_image, color_image, bbox_x, bbox_y, bbox_w, bbox_h, fx, fy, cx, cy, margin=10):
        """Create a cropped point cloud for the detected object"""
        # Add margin around bounding box
        x1 = max(0, int(bbox_x - bbox_w/2) - margin)
        y1 = max(0, int(bbox_y - bbox_h/2) - margin)
        x2 = min(depth_image.shape[1], int(bbox_x + bbox_w/2) + margin)
        y2 = min(depth_image.shape[0], int(bbox_y + bbox_h/2) + margin)
        
        # Crop regions
        depth_crop = depth_image[y1:y2, x1:x2]
        color_crop = color_image[y1:y2, x1:x2]
        
        points = []
        colors = []
        
        for v in range(depth_crop.shape[0]):
            for u in range(depth_crop.shape[1]):
                depth_mm = depth_crop[v, u]
                if depth_mm > 100 and depth_mm < 5000:  # Valid depth range
                    depth_m = depth_mm / 1000.0
                    
                    # Convert to world coordinates
                    pixel_x = x1 + u
                    pixel_y = y1 + v
                    world_x, world_y, world_z = self.pixel_to_world(pixel_x, pixel_y, depth_m, fx, fy, cx, cy)
                    
                    points.append([world_x, world_y, world_z])
                    
                    # Get RGB color (note: OpenCV uses BGR)
                    b, g, r = color_crop[v, u]
                    colors.append([r, g, b])
        
        return np.array(points), np.array(colors)
    
    def create_pointcloud2_msg(self, points, colors, header):
        """Create a PointCloud2 message with XYZ and RGB data"""
        if len(points) == 0:
            return None
            
        # Define point cloud fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        
        # Pack point data
        point_data = []
        for i in range(len(points)):
            x, y, z = points[i]
            r, g, b = colors[i].astype(np.uint8)
            
            # Pack RGB into a single uint32
            rgb = (r << 16) | (g << 8) | b
            
            point_data.append(struct.pack('fffI', x, y, z, rgb))
        
        # Create PointCloud2 message
        pc2_msg = PointCloud2()
        pc2_msg.header = header
        pc2_msg.height = 1
        pc2_msg.width = len(points)
        pc2_msg.fields = fields
        pc2_msg.is_bigendian = False
        pc2_msg.point_step = 16  # 4 bytes * 4 fields
        pc2_msg.row_step = pc2_msg.point_step * len(points)
        pc2_msg.data = b''.join(point_data)
        pc2_msg.is_dense = True
        
        return pc2_msg
    
    def process_detections(self, detection_msg, depth_msg, color_msg, depth_info_msg, yolo_info_msg):
        """Process synchronized detection, depth, and color data"""
        try:
            # Convert images
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
            color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            
            # Camera parameters
            fx = depth_info_msg.k[0]
            fy = depth_info_msg.k[4] 
            cx = depth_info_msg.k[2]
            cy = depth_info_msg.k[5]
            
            color_width = depth_info_msg.width
            color_height = depth_info_msg.height
            yolo_width = yolo_info_msg.width
            yolo_height = yolo_info_msg.height
            
            if not detection_msg.detections:
                return
            
            detection_data = []
            
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
                
                # Transform coordinates
                depth_x, depth_y = self.transform_coordinates(
                    bbox_x, bbox_y, yolo_width, yolo_height, color_width, color_height
                )
                
                # Transform bounding box size
                depth_bbox_w = bbox_w * (color_width / yolo_width)
                depth_bbox_h = bbox_h * (color_height / yolo_height)
                
                # Check bounds
                if 0 <= depth_x < color_width and 0 <= depth_y < color_height:
                    # Get depth value
                    depth_mm = depth_image[depth_y, depth_x]
                    depth_m = depth_mm / 1000.0
                    
                    if depth_m > 0.1 and depth_m < 10.0:
                        # Convert to 3D coordinates
                        world_x, world_y, world_z = self.pixel_to_world(
                            depth_x, depth_y, depth_m, fx, fy, cx, cy
                        )
                        
                        # Create point cloud for this object
                        points, colors = self.create_object_pointcloud(
                            depth_image, color_image, depth_x, depth_y, 
                            depth_bbox_w, depth_bbox_h, fx, fy, cx, cy
                        )
                        
                        if len(points) > 0:
                            # Create and publish point cloud
                            pc_msg = self.create_pointcloud2_msg(points, colors, depth_msg.header)
                            if pc_msg:
                                self.pointcloud_pub.publish(pc_msg)
                        
                        # Publish 3D point
                        point_msg = PointStamped()
                        point_msg.header = depth_msg.header
                        point_msg.point.x = world_x
                        point_msg.point.y = world_y  
                        point_msg.point.z = world_z
                        self.point_pub.publish(point_msg)
                        
                        # Store detection data for JSON
                        detection_data.append({
                            'class_name': class_name,
                            'confidence': float(confidence),
                            'bbox_2d': [float(bbox_x), float(bbox_y), float(bbox_w), float(bbox_h)],
                            'position_3d': [float(world_x), float(world_y), float(world_z)],
                            'depth_mm': int(depth_mm),
                            'num_points': len(points)
                        })
                        
                        self.get_logger().info(
                            f'🎯 {class_name} (conf: {confidence:.2f}) '
                            f'3D: ({world_x:.3f}, {world_y:.3f}, {world_z:.3f})m '
                            f'Points: {len(points)}'
                        )
            
            # Publish detection metadata as JSON (encoded in image message for web streaming)
            if detection_data:
                json_str = json.dumps(detection_data)
                # Create a simple image message to carry JSON data
                metadata_msg = Image()
                metadata_msg.header = depth_msg.header
                metadata_msg.height = 1
                metadata_msg.width = len(json_str)
                metadata_msg.encoding = 'mono8'
                metadata_msg.step = len(json_str)
                metadata_msg.data = json_str.encode('utf-8')
                self.metadata_pub.publish(metadata_msg)
                    
        except Exception as e:
            self.get_logger().error(f'❌ Error processing detections: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    node = ObjectDetection3D()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutting down Enhanced 3D Object Detection Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()