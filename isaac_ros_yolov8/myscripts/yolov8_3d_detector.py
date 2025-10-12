#!/usr/bin/env python3
# filepath: /home/taylor/workspaces/isaac_ros-dev/src/isaac_ros_object_detection/isaac_ros_yolov8/scripts/yolov8_3d_grasp_detector.py

"""
3D Object Detection with Point Cloud Generation
Combines YOLOv8 object detection with depth data for 3D pose estimation
"""

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
import message_filters
from message_filters import ApproximateTimeSynchronizer
import struct
import json
from std_msgs.msg import String, Header
import math
import random

# No additional imports needed for basic 3D pose estimation

def normalize_bbox_from_detection(bbox_x, bbox_y, bbox_w, bbox_h, yolo_w, yolo_h):
    """Convert normalized/pixel bbox coords to pixel coordinates."""
    if bbox_x <= 1.0 and bbox_y <= 1.0 and bbox_w <= 1.0 and bbox_h <= 1.0:
        cx = float(bbox_x) * yolo_w
        cy = float(bbox_y) * yolo_h
        w = float(bbox_w) * yolo_w
        h = float(bbox_h) * yolo_h
    else:
        cx = float(bbox_x)
        cy = float(bbox_y)
        w = float(bbox_w)
        h = float(bbox_h)
    return cx, cy, w, h

def get_median_depth_m(depth_img, cx, cy, half_win=3, max_half_win=15):
    """Robust depth estimation using median filtering."""
    h, w = depth_img.shape
    cx = int(round(cx))
    cy = int(round(cy))

    if cx < 0 or cx >= w or cy < 0 or cy >= h:
        return None

    is_uint16 = np.issubdtype(depth_img.dtype, np.integer)

    for hw in range(half_win, max_half_win + 1):
        x1 = max(0, cx - hw)
        y1 = max(0, cy - hw)
        x2 = min(w, cx + hw + 1)
        y2 = min(h, cy + hw + 1)

        patch = depth_img[y1:y2, x1:x2].ravel()
        if patch.size == 0:
            continue

        if is_uint16:
            nonzero = patch[patch > 0]
            if nonzero.size > 0:
                return float(np.median(nonzero)) / 1000.0
        else:
            patchf = patch.astype(np.float32)
            valid = patchf[np.isfinite(patchf) & (patchf > 0.0)]
            if valid.size > 0:
                return float(np.median(valid))
    return None

class ObjectDetection3D(Node):
    def __init__(self):
        super().__init__('object_detection_3d')
        
        self.bridge = CvBridge()
        
        # Publishers
        self.point_pub = self.create_publisher(PointStamped, 'detected_objects_3d', 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, 'detected_objects_pointcloud', 10)
        self.full_scene_pub = self.create_publisher(PointCloud2, 'full_scene_pointcloud', 10)
        self.metadata_pub = self.create_publisher(String, 'detection_metadata', 10)
        self.debug_image_pub = self.create_publisher(Image, 'yolov8_debug', 10)
        
        # Message filter subscribers
        self.detection_sub = message_filters.Subscriber(self, Detection2DArray, '/detections_output')
        self.depth_sub = message_filters.Subscriber(self, Image, '/aligned_depth_to_color/image_raw')
        self.color_sub = message_filters.Subscriber(self, Image, '/image_rect')
        self.depth_info_sub = message_filters.Subscriber(self, CameraInfo, '/aligned_depth_to_color/camera_info')
        self.yolo_info_sub = message_filters.Subscriber(self, CameraInfo, '/yolov8_encoder/resize/camera_info')
        
        # Synchronizer
        self.sync = ApproximateTimeSynchronizer(
            [self.detection_sub, self.depth_sub, self.color_sub, self.depth_info_sub, self.yolo_info_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.process_detections)
        
        # COCO class names
        self.coco_classes = [
            'sock','person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat',
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
        self.get_logger().info('📡 Waiting for synchronized detection, depth, and color data...')

    def pixel_to_world(self, pixel_x, pixel_y, depth_m, fx, fy, cx, cy):
        """Convert pixel coordinates + depth to 3D world coordinates"""
        world_x = (pixel_x - cx) * depth_m / fx
        world_y = (pixel_y - cy) * depth_m / fy
        world_z = depth_m
        return world_x, world_y, world_z
    
    def create_object_pointcloud(self, depth_image, color_image,
                             bbox_cx, bbox_cy, bbox_w, bbox_h,
                             fx, fy, cx, cy, margin=15):
        # 1) Rect crop
        x1 = max(0, int(bbox_cx - bbox_w/2) - margin)
        y1 = max(0, int(bbox_cy - bbox_h/2) - margin)
        x2 = min(depth_image.shape[1], int(bbox_cx + bbox_w/2) + margin)
        y2 = min(depth_image.shape[0], int(bbox_cy + bbox_h/2) + margin)

        depth_crop = depth_image[y1:y2, x1:x2].astype(np.float32)  # mm
        color_crop = color_image[y1:y2, x1:x2]

        if depth_crop.size == 0:
            return np.empty((0,3)), np.empty((0,3)), depth_crop, x1, y1

        # 2) Robust object depth (use central 40% window to avoid edges)
        H, W = depth_crop.shape
        cx0 = int(0.3 * W); cx1 = int(0.7 * W)
        cy0 = int(0.3 * H); cy1 = int(0.7 * H)
        center_patch = depth_crop[cy0:cy1, cx0:cx1]

        valid = center_patch[(center_patch > 100.0) & (center_patch < 5000.0)]
        if valid.size == 0:
            return np.empty((0,3)), np.empty((0,3)), depth_crop, x1, y1

        z_med_mm = float(np.median(valid))
        # MAD-based scale (robust). Fallback bandwidth if too thin.
        mad = np.median(np.abs(valid - z_med_mm)) if valid.size > 5 else 0.0
        band_mm = max(30.0, 2.5 * 1.4826 * mad)  # >=3 cm, expand with MAD

        # 3) Depth inlier mask
        z = depth_crop
        mask = (z > 100.0) & (z < 5000.0) & (np.abs(z - z_med_mm) <= band_mm)

        # Optional light morphology to fill small holes
        mask = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_CLOSE,
                                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))).astype(bool)

        # 4) Generate XYZ + RGB only for inliers
        v_idx, u_idx = np.nonzero(mask)
        if v_idx.size == 0:
            return np.empty((0,3)), np.empty((0,3)), depth_crop, x1, y1

        depth_m = z[v_idx, u_idx] / 1000.0
        pixel_x = x1 + u_idx
        pixel_y = y1 + v_idx

        X = (pixel_x - cx) * depth_m / fx
        Y = (pixel_y - cy) * depth_m / fy
        Z = depth_m

        points = np.stack([X, Y, Z], axis=1)

        rgb = color_crop[v_idx, u_idx]  # BGR
        colors = np.stack([rgb[:,2], rgb[:,1], rgb[:,0]], axis=1)  # to RGB

        return points, colors, depth_crop, x1, y1

    def create_full_scene_pointcloud(self, depth_image, color_image, fx, fy, cx, cy, 
                                   downsample_factor=6, max_points=25000):
        """Create a downsampled point cloud of the entire scene with proper filtering"""
        try:
            h, w = depth_image.shape
            
            # Create coordinate meshes for downsampling with some randomization to avoid grid artifacts
            y_coords = np.arange(0, h, downsample_factor)
            x_coords = np.arange(0, w, downsample_factor)
            
            # Add small random offsets to break up the grid pattern
            y_offset = np.random.randint(-downsample_factor//3, downsample_factor//3 + 1, size=len(y_coords))
            x_offset = np.random.randint(-downsample_factor//3, downsample_factor//3 + 1, size=len(x_coords))
            
            y_coords = np.clip(y_coords + y_offset, 0, h-1)
            x_coords = np.clip(x_coords + x_offset, 0, w-1)
            
            xx, yy = np.meshgrid(x_coords, y_coords)
            
            # Flatten coordinates for easier processing
            pixel_x = xx.flatten()
            pixel_y = yy.flatten()
            
            # Get depth values at these coordinates
            depth_values = depth_image[pixel_y, pixel_x].astype(np.float32)
            
            # Use the same depth filtering as the object point cloud
            # More permissive range: 10cm to 8m, exclude exactly zero values
            valid_mask = (depth_values > 100.0) & (depth_values < 8000.0)
            
            if not np.any(valid_mask):
                self.get_logger().warn("No valid depth values found in full scene")
                return np.empty((0,3)), np.empty((0,3))
            
            # Filter coordinates and depth values
            valid_x = pixel_x[valid_mask]
            valid_y = pixel_y[valid_mask]
            valid_depth_mm = depth_values[valid_mask]
            valid_depth_m = valid_depth_mm / 1000.0
            
            # Get corresponding color values
            valid_colors = color_image[valid_y, valid_x]
            
            # Convert to 3D coordinates using the same math as object point clouds
            X = (valid_x - cx) * valid_depth_m / fx
            Y = (valid_y - cy) * valid_depth_m / fy
            Z = valid_depth_m
            
            points = np.stack([X, Y, Z], axis=1)
            
            # Convert BGR to RGB and ensure proper range [0-255]
            colors = np.stack([valid_colors[:,2], valid_colors[:,1], valid_colors[:,0]], axis=1)
            colors = np.clip(colors, 0, 255).astype(np.uint8)
            
            # Debug: Check for problematic color values
            black_points = np.sum(np.all(colors == 0, axis=1))
            if black_points > 0:
                self.get_logger().info(f"🎨 Full scene has {black_points}/{len(colors)} black points (may appear green)")
            
            # Filter out completely black points (they're usually invalid depth pixels)
            non_black_mask = ~np.all(colors == 0, axis=1)
            if np.sum(non_black_mask) < len(colors):
                self.get_logger().info(f"🧹 Filtering out {len(colors) - np.sum(non_black_mask)} black points")
                points = points[non_black_mask]
                colors = colors[non_black_mask]
            
            # Subsample if we have too many points
            if len(points) > max_points:
                idx = np.random.choice(len(points), max_points, replace=False)
                points = points[idx]
                colors = colors[idx]
            
            # Debug: Check for problematic values
            x_range = f"X[{X.min():.2f}, {X.max():.2f}]"
            y_range = f"Y[{Y.min():.2f}, {Y.max():.2f}]"
            z_range = f"Z[{Z.min():.2f}, {Z.max():.2f}]"
            
            self.get_logger().info(f"🌍 Generated full scene: {len(points)} points, "
                                 f"depth range: {valid_depth_m.min():.2f}-{valid_depth_m.max():.2f}m, "
                                 f"coords: {x_range} {y_range} {z_range}")
            
            return points, colors
            
        except Exception as e:
            self.get_logger().error(f"Error creating full scene point cloud: {e}")
            import traceback
            traceback.print_exc()
            return np.empty((0,3)), np.empty((0,3))

    def create_pointcloud2_msg(self, points, colors, header):
        if len(points) == 0:
            return None

        # Ensure shapes and types
        pts = points.astype(np.float32)
        cols = colors.astype(np.uint8)  # RGB in 0..255

        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='r', offset=12, datatype=PointField.UINT8,   count=1),
            PointField(name='g', offset=13, datatype=PointField.UINT8,   count=1),
            PointField(name='b', offset=14, datatype=PointField.UINT8,   count=1),
        ]

        # stride = 3*4 (xyz) + 3*1 (rgb) + 1 padding = 16 bytes
        point_step = 16
        row = bytearray()
        pad = b'\x00'  # 1-byte pad to keep 16B stride

        for (x, y, z), (r, g, b) in zip(pts, cols):
            row += struct.pack('<fff', x, y, z) + bytes((int(r), int(g), int(b))) + pad

        pc2_msg = PointCloud2()
        pc2_msg.header = header
        pc2_msg.height = 1
        pc2_msg.width = len(pts)
        pc2_msg.fields = fields
        pc2_msg.is_bigendian = False
        pc2_msg.point_step = point_step
        pc2_msg.row_step = point_step * len(pts)
        pc2_msg.data = bytes(row)
        pc2_msg.is_dense = True
        return pc2_msg


    def process_detections(self, detection_msg, depth_msg, color_msg, depth_info_msg, yolo_info_msg):
        """Process synchronized detection, depth, and color data for 3D pose estimation"""
        try:
            # Convert images
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
            color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            
            if (color_image.shape[1], color_image.shape[0]) != (depth_image.shape[1], depth_image.shape[0]):
                color_image = cv2.resize(color_image, (depth_image.shape[1], depth_image.shape[0]), interpolation=cv2.INTER_LINEAR)

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
            
            # Process each detection
            for i, detection in enumerate(detection_msg.detections):
                if detection.results:
                    class_id = int(detection.results[0].hypothesis.class_id)
                    confidence = detection.results[0].hypothesis.score
                    class_name = self.coco_classes[class_id] if class_id < len(self.coco_classes) else f"class_{class_id}"
                    # Confidence filtering can be added here if needed
                else:
                    class_name = "unknown"
                    confidence = 0.0
                    continue

                bbox_x = detection.bbox.center.position.x
                bbox_y = detection.bbox.center.position.y
                bbox_w = detection.bbox.size_x
                bbox_h = detection.bbox.size_y

                # Convert coordinates
                cx_pixel, cy_pixel, w_pixel, h_pixel = normalize_bbox_from_detection(
                    bbox_x, bbox_y, bbox_w, bbox_h, yolo_width, yolo_height
                )

                # Scale to depth/color resolution
                scale_x = color_width / yolo_width
                scale_y = color_height / yolo_height
                
                center_x = int(cx_pixel * scale_x)
                center_y = int(cy_pixel * scale_y)
                bbox_w_scaled = int(w_pixel * scale_x)
                bbox_h_scaled = int(h_pixel * scale_y)
                
                # Check bounds
                if not (0 <= center_x < color_width and 0 <= center_y < color_height):
                    continue

                # Get robust depth
                depth_m = get_median_depth_m(depth_image, center_x, center_y, half_win=3, max_half_win=15)
                if depth_m is None or not (0.05 < depth_m < 10.0):
                    continue

                # Convert to 3D
                world_x, world_y, world_z = self.pixel_to_world(center_x, center_y, depth_m, fx, fy, cx, cy)
                
                # Create point cloud
                points, colors, depth_crop, crop_x1, crop_y1 = self.create_object_pointcloud(
                    depth_image, color_image, center_x, center_y,
                    bbox_w_scaled, bbox_h_scaled, fx, fy, cx, cy, margin=15
                )

                # Downsample if needed
                MAX_POINTS = 20000
                if len(points) > MAX_POINTS:
                    idx = np.random.choice(len(points), MAX_POINTS, replace=False)
                    points = points[idx]
                    colors = colors[idx]

                # No grasp pose generation - just 3D detection

                # Publish point cloud
                if len(points) > 0:
                    pc_msg = self.create_pointcloud2_msg(points, colors, depth_msg.header)
                    if pc_msg:
                        self.pointcloud_pub.publish(pc_msg)

                # Publish 3D center point
                point_msg = PointStamped()
                point_msg.header = depth_msg.header
                point_msg.point.x = world_x
                point_msg.point.y = world_y
                point_msg.point.z = world_z
                self.point_pub.publish(point_msg)

                # Add to detection data
                detection_entry = {
                    'class_name': class_name,
                    'confidence': float(confidence),
                    'bbox_2d': [float(cx_pixel), float(cy_pixel), float(w_pixel), float(h_pixel)],
                    'position_3d': [float(world_x), float(world_y), float(world_z)],
                    'depth_m': float(depth_m),
                    'num_points': int(len(points))
                }
                
                detection_data.append(detection_entry)
                
                # Draw detection on debug image
                cv2.rectangle(color_image, 
                            (center_x - bbox_w_scaled//2, center_y - bbox_h_scaled//2),
                            (center_x + bbox_w_scaled//2, center_y + bbox_h_scaled//2),
                            (0, 255, 0), 2)
                cv2.putText(color_image, f"{class_name} {confidence:.2f}", 
                          (center_x - bbox_w_scaled//2, center_y - bbox_h_scaled//2 - 5),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
                self.get_logger().info(
                    f'🎯 {class_name} (conf: {confidence:.2f}) '
                    f'3D: ({world_x:.3f}, {world_y:.3f}, {world_z:.3f})m '
                    f'Points: {len(points)}'
                )
            
            # Generate and publish full scene point cloud
            full_scene_points, full_scene_colors = self.create_full_scene_pointcloud(
                depth_image, color_image, fx, fy, cx, cy, 
                downsample_factor=4, max_points=50000
            )
            
            if len(full_scene_points) > 0:
                full_scene_msg = self.create_pointcloud2_msg(full_scene_points, full_scene_colors, depth_msg.header)
                if full_scene_msg:
                    self.full_scene_pub.publish(full_scene_msg)
            
            # Publish debug image
            debug_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
            debug_msg.header = depth_msg.header
            self.debug_image_pub.publish(debug_msg)
            
            # Publish metadata
            if detection_data:
                json_str = json.dumps(detection_data)
                meta_msg = String()
                meta_msg.data = json_str
                self.metadata_pub.publish(meta_msg)
                    
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