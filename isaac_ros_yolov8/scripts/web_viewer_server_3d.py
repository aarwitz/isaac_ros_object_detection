#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge

import cv2
import asyncio
import websockets
import threading
import time
import json
import numpy as np
import struct
import traceback

class Enhanced3DViewer(Node):
    def __init__(self):
        super().__init__('enhanced_3d_viewer_debug')
        self.bridge = CvBridge()
        
        # Store latest data with timestamps
        self._latest_2d_frame = None
        self._latest_pointcloud = None
        self._latest_metadata = None
        self._last_2d_time = 0
        self._last_3d_time = 0
        self._last_meta_time = 0
        self._lock = threading.Lock()

        # Counters for debugging
        self._2d_count = 0
        self._3d_count = 0
        self._meta_count = 0

        # Subscribers with debugging
        self.create_subscription(Image, '/yolov8_processed_image', self.image_callback, 10)
        self.create_subscription(PointCloud2, '/detected_objects_pointcloud', self.pointcloud_callback, 10)
        self.create_subscription(Image, '/detection_metadata', self.metadata_callback, 10)

        # Create a timer to report status
        self.create_timer(5.0, self.report_status)

        self.get_logger().info('🚀 Enhanced 3D Viewer Debug Node Started!')

    def report_status(self):
        """Report status every 5 seconds"""
        with self._lock:
            now = time.time()
            self.get_logger().info(
                f"📊 Status - 2D: {self._2d_count} msgs "
                f"(last: {now - self._last_2d_time:.1f}s ago), "
                f"3D: {self._3d_count} msgs "
                f"(last: {now - self._last_3d_time:.1f}s ago), "
                f"Meta: {self._meta_count} msgs "
                f"(last: {now - self._last_meta_time:.1f}s ago)"
            )

    def image_callback(self, msg):
        try:
            self._2d_count += 1
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ok:
                with self._lock:
                    self._latest_2d_frame = jpeg.tobytes()
                    self._last_2d_time = time.time()
                if self._2d_count % 10 == 0:  # Log every 10th frame
                    self.get_logger().info(f"🖼️ Processed 2D frame #{self._2d_count}, size: {len(jpeg.tobytes())} bytes")
        except Exception as e:
            self.get_logger().error(f"Failed to process 2D image: {e}")
            traceback.print_exc()

    def pointcloud_callback(self, msg):
        try:
            self._3d_count += 1
            # Convert PointCloud2 to JSON for web transmission
            points_data = self.pointcloud2_to_json(msg)
            with self._lock:
                self._latest_pointcloud = json.dumps(points_data).encode('utf-8')
                self._last_3d_time = time.time()
            if self._3d_count % 5 == 0:  # Log every 5th point cloud
                self.get_logger().info(f"📡 Processed point cloud #{self._3d_count} with {points_data['num_points']} points")
        except Exception as e:
            self.get_logger().error(f"Failed to process point cloud: {e}")
            traceback.print_exc()

    def metadata_callback(self, msg):
        try:
            self._meta_count += 1
            # Convert array.array to bytes first, then decode
            if hasattr(msg.data, 'tobytes'):
                json_bytes = msg.data.tobytes()
            else:
                # Handle different array types
                json_bytes = bytes(msg.data)
            
            json_str = json_bytes.decode('utf-8')
            with self._lock:
                self._latest_metadata = json_str.encode('utf-8')
                self._last_meta_time = time.time()
            if self._meta_count % 5 == 0:  # Log every 5th metadata
                self.get_logger().info(f"📊 Received metadata #{self._meta_count}: {json_str[:100]}...")
        except Exception as e:
            self.get_logger().error(f"Failed to process metadata: {e}")
            traceback.print_exc()

    def pointcloud2_to_json(self, pc_msg):
        """Convert PointCloud2 to JSON format for web visualization"""
        try:
            # Parse point cloud data
            points = []
            colors = []
            
            # Get field offsets
            x_offset = y_offset = z_offset = rgb_offset = None
            for field in pc_msg.fields:
                if field.name == 'x':
                    x_offset = field.offset
                elif field.name == 'y':
                    y_offset = field.offset
                elif field.name == 'z':
                    z_offset = field.offset
                elif field.name == 'rgb':
                    rgb_offset = field.offset
            
            if any(offset is None for offset in [x_offset, y_offset, z_offset, rgb_offset]):
                self.get_logger().warn(f"Missing required point cloud fields. Available: {[f.name for f in pc_msg.fields]}")
                return {'points': [], 'colors': [], 'num_points': 0, 'timestamp': time.time()}
            
            # Convert array.array to bytes if needed
            if hasattr(pc_msg.data, 'tobytes'):
                data_bytes = pc_msg.data.tobytes()
            else:
                data_bytes = bytes(pc_msg.data)
            
            # Extract points
            point_step = pc_msg.point_step
            max_points = min(pc_msg.width, 5000)  # Increased limit from 1000 to 5000 points
            for i in range(max_points):
                offset = i * point_step
                
                try:
                    # Extract XYZ
                    x = struct.unpack('f', data_bytes[offset + x_offset:offset + x_offset + 4])[0]
                    y = struct.unpack('f', data_bytes[offset + y_offset:offset + y_offset + 4])[0]
                    z = struct.unpack('f', data_bytes[offset + z_offset:offset + z_offset + 4])[0]
                    
                    # Extract RGB
                    rgb_packed = struct.unpack('I', data_bytes[offset + rgb_offset:offset + rgb_offset + 4])[0]
                    r = (rgb_packed >> 16) & 0xFF
                    g = (rgb_packed >> 8) & 0xFF
                    b = rgb_packed & 0xFF
                    
                    points.extend([float(x), float(y), float(z)])  # Ensure float type
                    colors.extend([r/255.0, g/255.0, b/255.0])  # Normalize to 0-1
                except struct.error as e:
                    self.get_logger().warn(f"Failed to unpack point {i}: {e}")
                    continue
            
            return {
                'points': points,
                'colors': colors,
                'num_points': len(points) // 3,
                'timestamp': time.time()
            }
        except Exception as e:
            self.get_logger().error(f"Error in pointcloud2_to_json: {e}")
            traceback.print_exc()
            return {'points': [], 'colors': [], 'num_points': 0, 'timestamp': time.time()}

    def get_latest_data(self, data_type):
        with self._lock:
            if data_type == '2d':
                return self._latest_2d_frame
            elif data_type == '3d':
                return self._latest_pointcloud
            elif data_type == 'metadata':
                return self._latest_metadata
        return None

    def has_data(self, data_type):
        """Check if we have recent data"""
        now = time.time()
        with self._lock:
            if data_type == '2d':
                return self._latest_2d_frame is not None and (now - self._last_2d_time) < 10
            elif data_type == '3d':
                return self._latest_pointcloud is not None and (now - self._last_3d_time) < 10
            elif data_type == 'metadata':
                return self._latest_metadata is not None and (now - self._last_meta_time) < 10
        return False

node_ref = None

async def stream_handler(websocket, path=None):
    """Handle WebSocket connections with better error handling"""
    
    # Extract path from websocket if not provided as parameter
    if path is None:
        try:
            path = websocket.path
        except AttributeError:
            # For newer websockets library versions
            path = websocket.request.path if hasattr(websocket, 'request') else '/'
    
    print(f"🔌 Client connected to {path}")
    
    try:
        # Determine what type of data to stream based on path
        if path == '/2d':
            data_type = '2d'
        elif path == '/3d':
            data_type = '3d'
        elif path == '/metadata':
            data_type = 'metadata'
        else:
            await websocket.send("Invalid path. Use /2d, /3d, or /metadata")
            return

        # Send initial status
        if node_ref and not node_ref.has_data(data_type):
            await websocket.send(f"Waiting for {data_type} data...".encode('utf-8'))
            print(f"⏳ Waiting for {data_type} data...")

        frame_count = 0
        while True:
            try:
                data = node_ref.get_latest_data(data_type) if node_ref else None
                if data:
                    await websocket.send(data)
                    frame_count += 1
                    if frame_count % 20 == 0:  # Log every 20 frames
                        print(f"📤 Sent {frame_count} {data_type} frames to client")
                else:
                    # Send a small keepalive message
                    await websocket.send(b'')
                
                await asyncio.sleep(0.05)  # 20 FPS
                
            except websockets.exceptions.ConnectionClosed:
                print(f"🔌 Client disconnected from {path}")
                break
            except Exception as e:
                print(f"❌ Send error on {path}: {e}")
                traceback.print_exc()
                break
            
    except Exception as e:
        print(f"❌ Stream handler error on {path}: {e}")
        traceback.print_exc()
    finally:
        print(f"✅ Stream handler done for {path}")

async def main_ws_server():
    print("Starting Enhanced WebSocket server on port 8765...")
    print("Available endpoints:")
    print("  ws://localhost:8765/2d - 2D detection images")
    print("  ws://localhost:8765/3d - 3D point cloud data")
    print("  ws://localhost:8765/metadata - Detection metadata")
    
    try:
        server = await websockets.serve(stream_handler, "0.0.0.0", 8765)
        print("✅ WebSocket server running!")
        await server.wait_closed()
    except Exception as e:
        print(f"❌ Server error: {e}")
        traceback.print_exc()

def spin_ros():
    try:
        print("🔄 Starting ROS spin...")
        rclpy.spin(node_ref)
    except Exception as e:
        print(f"❌ ROS spin error: {e}")
        traceback.print_exc()

def main():
    global node_ref
    
    try:
        print("🚀 Starting Enhanced 3D Viewer...")
        rclpy.init()
        node_ref = Enhanced3DViewer()

        ros_thread = threading.Thread(target=spin_ros, daemon=True)
        ros_thread.start()
        
        print("⏳ Waiting 3 seconds for ROS to initialize...")
        time.sleep(3)

        # Run the websocket server
        asyncio.run(main_ws_server())
        
    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")
    except Exception as e:
        print(f"❌ Main error: {e}")
        traceback.print_exc()
    finally:
        if node_ref:
            node_ref.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()