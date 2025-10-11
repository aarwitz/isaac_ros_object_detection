#!/usr/bin/env python3
"""run_robot.py

Automatic robot control script that listens to ROS detection topics
and continuously picks up detected objects using the robot arm.

Usage:
python run_robot.py /dev/ttyUSB0                    # Run with robot
python run_robot.py /dev/ttyUSB0 --dry-run         # Simulate only
python run_robot.py /dev/ttyUSB0 --once            # Pick once and exit
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import time
import argparse
import json
import threading
import queue

try:
    import serial
except Exception:
    serial = None


def realsense_to_robot_coords(rs_x, rs_y, rs_z):
    """
    Convert RealSense coordinates to robot coordinates using calibration math
    
    Args:
        rs_x, rs_y, rs_z: RealSense coordinates in meters
    
    Returns:
        dict with 'above_pick' and 'just_above_pick' robot coordinates in mm
    """

    # Above pick point (100mm higher)
    above_x = (1000 * rs_z) + 100
    above_y = (-1 * 1000 * rs_x)
    above_z = (-1 * 1000 * rs_y) + 130
    
    # Just above pick point (touching ball level)
    just_above_x = (1000 * rs_z) + 115
    just_above_y = (-1 * 1000 * rs_x) + 25
    just_above_z = (-1 * 1000 * rs_y) - 40
    
    return {
        'above_pick': {'x': above_x, 'y': above_y, 'z': above_z},
        'just_above_pick': {'x': just_above_x, 'y': just_above_y, 'z': just_above_z},
        'pick_point': {'x': just_above_x, 'y': just_above_y, 'z': just_above_z - 115},
    }

def make_robot_sequence(rs_x, rs_y, rs_z):
    """Generate robot pick sequence from RealSense coordinates"""
    
    coords = realsense_to_robot_coords(rs_x, rs_y, rs_z)
    above = coords['above_pick']
    just_above = coords['just_above_pick']
    pick_point = coords['pick_point']

    return [
        # 1. Move to above pick point
        ({"T": 104, "x": above['x'], "y": above['y'], "z": above['z'], "t": 0, "spd": 0.25}, 2.0),
        
        # 2. Move to just above pick point (about to touch)
        ({"T": 104, "x": just_above['x'], "y": just_above['y'], "z": just_above['z'], "t": 0, "spd": 0.25}, 2.0),
        
        # 3. Move to pick point actual (close gripper)
        ({"T": 104, "x": pick_point['x'], "y": pick_point['y'], "z": pick_point['z'], "t": 3.14, "spd": 0.25}, 4.0),
        
        # 4. Move back to above pick point with gripper closed
        ({"T": 104, "x": above['x'], "y": above['y'], "z": above['z'], "t": 3.14, "spd": 0.25}, 3.0),

        # 5. Stay above pick point but release sock out of gripper
        ({"T": 104, "x": above['x'], "y": above['y'], "z": above['z'], "t": 0, "spd": 0.25}, 3.0),
    ]


class RobotController(Node):
    def __init__(self, port, dry_run=False, pick_once=False):
        super().__init__('robot_controller')
        
        self.port = port
        self.dry_run = dry_run
        self.pick_once = pick_once
        self.ser = None
        
        # Queue to store detection coordinates
        self.detection_queue = queue.Queue(maxsize=5)  # Keep only last 5 detections
        
        # Subscribers
        self.point_sub = self.create_subscription(
            PointStamped, 
            'detected_objects_3d', 
            self.point_callback, 
            10
        )
        
        self.metadata_sub = self.create_subscription(
            String,
            'detection_metadata',
            self.metadata_callback,
            10
        )
        
        # Status tracking
        self.last_detection_time = 0
        self.robot_busy = False
        
        # Initialize serial connection
        if not self.dry_run:
            self.init_serial()
        
        self.get_logger().info(f'🚀 Robot Controller Started!')
        self.get_logger().info(f'📡 Port: {port}, Dry-run: {dry_run}, Pick-once: {pick_once}')
        self.get_logger().info('🎯 Waiting for object detections...')

    def init_serial(self):
        """Initialize serial connection to robot"""
        try:
            if serial is None:
                raise RuntimeError("pyserial required: pip install pyserial")
            
            self.ser = serial.Serial(self.port, baudrate=115200, dsrdtr=None)
            self.ser.setRTS(False)
            self.ser.setDTR(False)
            time.sleep(0.1)
            self.get_logger().info(f"🔌 Connected to robot on {self.port}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to connect to robot: {e}")
            raise

    def point_callback(self, msg):
        """Callback for 3D point detections"""
        try:
            x = msg.point.x
            y = msg.point.y 
            z = msg.point.z
            
            # Update detection queue (remove oldest if full)
            if self.detection_queue.full():
                try:
                    self.detection_queue.get_nowait()
                except queue.Empty:
                    pass
            
            self.detection_queue.put((x, y, z, time.time()))
            self.last_detection_time = time.time()
            
            self.get_logger().info(f'🎯 Detection: ({x:.3f}, {y:.3f}, {z:.3f})')
            
            # Start robot sequence if not busy
            if not self.robot_busy:
                self.start_robot_sequence()
                
        except Exception as e:
            self.get_logger().error(f'❌ Error in point callback: {e}')

    def metadata_callback(self, msg):
        """Callback for detection metadata (for additional info)"""
        try:
            data = json.loads(msg.data)
            if data:
                # Log first detection info
                detection = data[0]
                self.get_logger().debug(
                    f"📊 {detection['class_name']} confidence: {detection['confidence']:.3f}, "
                    f"graspable: {detection.get('graspable', False)}"
                )
        except Exception as e:
            self.get_logger().debug(f'Metadata parse error: {e}')

    def start_robot_sequence(self):
        """Start robot pick sequence in separate thread"""
        if self.robot_busy:
            return
            
        try:
            # Get latest detection
            x, y, z, timestamp = self.detection_queue.get_nowait()
            
            # Start robot sequence in background thread
            robot_thread = threading.Thread(
                target=self.execute_robot_sequence,
                args=(x, y, z),
                daemon=True
            )
            robot_thread.start()
            
        except queue.Empty:
            self.get_logger().debug('No detections available for robot sequence')

    def execute_robot_sequence(self, rs_x, rs_y, rs_z):
        """Execute complete robot pick sequence"""
        self.robot_busy = True
        
        try:
            self.get_logger().info(f'🤖 Starting robot sequence for ({rs_x:.3f}, {rs_y:.3f}, {rs_z:.3f})')
            
            # Generate robot sequence
            sequence = make_robot_sequence(rs_x, rs_y, rs_z)
            
            # Execute sequence
            self.send_sequence(sequence)
            
            self.get_logger().info('✅ Robot sequence completed!')
            
            # If pick_once mode, shutdown
            if self.pick_once:
                self.get_logger().info('🛑 Pick-once mode: shutting down')
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'❌ Robot sequence failed: {e}')
        finally:
            self.robot_busy = False

    def send_sequence(self, sequence):
        """Send command sequence to robot"""
        if self.dry_run:
            self.get_logger().info("🧪 Dry-run mode: printing commands only")
        
        try:
            for i, (msg, delay) in enumerate(sequence, start=1):
                line = json.dumps(msg, separators=(',', ':'))
                self.get_logger().info(f"Step {i}/{len(sequence)}: {line} (delay {delay}s)")
                
                if not self.dry_run and self.ser:
                    self.ser.write(line.encode('utf-8') + b"\n")
                
                time.sleep(delay)
                
        except Exception as e:
            self.get_logger().error(f'❌ Send sequence error: {e}')

    def cleanup(self):
        """Clean up resources"""
        if self.ser:
            self.ser.close()
            self.get_logger().info('🔌 Serial connection closed')


def main():
    parser = argparse.ArgumentParser(description='Automatic robot controller for object picking')
    parser.add_argument('port', type=str, help='Serial port (e.g., /dev/ttyUSB0)')
    parser.add_argument('--dry-run', action='store_true', help='Print commands instead of sending')
    parser.add_argument('--once', action='store_true', help='Pick once and exit')
    
    args = parser.parse_args()
    
    # Initialize ROS
    rclpy.init()
    
    controller = None
    try:
        # Create robot controller
        controller = RobotController(args.port, dry_run=args.dry_run, pick_once=args.once)
        
        # Start spinning
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        print('\n🛑 Shutting down robot controller...')
    except Exception as e:
        print(f'❌ Error: {e}')
    finally:
        if controller:
            controller.cleanup()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()