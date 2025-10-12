#!/usr/bin/env python3
"""simple_robot_pick_test.py

Simple test script to convert RealSense coordinates to robot coordinates
Takes x,y,z from RealSense and calculates robot positions using your calibration math

Usage:
python simple_realsense_test.py -0.154 0.035 0.308 /dev/ttyUSB0 --dry-run
"""

import time
import argparse
import json

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
    above_x = (1000 * rs_z) + 50
    above_y = (-1 * 1000 * rs_x)
    above_z = (-1 * 1000 * rs_y) + 130
    
    # Just above pick point (touching ball level)
    just_above_x = (1000 * rs_z) + 60
    just_above_y = (-1 * 1000 * rs_x) + 25
    just_above_z = (-1 * 1000 * rs_y) - 40
    
    return {
        'above_pick': {'x': above_x, 'y': above_y, 'z': above_z},
        'just_above_pick': {'x': just_above_x, 'y': just_above_y, 'z': just_above_z},
        'pick_point': {'x': just_above_x, 'y': just_above_y, 'z': just_above_z - 95},
    }

def make_test_sequence(rs_x, rs_y, rs_z):
    """Generate robot test sequence from RealSense coordinates"""
    
    coords = realsense_to_robot_coords(rs_x, rs_y, rs_z)
    above = coords['above_pick']
    just_above = coords['just_above_pick']
    pick_point = coords['pick_point']

    return [
     
        # 2. Move to above pick point
        ({"T": 104, "x": above['x'], "y": above['y'], "z": above['z'], "t": 0, "spd": 0.25}, 2.0),
        
        # 3. Move to just above pick point (about to touch)
        ({"T": 104, "x": just_above['x'], "y": just_above['y'], "z": just_above['z'], "t": 0, "spd": 0.25}, 1.0),
        # 3. Move to pick point actual
        ({"T": 104, "x": pick_point['x'], "y": pick_point['y'], "z": pick_point['z'], "t": 3.14, "spd": 0.25}, 2.0),
        #4. Move back to above pick point but with gripper closed
        ({"T": 104, "x": above['x'], "y": above['y'], "z": above['z'], "t": 3.14, "spd": 0.25}, 1.5),
        #5 Stay above pick point but release sock out of gripper
        ({"T": 104, "x": above['x'], "y": above['y'], "z": above['z'], "t": 3.14, "spd": 0.25}, 5.0),
    ]


def send_sequence(port: str, sequence, dry_run: bool = False):
    """Send the sequence to the serial port"""
    if dry_run:
        print("🧪 Dry-run mode: printing commands only")
    else:
        if serial is None:
            raise RuntimeError("pyserial required: pip install pyserial")

    ser = None
    try:
        if not dry_run:
            ser = serial.Serial(port, baudrate=115200, dsrdtr=None)
            ser.setRTS(False)
            ser.setDTR(False)
            time.sleep(0.1)
            print(f"🔌 Connected to {port}")

        for i, (msg, delay) in enumerate(sequence, start=1):
            line = json.dumps(msg, separators=(',', ':'))
            print(f"Step {i}/{len(sequence)}: {line} (delay {delay}s)")
            
            if not dry_run:
                ser.write(line.encode('utf-8') + b"\n")
            
            time.sleep(delay)

    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if ser is not None:
            ser.close()
            print("🔌 Serial connection closed")


def main():
    parser = argparse.ArgumentParser(description='Test RealSense to robot coordinate conversion')
    parser.add_argument('rs_x', type=float, help='RealSense X coordinate in meters')
    parser.add_argument('rs_y', type=float, help='RealSense Y coordinate in meters') 
    parser.add_argument('rs_z', type=float, help='RealSense Z coordinate in meters')
    parser.add_argument('port', type=str, help='Serial port (e.g., /dev/ttyUSB0)')
    parser.add_argument('--dry-run', action='store_true', help='Print commands instead of sending')
    
    args = parser.parse_args()
    
    # Generate sequence
    sequence = make_test_sequence(args.rs_x, args.rs_y, args.rs_z)
    
    print(f"\n🚀 Starting coordinate conversion test...")
    print(f"⏱️  Total steps: {len(sequence)}")
    print(f"⏱️  Estimated time: {sum(delay for _, delay in sequence):.1f} seconds")
    
    if not args.dry_run:
        response = input(f"\n⚠️  Ready to send commands to robot on {args.port}? (y/N): ")
        if response.lower() != 'y':
            print("❌ Aborted")
            return
    
    print("\n" + "="*60)
    send_sequence(args.port, sequence, dry_run=args.dry_run)
    print("="*60)
    print("✅ Test complete!")


if __name__ == '__main__':
    main()


"""
Example usage:
python /workspaces/isaac_ros-dev/src/isaac_ros_object_detection/isaac_ros_yolov8/myscripts/simple_robot_pick_test.py -0.182 0.071 0.615 /dev/ttyUSB0 --dry-run

"""