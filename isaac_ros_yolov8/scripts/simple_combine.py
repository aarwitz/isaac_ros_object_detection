#!/usr/bin/env python3
"""
Simple Script Combiner for Core YOLOv8 3D Grasp Detection Files
Combines only the 4 essential scripts into a single file
"""

import os
from datetime import datetime

def combine_core_scripts():
    """Combine the 4 core YOLOv8 3D grasp detection scripts"""
    
    # Define only the core script files
    script_files = [
        {
            'name': 'yolov8_3d_grasp_detector.py',
            'description': 'GGCNN detection + grasp',
            'path': '/home/taylor/workspaces/isaac_ros-dev/src/isaac_ros_object_detection/isaac_ros_yolov8/scripts/yolov8_3d_grasp_detector.py'
        },
        {
            'name': 'web_viewer_server_3d.py', 
            'description': 'WebSocket server',
            'path': '/home/taylor/workspaces/isaac_ros-dev/src/isaac_ros_object_detection/isaac_ros_yolov8/scripts/web_viewer_server_3d.py'
        },
        {
            'name': 'enhanced_viewer_fixed.html',
            'description': 'Web interface', 
            'path': '/home/taylor/workspaces/isaac_ros-dev/src/isaac_ros_object_detection/isaac_ros_yolov8/scripts/enhanced_viewer_fixed.html'
        },
        {
            'name': 'ggcnn_robot_controller.py',
            'description': 'Robot control',
            'path': '/home/taylor/workspaces/isaac_ros-dev/src/isaac_ros_object_detection/isaac_ros_yolov8/scripts/ggcnn_robot_controller.py'
        }
    ]
    
    # Output file
    output_file = '/home/taylor/workspaces/isaac_ros-dev/src/isaac_ros_object_detection/isaac_ros_yolov8/scripts/core_scripts_combined.txt'
    
    # Generate combined file
    with open(output_file, 'w') as outfile:
        # Write header
        outfile.write("=" * 60 + "\n")
        outfile.write("CORE YOLOV8 3D GRASP DETECTION SCRIPTS\n")
        outfile.write("=" * 60 + "\n")
        outfile.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        outfile.write("=" * 60 + "\n\n")
        
        # Write table of contents
        outfile.write("CONTENTS:\n")
        outfile.write("-" * 20 + "\n")
        for i, script in enumerate(script_files, 1):
            outfile.write(f"{i}. {script['name']} ({script['description']})\n")
        outfile.write("\n" + "=" * 60 + "\n\n")
        
        # Process each script file
        for i, script in enumerate(script_files, 1):
            print(f"Processing {script['name']}...")
            
            # Write section header
            outfile.write("=" * 60 + "\n")
            outfile.write(f"{i}. {script['name'].upper()}\n")
            outfile.write("=" * 60 + "\n")
            outfile.write(f"Description: {script['description']}\n")
            outfile.write("-" * 60 + "\n\n")
            
            # Read and write file content
            try:
                with open(script['path'], 'r', encoding='utf-8') as infile:
                    content = infile.read()
                    outfile.write(content)
                    
                    # Ensure proper spacing between files
                    if not content.endswith('\n'):
                        outfile.write('\n')
                    outfile.write('\n\n')
                    
                print(f"✅ Added {script['name']}")
                
            except FileNotFoundError:
                error_msg = f"❌ File not found: {script['path']}\n"
                outfile.write(error_msg)
                print(error_msg.strip())
                
            except Exception as e:
                error_msg = f"❌ Error reading {script['name']}: {str(e)}\n"
                outfile.write(error_msg)
                print(error_msg.strip())
        
        # Write footer
        outfile.write("=" * 60 + "\n")
        outfile.write("END OF CORE SCRIPTS\n")
        outfile.write("=" * 60 + "\n")
    
    print(f"\n✅ Created: {output_file}")
    
    # Display file size
    try:
        file_size = os.path.getsize(output_file)
        print(f"📊 Size: {file_size:,} bytes ({file_size/1024:.1f} KB)")
    except:
        pass
    
    return output_file

def main():
    """Main function"""
    print("🚀 Core YOLOv8 3D Grasp Scripts Combiner")
    print("=" * 40)
    
    try:
        output_file = combine_core_scripts()
        
        print(f"\n✅ Combined core scripts!")
        print(f"📂 Output: {os.path.basename(output_file)}")
        print("\n📋 Combined:")
        print("   1. yolov8_3d_grasp_detector.py (GGCNN detection + grasp)")
        print("   2. web_viewer_server_3d.py (WebSocket server)")
        print("   3. enhanced_viewer_fixed.html (Web interface)")
        print("   4. ggcnn_robot_controller.py (Robot control)")
        
    except Exception as e:
        print(f"❌ Error: {str(e)}")
        return 1
    
    return 0

if __name__ == "__main__":
    exit(main())