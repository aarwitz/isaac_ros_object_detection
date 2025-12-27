#!/usr/bin/env python3
"""
Simple Script Combiner - Lists deployment and build files
Creates a manifest of all files used for frozen deployment
"""

import os
from datetime import datetime

def combine_deployment_files():
    """List all deployment and build files"""

    # Define deployment files
    deployment_files = [
        # Core deployment scripts
        {
            'name': 'build.sh',
            'description': 'Build script - builds frozen container',
            'path': 'build.sh'
        },
        {
            'name': 'deploy.sh',
            'description': 'Deploy script - runs container with Jetson mounts',
            'path': 'deploy.sh'
        },
        {
            'name': 'restore-base-image.sh',
            'description': 'Restores frozen base image from tarball',
            'path': 'restore-base-image.sh'
        },

        # Container definition
        {
            'name': 'Dockerfile.production.v2',
            'description': 'Production container - single provenance (no apt Isaac ROS)',
            'path': 'Dockerfile.production.v2'
        },
        {
            'name': 'entrypoint.sh',
            'description': 'Container startup script',
            'path': 'entrypoint.sh'
        },

        # Configuration
        {
            'name': 'requirements.txt',
            'description': 'Python dependencies (pinned versions)',
            'path': 'requirements.txt'
        },
        {
            'name': 'IMAGE_MANIFEST.txt',
            'description': 'Build metadata (JetPack, kernel, versions)',
            'path': 'IMAGE_MANIFEST.txt'
        },

        # Documentation
        # {
        #     'name': 'README.md',
        #     'description': 'Quick start guide for frozen deployment',
        #     'path': 'README.md'
        # },

        # cleaningrobot_bringup package files
        {
            'name': 'cleaningrobot_bringup/CMakeLists.txt',
            'description': 'Build configuration for cleaningrobot_bringup package',
            'path': '../../../cleaningrobot_bringup/CMakeLists.txt'
        },
        {
            'name': 'cleaningrobot_bringup/package.xml',
            'description': 'ROS 2 package manifest for cleaningrobot_bringup',
            'path': '../../../cleaningrobot_bringup/package.xml'
        },
        {
            'name': 'cleaningrobot_bringup/launch/vision_yolov8.launch.py',
            'description': 'PRODUCTION: Full YOLOv8 pipeline (RealSense → inference)',
            'path': '../../../cleaningrobot_bringup/launch/vision_yolov8.launch.py'
        },
        {
            'name': 'cleaningrobot_bringup/launch/realsense_only.launch.py',
            'description': 'TEST 1: Camera only',
            'path': '../../../cleaningrobot_bringup/launch/realsense_only.launch.py'
        },
        {
            'name': 'cleaningrobot_bringup/launch/image_proc_only.launch.py',
            'description': 'TEST 2: Image processing pipeline',
            'path': '../../../cleaningrobot_bringup/launch/image_proc_only.launch.py'
        },
        {
            'name': 'cleaningrobot_bringup/launch/tensor_rt_only.launch.py',
            'description': 'TEST 3: TensorRT inference only',
            'path': '../../../cleaningrobot_bringup/launch/tensor_rt_only.launch.py'
        },

        # Base image (stored separately)
        {
            'name': 'isaac_ros_dev-aarch64-frozen.tar.gz',
            'description': '13GB frozen base image (store separately, not in git)',
            'path': '../../../../../isaac_ros_dev-aarch64-frozen.tar.gz'
        }
    ]

    # Output file
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_file = os.path.join(script_dir, 'DEPLOYMENT_FILES_MANIFEST.txt')

    # Generate manifest
    with open(output_file, 'w') as outfile:
        # Write header
        outfile.write("=" * 80 + "\n")
        outfile.write("FROZEN DEPLOYMENT FILES MANIFEST\n")
        outfile.write("=" * 80 + "\n")
        outfile.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        outfile.write(f"Location: myscripts/\n")
        outfile.write("=" * 80 + "\n\n")

        # Write file list
        outfile.write("DEPLOYMENT FILES:\n")
        outfile.write("-" * 80 + "\n\n")

        # Write file list with details
        for i, file_info in enumerate(deployment_files, 1):
            outfile.write(f"{i}. {file_info['name']}\n")
            outfile.write(f"   Description: {file_info['description']}\n")
            outfile.write(f"   Path: {file_info['path']}\n")

            # Check if file exists
            file_path = os.path.join(script_dir, file_info['path'])
            if os.path.exists(file_path):
                if os.path.isfile(file_path):
                    size = os.path.getsize(file_path)
                    if size > 1024*1024:
                        outfile.write(f"   Size: {size/(1024*1024*1024):.1f}GB\n")
                    elif size > 1024:
                        outfile.write(f"   Size: {size/1024:.1f}KB\n")
                    else:
                        outfile.write(f"   Size: {size}B\n")
                    outfile.write(f"   Status: ✓ EXISTS\n")
                else:
                    outfile.write(f"   Status: ✓ EXISTS (directory)\n")
            else:
                outfile.write(f"   Status: ✗ NOT FOUND\n")
            outfile.write("\n")

        # Write summary
        outfile.write("=" * 80 + "\n")
        outfile.write("SUMMARY\n")
        outfile.write("=" * 80 + "\n\n")
        outfile.write("Essential files for frozen deployment:\n")
        outfile.write("  • Build/Deploy: build.sh, deploy.sh, restore-base-image.sh\n")
        outfile.write("  • Container: Dockerfile.production.v2, entrypoint.sh\n")
        outfile.write("  • Config: requirements.txt, IMAGE_MANIFEST.txt\n")
        outfile.write("  • Launch Files: vision_yolov8.launch.py + 3 test files\n")
        outfile.write("  • Docs: README.md (myscripts + cleaningrobot_bringup)\n")
        outfile.write("  • Base Image: isaac_ros_dev-aarch64-frozen.tar.gz (13GB)\n\n")
        outfile.write("What to commit to git:\n")
        outfile.write("  ✓ All scripts, configs, launch files, and docs\n")
        outfile.write("  ✗ isaac_ros_dev-aarch64-frozen.tar.gz (too large, store separately)\n\n")
        outfile.write("Frozen deployment model:\n")
        outfile.write("  • Isaac ROS infrastructure frozen in base image\n")
        outfile.write("  • Application packages (isaac_ros_yolov8, cleaningrobot_bringup) built from source\n")
        outfile.write("  • No isaac_ros_examples dependency (prevents ABI conflicts)\n")
        outfile.write("  • Deterministic builds via frozen base + selective package compilation\n\n")

        # Write full file contents
        outfile.write("=" * 80 + "\n")
        outfile.write("FILE CONTENTS\n")
        outfile.write("=" * 80 + "\n\n")

        for i, file_info in enumerate(deployment_files, 1):
            # Skip the tarball
            if file_info['name'] == 'isaac_ros_dev-aarch64-frozen.tar.gz':
                continue

            file_path = os.path.join(script_dir, file_info['path'])

            outfile.write("=" * 80 + "\n")
            outfile.write(f"FILE {i}: {file_info['name']}\n")
            outfile.write("=" * 80 + "\n")
            outfile.write(f"Description: {file_info['description']}\n")
            outfile.write(f"Path: {file_info['path']}\n")
            outfile.write("-" * 80 + "\n\n")

            try:
                if os.path.isfile(file_path):
                    with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
                        content = f.read()
                        outfile.write(content)
                        if not content.endswith('\n'):
                            outfile.write('\n')
                    print(f"✓ Added {file_info['name']}")
                elif os.path.isdir(file_path):
                    outfile.write(f"[This is a directory - contents not shown]\n")
                    print(f"⊘ Skipped directory {file_info['name']}")
                else:
                    outfile.write(f"[File not found]\n")
                    print(f"✗ Not found: {file_info['name']}")
            except Exception as e:
                outfile.write(f"[Error reading file: {e}]\n")
                print(f"✗ Error reading {file_info['name']}: {e}")

            outfile.write("\n\n")

    print(f"\n✓ Manifest created: {output_file}")
    return output_file

if __name__ == "__main__":
    combine_deployment_files()
