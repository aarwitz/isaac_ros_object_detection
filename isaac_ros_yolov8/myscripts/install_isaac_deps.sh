#!/bin/bash

# Isaac ROS Dependencies Installation Script
# This script installs the required Isaac ROS packages and dependencies

set -e  # Exit on any error

echo "🚀 Installing Isaac ROS Dependencies..."
echo "======================================"

# Update package lists
echo "📦 Updating package lists..."
sudo apt-get update

# Install core Isaac ROS packages
echo "🤖 Installing Isaac ROS core packages..."
sudo apt-get install -y \
    ros-humble-isaac-ros-yolov8 \
    ros-humble-isaac-ros-dnn-image-encoder \
    ros-humble-isaac-ros-tensor-rt

# Install Isaac ROS examples and RealSense packages
echo "📸 Installing Isaac ROS examples and RealSense packages..."
sudo apt-get install -y \
    ros-humble-isaac-ros-examples \
    ros-humble-isaac-ros-realsense

# Install Python websockets package
echo "🐍 Installing Python websockets..."
pip install websockets

# Install additional debugging and development tools for devcontainer
echo "🔧 Installing development tools..."
sudo apt-get install -y \
    gdb \
    python3-dev \
    python3-pip \
    git

# Install Python debugging packages
echo "🐛 Installing Python debugging packages..."
pip install debugpy ptvsd

echo "✅ All Isaac ROS dependencies installed successfully!"
echo ""
echo "💡 Next steps:"
echo "   1. Source ROS: source /opt/ros/humble/setup.bash"
echo "   2. Build workspace: colcon build --symlink-install"
echo "   3. Source workspace: source install/setup.bash"
echo "   4. You can now debug with VS Code breakpoints!"