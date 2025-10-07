# CleaningRobot

This repository contains scripts and notes for the main hardware pieces of my robot:
- Intel RealSense in Isaac Ros environment
- Waveshare RoArm M2-S 4 axis arm
- HiWonder 4 wheel motor driver

Below are clean, copy-paste friendly commands and examples. All commands use plain ASCII quotes and are given as single lines so you can run them one at a time in a terminal.

## Isaac ROS / development helper commands for setting up vision system

Change to the workspace subdirectory:

```bash
cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common
```

Run the development script (example):

```bash
./scripts/run_dev.sh -d ~/workspaces/isaac_ros-dev -i ros2_humble.realsense
```

If that doesn't work or can't exist GPU or camera in container, add options:
```bash
docker run -it --rm --privileged --network host --ipc=host \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$HOME/.Xauthority":/home/admin/.Xauthority:rw \
  -e DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e ROS_DOMAIN_ID -e USER -e ISAAC_ROS_WS=/workspaces/isaac_ros-dev \
  -e HOST_USER_UID=$(id -u) -e HOST_USER_GID=$(id -g) \
  --pid=host -v /dev/input:/dev/input \
  -v /dev/bus/usb:/dev/bus/usb \
  -v /home/taylor/workspaces/isaac_ros-dev:/workspaces/isaac_ros-dev \
  --name isaac_ros_dev-aarch64-container \
  --gpus all \
  isaac_ros_dev-aarch64 /bin/bash
```

Add the ROS apt key if needed:

```bash
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Update apt and install example Isaac ROS packages:

```bash
sudo apt-get update
```

```bash
sudo apt-get install -y ros-humble-isaac-ros-yolov8 ros-humble-isaac-ros-dnn-image-encoder ros-humble-isaac-ros-tensor-rt
```

```bash
sudo apt-get install -y ros-humble-isaac-ros-examples ros-humble-isaac-ros-realsense
```

```bash
pip install websockets
```

Run the launch file for RealSense
```bash
ros2 launch isaac_ros_examples isaac_ros_examples.launch.py launch_fragments:=realsense_mono_rect_depth,yolov8 model_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8/yolov8s.onnx engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8/yolov8s.plan
```

My model for dirt on wood floor
```bash
ros2 launch isaac_ros_examples isaac_ros_examples.launch.py launch_fragments:=realsense_mono_rect_depth,yolov8    model_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8/fixed.onnx engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8/fixed.plan
```

In another terminal in same container run the YOLOv8 visualization script:
```bash
ros2 run isaac_ros_yolov8 isaac_ros_yolov8_visualizer.py
```

In another terminal in same container run my 3d detector:
```bash
python /workspaces/isaac_ros-dev/src/isaac_ros_object_detection/isaac_ros_yolov8/scripts/yolov8_3d_grasp_detector.py
```

In another terminal in same container run my webviewer script:
```bash
python /workspaces/isaac_ros-dev/src/isaac_ros_object_detection/isaac_ros_yolov8/scripts/web_viewer_server_3d.py
```

And then on a client (I am just using my mac laptop) open up viewer.html