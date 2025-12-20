# CleaningRobot vision system

## Isaac ROS / development helper commands for setting up vision system

Enter docker container
```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
   ./scripts/run_dev.sh
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

Run the launch file for yolov8 pretrained
```bash
ros2 launch isaac_ros_examples isaac_ros_examples.launch.py launch_fragments:=realsense_mono_rect_depth,yolov8 model_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8/yolov8s.onnx engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8/yolov8s.plan
```

Check topics to see if they are being published to:
```bash
ros2 topic hz /image_rect # color
```
```bash
ros2 topic hz /aligned_depth_to_color/image_raw  # depth
```
```bash
ros2 topic hz /detections_output # yolo detections
```

To run finetuned models, make sure to source modified decoder build (after sourcing humble)
Make sure to source both in both yolov8 launch and yolov8 visualizer!
```bash
source /opt/ros/humble/setup.bash
```
```bash
source /workspaces/isaac_ros-dev/install/setup.bash
```

My model for dirt on wood floor
```bash
ros2 launch isaac_ros_examples isaac_ros_examples.launch.py launch_fragments:=realsense_mono_rect_depth,yolov8    model_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8/fixed.onnx engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8/fixed.plan
```

My model for socks on white carpter
```bash
ros2 launch isaac_ros_examples isaac_ros_examples.launch.py launch_fragments:=realsense_mono_rect_depth,yolov8 model_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8/socks2.onnx engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8/socks2.plan confidence_threshold:=0.83
```


In another terminal in same container run the YOLOv8 visualization script:
```bash
ros2 run isaac_ros_yolov8 isaac_ros_yolov8_visualizer.py
```

In another terminal in same container run my 3d detector:
```bash
python /workspaces/isaac_ros-dev/src/isaac_ros_object_detection/isaac_ros_yolov8/myscripts/yolov8_3d_detector.py
```

In another terminal in same container run my webviewer script:
```bash
python /workspaces/isaac_ros-dev/src/isaac_ros_object_detection/isaac_ros_yolov8/myscripts/web_viewer_server_3d.py
```

And then on a client (I am just using my mac laptop) open up viewer.html


# Pick and place detected items with Waseshare RoArm M2-S
Source the venv
```bash
source /workspaces/isaac_ros-dev/src/RoArm-M2-S_python/roarmpython-env/bin/activate
```

Copy an (x,y,z) from the detection output in the GUI and see what poses our script generates with a dry run before running
```bash
python /workspaces/isaac_ros-dev/src/isaac_ros_object_detection/isaac_ros_yolov8/myscripts/simple_robot_pick_test.py -0.002 0.039 0.330 /dev/ttyUSB0 --dry-run 
```

And then you can try them out with
```bash
python /workspaces/isaac_ros-dev/src/RoArm-M2-S_python/serial_simple_ctrl.py /dev/ttyUSB0
```
You may need
```bash
python -m pip install pyserial
```

# Finetuning yolov8
- Run yolo launch.py as done at top
- Place objects and run 
```bash
python /workspaces/isaac_ros-dev/src/isaac_ros_object_detection/isaac_ros_yolov8/myscripts/collectData2DRGB.py
```
- Label with Roboflow and download for yolov8 format
- Download on PC and cd into the downloaded data folder
- The data folder's structure should look like:
ls data
data.yaml README.dataset.txt README.roboflow.txt train valid
- run
```bash
yolo detect train model=yolov8s.pt data=/whatever/you/named/it/data.yaml epochs=100 imgsz=640 device=0
```
- Visualize results of .pt
yolo val task=detect model=runs/detect/train/weights/best.pt device=0 imgsz=640 data=/whatever/you/named/it/data.yaml
- Then export to .onnx to run on jetson
```bash
yolo export model=runs/detect/train/weights/best.pt format=onnx
```
- scp onnx to jetson
scp aaron@192.168.1.166:~/Downloads/train_SockModel/runs/detect/train2/weights/best.onnx /workspaces/isaac_ros-dev/isaac_ros_assets/models/yolov8

- Go to isaac_ros_yolov8/scripts/isaac_ros_yolov8_visualizer.py and edit like:
names = {
        0: 'clean',
        ...
}

- on jetson
```bash
pip install ultralytics
```

```bash
cd /workspaces/isaac_ros-dev
```

```bash
colcon build --packages-select isaac_ros_yolov8
```

```bash
pip uninstall numpy
```
```bash
pip install "numpy<2"
```

not sure what worked but:
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_CURRENT_PREFIX
cd /workspaces/isaac_ros-dev
rm -rf build install log

colcon build --symlink-install \
  --packages-up-to isaac_ros_yolov8 \
  --cmake-args -DBUILD_TESTING=OFF \
  --allow-overriding isaac_ros_yolov8 isaac_ros_tensor_list_interfaces \
  isaac_ros_common

# Make sure to source both in both yolov8 launch and yolov8 visualizer!
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
# Your overlay should appear BEFORE /opt/ros/humble
echo $AMENT_PREFIX_PATH
# This must print your overlay path, not /opt/ros/humble

Might need to regenerate the engine .plan
```bash
trtexec \
  --onnx=/workspaces/isaac_ros-dev/isaac_ros_assets/models/yolov8/socks.onnx \
  --saveEngine=/workspaces/isaac_ros-dev/isaac_ros_assets/models/yolov8/socks.plan \
  --memPoolSize=workspace:2048
  --verbose
```
