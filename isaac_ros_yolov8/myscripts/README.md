# CleaningRobot Frozen Deployment Scripts

**Single-provenance YOLOv8 vision system for Jetson Orin Nano**

This directory contains all scripts needed for frozen container deployment. NO Isaac ROS apt packages are installed - everything is built from source for single provenance.

## Quick Start

```bash
# 1. Build container (first time or after changes)
./build.sh

# 2. Deploy and run
./deploy.sh

# Inside container - verify single provenance
ros2 pkg list | grep isaac_ros_examples  # Should return NOTHING
ros2 pkg list | grep isaac_ros           # Should list only your source-built packages

# Run production pipeline
ros2 launch cleaningrobot_bringup vision_yolov8.launch.py \
    model_file_path:=/workspaces/isaac_ros-dev/isaac_ros_assets/models/yolov8/socks2.onnx \
    engine_file_path:=/workspaces/isaac_ros-dev/isaac_ros_assets/models/yolov8/socks2.plan \
    confidence_threshold:=0.83
```

## Files in This Directory

### Core Deployment Scripts
- **`build.sh`** - Builds frozen container from Dockerfile.production.v2
- **`deploy.sh`** - Runs container with Jetson device mounts
- **`restore-base-image.sh`** - Restores frozen base from tarball

### Container Definition
- **`Dockerfile.production.v2`** - Production container (NO apt Isaac ROS packages)
- **`entrypoint.sh`** - Sources ROS environments on startup

### Configuration
- **`requirements.txt`** - Pinned Python dependencies
- **`IMAGE_MANIFEST.txt`** - Build metadata (JetPack, kernel versions)

### Documentation
- **`README.md`** - This file
- **`DEPLOYMENT_FILES_MANIFEST.txt`** - Complete file listing with contents

### Helper Scripts
- **`simple_combine.py`** - Generates DEPLOYMENT_FILES_MANIFEST.txt

### Application Scripts (not in manifest)
These are your robot application scripts, not part of the frozen deployment infrastructure:
- `collectData2DRGB.py` - Data collection utility
- `run_robot.py` - Robot control script
- `simple_robot_pick_test.py` - Pick test
- `yolov8_3d_detector.py` / `yolov8_3d_detector2.py` - 3D detection nodes
- `web_viewer_server_3d.py` - 3D visualization server
- `viewer.html` - Web viewer frontend

## Launch Files

Launch files are in `../../../cleaningrobot_bringup/launch/`:

1. **`vision_yolov8.launch.py`** - **PRODUCTION:** Full pipeline
2. **`realsense_only.launch.py`** - TEST 1: Camera only
3. **`image_proc_only.launch.py`** - TEST 2: Image preprocessing
4. **`tensor_rt_only.launch.py`** - TEST 3: Inference only

See `../../../cleaningrobot_bringup/README.md` for detailed pipeline documentation.

## Single-Provenance Principle

**CRITICAL:** This deployment uses ONLY source-built Isaac ROS packages.

✅ **DO:**
- Build all Isaac ROS packages from source in `/workspaces/isaac_ros-dev`
- Use launch files from `cleaningrobot_bringup`
- Verify `ros2 pkg list | grep isaac_ros_examples` returns nothing

❌ **DO NOT:**
- Install `ros-humble-isaac-ros-*` packages via apt
- Use `isaac_ros_examples` launch files
- Mix apt and source-built Isaac ROS packages

**Why?** Mixed package sources cause NITROS ABI conflicts and GXF tensor stream failures.

## Frozen Base Image

The base image is stored separately (too large for git):
- **File:** `isaac_ros_dev-aarch64-frozen.tar.gz` (13GB)
- **Location:** Repository root (5 directories up)
- **Restore:** `./restore-base-image.sh` (if missing)

## Build Information

- **Base:** `isaac_ros_dev-aarch64:r36.4.4_ir3.2_frozen`
- **Platform:** Jetson Orin Nano, L4T R36.4.4
- **Isaac ROS:** 3.2.0
- **ROS:** Humble
- **CUDA:** 12.6, TensorRT 10.3, VPI 3.2.4

## Troubleshooting

### "Package isaac_ros_examples not found"
✅ **GOOD!** This is expected. We don't use isaac_ros_examples.

### NITROS negotiation failures
Check provenance:
```bash
ros2 pkg prefix isaac_ros_tensor_rt
# Should be: /workspaces/isaac_ros-dev/install/isaac_ros_tensor_rt
# NOT: /opt/ros/humble/...
```

### GXF tensor stream creation failed
1. Check power mode: `sudo nvpmodel -q`
2. Verify single provenance (no apt Isaac ROS packages)
3. Rebuild workspace: `colcon build --symlink-install`

## Power Mode

Target: **15W mode** (nvpmodel mode 1)
```bash
sudo nvpmodel -m 1  # Set 15W mode
sudo nvpmodel -q    # Verify
```

If issues persist, temporarily try MAXN for debugging:
```bash
sudo nvpmodel -m 0  # MAXN (debug only)
```
