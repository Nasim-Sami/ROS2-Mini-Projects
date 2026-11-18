#!/usr/bin/env bash
# Autonomous Drone Vision & Camera Bridge Launcher
set -e

source /opt/ros/jazzy/setup.bash
source /opt/px4_ros2/install/setup.bash

WORLD_NAME="${PX4_GZ_WORLD:-drone_obstacle_world}"
MODEL_NAME="${PX4_SIM_MODEL:-x500_mono_cam}_0"

echo "======================================================================"
echo "👁️ Starting Drone Onboard Camera Bridge & YOLOv8 Vision Perception"
echo "📷 Bridging: /world/${WORLD_NAME}/model/${MODEL_NAME}/... -> /camera/image_raw"
echo "🧠 Neural Network: Ultralytics YOLOv8 Nano Real-time Detector"
echo "======================================================================"

# Start ROS 2 Gazebo Parameter Bridge in background
ros2 run ros_gz_bridge parameter_bridge "/world/${WORLD_NAME}/model/${MODEL_NAME}/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image" --ros-args -r "/world/${WORLD_NAME}/model/${MODEL_NAME}/link/camera_link/sensor/camera/image:=/camera/image_raw" &

BRIDGE_PID=$!
sleep 2

# Cleanup on exit
trap "kill -9 $BRIDGE_PID 2>/dev/null || true" EXIT

# Start YOLOv8 Vision Detector
python3 /workspace/scripts/drone_vision_detector.py
