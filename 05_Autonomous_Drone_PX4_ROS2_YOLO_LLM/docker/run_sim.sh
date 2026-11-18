#!/usr/bin/env bash
# Dedicated Simulator Launcher for Autonomous Drone Project 05
set -e

export PX4_GZ_WORLD="drone_obstacle_world"
export PX4_SYS_AUTOSTART=4010
export PX4_SIM_MODEL="x500_mono_cam"
export GZ_CONFIG_PATH="/usr/share/gz${GZ_CONFIG_PATH:+:${GZ_CONFIG_PATH}}"

echo "======================================================================"
echo "🚁 Launching Autonomous Drone Simulation (PX4 SITL + Gazebo Harmonic)"
echo "🗺️ World: drone_obstacle_world.sdf (Human at 4m, SUV at 6m, Cube at 3m)"
echo "📷 Airframe: 4010 (x500 Quadcopter with Forward-Facing Mono Camera)"
echo "======================================================================"

# Ensure world SDF is up-to-date with workspace
cp -u /workspace/docker/drone_obstacle_world.sdf /opt/px4-gazebo/share/gz/worlds/drone_obstacle_world.sdf 2>/dev/null || true

exec /opt/px4-gazebo/bin/px4-gazebo
