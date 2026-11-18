#!/usr/bin/env bash
# Build script for Autonomous Drone Docker Image (Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic)
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_NAME="drone_jazzy_sim:latest"

echo "=========================================================="
echo "🚀 Building Docker Image: ${IMAGE_NAME}"
echo "=========================================================="

docker build -t "${IMAGE_NAME}" -f "${SCRIPT_DIR}/Dockerfile" "${SCRIPT_DIR}"

echo "=========================================================="
echo "✅ Build completed successfully! Tagged as: ${IMAGE_NAME}"
echo "=========================================================="
