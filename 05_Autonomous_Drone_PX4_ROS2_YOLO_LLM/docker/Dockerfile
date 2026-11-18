# ==============================================================================
# Autonomous Drone Simulation Dockerfile
# Base: Official Dronecode PX4 SITL + ROS 2 Jazzy + Gazebo Harmonic
# OS: Ubuntu 24.04 LTS (Noble Numbat)
# Robotics: ROS 2 Jazzy, Gazebo Harmonic, PX4 SITL, Micro-XRCE-DDS-Agent
# Perception: YOLOv8 (Ultralytics) + OpenCV
# LLM Integration: Requests / REST API to Host Ollama
# ==============================================================================

FROM px4io/px4-sitl-gazebo-ros2:latest

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# 1. Install AI & Vision system dependencies
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-dev \
    tmux \
    mesa-utils \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    && rm -rf /var/lib/apt/lists/*

# 2. Install YOLOv8, OpenCV, Requests with --ignore-installed to prevent Debian package conflicts
RUN pip3 install --no-cache-dir --break-system-packages --ignore-installed \
    "setuptools<80" \
    ultralytics \
    opencv-python-headless \
    requests \
    numpy

# 3. Automatic ROS 2 & PX4 environment sourcing for all bash shells
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc && \
    echo "source /opt/px4_ros2/install/setup.bash" >> /etc/bash.bashrc

# 4. Set working directory to mounted workspace
WORKDIR /workspace

CMD ["/bin/bash"]
