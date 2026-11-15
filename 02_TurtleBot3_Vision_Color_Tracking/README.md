# 🎯 TurtleBot3 Vision-Based Color Tracking & Teleoperation

[![ROS 2](https://img.shields.io/badge/ROS_2-Lyrical-blue?logo=ros&logoColor=white)](https://docs.ros.org/)
[![OpenCV](https://img.shields.io/badge/Computer_Vision-OpenCV-brightgreen?logo=opencv&logoColor=white)](https://opencv.org/)
[![Author](https://img.shields.io/badge/Author-Nasim--Samiu-black?logo=github)](https://github.com/Nasim-Sami)

> Vision-based mobile robot control using real-time HSV color segmentation with OpenCV. The node tracks a designated colored object via webcam/camera feed and translates its spatial offset into proportional angular and linear velocity commands (`geometry_msgs/msg/TwistStamped`).

---

## 📹 Video Demonstration

- 🎬 **Video File:** [`turtlebot3_color_tracking_demo.webm`](turtlebot3_color_tracking_demo.webm)
- **Watch in Linux Terminal:**
  ```bash
  vlc turtlebot3_color_tracking_demo.webm
  ```

---

## 📌 Project Overview

This project bridges classical Computer Vision with ROS 2 mobile robotics control:
1. **Camera Stream Capture**: Captures real-time frames via OpenCV VideoCapture.
2. **HSV Color Filtering**: Converts frames to HSV color space, applying lower and upper HSV thresholds tuned for the target object (e.g., green, blue, or red).
3. **Contour Detection & Centroid Extraction**: Computes spatial moments to find the center $(C_x, C_y)$ and radius of the largest colored contour.
4. **Proportional Closed-Loop Controller**:
   - **Angular Steering**: Proportional to horizontal offset from frame center $(C_x - \text{width}/2)$.
   - **Linear Drive**: Proportional to target distance (estimated from contour area / radius).
5. **TwistStamped Output**: Publishes to `/cmd_vel` supporting both standard `Twist` and modern `TwistStamped` message types.

---

## 🚀 How to Run

### 1. Test Camera Feed
```bash
python3 cam_test.py
```

### 2. Calibrate Target HSV Color Values
Run the interactive HSV trackbar tuner to find optimal threshold values for your lighting and object:
```bash
python3 hsv_tune.py
```

### 3. Launch TurtleBot3 Simulation or Hardware
In a separate terminal, launch the TurtleBot3 simulation or physical robot:
```bash
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```

### 4. Run Color Tracking Teleoperation Node
```bash
python3 color_control.py
```

---

## 📂 File Structure

```text
02_TurtleBot3_Vision_Color_Tracking/
├── README.md                           # Documentation
├── color_control.py                   # Main ROS 2 color tracking teleop node
├── hsv_tune.py                        # Interactive HSV calibration trackbar tool
├── cam_test.py                        # Webcam test utility
└── turtlebot3_color_tracking_demo.webm # Recorded video demonstration
```

---

## 👤 Author
**Nasim Samiu**  
- GitHub: [@Nasim-Sami](https://github.com/Nasim-Sami)
- Portfolio Repository: [ROS2-Mini-Projects](https://github.com/Nasim-Sami/ROS2-Mini-Projects)
