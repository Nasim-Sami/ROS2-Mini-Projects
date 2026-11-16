# 🎯 TurtleBot3 Vision-Based Color Teleoperation (Virtual Air-Joystick)

<div align="center">

[![ROS 2](https://img.shields.io/badge/ROS_2-Lyrical-blue?logo=ros&logoColor=white)](https://docs.ros.org/)
[![OpenCV](https://img.shields.io/badge/Computer_Vision-OpenCV-brightgreen?logo=opencv&logoColor=white)](https://opencv.org/)
[![Python](https://img.shields.io/badge/Python-3.14-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Author](https://img.shields.io/badge/Author-Nasim--Sami-black?logo=github)](https://github.com/Nasim-Sami)

**A vision-based virtual air-joystick using real-time HSV color segmentation with OpenCV. The node tracks a designated colored object via laptop webcam and maps its 2D coordinates into directional velocity commands (`geometry_msgs/msg/TwistStamped`) with a deadzone threshold.**

</div>

---

## 📹 Video Demonstration

- 🎬 **Video File:** [`turtlebot3_color_tracking_demo.webm`](turtlebot3_color_tracking_demo.webm)
- **Watch in Linux Terminal:**
  ```bash
  vlc turtlebot3_color_tracking_demo.webm
  ```

---

## 📌 Project Overview & How It Works

Instead of using a physical joystick or keyboard, this project transforms an ordinary webcam and colored object into a **touchless virtual air-joystick**:

1. **Camera Stream Capture**: Captures real-time webcam frames using OpenCV and mirrors the feed (`cv2.flip(frame, 1)`) for intuitive natural hand interaction.
2. **HSV Color Segmentation & Noise Reduction**: Converts frames from BGR to HSV color space, filters the target color via calibrated thresholds, and applies morphological erosion and dilation to eliminate pixel noise.
3. **Centroid Extraction**: Calculates spatial image moments ($M_{00}, M_{10}, M_{01}$) on the largest contour to track the exact center coordinates $(C_x, C_y)$ of the colored object.
4. **Virtual Air-Joystick with Deadzone Logic**:
   - **Center Deadzone Box (100×100 px)**: Placed in the dead center of the screen as a neutral holding area. When the colored object rests inside this box, the node commands `lin_x = 0.0, ang_z = 0.0` (`In Deadzone (Holding)`), keeping the robot stationary.
   - **Directional Steering & Drive**: Moving the colored object outside the deadzone triggers directional motion at configured speeds:
     - ⬆️ **Up (`cy < box_y1`)**: Drives forward (`linear.x = +max_lin`).
     - ⬇️ **Down (`cy > box_y2`)**: Drives backward (`linear.x = -max_lin`).
     - ⬅️ **Left (`cx < box_x1`)**: Turns left (`angular.z = +max_ang`).
     - ➡️ **Right (`cx > box_x2`)**: Turns right (`angular.z = -max_ang`).
   - **Bang-Bang Controller with Deadband**: By utilizing a deadband threshold rather than continuous linear scaling, minor involuntary hand tremors are filtered out, ensuring stable and predictable robot teleoperation.
5. **ROS 2 Velocity Publishing**: Publishes velocity commands to `/cmd_vel` supporting both standard `geometry_msgs/msg/Twist` and modern `geometry_msgs/msg/TwistStamped` (with timestamp).

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

### 3. Launch TurtleBot3 Simulation
In a separate terminal, launch the TurtleBot3 simulation in Gazebo:
```bash
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```

### 4. Run Color Teleoperation Node
```bash
# For TwistStamped (Modern ROS 2 simulation default):
python3 color_control.py --topic /cmd_vel --stamped --color red

# Optional arguments:
# --color [red|green|blue]
# --max-lin 1.5 --max-ang 1.5
```

Hold your colored object in front of your webcam. Move it Up, Down, Left, or Right outside the white box to drive the robot, and return it to the center box to stop!

---

## 📂 File Structure

```text
02_TurtleBot3_Vision_Color_Tracking/
├── README.md                           # Comprehensive documentation
├── color_control.py                   # Main ROS 2 virtual air-joystick teleop node
├── hsv_tune.py                        # Interactive HSV calibration trackbar tool
├── cam_test.py                        # Webcam test utility
└── turtlebot3_color_tracking_demo.webm # Recorded video demonstration
```

---

## 👤 Author
**Nasim Sami**  
- GitHub: [@Nasim-Sami](https://github.com/Nasim-Sami)
- Portfolio Repository: [ROS2-Mini-Projects](https://github.com/Nasim-Sami/ROS2-Mini-Projects)
