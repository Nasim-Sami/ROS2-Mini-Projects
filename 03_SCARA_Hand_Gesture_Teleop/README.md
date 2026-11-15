# ✋ SCARA Manipulator Hand Gesture Teleoperation

[![ROS 2](https://img.shields.io/badge/ROS_2-Lyrical-blue?logo=ros&logoColor=white)](https://docs.ros.org/)
[![MediaPipe](https://img.shields.io/badge/AI_Vision-Google_MediaPipe-orange?logo=google&logoColor=white)](https://developers.google.com/mediapipe)
[![RViz2](https://img.shields.io/badge/Visualization-RViz2-red?logo=ros)](https://docs.ros.org/)
[![Author](https://img.shields.io/badge/Author-Nasim--Samiu-black?logo=github)](https://github.com/Nasim-Sami)

> Contactless, intuitive robot teleoperation: real-time AI hand landmark detection using Google MediaPipe mapped into joint states (`sensor_msgs/msg/JointState`) to control a 4-DOF SCARA robotic manipulator live in RViz.

---

## 📹 Video Demonstration

- 🎬 **Video File:** [`scara_hand_gesture_teleop_demo.webm`](scara_hand_gesture_teleop_demo.webm)
- **Watch in Linux Terminal:**
  ```bash
  vlc scara_hand_gesture_teleop_demo.webm
  ```

---

## 📌 Project Overview

This project implements a touchless human-robot interface (HRI) for a SCARA robotic manipulator:
1. **Kinematic Modeling (`scara_robot.urdf`)**: A 4-DOF SCARA manipulator defined with base link, revolute shoulder joint (`joint_1`), revolute elbow joint (`joint_2`), prismatic vertical z-axis joint (`joint_3`), and revolute wrist roll joint (`joint_4`).
2. **AI Hand Tracking**: Uses MediaPipe Tasks `HandLandmarker` to extract 21 3D coordinates representing finger joints and palm orientation.
3. **Joint Angle Mapping**:
   - Palm horizontal position / wrist angle $\rightarrow$ Shoulder joint (`joint_1`).
   - Elbow bend / finger spread $\rightarrow$ Elbow joint (`joint_2`).
   - Hand vertical elevation $\rightarrow$ Prismatic Z-axis joint (`joint_3`).
   - Index-thumb pinch / pitch $\rightarrow$ End-effector rotation (`joint_4`).
4. **ROS 2 Integration**: Publishes computed joint positions to `/joint_states` at 30+ FPS, visualized dynamically in RViz via `robot_state_publisher`.

---

## 🚀 How to Run

### 1. Test Hand Landmark Detection
```bash
python3 hand_test.py
```

### 2. Launch SCARA Model in RViz
```bash
ros2 launch scara_display.launch.py
```

### 3. Run Gesture Teleoperation Node
In a separate terminal, activate the environment and run:
```bash
python3 scara_gesture_control.py
```
Move your hand in front of your webcam to control the SCARA robot in real time!

---

## 📂 File Structure

```text
03_SCARA_Hand_Gesture_Teleop/
├── README.md                            # Documentation
├── scara_robot.urdf                     # 4-DOF SCARA manipulator URDF description
├── scara_display.launch.py              # Launch file for Robot State Publisher & RViz
├── scara_gesture_control.py             # MediaPipe gesture tracking & JointState publisher
├── hand_test.py                         # Standalone MediaPipe hand tracking test
└── scara_hand_gesture_teleop_demo.webm  # Recorded video demonstration
```

---

## 👤 Author
**Nasim Samiu**  
- GitHub: [@Nasim-Sami](https://github.com/Nasim-Sami)
- Portfolio Repository: [ROS2-Mini-Projects](https://github.com/Nasim-Sami/ROS2-Mini-Projects)
