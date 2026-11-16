# ✋ SCARA Manipulator Hand Gesture Teleoperation

<div align="center">

[![ROS 2](https://img.shields.io/badge/ROS_2-Lyrical-blue?logo=ros&logoColor=white)](https://docs.ros.org/)
[![MediaPipe](https://img.shields.io/badge/AI_Vision-Google_MediaPipe-orange?logo=google&logoColor=white)](https://developers.google.com/mediapipe)
[![Python](https://img.shields.io/badge/Python-3.14-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![RViz2](https://img.shields.io/badge/Visualization-RViz2-red?logo=ros)](https://docs.ros.org/)
[![Author](https://img.shields.io/badge/Author-Nasim--Sami-black?logo=github)](https://github.com/Nasim-Sami)

**A touchless human-robot interface (HRI) using real-time hand landmark detection with Google MediaPipe to teleoperate a custom 4-DOF SCARA robotic manipulator live in RViz.**

</div>

---

## 📹 Video Demonstration

- 🎬 **Video File:** [`scara_hand_gesture_teleop_demo.webm`](scara_hand_gesture_teleop_demo.webm)
- **Watch in Linux Terminal:**
  ```bash
  vlc scara_hand_gesture_teleop_demo.webm
  ```

---

## 📌 Project Overview & Gesture Architecture

This project implements an intuitive, multi-modal gesture teleoperation system mapping human hand gestures to a 4-DOF SCARA manipulator:

1. **Robot Kinematic Description (`scara_robot.urdf`)**:
   - `joint_1` (Revolute): Shoulder rotation ($[-2.8, +2.8]$ rad).
   - `joint_2` (Revolute): Elbow flexion/extension ($[-2.0, +2.0]$ rad).
   - `joint_3` (Prismatic): Vertical Z-axis elevator shaft ($[-0.15, +0.03]$ m).
   - `joint_4` (Revolute): End-effector / gripper roll ($[-3.14, +3.14]$ rad).

2. **3D Landmark Tracking**: Uses MediaPipe Tasks `HandLandmarker` to extract 21 3D normalized coordinates at 30+ FPS from a webcam feed.

3. **Intelligent Gesture State Machine (`scara_gesture_control.py`)**:
   - ✊ **Fist Gesture (`fist_ratio < 1.08`)**:
     - **Fist Tilted Left/Right**: Calculates wrist-to-knuckle angle ($\theta = \text{atan2}(dx, -dy)$) to command **Joint 4 (Gripper Roll $360^\circ$)**.
     - **Fist Upright**: Vertical hand movement controls **Joint 3 (Z-axis Elevator)**. Arm joints J1 & J2 stay securely locked.
   - 🤏 **Pinch Gesture (`pinch_dist / palm_len < 0.28`)**:
     - Index and thumb pinch triggers a fast **Pick Plunge** action, lowering **Joint 3** to $-0.15$ m while locking other joints.
   - 🖐️ **Open Hand Navigation**:
     - **Dominant Horizontal Motion**: Steers **Joint 1 (Shoulder)** proportionally to $dx$.
     - **Dominant Vertical Motion**: Moves **Joint 2 (Elbow)** proportionally to $-dy$.
     - **Neutral Center Box**: Resting hand inside the $80\times 80$ center box holds arm position.

4. **Exponential Smoothing Filter (Low-Pass Filter)**:
   - Employs an exponential moving average ($j_{t} = \alpha \cdot \text{target} + (1 - \alpha) \cdot j_{t-1}$) with $\alpha = 0.16$ to filter out camera noise and hand jitter, delivering buttery-smooth manipulator motion in RViz.

5. **ROS 2 Pipeline**: Publishes `sensor_msgs/msg/JointState` to `/joint_states`, dynamically rendered in RViz via `robot_state_publisher`.

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
```bash
python3 scara_gesture_control.py
```
Position your hand in front of your webcam:
- Open hand to move shoulder/elbow.
- Make a fist and tilt to rotate gripper, or move fist up/down to raise/lower elevator.
- Pinch thumb and index to perform a pick action!

---

## 📂 File Structure

```text
03_SCARA_Hand_Gesture_Teleop/
├── README.md                            # Comprehensive documentation
├── scara_robot.urdf                     # 4-DOF SCARA manipulator URDF description
├── scara_display.launch.py              # Launch file for Robot State Publisher & RViz
├── scara_gesture_control.py             # Gesture classifier, low-pass filter & JointState publisher
├── hand_test.py                         # Standalone MediaPipe verification test
└── scara_hand_gesture_teleop_demo.webm  # Recorded video demonstration
```

---

## 👤 Author
**Nasim Sami**  
- GitHub: [@Nasim-Sami](https://github.com/Nasim-Sami)
- Portfolio Repository: [ROS2-Mini-Projects](https://github.com/Nasim-Sami/ROS2-Mini-Projects)
