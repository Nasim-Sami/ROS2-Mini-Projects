# 🦾 Universal Robots UR3 Manipulation & OMPL Obstacle Avoidance

[![ROS 2](https://img.shields.io/badge/ROS_2-Lyrical-blue?logo=ros&logoColor=white)](https://docs.ros.org/)
[![MoveIt 2](https://img.shields.io/badge/Motion_Planning-MoveIt_2-brightgreen?logo=ros)](https://moveit.picknik.ai/)
[![OMPL](https://img.shields.io/badge/Planner-OMPL-orange)](https://ompl.kavrakilab.org/)
[![Gazebo](https://img.shields.io/badge/Simulation-Gazebo-blueviolet)](https://gazebosim.org/)
[![Author](https://img.shields.io/badge/Author-Nasim--Sami-black?logo=github)](https://github.com/Nasim-Sami)

> Industrial 6-DOF manipulator trajectory planning and autonomous execution using Universal Robots UR3 in Gazebo simulation, MoveIt 2, `ros2_control` Joint Trajectory Controllers, and OMPL sampling-based obstacle collision avoidance.

---

## 📹 Demonstrations & Architecture

- 🌐 **Interactive Architecture Guide:** [`ros2_ur3_ompl_architecture.html`](ros2_ur3_ompl_architecture.html) *(Open in any browser for an interactive SVG topology & step-by-step lifecycle flow)*
- 🎬 **Obstacle Avoidance Video:** [`ur3_obstacle_avoidance_ompl_demo.webm`](ur3_obstacle_avoidance_ompl_demo.webm)
- 🎬 **Pick & Place Trajectory:** [`ur3_pick_and_place_demo.webm`](ur3_pick_and_place_demo.webm)
- **Watch in Linux Terminal:**
  ```bash
  vlc ur3_obstacle_avoidance_ompl_demo.webm
  ```

---

## 📌 Project Overview

This project showcases robotic arm motion planning and execution across complex kinematic spaces:
1. **Simulation & Controllers**: UR3 industrial robot simulated inside Gazebo with `ros2_control` `JointTrajectoryController`.
2. **Trajectory Interpolation**: Sends smoothly interpolated spline waypoints to the action server `/joint_trajectory_controller/follow_joint_trajectory`.
3. **MoveIt 2 Integration**: Integrates kinematics solvers (KDL / TRAC-IK) and collision scene monitoring.
4. **OMPL Collision-Free Motion Planning**: Plans collision-free paths around static green/red obstacle boxes placed between the pick and drop locations using sampling-based planners (RRTConnect, BKPIECE).

---

## 🚀 How to Run

### 1. Launch UR3 Simulation & Controller
```bash
ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur3 initial_joint_controller:=joint_trajectory_controller
```

### 2. Launch MoveIt 2 Motion Planning Framework
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3 use_sim_time:=true
```

### 3. Spawn Obstacle in Gazebo
```bash
ros2 run ros_gz_sim create -world empty -name gazebo_obstacle \
  -string '<sdf version="1.6"><model name="gazebo_obstacle"><static>true</static><link name="link"><visual name="visual"><geometry><box><size>0.12 0.20 0.25</size></box></geometry><material><ambient>0.2 0.8 0.2 1</ambient><diffuse>0.2 0.8 0.2 1</diffuse></material></visual><collision name="collision"><geometry><box><size>0.12 0.20 0.25</size></box></geometry></collision></link></model></sdf>' \
  -x 0.40 -y 0.0 -z 0.125
```

### 4. Execute Collision Avoidance Script
```bash
python3 ur3_pick_and_place_obstacle.py
```

---

## 📂 File Structure

```text
04_UR3_Pick_and_Place_Obstacle_Avoidance/
├── README.md                            # Documentation
├── ros2_ur3_ompl_architecture.html      # Interactive End-to-End Architecture & Flow Guide
├── ur3_pick_and_place.py                # Direct JointTrajectory action client script
├── ur3_moveit_autonomous.py             # MoveIt 2 autonomous planning script
├── ur3_pick_and_place_obstacle.py       # OMPL collision avoidance around obstacle
├── ur3_obstacle_avoidance_ompl_demo.webm# OMPL collision avoidance demo video
└── ur3_pick_and_place_demo.webm         # Pick and place trajectory demo video
```

---

## 👤 Author
**Nasim Sami**  
- GitHub: [@Nasim-Sami](https://github.com/Nasim-Sami)
- Portfolio Repository: [ROS2-Mini-Projects](https://github.com/Nasim-Sami/ROS2-Mini-Projects)
