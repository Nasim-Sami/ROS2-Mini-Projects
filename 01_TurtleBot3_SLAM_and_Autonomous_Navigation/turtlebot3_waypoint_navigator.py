#!/usr/bin/env python3
"""
=============================================================================
TurtleBot3 Autonomous Waypoint Navigator
Author: Nasim Sami (https://github.com/Nasim-Sami)
Repository: ROS2-Mini-Projects
Project: 01_TurtleBot3_SLAM_and_Autonomous_Navigation
=============================================================================
Description:
    Autonomous navigation script using Nav2 Simple Commander API (BasicNavigator).
    The robot sequentially visits user-defined waypoints inside the mapped arena,
    verifying arrival status before proceeding to the next goal.
=============================================================================
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def make_pose(navigator, x, y, orientation_w=1.0):
    """
    Construct a stamped pose message in the 'map' frame.
    
    Args:
        navigator: BasicNavigator instance (used for clock timestamping).
        x (float): Target X coordinate in meters.
        y (float): Target Y coordinate in meters.
        orientation_w (float): Quaternion orientation (default 1.0 = forward).
        
    Returns:
        PoseStamped: Formatted pose ready for Nav2.
    """
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.w = float(orientation_w)
    return pose


def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Target waypoints mapped inside the hexagonal arena (X, Y)
    # Coordinates collected using RViz 'Publish Point' (/clicked_point)
    points = [
        (-0.67, -2.12),  # Waypoint 1: Bottom-Left perimeter
        (0.83, -2.00),   # Waypoint 2: Bottom-Right perimeter
        (2.10, -0.53),   # Waypoint 3: Rightmost zone
        (1.64, 1.37),    # Waypoint 4: Top-Right perimeter
        (-0.70, 1.78),   # Waypoint 5: Top-Left perimeter
    ]

    print("=====================================================")
    print("  TurtleBot3 Nav2 Autonomous Waypoint Navigator")
    print("=====================================================")
    print("[INFO] Waiting for Nav2 stack to become active...")
    navigator.waitUntilNav2Active()
    print("[SUCCESS] Nav2 is active! Commencing autonomous patrol...\n")

    for i, (x, y) in enumerate(points, start=1):
        target_pose = make_pose(navigator, x, y)
        print(f"--> [Waypoint {i}/{len(points)}] Dispatching goal: ({x:.2f}, {y:.2f})")
        navigator.goToPose(target_pose)

        # Spin while task is in progress to receive feedback
        while not navigator.isTaskComplete():
            rclpy.spin_once(navigator, timeout_sec=0.5)

        # Evaluate navigation outcome
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"[REACHED] Waypoint {i} arrived successfully! ✓\n")
        elif result == TaskResult.CANCELED:
            print(f"[CANCELED] Navigation to Waypoint {i} was canceled.\n")
        elif result == TaskResult.FAILED:
            print(f"[FAILED] Failed to navigate to Waypoint {i}.\n")
        else:
            print(f"[UNKNOWN] Unexpected status for Waypoint {i}: {result}\n")

    print("=====================================================")
    print("  All waypoints reached successfully! Mission complete.")
    print("=====================================================")

    # NOTE: Keep lifecycleShutdown() commented out if you want to rerun this script
    # repeatedly without restarting the Gazebo/Nav2 simulation launch session.
    # navigator.lifecycleShutdown()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
