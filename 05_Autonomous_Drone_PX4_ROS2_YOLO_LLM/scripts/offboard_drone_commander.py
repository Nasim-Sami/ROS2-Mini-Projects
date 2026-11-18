#!/usr/bin/env python3
"""
Autonomous Drone Commander Node
Platform: ROS 2 Jazzy + PX4 v1.15 SITL + Gazebo Harmonic
Author: Nasim Sami
Portfolio Project 05: Autonomous Aerial Robotics, Vision & LLM Control
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition
import subprocess
import math


class OffboardDroneCommander(Node):
    def __init__(self):
        super().__init__('offboard_drone_commander')

        # 1. Disable GCS & RC loss failsafe checks in PX4 SITL so drone arms via ROS 2
        try:
            subprocess.run(['/opt/px4-gazebo/bin/px4-param', 'set', 'NAV_RCL_ACT', '0'], capture_output=True)
            subprocess.run(['/opt/px4-gazebo/bin/px4-param', 'set', 'NAV_DLL_ACT', '0'], capture_output=True)
            self.get_logger().info("✅ PX4 Failsafe parameters updated: NAV_RCL_ACT=0, NAV_DLL_ACT=0")
        except Exception as e:
            self.get_logger().warn(f"Param set warning: {e}")

        # QoS profile matching PX4 Micro-XRCE-DDS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile
        )

        # Subscribers
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile
        )
        self.vehicle_local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.local_pos_callback, qos_profile
        )

        # State variables
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.current_z = 0.0
        self.setpoint_counter = 0

        # Target setpoint in NED frame (North, East, Down).
        # In NED, -2.5m Z is 2.5m ABOVE ground!
        self.target_position = [0.0, 0.0, -2.5]
        self.target_yaw = 0.0

        # Heartbeat timer at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("🚁 Offboard Drone Commander Initialized. Waiting for connection...")

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.arming_state = msg.arming_state
        self.nav_state = msg.nav_state

    def local_pos_callback(self, msg: VehicleLocalPosition):
        self.current_z = msg.z
        # Log altitude every 2 seconds
        if self.setpoint_counter % 20 == 0:
            altitude_agl = -self.current_z
            self.get_logger().info(f"📊 Live Telemetry -> Altitude: {altitude_agl:.2f}m | Target: {-self.target_position[2]:.2f}m")

    def timer_callback(self):
        # 1. Publish Offboard Control Mode & Trajectory Setpoint (PX4 Heartbeat)
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        # 2. Engage Offboard & Arm vehicle once heartbeat stream is established
        if self.setpoint_counter == 10:
            self.get_logger().info("📡 Engaging Offboard Flight Mode...")
            self.engage_offboard_mode()
            self.get_logger().info("⚡ Arming vehicle motors...")
            self.arm()

        # Keep arming if not yet armed during initial 3 seconds
        if self.setpoint_counter < 30 and self.setpoint_counter > 10:
            if self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self.arm()

        # 3. Autonomous Flight Mission:
        # Ticks 0-120 (12s): Takeoff & Hover at 2.5m
        # Ticks 120-220 (10s): Translate Forward 3.0m (North)
        # Ticks 220-300 (8s): Yaw Rotation +90 deg
        # Ticks 300+: Safe Landing
        if self.setpoint_counter == 120:
            self.get_logger().info("🎯 Waypoint 1: Moving Forward 3.0 meters (North)...")
            self.target_position = [3.0, 0.0, -2.5]

        elif self.setpoint_counter == 220:
            self.get_logger().info("🔄 Waypoint 2: Rotating Yaw +90 degrees...")
            self.target_yaw = 1.5708  # 90 degrees

        elif self.setpoint_counter == 300:
            self.get_logger().info("🛬 Mission Complete: Commanding Safe Landing...")
            self.land()

        self.setpoint_counter += 1

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [float(x) for x in self.target_position]
        # Crucial for PX4: Uncontrolled dimensions MUST be NaN
        nan = float('nan')
        msg.velocity = [nan, nan, nan]
        msg.acceleration = [nan, nan, nan]
        msg.jerk = [nan, nan, nan]
        msg.yaw = float(self.target_yaw)
        msg.yawspeed = nan
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardDroneCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down commander node...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
