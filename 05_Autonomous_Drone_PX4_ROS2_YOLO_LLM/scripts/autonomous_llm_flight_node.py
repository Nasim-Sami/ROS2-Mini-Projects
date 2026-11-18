#!/usr/bin/env python3
"""
Autonomous LLM Flight Node (Master Controller)
-----------------------------------------------
Integrates:
1. Natural Language Intent Parser (Ollama Qwen 2.5 1.5B)
2. Physics-Based Safety Gatekeeper & State Resolver
3. Parametric 3D Trajectory Generator
4. PX4 Offboard Mode & ROS 2 Jazzy Interface

Allows commanding the drone via:
- Interactive console input
- ROS 2 topic: /drone/natural_command (std_msgs/msg/String)
"""

import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition
from std_msgs.msg import String
import subprocess
import threading
import time
import math

from drone_intent_parser import parse_user_command
from drone_safety_validator import DronePhysicalState, validate_and_resolve_mission
from trajectory_generator import generate_timed_trajectory


class AutonomousLLMFlightNode(Node):
    def __init__(self):
        super().__init__('autonomous_llm_flight_node')

        # 1. Disable GCS & RC loss failsafe checks in PX4 SITL
        try:
            subprocess.run(['/opt/px4-gazebo/bin/px4-param', 'set', 'NAV_RCL_ACT', '0'], capture_output=True)
            subprocess.run(['/opt/px4-gazebo/bin/px4-param', 'set', 'NAV_DLL_ACT', '0'], capture_output=True)
            self.get_logger().info("✅ PX4 Failsafe parameters updated: NAV_RCL_ACT=0, NAV_DLL_ACT=0")
        except Exception as e:
            self.get_logger().warn(f"Param set note: {e}")

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
        self.command_sub = self.create_subscription(
            String, '/drone/natural_command', self.natural_command_callback, 10
        )

        # Vehicle State
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.current_ned_x = 0.0
        self.current_ned_y = 0.0
        self.current_ned_z = 0.0
        self.current_yaw = 0.0
        self.is_airborne = False

        # Flight Setpoint Buffer
        self.active_trajectory = []
        self.trajectory_index = 0
        self.current_target = [0.0, 0.0, 0.0]  # [x, y, z] in NED
        self.current_target_yaw = 0.0
        self.is_executing = False

        # Heartbeat timer at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("🤖 Autonomous LLM Flight Node Ready.")
        self.get_logger().info("👉 Send commands via topic: /drone/natural_command")
        self.get_logger().info("👉 Or type commands directly in this terminal!")

        # Start interactive CLI thread
        self.cli_thread = threading.Thread(target=self.cli_loop, daemon=True)
        self.cli_thread.start()

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.arming_state = msg.arming_state
        self.nav_state = msg.nav_state

    def local_pos_callback(self, msg: VehicleLocalPosition):
        self.current_ned_x = msg.x
        self.current_ned_y = msg.y
        self.current_ned_z = msg.z
        self.current_yaw = msg.heading
        # If altitude > 0.3m, vehicle is airborne
        self.is_airborne = (-self.current_ned_z > 0.3)

    def natural_command_callback(self, msg: String):
        self.process_command(msg.data)

    def process_command(self, user_command: str):
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"🗣️ Incoming Natural Command: \"{user_command}\"")

        # Stage 1: LLM Parsing
        self.get_logger().info("🧠 Translating via Qwen 2.5 1.5B LLM...")
        parsed = parse_user_command(user_command)
        self.get_logger().info(f"📋 LLM Mission Goal: {parsed.get('mission_goal')}")

        # Stage 2 & 3: State-Aware Physics Safety Gatekeeper
        initial_alt = -self.current_ned_z
        state = DronePhysicalState(
            is_airborne=self.is_airborne,
            x=self.current_ned_x,
            y=self.current_ned_y,
            z=max(0.0, initial_alt),
            yaw=self.current_yaw
        )

        is_safe, resolved_steps, simulated_waypoints, logs = validate_and_resolve_mission(parsed, state)

        for l in logs:
            self.get_logger().info(f"   {l}")

        if not is_safe:
            self.get_logger().error("🛑 SAFETY GATEKEEPER REJECTED FLIGHT PLAN!")
            self.get_logger().error(f"Reason: {logs[-1] if logs else 'Violated physical safety constraints'}")
            return

        self.get_logger().info("🛡️ Safety Gatekeeper: [ APPROVED ✅ ] Physical flight envelope safe.")

        # Stage 4: Trajectory Generation
        self.get_logger().info("📐 Computing 3D Parametric Trajectory...")
        trajectory = generate_timed_trajectory(
            resolved_steps,
            current_x=self.current_ned_x,
            current_y=self.current_ned_y,
            current_z=self.current_ned_z,
            current_yaw=self.current_yaw,
            rate_hz=10.0
        )

        # Check for landing action in final step
        has_land = any(s.get("action") == "land" for s in resolved_steps)

        # Arm and engage offboard mode
        self.engage_offboard_mode()
        self.arm()

        # Load trajectory
        self.active_trajectory = trajectory
        self.trajectory_index = 0
        self.has_land_at_end = has_land
        self.is_executing = True
        self.get_logger().info(f"🚀 Mission Launched! Executing {len(trajectory)} setpoints (~{len(trajectory)/10.0:.1f}s)...")

    def timer_callback(self):
        # Always publish heartbeat
        self.publish_offboard_control_mode()

        if self.is_executing and self.trajectory_index < len(self.active_trajectory):
            pt = self.active_trajectory[self.trajectory_index]
            self.current_target = [pt[0], pt[1], pt[2]]
            self.current_target_yaw = pt[3]
            self.trajectory_index += 1

            if self.trajectory_index % 20 == 0:
                alt = -self.current_ned_z
                pct = (self.trajectory_index / len(self.active_trajectory)) * 100.0
                self.get_logger().info(f"✈️ Progress: {pct:.0f}% | Altitude: {alt:.2f}m | Target: {-self.current_target[2]:.2f}m")

        elif self.is_executing and self.trajectory_index >= len(self.active_trajectory):
            self.is_executing = False
            self.get_logger().info("🏁 Trajectory Completed.")
            if getattr(self, 'has_land_at_end', False):
                self.get_logger().info("🛬 Autonomous Landing Initiated...")
                self.land()
            else:
                self.get_logger().info("⚓ Holding Position (Hovering).")

        self.publish_trajectory_setpoint()

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
        msg.position = [float(x) for x in self.current_target]
        nan = float('nan')
        msg.velocity = [nan, nan, nan]
        msg.acceleration = [nan, nan, nan]
        msg.jerk = [nan, nan, nan]
        msg.yaw = float(self.current_target_yaw)
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
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def cli_loop(self):
        time.sleep(2.0)
        while rclpy.ok():
            try:
                # Wait while current trajectory is actively executing
                while self.is_executing and rclpy.ok():
                    time.sleep(0.5)

                # Give a small pause for status to settle
                time.sleep(0.5)

                alt = -self.current_ned_z
                status_str = f"AIRBORNE (Altitude: {alt:.2f}m - Hovering)" if self.is_airborne else "GROUNDED (Ready)"
                print("\n" + "=" * 60)
                print(f"🚁 Drone State: {status_str}")
                cmd = input("👉 Enter Drone Natural Language Command (e.g. 'land', 'dance', 'move forward 2m'): ").strip()
                if cmd:
                    if cmd.lower() in ["exit", "quit"]:
                        break
                    self.process_command(cmd)
            except EOFError:
                break
            except Exception as e:
                print(f"CLI error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousLLMFlightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
