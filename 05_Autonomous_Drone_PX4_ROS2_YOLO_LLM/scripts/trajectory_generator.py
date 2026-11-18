#!/usr/bin/env python3
"""
Parametric Trajectory Generator (Stage 4)
-----------------------------------------
Translates validated Universal Motion Primitives into time-parameterized 3D setpoints
for PX4 Offboard Control (NED frame: North, East, Down).
Generates smooth paths for:
- Takeoff, Land, Hover
- Linear Translations (Forward, Backward, Left, Right, Up, Down)
- In-place Yaw Rotations
- 3D Continuous Helical Spirals
- Circular Orbits
- Zigzag Search Sweeps
- Celebratory Aerial Dance
"""

import math
from typing import List, Dict, Any, Tuple

class TrajectoryWaypoint:
    def __init__(self, x: float, y: float, z: float, yaw: float, duration_s: float):
        self.x = x      # North (meters)
        self.y = y      # East (meters)
        self.z = z      # Down in NED (negative is UP!)
        self.yaw = yaw  # Yaw angle in radians
        self.duration_s = duration_s

def generate_timed_trajectory(
    resolved_steps: List[Dict[str, Any]],
    current_x: float = 0.0,
    current_y: float = 0.0,
    current_z: float = 0.0,  # in NED (e.g. -2.5 is 2.5m altitude)
    current_yaw: float = 0.0,
    rate_hz: float = 10.0
) -> List[Tuple[float, float, float, float]]:
    """
    Returns a list of 10Hz setpoints: [(x, y, z, yaw)] in PX4 NED frame.
    """
    dt = 1.0 / rate_hz
    stream_setpoints: List[Tuple[float, float, float, float]] = []

    cur_x = current_x
    cur_y = current_y
    cur_z = current_z
    cur_yaw = current_yaw

    for step in resolved_steps:
        action = step.get("action")
        speed = step.get("speed_mps", 1.0) or 1.0

        if action == "takeoff":
            target_alt = step.get("altitude_m", 2.5)
            target_ned_z = -abs(target_alt)  # In NED, negative is UP
            dist = abs(target_ned_z - cur_z)
            duration = max(3.0, dist / speed)
            num_ticks = int(duration * rate_hz)

            for i in range(num_ticks):
                alpha = (i + 1) / num_ticks
                # Smooth cosine interpolation (S-curve)
                smooth_alpha = 0.5 * (1 - math.cos(alpha * math.pi))
                z = cur_z + (target_ned_z - cur_z) * smooth_alpha
                stream_setpoints.append((cur_x, cur_y, z, cur_yaw))

            cur_z = target_ned_z

        elif action == "hover":
            duration = step.get("duration_s", 3.0) or 3.0
            num_ticks = int(duration * rate_hz)
            for _ in range(num_ticks):
                stream_setpoints.append((cur_x, cur_y, cur_z, cur_yaw))

        elif action == "rotate":
            angle_deg = step.get("angle_deg", 90.0) or 90.0
            direction = step.get("direction", "clockwise")
            delta_rad = math.radians(angle_deg) * (1 if direction == "clockwise" else -1)
            target_yaw = cur_yaw + delta_rad
            # Rotate at ~45 deg/sec
            duration = max(2.0, math.radians(abs(angle_deg)) / 0.8)
            num_ticks = int(duration * rate_hz)

            for i in range(num_ticks):
                alpha = (i + 1) / num_ticks
                smooth_alpha = 0.5 * (1 - math.cos(alpha * math.pi))
                yaw = cur_yaw + (target_yaw - cur_yaw) * smooth_alpha
                stream_setpoints.append((cur_x, cur_y, cur_z, yaw))

            cur_yaw = target_yaw

        elif action == "move":
            direction = step.get("direction", "forward")
            dist = step.get("distance_m", 2.0) or 2.0
            duration = max(2.0, dist / speed)
            num_ticks = int(duration * rate_hz)

            dx, dy, dz = 0.0, 0.0, 0.0
            if direction == "forward":
                dx = dist * math.cos(cur_yaw)
                dy = dist * math.sin(cur_yaw)
            elif direction == "backward":
                dx = -dist * math.cos(cur_yaw)
                dy = -dist * math.sin(cur_yaw)
            elif direction == "right":
                dx = dist * math.sin(cur_yaw)
                dy = -dist * math.cos(cur_yaw)
            elif direction == "left":
                dx = -dist * math.sin(cur_yaw)
                dy = dist * math.cos(cur_yaw)
            elif direction == "up":
                dz = -dist  # in NED, negative is UP
            elif direction == "down":
                dz = dist   # in NED, positive is DOWN

            target_x = cur_x + dx
            target_y = cur_y + dy
            target_z = cur_z + dz

            for i in range(num_ticks):
                alpha = (i + 1) / num_ticks
                smooth_alpha = 0.5 * (1 - math.cos(alpha * math.pi))
                x = cur_x + (target_x - cur_x) * smooth_alpha
                y = cur_y + (target_y - cur_y) * smooth_alpha
                z = cur_z + (target_z - cur_z) * smooth_alpha
                stream_setpoints.append((x, y, z, cur_yaw))

            cur_x, cur_y, cur_z = target_x, target_y, target_z

        elif action == "pattern":
            ptype = step.get("pattern_type", "spiral")
            radius = step.get("radius_m", 2.0) or 2.0
            height = step.get("height_m", 2.0) or 2.0
            turns = step.get("turns", 2) or 2
            direction = step.get("direction", "up")

            # Duration calculated for smooth tangential speed (~0.8 m/s)
            circ = 2 * math.pi * radius * turns
            duration = max(10.0, circ / 0.8)
            num_ticks = int(duration * rate_hz)
            total_angle = 2 * math.pi * turns

            center_x = cur_x - radius * math.sin(cur_yaw)
            center_y = cur_y + radius * math.cos(cur_yaw)

            if ptype == "spiral":
                z_delta = -height if direction == "up" else height
                for i in range(num_ticks):
                    t_ratio = (i + 1) / num_ticks
                    theta = cur_yaw - (math.pi / 2) + total_angle * t_ratio
                    x = center_x + radius * math.cos(theta)
                    y = center_y + radius * math.sin(theta)
                    z = cur_z + z_delta * t_ratio
                    yaw = theta + (math.pi / 2)
                    stream_setpoints.append((x, y, z, yaw))
                cur_z += z_delta

            elif ptype == "circle":
                for i in range(num_ticks):
                    t_ratio = (i + 1) / num_ticks
                    theta = cur_yaw - (math.pi / 2) + total_angle * t_ratio
                    x = center_x + radius * math.cos(theta)
                    y = center_y + radius * math.sin(theta)
                    yaw = theta + (math.pi / 2)
                    stream_setpoints.append((x, y, cur_z, yaw))

            elif ptype == "zigzag":
                # Generate a 2-pass boustrophedon sweep
                sweep_len = 3.0
                step_lat = radius
                legs = [
                    (sweep_len, 0.0),
                    (0.0, step_lat),
                    (-sweep_len, 0.0),
                    (0.0, step_lat),
                    (sweep_len, 0.0)
                ]
                for lx, ly in legs:
                    leg_ticks = int(3.0 * rate_hz)
                    tx = cur_x + lx
                    ty = cur_y + ly
                    for ti in range(leg_ticks):
                        a = (ti + 1) / leg_ticks
                        stream_setpoints.append((cur_x + (tx - cur_x) * a, cur_y + (ty - cur_y) * a, cur_z, cur_yaw))
                    cur_x, cur_y = tx, ty

            cur_x = stream_setpoints[-1][0]
            cur_y = stream_setpoints[-1][1]
            cur_yaw = stream_setpoints[-1][3]

        elif action == "take_photo":
            # Hover in place for 2 seconds to ensure crisp image
            for _ in range(int(2.0 * rate_hz)):
                stream_setpoints.append((cur_x, cur_y, cur_z, cur_yaw))

        elif action == "land":
            # Land command will be dispatched via vehicle_command, but descend gracefully
            duration = 4.0
            num_ticks = int(duration * rate_hz)
            for i in range(num_ticks):
                alpha = (i + 1) / num_ticks
                z = cur_z + (0.0 - cur_z) * alpha
                stream_setpoints.append((cur_x, cur_y, z, cur_yaw))
            cur_z = 0.0

    return stream_setpoints

if __name__ == "__main__":
    sample_steps = [
        {"action": "takeoff", "altitude_m": 2.5, "speed_mps": 1.0},
        {"action": "pattern", "pattern_type": "spiral", "direction": "up", "height_m": 1.5, "radius_m": 2.0, "turns": 2}
    ]
    points = generate_timed_trajectory(sample_steps)
    print(f"Generated {len(points)} setpoints ({len(points)/10.0:.1f} seconds total flight duration)")
    print("Start point:", points[0])
    print("Peak point:", points[-1])
