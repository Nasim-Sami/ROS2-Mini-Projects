#!/usr/bin/env python3
"""
Drone Safety Validator & Normalizer (Stage 2 & 3)
------------------------------------------------
1. Injects intelligent defaults for any unspecified (null) values.
2. Tracks Drone Physical State (GROUNDED vs AIRBORNE).
3. Auto-prepends takeoff if user requests airborne maneuvers while grounded.
4. Simulates continuous 3D Trajectory Envelope.
5. Enforces hard physical safety limits (Ground Clearance, Ceiling, Geofence, Max Velocity).
"""

import math
from typing import Dict, Any, List, Tuple

# Hard Aviation Safety Boundaries (Expanded 10x by User Directive)
MIN_GROUND_CLEARANCE_M = 0.5
MAX_CEILING_ALTITUDE_M = 150.0   # Expanded 10x to 150.0m ceiling
MAX_GEOFENCE_RADIUS_M = 300.0    # Expanded 10x to 300.0m geofence radius
MAX_FLIGHT_SPEED_MPS = 10.0      # Expanded to 10.0 m/s max velocity

# Safe Standard Defaults
DEFAULT_TAKEOFF_ALT_M = 2.5
DEFAULT_MOVE_DIST_M = 2.0
DEFAULT_MOVE_SPEED_MPS = 1.0
DEFAULT_ROTATE_DEG = 90.0
DEFAULT_HOVER_DUR_S = 3.0

class DronePhysicalState:
    def __init__(self, is_airborne: bool = False, x: float = 0.0, y: float = 0.0, z: float = 0.0, yaw: float = 0.0):
        self.is_airborne = is_airborne
        self.x = x
        self.y = y
        self.z = z  # Height above ground in meters (always positive for altitude)
        self.yaw = yaw

    def clone(self):
        return DronePhysicalState(self.is_airborne, self.x, self.y, self.z, self.yaw)

def normalize_step(step: Dict[str, Any]) -> Dict[str, Any]:
    """
    Replaces null fields with safe defaults based on primitive type.
    """
    s = dict(step)
    action = s.get("action")

    if action == "takeoff":
        alt = s.get("altitude_m") or s.get("height_m") or s.get("distance_m")
        s["altitude_m"] = alt if alt is not None else DEFAULT_TAKEOFF_ALT_M
        if s.get("speed_mps") is None:
            s["speed_mps"] = DEFAULT_MOVE_SPEED_MPS

    elif action == "move":
        dist = s.get("distance_m") or s.get("height_m") or s.get("altitude_m")
        s["distance_m"] = dist if dist is not None else DEFAULT_MOVE_DIST_M
        if s.get("speed_mps") is None:
            s["speed_mps"] = DEFAULT_MOVE_SPEED_MPS

    elif action == "rotate":
        if s.get("angle_deg") is None:
            s["angle_deg"] = DEFAULT_ROTATE_DEG
        if s.get("direction") is None:
            s["direction"] = "clockwise"

    elif action == "pattern":
        if s.get("radius_m") is None:
            s["radius_m"] = 2.0
        if s.get("height_m") is None:
            s["height_m"] = 2.0
        if s.get("turns") is None:
            s["turns"] = 2
        if s.get("speed_mps") is None:
            s["speed_mps"] = DEFAULT_MOVE_SPEED_MPS

    elif action == "hover":
        if s.get("duration_s") is None:
            s["duration_s"] = DEFAULT_HOVER_DUR_S

    return s

def validate_and_resolve_mission(
    raw_mission: Dict[str, Any],
    initial_state: DronePhysicalState,
    obstacle_ahead: bool = False,
    obstacle_type: str = None,
    obstacle_dist: float = None
) -> Tuple[bool, List[Dict[str, Any]], List[Tuple[float, float, float]], List[str]]:
    """
    Validates mission against physical rules, performs state-aware repairs,
    evaluates YOLOv8 vision obstacle proximity,
    and returns (is_safe, resolved_steps, simulated_waypoints, log_messages).
    """
    steps = raw_mission.get("steps", [])
    if not steps:
        return False, [], [], ["Mission has no executable steps."]

    state = initial_state.clone()
    resolved_steps: List[Dict[str, Any]] = []
    simulated_waypoints: List[Tuple[float, float, float]] = [(state.x, state.y, state.z)]
    logs: List[str] = []

    # 1. State-Aware Inversion Check:
    # If the drone is grounded and the first flight action is NOT takeoff, auto-insert takeoff
    first_action = steps[0].get("action")
    if not state.is_airborne and first_action in ["move", "rotate", "pattern", "hover"]:
        auto_takeoff = {
            "action": "takeoff",
            "altitude_m": DEFAULT_TAKEOFF_ALT_M,
            "speed_mps": DEFAULT_MOVE_SPEED_MPS
        }
        resolved_steps.append(auto_takeoff)
        state.is_airborne = True
        state.z = DEFAULT_TAKEOFF_ALT_M
        simulated_waypoints.append((state.x, state.y, state.z))
        logs.append(f"⚡ State-Aware Auto-Repair: Prepending safe takeoff to {DEFAULT_TAKEOFF_ALT_M}m before maneuver.")

    # 2. Iterate through steps, normalize, and simulate physical 3D path
    for idx, raw_step in enumerate(steps, 1):
        action = raw_step.get("action")
        reason = raw_step.get("reason")

        if action == "unsupported" or (reason and any(w in reason.lower() for w in ["crash", "danger", "prohibit", "reject"])):
            msg = reason or "Command is dangerous or unsupported."
            return False, [], [], [f"🛑 Action Rejected: {msg}"]

        s = normalize_step(raw_step)

        # Check speed constraint
        speed = s.get("speed_mps") or 0.0
        if speed > MAX_FLIGHT_SPEED_MPS:
            return False, [], [], [f"🛑 Step {idx} Rejected: Speed {speed} m/s exceeds max limit ({MAX_FLIGHT_SPEED_MPS} m/s)."]

        if action == "takeoff":
            if state.is_airborne:
                logs.append(f"ℹ️ Step {idx}: Drone already airborne at {state.z:.1f}m. Adjusting altitude to {s['altitude_m']}m.")
            state.is_airborne = True
            state.z = s["altitude_m"]
            simulated_waypoints.append((state.x, state.y, state.z))

        elif action == "land":
            state.is_airborne = False
            state.z = 0.0
            simulated_waypoints.append((state.x, state.y, state.z))

        elif action == "hover":
            pass

        elif action in ["take_photo", "search_object"]:
            logs.append(f"📸 Step {idx}: Aerial snapshot camera trigger activated.")

        elif action == "rotate":
            angle = s.get("angle_deg", DEFAULT_ROTATE_DEG)
            direction = s.get("direction", "clockwise")
            delta_rad = math.radians(angle) * (1 if direction == "clockwise" else -1)
            state.yaw += delta_rad

        elif action == "move":
            direction = s.get("direction", "forward")
            dist = s.get("distance_m", DEFAULT_MOVE_DIST_M)

            # 3D Altitude-Aware Vision Obstacle Gatekeeper:
            # Ground obstacles (person ~1.8m, vehicle/bus ~3.0m) only pose a collision risk if drone is at low altitude (<3.0m).
            # If the drone is flying at or above 3.0m altitude, it safely overflies the obstacle with zero collision risk!
            heading_aligned = abs(state.yaw - initial_state.yaw) < math.radians(40)
            ground_obstacle_ceiling_m = 3.0

            if obstacle_ahead and direction == "forward" and heading_aligned:
                target_str = obstacle_type or "Obstacle"
                dist_str = f"~{obstacle_dist:.1f}m" if obstacle_dist else "close range"

                if state.z < ground_obstacle_ceiling_m:
                    return False, [], [], [
                        f"🛑 COLLISION HAZARD REJECTED: Low-altitude ({state.z:.1f}m) forward flight blocked! Detected '{target_str.upper()}' {dist_str} directly ahead. Climb to >=3.0m to fly over, or rotate to avoid!"
                    ]
                else:
                    clearance = state.z - ground_obstacle_ceiling_m
                    logs.append(
                        f"✈️ Overflight Safe: Detected '{target_str.upper()}' on ground, but drone is cruising safely above at {state.z:.1f}m (+{clearance:.1f}m vertical clearance)."
                    )

            if direction == "forward":
                state.x += dist * math.cos(state.yaw)
                state.y += dist * math.sin(state.yaw)
            elif direction == "backward":
                state.x -= dist * math.cos(state.yaw)
                state.y -= dist * math.sin(state.yaw)
            elif direction == "right":
                state.x += dist * math.sin(state.yaw)
                state.y -= dist * math.cos(state.yaw)
            elif direction == "left":
                state.x -= dist * math.sin(state.yaw)
                state.y += dist * math.cos(state.yaw)
            elif direction == "up":
                state.z += dist
            elif direction == "down":
                state.z -= dist

            simulated_waypoints.append((state.x, state.y, state.z))

        elif action == "pattern":
            ptype = s.get("pattern_type", "spiral")
            radius = s.get("radius_m", 2.0)
            height = s.get("height_m", 2.0)
            direction = s.get("direction", "up")

            # Simulate the extreme envelope of the pattern
            if ptype == "spiral":
                z_delta = height if direction == "up" else -height
                state.z += z_delta
                # Maximum horizontal excursion
                state.x += radius
                simulated_waypoints.append((state.x, state.y, state.z))
            elif ptype == "circle":
                state.x += radius
                simulated_waypoints.append((state.x, state.y, state.z))
            elif ptype == "zigzag":
                state.x += radius
                simulated_waypoints.append((state.x, state.y, state.z))

        resolved_steps.append(s)

        # 3. Physics Safety Envelope Verification
        # Rule A: Ground Clearance (if airborne, z must be >= 0.5m)
        if state.is_airborne and state.z < MIN_GROUND_CLEARANCE_M:
            return False, [], [], [
                f"🛑 Step {idx} Physical Violation: Altitude {state.z:.2f}m is below minimum ground clearance ({MIN_GROUND_CLEARANCE_M}m)!"
            ]

        # Rule B: Ceiling Altitude (z must be <= 5.0m)
        if state.z > MAX_CEILING_ALTITUDE_M:
            return False, [], [], [
                f"🛑 Step {idx} Physical Violation: Altitude {state.z:.2f}m exceeds maximum safety ceiling ({MAX_CEILING_ALTITUDE_M}m)!"
            ]

        # Rule C: Geofence Radius (sqrt(x^2 + y^2) <= 10.0m)
        r = math.hypot(state.x, state.y)
        if r > MAX_GEOFENCE_RADIUS_M:
            return False, [], [], [
                f"🛑 Step {idx} Physical Violation: Horizontal position {r:.2f}m breaches geofence radius ({MAX_GEOFENCE_RADIUS_M}m)!"
            ]

    return True, resolved_steps, simulated_waypoints, logs

if __name__ == "__main__":
    sample_mission = {
        "mission_goal": "Takeoff and spiral",
        "steps": [
            {"action": "takeoff", "altitude_m": None},
            {"action": "pattern", "pattern_type": "spiral", "direction": "up", "height_m": 2.0}
        ]
    }
    state = DronePhysicalState(is_airborne=False)
    safe, steps, waypoints, logs = validate_and_resolve_mission(sample_mission, state)
    print("Safe:", safe)
    print("Steps:", steps)
    print("Logs:", logs)
