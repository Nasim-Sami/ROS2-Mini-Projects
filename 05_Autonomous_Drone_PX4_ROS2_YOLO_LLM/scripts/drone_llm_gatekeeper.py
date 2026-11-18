#!/usr/bin/env python3
"""
Autonomous Drone LLM & Safety Gatekeeper
Translates Bengali & English commands into validated, safe drone trajectories.
Platform: Ollama (Qwen2.5) + Python
Author: Nasim Sami
Portfolio Project 05: Autonomous Aerial Robotics, Vision & LLM Control
"""

import json
import requests
import sys

OLLAMA_URL = "http://localhost:11434/api/generate"
MODEL_NAME = "qwen2.5:0.5b"

# ==============================================================================
# SAFETY THRESHOLDS (Aviation Geofence Limits)
# ==============================================================================
SAFETY_LIMITS = {
    "min_altitude": 0.5,    # meters (minimum safe hover above ground)
    "max_altitude": 5.0,    # meters (maximum ceiling in testing arena)
    "max_distance": 10.0,   # meters (maximum single-hop translation)
    "max_speed": 2.0,       # meters/second (safe terminal velocity)
    "max_yaw_rate": 90.0,   # degrees/second (rotation speed limit)
}

ALLOWED_ACTIONS = [
    "takeoff", "land", "hover",
    "move_forward", "move_backward", "move_left", "move_right",
    "rotate", "dance", "zigzag", "take_photo", "make_map"
]

SYSTEM_PROMPT = """You are an aerial robot flight planner.
Given a user command in Bengali or English, output ONLY a valid JSON object.
Do NOT output any markdown, explanations, or commentary. Output pure JSON only.

Keys to include in JSON:
- "action": string (one of: takeoff, land, hover, move_forward, move_backward, move_left, move_right, rotate, dance, zigzag, take_photo, make_map)
- "distance": number or null (in meters)
- "altitude": number or null (in meters)
- "angle": number or null (in degrees)
- "speed": number or null (in m/s, default 1.0)

Examples:
- "সামনে ৩ মিটার যাও" -> {"action": "move_forward", "distance": 3.0, "altitude": null, "angle": null, "speed": 1.0}
- "move 2 meters left" -> {"action": "move_left", "distance": 2.0, "altitude": null, "angle": null, "speed": 1.0}
- "২.৫ মিটার উপরে ওঠো" -> {"action": "takeoff", "distance": null, "altitude": 2.5, "angle": null, "speed": 1.0}
- "takeoff to 3 meters" -> {"action": "takeoff", "distance": null, "altitude": 3.0, "angle": null, "speed": 1.0}
- "ডানে ৯০ ডিগ্রি ঘোরো" -> {"action": "rotate", "distance": null, "altitude": null, "angle": 90.0, "speed": 1.0}
- "dance" or "একটু নাচো" -> {"action": "dance", "distance": null, "altitude": 2.0, "angle": null, "speed": 1.0}
- "zigzag search" or "জিগজ্যাগ করে যাও" -> {"action": "zigzag", "distance": 4.0, "altitude": 2.0, "angle": null, "speed": 1.0}
- "take a photo" or "ছবি তোলো" -> {"action": "take_photo", "distance": null, "altitude": null, "angle": null, "speed": null}
- "make a map" or "ম্যাপ বানাও" -> {"action": "make_map", "distance": 5.0, "altitude": 2.5, "angle": 360.0, "speed": 1.0}
- "ল্যান্ড করো" or "land" -> {"action": "land", "distance": null, "altitude": 0.0, "angle": null, "speed": 0.5}
"""

def query_llm(user_prompt: str) -> dict:
    """Send natural language command to Ollama and receive structured JSON."""
    payload = {
        "model": MODEL_NAME,
        "system": SYSTEM_PROMPT,
        "prompt": user_prompt,
        "format": "json",
        "stream": False,
        "options": {
            "temperature": 0.1
        }
    }
    try:
        response = requests.post(OLLAMA_URL, json=payload, timeout=10)
        response.raise_for_status()
        raw_output = response.json().get("response", "{}")
        parsed_json = json.loads(raw_output)
        return parsed_json
    except Exception as e:
        return {"error": f"LLM Query Failed: {e}"}


def safety_gatekeeper(command: dict) -> tuple[bool, str, dict]:
    """
    Validates LLM-generated command against strict drone safety rules.
    Returns: (is_approved: bool, reason: str, sanitized_command: dict)
    """
    if "error" in command:
        return False, command["error"], {}

    action = command.get("action")
    if not action or action not in ALLOWED_ACTIONS:
        return False, f"Unknown or disallowed action: '{action}'", {}

    distance = command.get("distance")
    altitude = command.get("altitude")
    speed = command.get("speed")
    angle = command.get("angle")

    # 1. Distance Check (Geofence horizontal boundary)
    if distance is not None:
        try:
            dist_val = float(distance)
            if dist_val < 0:
                return False, f"Distance cannot be negative: {dist_val}m", {}
            if dist_val > SAFETY_LIMITS["max_distance"]:
                return False, f"Distance {dist_val}m exceeds maximum safety limit ({SAFETY_LIMITS['max_distance']}m)!", {}
        except (ValueError, TypeError):
            return False, f"Invalid distance value: {distance}", {}

    # 2. Altitude Check (Ceiling and Floor boundary)
    if altitude is not None:
        try:
            alt_val = float(altitude)
            if action != "land" and alt_val < SAFETY_LIMITS["min_altitude"]:
                return False, f"Altitude {alt_val}m is below minimum ground clearance ({SAFETY_LIMITS['min_altitude']}m)!", {}
            if alt_val > SAFETY_LIMITS["max_altitude"]:
                return False, f"Altitude {alt_val}m exceeds maximum arena ceiling ({SAFETY_LIMITS['max_altitude']}m)!", {}
        except (ValueError, TypeError):
            return False, f"Invalid altitude value: {altitude}", {}

    # 3. Speed Check (Velocity limits)
    if speed is not None:
        try:
            spd_val = float(speed)
            if spd_val > SAFETY_LIMITS["max_speed"]:
                return False, f"Speed {spd_val}m/s exceeds maximum safe velocity ({SAFETY_LIMITS['max_speed']}m/s)!", {}
        except (ValueError, TypeError):
            return False, f"Invalid speed value: {speed}", {}

    # 4. Sanitization and defaults
    sanitized = {
        "action": action,
        "distance": float(distance) if distance is not None else 1.0,
        "altitude": float(altitude) if altitude is not None else 2.5,
        "angle": float(angle) if angle is not None else 0.0,
        "speed": float(speed) if speed is not None else 1.0
    }

    return True, "Command complies with all aviation safety standards.", sanitized


def run_pipeline(user_input: str):
    print("=" * 65)
    print(f"🗣️  User Command: \"{user_input}\"")
    print("-" * 65)

    # 1. Query LLM
    llm_output = query_llm(user_input)
    print(f"🤖 LLM Generated JSON:\n{json.dumps(llm_output, indent=2, ensure_ascii=False)}")
    print("-" * 65)

    # 2. Safety Gatekeeper Examination
    approved, reason, sanitized_cmd = safety_gatekeeper(llm_output)

    if approved:
        print(f"🛡️  Safety Gatekeeper: \033[92m[ APPROVED ✅ ]\033[0m")
        print(f"   Reason: {reason}")
        print(f"   🚀 Dispatching to Drone ROS 2 Controller: {sanitized_cmd}")
    else:
        print(f"🛡️  Safety Gatekeeper: \033[91m[ REJECTED ❌ ]\033[0m")
        print(f"   Reason: {reason}")
        print(f"   🛑 Execution blocked for safety. Drone holds current position.")
    print("=" * 65 + "\n")


def interactive_mode():
    print("=================================================================")
    print("  🎮 Interactive Drone Voice/Text Command Console (ଧাপ ২: LLM)")
    print("  Type any command in Bengali or English (or 'exit' to quit).")
    print("=================================================================\n")
    while True:
        try:
            cmd = input("👉 Enter Command: ").strip()
            if not cmd:
                continue
            if cmd.lower() in ["exit", "quit", "q"]:
                print("Exiting console. Goodbye!")
                break
            run_pipeline(cmd)
        except (KeyboardInterrupt, EOFError):
            print("\nExiting console.")
            break


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        test_cases = [
            "সামনে ৩ মিটার যাও",                     # Valid translation
            "৫০ মিটার উপরে ওঠো",                     # REJECT: Exceeds 5m ceiling
            "dance",                                  # Valid macro
            "move forward 20 meters at 10 m/s",       # REJECT: Exceeds 10m dist & 2m/s speed
            "take a photo",                           # Valid vision action
            "জিগজ্যাগ করে সামনের এলাকা খোঁজো",      # Valid patrol macro
            "ল্যান্ড করো",                           # Valid landing
        ]
        print("=================================================================")
        print("  🧪 Running Standalone Test Suite (ধাপ ২: LLM + Safety Gatekeeper)")
        print("=================================================================\n")
        for test in test_cases:
            run_pipeline(test)
    else:
        interactive_mode()


if __name__ == "__main__":
    main()
