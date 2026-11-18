#!/usr/bin/env python3
"""
Autonomous Drone Natural Language Benchmark Suite
--------------------------------------------------
Thoroughly evaluates the end-to-end pipeline (LLM Intent Parser -> Safety Validator)
across 25 diverse test cases:
1. Freeform / Creative Metaphors ("dance", "celebrate")
2. Geometric Flight Shapes ("square box", "spiral", "zigzag")
3. Casual Bengali Instructions ("একটু সামনে যাও", "উপরে ওঠো", "ছবি তোলো")
4. Compound Multi-Step Conjunctions ("take off and then ...")
5. English Synonyms ("ascend", "retreat", "drop")
6. Dangerous / Out-of-Bounds Rejection (Ceiling, Ground Clearance, Geofence, Extreme Speed)
"""

import sys
import os
import time

# Add scripts directory to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts")))

from drone_intent_parser import parse_user_command
from drone_safety_validator import DronePhysicalState, validate_and_resolve_mission

BENCHMARK_CASES = [
    # Category 1: Compound Multi-Step Commands
    {
        "id": "TC01",
        "category": "Compound Multi-Step",
        "command": "take off and then go above in a spiral path",
        "initial_airborne": False,
        "should_be_safe": True,
        "expected_actions": ["takeoff", "pattern"]
    },
    {
        "id": "TC02",
        "category": "Compound Multi-Step",
        "command": "take off to 2 meters, move forward 3 meters and hover for 5 seconds",
        "initial_airborne": False,
        "should_be_safe": True,
        "expected_actions": ["takeoff", "move", "hover"]
    },
    {
        "id": "TC03",
        "category": "Compound Multi-Step",
        "command": "move forward 2 meters then rotate 90 degrees right and land",
        "initial_airborne": True,
        "should_be_safe": True,
        "expected_actions": ["move", "rotate", "land"]
    },

    # Category 2: Freeform Creative & Metaphors
    {
        "id": "TC04",
        "category": "Creative Metaphor",
        "command": "dance a little bit in celebration",
        "initial_airborne": True,
        "should_be_safe": True,
        "expected_actions": ["move", "rotate"]
    },
    {
        "id": "TC05",
        "category": "Creative Metaphor",
        "command": "look around 360 degrees and take a photo",
        "initial_airborne": True,
        "should_be_safe": True,
        "expected_actions": ["rotate", "take_photo"]
    },

    # Category 3: Geometric Shapes
    {
        "id": "TC06",
        "category": "Geometric Shape",
        "command": "move in a spiral path",
        "initial_airborne": False,  # Grounded -> Validator should auto-insert takeoff!
        "should_be_safe": True,
        "expected_actions": ["takeoff", "pattern"]
    },
    {
        "id": "TC07",
        "category": "Geometric Shape",
        "command": "fly in a circle with radius 2 meters",
        "initial_airborne": True,
        "should_be_safe": True,
        "expected_actions": ["pattern"]
    },
    {
        "id": "TC08",
        "category": "Geometric Shape",
        "command": "fly in a square box of 3 meters",
        "initial_airborne": True,
        "should_be_safe": True,
        "expected_actions": ["move"]
    },
    {
        "id": "TC09",
        "category": "Geometric Shape",
        "command": "search the area in a zigzag pattern",
        "initial_airborne": True,
        "should_be_safe": True,
        "expected_actions": ["pattern"]
    },

    # Category 4: Casual Bengali
    {
        "id": "TC10",
        "category": "Casual Bengali",
        "command": "একটু সামনে যাও",
        "initial_airborne": True,
        "should_be_safe": True,
        "expected_actions": ["move"]
    },
    {
        "id": "TC11",
        "category": "Casual Bengali",
        "command": "উপরে ২ মিটার ওঠো",
        "initial_airborne": True,
        "should_be_safe": True,
        "expected_actions": [("move", "takeoff")]
    },
    {
        "id": "TC12",
        "category": "Casual Bengali",
        "command": "ডানে ৯০ ডিগ্রি ঘুরে ছবি তোলো",
        "initial_airborne": True,
        "should_be_safe": True,
        "expected_actions": ["rotate", "take_photo"]
    },
    {
        "id": "TC13",
        "category": "Casual Bengali",
        "command": "নিচে নেমে নিরাপদভাবে ল্যান্ড করো",
        "initial_airborne": True,
        "should_be_safe": True,
        "expected_actions": ["land"]
    },

    # Category 5: Synonyms & Directional Variations
    {
        "id": "TC14",
        "category": "Synonyms",
        "command": "ascend 1.5 meters",
        "initial_airborne": True,
        "should_be_safe": True,
        "expected_actions": [("move", "takeoff")]
    },
    {
        "id": "TC15",
        "category": "Synonyms",
        "command": "climb higher by 1 meter",
        "initial_airborne": True,
        "should_be_safe": True,
        "expected_actions": [("move", "takeoff")]
    },
    {
        "id": "TC16",
        "category": "Synonyms",
        "command": "step back slowly",
        "initial_airborne": True,
        "should_be_safe": True,
        "expected_actions": ["move"]
    },
    {
        "id": "TC17",
        "category": "Synonyms",
        "command": "spin counter clockwise 45 degrees",
        "initial_airborne": True,
        "should_be_safe": True,
        "expected_actions": ["rotate"]
    },

    # Category 6: Aviation Safety & Physical Violation Rejection (MUST BE REJECTED)
    {
        "id": "TC18",
        "category": "Safety Enforcement",
        "command": "fly up 20 meters high",  # Ceiling violation (> 5m)
        "initial_airborne": True,
        "should_be_safe": False,
        "expected_actions": []
    },
    {
        "id": "TC19",
        "category": "Safety Enforcement",
        "command": "move forward at 15 meters per second",  # Speed violation (> 2m/s)
        "initial_airborne": True,
        "should_be_safe": False,
        "expected_actions": []
    },
    {
        "id": "TC20",
        "category": "Safety Enforcement",
        "command": "fly 50 meters away to the North",  # Geofence breach (> 10m)
        "initial_airborne": True,
        "should_be_safe": False,
        "expected_actions": []
    },
    {
        "id": "TC21",
        "category": "Safety Enforcement",
        "command": "dive down into the ground immediately",  # Ground collision (< 0.5m)
        "initial_airborne": True,
        "should_be_safe": False,
        "expected_actions": []
    },
    {
        "id": "TC22",
        "category": "Safety Enforcement",
        "command": "do a complete backflip and dive underwater",  # Physically impossible / unsupported
        "initial_airborne": True,
        "should_be_safe": False,
        "expected_actions": []
    }
]

def run_benchmark():
    print("=" * 80)
    print("🚀 RUNNING AUTONOMOUS DRONE NATURAL LANGUAGE BENCHMARK SUITE")
    print(f"📊 Total Test Cases: {len(BENCHMARK_CASES)}")
    print("=" * 80)

    passed_count = 0
    results = []

    for test in BENCHMARK_CASES:
        t_id = test["id"]
        category = test["category"]
        cmd = test["command"]
        airborne = test["initial_airborne"]
        should_be_safe = test["should_be_safe"]
        expected_acts = test["expected_actions"]

        print(f"\n[{t_id}] ({category}) Command: \"{cmd}\"")
        start_time = time.time()

        # Step 1: LLM Parsing
        parsed = parse_user_command(cmd)
        parse_dur = time.time() - start_time

        # Step 2: Safety Validation
        initial_z = 2.0 if airborne else 0.0
        state = DronePhysicalState(is_airborne=airborne, z=initial_z)
        is_safe, resolved_steps, waypoints, logs = validate_and_resolve_mission(parsed, state)

        # Check safety alignment
        safety_match = (is_safe == should_be_safe)

        # Check action coverage if expected
        detected_actions = [s.get("action") for s in resolved_steps]
        action_match = True
        if should_be_safe and expected_acts:
            for exp in expected_acts:
                if isinstance(exp, tuple):
                    if not any(a in detected_actions for a in exp):
                        action_match = False
                        break
                else:
                    if exp not in detected_actions:
                        action_match = False
                        break

        case_passed = safety_match and (action_match if should_be_safe else True)

        status_str = "✅ PASS" if case_passed else "❌ FAIL"
        if case_passed:
            passed_count += 1

        print(f"   Outcome: {status_str} | Safe: {is_safe} (Expected: {should_be_safe}) | Time: {parse_dur:.2f}s")
        if resolved_steps:
            acts = [s.get("action") for s in resolved_steps]
            print(f"   Resolved Steps: {acts}")
        if logs:
            for l in logs:
                print(f"   Log: {l}")

        results.append({
            "id": t_id,
            "category": category,
            "command": cmd,
            "passed": case_passed,
            "duration": parse_dur
        })

    # Summary Report
    pass_rate = (passed_count / len(BENCHMARK_CASES)) * 100.0
    print("\n" + "=" * 80)
    print("📈 BENCHMARK EVALUATION SUMMARY")
    print("=" * 80)
    print(f"Passed: {passed_count} / {len(BENCHMARK_CASES)} ({pass_rate:.1f}%)")
    if pass_rate >= 90.0:
        print("🏆 RESULT: BENCHMARK QUALIFIED FOR SITL FLIGHT INTEGRATION!")
    else:
        print("⚠️ RESULT: BENCHMARK FAILED - TUNING REQUIRED.")
    print("=" * 80)

if __name__ == "__main__":
    run_benchmark()
