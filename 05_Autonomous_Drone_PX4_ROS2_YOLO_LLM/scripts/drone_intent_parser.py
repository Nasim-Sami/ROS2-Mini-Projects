#!/usr/bin/env python3
"""
Drone Intent Parser (Stage 1)
------------------------------
Leverages local LLM (Qwen 2.5 1.5B via Ollama) to translate open-ended,
arbitrary natural language (English & Bengali, metaphors, geometric shapes)
into structured Universal Motion Primitives.
"""

import json
import requests
import re
from typing import Dict, Any, List, Optional

OLLAMA_API_URL = "http://localhost:11434/api/chat"
MODEL_NAME = "qwen2.5:1.5b"

SYSTEM_PROMPT = """You are an Autonomous Drone Flight Mission Planner.
Your role is to translate ANY user command (English, Bengali, casual speech, metaphors like 'dance', geometric shapes like 'box' or 'spiral') into an ordered sequence of Universal Motion Primitives.

### Universal Motion Primitives:
1. "takeoff": Ascend vertically from ground. Params: altitude_m (float or null)
2. "land": Descend vertically and disarm motors. Params: none
3. "hover": Hold current 3D position. Params: duration_s (float or null)
4. "move": Translate along an axis. Params: direction ("forward"|"backward"|"left"|"right"|"up"|"down"), distance_m (float or null), speed_mps (float or null)
5. "rotate": Yaw in place. Params: direction ("clockwise"|"counter_clockwise"), angle_deg (float or null)
6. "pattern": Continuous geometric flight. Params: pattern_type ("spiral"|"circle"|"zigzag"|"figure_8"|"wave"), radius_m (float or null), height_m (float or null), turns (int or null), speed_mps (float or null)
7. "take_photo": Trigger camera capture. Params: target_label (string or null)
8. "search_object": Scan/sweep for object. Params: target_label (string or null)
9. "unsupported": For physical impossibilities (e.g., "backflip", "underwater dive"). Params: reason (string)

### CRITICAL RULES:
- If a value (altitude, distance, speed, angle, duration) is NOT explicitly specified by the user, set it to null. NEVER invent 0.0!
- Synonyms & Relative Altitude:
  - When the drone is airborne, "ascend", "climb", "rise", "go above", "fly higher", "উপরে ওঠো" -> move(direction="up", distance_m=...)
  - "descend", "drop", "sink", "go lower", "নিচে নামো" -> move(direction="down", distance_m=...)
  - "step back", "retreat", "পিছিয়ে আসো" -> move(direction="backward", distance_m=...)
  - "circle", "orbit", "fly in a circle", "চক্কর মারো" -> pattern(pattern_type="circle", radius_m=...)
  - "turn right", "ডানে ঘোরো" -> rotate(direction="clockwise", angle_deg=...)
  - "turn left", "বামে ঘোরো" -> rotate(direction="counter_clockwise", angle_deg=...)
- Multi-Action Bengali Conjunctions:
  - If user says multiple actions in Bengali like "ডানে ৯০ ডিগ্রি ঘুরে ছবি তোলো" -> MUST produce 2 steps: rotate AND take_photo.
- Dangerous / Suicidal Commands:
  - "dive into ground", "crash", "মাটিতে আছড়ে পড়ো" -> move(direction="down", distance_m=10.0) or unsupported(reason="Crash maneuver requested"). NEVER convert a violent dive into a gentle land!
- Metaphorical / Creative commands:
  - "dance" / "নাচো" -> Decompose into a celebratory sequence: bob up 0.5m, bob down 0.5m, rotate 30 deg, rotate -60 deg, rotate 30 deg.
  - "square" / "box" of X meters -> 4 consecutive moves forward, right, backward, left with 90 deg turns or axis moves.
- Output strictly a valid JSON object matching the schema below, with no markdown fences, no conversational prose.

### Output JSON Schema:
{
  "mission_goal": "<brief English description of intent>",
  "steps": [
    {
      "action": "<primitive_name>",
      "direction": "<direction or null>",
      "distance_m": <float or null>,
      "altitude_m": <float or null>,
      "angle_deg": <float or null>,
      "speed_mps": <float or null>,
      "duration_s": <float or null>,
      "pattern_type": "<pattern_type or null>",
      "radius_m": <float or null>,
      "height_m": <float or null>,
      "turns": <int or null>,
      "target_label": "<string or null>",
      "reason": "<string or null>"
    }
  ]
}
"""

FEW_SHOT_EXAMPLES = [
    {
        "role": "user",
        "content": "take off and then go above in a spiral path"
    },
    {
        "role": "assistant",
        "content": json.dumps({
            "mission_goal": "Takeoff and ascend along a 3D spiral trajectory",
            "steps": [
                {
                    "action": "takeoff",
                    "direction": None,
                    "distance_m": None,
                    "altitude_m": None,
                    "angle_deg": None,
                    "speed_mps": None,
                    "duration_s": None,
                    "pattern_type": None,
                    "radius_m": None,
                    "height_m": None,
                    "turns": None,
                    "target_label": None,
                    "reason": None
                },
                {
                    "action": "pattern",
                    "direction": "up",
                    "distance_m": None,
                    "altitude_m": None,
                    "angle_deg": None,
                    "speed_mps": None,
                    "duration_s": None,
                    "pattern_type": "spiral",
                    "radius_m": None,
                    "height_m": None,
                    "turns": None,
                    "target_label": None,
                    "reason": None
                }
            ]
        })
    },
    {
        "role": "user",
        "content": "dance a little bit in celebration"
    },
    {
        "role": "assistant",
        "content": json.dumps({
            "mission_goal": "Perform a celebratory aerial dance sequence",
            "steps": [
                {
                    "action": "move",
                    "direction": "up",
                    "distance_m": 0.5,
                    "altitude_m": None,
                    "angle_deg": None,
                    "speed_mps": 0.8,
                    "duration_s": None,
                    "pattern_type": None,
                    "radius_m": None,
                    "height_m": None,
                    "turns": None,
                    "target_label": None,
                    "reason": None
                },
                {
                    "action": "move",
                    "direction": "down",
                    "distance_m": 0.5,
                    "altitude_m": None,
                    "angle_deg": None,
                    "speed_mps": 0.8,
                    "duration_s": None,
                    "pattern_type": None,
                    "radius_m": None,
                    "height_m": None,
                    "turns": None,
                    "target_label": None,
                    "reason": None
                },
                {
                    "action": "rotate",
                    "direction": "clockwise",
                    "distance_m": None,
                    "altitude_m": None,
                    "angle_deg": 30.0,
                    "speed_mps": None,
                    "duration_s": None,
                    "pattern_type": None,
                    "radius_m": None,
                    "height_m": None,
                    "turns": None,
                    "target_label": None,
                    "reason": None
                },
                {
                    "action": "rotate",
                    "direction": "counter_clockwise",
                    "distance_m": None,
                    "altitude_m": None,
                    "angle_deg": 60.0,
                    "speed_mps": None,
                    "duration_s": None,
                    "pattern_type": None,
                    "radius_m": None,
                    "height_m": None,
                    "turns": None,
                    "target_label": None,
                    "reason": None
                },
                {
                    "action": "rotate",
                    "direction": "clockwise",
                    "distance_m": None,
                    "altitude_m": None,
                    "angle_deg": 30.0,
                    "speed_mps": None,
                    "duration_s": None,
                    "pattern_type": None,
                    "radius_m": None,
                    "height_m": None,
                    "turns": None,
                    "target_label": None,
                    "reason": None
                }
            ]
        })
    },
    {
        "role": "user",
        "content": "সামনে ৩ মিটার গিয়ে চারপাশ দেখে আসো"
    },
    {
        "role": "assistant",
        "content": json.dumps({
            "mission_goal": "Move forward 3 meters, rotate 360 degrees to observe surroundings, and hover",
            "steps": [
                {
                    "action": "move",
                    "direction": "forward",
                    "distance_m": 3.0,
                    "altitude_m": None,
                    "angle_deg": None,
                    "speed_mps": 1.0,
                    "duration_s": None,
                    "pattern_type": None,
                    "radius_m": None,
                    "height_m": None,
                    "turns": None,
                    "target_label": None,
                    "reason": None
                },
                {
                    "action": "rotate",
                    "direction": "clockwise",
                    "distance_m": None,
                    "altitude_m": None,
                    "angle_deg": 360.0,
                    "speed_mps": None,
                    "duration_s": None,
                    "pattern_type": None,
                    "radius_m": None,
                    "height_m": None,
                    "turns": None,
                    "target_label": None,
                    "reason": None
                },
                {
                    "action": "hover",
                    "direction": None,
                    "distance_m": None,
                    "altitude_m": None,
                    "angle_deg": None,
                    "speed_mps": None,
                    "duration_s": 3.0,
                    "pattern_type": None,
                    "radius_m": None,
                    "height_m": None,
                    "turns": None,
                    "target_label": None,
                    "reason": None
                }
            ]
        })
    },
    {
        "role": "user",
        "content": "ডানে ৯০ ডিগ্রি ঘুরে ছবি তোলো"
    },
    {
        "role": "assistant",
        "content": json.dumps({
            "mission_goal": "Rotate 90 degrees right and capture a photo",
            "steps": [
                {
                    "action": "rotate",
                    "direction": "clockwise",
                    "distance_m": None,
                    "altitude_m": None,
                    "angle_deg": 90.0,
                    "speed_mps": None,
                    "duration_s": None,
                    "pattern_type": None,
                    "radius_m": None,
                    "height_m": None,
                    "turns": None,
                    "target_label": None,
                    "reason": None
                },
                {
                    "action": "take_photo",
                    "direction": None,
                    "distance_m": None,
                    "altitude_m": None,
                    "angle_deg": None,
                    "speed_mps": None,
                    "duration_s": None,
                    "pattern_type": None,
                    "radius_m": None,
                    "height_m": None,
                    "turns": None,
                    "target_label": None,
                    "reason": None
                }
            ]
        })
    },
    {
        "role": "user",
        "content": "fly in a circle with radius 2 meters"
    },
    {
        "role": "assistant",
        "content": json.dumps({
            "mission_goal": "Perform circular orbit trajectory with 2m radius",
            "steps": [
                {
                    "action": "pattern",
                    "direction": None,
                    "distance_m": None,
                    "altitude_m": None,
                    "angle_deg": None,
                    "speed_mps": None,
                    "duration_s": None,
                    "pattern_type": "circle",
                    "radius_m": 2.0,
                    "height_m": None,
                    "turns": 1,
                    "target_label": None,
                    "reason": None
                }
            ]
        })
    },
    {
        "role": "user",
        "content": "উপরে ২ মিটার ওঠো"
    },
    {
        "role": "assistant",
        "content": json.dumps({
            "mission_goal": "Ascend vertically by 2 meters",
            "steps": [
                {
                    "action": "move",
                    "direction": "up",
                    "distance_m": 2.0,
                    "altitude_m": None,
                    "angle_deg": None,
                    "speed_mps": 1.0,
                    "duration_s": None,
                    "pattern_type": None,
                    "radius_m": None,
                    "height_m": None,
                    "turns": None,
                    "target_label": None,
                    "reason": None
                }
            ]
        })
    },
    {
        "role": "user",
        "content": "dive down into the ground immediately"
    },
    {
        "role": "assistant",
        "content": json.dumps({
            "mission_goal": "Prohibited ground crash request",
            "steps": [
                {
                    "action": "unsupported",
                    "direction": None,
                    "distance_m": None,
                    "altitude_m": None,
                    "angle_deg": None,
                    "speed_mps": None,
                    "duration_s": None,
                    "pattern_type": None,
                    "radius_m": None,
                    "height_m": None,
                    "turns": None,
                    "target_label": None,
                    "reason": "Crash maneuver into the ground is dangerous and rejected."
                }
            ]
        })
    }
]

def parse_user_command(user_prompt: str, timeout: int = 60) -> Dict[str, Any]:
    """
    Calls Ollama to parse natural language prompt into structured steps.
    """
    messages = [{"role": "system", "content": SYSTEM_PROMPT}]
    messages.extend(FEW_SHOT_EXAMPLES)
    messages.append({"role": "user", "content": user_prompt})

    payload = {
        "model": MODEL_NAME,
        "messages": messages,
        "format": "json",
        "stream": False,
        "options": {
            "temperature": 0.0,
            "num_predict": 512
        }
    }

    try:
        response = requests.post(OLLAMA_API_URL, json=payload, timeout=timeout)
        response.raise_for_status()
        raw_output = response.json().get("message", {}).get("content", "").strip()

        # Clean potential markdown wrapping
        cleaned = re.sub(r"^```json\s*", "", raw_output, flags=re.MULTILINE)
        cleaned = re.sub(r"^```\s*", "", cleaned, flags=re.MULTILINE)
        cleaned = cleaned.strip()

        parsed = json.loads(cleaned)
        if "steps" not in parsed or not isinstance(parsed["steps"], list):
            raise ValueError("Output missing 'steps' array")
        return parsed

    except requests.exceptions.RequestException as e:
        return {
            "mission_goal": "Connection error",
            "steps": [{"action": "unsupported", "reason": f"Ollama API connection failed: {e}"}]
        }
    except (json.JSONDecodeError, ValueError) as e:
        return {
            "mission_goal": "Parse error",
            "steps": [{"action": "unsupported", "reason": f"Failed to parse LLM JSON: {e}"}]
        }

if __name__ == "__main__":
    import sys
    test_query = "take off and then go above in a spiral path"
    if len(sys.argv) > 1:
        test_query = " ".join(sys.argv[1:])

    print(f"Query: '{test_query}'")
    result = parse_user_command(test_query)
    print(json.dumps(result, indent=2, ensure_ascii=False))
