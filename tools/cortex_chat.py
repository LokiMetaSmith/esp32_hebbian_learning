#!/usr/bin/env python3
"""
Optional Natural Language Gesture & Surgery CLI for the Hebbian Robot.
Interacts with the local Strix Halo LLM to translate natural language directives
into gesture embeddings, kinematic commands, or sparse weight-space surgical patches.
"""

import sys
import os
import json
import argparse
import urllib.request
from typing import Dict, Any, List

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from tools.bicameral_cortex import BicameralCortexBridge, DEFAULT_LLAMA_URL

INTENT_PARSER_SCHEMA = {
    "type": "object",
    "properties": {
        "action_type": {
            "type": "string",
            "enum": ["ik_move", "surgery", "modulate", "status", "teach", "surface", "fusion", "flow"]
        },
        "description": {
            "type": "string"
        },
        "target_coordinates": {
            "type": "array",
            "items": {"type": "number"},
            "description": "[x, y, z] in meters for ik_move",
            "minItems": 3,
            "maxItems": 3
        },
        "gesture_id": {
            "type": "integer",
            "description": "Gesture ID (1-10) for teach mode"
        },
        "modulator_value": {
            "type": "number"
        },
        "surgery_directive": {
            "type": "string"
        }
    },
    "required": ["action_type", "description"],
    "additionalProperties": False
}


class CortexChatCLI:
    def __init__(self, llama_url: str = DEFAULT_LLAMA_URL, mock_hw: bool = True, timeout: float = 10.0,
                 serial_port: str = "/dev/ttyUSB0", baudrate: int = 115200):
        self.llama_url = llama_url
        self.mock_hw = mock_hw
        self.timeout = timeout
        self.bridge = BicameralCortexBridge(serial_port=serial_port, baudrate=baudrate, llama_url=llama_url, mock_hw=mock_hw)
        self.bridge.connect()

    def parse_natural_language_intent(self, user_text: str) -> Dict[str, Any]:
        """Translates user natural language into structured robotic action schema."""
        prompt = f"Map the user command to robot actions: '{user_text}'"
        body = {
            "model": "local-strix-halo-model",
            "messages": [
                {
                    "role": "system",
                    "content": "You are a robotics intent parser for a 6-DOF arm. Parse commands into ik_move, surgery, modulate, status, teach, surface, fusion, or flow."
                },
                {"role": "user", "content": prompt}
            ],
            "temperature": 0.1,
            "response_format": {
                "type": "json_schema",
                "json_schema": {
                    "name": "robot_intent",
                    "strict": True,
                    "schema": INTENT_PARSER_SCHEMA
                }
            }
        }

        try:
            req = urllib.request.Request(
                self.llama_url,
                data=json.dumps(body).encode("utf-8"),
                headers={"Content-Type": "application/json"},
                method="POST"
            )
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                data = json.loads(resp.read().decode("utf-8"))
                return json.loads(data["choices"][0]["message"]["content"])
        except Exception:
            # Heuristic offline fallback parser
            lower = user_text.lower()
            if "reach" in lower or "move" in lower or "pick" in lower or "ik" in lower:
                # Try to extract numbers if present
                import re
                nums = [float(n) for n in re.findall(r"[-+]?(?:\d*\.\d+|\d+)", user_text)]
                coords = [0.25, 0.0, 0.15]
                if len(nums) >= 3:
                    coords = nums[:3]
                return {
                    "action_type": "ik_move",
                    "description": "Moving arm towards target position",
                    "target_coordinates": coords
                }
            elif "teach" in lower or "record" in lower or "gesture" in lower:
                import re
                nums = [int(n) for n in re.findall(r"\b\d+\b", user_text)]
                gid = nums[0] if nums else 1
                return {
                    "action_type": "teach",
                    "description": f"Recording current pose as gesture {gid}",
                    "gesture_id": gid
                }
            elif "surface" in lower or "texture" in lower or "stiffness" in lower or "roughness" in lower:
                return {
                    "action_type": "surface",
                    "description": "Profiling surface roughness and stiffness"
                }
            elif "fusion" in lower or "som" in lower:
                return {
                    "action_type": "fusion",
                    "description": "Querying sensory fusion SOM state"
                }
            elif "flow" in lower or "optical" in lower or "slip" in lower:
                return {
                    "action_type": "flow",
                    "description": "Querying optical flow and slip detection"
                }
            elif "modulat" in lower or "reward" in lower or "dopamine" in lower:
                import re
                nums = [float(n) for n in re.findall(r"[-+]?(?:\d*\.\d+|\d+)", user_text)]
                mod_val = nums[0] if nums else 1.5
                return {
                    "action_type": "modulate",
                    "description": f"Adjusting neuromodulator M(t) to {mod_val}",
                    "modulator_value": mod_val
                }
            elif "status" in lower or "state" in lower or "telemetry" in lower:
                return {
                    "action_type": "status",
                    "description": "Querying robot CNS status"
                }
            else:
                return {
                    "action_type": "surgery",
                    "description": "Performing weight-space surgery",
                    "surgery_directive": user_text
                }

    def execute_intent(self, intent: Dict[str, Any]) -> str:
        action = intent.get("action_type")
        desc = intent.get("description", "")
        print(f"[CORTEX_CHAT] Intent: {action.upper()} - {desc}")

        if action == "ik_move":
            coords = intent.get("target_coordinates", [0.2, 0.0, 0.2])
            res = self.bridge.send_command(f"ik-move {coords[0]} {coords[1]} {coords[2]}")
            return f"IK Move commanded to ({coords[0]}, {coords[1]}, {coords[2]}): {res}"
        elif action == "modulate":
            val = intent.get("modulator_value", 1.0)
            res = self.bridge.send_command(f"modulate {val}")
            return f"Modulator set to {val}: {res}"
        elif action == "status":
            res = self.bridge.send_command("get-full-status")
            return f"Full status queried: {res}"
        elif action == "teach":
            gid = intent.get("gesture_id", 1)
            res = self.bridge.send_command(f"teach-gesture {gid}")
            return f"Teach gesture {gid}: {res}"
        elif action == "surface":
            res = self.bridge.send_command("get-surface-profile")
            return f"Surface profile queried: {res}"
        elif action == "fusion":
            res = self.bridge.send_command("get-fusion-state")
            return f"Fusion state queried: {res}"
        elif action == "flow":
            res = self.bridge.send_command("get-optical-flow")
            return f"Optical flow queried: {res}"
        elif action == "surgery":
            directive = intent.get("surgery_directive", desc)
            success = self.bridge.run_surgery_cycle(directive)
            return f"Surgery Cycle Result: {'SUCCESS' if success else 'FAILED'}"
        return "Unknown intent"

    def close(self):
        self.bridge.close()


def main():
    parser = argparse.ArgumentParser(description="Natural Language Cortex Chat CLI")
    parser.add_argument("--query", help="Single natural language command to execute directly")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port for ESP32 console")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--llama-url", default=DEFAULT_LLAMA_URL, help="llama-server HTTP completion URL")
    parser.add_argument("--mock-hw", action="store_true", default=True, help="Run with mock hardware")
    parser.add_argument("--dry-run", action="store_true", help="Alias for mock hardware dry-run")
    parser.add_argument("--timeout", type=float, default=10.0, help="HTTP timeout for llama-server")
    args = parser.parse_args()

    is_mock = args.mock_hw or args.dry_run
    cli = CortexChatCLI(
        llama_url=args.llama_url,
        mock_hw=is_mock,
        timeout=args.timeout,
        serial_port=args.port,
        baudrate=args.baud
    )

    if args.query:
        intent = cli.parse_natural_language_intent(args.query)
        res = cli.execute_intent(intent)
        print(res)
    else:
        print("=== Hebbian Robot Cortex Chat (Type 'exit' or 'quit' to quit) ===")
        while True:
            try:
                line = input("cortex> ").strip()
                if not line or line in ("exit", "quit"):
                    break
                intent = cli.parse_natural_language_intent(line)
                res = cli.execute_intent(intent)
                print(f"Result: {res}\n")
            except (KeyboardInterrupt, EOFError):
                break

    cli.close()


if __name__ == "__main__":
    main()
