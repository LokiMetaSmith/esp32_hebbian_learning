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
            "enum": ["ik_move", "surgery", "modulate", "status", "teach"]
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
    def __init__(self, llama_url: str = DEFAULT_LLAMA_URL, mock_hw: bool = True):
        self.llama_url = llama_url
        self.mock_hw = mock_hw
        self.bridge = BicameralCortexBridge(llama_url=llama_url, mock_hw=mock_hw)
        self.bridge.connect()

    def parse_natural_language_intent(self, user_text: str) -> Dict[str, Any]:
        """Translates user natural language into structured robotic action schema."""
        prompt = f"Map the user command to robot actions: '{user_text}'"
        body = {
            "model": "local-strix-halo-model",
            "messages": [
                {
                    "role": "system",
                    "content": "You are a robotics intent parser for a 6-DOF arm. Parse commands into ik_move, surgery, modulate, status, or teach."
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
            with urllib.request.urlopen(req, timeout=10.0) as resp:
                data = json.loads(resp.read().decode("utf-8"))
                return json.loads(data["choices"][0]["message"]["content"])
        except Exception:
            # Heuristic offline fallback parser
            lower = user_text.lower()
            if "reach" in lower or "move" in lower or "pick" in lower:
                return {
                    "action_type": "ik_move",
                    "description": "Moving arm towards target position",
                    "target_coordinates": [0.25, 0.0, 0.15]
                }
            elif "modulat" in lower or "reward" in lower:
                return {
                    "action_type": "modulate",
                    "description": "Adjusting neuromodulator M(t)",
                    "modulator_value": 1.5
                }
            elif "status" in lower or "state" in lower:
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
    parser.add_argument("--mock-hw", action="store_true", default=True, help="Run with mock hardware")
    args = parser.parse_args()

    cli = CortexChatCLI(mock_hw=args.mock_hw)

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
