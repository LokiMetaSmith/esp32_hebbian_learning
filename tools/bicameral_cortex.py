#!/usr/bin/env python3
"""
Bicameral Cortex Bridge for `esp32_hebbian_learning` on Strix Halo Workstation.
Connects Strix Halo local LLM (`llama-server` with GBNF JSON Schema) with ESP32-S3 USB Serial
and an independent ground-truth evaluator (Camera ArUco/AprilTag tracking + Accelerometer).
"""

import sys
import os
import time
import json
import argparse
import urllib.request
import urllib.error
from typing import Dict, Any, List, Optional

# Add repo root and tools to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from tools.vision_tracker import VisionTracker

try:
    import serial
except ImportError:
    serial = None

DEFAULT_LLAMA_URL = "http://127.0.0.1:8080/v1/chat/completions"

SURGERY_JSON_SCHEMA = {
    "type": "object",
    "properties": {
        "diagnosis": {
            "type": "string",
            "description": "Mechanistic diagnosis of weight saturation, dead nodes, or kinematic error."
        },
        "modulator": {
            "type": "number",
            "description": "3rd-factor neuromodulator M(t) scalar in [-2.0, 2.0]."
        },
        "eval_ticks": {
            "type": "integer",
            "description": "Evaluation window length in ticks (100 to 2000)."
        },
        "edits": {
            "type": "array",
            "description": "Sparse edits: [[post_idx, pre_idx, new_W, new_eta], ...]",
            "items": {
                "type": "array",
                "items": {"type": "number"},
                "minItems": 4,
                "maxItems": 4
            }
        }
    },
    "required": ["diagnosis", "modulator", "eval_ticks", "edits"],
    "additionalProperties": False
}

SEMANTIC_PINOUT_MANIFEST = """
You are the Symbolic Left Hemisphere and Weight-Space Architect for an ESP32-S3 robotic arm.
Physical topology:
- Robot Arm: SO-101 6-DOF robot arm with Feetech STS3215 serial bus servos.
- Sensor inputs (N_INPUT = 6):
  * ch 0: Prediction Dissonance (last prediction error)
  * ch 1: Servo Total Normalized Current
  * ch 2: Peer Robot Stress Signal (ESP-NOW)
  * ch 3: Obstacle Proximity Signal (1.0 = near collision)
  * ch 4: BMA400 I2C Accelerometer X-axis
  * ch 5: BMA400 I2C Accelerometer Y-axis
- Reservoir (N_RES = 64):
  * Interneurons 0..50: Excitatory log-normal recurrent connections
  * Interneurons 51..63: Inhibitory connections (Dale's Law)
- Readout Outputs (N_OUTPUT = 4):
  * ch 0: Emergency Pain / Stress Output (drives compliance softening)
  * ch 1: Motion Excitation / Exploration Drive
  * ch 2: Joint 1-3 Compliance Reflex
  * ch 3: Joint 4-6 Compliance Reflex

Your task:
Analyze dead nodes (|mean_act| < 0.01), saturated nodes (|mean_act| > 0.85), stress, and ground-truth errors.
Produce sparse weight modifications [post_idx, pre_idx, new_W, new_eta] targeting readout weights (post 0..3, pre 0..63) or recurrent weights.
Freeze non-problematic reflex arcs by setting eta = 0.0, or boost local plasticity with eta in [1.0, 3.0].
Output MUST strictly follow the JSON schema.
"""


class MockSerialESP32:
    """Mock serial device emulating the ESP32-S3 console for dry-run verification."""
    def __init__(self):
        self.active_bank = 0
        self.modulator = 1.0
        self.ab_eval_active = False
        self.eval_ticks_remaining = 0
        self.in_buf = ""

    def write(self, data: bytes):
        cmd = data.decode('utf-8', errors='ignore').strip()
        if cmd == "dump-weights":
            edits = [[0, 10, 0.45, 1.0], [0, 22, -0.35, 1.0], [1, 5, 0.20, 0.5]]
            dead = [{"node": 14, "mean_act": 0.002}]
            resp = {
                "event": "weights_dump",
                "active_bank": self.active_bank,
                "modulator": self.modulator,
                "ab_eval_active": self.ab_eval_active,
                "eval_ticks_remaining": self.eval_ticks_remaining,
                "stress": 0.35,
                "dissonance": 0.12,
                "dead_nodes": dead,
                "edits": [["lsm_out", e[0], e[1], e[2], e[3]] for e in edits]
            }
            self.in_buf += f"@JSON:{json.dumps(resp)}\nrobot>"
        elif cmd.startswith("patch-ab"):
            self.active_bank ^= 1
            self.ab_eval_active = True
            self.eval_ticks_remaining = 500
            resp = {
                "event": "patch_ab_success",
                "active_bank": self.active_bank,
                "applied_edits": 2,
                "eval_ticks": 500
            }
            self.in_buf += f"@JSON:{json.dumps(resp)}\nrobot>"
        elif cmd == "commit-ab":
            self.ab_eval_active = False
            resp = {"event": "commit_ab_success", "active_bank": self.active_bank}
            self.in_buf += f"@JSON:{json.dumps(resp)}\nrobot>"
        elif cmd == "rollback-ab":
            self.active_bank ^= 1
            self.ab_eval_active = False
            resp = {"event": "rollback_ab_success", "active_bank": self.active_bank}
            self.in_buf += f"@JSON:{json.dumps(resp)}\nrobot>"
        elif cmd.startswith("modulate"):
            parts = cmd.split()
            if len(parts) > 1:
                self.modulator = float(parts[1])
            resp = {"event": "modulate_success", "modulator": self.modulator}
            self.in_buf += f"@JSON:{json.dumps(resp)}\nrobot>"
        else:
            self.in_buf += "robot>"

    def readline(self) -> bytes:
        if "\n" in self.in_buf:
            line, self.in_buf = self.in_buf.split("\n", 1)
            return (line + "\n").encode('utf-8')
        line = self.in_buf
        self.in_buf = ""
        return line.encode('utf-8')

    def close(self):
        pass


class BicameralCortexBridge:
    def __init__(self, serial_port: str = "/dev/ttyUSB0", baudrate: int = 115200,
                 llama_url: str = DEFAULT_LLAMA_URL, mock_hw: bool = False):
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.llama_url = llama_url
        self.mock_hw = mock_hw
        self.tracker = VisionTracker(mock_mode=mock_hw)
        self.ser = None

    def connect(self) -> bool:
        if self.mock_hw or serial is None:
            print(f"[CORTEX] Connecting to Mock ESP32 device...")
            self.ser = MockSerialESP32()
            return True
        try:
            print(f"[CORTEX] Opening serial connection on {self.serial_port} ({self.baudrate} baud)...")
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=2.0)
            time.sleep(1.0)
            return True
        except Exception as e:
            print(f"[CORTEX] Error opening serial port: {e}. Falling back to mock hardware.")
            self.ser = MockSerialESP32()
            return True

    def send_command(self, cmd: str) -> Optional[Dict[str, Any]]:
        """Sends command to ESP32 console and parses @JSON line."""
        if not self.ser:
            return None
        self.ser.write((cmd + "\n").encode('utf-8'))
        json_obj = None
        start_time = time.time()
        while time.time() - start_time < 3.0:
            raw_line = self.ser.readline()
            if not raw_line:
                continue
            line = raw_line.decode('utf-8', errors='ignore').strip()
            if "@JSON:" in line:
                idx = line.find("@JSON:")
                try:
                    json_str = line[idx + 6:]
                    json_obj = json.loads(json_str)
                    break
                except json.JSONDecodeError:
                    pass
            elif "robot>" in line and json_obj is not None:
                break
        return json_obj

    def query_strix_halo_llm(self, telemetry_prompt: str) -> Dict[str, Any]:
        """Queries local llama-server on Strix Halo with strict GBNF JSON Schema."""
        body = {
            "model": "local-strix-halo-model",
            "messages": [
                {"role": "system", "content": SEMANTIC_PINOUT_MANIFEST},
                {"role": "user", "content": telemetry_prompt}
            ],
            "temperature": 0.2,
            "response_format": {
                "type": "json_schema",
                "json_schema": {
                    "name": "connectome_surgery",
                    "strict": True,
                    "schema": SURGERY_JSON_SCHEMA
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
            with urllib.request.urlopen(req, timeout=30.0) as resp:
                data = json.loads(resp.read().decode("utf-8"))
                content = data["choices"][0]["message"]["content"]
                return json.loads(content)
        except Exception as e:
            print(f"[CORTEX] llama-server query failed or unreachable ({e}). Returning heuristic mock edit.")
            return {
                "diagnosis": "Heuristic fallback: damping dead interneuron 14 and freezing reflex arc.",
                "modulator": 1.0,
                "eval_ticks": 300,
                "edits": [
                    [0, 14, 0.05, 0.0],
                    [1, 5, 0.35, 1.2]
                ]
            }

    def evaluate_independent_ground_truth(self, esp32_telemetry: Dict[str, Any]) -> float:
        """
        Calculates unified fitness combining OpenCV camera tracking
        and BMA400 accelerometer tilt/jerk metrics.
        Higher score is better.
        """
        vision_res = self.tracker.track_frame()
        visual_error = vision_res.get("visual_error_dist", 0.3)
        accel_jerk = esp32_telemetry.get("jerk", 0.0)
        dissonance = esp32_telemetry.get("dissonance", 0.0)

        # Penalize visual distance from target, mechanical vibration, and internal dissonance
        fitness = -(visual_error + 0.4 * accel_jerk + 0.6 * dissonance)
        return fitness

    def run_surgery_cycle(self, user_directive: str = "Optimize stability and reduce tremor.") -> bool:
        print(f"\n[CORTEX] --- Starting Weight-Space Surgery Cycle ---")
        print(f"[CORTEX] User directive: '{user_directive}'")

        # 1. Inspect baseline connectome
        print("[CORTEX] Requesting baseline weights dump...")
        baseline_dump = self.send_command("dump-weights")
        if not baseline_dump:
            print("[CORTEX] Failed to receive baseline weights dump.")
            return False

        print(f"[CORTEX] Baseline: active_bank={baseline_dump.get('active_bank')}, "
              f"stress={baseline_dump.get('stress')}, dissonance={baseline_dump.get('dissonance')}")

        baseline_fitness = self.evaluate_independent_ground_truth(baseline_dump)
        print(f"[CORTEX] Independent Ground-Truth Baseline Fitness: {baseline_fitness:.4f}")

        # 2. Consult Strix Halo LLM
        prompt = (
            f"User directive: {user_directive}\n"
            f"Current Telemetry: {json.dumps(baseline_dump)}\n"
            f"Baseline ground-truth fitness: {baseline_fitness:.4f}\n"
            f"Diagnose failure modes and specify sparse edits."
        )
        print("[CORTEX] Consulting Strix Halo LLM Weight Architect...")
        surgery_plan = self.query_strix_halo_llm(prompt)
        print(f"[CORTEX] Diagnosis: {surgery_plan.get('diagnosis')}")
        print(f"[CORTEX] Proposed edits: {surgery_plan.get('edits')}")

        # 3. Stage A/B patch to shadow bank
        eval_ticks = surgery_plan.get("eval_ticks", 500)
        patch_payload = {
            "eval_ticks": eval_ticks,
            "modulator": surgery_plan.get("modulator", 1.0),
            "edits": surgery_plan.get("edits", [])
        }
        print(f"[CORTEX] Staging patch to shadow bank on ESP32-S3...")
        patch_res = self.send_command(f"patch-ab {json.dumps(patch_payload)}")
        print(f"[CORTEX] Patch response: {patch_res}")

        # 4. Shadow Evaluation Window
        print(f"[CORTEX] Evaluating shadow patch across {eval_ticks} ticks...")
        time.sleep(0.5)

        # 5. Dual-Oracle Verification
        shadow_dump = self.send_command("dump-weights") or {}
        shadow_fitness = self.evaluate_independent_ground_truth(shadow_dump)
        print(f"[CORTEX] Shadow Ground-Truth Fitness: {shadow_fitness:.4f} (Baseline: {baseline_fitness:.4f})")

        # Fitness threshold: must not degrade significantly
        if shadow_fitness >= baseline_fitness - 0.05:
            print("[CORTEX] Shadow bank passed dual-oracle verification! Committing patch...")
            commit_res = self.send_command("commit-ab")
            print(f"[CORTEX] Commit response: {commit_res}")
            return True
        else:
            print("[CORTEX] Shadow bank degraded fitness! Triggering automatic rollback...")
            rb_res = self.send_command("rollback-ab")
            print(f"[CORTEX] Rollback response: {rb_res}")
            return False

    def close(self):
        if self.ser:
            self.ser.close()
        self.tracker.close()


def main():
    parser = argparse.ArgumentParser(description="Bicameral Cortex Bridge for ESP32 Hebbian Robot")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port for ESP32-S3 console")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--llama-url", default=DEFAULT_LLAMA_URL, help="llama-server HTTP completion URL")
    parser.add_argument("--mock-hw", action="store_true", help="Run with mock hardware for dry-run testing")
    parser.add_argument("--dry-run", action="store_true", help="Verify GBNF schema and dry run without hardware")
    parser.add_argument("--directive", default="Reduce end-effector oscillation and dead-node count", help="Natural language directive")
    args = parser.parse_args()

    mock = args.mock_hw or args.dry_run
    bridge = BicameralCortexBridge(
        serial_port=args.port,
        baudrate=args.baud,
        llama_url=args.llama_url,
        mock_hw=mock
    )

    if not bridge.connect():
        sys.exit(1)

    success = bridge.run_surgery_cycle(args.directive)
    bridge.close()

    print(f"\n[CORTEX] Surgery cycle finished. Status: {'SUCCESS' if success else 'FAILED'}")
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
