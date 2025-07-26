import socket
import json
import time
import argparse
import sys
import os
import cv2 # OpenCV for computer vision
import numpy as np
from openai import OpenAI # To interact with a real LLM

# Builds on https://github.com/Ashenstride/Wheres_WALDO

# --- Configuration ---
MCP_PORT = 8888

# --- Real Computer Vision Implementation ---
def find_waldo_in_image(scene_image_path, waldo_template_path):
    """
    Finds a template image (Waldo) within a larger scene image using OpenCV.

    Args:
        scene_image_path (str): The path to the main puzzle image.
        waldo_template_path (str): The path to the small image of Waldo.

    Returns:
        A tuple (x, y) of normalized coordinates (0.0 to 1.0) or None if not found.
    """
    print(f"\n[CV] Searching for '{waldo_template_path}' in '{scene_image_path}'...")

    try:
        scene_img = cv2.imread(scene_image_path, cv2.IMREAD_COLOR)
        waldo_img = cv2.imread(waldo_template_path, cv2.IMREAD_COLOR)

        if scene_img is None:
            print(f"[CV] ERROR: Could not read scene image at: {scene_image_path}")
            return None
        if waldo_img is None:
            print(f"[CV] ERROR: Could not read Waldo template image at: {waldo_template_path}")
            return None

        # Perform template matching
        result = cv2.matchTemplate(scene_img, waldo_img, cv2.TM_CCOEFF_NORMED)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)

        # The top-left corner of the matched area
        top_left = max_loc
        h, w = waldo_img.shape[:2]
        # The center of the matched area
        center_x = top_left[0] + w / 2
        center_y = top_left[1] + h / 2

        # Normalize coordinates to a 0.0 - 1.0 range
        scene_h, scene_w = scene_img.shape[:2]
        norm_x = center_x / scene_w
        norm_y = center_y / scene_h

        print(f"[CV] Waldo found! Confidence: {max_val:.2f}")
        print(f"[CV] Normalized coordinates: (x={norm_x:.2f}, y={norm_y:.2f})")
        return (norm_x, norm_y)

    except Exception as e:
        print(f"[CV] ERROR: An exception occurred during image processing: {e}")
        return None

# --- MCP Client (no changes) ---
class MCPClient:
    """A client to interact with the ESP32's MCP TCP server."""
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None

    def connect(self):
        print(f"[MCP] Connecting to robot at {self.host}:{self.port}...")
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(10)
            self.sock.connect((self.host, self.port))
            print("[MCP] Connection successful.")
            return True
        except (socket.error, socket.timeout) as e:
            print(f"[MCP] ERROR: Robot connection failed: {e}")
            return False

    def disconnect(self):
        if self.sock:
            self.sock.close()
            self.sock = None
            print("[MCP] Disconnected from robot.")

    def _send_request(self, request_obj):
        if not self.sock:
            print("[MCP] ERROR: Not connected.")
            return None
        try:
            self.sock.sendall((json.dumps(request_obj) + '\n').encode('utf-8'))
            response_data = self.sock.recv(1024)
            response_str = response_data.decode('utf-8').strip()
            return json.loads(response_str) if response_str else None
        except (socket.error, json.JSONDecodeError, socket.timeout) as e:
            print(f"[MCP] ERROR: Request failed: {e}")
            return None

    def list_tools(self):
        print("[MCP] Asking robot for its available tools...")
        return self._send_request({"command": "list_tools"})

    def call_tool(self, tool_name, arguments=None):
        print(f"[MCP] Commanding robot to run tool '{tool_name}' with args: {arguments}")
        request = {"command": "call_tool", "tool_name": tool_name}
        if arguments:
            request["arguments"] = arguments
        return self._send_request(request)

# --- Real LLM Controller ---
class LLMController:
    """Uses a real LLM to generate robot commands."""
    def __init__(self, api_key):
        if not api_key:
            raise ValueError("API key not provided. Please set the OPENAI_API_KEY environment variable.")
        self.client = OpenAI(api_key=api_key)

    def get_robot_commands(self, tools, waldo_coords):
        """
        Generates a sequence of robot commands by asking an LLM.
        """
        print("\n[LLM] Asking AI model to create a plan to point at the coordinates...")

        x, y = waldo_coords

        # This prompt is crucial. It gives the LLM context and constraints.
        prompt = f"""
        You are a controller for a 2-axis robot arm. Your goal is to point the arm at a target located at normalized screen coordinates (x={x:.3f}, y={y:.3f}).
        The robot has the following tools available: {json.dumps(tools)}.

        - The 'set_pos' tool controls the servos.
        - Servo ID 1 is the base (pan), controlling the x-axis.
        - Servo ID 2 is the shoulder (pitch), controlling the y-axis.
        - Servo positions range from 0 to 4095. The center position is 2048.
        - For the x-axis (servo 1), a coordinate of 0.0 should map to a low servo position (e.g., 1024) and 1.0 to a high position (e.g., 3072).
        - For the y-axis (servo 2), a coordinate of 0.0 should map to a low servo position (e.g., 1024) and 1.0 to a high position (e.g., 3072).

        Generate a JSON array of commands to execute. Each command should be an object with "tool_name" and "arguments" keys.
        Example format: [{"tool_name": "set_pos", "arguments": {{"id": 1, "pos": 2048}}}]
        """

        try:
            print("[LLM] Sending prompt to AI...")
            completion = self.client.chat.completions.create(
                model="gpt-4o", # Or another capable model
                messages=[
                    {"role": "system", "content": "You are a helpful robot control assistant that responds only in JSON format."},
                    {"role": "user", "content": prompt}
                ],
                response_format={"type": "json_object"}
            )

            response_content = completion.choices[0].message.content
            print(f"[LLM] Received AI response: {response_content}")

            # The LLM should return a JSON object with a "commands" key
            command_plan = json.loads(response_content).get("commands")
            if not isinstance(command_plan, list):
                print("[LLM] ERROR: AI did not return a valid list of commands.")
                return None

            print("[LLM] Plan generated successfully.")
            return command_plan

        except Exception as e:
            print(f"[LLM] ERROR: Failed to get response from AI: {e}")
            return None

def main(robot_ip, scene_path, template_path):
    """Main function to run the Where's Waldo demo."""
    print("="*50)
    print(">>> WHERE'S WALDO - ROBOTIC POINTER DEMO (v2) <<<")
    print("="*50)

    # Step 1: Find Waldo using real computer vision
    waldo_coords = find_waldo_in_image(scene_path, template_path)
    if waldo_coords is None:
        print("\n[CV] Could not find Waldo in the image. Exiting.")
        return

    # Step 2: Connect to the robot
    mcp_client = MCPClient(robot_ip, MCP_PORT)
    if not mcp_client.connect():
        return

    try:
        # Step 3: Get robot's capabilities
        tool_info = mcp_client.list_tools()
        if not tool_info or "tools" not in tool_info:
            print("[MCP] ERROR: Could not get tool list from robot. Exiting.")
            return

        # Step 4: Use the LLM to create a plan
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            print("[LLM] ERROR: OPENAI_API_KEY environment variable not set. Exiting.")
            return

        llm = LLMController(api_key=api_key)
        command_plan = llm.get_robot_commands(tool_info["tools"], waldo_coords)

        if not command_plan:
            print("[LLM] ERROR: Could not generate a command plan. Exiting.")
            return

        # Step 5: Execute the plan
        print("\n[CONTROLLER] Executing the LLM's plan on the robot...")
        for command in command_plan:
            mcp_client.call_tool(command["tool_name"], command.get("arguments"))
            time.sleep(0.5)

        print("\n[CONTROLLER] Robot is now pointing at Waldo! Mission complete.")

    finally:
        # Step 6: Disconnect
        mcp_client.disconnect()
        print("="*50)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Where's Waldo Robotic Pointer Demo")
    parser.add_argument("ip_address", help="IP address of the ESP32 robot.")
    parser.add_argument("scene_path", help="Path to the main 'Where's Waldo' puzzle image.")
    parser.add_argument("template_path", help="Path to the small template image of Waldo's head.")
    args = parser.parse_args()

    main(args.ip_address, args.scene_path, args.template_path)
