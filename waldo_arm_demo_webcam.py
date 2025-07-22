import socket
import json
import time
import argparse
import sys
import os
import cv2 # OpenCV for computer vision
import numpy as np
from openai import OpenAI # To interact with a real LLM

# --- Configuration ---
MCP_PORT = 8888
CAMERA_INDEX = 0 # 0 is usually the default webcam

# --- New Camera Capture Function ---
def capture_image_from_camera(output_filename="live_scene.jpg"):
    """
    Captures a single frame from the webcam and saves it to a file.

    Returns:
        The path to the saved image file, or None on failure.
    """
    print("\n[CAM] Initializing camera...")
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("[CAM] ERROR: Could not open camera. Please check the camera index and connections.")
        return None

    # Allow camera to adjust to lighting
    time.sleep(2) 

    ret, frame = cap.read()
    cap.release()

    if not ret:
        print("[CAM] ERROR: Failed to capture frame from camera.")
        return None

    cv2.imwrite(output_filename, frame)
    print(f"[CAM] Image captured and saved as '{output_filename}'")
    return output_filename

# --- Real Computer Vision Implementation (no changes) ---
def find_waldo_in_image(scene_image_path, waldo_template_path):
    """
    Finds a template image (Waldo) within a larger scene image using OpenCV.
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

        result = cv2.matchTemplate(scene_img, waldo_img, cv2.TM_CCOEFF_NORMED)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
        
        top_left = max_loc
        h, w = waldo_img.shape[:2]
        center_x = top_left[0] + w / 2
        center_y = top_left[1] + h / 2

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
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None

    def connect(self):
        print(f"\n[MCP] Connecting to robot at {self.host}:{self.port}...")
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

# --- Real LLM Controller (no changes) ---
class LLMController:
    def __init__(self, api_key):
        if not api_key:
            raise ValueError("API key not provided. Please set the OPENAI_API_KEY environment variable.")
        self.client = OpenAI(api_key=api_key)

    def get_robot_commands(self, tools, waldo_coords):
        print("\n[LLM] Asking AI model to create a plan to point at the coordinates...")
        x, y = waldo_coords
        prompt = f"""
        You are a controller for a 2-axis robot arm. Your goal is to point the arm at a target located at normalized screen coordinates (x={x:.3f}, y={y:.3f}).
        The robot has the following tools available: {json.dumps(tools)}.
        The 'set_pos' tool controls the servos, which range from 0 to 4095. The center is 2048.
        Servo 1 (base/pan) controls the x-axis. A coordinate of 0.0 should map to a low position (e.g., 1024) and 1.0 to a high position (e.g., 3072).
        Servo 2 (shoulder/pitch) controls the y-axis.
        Generate a JSON object containing a "commands" key with an array of command objects to execute.
        Example format: {{"commands": [{{"tool_name": "set_pos", "arguments": {{"id": 1, "pos": 2048}}}}]}}
        """
        try:
            completion = self.client.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {"role": "system", "content": "You are a helpful robot control assistant that responds only in the requested JSON format."},
                    {"role": "user", "content": prompt}
                ],
                response_format={"type": "json_object"}
            )
            response_content = completion.choices[0].message.content
            print(f"[LLM] Received AI response: {response_content}")
            command_plan = json.loads(response_content).get("commands")
            if not isinstance(command_plan, list):
                print("[LLM] ERROR: AI did not return a valid list of commands.")
                return None
            print("[LLM] Plan generated successfully.")
            return command_plan
        except Exception as e:
            print(f"[LLM] ERROR: Failed to get response from AI: {e}")
            return None

def main(robot_ip, template_path):
    """Main function to run the Where's Waldo demo."""
    print("="*50)
    print(">>> WHERE'S WALDO - ROBOTIC POINTER DEMO (Live Camera) <<<")
    print("="*50)

    # Step 1: Capture a live image from the webcam
    scene_path = capture_image_from_camera()
    if not scene_path:
        print("\n[CAM] Could not capture image. Exiting.")
        return

    # Step 2: Find Waldo in the captured image
    waldo_coords = find_waldo_in_image(scene_path, template_path)
    if waldo_coords is None:
        print("\n[CV] Could not find Waldo in the live image. Exiting.")
        return

    # Step 3: Connect to the robot
    mcp_client = MCPClient(robot_ip, MCP_PORT)
    if not mcp_client.connect():
        return

    try:
        # Step 4: Get robot's capabilities
        tool_info = mcp_client.list_tools()
        if not tool_info or "tools" not in tool_info:
            print("[MCP] ERROR: Could not get tool list from robot. Exiting.")
            return
        
        # Step 5: Use the LLM to create a plan
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            print("[LLM] ERROR: OPENAI_API_KEY environment variable not set. Exiting.")
            return
            
        llm = LLMController(api_key=api_key)
        command_plan = llm.get_robot_commands(tool_info["tools"], waldo_coords)

        if not command_plan:
            print("[LLM] ERROR: Could not generate a command plan. Exiting.")
            return
            
        # Step 6: Execute the plan
        print("\n[CONTROLLER] Executing the LLM's plan on the robot...")
        for command in command_plan:
            mcp_client.call_tool(command["tool_name"], command.get("arguments"))
            time.sleep(0.5)

        print("\n[CONTROLLER] Robot is now pointing at Waldo! Mission complete.")

    finally:
        # Step 7: Disconnect
        mcp_client.disconnect()
        print("="*50)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Where's Waldo Robotic Pointer Demo with Live Camera")
    parser.add_argument("ip_address", help="IP address of the ESP32 robot.")
    parser.add_argument("template_path", help="Path to the small template image of Waldo's head.")
    args = parser.parse_args()

    main(args.ip_address, args.template_path)
