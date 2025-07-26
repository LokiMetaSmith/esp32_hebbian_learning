import socket
import json
import time
import argparse
import sys
import os
import cv2 # OpenCV for computer vision
import base64
from openai import OpenAI, APIError

# --- Oklathon 2025 NRP AI API Configuration ---
# This section is adapted from the Oklathon Challenge Boilerplate.

# Define the base URL for the NRP AI API
BASE_URL = "https://llm.nrp-nautilus.io/v1"
MODEL_TO_USE = "llava-onevision" # Using the recommended multimodal model

# Safely get the API key from environment variables
# In Colab, this would be handled by `userdata.get('NRP_API_KEY')`
NRP_API_KEY = os.environ.get("NRP_API_KEY")

# --- Project Configuration ---
MCP_PORT = 8888
CAMERA_INDEX = 0 # 0 is usually the default webcam

# --- Helper Functions ---
def encode_image_to_base64(image_path):
    """Encodes an image file to a base64 string."""
    try:
        with open(image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode('utf-8')
    except Exception as e:
        print(f"[ERROR] Failed to encode image {image_path}: {e}")
        return None

def capture_image_from_camera(output_filename="live_scene.jpg"):
    """Captures a single frame from the webcam and saves it to a file."""
    print("\n[CAM] Initializing camera...")
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("[CAM] ERROR: Could not open camera.")
        return None
    time.sleep(2)
    ret, frame = cap.read()
    cap.release()
    if not ret:
        print("[CAM] ERROR: Failed to capture frame.")
        return None
    cv2.imwrite(output_filename, frame)
    print(f"[CAM] Image captured and saved as '{output_filename}'")
    return output_filename

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
        if not self.sock: return None
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
        if arguments: request["arguments"] = arguments
        return self._send_request(request)

# --- LLM Controller (Updated for NRP API and Multimodal Input) ---
class LLMController:
    def __init__(self, api_key, base_url):
        if not api_key:
            raise ValueError("API key not provided. Please set the NRP_API_KEY environment variable.")
        self.client = OpenAI(api_key=api_key, base_url=base_url)

    def find_target_with_vision(self, image_path, target_description):
        """Asks the LLaVA model to find a target in an image and return its coordinates."""
        print(f"\n[LLM] Asking vision model to find '{target_description}' in the image...")

        base64_image = encode_image_to_base64(image_path)
        if not base64_image:
            return None

        prompt = f"""
        Analyze the provided image to locate the character "{target_description}".
        Respond with a JSON object containing the normalized coordinates of the character's center.
        The JSON object should have two keys: "x" and "y", with values between 0.0 and 1.0.
        If the character is not found, respond with "x": null and "y": null.
        Example response: {{"x": 0.54, "y": 0.38}}
        """
        try:
            completion = self.client.chat.completions.create(
                model=MODEL_TO_USE,
                messages=[{
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                        {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}}
                    ]
                }],
                max_tokens=300,
                response_format={"type": "json_object"}
            )
            response_content = completion.choices[0].message.content
            print(f"[LLM] Vision model response: {response_content}")
            coords = json.loads(response_content)
            if coords.get("x") is not None and coords.get("y") is not None:
                return (coords["x"], coords["y"])
            else:
                print("[LLM] Vision model could not find the target.")
                return None
        except Exception as e:
            print(f"[LLM] ERROR: Vision model request failed: {e}")
            return None

    def get_robot_commands(self, tools, target_coords):
        """Generates robot commands based on target coordinates (hardcoded logic)."""
        print("\n[PLANNER] Generating robot commands for the target coordinates...")
        x, y = target_coords

        def clamp(val, min_val=0, max_val=4095):
            return int(max(min_val, min(val, max_val)))

        # Simplified mapping from (x, y) to servo positions
        base_pos = clamp(2048 + (x - 0.5) * 2000)
        shoulder_pos = clamp(2048 + (y - 0.5) * 1500)

        print(f"[PLANNER] Plan: Set servo 1 to {base_pos}, servo 2 to {shoulder_pos}.")

        command_plan = [
            {"tool_name": "set_pos", "arguments": {"id": 1, "pos": base_pos}},
            {"tool_name": "set_pos", "arguments": {"id": 2, "pos": shoulder_pos}}
        ]
        return command_plan

def main(robot_ip):
    """Main function to run the Where's Waldo demo."""
    print("="*50)
    print(">>> WHERE'S WALDO - ROBOTIC POINTER DEMO (NRP Vision API) <<<")
    print("="*50)

    if not NRP_API_KEY:
        print("[ERROR] NRP_API_KEY environment variable not set. Exiting.")
        return

    # Step 1: Capture a live image from the webcam
    scene_path = capture_image_from_camera()
    if not scene_path:
        return

    # Step 2: Use the LLM vision model to find Waldo
    llm = LLMController(api_key=NRP_API_KEY, base_url=BASE_URL)
    waldo_coords = llm.find_target_with_vision(scene_path, "Waldo")
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

        # Step 5: Generate a plan to point at the coordinates
        command_plan = llm.get_robot_commands(tool_info["tools"], waldo_coords)
        if not command_plan:
            print("[PLANNER] ERROR: Could not generate a command plan. Exiting.")
            return

        # Step 6: Execute the plan
        print("\n[CONTROLLER] Executing the plan on the robot...")
        for command in command_plan:
            mcp_client.call_tool(command["tool_name"], command.get("arguments"))
            time.sleep(0.5)

        print("\n[CONTROLLER] Robot is now pointing at Waldo! Mission complete.")

    finally:
        # Step 7: Disconnect
        mcp_client.disconnect()
        print("="*50)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Where's Waldo Demo with NRP Vision API")
    parser.add_argument("ip_address", help="IP address of the ESP32 robot.")
    args = parser.parse_args()

    main(args.ip_address)
