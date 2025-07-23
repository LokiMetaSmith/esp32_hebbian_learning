import socket
import json
import time
import argparse
import sys
import os
from openai import OpenAI

# --- NRP AI API Configuration ---
BASE_URL = "https://llm.nrp-nautilus.io/v1"
MODEL_TO_USE = "llama3" # A good general-purpose model for command interpretation
NRP_API_KEY = os.environ.get("NRP_API_KEY")

# --- Project Configuration ---
MCP_PORT = 8888

# --- MCP Client (from the Waldo demo) ---
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
            print("[MCP] Connection successful. You can now chat with the robot.")
            return True
        except (socket.error, socket.timeout) as e:
            print(f"[MCP] ERROR: Robot connection failed: {e}")
            return False

    def disconnect(self):
        if self.sock:
            self.sock.close()
            self.sock = None
            print("\n[MCP] Disconnected from robot.")

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
        return self._send_request({"command": "list_tools"})

    def call_tool(self, tool_name, arguments=None):
        request = {"command": "call_tool", "tool_name": tool_name}
        if arguments: request["arguments"] = arguments
        return self._send_request(request)

# --- LLM Controller for Chat ---
class LLMChatController:
    def __init__(self, api_key, base_url):
        if not api_key:
            raise ValueError("API key not provided. Please set the NRP_API_KEY environment variable.")
        self.client = OpenAI(api_key=api_key, base_url=base_url)

    def get_robot_plan_from_chat(self, tools, user_command):
        """
        Asks an LLM to convert a natural language command into a robot action plan.
        """
        print("[LLM] Thinking...")

        prompt = f"""
        You are a controller for a robot arm. Your task is to convert a user's natural language command into a sequence of tool calls for the robot to execute.
        The user said: "{user_command}"

        Here are the available tools: {json.dumps(tools)}

        - The 'set_pos' tool controls individual servos.
        - Servo positions range from 0 to 4095. The center position is 2048.
        - Servo 1 is the base (pan).
        - Servo 2 is the shoulder (pitch).
        - "point left" should correspond to a high position for servo 1 (e.g., 3000).
        - "point right" should correspond to a low position for servo 1 (e.g., 1000).
        - "point up" should correspond to a low position for servo 2 (e.g., 1500).
        - "point down" should correspond to a high position for servo 2 (e.g., 2500).
        - "center" means move to position 2048.

        Respond with a JSON object containing a "commands" key, which is an array of tool call objects.
        Each object must have a "tool_name" and an "arguments" key.
        If the user's command is unclear or cannot be mapped to a tool, return an empty commands array.
        Example response for "point up and left":
        {{"commands": [
            {{"tool_name": "set_pos", "arguments": {{"id": 1, "pos": 3000}}}},
            {{"tool_name": "set_pos", "arguments": {{"id": 2, "pos": 1500}}}}
        ]}}
        """

        try:
            completion = self.client.chat.completions.create(
                model=MODEL_TO_USE,
                messages=[
                    {"role": "system", "content": "You are a helpful robot control assistant that responds only in the requested JSON format."},
                    {"role": "user", "content": prompt}
                ],
                response_format={"type": "json_object"}
            )
            response_content = completion.choices[0].message.content
            command_plan = json.loads(response_content).get("commands", [])

            if not isinstance(command_plan, list):
                print("[LLM] ERROR: AI returned an invalid format. No action taken.")
                return []

            return command_plan
        except Exception as e:
            print(f"[LLM] ERROR: Failed to get response from AI: {e}")
            return []

def main(args):
    """Main function to run the interactive chat client."""
    print("="*50)
    print(">>> INTERACTIVE ROBOT CHAT CLIENT <<<")
    print("="*50)

    if not NRP_API_KEY:
        print("[ERROR] NRP_API_KEY environment variable not set. Exiting.")
        return

    # Initialize clients
    mcp_client = MCPClient(args.ip_address, MCP_PORT)
    llm_controller = LLMChatController(api_key=NRP_API_KEY, base_url=BASE_URL)

    if not mcp_client.connect():
        return

    try:
        # Get the list of tools once at the start
        tool_info = mcp_client.list_tools()
        if not tool_info or "tools" not in tool_info:
            print("[MCP] ERROR: Could not get tool list from robot. Exiting.")
            return

        print(f"[INFO] Robot tools available: {[t['name'] for t in tool_info['tools']]}")
        print("-" * 50)

        # Main chat loop
        while True:
            user_input = input("You> ").strip()
            if user_input.lower() in ["quit", "exit"]:
                break
            if not user_input:
                continue

            # Get a plan from the LLM
            command_plan = llm_controller.get_robot_plan_from_chat(tool_info["tools"], user_input)

            # Execute the plan
            if command_plan:
                print(f"[CONTROLLER] Executing plan: {command_plan}")
                for command in command_plan:
                    mcp_client.call_tool(command["tool_name"], command.get("arguments"))
                    time.sleep(0.1) # Small delay between commands
            else:
                print("[CONTROLLER] No action taken.")

    except KeyboardInterrupt:
        print("\nCaught interrupt, exiting...")
    finally:
        mcp_client.disconnect()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Interactive chat client for robot control.")
    parser.add_argument("--ip", dest="ip_address", required=True, help="IP address of the ESP32 robot.")
    args = parser.parse_args()
    main(args)
