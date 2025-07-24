import socket
import json
import time
import os
import numpy as np
from socket_utils import send_json, recv_json

class MCPClient:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None

    def connect(self):
        print(f"[TEST] Connecting to server at {self.host}:{self.port}...")
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(10)
            self.sock.connect((self.host, self.port))
            print("[TEST] Connection successful.")
            return True
        except (socket.error, socket.timeout) as e:
            print(f"[TEST] ERROR: Connection failed: {e}")
            return False

    def disconnect(self):
        if self.sock:
            self.sock.close()
            self.sock = None
            print("\n[TEST] Disconnected from server.")

    def _send_request(self, request_obj):
        if not self.sock: return None
        send_json(self.sock, request_obj)
        return recv_json(self.sock)

    def list_tools(self):
        return self._send_request({"command": "list_tools"})

    def call_tool(self, tool_name, arguments=None):
        request = {"command": "call_tool", "tool_name": tool_name}
        if arguments: request["arguments"] = arguments
        return self._send_request(request)

def main():
    client = MCPClient('localhost', 8888)
    if not client.connect():
        return

    # 1. Test list_tools
    print("\n--- Testing list_tools ---")
    tools = client.list_tools()
    print(f"Available tools: {tools}")
    assert "tools" in tools
    tool_names = [t["name"] for t in tools["tools"]]
    assert "record_video" in tool_names
    assert "analyze_joint_positions" in tool_names
    assert "import_nn_json" in tool_names
    assert "learn" in tool_names

    # 2. Test record_video
    print("\n--- Testing record_video ---")
    video_filename = "test_video.avi"
    response = client.call_tool("record_video", {"output_filename": video_filename, "duration": 1})
    print(f"Response: {response}")
    if "error" in response and response["error"] == "Could not open camera":
        print("Skipping record_video test because no camera is available.")
    else:
        assert response["result"] == "OK"
        assert os.path.exists(video_filename)
        os.remove(video_filename)

    # 3. Test analyze_joint_positions
    print("\n--- Testing analyze_joint_positions ---")
    response = client.call_tool("analyze_joint_positions", {"video_filename": "dummy.avi"})
    print(f"Response: {response}")
    assert response["result"] == "OK"
    assert "joint_positions" in response

    # 4. Test learn
    print("\n--- Testing learn ---")
    commanded_positions = {"joint1": 0.5, "joint2": -0.2, "joint3": 0.8}
    observed_positions = {"joint1": 0.45, "joint2": -0.25, "joint3": 0.85}
    response = client.call_tool("learn", {"commanded_positions": commanded_positions, "observed_positions": observed_positions})
    print(f"Response: {response}")
    assert response["result"] == "OK"
    assert "updated_nn" in response

    # 5. Test import_nn_json
    print("\n--- Testing import_nn_json ---")
    nn_data = response["updated_nn"]
    response = client.call_tool("import_nn_json", {"nn_data": nn_data})
    print(f"Response: {response}")
    assert response["result"] == "OK"
    assert os.path.exists("imported_nn.json")
    os.remove("imported_nn.json")

    # 6. Test collect_kinematic_data
    print("\n--- Testing collect_kinematic_data ---")
    data_filename = "test_kinematic_data.json"
    response = client.call_tool("collect_kinematic_data", {"output_filename": data_filename, "num_samples": 10})
    print(f"Response: {response}")
    assert response["result"] == "OK"
    assert os.path.exists(data_filename)

    # 7. Test train_kinematic_model
    print("\n--- Testing train_kinematic_model ---")
    model_filename = "test_kinematic_model.pt"
    response = client.call_tool("train_kinematic_model", {"data_filename": data_filename, "model_filename": model_filename, "num_epochs": 2})
    print(f"Response: {response}")
    assert response["result"] == "OK"
    assert os.path.exists(model_filename)

    # 8. Test estimate_state_from_sensors
    print("\n--- Testing estimate_state_from_sensors ---")
    with open(data_filename, 'r') as f:
        sensor_data = json.load(f)[0]
    response = client.call_tool("estimate_state_from_sensors", {"model_filename": model_filename, "sensor_data": sensor_data})
    print(f"Response: {response}")
    assert response["result"] == "OK"
    assert "estimated_positions" in response

    # 9. Test learn with SNN
    print("\n--- Testing learn with SNN ---")
    commanded_positions = sensor_data["commanded_positions"]
    response = client.call_tool("learn", {"commanded_positions": commanded_positions, "sensor_data": sensor_data, "model_filename": model_filename})
    print(f"Response: {response}")
    assert response["result"] == "OK"
    assert "updated_nn" in response

    # Clean up
    os.remove(data_filename)
    os.remove(model_filename)

    client.disconnect()
    print("\n[TEST] All tests passed!")

if __name__ == "__main__":
    main()
