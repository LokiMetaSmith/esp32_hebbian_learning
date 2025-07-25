import json
import socket
import time
import numpy as np
import torch
import torch.nn as nn
import snntorch as snn
from snntorch import surrogate
from snn_kinematic_model import SNNKinematicModel

class MCPClient:
    """A client to interact with the ESP32's MCP TCP server."""
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.host, self.port))

    def disconnect(self):
        if self.sock:
            self.sock.close()

    def send_command(self, command, arguments=None):
        request = {"command": command}
        if arguments:
            request["arguments"] = arguments

        self.sock.sendall((json.dumps(request) + "\n").encode('utf-8'))

        response_str = self.sock.recv(1024).decode('utf-8')
        return json.loads(response_str)

def collect_kinematic_data(mcp_client, num_samples=100):
    print("Collecting kinematic data...")
    data = []
    for _ in range(num_samples):
        commanded_positions = np.random.uniform(-1.0, 1.0, 6)
        mcp_client.send_command("call_tool", {"tool_name": "set_pos_all", "arguments": {"positions": commanded_positions.tolist()}})
        time.sleep(0.5)
        sensor_data = mcp_client.send_command("call_tool", {"tool_name": "get_all_sensors"})
        data.append({"commanded": commanded_positions.tolist(), "sensed": sensor_data["result"]})
    print("Data collection complete.")
    return data

def train_kinematic_model(data):
    print("Training kinematic model...")
    input_size = len(data[0]["sensed"])
    output_size = len(data[0]["commanded"])
    hidden_size = 128
    model = SNNKinematicModel(input_size, hidden_size, output_size)
    criterion = nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)

    for epoch in range(10):
        for sample in data:
            inputs = torch.tensor(sample["sensed"], dtype=torch.float)
            targets = torch.tensor(sample["commanded"], dtype=torch.float)
            optimizer.zero_grad()
            outputs, _ = model(inputs)
            loss = criterion(outputs.sum(0), targets)
            loss.backward()
            optimizer.step()
        print(f"Epoch {epoch+1}, Loss: {loss.item()}")

    print("Training complete.")
    return model

def estimate_state_from_sensors(model, sensor_data):
    print("Estimating state from sensors...")
    inputs = torch.tensor(sensor_data, dtype=torch.float)
    outputs, _ = model(inputs)
    return outputs.sum(0).detach().numpy()

def main():
    # --- Configuration ---
    ESP32_IP = "192.168.4.1"  # IMPORTANT: Replace with your ESP32's IP
    ESP32_PORT = 8888

    # --- Initialization ---
    mcp_client = MCPClient(host=ESP32_IP, port=ESP32_PORT)

    try:
        print("Connecting to MCP server...")
        mcp_client.connect()

        # 1. Collect kinematic data
        kinematic_data = collect_kinematic_data(mcp_client)

        # 2. Train the kinematic model
        kinematic_model = train_kinematic_model(kinematic_data)

        # 3. Estimate state from sensors
        sensor_data = mcp_client.send_command("call_tool", {"tool_name": "get_all_sensors"})
        estimated_state = estimate_state_from_sensors(kinematic_model, sensor_data["result"])
        print(f"Estimated state: {estimated_state}")

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        print("\nDisconnecting from MCP server.")
        mcp_client.disconnect()

if __name__ == "__main__":
    main()
