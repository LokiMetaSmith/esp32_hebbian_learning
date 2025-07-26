import socket
import json
import threading
import cv2
import time
import numpy as np
from socket_utils import send_json, recv_json
import torch
import torch.nn as nn
from kinematic_model import KinematicModel

class HebbianLearner:
    """
    A simple Hebbian learning component that updates a neural network's weights.
    """
    def __init__(self, input_size, hidden_size, output_size, learning_rate=0.01):
        self.learning_rate = learning_rate
        # Initialize weights with small random values
        self.weights_ih = np.random.randn(hidden_size, input_size) * 0.1
        self.weights_ho = np.random.randn(output_size, hidden_size) * 0.1

    def update_weights(self, input_vector, commanded_state, observed_state):
        """
        Updates the weights based on the correlation between commanded and observed states.
        """
        hidden_activations = np.tanh(np.dot(self.weights_ih, input_vector))

        # The "error" is the difference between what was observed and what was commanded
        error = observed_state - commanded_state

        # Hebbian update rule: strengthen connections based on correlation
        delta_weights_ho = self.learning_rate * np.outer(error, hidden_activations)
        self.weights_ho += delta_weights_ho

        # A simplified update for the input-to-hidden weights
        delta_weights_ih = self.learning_rate * np.outer(np.dot(self.weights_ho.T, error), input_vector)
        self.weights_ih += delta_weights_ih

    def get_nn_as_json(self):
        """
        Returns the neural network weights in the JSON format expected by the ESP32.
        """
        # This is a simplified representation. A real implementation would need to match
        # the exact structure of the HiddenLayer, OutputLayer, and PredictionLayer structs.
        return {
            "hidden_layer": {
                "weights": self.weights_ih.tolist(),
                "biases": [0.1] * self.weights_ih.shape[0] # Dummy biases
            },
            "output_layer": {
                "weights": self.weights_ho.tolist(),
                "biases": [0.1] * self.weights_ho.shape[0] # Dummy biases
            },
            "prediction_layer": { # Dummy prediction layer
                "weights": [[0.0]],
                "biases": [0.0]
            }
        }

class MCPServer:
    def __init__(self, host='0.0.0.0', port=8888):
        print("[MCP] Initializing server...")
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        print("[MCP] Socket created and configured.")
        self.hebbian_learner = HebbianLearner(input_size=10, hidden_size=16, output_size=3)
        print("[MCP] Hebbian learner initialized.")
        self.tools = {
            "list_tools": self.list_tools,
            "record_video": self.record_video,
            "analyze_joint_positions": self.analyze_joint_positions,
            "import_nn_json": self.import_nn_json,
            "learn": self.learn,
            "collect_kinematic_data": self.collect_kinematic_data,
            "train_kinematic_model": self.train_kinematic_model,
            "estimate_state_from_sensors": self.estimate_state_from_sensors,
        }

    def list_tools(self, args):
        return {"tools": [{"name": name} for name in self.tools.keys()]}

    def record_video(self, args):
        output_filename = args.get("output_filename", "video.avi")
        duration = args.get("duration", 5)
        camera_index = args.get("camera_index", 0)

        print(f"[MCP] Recording video to {output_filename} for {duration} seconds...")
        cap = cv2.VideoCapture(camera_index)
        if not cap.isOpened():
            print("[MCP] ERROR: Could not open camera.")
            return {"error": "Could not open camera"}

        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter(output_filename, fourcc, 20.0, (640, 480))

        start_time = time.time()
        while (time.time() - start_time) < duration:
            ret, frame = cap.read()
            if ret:
                out.write(frame)
            else:
                break

        cap.release()
        out.release()
        print(f"[MCP] Video recording complete.")
        return {"result": "OK", "output_filename": output_filename}

    def analyze_joint_positions(self, args):
        video_filename = args.get("video_filename")
        if not video_filename:
            return {"error": "video_filename not provided"}

        print(f"[MCP] Analyzing joint positions in {video_filename}...")
        # In a real application, this would involve a computer vision model.
        # For this example, we'll just return some mock data.
        mock_joint_positions = {
            "joint1": 0.5,
            "joint2": -0.2,
            "joint3": 0.8,
        }
        print(f"[MCP] Analysis complete.")
        return {"result": "OK", "joint_positions": mock_joint_positions}

    def import_nn_json(self, args):
        nn_data = args.get("nn_data")
        if not nn_data:
            return {"error": "nn_data not provided"}

        output_filename = "imported_nn.json"
        print(f"[MCP] Importing neural network to {output_filename}...")
        with open(output_filename, 'w') as f:
            json.dump(nn_data, f, indent=4)

        print(f"[MCP] Neural network import complete.")
        return {"result": "OK"}

    def learn(self, args):
        commanded_positions = args.get("commanded_positions")
        sensor_data = args.get("sensor_data")
        model_filename = args.get("model_filename", "kinematic_model.pt")

        if not commanded_positions or not sensor_data:
            return {"error": "commanded_positions or sensor_data not provided"}

        # Get observed positions from the SNN model
        estimation_args = {"model_filename": model_filename, "sensor_data": sensor_data}
        estimation_result = self.estimate_state_from_sensors(estimation_args)

        if estimation_result.get("error"):
            return estimation_result

        observed_positions = estimation_result["estimated_positions"]

        # Convert to numpy arrays
        commanded_state = np.array(list(commanded_positions.values()))
        observed_state = np.array(observed_positions)

        # Dummy input vector
        dummy_input = np.random.randn(10)

        self.hebbian_learner.update_weights(dummy_input, commanded_state, observed_state)

        updated_nn_json = self.hebbian_learner.get_nn_as_json()

        return {"result": "OK", "updated_nn": updated_nn_json}

    def collect_kinematic_data(self, args):
        num_samples = args.get("num_samples", 100)
        output_filename = args.get("output_filename", "kinematic_data.json")

        print(f"[MCP] Collecting {num_samples} samples of kinematic data...")

        data = []
        for i in range(num_samples):
            # Mock commanded positions
            commanded_positions = {
                "joint1": np.random.uniform(-1, 1),
                "joint2": np.random.uniform(-1, 1),
                "joint3": np.random.uniform(-1, 1),
            }

            # Mock observed feedback
            observed_feedback = {
                "position": {
                    "joint1": commanded_positions["joint1"] + np.random.normal(0, 0.05),
                    "joint2": commanded_positions["joint2"] + np.random.normal(0, 0.05),
                    "joint3": commanded_positions["joint3"] + np.random.normal(0, 0.05),
                },
                "current": {
                    "joint1": np.random.uniform(0.5, 1.5),
                    "joint2": np.random.uniform(0.5, 1.5),
                    "joint3": np.random.uniform(0.5, 1.5),
                },
                "voltage": {
                    "joint1": 12.0,
                    "joint2": 12.0,
                    "joint3": 12.0,
                }
            }

            # Mock accelerometer data
            accelerometer_data = {
                "x": np.random.normal(0, 0.1),
                "y": np.random.normal(0, 0.1),
                "z": np.random.normal(9.8, 0.1),
            }

            # Mock camera image (just the path)
            image_path = f"images/frame_{i}.jpg"

            data.append({
                "commanded_positions": commanded_positions,
                "observed_feedback": observed_feedback,
                "accelerometer_data": accelerometer_data,
                "image_path": image_path,
            })

        with open(output_filename, 'w') as f:
            json.dump(data, f, indent=4)

        print(f"[MCP] Kinematic data collection complete. Saved to {output_filename}")
        return {"result": "OK", "output_filename": output_filename}

    def train_kinematic_model(self, args):
        data_filename = args.get("data_filename", "kinematic_data.json")
        model_filename = args.get("model_filename", "kinematic_model.pt")
        num_epochs = args.get("num_epochs", 10)

        print(f"[MCP] Training kinematic model with data from {data_filename}...")

        # Load data
        with open(data_filename, 'r') as f:
            data = json.load(f)

        # Process data (this is a simplified example)
        input_data = []
        target_data = []
        for sample in data:
            # Combine all sensor data into a single input vector
            input_vector = []
            input_vector.extend(sample["commanded_positions"].values())
            input_vector.extend(sample["observed_feedback"]["position"].values())
            input_vector.extend(sample["observed_feedback"]["current"].values())
            input_vector.extend(sample["accelerometer_data"].values())
            input_data.append(input_vector)

            # Target is the observed joint positions
            target_vector = list(sample["observed_feedback"]["position"].values())
            target_data.append(target_vector)

        input_tensor = torch.tensor(input_data, dtype=torch.float)
        target_tensor = torch.tensor(target_data, dtype=torch.float)

        # Initialize model, loss, and optimizer
        input_size = input_tensor.shape[1]
        hidden_size = 32
        output_size = target_tensor.shape[1]
        model = KinematicModel(input_size, hidden_size, output_size)
        loss_fn = nn.MSELoss()
        optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)

        # Training loop
        for epoch in range(num_epochs):
            spk_rec, mem_rec = model(input_tensor)
            loss = loss_fn(mem_rec.sum(dim=0), target_tensor) # Using membrane potential as output

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            if (epoch + 1) % 10 == 0:
                print(f"Epoch [{epoch+1}/{num_epochs}], Loss: {loss.item():.4f}")

        # Save the trained model
        torch.save(model.state_dict(), model_filename)

        print(f"[MCP] Kinematic model training complete. Saved to {model_filename}")
        return {"result": "OK", "model_filename": model_filename}

    def estimate_state_from_sensors(self, args):
        model_filename = args.get("model_filename", "kinematic_model.pt")
        sensor_data = args.get("sensor_data")

        if not sensor_data:
            return {"error": "sensor_data not provided"}

        print(f"[MCP] Estimating state from sensors using model {model_filename}...")

        # Load model
        input_size = 12 # This should be determined from the data, but hardcoded for now
        hidden_size = 32
        output_size = 3
        model = KinematicModel(input_size, hidden_size, output_size)
        model.load_state_dict(torch.load(model_filename))
        model.eval()

        # Process sensor data
        input_vector = []
        input_vector.extend(sensor_data["commanded_positions"].values())
        input_vector.extend(sensor_data["observed_feedback"]["position"].values())
        input_vector.extend(sensor_data["observed_feedback"]["current"].values())
        input_vector.extend(sensor_data["accelerometer_data"].values())
        input_tensor = torch.tensor(input_vector, dtype=torch.float).unsqueeze(0) # Add batch dimension

        # Run inference
        spk_rec, mem_rec = model(input_tensor)
        estimated_positions_tensor = mem_rec.sum(dim=0).squeeze(0) # Squeeze batch dimension
        estimated_positions = estimated_positions_tensor.tolist()

        print(f"[MCP] State estimation complete.")
        return {"result": "OK", "estimated_positions": estimated_positions}

    def handle_client(self, conn, addr):
        print(f"[MCP] Accepted connection from {addr}")
        with conn:
            while True:
                request = recv_json(conn)
                if request is None:
                    break

                print(f"[MCP] Received: {request}")

                command = request.get("command")
                tool_name = request.get("tool_name")
                arguments = request.get("arguments")

                response = None
                if command == "list_tools":
                    response = self.list_tools(arguments)
                elif command == "call_tool" and tool_name in self.tools:
                    response = self.tools[tool_name](arguments)
                else:
                    response = {"error": "Unknown command or tool"}

                if response:
                    send_json(conn, response)
        print(f"[MCP] Connection from {addr} closed")

    def start(self):
        self.sock.bind((self.host, self.port))
        self.sock.listen()
        print(f"[MCP] Server listening on {self.host}:{self.port}")
        try:
            while True:
                conn, addr = self.sock.accept()
                client_thread = threading.Thread(target=self.handle_client, args=(conn, addr))
                client_thread.daemon = True
                client_thread.start()
        except KeyboardInterrupt:
            print("\n[MCP] Server shutting down...")
        finally:
            self.sock.close()

if __name__ == "__main__":
    try:
        server = MCPServer()
        server.start()
    except Exception as e:
        print(f"Error starting server: {e}")
