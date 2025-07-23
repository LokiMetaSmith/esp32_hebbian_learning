import socket
import json
import threading
import cv2
import time
import numpy as np
from socket_utils import send_json, recv_json

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
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.hebbian_learner = HebbianLearner(input_size=10, hidden_size=16, output_size=3)
        self.tools = {
            "list_tools": self.list_tools,
            "record_video": self.record_video,
            "analyze_joint_positions": self.analyze_joint_positions,
            "import_nn_json": self.import_nn_json,
            "learn": self.learn,
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
        observed_positions = args.get("observed_positions")
        if not commanded_positions or not observed_positions:
            return {"error": "commanded_positions or observed_positions not provided"}

        # Convert to numpy arrays
        commanded_state = np.array(list(commanded_positions.values()))
        observed_state = np.array(list(observed_positions.values()))

        # Dummy input vector
        dummy_input = np.random.randn(10)

        self.hebbian_learner.update_weights(dummy_input, commanded_state, observed_state)

        updated_nn_json = self.hebbian_learner.get_nn_as_json()

        return {"result": "OK", "updated_nn": updated_nn_json}

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
    server = MCPServer()
    server.start()
