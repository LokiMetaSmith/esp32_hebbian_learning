import http.server
import socketserver
import json
import os
try:
    import numpy as np
    from sklearn.linear_model import LinearRegression
    HAS_ML = True
except ImportError:
    print("Warning: numpy or sklearn not found. Learning features disabled.")
    HAS_ML = False
import threading
import time

PORT = 8000
DATA_FILE = "dynamics_data.json"

class DynamicsHandler(http.server.SimpleHTTPRequestHandler):
    def do_POST(self):
        if self.path == '/dynamics':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)

            try:
                data = json.loads(post_data)
                self.process_data(data)

                response = {"status": "OK", "message": "Data received"}
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps(response).encode('utf-8'))
            except Exception as e:
                print(f"Error processing POST: {e}")
                self.send_response(500)
                self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()

    def process_data(self, payload):
        # Payload structure: {"data": [{"action": [...], "state": [...]}, ...]}
        new_samples = payload.get("data", [])
        print(f"Received {len(new_samples)} samples.")

        # Save to file
        all_data = []
        if os.path.exists(DATA_FILE):
            with open(DATA_FILE, 'r') as f:
                try:
                    all_data = json.load(f)
                except:
                    pass

        all_data.extend(new_samples)

        with open(DATA_FILE, 'w') as f:
            json.dump(all_data, f)

        # Perform Learning
        self.learn_dynamics(all_data)

    def learn_dynamics(self, data):
        if not HAS_ML:
            return

        # Extract X (Action/Torque) and Y (State/Acceleration)

        if not data:
            return

        X = []
        y = []

        for sample in data:
            action = sample.get("action")
            state = sample.get("state")
            if action and state:
                # Use all action dims as input
                X.append(action)
                # Use state dims (Accel X, Y, Z) as target
                # For regression we predict one variable or multi-output.
                # Let's predict Accel X (state[0])
                if len(state) > 0:
                    y.append(state[0])

        if len(X) > 10:
            try:
                X_np = np.array(X)
                y_np = np.array(y)

                model = LinearRegression()
                model.fit(X_np, y_np)

                score = model.score(X_np, y_np)
                print(f"--- Dynamics Model (Linear Regression: Action -> AccelX) ---")
                print(f"Samples: {len(X)}")
                print(f"R^2 Score: {score:.4f}")
                print(f"Coefficients: {model.coef_}")
                print(f"Intercept: {model.intercept_}")
                print("-----------------------------------------------------------")
            except Exception as e:
                print(f"Learning Error: {e}")

def run_server():
    # Allow address reuse to prevent "Address already in use" errors during testing
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("", PORT), DynamicsHandler) as httpd:
        print(f"Dynamics Learner Server running on port {PORT}")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            pass
        print("Server stopped.")

if __name__ == "__main__":
    # Create empty data file if not exists
    if not os.path.exists(DATA_FILE):
        with open(DATA_FILE, 'w') as f:
            json.dump([], f)

    run_server()
