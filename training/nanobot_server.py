import http.server
import socketserver
import json
import os
try:
    import numpy as np
    from sklearn.linear_model import LinearRegression
    from sklearn.cluster import KMeans
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    HAS_ML = True
except ImportError:
    print("Warning: numpy, sklearn, or matplotlib not found. Learning features disabled.")
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
                calibration = self.process_data(data)

                response = {"status": "OK", "message": "Data received"}
                if calibration:
                    response["calibration"] = calibration

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
        calibration = self.learn_dynamics(all_data)

        # Check if we should re-cluster (Adaptive Mapping)
        if new_samples:
            last_sample = new_samples[-1]
            dissonance = last_sample.get("dissonance", 0)
            if dissonance > 0.3: # Threshold
                print(f"High Dissonance detected: {dissonance}. Re-clustering state space...")
                centroids = self.learn_clusters(all_data)
                if centroids:
                    if not calibration: calibration = {}
                    calibration["centroids"] = centroids

        return calibration

    def learn_clusters(self, data):
        if not HAS_ML or not data: return None
        states = []
        for s in data:
            if "state" in s and len(s["state"]) > 0: states.append(s["state"])

        if len(states) < 16: return None

        try:
            X = np.array(states)
            # Use min(16, samples) clusters
            n_clusters = min(16, len(states))
            kmeans = KMeans(n_clusters=n_clusters, n_init=10)
            kmeans.fit(X)
            centroids = kmeans.cluster_centers_.tolist()
            print(f"New Centroids generated ({len(centroids)}).")
            return centroids
        except Exception as e:
            print(f"Clustering Error: {e}")
            return None

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

                # Plotting
                plt.figure(figsize=(10, 6))
                # Using Dim 0 vs Dim 0 for visualization
                plt.scatter(X_np[:, 0], y_np, alpha=0.5, label='Data')

                # Plot regression line
                x_range = np.linspace(X_np[:, 0].min(), X_np[:, 0].max(), 100).reshape(-1, 1)
                # We need to pad x_range to match input dim if > 1
                if X_np.shape[1] > 1:
                    # Simple visualization only works well for 1D.
                    # We'll just plot the predictions on the actual X points sorted.
                    sorted_idx = np.argsort(X_np[:, 0])
                    plt.plot(X_np[sorted_idx, 0], model.predict(X_np)[sorted_idx], color='red', label='Linear Fit')
                else:
                    plt.plot(x_range, model.predict(x_range), color='red', label='Linear Fit')

                plt.title("Dynamics: Input Torque (Dim 0) vs Output Accel (Dim 0)")
                plt.xlabel("Input Torque (Action[0])")
                plt.ylabel("Output Acceleration (State[0])")
                plt.grid(True)
                plt.legend()
                plt.savefig("dynamics_plot.png")
                plt.close()
                print("Plot saved to dynamics_plot.png")

                # Calibration calculation
                slope = model.coef_[0]
                intercept = model.intercept_
                cal_gain = 1.0 / slope if abs(slope) > 1e-5 else 1.0
                cal_offset = -intercept / slope if abs(slope) > 1e-5 else 0.0
                return {"gain": cal_gain, "offset": cal_offset}

            except Exception as e:
                print(f"Learning/Plotting Error: {e}")
                return None
        return None

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
