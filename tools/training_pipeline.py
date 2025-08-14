import argparse
import sys
import os
import json
import matplotlib.pyplot as plt
import numpy as np
from dtaidistance import dtw
from sklearn.cluster import KMeans

# Add the parent directory of 'tests' to the Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from tests.robot_client import ConsoleClient, MCPClient

def generate_dummy_data():
    """Generates a default gesture library and graph for offline use."""
    print("--- Generating Dummy Gesture Data ---")

    # Define a simple, single gesture
    gesture_library = [{
        "id": 0,
        "num_waypoints": 1,
        "energy_cost": 0.0,
        "embedding": [0.0] * 16, # Assuming embedding size of 16
        "waypoints": [
            [0.0] * 12 # Assuming 6 positions and 6 velocities
        ]
    }]

    # Define a simple graph with one token
    gesture_graph = {
        "num_tokens": 1,
        "transition_costs": [[0.0]]
    }

    print("--- Dummy data generated. ---")
    return gesture_library, gesture_graph

def main():
    parser = argparse.ArgumentParser(description="Offline Training Pipeline for Hebbian Robot")
    parser.add_argument("--ip_address", required=True, help="IP address of the ESP32 device.")
    parser.add_argument("--console_port", required=True, help="Serial port for the Console CLI.")
    parser.add_argument("--plot-energy", action="store_true", help="Plot the average energy consumption per trajectory.")
    args = parser.parse_args()

    print("--- Starting Offline Training Pipeline ---")

    gesture_library = []
    gesture_graph = {}
    trajectories = []

    # If using a dummy IP, generate dummy data instead of connecting to the robot
    if args.ip_address == "127.0.0.1":
        gesture_library, gesture_graph = generate_dummy_data()
    else:
        # Initialize clients
        mcp_client = MCPClient(args.ip_address, 8888)
        console_client = ConsoleClient(args.console_port)

        # Connect to the robot
        if not mcp_client.connect():
            print("ERROR: Could not connect to MCP server. Exiting.")
            sys.exit(1)

        if not console_client.connect():
            print("ERROR: Could not connect to console. Exiting.")
            mcp_client.disconnect()
            sys.exit(1)

        print("\n--- Successfully connected to robot ---")

        # --- Data Acquisition ---
        print("\n--- Starting Data Acquisition ---")

        console_client.send_command("start-data-acq")

        print("--- Waiting for trajectory data... ---")

        current_trajectory = []
        in_trajectory = False

        while True:
            line = console_client.ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            if "--- BEGIN TRAJECTORY DATA ---" in line:
                print("--- Started receiving data ---")
                continue

            if "--- END TRAJECTORY DATA ---" in line:
                print("--- Finished receiving data ---")
                break

            if "TRAJ_START" in line:
                current_trajectory = []
                in_trajectory = True
                continue

            if "TRAJ_END" in line:
                if in_trajectory:
                    trajectories.append(current_trajectory)
                    in_trajectory = False
                continue

            if in_trajectory:
                try:
                    waypoint = [float(x) for x in line.split(',') if x]
                    current_trajectory.append(waypoint)
                except ValueError:
                    print(f"Warning: Could not parse line: {line}")

        # --- Save Data ---
        output_filename = "trajectories.json"
        with open(output_filename, "w") as f:
            json.dump(trajectories, f, indent=2)
        print(f"\n--- Saved {len(trajectories)} trajectories to {output_filename} ---")

        # --- Clustering ---
        print("\n--- Clustering Trajectories ---")
        if not trajectories:
            print("No trajectories to cluster. Exiting.")
            sys.exit(1)

        # Convert trajectories to numpy arrays for DTW
        trajectories_np = [np.array(t) for t in trajectories]

        # Calculate the DTW distance matrix
        distance_matrix = dtw.distance_matrix_fast(trajectories_np)

        # Perform K-Means clustering
        num_clusters = 16 # This should match NUM_STATE_TOKENS in the C code
        kmeans = KMeans(n_clusters=num_clusters, random_state=0, n_init=10).fit(distance_matrix)

        print(f"--- Clustering complete. Found {len(kmeans.cluster_centers_)} clusters. ---")

        # --- Generate Gesture Library ---
        print("\n--- Generating Gesture Library ---")
        for i in range(num_clusters):
            cluster_indices = np.where(kmeans.labels_ == i)[0]
            if len(cluster_indices) == 0:
                continue

            # Find the medoid trajectory for this cluster
            cluster_distances = distance_matrix[cluster_indices][:, cluster_indices]
            medoid_index_in_cluster = np.argmin(cluster_distances.sum(axis=0))
            medoid_index_in_dataset = cluster_indices[medoid_index_in_cluster]
            centroid_traj = trajectories_np[medoid_index_in_dataset]
            embedding = centroid_traj[0, :16].tolist()

            gesture_token = {
                "id": i,
                "waypoints": centroid_traj.tolist(),
                "num_waypoints": len(centroid_traj),
                "energy_cost": np.sum(centroid_traj[:, -1]),
                "embedding": embedding
            }
            gesture_library.append(gesture_token)
        print(f"--- Generated {len(gesture_library)} gestures. ---")

        # --- Generate Gesture Graph ---
        print("\n--- Generating Gesture Graph ---")
        gesture_graph = {
            "num_tokens": len(gesture_library),
            "transition_costs": np.full((len(gesture_library), len(gesture_library)), -1.0).tolist()
        }
        for i, gesture_i in enumerate(gesture_library):
            for j, gesture_j in enumerate(gesture_library):
                if i == j:
                    gesture_graph["transition_costs"][i][j] = 0.0
                else:
                    end_pose_i = np.array(gesture_i["waypoints"][-1])
                    start_pose_j = np.array(gesture_j["waypoints"][0])
                    dist = np.linalg.norm(end_pose_i[:6] - start_pose_j[:6])
                    gesture_graph["transition_costs"][i][j] = dist
        print("--- Gesture graph generated with calculated transition costs. ---")

        # --- Disconnect ---
        mcp_client.disconnect()
        console_client.disconnect()


    # --- Export to C Header ---
    if gesture_library:
        output_header_filename = "main/generated_gestures.h"
        print(f"\n--- Exporting to {output_header_filename} ---")
        with open(output_header_filename, "w") as f:
            f.write("#ifndef GENERATED_GESTURES_H\n")
            f.write("#define GENERATED_GESTURES_H\n\n")
            f.write('#include "planner.h"\n\n')

            # Write gesture library
            f.write("static GestureToken g_gesture_library[] = {\n")
            for token in gesture_library:
                f.write(f"  {{ // Gesture {token['id']}\n")
                f.write(f"    .id = {token['id']},\n")
                f.write(f"    .num_waypoints = {token['num_waypoints']},\n")
                f.write(f"    .energy_cost = {token['energy_cost']:.4f}f,\n")
                f.write("    .embedding = {")
                f.write(", ".join([f"{x:.4f}f" for x in token['embedding']]))
                f.write("},\n")
                f.write("    .waypoints = {\n")
                for waypoint in token['waypoints']:
                    f.write("      {")
                    f.write("{")
                    f.write(", ".join([f"{p:.4f}f" for p in waypoint[:6]])) # Positions
                    f.write("}, {")
                    f.write(", ".join([f"{v:.4f}f" for v in waypoint[6:12]])) # Velocities
                    f.write("}")
                    f.write("},\n")
                f.write("    }\n")
                f.write("  },\n")
            f.write("};\n\n")

            # Write gesture graph
            f.write("static GestureGraph g_gesture_graph = {\n")
            f.write(f"  .num_tokens = {gesture_graph['num_tokens']},\n")
            f.write("  .transition_costs = {\n")
            for row in gesture_graph['transition_costs']:
                f.write("    {")
                f.write(", ".join([f"{c:.4f}f" for c in row]))
                f.write("},\n")
            f.write("  },\n")
            f.write("  .gesture_library = g_gesture_library\n")
            f.write("};\n\n")

            f.write("#endif // GENERATED_GESTURES_H\n")

        print("--- Export complete. ---")


    # --- Plot Energy Consumption ---
    if args.plot_energy and trajectories:
        avg_currents = []
        for traj in trajectories:
            total_current = 0
            num_waypoints = 0
            for waypoint in traj:
                total_current += waypoint[-1]
                num_waypoints += 1
            avg_currents.append(total_current / num_waypoints)

        plt.figure()
        plt.plot(avg_currents)
        plt.xlabel("Trajectory")
        plt.ylabel("Average Current (A)")
        plt.title("Average Current per Trajectory")
        plt.show()

    print("\n--- Training Pipeline Finished ---")


if __name__ == "__main__":
    main()
