import argparse
import sys
import os
import json
import matplotlib.pyplot as plt

# Add the parent directory of 'tests' to the Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from tests.robot_client import ConsoleClient, MCPClient

def main():
    parser = argparse.ArgumentParser(description="Offline Training Pipeline for Hebbian Robot")
    parser.add_argument("--ip_address", required=True, help="IP address of the ESP32 device.")
    parser.add_argument("--console_port", required=True, help="Serial port for the Console CLI.")
    parser.add_argument("--plot-energy", action="store_true", help="Plot the average energy consumption per trajectory.")
    args = parser.parse_args()

    print("--- Starting Offline Training Pipeline ---")

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

    trajectories = []
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

    # --- Plot Energy Consumption ---
    if args.plot_energy:
        avg_currents = []
        for traj in trajectories:
            total_current = 0
            num_waypoints = 0
            for waypoint in traj:
                # The current is the last value in the sensor data
                total_current += waypoint[-1]
                num_waypoints += 1
            avg_currents.append(total_current / num_waypoints)

        plt.figure()
        plt.plot(avg_currents)
        plt.xlabel("Trajectory")
        plt.ylabel("Average Current (A)")
        plt.title("Average Current per Trajectory")
        plt.show()


    # --- Disconnect ---
    mcp_client.disconnect()
    console_client.disconnect()

    print("\n--- Training Pipeline Finished ---")


if __name__ == "__main__":
    main()
