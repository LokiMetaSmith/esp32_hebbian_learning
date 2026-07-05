import numpy as np
import json
from sklearn.cluster import KMeans
from minisom import MiniSom
import argparse

# --- Configuration ---
# Must match PRED_NEURONS in main.h
NUM_STATE_DIMS = 24
# Choose how many "tokens" you want to discover
NUM_STATE_TOKENS = 16

# Load the data exported from the ESP32
def load_data(filename):
    try:
        data = np.loadtxt(filename, delimiter=',')
        # Remove the last column if it's all NaNs from trailing commas
        if len(data.shape) > 1 and np.isnan(data[:, -1]).all():
            data = data[:, :-1]
        return data
    except Exception as e:
        print(f"Error loading {filename}: {e}")
        return None

# Main pipeline
def main():
    parser = argparse.ArgumentParser(description="Process robot states into tokens.")
    parser.add_argument("--method", choices=['kmeans', 'som'], default='som', help="Clustering method to use.")
    parser.add_argument("--file", default='states.csv', help="Input CSV file.")
    args = parser.parse_args()

    data = load_data(args.file)
    if data is None:
        print("Generating dummy data for demo...")
        data = np.random.rand(100, NUM_STATE_DIMS)

    if data.shape[1] != NUM_STATE_DIMS:
        print(f"Error: Data has {data.shape[1]} dimensions, but expected {NUM_STATE_DIMS}.")
        exit()

    print(f"Loaded {data.shape[0]} state samples.")

    if args.method == 'kmeans':
        print(f"Performing K-Means clustering to find {NUM_STATE_TOKENS} tokens...")
        kmeans = KMeans(n_clusters=NUM_STATE_TOKENS, random_state=0, n_init='auto').fit(data)
        centroids = kmeans.cluster_centers_
    else:
        print(f"Performing SOM clustering to find {NUM_STATE_TOKENS} tokens...")
        # SOM Grid size (e.g., 4x4 = 16 tokens)
        grid_size = int(np.sqrt(NUM_STATE_TOKENS))
        # Data normalization for SOM
        data_mean = np.mean(data, axis=0)
        data_std = np.std(data, axis=0) + 1e-8
        data_norm = (data - data_mean) / data_std

        som = MiniSom(grid_size, grid_size, NUM_STATE_DIMS, sigma=1.0, learning_rate=0.5)
        som.random_weights_init(data_norm)
        som.train_random(data_norm, 1000)

        # Reshape and denormalize centroids
        centroids_norm = som.get_weights().reshape(-1, NUM_STATE_DIMS)
        centroids = centroids_norm * data_std + data_mean

    # Prepare JSON output to be pasted back into the ESP32 console
    output_json = {"centroids": centroids.tolist()}

    print("\n--- COPY AND PASTE THE FOLLOWING JSON INTO THE CONSOLE ---")
    # Use a compact format for easier pasting
    print(f"import_states {json.dumps(output_json, separators=(',', ':'))}")
    print("--- END JSON ---")

if __name__ == "__main__":
    main()
