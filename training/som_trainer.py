import numpy as np
import json
from minisom import MiniSom
import matplotlib.pyplot as plt

# --- Configuration ---
# Must match PRED_NEURONS in main.h
NUM_STATE_DIMS = 24
# SOM Grid size (e.g., 4x4 = 16 tokens)
SOM_GRID_X = 4
SOM_GRID_Y = 4
NUM_STATE_TOKENS = SOM_GRID_X * SOM_GRID_Y

def train_som(data_file='states.csv'):
    # Load the data exported from the ESP32
    try:
        data = np.loadtxt(data_file, delimiter=',')
        # Remove the last column if it's all NaNs from trailing commas
        if np.isnan(data[:, -1]).all():
            data = data[:, :-1]
    except Exception as e:
        print(f"Error loading {data_file}: {e}")
        # Create dummy data if file not found for prototyping
        print("Generating dummy data for prototyping...")
        data = np.random.rand(100, NUM_STATE_DIMS)

    if data.shape[1] != NUM_STATE_DIMS:
        print(f"Error: Data has {data.shape[1]} dimensions, but expected {NUM_STATE_DIMS}.")
        return

    print(f"Loaded {data.shape[0]} state samples.")

    # Data normalization
    data = (data - np.mean(data, axis=0)) / np.std(data, axis=0)

    # Initialize and train SOM
    print(f"Training {SOM_GRID_X}x{SOM_GRID_Y} SOM...")
    som = MiniSom(SOM_GRID_X, SOM_GRID_Y, NUM_STATE_DIMS, sigma=1.0, learning_rate=0.5)
    som.random_weights_init(data)
    som.train_random(data, 1000) # 1000 iterations

    # The weights are our state centroids
    # Reshape (X, Y, Dims) to (X*Y, Dims)
    centroids = som.get_weights().reshape(-1, NUM_STATE_DIMS)

    # Prepare JSON output
    output_json = {"centroids": centroids.tolist()}

    print("\n--- COPY AND PASTE THE FOLLOWING JSON INTO THE CONSOLE ---")
    print(f"import_states {json.dumps(output_json, separators=(',', ':'))}")
    print("--- END JSON ---")

    # Optional: Visualize the U-Matrix (distance between neighbor neurons)
    plt.figure(figsize=(7, 7))
    plt.pcolor(som.distance_map().T, cmap='bone_r')
    plt.colorbar()
    plt.title('SOM U-Matrix')
    plt.savefig('som_umatrix.png')
    print("U-Matrix saved to som_umatrix.png")

if __name__ == '__main__':
    train_som()
