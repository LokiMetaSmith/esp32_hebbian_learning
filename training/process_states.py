import numpy as np
import json
from sklearn.cluster import KMeans

# --- Configuration ---
# Must match PRED_NEURONS in main.h
NUM_STATE_DIMS = 24
# Choose how many "tokens" you want to discover
NUM_STATE_TOKENS = 16

# Load the data exported from the ESP32
try:
    data = np.loadtxt('states.csv', delimiter=',')
    # Remove the last column if it's all NaNs from trailing commas
    if np.isnan(data[:, -1]).all():
        data = data[:, :-1]
except Exception as e:
    print(f"Error loading states.csv: {e}")
    exit()

if data.shape[1] != NUM_STATE_DIMS:
    print(f"Error: Data has {data.shape[1]} dimensions, but expected {NUM_STATE_DIMS}.")
    exit()

print(f"Loaded {data.shape[0]} state samples.")

# Perform K-Means clustering
print(f"Performing K-Means clustering to find {NUM_STATE_TOKENS} tokens...")
kmeans = KMeans(n_clusters=NUM_STATE_TOKENS, random_state=0, n_init='auto').fit(data)

# The cluster centers are our state tokens
centroids = kmeans.cluster_centers_

# Prepare JSON output to be pasted back into the ESP32 console
output_json = {"centroids": centroids.tolist()}

print("\n--- COPY AND PASTE THE FOLLOWING JSON INTO THE CONSOLE ---")
# Use a compact format for easier pasting
print(f"import_states {json.dumps(output_json, separators=(',', ':'))}")
print("--- END JSON ---")
