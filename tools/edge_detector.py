import torch
import sinabs
import samna
import numpy as np
import time

def create_edge_detector_network():
    """
    Creates the SNN for edge detection using Sobel filters.
    """
    # Define the Sobel kernels
    sobel_x = torch.tensor([[-1., 0., 1.], [-2., 0., 2.], [-1., 0., 1.]])
    sobel_y = torch.tensor([[-1., -2., -1.], [0., 0., 0.], [1., 2., 1.]])

    # Create a convolutional layer with 8 filters (4 orientations, ON/OFF)
    num_filters = 8
    conv = torch.nn.Conv2d(1, num_filters, kernel_size=3, stride=1, padding=1, bias=False)

    # Set the weights of the filters
    conv.weight.data[0, 0, :, :] = sobel_x
    conv.weight.data[1, 0, :, :] = -sobel_x
    conv.weight.data[2, 0, :, :] = sobel_y
    conv.weight.data[3, 0, :, :] = -sobel_y
    conv.weight.data[4, 0, :, :] = (sobel_x + sobel_y) / 2
    conv.weight.data[5, 0, :, :] = -(sobel_x + sobel_y) / 2
    conv.weight.data[6, 0, :, :] = (sobel_x - sobel_y) / 2
    conv.weight.data[7, 0, :, :] = -(sobel_x - sobel_y) / 2

    # Create the sinabs network
    snn = torch.nn.Sequential(
        sinabs.layers.SumPool(
            kernel_size=(16, 16),
            stride=(16, 16),
            spiking=True
        ),
        conv,
        sinabs.layers.IAF()
    )
    return snn

def main():
    print("--- Starting Edge Detector Application ---")

    # 1. Create the SNN model
    edge_detector_snn = create_edge_detector_network()
    print("SNN model created.")

    # 2. Connect to the Speck device
    try:
        device = samna.speck.open_device()
        print("Successfully connected to Speck device.")
    except Exception as e:
        print(f"Error: Could not connect to Speck device: {e}")
        return

    # 3. Map the SNN to the hardware configuration
    config = samna.speck.map_model(edge_detector_snn)
    print("SNN model mapped to hardware configuration.")

    # 4. Apply the configuration to the device
    samna.speck.apply_configuration(device, config)
    print("Configuration applied to device.")

    # 5. Start the inference loop
    print("\n--- Starting real-time inference. Press Ctrl+C to stop. ---")
    orientations = {
        0: "Horizontal", 1: "Horizontal (neg)",
        2: "Vertical",   3: "Vertical (neg)",
        4: "Diagonal 45", 5: "Diagonal 45 (neg)",
        6: "Diagonal 135", 7: "Diagonal 135 (neg)"
    }
    try:
        while True:
            # Read the classification index from the device
            # This part is a placeholder as the exact API call is unknown.
            # It might be something like `samna.speck.get_readout(device)`
            # For now, we'll simulate it.
            time.sleep(0.5)
            mock_index = np.random.randint(0, 8)
            print(f"Detected edge: {orientations.get(mock_index, 'Unknown')}")

    except KeyboardInterrupt:
        print("\n--- Stopping inference ---")
    finally:
        # 6. Close the device
        samna.speck.close_device(device)
        print("Device closed.")

if __name__ == "__main__":
    main()
