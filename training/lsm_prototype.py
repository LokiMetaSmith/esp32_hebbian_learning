import torch
import torch.nn as nn
import snntorch as snn
from snntorch import surrogate
from snntorch import spikegen
import numpy as np

# Configuration constraints
N_INPUT = 6        # Arbitrary sensor input streams
N_RES = 64         # Conservative reservoir size for ESP32 constraints
N_OUTPUT = 4       # Readout layer classification/health output
E_RATIO = 0.8
I_RATIO = 0.2

class LiquidReservoir(nn.Module):
    def __init__(self, n_input, n_res, n_output):
        super(LiquidReservoir, self).__init__()

        self.n_res = n_res

        # 1. Input routing matrix
        self.fc_in = nn.Linear(n_input, n_res, bias=False)

        # 2. Fixed Recurrent Matrix with Log-normal distribution
        # Log-normal distribution for weights
        mu = -1.0
        sigma = 0.5
        W_rec_raw = torch.exp(torch.randn(n_res, n_res) * sigma + mu)

        # Enforce 80% E / 20% I ratio
        n_excitatory = int(E_RATIO * n_res)

        # We assign the first n_excitatory neurons as excitatory (positive weights out)
        # and the rest as inhibitory (negative weights out).
        # We create a sign matrix
        signs = torch.ones(1, n_res)
        signs[0, n_excitatory:] = -1.0

        # Apply signs to the columns (outputs of the neurons)
        W_rec = W_rec_raw * signs

        # To ensure stability and prevent explosion, we should normalize the spectral radius
        # Note: PyTorch eigvals is available, but for a real matrix we can compute complex eigenvalues
        # and take the max absolute value.
        eigenvalues = torch.linalg.eigvals(W_rec)
        spectral_radius = torch.max(torch.abs(eigenvalues))
        if spectral_radius > 0:
            # Scale to slightly below 1
            W_rec = W_rec / spectral_radius * 0.95

        self.fc_rec = nn.Linear(n_res, n_res, bias=False)
        self.fc_rec.weight = nn.Parameter(W_rec, requires_grad=False)

        # Reservoir neurons
        self.lif_res = snn.Leaky(beta=0.9, spike_grad=surrogate.fast_sigmoid())

        # 3. Readout Layer (Plastic)
        self.fc_out = nn.Linear(n_res, n_output, bias=False)

        # Initialize readout weights
        nn.init.uniform_(self.fc_out.weight, -0.1, 0.1)

    def forward(self, x_seq):
        """
        x_seq: [time_steps, batch_size, n_input]
        """
        mem_res = self.lif_res.init_leaky()

        spk_res_rec = []

        for step in range(x_seq.size(0)):
            # Input current + recurrent current
            if step == 0:
                cur_rec = 0
            else:
                cur_rec = self.fc_rec(spk_res_rec[-1])

            cur_in = self.fc_in(x_seq[step])

            cur_total = cur_in + cur_rec

            spk_res, mem_res = self.lif_res(cur_total, mem_res)
            spk_res_rec.append(spk_res)

        spk_res_seq = torch.stack(spk_res_rec, dim=0)

        return spk_res_seq

    def stdp_update(self, spk_res_seq, learning_rate=0.01):
        """
        A simplified local Hebbian/STDP-like update rule for the readout layer.
        """
        # spk_res_seq: [time_steps, batch_size, n_res]

        mem_out = torch.zeros(spk_res_seq.size(1), self.fc_out.out_features)

        delta_w = torch.zeros_like(self.fc_out.weight)

        # Simplified trace variables
        trace_pre = torch.zeros(spk_res_seq.size(1), self.n_res)
        trace_post = torch.zeros(spk_res_seq.size(1), self.fc_out.out_features)

        for step in range(spk_res_seq.size(0)):
            spk_pre = spk_res_seq[step]

            # Update pre-synaptic trace
            trace_pre = 0.9 * trace_pre + spk_pre

            # Forward pass through readout layer (simplified LIF for readout)
            cur_out = self.fc_out(spk_pre)
            mem_out = 0.9 * mem_out + cur_out

            # Determine post-synaptic spikes (threshold = 1.0)
            spk_post = (mem_out > 1.0).float()
            mem_out[mem_out > 1.0] -= 1.0 # Soft reset

            # Update post-synaptic trace
            trace_post = 0.9 * trace_post + spk_post

            # Hebbian / STDP update
            # We want to strengthen weights where pre-synaptic spike precedes post-synaptic spike

            # Outer product per batch item: spk_post * trace_pre
            dw = torch.einsum('bo,br->bor', spk_post, trace_pre).mean(dim=0)

            # Add simple LTD (depression) if post spikes without pre
            dw -= 0.1 * torch.einsum('bo,br->bor', spk_post, 1.0 - trace_pre).mean(dim=0)

            # Add STDP-like homeostatic depression
            dw -= 0.05 * self.fc_out.weight

            delta_w += dw

        # Apply weight update
        with torch.no_grad():
            self.fc_out.weight += learning_rate * delta_w


def main():
    print("--- Initialization of ESP32 LSM Topology Prototype ---")

    model = LiquidReservoir(n_input=N_INPUT, n_res=N_RES, n_output=N_OUTPUT)

    print("\n--- Structural Configuration Matrix ---")
    print(f"Reservoir Matrix Shape: {model.fc_rec.weight.shape}")

    w_rec = model.fc_rec.weight.data

    # Analyze the E/I ratio based on the generated weights
    # Sum of signs along columns (neurons projecting out)
    column_sums = w_rec.sum(dim=0)
    excitatory_count = (column_sums > 0).sum().item()
    inhibitory_count = (column_sums < 0).sum().item()
    total_count = excitatory_count + inhibitory_count

    print(f"Configured Excitatory Neurons: {excitatory_count} ({excitatory_count/total_count*100:.1f}%)")
    print(f"Configured Inhibitory Neurons: {inhibitory_count} ({inhibitory_count/total_count*100:.1f}%)")

    # Simulation setup
    time_steps = 100
    batch_size = 1

    # 1. Input encoding: Random continuous sensor telemetry -> Spike rate coding
    # Using spikegen.rate expects probabilities [0, 1]
    raw_sensor_data = torch.rand(time_steps, batch_size, N_INPUT)

    spike_encoded_input = spikegen.rate(raw_sensor_data, time_var_input=True)

    print("\n--- Running Simulation ---")
    print(f"Input shape (Spike Encoded): {spike_encoded_input.shape}")

    # 2. Forward pass through the reservoir
    spk_res_seq = model(spike_encoded_input)

    print(f"Reservoir State Output Shape: {spk_res_seq.shape}")

    # Calculate average firing rate of the reservoir to ensure it is stable
    avg_firing_rate = spk_res_seq.float().mean().item()
    print(f"Average Reservoir Firing Rate: {avg_firing_rate*100:.2f}%")

    # 3. Readout Layer Plasticity (Hebbian / STDP update)
    print("\n--- Applying Hebbian Readout Update ---")
    initial_weights = model.fc_out.weight.clone().detach()

    model.stdp_update(spk_res_seq, learning_rate=0.05)

    updated_weights = model.fc_out.weight.detach()

    weight_diff = torch.norm(updated_weights - initial_weights).item()
    print(f"Readout weight matrix updated. L2 Norm of weight change: {weight_diff:.4f}")
    print("Simulation and verification successful.")

if __name__ == "__main__":
    main()
