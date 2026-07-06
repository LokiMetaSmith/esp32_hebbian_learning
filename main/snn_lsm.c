#include "snn_lsm.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Helper to generate a random float between -1.0 and 1.0
static float rand_float() {
    return ((float)rand() / (float)(RAND_MAX)) * 2.0f - 1.0f;
}

// Box-Muller transform for normally distributed random numbers
static float rand_normal(float mu, float sigma) {
    float u1 = ((float)rand() / (float)(RAND_MAX));
    float u2 = ((float)rand() / (float)(RAND_MAX));
    if (u1 <= 0.0001f) u1 = 0.0001f;

    float z0 = sqrtf(-2.0f * logf(u1)) * cosf(2.0f * (float)M_PI * u2);
    return z0 * sigma + mu;
}

void snn_lsm_init(snn_lsm_t *lsm) {
    memset(lsm, 0, sizeof(snn_lsm_t));

    lsm->beta = 0.9f;
    lsm->v_th = 1.0f;
    lsm->learning_rate = 0.05f;

    // 1. Initialize input weights (random uniform)
    for (int i = 0; i < N_RES; i++) {
        for (int j = 0; j < N_INPUT; j++) {
            lsm->w_in[i][j] = rand_float() * 0.5f;
        }
    }

    // 2. Initialize fixed recurrent weights (Log-Normal & Dale's Law)
    float mu = -1.0f;
    float sigma = 0.5f;
    int n_excitatory = (int)(0.8f * N_RES);

    for (int i = 0; i < N_RES; i++) { // Post-synaptic
        for (int j = 0; j < N_RES; j++) { // Pre-synaptic
            float w_raw = expf(rand_normal(mu, sigma));

            // Dale's law: Columns dictate sign (Neuron j's outgoing connections)
            if (j >= n_excitatory) {
                w_raw = -w_raw; // Inhibitory
            }

            // Simple static scaling to approximate spectral radius constraint
            // (A true eigen-decomposition is too heavy for ESP32 init, so we scale empirically)
            lsm->w_rec[i][j] = w_raw * 0.1f;
        }
    }

    // 3. Initialize readout weights (random uniform)
    for (int i = 0; i < N_OUTPUT; i++) {
        for (int j = 0; j < N_RES; j++) {
            lsm->w_out[i][j] = rand_float() * 0.5f; // Increased initial readout weights
        }
    }
}

void snn_lsm_forward(snn_lsm_t *lsm, const float inputs[N_INPUT]) {
    // 1. Reservoir Dynamics
    for (int i = 0; i < N_RES; i++) {
        float cur_in = 0.0f;
        float cur_rec = 0.0f;

        // Input current
        for (int j = 0; j < N_INPUT; j++) {
            // Treat continuous input directly as current for rate coding efficiency
            cur_in += lsm->w_in[i][j] * inputs[j];
        }

        // Recurrent current (from previous step's spikes)
        for (int j = 0; j < N_RES; j++) {
            if (lsm->spk_res[j]) {
                cur_rec += lsm->w_rec[i][j];
            }
        }

        // LIF Update
        lsm->mem_res[i] = lsm->beta * lsm->mem_res[i] + cur_in + cur_rec;

        // Spike condition
        if (lsm->mem_res[i] >= lsm->v_th) {
            lsm->spk_res[i] = 1;
            lsm->mem_res[i] -= lsm->v_th; // Soft reset
        } else {
            lsm->spk_res[i] = 0;
        }
    }
}

void snn_lsm_stdp_update(snn_lsm_t *lsm) {
    // 1. Readout Dynamics & Trace updates
    for (int i = 0; i < N_OUTPUT; i++) {
        float cur_out = 0.0f;

        // Sum currents from reservoir
        for (int j = 0; j < N_RES; j++) {
            if (lsm->spk_res[j]) {
                cur_out += lsm->w_out[i][j];
            }
        }

        // Readout LIF
        lsm->mem_out[i] = lsm->beta * lsm->mem_out[i] + cur_out;

        // Readout Spike
        if (lsm->mem_out[i] >= lsm->v_th) {
            lsm->spk_out[i] = 1;
            lsm->mem_out[i] -= lsm->v_th; // Soft reset
        } else {
            lsm->spk_out[i] = 0;
        }

        // Update Post-Trace
        lsm->trace_post[i] = lsm->beta * lsm->trace_post[i] + lsm->spk_out[i];
    }

    // Update Pre-Trace for Reservoir
    for (int j = 0; j < N_RES; j++) {
        lsm->trace_pre[j] = lsm->beta * lsm->trace_pre[j] + lsm->spk_res[j];
    }

    // 2. Weight Update (STDP)
    for (int i = 0; i < N_OUTPUT; i++) {
        for (int j = 0; j < N_RES; j++) {
            float dw = 0.0f;

            // LTP: Post-synaptic spike with active pre-synaptic trace
            if (lsm->spk_out[i]) {
                dw += lsm->trace_pre[j];
            }

            // LTD: Post-synaptic spike without pre-synaptic trace (Depression)
            if (lsm->spk_out[i]) {
                dw -= 0.1f * (1.0f - lsm->trace_pre[j]);
            }

            // Homeostasis
            dw -= 0.05f * lsm->w_out[i][j];

            lsm->w_out[i][j] += lsm->learning_rate * dw;
        }
    }
}

void snn_lsm_mutate_hyperparams(snn_lsm_t *lsm, float new_beta, float new_v_th, float new_lr) {
    if (new_beta > 0.0f && new_beta < 1.0f) lsm->beta = new_beta;
    if (new_v_th > 0.0f) lsm->v_th = new_v_th;
    if (new_lr > 0.0f) lsm->learning_rate = new_lr;
}
