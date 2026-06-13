#ifndef SNN_LSM_H
#define SNN_LSM_H

#include <stdint.h>
#include <stdbool.h>

// ESP32 Memory constraints -> Conservative SNN size
#define N_INPUT  6
#define N_RES    64
#define N_OUTPUT 4

// Fixed-point / Int representation constants to avoid heavy floats if possible.
// For the prototype, we use floats for simplicity, but in a production ESP32 setup, Q15 or integer scaling is preferred.
// Here we use floats to match the SNN physics (decay, threshold) directly for clarity.

typedef struct {
    // Reservoir (Fixed Recurrent Weights)
    float w_in[N_RES][N_INPUT];
    float w_rec[N_RES][N_RES];

    // Readout (Plastic Weights)
    float w_out[N_OUTPUT][N_RES];

    // Neuron State (LIF variables)
    float mem_res[N_RES];
    float mem_out[N_OUTPUT];

    // Traces for STDP
    float trace_pre[N_RES];
    float trace_post[N_OUTPUT];

    // Spiking outputs for the current step
    uint8_t spk_res[N_RES];
    uint8_t spk_out[N_OUTPUT];

    // Hyperparameters (Mutable by LLM Supervisor)
    float beta;        // Membrane decay (tau_m)
    float v_th;        // Threshold voltage
    float learning_rate;
} snn_lsm_t;

/**
 * @brief Initialize the Liquid State Machine topology.
 * Allocates and initializes the connection weights (E/I ratio, log-normal).
 *
 * @param lsm Pointer to the LSM structure.
 */
void snn_lsm_init(snn_lsm_t *lsm);

/**
 * @brief Forward pass of the SNN for a single timestep.
 *
 * @param lsm Pointer to the LSM structure.
 * @param inputs Array of input spike probabilities or encoded values (size N_INPUT).
 */
void snn_lsm_forward(snn_lsm_t *lsm, const float inputs[N_INPUT]);

/**
 * @brief Apply local unsupervised Hebbian / STDP learning to the readout layer.
 *
 * @param lsm Pointer to the LSM structure.
 */
void snn_lsm_stdp_update(snn_lsm_t *lsm);

/**
 * @brief Modify the hyperparameters from an external source (LLM Supervisor).
 *
 * @param lsm Pointer to the LSM structure.
 * @param new_beta New membrane decay constant.
 * @param new_v_th New threshold voltage.
 * @param new_lr New learning rate.
 */
void snn_lsm_mutate_hyperparams(snn_lsm_t *lsm, float new_beta, float new_v_th, float new_lr);

#endif // SNN_LSM_H
