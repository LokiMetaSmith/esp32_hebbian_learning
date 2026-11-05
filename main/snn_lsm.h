#ifndef SNN_LSM_H
#define SNN_LSM_H

#include <stdint.h>
#include <stdbool.h>

// ESP32 Memory constraints -> Conservative SNN size
#define N_INPUT  6
#define N_RES    64
#define N_OUTPUT 4

#define MAX_SAFE_ACCEL_JERK 2.5f

// Single weight bank containing recurrent and readout weights plus per-synapse plasticity masks
typedef struct {
    float w_rec[N_RES][N_RES];
    float eta_rec[N_RES][N_RES]; // Plasticity mask for recurrent connections (0.0 = frozen ROM)

    float w_out[N_OUTPUT][N_RES];
    float eta_out[N_OUTPUT][N_RES]; // Plasticity mask for readout connections (0.0 = frozen ROM)
} snn_weight_bank_t;

typedef struct {
    // Input weights (feedforward from sensors into reservoir)
    float w_in[N_RES][N_INPUT];

    // Double-buffered weight banks for zero-stall A/B shadow testing
    snn_weight_bank_t banks[2];
    volatile uint8_t active_bank; // 0 or 1

    // A/B evaluation tracking
    bool ab_eval_active;
    uint32_t ab_eval_ticks_remaining;
    float fitness_baseline;
    float fitness_shadow;

    // Neuron State (LIF variables)
    float mem_res[N_RES];
    float mem_out[N_OUTPUT];

    // Running activation exponential moving average (for dead-node / saturation detection)
    float mean_act_res[N_RES];
    float mean_act_out[N_OUTPUT];

    // Traces for STDP
    float trace_pre[N_RES];
    float trace_post[N_OUTPUT];

    // Eligibility trace for R-STDP
    float elig_out[N_OUTPUT][N_RES];

    // Spiking outputs for the current step
    uint8_t spk_res[N_RES];
    uint8_t spk_out[N_OUTPUT];

    // Neuromodulation and Hyperparameters
    float modulator;      // 3rd-factor reward/dopamine scalar M(t) (default 1.0f)
    float beta;           // Membrane decay (tau_m)
    float v_th;           // Threshold voltage
    float learning_rate;  // Base learning rate
    float oja_alpha;      // Homeostatic normalization factor (default 0.05f)
    float spectral_clamp; // Hard weight bound (e.g. 4.0f)

    // Hardware Ground-Truth Tracking
    float accel_rms_jerk;
    uint32_t stress_high_ticks;
} snn_lsm_t;

/**
 * @brief Initialize the Liquid State Machine topology.
 * Allocates and initializes connection weights (E/I ratio, log-normal) into both banks.
 *
 * @param lsm Pointer to the LSM structure.
 */
void snn_lsm_init(snn_lsm_t *lsm);

/**
 * @brief Forward pass of the SNN for a single timestep using the currently active bank.
 *
 * @param lsm Pointer to the LSM structure.
 * @param inputs Array of input spike probabilities or encoded values (size N_INPUT).
 */
void snn_lsm_forward(snn_lsm_t *lsm, const float inputs[N_INPUT]);

/**
 * @brief Apply local unsupervised Hebbian / STDP learning with per-synapse plasticity masks,
 * 3rd-factor neuromodulation M(t), and Oja-style homeostatic decay to active bank.
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

/**
 * @brief Set the 3rd-factor neuromodulator M(t) scalar.
 * @param lsm Pointer to LSM structure.
 * @param modulator New modulator value.
 */
void snn_lsm_set_modulator(snn_lsm_t *lsm, float modulator);

/**
 * @brief Prune weights below a given threshold in both banks.
 * @param lsm Pointer to the LSM structure.
 * @param threshold Absolute weight value below which a connection is pruned (set to zero).
 */
void snn_lsm_prune_weights(snn_lsm_t *lsm, float threshold);

/**
 * @brief Stage a sparse patch to the shadow bank, copy unchanged weights from active bank,
 * flip active_bank atomically, and start evaluation window.
 *
 * @param lsm Pointer to the LSM structure.
 * @param layer Target layer: "lsm_out", "lsm_rec" (or secondary bridge).
 * @param post_idx Post-synaptic neuron index.
 * @param pre_idx Pre-synaptic neuron index.
 * @param new_w New synaptic weight.
 * @param new_eta New plasticity mask rate.
 * @return true if successful, false on index error.
 */
bool snn_lsm_patch_synapse(snn_lsm_t *lsm, uint8_t bank, const char *layer, int post_idx, int pre_idx, float new_w, float new_eta);

/**
 * @brief Prepare shadow bank by copying active bank to shadow bank.
 * @param lsm Pointer to LSM structure.
 * @return index of shadow bank (active_bank ^ 1).
 */
uint8_t snn_lsm_prepare_shadow_bank(snn_lsm_t *lsm);

/**
 * @brief Atomically switch active bank to shadow bank and begin evaluation timer.
 * @param lsm Pointer to LSM structure.
 * @param eval_ticks Number of ticks to evaluate the shadow patch.
 */
void snn_lsm_activate_shadow_bank(snn_lsm_t *lsm, uint32_t eval_ticks);

/**
 * @brief Commit current active bank (copies active bank to other bank, ends eval).
 * @param lsm Pointer to LSM structure.
 */
void snn_lsm_commit_active_bank(snn_lsm_t *lsm);

/**
 * @brief Roll back to the previous bank immediately (flips active_bank ^ 1, ends eval).
 * @param lsm Pointer to LSM structure.
 */
void snn_lsm_rollback_active_bank(snn_lsm_t *lsm);

/**
 * @brief Check safety conditions during A/B evaluation (accel jerk > MAX_SAFE_ACCEL_JERK or stress > 0.85).
 * If safety threshold is exceeded, triggers immediate single-tick rollback.
 *
 * @param lsm Pointer to LSM structure.
 * @param current_jerk Instantaneous jerk from BMA400.
 * @param stress_level Current LSM stress level.
 * @return true if emergency rollback was triggered, false otherwise.
 */
bool snn_lsm_check_emergency_rollback(snn_lsm_t *lsm, float current_jerk, float stress_level);

#endif // SNN_LSM_H
