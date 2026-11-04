#include "snn_lsm.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "main.h"

static const char *TAG = "SNN_LSM";

// Helper to generate a random float between -1.0 and 1.0
static float rand_float(void) {
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

    lsm->active_bank = 0;
    lsm->ab_eval_active = false;
    lsm->ab_eval_ticks_remaining = 0;

    lsm->modulator = 1.0f;
    lsm->beta = 0.9f;
    lsm->v_th = 1.0f;
    lsm->learning_rate = 0.05f;
    lsm->oja_alpha = 0.05f;
    lsm->spectral_clamp = 4.0f;

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

    snn_weight_bank_t *bank0 = &lsm->banks[0];

    for (int i = 0; i < N_RES; i++) { // Post-synaptic
        for (int j = 0; j < N_RES; j++) { // Pre-synaptic
            float w_raw = expf(rand_normal(mu, sigma));

            // Dale's law: Columns dictate sign (Neuron j's outgoing connections)
            if (j >= n_excitatory) {
                w_raw = -w_raw; // Inhibitory
            }

            bank0->w_rec[i][j] = w_raw * 0.1f;
            bank0->eta_rec[i][j] = 0.0f; // Recurrent layer default frozen; LLM can unfreeze
        }
    }

    // 3. Initialize readout weights (random uniform)
    for (int i = 0; i < N_OUTPUT; i++) {
        for (int j = 0; j < N_RES; j++) {
            bank0->w_out[i][j] = rand_float() * 0.5f;
            bank0->eta_out[i][j] = 1.0f; // Readout layer plastic by default
        }
    }

    // Copy bank 0 to bank 1 for identical baseline
    memcpy(&lsm->banks[1], &lsm->banks[0], sizeof(snn_weight_bank_t));
}

void snn_lsm_forward(snn_lsm_t *lsm, const float inputs[N_INPUT]) {
    snn_weight_bank_t *active = &lsm->banks[lsm->active_bank];

    // 1. Reservoir Dynamics
    for (int i = 0; i < N_RES; i++) {
        float cur_in = 0.0f;
        float cur_rec = 0.0f;

        // Input current
        for (int j = 0; j < N_INPUT; j++) {
            cur_in += lsm->w_in[i][j] * inputs[j];
        }

        // Recurrent current (from previous step's spikes)
        for (int j = 0; j < N_RES; j++) {
            if (lsm->spk_res[j]) {
                cur_rec += active->w_rec[i][j];
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

        // EMA of firing rate (alpha = 0.02)
        lsm->mean_act_res[i] = 0.98f * lsm->mean_act_res[i] + 0.02f * (float)lsm->spk_res[i];
    }
}

void snn_lsm_stdp_update(snn_lsm_t *lsm) {
    snn_weight_bank_t *active = &lsm->banks[lsm->active_bank];
    float eff_lr = lsm->learning_rate * lsm->modulator;

    // 1. Readout Dynamics & Trace updates
    for (int i = 0; i < N_OUTPUT; i++) {
        float cur_out = 0.0f;

        // Sum currents from reservoir
        for (int j = 0; j < N_RES; j++) {
            if (lsm->spk_res[j]) {
                cur_out += active->w_out[i][j];
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

        // EMA of output firing rate
        lsm->mean_act_out[i] = 0.98f * lsm->mean_act_out[i] + 0.02f * (float)lsm->spk_out[i];
    }

    // Update Pre-Trace for Reservoir
    for (int j = 0; j < N_RES; j++) {
        lsm->trace_pre[j] = lsm->beta * lsm->trace_pre[j] + lsm->spk_res[j];
    }

    // 2. Weight Update (STDP) for Readout Layer
    for (int i = 0; i < N_OUTPUT; i++) {
        for (int j = 0; j < N_RES; j++) {
            float eta = active->eta_out[i][j];
            if (eta <= 0.0f) {
                continue; // Frozen synapse
            }

            float dw = 0.0f;

            // LTP: Post-synaptic spike with active pre-synaptic trace
            if (lsm->spk_out[i]) {
                dw += lsm->trace_pre[j];
            }

            // LTD: Post-synaptic spike without pre-synaptic trace (Depression)
            if (lsm->spk_out[i]) {
                dw -= 0.1f * (1.0f - lsm->trace_pre[j]);
            }

            // Oja-style Homeostasis decay: - alpha * w * y^2
            float y = (float)lsm->spk_out[i];
            dw -= lsm->oja_alpha * active->w_out[i][j] * (y * y + 0.1f);

            active->w_out[i][j] += eff_lr * eta * dw;

            // Spectral / magnitude clamp
            if (active->w_out[i][j] > lsm->spectral_clamp) {
                active->w_out[i][j] = lsm->spectral_clamp;
            } else if (active->w_out[i][j] < -lsm->spectral_clamp) {
                active->w_out[i][j] = -lsm->spectral_clamp;
            }
        }
    }

    // 3. Optional Plasticity for Recurrent Reservoir Layer (if eta_rec > 0)
    for (int i = 0; i < N_RES; i++) {
        for (int j = 0; j < N_RES; j++) {
            float eta = active->eta_rec[i][j];
            if (eta <= 0.0f) continue;

            float dw = 0.0f;
            if (lsm->spk_res[i]) {
                dw += lsm->trace_pre[j];
                dw -= 0.05f * (1.0f - lsm->trace_pre[j]);
            }
            dw -= lsm->oja_alpha * active->w_rec[i][j];
            active->w_rec[i][j] += eff_lr * eta * dw;

            if (active->w_rec[i][j] > lsm->spectral_clamp) {
                active->w_rec[i][j] = lsm->spectral_clamp;
            } else if (active->w_rec[i][j] < -lsm->spectral_clamp) {
                active->w_rec[i][j] = -lsm->spectral_clamp;
            }
        }
    }
}

void snn_lsm_mutate_hyperparams(snn_lsm_t *lsm, float new_beta, float new_v_th, float new_lr) {
    if (new_beta > 0.0f && new_beta < 1.0f) lsm->beta = new_beta;
    if (new_v_th > 0.0f) lsm->v_th = new_v_th;
    if (new_lr > 0.0f) lsm->learning_rate = new_lr;
}

void snn_lsm_set_modulator(snn_lsm_t *lsm, float modulator) {
    if (modulator < -2.0f) modulator = -2.0f;
    if (modulator > 2.0f) modulator = 2.0f;
    lsm->modulator = modulator;
}

void snn_lsm_prune_weights(snn_lsm_t *lsm, float threshold) {
    for (int b = 0; b < 2; b++) {
        snn_weight_bank_t *bank = &lsm->banks[b];
        // Prune readout weights
        for (int i = 0; i < N_OUTPUT; i++) {
            for (int j = 0; j < N_RES; j++) {
                if (fabsf(bank->w_out[i][j]) < threshold) {
                    bank->w_out[i][j] = 0.0f;
                }
            }
        }

        // Prune recurrent weights
        for (int i = 0; i < N_RES; i++) {
            for (int j = 0; j < N_RES; j++) {
                if (fabsf(bank->w_rec[i][j]) < threshold) {
                    bank->w_rec[i][j] = 0.0f;
                }
            }
        }
    }
}

uint8_t snn_lsm_prepare_shadow_bank(snn_lsm_t *lsm) {
    uint8_t shadow_bank = lsm->active_bank ^ 1;
    memcpy(&lsm->banks[shadow_bank], &lsm->banks[lsm->active_bank], sizeof(snn_weight_bank_t));
    return shadow_bank;
}

bool snn_lsm_patch_synapse(snn_lsm_t *lsm, uint8_t bank, const char *layer, int post_idx, int pre_idx, float new_w, float new_eta) {
    if (bank > 1) return false;
    snn_weight_bank_t *target_bank = &lsm->banks[bank];

    // Clamp weight and eta to safe boundaries
    if (new_w > lsm->spectral_clamp) new_w = lsm->spectral_clamp;
    if (new_w < -lsm->spectral_clamp) new_w = -lsm->spectral_clamp;
    if (new_eta < 0.0f) new_eta = 0.0f;
    if (new_eta > 5.0f) new_eta = 5.0f;

    if (strcmp(layer, "lsm_out") == 0) {
        if (post_idx < 0 || post_idx >= N_OUTPUT || pre_idx < 0 || pre_idx >= N_RES) {
            return false;
        }
        target_bank->w_out[post_idx][pre_idx] = new_w;
        target_bank->eta_out[post_idx][pre_idx] = new_eta;
        return true;
    } else if (strcmp(layer, "lsm_rec") == 0) {
        if (post_idx < 0 || post_idx >= N_RES || pre_idx < 0 || pre_idx >= N_RES) {
            return false;
        }
        target_bank->w_rec[post_idx][pre_idx] = new_w;
        target_bank->eta_rec[post_idx][pre_idx] = new_eta;
        return true;
    } else if (strcmp(layer, "pred_coding") == 0) {
        // Secondary bridge: patch Predictive Coding Network in main.c
        extern HiddenLayer* g_hl;
        extern OutputLayer* g_ol;
        if (post_idx >= 0 && post_idx < HIDDEN_NEURONS && pre_idx >= 0 && pre_idx < INPUT_NEURONS && g_hl) {
            g_hl->weights[post_idx][pre_idx] = new_w;
            return true;
        } else if (post_idx >= 0 && post_idx < OUTPUT_NEURONS && pre_idx >= 0 && pre_idx < HIDDEN_NEURONS && g_ol) {
            g_ol->weights[post_idx][pre_idx] = new_w;
            return true;
        }
        return false;
    }

    return false;
}

void snn_lsm_activate_shadow_bank(snn_lsm_t *lsm, uint32_t eval_ticks) {
    lsm->active_bank ^= 1; // Atomic bank flip
    lsm->ab_eval_active = true;
    lsm->ab_eval_ticks_remaining = eval_ticks;
    lsm->stress_high_ticks = 0;
    ESP_LOGI(TAG, "A/B Shadow Bank activated: active_bank=%d, eval_ticks=%lu",
             lsm->active_bank, (unsigned long)eval_ticks);
}

void snn_lsm_commit_active_bank(snn_lsm_t *lsm) {
    lsm->ab_eval_active = false;
    lsm->ab_eval_ticks_remaining = 0;
    uint8_t inactive_bank = lsm->active_bank ^ 1;
    // Mirror active bank over to inactive bank so both are synchronized
    memcpy(&lsm->banks[inactive_bank], &lsm->banks[lsm->active_bank], sizeof(snn_weight_bank_t));
    ESP_LOGI(TAG, "A/B Bank %d committed successfully", lsm->active_bank);
}

void snn_lsm_rollback_active_bank(snn_lsm_t *lsm) {
    lsm->active_bank ^= 1; // Instant flip back to baseline
    lsm->ab_eval_active = false;
    lsm->ab_eval_ticks_remaining = 0;
    ESP_LOGW(TAG, "A/B Bank rolled back to bank %d", lsm->active_bank);
}

bool snn_lsm_check_emergency_rollback(snn_lsm_t *lsm, float current_jerk, float stress_level) {
    if (!lsm->ab_eval_active) {
        return false;
    }

    lsm->accel_rms_jerk = current_jerk;

    bool jerk_exceeded = (current_jerk > MAX_SAFE_ACCEL_JERK);
    if (stress_level > 0.85f) {
        lsm->stress_high_ticks++;
    } else {
        lsm->stress_high_ticks = 0;
    }

    bool stress_exceeded = (lsm->stress_high_ticks >= 3);

    if (jerk_exceeded || stress_exceeded) {
        snn_lsm_rollback_active_bank(lsm);
        const char *reason = jerk_exceeded ? "accel_jerk_exceeded" : "lsm_stress_exceeded";
        printf("@JSON:{\"event\":\"ab_emergency_rollback\",\"reason\":\"%s\",\"jerk\":%.4f,\"stress\":%.4f,\"bank\":%d}\n",
               reason, current_jerk, stress_level, lsm->active_bank);
        fflush(stdout);
        return true;
    }

    return false;
}
