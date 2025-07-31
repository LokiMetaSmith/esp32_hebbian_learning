/**
 * @file nvs_storage.c
 * @brief Implementation of Non-Volatile Storage (NVS) operations.
 *
 * This file provides the functions for saving and loading critical data,
 * such as the neural network weights and servo calibration maps, to the
 * ESP32's non-volatile flash memory. This allows the robot's learned
 * state and configuration to persist across reboots.
 */

#include "nvs_storage.h"
#include "common.h"

#define NVS_NAMESPACE "nn_storage"
static const char *TAG = "NVS_STORAGE";

esp_err_t nvs_storage_initialize() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, "NVS Initialized.");
    return err;
}

esp_err_t save_network_to_nvs(const HiddenLayer* hl, const OutputLayer* ol, const PredictionLayer* pl) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return err;
    }

    // Save each layer as a binary blob
    err = nvs_set_blob(nvs_handle, "hidden_layer", hl, sizeof(HiddenLayer));
    if(err != ESP_OK) { ESP_LOGE(TAG, "Failed to save hidden layer: %s", esp_err_to_name(err)); goto cleanup; }

    err = nvs_set_blob(nvs_handle, "output_layer", ol, sizeof(OutputLayer));
    if(err != ESP_OK) { ESP_LOGE(TAG, "Failed to save output layer: %s", esp_err_to_name(err)); goto cleanup; }

    err = nvs_set_blob(nvs_handle, "predict_layer", pl, sizeof(PredictionLayer));
    if(err != ESP_OK) { ESP_LOGE(TAG, "Failed to save prediction layer: %s", esp_err_to_name(err)); goto cleanup; }

    // Commit changes
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS commit failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Network state saved successfully to NVS.");
    }

cleanup:
    nvs_close(nvs_handle);
    return err;
}

esp_err_t save_state_tokens_to_nvs(const float centroids[NUM_STATE_TOKENS][STATE_VECTOR_DIM], const float embeddings[NUM_STATE_TOKENS][HIDDEN_NEURONS]) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle for state tokens!", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_blob(nvs_handle, "centroids", centroids, sizeof(float) * NUM_STATE_TOKENS * STATE_VECTOR_DIM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save centroids: %s", esp_err_to_name(err));
        goto cleanup_state_tokens;
    }

    err = nvs_set_blob(nvs_handle, "embeddings", embeddings, sizeof(float) * NUM_STATE_TOKENS * HIDDEN_NEURONS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save embeddings: %s", esp_err_to_name(err));
        goto cleanup_state_tokens;
    }

    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS commit for state tokens failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "State tokens saved successfully to NVS.");
    }

cleanup_state_tokens:
    nvs_close(nvs_handle);
    return err;
}

esp_err_t load_state_tokens_from_nvs(float centroids[NUM_STATE_TOKENS][STATE_VECTOR_DIM], float embeddings[NUM_STATE_TOKENS][HIDDEN_NEURONS]) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle for state tokens!", esp_err_to_name(err));
        return err;
    }

    size_t required_size;

    required_size = sizeof(float) * NUM_STATE_TOKENS * STATE_VECTOR_DIM;
    err = nvs_get_blob(nvs_handle, "centroids", centroids, &required_size);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load centroids: %s", esp_err_to_name(err));
        goto cleanup_load_state_tokens;
    }

    required_size = sizeof(float) * NUM_STATE_TOKENS * HIDDEN_NEURONS;
    err = nvs_get_blob(nvs_handle, "embeddings", embeddings, &required_size);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load embeddings: %s", esp_err_to_name(err));
        goto cleanup_load_state_tokens;
    }

    ESP_LOGI(TAG, "State tokens loaded successfully from NVS.");

cleanup_load_state_tokens:
    nvs_close(nvs_handle);
    return err;
}

static void json_to_float_array(cJSON *json_array, float *target_array, int array_size) {
    if (!cJSON_IsArray(json_array)) return;
    int i = 0;
    cJSON *element;
    cJSON_ArrayForEach(element, json_array) {
        if (i < array_size && cJSON_IsNumber(element)) {
            target_array[i++] = (float)element->valuedouble;
        }
    }
}

#include "main.h"
esp_err_t save_network_from_json(const cJSON *nn_json) {
    if (!nn_json) return ESP_ERR_INVALID_ARG;

    HiddenLayer hl;
    OutputLayer ol;
    PredictionLayer pl;

    cJSON *hidden_layer_json = cJSON_GetObjectItem(nn_json, "hidden_layer");
    if (hidden_layer_json) {
        json_to_float_array(cJSON_GetObjectItem(hidden_layer_json, "weights"), (float *)hl.weights, INPUT_NEURONS * HIDDEN_NEURONS);
        json_to_float_array(cJSON_GetObjectItem(hidden_layer_json, "biases"), hl.hidden_bias, HIDDEN_NEURONS);
    }

    cJSON *output_layer_json = cJSON_GetObjectItem(nn_json, "output_layer");
    if (output_layer_json) {
        json_to_float_array(cJSON_GetObjectItem(output_layer_json, "weights"), (float *)ol.weights, HIDDEN_NEURONS * OUTPUT_NEURONS);
        json_to_float_array(cJSON_GetObjectItem(output_layer_json, "biases"), ol.output_bias, OUTPUT_NEURONS);
    }

    cJSON *prediction_layer_json = cJSON_GetObjectItem(nn_json, "prediction_layer");
    if (prediction_layer_json) {
        json_to_float_array(cJSON_GetObjectItem(prediction_layer_json, "weights"), (float *)pl.weights, HIDDEN_NEURONS * PRED_NEURONS);
        json_to_float_array(cJSON_GetObjectItem(prediction_layer_json, "biases"), pl.pred_bias, PRED_NEURONS);
    }

    return save_network_to_nvs(&hl, &ol, &pl);
}

esp_err_t get_raw_network_blob(uint8_t **buffer, size_t *size) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) return err;

    size_t total_size = sizeof(HiddenLayer) + sizeof(OutputLayer) + sizeof(PredictionLayer);
    *buffer = malloc(total_size);
    if (*buffer == NULL) {
        nvs_close(nvs_handle);
        return ESP_ERR_NO_MEM;
    }

    uint8_t *ptr = *buffer;
    size_t required_size;

    required_size = sizeof(HiddenLayer);
    err = nvs_get_blob(nvs_handle, "hidden_layer", ptr, &required_size);
    if (err != ESP_OK) goto fail;
    ptr += required_size;

    required_size = sizeof(OutputLayer);
    err = nvs_get_blob(nvs_handle, "output_layer", ptr, &required_size);
    if (err != ESP_OK) goto fail;
    ptr += required_size;

    required_size = sizeof(PredictionLayer);
    err = nvs_get_blob(nvs_handle, "predict_layer", ptr, &required_size);
    if (err != ESP_OK) goto fail;

    *size = total_size;
    nvs_close(nvs_handle);
    return ESP_OK;

fail:
    free(*buffer);
    *buffer = NULL;
    nvs_close(nvs_handle);
    return err;
}

esp_err_t set_raw_network_blob(const uint8_t *buffer, size_t size) {
    if (size != sizeof(HiddenLayer) + sizeof(OutputLayer) + sizeof(PredictionLayer)) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) return err;

    const uint8_t *ptr = buffer;

    err = nvs_set_blob(nvs_handle, "hidden_layer", ptr, sizeof(HiddenLayer));
    if (err != ESP_OK) goto cleanup_set;
    ptr += sizeof(HiddenLayer);

    err = nvs_set_blob(nvs_handle, "output_layer", ptr, sizeof(OutputLayer));
    if (err != ESP_OK) goto cleanup_set;
    ptr += sizeof(OutputLayer);

    err = nvs_set_blob(nvs_handle, "predict_layer", ptr, sizeof(PredictionLayer));
    if (err != ESP_OK) goto cleanup_set;

    err = nvs_commit(nvs_handle);

cleanup_set:
    nvs_close(nvs_handle);
    return err;
}

esp_err_t save_correction_map_to_nvs(const ServoCorrectionMap* maps) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle for map save!", esp_err_to_name(err));
        return err;
    }

    size_t total_size = sizeof(ServoCorrectionMap) * NUM_SERVOS;
    err = nvs_set_blob(nvs_handle, "corr_maps", maps, total_size);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save correction maps: %s", esp_err_to_name(err));
        goto map_cleanup;
    }

    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS commit for maps failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Correction maps saved successfully to NVS.");
    }

map_cleanup:
    nvs_close(nvs_handle);
    return err;
}

esp_err_t load_correction_map_from_nvs(ServoCorrectionMap* maps) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle for map load!", esp_err_to_name(err));
        return err;
    }

    size_t required_size = sizeof(ServoCorrectionMap) * NUM_SERVOS;
    err = nvs_get_blob(nvs_handle, "corr_maps", maps, &required_size);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load correction maps: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Correction maps loaded successfully from NVS.");
    }

    nvs_close(nvs_handle);
    return err;
}


esp_err_t load_network_from_nvs(HiddenLayer* hl, OutputLayer* ol, PredictionLayer* pl) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return err;
    }

    size_t required_size;

    // Load hidden layer
    required_size = sizeof(HiddenLayer);
    err = nvs_get_blob(nvs_handle, "hidden_layer", hl, &required_size);
    if (err != ESP_OK) { ESP_LOGW(TAG, "Failed to load hidden layer: %s", esp_err_to_name(err)); goto cleanup; }

    // Load output layer
    required_size = sizeof(OutputLayer);
    err = nvs_get_blob(nvs_handle, "output_layer", ol, &required_size);
    if (err != ESP_OK) { ESP_LOGW(TAG, "Failed to load output layer: %s", esp_err_to_name(err)); goto cleanup; }
    
    // Load prediction layer
    required_size = sizeof(PredictionLayer);
    err = nvs_get_blob(nvs_handle, "predict_layer", pl, &required_size);
     if (err != ESP_OK) { ESP_LOGW(TAG, "Failed to load prediction layer: %s", esp_err_to_name(err)); goto cleanup; }

    ESP_LOGI(TAG, "Network loaded successfully from NVS.");

cleanup:
    nvs_close(nvs_handle);
    return err;
}