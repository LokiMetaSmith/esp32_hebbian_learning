#include "nvs_storage.h"
#include "nvs_flash.h"
#include "esp_log.h"

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