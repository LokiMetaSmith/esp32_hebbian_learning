#ifndef NVS_STORAGE_H
#define NVS_STORAGE_H

#include "main.h" // Include main header to get network struct definitions
#include "esp_err.h"
/**
 * @brief Initializes the Non-Volatile Storage (NVS) system.
 * @return esp_err_t Result of the initialization.
 */
esp_err_t nvs_storage_initialize();

/**
 * @brief Saves the entire neural network state to NVS.
 *
 * @param hl Pointer to the hidden layer struct.
 * @param ol Pointer to the output layer struct.
 * @param pl Pointer to the prediction layer struct.
 * @return esp_err_t Result of the save operation.
 */
esp_err_t save_network_to_nvs(const HiddenLayer* hl, const OutputLayer* ol, const PredictionLayer* pl);

/**
 * @brief Loads the entire neural network state from NVS.
 *
 * @param hl Pointer to the hidden layer struct to be populated.
 * @param ol Pointer to the output layer struct to be populated.
 * @param pl Pointer to the prediction layer struct to be populated.
 * @return esp_err_t Result of the load operation. ESP_ERR_NVS_NOT_FOUND if no network is saved.
 */
esp_err_t load_network_from_nvs(HiddenLayer* hl, OutputLayer* ol, PredictionLayer* pl);

/**
 * @brief Saves the servo correction maps to NVS.
 *
 * @param maps Pointer to the array of ServoCorrectionMap structs.
 * @return esp_err_t Result of the save operation.
 */
esp_err_t save_correction_map_to_nvs(const ServoCorrectionMap* maps);

/**
 * @brief Loads the servo correction maps from NVS.
 *
 * @param maps Pointer to the array of ServoCorrectionMap structs to be populated.
 * @return esp_err_t Result of the load operation. ESP_ERR_NVS_NOT_FOUND if no map is saved.
 */
esp_err_t load_correction_map_from_nvs(ServoCorrectionMap* maps);

#endif // NVS_STORAGE_H