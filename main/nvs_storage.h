#ifndef NVS_STORAGE_H
#define NVS_STORAGE_H

#include "main.h" // Include main header to get network struct definitions
#include "esp_err.h"

// Forward declaration to avoid circular dependency if main.h includes this file.
struct HiddenLayer;
struct OutputLayer;
struct PredictionLayer;
struct ServoCorrectionMap;

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

/**
 * @brief Retrieves the entire neural network as a single binary blob.
 *
 * @param buffer Pointer to the buffer where the network data will be stored.
 * @param size Pointer to a variable that will hold the size of the network data.
 * @return esp_err_t Result of the operation.
 */
esp_err_t get_raw_network_blob(uint8_t **buffer, size_t *size);

/**
 * @brief Writes a binary blob containing the entire neural network to NVS.
 *
 * @param buffer Pointer to the buffer containing the network data.
 * @param size Size of the network data.
 * @return esp_err_t Result of the operation.
 */
esp_err_t set_raw_network_blob(const uint8_t *buffer, size_t size);

/**
 * @brief Saves the neural network state from a cJSON object to NVS.
 *
 * @param nn_json Pointer to the cJSON object containing the neural network data.
 * @return esp_err_t Result of the save operation.
 */
esp_err_t save_network_from_json(const cJSON *nn_json);


#endif // NVS_STORAGE_H