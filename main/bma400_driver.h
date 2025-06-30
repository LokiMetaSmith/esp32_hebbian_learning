#ifndef BMA400_DRIVER_H
#define BMA400_DRIVER_H

#include "driver/i2c_master.h"
#include "driver/gpio.h"

// --- I2C Configuration ---
// CORRECTED PINS: Using GPIO 8 and 9 which are available on the DevKitC-1
#define I2C_MASTER_SCL_IO          9       // GPIO number for I2C clock
#define I2C_MASTER_SDA_IO          8       // GPIO number for I2C data
#define I2C_MASTER_NUM             I2C_NUM_0 // I2C port number
#define I2C_MASTER_FREQ_HZ         400000    // I2C master clock frequency (400kHz)
#define I2C_MASTER_TX_BUF_DISABLE  0
#define I2C_MASTER_RX_BUF_DISABLE  0

// --- BMA400 Sensor Addresses ---
#define BMA400_I2C_ADDR             0x14

// --- BMA400 Register Map ---
#define BMA400_CHIP_ID_REG          0x00
#define BMA400_STATUS_REG           0x03
#define BMA400_ACCD_X_LSB_REG       0x04
#define BMA400_ACC_CONFIG0_REG      0x19
#define BMA400_ACC_CONFIG1_REG      0x1A
#define BMA400_CMD_REG              0x7E

/**
 * @brief Initializes the I2C bus and configures the BMA400 sensor.
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t bma400_initialize();

/**
 * @brief Reads the raw X, Y, and Z acceleration data and converts it to g's.
 *
 * @param ax Pointer to a float to store the X-axis acceleration.
 * @param ay Pointer to a float to store the Y-axis acceleration.
 * @param az Pointer to a float to store the Z-axis acceleration.
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t bma400_read_acceleration(float* ax, float* ay, float* az);

#endif // BMA400_DRIVER_H
