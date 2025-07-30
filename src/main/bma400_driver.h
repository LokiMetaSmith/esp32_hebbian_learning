/**
 * @file bma400_driver.h
 * @brief Header file for the BMA400 accelerometer driver.
 *
 * This file declares the functions for initializing the BMA400 sensor
 * and reading acceleration data from it via the I2C bus.
 */

#ifndef BMA400_DRIVER_H
#define BMA400_DRIVER_H

#include "driver/i2c_master.h"
#include "driver/gpio.h"

// --- I2C Configuration ---
#define I2C_MASTER_SCL_IO          9
#define I2C_MASTER_SDA_IO          8
#define I2C_MASTER_NUM             I2C_NUM_0
#define I2C_MASTER_FREQ_HZ         400000

// --- BMA400 Sensor Addresses ---
// The default address is 0x14. If that fails, try 0x15.
#define BMA400_I2C_ADDR             0x14

// --- BMA400 Register Map (from Seeed Studio & Bosch Datasheet) ---
#define BMA400_CHIP_ID_REG              0x00
#define BMA400_ERR_REG_REG              0x02
#define BMA400_STATUS_REG               0x03

#define BMA400_ACC_X_LSB_REG            0x04
#define BMA400_ACC_X_MSB_REG            0x05
#define BMA400_ACC_Y_LSB_REG            0x06
#define BMA400_ACC_Y_MSB_REG            0x07
#define BMA400_ACC_Z_LSB_REG            0x08
#define BMA400_ACC_Z_MSB_REG            0x09

#define BMA400_SENSOR_TIME_0_REG        0x0A
#define BMA400_SENSOR_TIME_1_REG        0x0B
#define BMA400_SENSOR_TIME_2_REG        0x0C

#define BMA400_EVENT_REG                0x0D

#define BMA400_INT_STAT0_REG            0x0E
#define BMA400_INT_STAT1_REG            0x0F
#define BMA400_INT_STAT2_REG            0x10

#define BMA400_TEMP_DATA_REG            0x11

#define BMA400_FIFO_LENGTH_0_REG        0x12
#define BMA400_FIFO_LENGTH_1_REG        0x13
#define BMA400_FIFO_DATA_REG            0x14

#define BMA400_STEP_CNT_0_REG           0x15
#define BMA400_STEP_STAT_REG            0x18

#define BMA400_ACC_CONFIG0_REG          0x19
#define BMA400_ACC_CONFIG1_REG          0x1A
#define BMA400_ACC_CONFIG2_REG          0x1B

#define BMA400_INT_CONFIG0_REG          0x1F
#define BMA400_INT_CONFIG1_REG          0x20
#define BMA400_INT_1_MAP_REG            0x21
#define BMA400_INT_2_MAP_REG            0x22
#define BMA400_INT_1_2_MAP_REG          0x23
#define BMA400_INT_1_2_CTRL_REG         0x24

#define BMA400_FIFO_CONFIG_0_REG        0x26
#define BMA400_FIFO_CONFIG_1_REG        0x27
#define BMA400_FIFO_CONFIG_2_REG        0x28
#define BMA400_FIFO_PWR_CONFIG_REG      0x29

#define BMA400_AUTO_LOW_POW_0_REG       0x2A
#define BMA400_AUTO_LOW_POW_1_REG       0x2B
#define BMA400_AUTO_WAKE_UP_0_REG       0x2C
#define BMA400_AUTO_WAKE_UP_1_REG       0x2D
#define BMA400_WAKE_UP_INT_CONFIG_0_REG 0x2F

#define BMA400_CMD_REG                  0x7E

esp_err_t bma400_initialize();
esp_err_t bma400_read_acceleration(float* ax, float* ay, float* az);

#endif // BMA400_DRIVER_H
