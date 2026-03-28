#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

// BNO055 I2C address
#define BNO055_ADDR 0x28

// Operating modes
#define BNO055_MODE_CONFIG      0x00
#define BNO055_MODE_ACCONLY     0x01
#define BNO055_MODE_NDOF        0x0C

// Calibration offsets (22 bytes: accel XYZ, mag XYZ, gyro XYZ offsets + accel/mag radius)
typedef struct {
    int16_t accel_offset_x;
    int16_t accel_offset_y;
    int16_t accel_offset_z;
    int16_t mag_offset_x;
    int16_t mag_offset_y;
    int16_t mag_offset_z;
    int16_t gyro_offset_x;
    int16_t gyro_offset_y;
    int16_t gyro_offset_z;
    int16_t accel_radius;
    int16_t mag_radius;
} bno055_offsets_t;

// Calibration status (0-3 each)
typedef struct {
    uint8_t sys;
    uint8_t gyro;
    uint8_t accel;
    uint8_t mag;
} bno055_cal_status_t;

// 3-axis vector
typedef struct {
    float x;
    float y;
    float z;
} bno055_vec3_t;

// Mounting orientation
typedef enum {
    MOUNT_FLOOR      = 0,
    MOUNT_LEFT_WALL  = 1,
    MOUNT_RIGHT_WALL = 2,
} bno055_mount_t;

/**
 * Initialize I2C bus and probe for BNO055.
 * Returns ESP_OK if sensor detected and initialized.
 */
esp_err_t bno055_init(int sda_pin, int scl_pin);

/**
 * Check if the BNO055 is responding on I2C.
 */
bool bno055_is_connected(void);

/**
 * Set operating mode (CONFIG, ACCONLY, NDOF, etc.).
 */
esp_err_t bno055_set_mode(uint8_t mode);

/**
 * Apply axis remapping for the given mounting orientation,
 * then switch to target_mode.
 */
esp_err_t bno055_apply_axis_remap(bno055_mount_t mount, uint8_t target_mode);

/**
 * Read accelerometer vector in m/s^2.
 */
esp_err_t bno055_read_accel(bno055_vec3_t *out);

/**
 * Read calibration status.
 */
esp_err_t bno055_get_cal_status(bno055_cal_status_t *out);

/**
 * Read calibration offsets from the sensor.
 */
esp_err_t bno055_get_offsets(bno055_offsets_t *out);

/**
 * Write calibration offsets to the sensor.
 * Sensor must be in CONFIG mode before calling.
 */
esp_err_t bno055_set_offsets(const bno055_offsets_t *offsets);
