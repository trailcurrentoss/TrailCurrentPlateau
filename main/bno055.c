#include "bno055.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "bno055";

// ---------------------------------------------------------------------------
// BNO055 register addresses
// ---------------------------------------------------------------------------

#define REG_CHIP_ID         0x00
#define REG_ACC_DATA_X_LSB  0x08
#define REG_CALIB_STAT      0x35
#define REG_OPR_MODE        0x3D
#define REG_AXIS_MAP_CONFIG 0x41
#define REG_AXIS_MAP_SIGN   0x42
#define REG_ACC_OFFSET_X_LSB 0x55

#define BNO055_CHIP_ID_VALUE 0xA0
#define ACC_SCALE            100.0f  // BNO055 accelerometer LSB = 1/100 m/s^2

// Mode switch delay (datasheet: 7ms from any to CONFIG, 19ms from CONFIG to any)
#define MODE_SWITCH_DELAY_MS 25

// ---------------------------------------------------------------------------
// I2C bus handle
// ---------------------------------------------------------------------------

static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_dev = NULL;
static bool s_connected = false;

// ---------------------------------------------------------------------------
// Low-level I2C helpers
// ---------------------------------------------------------------------------

static esp_err_t bno055_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_master_transmit(s_dev, buf, 2, 100);
}

static esp_err_t bno055_read_reg(uint8_t reg, uint8_t *out, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, out, len, 100);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

esp_err_t bno055_init(int sda_pin, int scl_pin)
{
    // Configure I2C master bus
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Add BNO055 device
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BNO055_ADDR,
        .scl_speed_hz = 400000,
    };

    ret = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C device add failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Allow I2C peripheral to settle
    vTaskDelay(pdMS_TO_TICKS(100));

    // Verify chip ID
    uint8_t chip_id = 0;
    ret = bno055_read_reg(REG_CHIP_ID, &chip_id, 1);
    if (ret != ESP_OK || chip_id != BNO055_CHIP_ID_VALUE) {
        ESP_LOGE(TAG, "BNO055 not detected (chip_id=0x%02X, expected 0x%02X)",
                 chip_id, BNO055_CHIP_ID_VALUE);
        s_connected = false;
        return ESP_ERR_NOT_FOUND;
    }

    s_connected = true;
    ESP_LOGI(TAG, "BNO055 detected (chip_id=0x%02X)", chip_id);
    return ESP_OK;
}

bool bno055_is_connected(void)
{
    if (!s_connected || s_dev == NULL) return false;

    uint8_t chip_id = 0;
    esp_err_t ret = bno055_read_reg(REG_CHIP_ID, &chip_id, 1);
    if (ret != ESP_OK || chip_id != BNO055_CHIP_ID_VALUE) {
        s_connected = false;
        return false;
    }
    return true;
}

esp_err_t bno055_set_mode(uint8_t mode)
{
    esp_err_t ret = bno055_write_reg(REG_OPR_MODE, mode);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(MODE_SWITCH_DELAY_MS));
    return ESP_OK;
}

esp_err_t bno055_apply_axis_remap(bno055_mount_t mount, uint8_t target_mode)
{
    esp_err_t ret = bno055_set_mode(BNO055_MODE_CONFIG);
    if (ret != ESP_OK) return ret;

    switch (mount) {
    case MOUNT_FLOOR:
        // P1 default: REMAP_CONFIG=0x24, REMAP_SIGN=0x00
        bno055_write_reg(REG_AXIS_MAP_CONFIG, 0x24);
        bno055_write_reg(REG_AXIS_MAP_SIGN, 0x00);
        break;

    case MOUNT_LEFT_WALL:
        // X=X, Y=Z, Z=Y with Z sign flipped
        bno055_write_reg(REG_AXIS_MAP_CONFIG, 0x18);
        bno055_write_reg(REG_AXIS_MAP_SIGN, 0x01);
        break;

    case MOUNT_RIGHT_WALL:
        // X=X, Y=Z, Z=Y with Y sign flipped
        bno055_write_reg(REG_AXIS_MAP_CONFIG, 0x18);
        bno055_write_reg(REG_AXIS_MAP_SIGN, 0x02);
        break;
    }

    return bno055_set_mode(target_mode);
}

esp_err_t bno055_read_accel(bno055_vec3_t *out)
{
    uint8_t buf[6];
    esp_err_t ret = bno055_read_reg(REG_ACC_DATA_X_LSB, buf, 6);
    if (ret != ESP_OK) return ret;

    int16_t raw_x = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t raw_y = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t raw_z = (int16_t)((buf[5] << 8) | buf[4]);

    out->x = raw_x / ACC_SCALE;
    out->y = raw_y / ACC_SCALE;
    out->z = raw_z / ACC_SCALE;

    return ESP_OK;
}

esp_err_t bno055_get_cal_status(bno055_cal_status_t *out)
{
    uint8_t cal = 0;
    esp_err_t ret = bno055_read_reg(REG_CALIB_STAT, &cal, 1);
    if (ret != ESP_OK) return ret;

    out->sys   = (cal >> 6) & 0x03;
    out->gyro  = (cal >> 4) & 0x03;
    out->accel = (cal >> 2) & 0x03;
    out->mag   = cal & 0x03;

    return ESP_OK;
}

esp_err_t bno055_get_offsets(bno055_offsets_t *out)
{
    // Offsets are 22 bytes starting at REG_ACC_OFFSET_X_LSB
    uint8_t buf[22];
    esp_err_t ret = bno055_read_reg(REG_ACC_OFFSET_X_LSB, buf, 22);
    if (ret != ESP_OK) return ret;

    out->accel_offset_x = (int16_t)((buf[1]  << 8) | buf[0]);
    out->accel_offset_y = (int16_t)((buf[3]  << 8) | buf[2]);
    out->accel_offset_z = (int16_t)((buf[5]  << 8) | buf[4]);
    out->mag_offset_x   = (int16_t)((buf[7]  << 8) | buf[6]);
    out->mag_offset_y   = (int16_t)((buf[9]  << 8) | buf[8]);
    out->mag_offset_z   = (int16_t)((buf[11] << 8) | buf[10]);
    out->gyro_offset_x  = (int16_t)((buf[13] << 8) | buf[12]);
    out->gyro_offset_y  = (int16_t)((buf[15] << 8) | buf[14]);
    out->gyro_offset_z  = (int16_t)((buf[17] << 8) | buf[16]);
    out->accel_radius   = (int16_t)((buf[19] << 8) | buf[18]);
    out->mag_radius     = (int16_t)((buf[21] << 8) | buf[20]);

    return ESP_OK;
}

esp_err_t bno055_set_offsets(const bno055_offsets_t *offsets)
{
    uint8_t buf[23];
    buf[0]  = REG_ACC_OFFSET_X_LSB;  // register address
    buf[1]  = offsets->accel_offset_x & 0xFF;
    buf[2]  = (offsets->accel_offset_x >> 8) & 0xFF;
    buf[3]  = offsets->accel_offset_y & 0xFF;
    buf[4]  = (offsets->accel_offset_y >> 8) & 0xFF;
    buf[5]  = offsets->accel_offset_z & 0xFF;
    buf[6]  = (offsets->accel_offset_z >> 8) & 0xFF;
    buf[7]  = offsets->mag_offset_x & 0xFF;
    buf[8]  = (offsets->mag_offset_x >> 8) & 0xFF;
    buf[9]  = offsets->mag_offset_y & 0xFF;
    buf[10] = (offsets->mag_offset_y >> 8) & 0xFF;
    buf[11] = offsets->mag_offset_z & 0xFF;
    buf[12] = (offsets->mag_offset_z >> 8) & 0xFF;
    buf[13] = offsets->gyro_offset_x & 0xFF;
    buf[14] = (offsets->gyro_offset_x >> 8) & 0xFF;
    buf[15] = offsets->gyro_offset_y & 0xFF;
    buf[16] = (offsets->gyro_offset_y >> 8) & 0xFF;
    buf[17] = offsets->gyro_offset_z & 0xFF;
    buf[18] = (offsets->gyro_offset_z >> 8) & 0xFF;
    buf[19] = offsets->accel_radius & 0xFF;
    buf[20] = (offsets->accel_radius >> 8) & 0xFF;
    buf[21] = offsets->mag_radius & 0xFF;
    buf[22] = (offsets->mag_radius >> 8) & 0xFF;

    return i2c_master_transmit(s_dev, buf, 23, 100);
}
