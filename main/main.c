#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "can_common.h"
#include "nvs.h"

#include "bno055.h"
#include "wifi_config.h"
#include "ota.h"
#include "discovery.h"

static const char *TAG = "plateau";

// =============================================================================
// Pin Definitions — Waveshare ESP32-S3-RS485-CAN
// =============================================================================

#define I2C_SDA_PIN  5    // ribbon cable wire 13
#define I2C_SCL_PIN  6    // ribbon cable wire 15
#define CAN_TX_PIN   15   // onboard isolated transceiver
#define CAN_RX_PIN   16

// =============================================================================
// CAN Bus Configuration
// =============================================================================

#define CAN_ID_OTA_TRIGGER       0x00
#define CAN_ID_WIFI_CONFIG       0x01
#define CAN_ID_DISCOVERY_TRIGGER 0x02
// Plateau CAN IDs were reallocated 2026-05-19 to the contiguous 0x36-0x39 block
// (matches TrailCurrent.dbc). Previous allocation was 0x20 / 0x30 / 0x31 / 0x32 —
// 0x20 was reassigned to Borealis safety frame, so the leveling group moved as one.
#define CAN_ID_LEVELING_CFG      0x36
#define CAN_ID_TILT_DATA         0x37
#define CAN_ID_CORNER_DATA       0x38
#define CAN_ID_STATUS_DATA       0x39

#define STATUS_TX_INTERVAL_MS    33    // ~30 Hz
#define TX_PROBE_INTERVAL_MS     2000  // slow probe when no peers

// =============================================================================
// Timing
// =============================================================================

#define LEVELING_INTERVAL_US     (500 * 1000LL)   // 500 ms
#define STATUS_INTERVAL_US       (2000 * 1000LL)  // 2 s
#define IMU_RETRY_INTERVAL_US    (5000 * 1000LL)  // 5 s

// =============================================================================
// Leveling State
// =============================================================================

typedef struct {
    bno055_mount_t mounting;
    uint16_t vehicle_length_cm;
    uint16_t vehicle_width_cm;
} leveling_config_t;

typedef struct {
    float pitch_deg;
    float roll_deg;
    int16_t front_back_diff_mm;
    int16_t left_right_diff_mm;
    bno055_cal_status_t cal;
    bool imu_connected;
} leveling_data_t;

static leveling_config_t s_config = {
    .mounting = MOUNT_FLOOR,
    .vehicle_length_cm = 500,
    .vehicle_width_cm  = 200,
};
static leveling_data_t s_data;
static bool s_has_stored_offsets = false;

// =============================================================================
// NVS — Leveling Config
// =============================================================================

static void save_leveling_config(void)
{
    nvs_handle_t handle;
    if (nvs_open("leveling", NVS_READWRITE, &handle) != ESP_OK) return;
    nvs_set_u8(handle, "mounting", (uint8_t)s_config.mounting);
    nvs_set_u16(handle, "length_cm", s_config.vehicle_length_cm);
    nvs_set_u16(handle, "width_cm", s_config.vehicle_width_cm);
    nvs_commit(handle);
    nvs_close(handle);
    ESP_LOGI(TAG, "Leveling config saved to NVS");
}

static void load_leveling_config(void)
{
    nvs_handle_t handle;
    if (nvs_open("leveling", NVS_READONLY, &handle) != ESP_OK) return;  // keep defaults

    uint8_t mount_val = 0;
    if (nvs_get_u8(handle, "mounting", &mount_val) == ESP_OK) {
        s_config.mounting = (bno055_mount_t)mount_val;
    }

    uint16_t len = s_config.vehicle_length_cm;
    uint16_t wid = s_config.vehicle_width_cm;
    nvs_get_u16(handle, "length_cm", &len);
    nvs_get_u16(handle, "width_cm", &wid);
    s_config.vehicle_length_cm = len;
    s_config.vehicle_width_cm = wid;
    nvs_close(handle);

    ESP_LOGI(TAG, "Loaded config: mount=%d, length=%dcm, width=%dcm",
             s_config.mounting, s_config.vehicle_length_cm, s_config.vehicle_width_cm);
}

// =============================================================================
// NVS — BNO055 Calibration Offsets
// =============================================================================

static bool load_calibration_offsets(void)
{
    nvs_handle_t handle;
    if (nvs_open("bno_cal", NVS_READONLY, &handle) != ESP_OK) return false;

    uint8_t valid = 0;
    nvs_get_u8(handle, "valid", &valid);
    if (!valid) {
        nvs_close(handle);
        return false;
    }

    bno055_offsets_t offsets;
    size_t len = sizeof(offsets);
    if (nvs_get_blob(handle, "offsets", &offsets, &len) != ESP_OK) {
        nvs_close(handle);
        return false;
    }
    nvs_close(handle);

    // Must be in CONFIG mode to set offsets
    bno055_set_mode(BNO055_MODE_CONFIG);
    bno055_set_offsets(&offsets);

    ESP_LOGI(TAG, "Calibration offsets loaded from NVS");
    return true;
}

static bool save_and_verify_calibration(void)
{
    bno055_offsets_t written;
    bno055_set_mode(BNO055_MODE_CONFIG);
    if (bno055_get_offsets(&written) != ESP_OK) return false;

    nvs_handle_t handle;
    if (nvs_open("bno_cal", NVS_READWRITE, &handle) != ESP_OK) return false;
    nvs_set_blob(handle, "offsets", &written, sizeof(written));
    nvs_set_u8(handle, "valid", 1);

    // Read back and verify
    bno055_offsets_t readback;
    size_t len = sizeof(readback);
    nvs_get_blob(handle, "offsets", &readback, &len);
    uint8_t valid = 0;
    nvs_get_u8(handle, "valid", &valid);
    nvs_commit(handle);
    nvs_close(handle);

    if (!valid || memcmp(&written, &readback, sizeof(written)) != 0) {
        ESP_LOGE(TAG, "Calibration save FAILED verification");
        return false;
    }

    ESP_LOGI(TAG, "Calibration offsets saved and verified");
    return true;
}

// =============================================================================
// Leveling Computation
// =============================================================================

static void compute_leveling(void)
{
    bno055_vec3_t accel;
    if (bno055_read_accel(&accel) != ESP_OK) {
        ESP_LOGW(TAG, "BNO055 accel read failed — marking disconnected");
        s_data.imu_connected = false;
        return;
    }

    s_data.pitch_deg = atan2f(accel.x, sqrtf(accel.y * accel.y + accel.z * accel.z)) * (180.0f / M_PI);
    s_data.roll_deg  = atan2f(accel.y, accel.z) * (180.0f / M_PI);

    float pitch_rad = s_data.pitch_deg * (M_PI / 180.0f);
    float roll_rad  = s_data.roll_deg * (M_PI / 180.0f);

    s_data.front_back_diff_mm = (int16_t)(s_config.vehicle_length_cm * 10.0f * tanf(pitch_rad));
    s_data.left_right_diff_mm = (int16_t)(s_config.vehicle_width_cm * 10.0f * tanf(roll_rad));

    bno055_get_cal_status(&s_data.cal);
}

// =============================================================================
// CAN Transmit Helpers
// =============================================================================

static void send_leveling_data(twai_message_t *tx_msg)
{
    // Tilt + height diffs
    tx_msg->identifier = CAN_ID_TILT_DATA;
    tx_msg->data_length_code = 8;

    int16_t pitch100 = (int16_t)(s_data.pitch_deg * 100.0f);
    int16_t roll100  = (int16_t)(s_data.roll_deg * 100.0f);

    tx_msg->data[0] = (pitch100 >> 8) & 0xFF;
    tx_msg->data[1] = pitch100 & 0xFF;
    tx_msg->data[2] = (roll100 >> 8) & 0xFF;
    tx_msg->data[3] = roll100 & 0xFF;
    tx_msg->data[4] = (s_data.front_back_diff_mm >> 8) & 0xFF;
    tx_msg->data[5] = s_data.front_back_diff_mm & 0xFF;
    tx_msg->data[6] = (s_data.left_right_diff_mm >> 8) & 0xFF;
    tx_msg->data[7] = s_data.left_right_diff_mm & 0xFF;
    twai_transmit(tx_msg, 0);

    // Corner adjustments normalized to lowest = 0
    int16_t fl = s_data.front_back_diff_mm / 2 + s_data.left_right_diff_mm / 2;
    int16_t fr = s_data.front_back_diff_mm / 2 - s_data.left_right_diff_mm / 2;
    int16_t rl = -s_data.front_back_diff_mm / 2 + s_data.left_right_diff_mm / 2;
    int16_t rr = -s_data.front_back_diff_mm / 2 - s_data.left_right_diff_mm / 2;

    int16_t min_val = fl;
    if (fr < min_val) min_val = fr;
    if (rl < min_val) min_val = rl;
    if (rr < min_val) min_val = rr;
    fl -= min_val;
    fr -= min_val;
    rl -= min_val;
    rr -= min_val;

    tx_msg->identifier = CAN_ID_CORNER_DATA;
    tx_msg->data[0] = (fl >> 8) & 0xFF;
    tx_msg->data[1] = fl & 0xFF;
    tx_msg->data[2] = (fr >> 8) & 0xFF;
    tx_msg->data[3] = fr & 0xFF;
    tx_msg->data[4] = (rl >> 8) & 0xFF;
    tx_msg->data[5] = rl & 0xFF;
    tx_msg->data[6] = (rr >> 8) & 0xFF;
    tx_msg->data[7] = rr & 0xFF;
    twai_transmit(tx_msg, 0);
}

static void send_status_data(twai_message_t *tx_msg)
{
    tx_msg->identifier = CAN_ID_STATUS_DATA;
    tx_msg->data_length_code = 4;

    uint8_t flags = 0;
    if (s_data.imu_connected) flags |= 0x01;
    if (s_has_stored_offsets) flags |= 0x02;

    uint8_t cal_packed = (s_data.cal.sys << 6) | (s_data.cal.gyro << 4) |
                         (s_data.cal.accel << 2) | s_data.cal.mag;

    tx_msg->data[0] = flags;
    tx_msg->data[1] = cal_packed;
    tx_msg->data[2] = (uint8_t)s_config.mounting;
    tx_msg->data[3] = 0x00;
    twai_transmit(tx_msg, 0);
}

// =============================================================================
// CAN RX Handlers
// =============================================================================

static void handle_leveling_config(const uint8_t *data, uint8_t len)
{
    if (len < 1) return;

    switch (data[0]) {
    case 0x01: {
        if (len < 7) return;
        uint8_t mount_val = data[1];
        if (mount_val <= MOUNT_RIGHT_WALL) {
            s_config.mounting = (bno055_mount_t)mount_val;
        }
        s_config.vehicle_length_cm = (data[2] << 8) | data[3];
        s_config.vehicle_width_cm  = (data[4] << 8) | data[5];

        if (s_data.imu_connected) {
            bno055_apply_axis_remap(s_config.mounting,
                s_has_stored_offsets ? BNO055_MODE_ACCONLY : BNO055_MODE_NDOF);
        }

        if (data[6] == 0x01) {
            save_leveling_config();
        }

        ESP_LOGI(TAG, "Config updated: mount=%d, length=%dcm, width=%dcm",
                 s_config.mounting, s_config.vehicle_length_cm, s_config.vehicle_width_cm);
        break;
    }
    case 0x02: {
        // Status request — will be sent on next status interval
        break;
    }
    case 0x03: {
        ESP_LOGI(TAG, "Save calibration requested");
        if (!s_data.imu_connected) break;
        if (save_and_verify_calibration()) {
            s_has_stored_offsets = true;
            bno055_apply_axis_remap(s_config.mounting, BNO055_MODE_ACCONLY);
            ESP_LOGI(TAG, "Calibration saved — switched to ACCONLY");
        }
        break;
    }
    }
}

// =============================================================================
// TWAI (CAN) task — runs independently
// =============================================================================


static void twai_task(void *arg)
{
    // Configure alerts BEFORE any bus activity so no error transitions are missed.
    twai_reconfigure_alerts(CAN_COMMON_ALERTS, NULL);

    // Alerts armed — version broadcast TX failure is caught by the state machine.
    can_common_version_broadcast();

    typedef enum { TX_ACTIVE, TX_PROBING } tx_state_t;
    bool bus_off = false;
    tx_state_t tx_state = TX_ACTIVE;
    int tx_fail_count = 0;
    const int TX_FAIL_THRESHOLD = 3;
    int64_t last_tx_us = 0;
    const int64_t tx_period_us = STATUS_TX_INTERVAL_MS * 1000LL;
    const int64_t tx_probe_period_us = TX_PROBE_INTERVAL_MS * 1000LL;

    // Leveling timing
    int64_t last_leveling_us = 0;
    int64_t last_status_us = 0;
    int64_t last_imu_retry_us = 0;

    twai_message_t tx_msg = {0};

    while (1) {
        uint32_t triggered;
        twai_read_alerts(&triggered, pdMS_TO_TICKS(10));

        // --- Bus error handling ---
        if (triggered & TWAI_ALERT_BUS_OFF) {
            ESP_LOGE(TAG, "TWAI bus-off, initiating recovery");
            bus_off = true;
            twai_initiate_recovery();
            // No continue — fall through so RX_DATA in the same poll is still processed.
        }
        if (triggered & TWAI_ALERT_BUS_RECOVERED) {
            ESP_LOGI(TAG, "TWAI bus recovered, restarting");
            twai_start();
            bus_off = false;
            tx_fail_count = 0;
            tx_state = TX_PROBING;
        }
        if (triggered & TWAI_ALERT_ERR_PASS) {
            ESP_LOGW(TAG, "TWAI error passive (no peers ACKing?)");
        }
        if (triggered & TWAI_ALERT_TX_FAILED) {
            if (tx_state == TX_ACTIVE) {
                tx_fail_count++;
                if (tx_fail_count >= TX_FAIL_THRESHOLD) {
                    tx_state = TX_PROBING;
                    ESP_LOGW(TAG, "TWAI no peers detected, entering slow probe");
                }
            }
        }
        if (triggered & TWAI_ALERT_TX_SUCCESS) {
            if (tx_state == TX_PROBING) {
                tx_state = TX_ACTIVE;
                tx_fail_count = 0;
                can_common_version_broadcast();
                ESP_LOGI(TAG, "TWAI probe ACK'd, peer detected, resuming normal TX");
            }
            tx_fail_count = 0;
        }

        // --- Drain received messages ---
        if (triggered & TWAI_ALERT_RX_DATA) {
            if (tx_state == TX_PROBING) {
                tx_state = TX_ACTIVE;
                tx_fail_count = 0;
                can_common_version_broadcast();
                ESP_LOGI(TAG, "TWAI peer detected via RX, resuming normal TX");
            }
            twai_message_t msg;
            while (twai_receive(&msg, 0) == ESP_OK) {
                if (msg.rtr) continue;

                if (msg.identifier == CAN_ID_OTA_TRIGGER) {
                    ota_handle_trigger(msg.data, msg.data_length_code);
                } else if (msg.identifier == CAN_ID_WIFI_CONFIG) {
                    wifi_config_handle_can(msg.data, msg.data_length_code);
                } else if (msg.identifier == CAN_ID_DISCOVERY_TRIGGER) {
                    discovery_handle_trigger();
                } else if (msg.identifier == CAN_ID_LEVELING_CFG) {
                    handle_leveling_config(msg.data, msg.data_length_code);
                }
            }
        }

        // --- WiFi config timeout check ---
        wifi_config_check_timeout();

        // --- IMU retry if disconnected ---
        int64_t now_us = esp_timer_get_time();

        if (!s_data.imu_connected) {
            if (now_us - last_imu_retry_us >= IMU_RETRY_INTERVAL_US) {
                last_imu_retry_us = now_us;
                if (bno055_is_connected()) {
                    s_data.imu_connected = true;
                    ESP_LOGI(TAG, "BNO055 connected on retry");
                    s_has_stored_offsets = load_calibration_offsets();
                    bno055_apply_axis_remap(s_config.mounting,
                        s_has_stored_offsets ? BNO055_MODE_ACCONLY : BNO055_MODE_NDOF);
                }
            }
            // Skip leveling when IMU is disconnected
            continue;
        }

        // --- Read IMU and compute leveling at 500 ms intervals ---
        if (now_us - last_leveling_us >= LEVELING_INTERVAL_US) {
            last_leveling_us = now_us;
            compute_leveling();
        }

        // --- Periodic CAN transmit ---
        int64_t effective_period = (tx_state == TX_PROBING) ? tx_probe_period_us : tx_period_us;
        if (!bus_off && (now_us - last_tx_us >= effective_period)) {
            last_tx_us = now_us;

            // Send leveling data with tilt+corner frames
            if (s_data.imu_connected) {
                send_leveling_data(&tx_msg);
            }
        }

        // --- Status at 2 s intervals ---
        if (!bus_off && (now_us - last_status_us >= STATUS_INTERVAL_US)) {
            last_status_us = now_us;
            send_status_data(&tx_msg);
        }
    }
}

// =============================================================================
// Main application
// =============================================================================

void app_main(void)
{
    // Initialize NVS and WiFi config
    wifi_config_init();
    char ssid[33], password[64];
    if (wifi_config_load(ssid, sizeof(ssid), password, sizeof(password))) {
        ESP_LOGI(TAG, "WiFi credentials loaded");
    } else {
        ESP_LOGW(TAG, "No WiFi credentials — OTA disabled until provisioned via CAN");
    }

    ota_init();
    discovery_init();

    ESP_LOGI(TAG, "=== TrailCurrent Plateau ===");
    ESP_LOGI(TAG, "Vehicle Leveling Module");
    ESP_LOGI(TAG, "Hostname: %s", wifi_config_get_hostname());

    // I2C master bus for the BNO055 (ribbon cable wires 13/15)
    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));

    // Initialize BNO055 IMU
    if (bno055_init(i2c_bus) == ESP_OK) {
        s_data.imu_connected = true;
        ESP_LOGI(TAG, "BNO055 detected");

        s_has_stored_offsets = load_calibration_offsets();
    } else {
        s_data.imu_connected = false;
        ESP_LOGW(TAG, "BNO055 not found — will retry");
    }

    // Load leveling config from NVS
    load_leveling_config();

    // Apply axis remap and set operating mode
    if (s_data.imu_connected) {
        if (s_has_stored_offsets) {
            bno055_apply_axis_remap(s_config.mounting, BNO055_MODE_ACCONLY);
            ESP_LOGI(TAG, "Offsets loaded — ACCONLY mode, ready to level");
        } else {
            bno055_apply_axis_remap(s_config.mounting, BNO055_MODE_NDOF);
            ESP_LOGI(TAG, "No stored offsets — NDOF mode for initial calibration");
        }
    }

    // CAN runs in its own task so bus errors never block app_main
    ESP_ERROR_CHECK(can_common_init(CAN_TX_PIN, CAN_RX_PIN));
    xTaskCreatePinnedToCore(twai_task, "twai", 4096, NULL, 5, NULL, 1);

    ESP_LOGI(TAG, "Setup complete");

    // Main task has nothing else to do — park it
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
