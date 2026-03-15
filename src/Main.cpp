#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <TwaiTaskBased.h>
#include <OtaUpdate.h>
#include <debug.h>
#include "wifiConfig.h"
#include "Globals.h"

// --- Pin definitions ---
#define I2C_SDA 1
#define I2C_SCL 2
#define CAN_TX GPIO_NUM_7
#define CAN_RX GPIO_NUM_8
#define RGB_LED_PIN 21

void setLed(uint8_t r, uint8_t g, uint8_t b) {
    rgbLedWrite(RGB_LED_PIN, g, r, b);
}

// --- CAN IDs ---
#define CAN_ID_OTA_TRIGGER    0x00
#define CAN_ID_WIFI_CONFIG    0x01
#define CAN_ID_LEVELING_CFG   0x20
#define CAN_ID_TILT_DATA      0x30
#define CAN_ID_CORNER_DATA    0x31
#define CAN_ID_STATUS_DATA    0x32

// --- Timing ---
#define LEVELING_INTERVAL_MS  500
#define STATUS_INTERVAL_MS    2000
#define IMU_RETRY_INTERVAL_MS 5000

// --- Global credential buffers ---
char runtimeSsid[33] = {0};
char runtimePassword[64] = {0};

// --- Objects ---
OtaUpdate otaUpdate(180000, runtimeSsid, runtimePassword);
Adafruit_BNO055 bno(55, 0x28, &Wire);

// --- State ---
leveling_config_t config;
leveling_data_t data;
bool calibrationSaved = false;
unsigned long lastLevelingMs = 0;
unsigned long lastStatusMs = 0;
unsigned long lastImuRetryMs = 0;
String cachedHostName;

// --- NVS helpers ---
void saveLevelingConfig() {
    Preferences prefs;
    prefs.begin("leveling", false);
    prefs.putUChar("mounting", (uint8_t)config.mounting);
    prefs.putUShort("length_cm", config.vehicle_length_cm);
    prefs.putUShort("width_cm", config.vehicle_width_cm);
    prefs.end();
    debugln("[Config] Leveling config saved to NVS");
}

void loadLevelingConfig() {
    Preferences prefs;
    prefs.begin("leveling", true);
    config.mounting = (MountingSurface)prefs.getUChar("mounting", MOUNT_FLOOR);
    config.vehicle_length_cm = prefs.getUShort("length_cm", 500);
    config.vehicle_width_cm = prefs.getUShort("width_cm", 200);
    prefs.end();
    debugf("[Config] Loaded: mount=%d, length=%dcm, width=%dcm\n",
           config.mounting, config.vehicle_length_cm, config.vehicle_width_cm);
}

void saveCalibrationOffsets() {
    adafruit_bno055_offsets_t offsets;
    bno.getSensorOffsets(offsets);
    Preferences prefs;
    prefs.begin("bno_cal", false);
    prefs.putBytes("offsets", &offsets, sizeof(offsets));
    prefs.putBool("valid", true);
    prefs.end();
    debugln("[IMU] Calibration offsets saved to NVS");
}

bool loadCalibrationOffsets() {
    Preferences prefs;
    prefs.begin("bno_cal", true);
    bool valid = prefs.getBool("valid", false);
    if (valid) {
        adafruit_bno055_offsets_t offsets;
        prefs.getBytes("offsets", &offsets, sizeof(offsets));
        prefs.end();
        bno.setSensorOffsets(offsets);
        debugln("[IMU] Calibration offsets loaded from NVS");
        return true;
    }
    prefs.end();
    return false;
}

// --- BNO055 axis remapping ---
void applyAxisRemap(MountingSurface mount) {
    bno.setMode(OPERATION_MODE_CONFIG);
    delay(25);

    switch (mount) {
        case MOUNT_FLOOR:
            bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P1);
            bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P1);
            break;

        case MOUNT_LEFT_WALL:
            // X=X, Y=Z, Z=Y with Z sign flipped
            Wire.beginTransmission(0x28);
            Wire.write(0x41);
            Wire.write(0x18);
            Wire.endTransmission();
            Wire.beginTransmission(0x28);
            Wire.write(0x42);
            Wire.write(0x01);
            Wire.endTransmission();
            break;

        case MOUNT_RIGHT_WALL:
            // X=X, Y=Z, Z=Y with Y sign flipped
            Wire.beginTransmission(0x28);
            Wire.write(0x41);
            Wire.write(0x18);
            Wire.endTransmission();
            Wire.beginTransmission(0x28);
            Wire.write(0x42);
            Wire.write(0x02);
            Wire.endTransmission();
            break;
    }

    bno.setMode(OPERATION_MODE_NDOF);
    delay(25);
}

// --- Leveling computation ---
void computeLeveling() {
    sensors_event_t event;
    bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);

    data.pitch_deg = event.orientation.z;
    data.roll_deg = event.orientation.y;

    float pitch_rad = data.pitch_deg * DEG_TO_RAD;
    float roll_rad = data.roll_deg * DEG_TO_RAD;

    data.front_back_diff_mm = (int16_t)(config.vehicle_length_cm * 10.0f * tanf(pitch_rad));
    data.left_right_diff_mm = (int16_t)(config.vehicle_width_cm * 10.0f * tanf(roll_rad));

    bno.getCalibration(&data.cal_sys, &data.cal_gyro, &data.cal_accel, &data.cal_mag);
}

// --- CAN transmit ---
void sendLevelingData() {
    twai_message_t msg = {};
    msg.data_length_code = 8;

    // Tilt + height diffs
    msg.identifier = CAN_ID_TILT_DATA;
    int16_t pitch100 = (int16_t)(data.pitch_deg * 100.0f);
    int16_t roll100 = (int16_t)(data.roll_deg * 100.0f);

    msg.data[0] = (pitch100 >> 8) & 0xFF;
    msg.data[1] = pitch100 & 0xFF;
    msg.data[2] = (roll100 >> 8) & 0xFF;
    msg.data[3] = roll100 & 0xFF;
    msg.data[4] = (data.front_back_diff_mm >> 8) & 0xFF;
    msg.data[5] = data.front_back_diff_mm & 0xFF;
    msg.data[6] = (data.left_right_diff_mm >> 8) & 0xFF;
    msg.data[7] = data.left_right_diff_mm & 0xFF;
    TwaiTaskBased::send(msg);

    // Corner adjustments normalized to lowest = 0
    int16_t fl = data.front_back_diff_mm / 2 + data.left_right_diff_mm / 2;
    int16_t fr = data.front_back_diff_mm / 2 - data.left_right_diff_mm / 2;
    int16_t rl = -data.front_back_diff_mm / 2 + data.left_right_diff_mm / 2;
    int16_t rr = -data.front_back_diff_mm / 2 - data.left_right_diff_mm / 2;

    int16_t minVal = fl;
    if (fr < minVal) minVal = fr;
    if (rl < minVal) minVal = rl;
    if (rr < minVal) minVal = rr;
    fl -= minVal;
    fr -= minVal;
    rl -= minVal;
    rr -= minVal;

    msg.identifier = CAN_ID_CORNER_DATA;
    msg.data[0] = (fl >> 8) & 0xFF;
    msg.data[1] = fl & 0xFF;
    msg.data[2] = (fr >> 8) & 0xFF;
    msg.data[3] = fr & 0xFF;
    msg.data[4] = (rl >> 8) & 0xFF;
    msg.data[5] = rl & 0xFF;
    msg.data[6] = (rr >> 8) & 0xFF;
    msg.data[7] = rr & 0xFF;
    TwaiTaskBased::send(msg);
}

void sendStatusData() {
    twai_message_t msg = {};
    msg.identifier = CAN_ID_STATUS_DATA;
    msg.data_length_code = 4;

    uint8_t flags = 0;
    if (data.imu_connected) flags |= 0x01;
    if (data.cal_sys == 3) flags |= 0x02;
    flags |= 0x04;

    uint8_t calPacked = (data.cal_sys << 6) | (data.cal_gyro << 4) |
                        (data.cal_accel << 2) | data.cal_mag;

    msg.data[0] = flags;
    msg.data[1] = calPacked;
    msg.data[2] = (uint8_t)config.mounting;
    msg.data[3] = 0x00;
    TwaiTaskBased::send(msg);
}

// --- CAN RX handlers ---
void handleOtaTrigger(const uint8_t *data) {
    char updateForHostName[16];

    snprintf(updateForHostName, sizeof(updateForHostName), "esp32-%02X%02X%02X",
             data[0], data[1], data[2]);

    debugf("[OTA] Target: %s, This device: %s\n",
           updateForHostName, cachedHostName.c_str());

    if (cachedHostName.equals(updateForHostName)) {
        debugln("[OTA] Hostname matched - entering OTA mode");
        setLed(0, 0, 40);
        otaUpdate.waitForOta();
        setLed(0, 40, 0);
        debugln("[OTA] OTA mode exited - resuming normal operation");
    }
}

void handleLevelingConfigMessage(const uint8_t *data) {
    switch (data[0]) {
        case 0x01: {
            uint8_t mountVal = data[1];
            if (mountVal <= MOUNT_RIGHT_WALL) {
                config.mounting = (MountingSurface)mountVal;
            }
            config.vehicle_length_cm = (data[2] << 8) | data[3];
            config.vehicle_width_cm = (data[4] << 8) | data[5];

            applyAxisRemap(config.mounting);

            if (data[6] == 0x01) {
                saveLevelingConfig();
            }

            debugf("[Config] Updated: mount=%d, length=%dcm, width=%dcm\n",
                   config.mounting, config.vehicle_length_cm, config.vehicle_width_cm);
            break;
        }
        case 0x02: {
            sendStatusData();
            break;
        }
        case 0x03: {
            debugln("[Config] Zero/tare requested (not yet implemented)");
            break;
        }
    }
}

void onCanRx(const twai_message_t &msg) {
    if (msg.rtr) return;

    if (msg.identifier == CAN_ID_OTA_TRIGGER && msg.data_length_code >= 3) {
        debugln("[OTA] OTA trigger received");
        handleOtaTrigger(msg.data);
    }
    else if (msg.identifier == CAN_ID_WIFI_CONFIG && msg.data_length_code >= 1) {
        wifiConfig::handleCanMessage(msg.data, msg.data_length_code);
    }
    else if (msg.identifier == CAN_ID_LEVELING_CFG && msg.data_length_code >= 1) {
        handleLevelingConfigMessage(msg.data);
    }
    else {
        debugf("CAN RX: ID=0x%03X DLC=%d\n", msg.identifier, msg.data_length_code);
    }
}

void onCanTx(bool success) {
    if (!success) {
        debugln("[CAN] TX FAILED");
    }
}

// --- Setup ---
void setup() {
    Serial.begin(115200);
    delay(1000);
    setLed(0, 40, 0);
    debugln("\n=== TrailCurrent Plateau ===");

    // WiFi config from NVS (provisioned via CAN bus)
    wifiConfig::init();
    wifiConfig::setRuntimeCredentialPtrs(runtimeSsid, runtimePassword);
    if (wifiConfig::loadCredentials(runtimeSsid, runtimePassword)) {
        debugln("[WiFi] Loaded credentials from NVS");
    } else {
        debugln("[WiFi] No credentials in NVS - OTA disabled until provisioned via CAN");
    }

    cachedHostName = otaUpdate.getHostName();
    debugf("[OTA] Device hostname: %s\n", cachedHostName.c_str());

    // IMU — delay lets the ESP32-S3 I2C peripheral settle before first transaction
    Wire.begin(I2C_SDA, I2C_SCL);
    delay(100);

    data.imu_connected = bno.begin();
    if (data.imu_connected) {
        debugln("[IMU] BNO055 detected");
        bno.setExtCrystalUse(true);
        loadCalibrationOffsets();
    } else {
        debugln("[IMU] BNO055 not found - will retry");
        setLed(40, 0, 0);
    }

    loadLevelingConfig();
    if (data.imu_connected) {
        applyAxisRemap(config.mounting);
    }

    // CAN bus
    TwaiTaskBased::onReceive(onCanRx);
    TwaiTaskBased::onTransmit(onCanTx);
    if (TwaiTaskBased::begin(CAN_TX, CAN_RX, 500000, TWAI_MODE_NO_ACK)) {
        debugln("[CAN] Bus initialized");
    } else {
        debugln("[CAN] ERROR: Bus init failed");
    }

    debugln("=== Setup Complete ===\n");
}

// --- Loop ---
void loop() {
    wifiConfig::checkTimeout();

    unsigned long now = millis();

    // Retry IMU connection if not connected
    if (!data.imu_connected) {
        if (now - lastImuRetryMs >= IMU_RETRY_INTERVAL_MS) {
            lastImuRetryMs = now;
            data.imu_connected = bno.begin();
            if (data.imu_connected) {
                debugln("[IMU] BNO055 connected on retry");
                bno.setExtCrystalUse(true);
                loadCalibrationOffsets();
                applyAxisRemap(config.mounting);
                setLed(0, 40, 0);
            }
        }
        return;
    }

    // Read IMU and compute leveling at 500ms intervals
    if (now - lastLevelingMs >= LEVELING_INTERVAL_MS) {
        lastLevelingMs = now;

        computeLeveling();

        if (!calibrationSaved && data.cal_sys == 3) {
            saveCalibrationOffsets();
            calibrationSaved = true;
        }

        if (data.cal_sys == 3) {
            setLed(0, 40, 0);
        } else {
            setLed(40, 40, 0);
        }

        sendLevelingData();
    }

    // Send status at 2s intervals
    if (now - lastStatusMs >= STATUS_INTERVAL_MS) {
        lastStatusMs = now;
        sendStatusData();
    }
}
