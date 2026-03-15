#pragma once
#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>

enum MountingSurface : uint8_t {
  MOUNT_FLOOR      = 0x00,
  MOUNT_LEFT_WALL  = 0x01,
  MOUNT_RIGHT_WALL = 0x02
};

struct leveling_config_t {
  MountingSurface mounting;
  uint16_t vehicle_length_cm;
  uint16_t vehicle_width_cm;
};

struct leveling_data_t {
  float pitch_deg;
  float roll_deg;
  int16_t front_back_diff_mm;
  int16_t left_right_diff_mm;
  uint8_t cal_sys;
  uint8_t cal_gyro;
  uint8_t cal_accel;
  uint8_t cal_mag;
  bool imu_connected;
};

#endif // GLOBALS_H
