// Yaw (heading) from an I2C IMU, auto-detected at startup.
// Shared by the micro-ROS firmware (main.cpp) and the bench test (motor_test.cpp).
//
// Supported (detected by I2C address + chip ID register):
//   BNO055            0x28/0x29, CHIP_ID 0xA0   -> fused heading (IMUPLUS mode, no magnetometer)
//   MPU6050/6500/9250 0x68/0x69, WHO_AM_I 0x68/0x70/0x71/0x73 -> gyro Z integrated
//   ICM-20948         0x68/0x69, WHO_AM_I 0xEA  -> gyro Z integrated
// Yaw is counter-clockwise positive (ROS convention), radians, unwrapped (keeps counting
// past +-pi). For gyro-only chips the bias is measured at startup (robot must stand still
// for ~1 s) and re-learned whenever the robot is commanded still.
#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "config.h"

class YawImu {
 public:
  enum Type { NONE, BNO055, MPU6XXX, ICM20948 };

  bool begin() {
    // 100 kHz: the BNO055 stretches the I2C clock, which the ESP32 handles badly at 400 kHz
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN, 100000);
    Wire.setTimeOut(20);
    // One quick probe: setup must stay short (a long boot breaks the micro-ROS connection
    // when the agent resets the board). The BNO055 needs ~650 ms after power-up, so callers
    // retry begin() later if it returns false.
    for (uint8_t a : {0x28, 0x29}) {
      if (read8(a, 0x00) == 0xA0) { addr_ = a; type_ = BNO055; break; }
    }
    if (type_ == NONE) {
      for (uint8_t a : {0x68, 0x69}) {
        uint8_t who = read8(a, 0x75);
        if (who == 0x68 || who == 0x70 || who == 0x71 || who == 0x73 || who == 0x72) {
          addr_ = a; type_ = MPU6XXX; break;
        }
        if (read8(a, 0x00) == 0xEA) { addr_ = a; type_ = ICM20948; break; }
      }
    }
    if (type_ == BNO055) {
      write8(0x3D, 0x00); delay(25);          // OPR_MODE = CONFIG
      write8(0x3E, 0x00); delay(10);          // PWR_MODE = normal
      write8(0x07, 0x00);                     // page 0
      write8(0x3D, 0x08); delay(30);          // OPR_MODE = IMUPLUS (gyro + accel fusion)
    } else if (type_ == MPU6XXX) {
      write8(0x6B, 0x01); delay(50);          // PWR_MGMT_1: wake, PLL clock
      write8(0x1A, 0x03);                     // DLPF 44 Hz
      write8(0x1B, 0x08);                     // GYRO_CONFIG +-500 dps -> 65.5 LSB/dps
      lsb_per_dps_ = 65.5f;
    } else if (type_ == ICM20948) {
      write8(0x7F, 0x00);                     // bank 0
      write8(0x06, 0x01); delay(50);          // PWR_MGMT_1: wake, auto clock
      write8(0x07, 0x00);                     // PWR_MGMT_2: gyro + accel on
      lsb_per_dps_ = 131.0f;                  // default +-250 dps
    }
    if (type_ == NONE) {      // nothing there: release the I2C driver, run without IMU
      Wire.end();
      return false;
    }
    if (type_ == MPU6XXX || type_ == ICM20948) calibrateBias(400);
    last_us_ = micros();
    if (type_ == BNO055) { last_heading_ = bnoHeading(); }
    return type_ != NONE;
  }

  // Call often (every control step). `still` = robot commanded to stand still.
  void update(bool still) {
    uint32_t now = micros();
    float dt = (now - last_us_) * 1e-6f;
    last_us_ = now;
    if (type_ == BNO055) {
      float h = bnoHeading();                 // degrees, clockwise, 0..360
      if (isnan(h)) return;
      float d = h - last_heading_;
      if (d > 180.0f) d -= 360.0f;
      if (d < -180.0f) d += 360.0f;
      last_heading_ = h;
      yaw_ += IMU_YAW_SIGN * -d * DEG_TO_RAD;  // clockwise heading -> CCW yaw
      rate_ = dt > 0 ? IMU_YAW_SIGN * -d * DEG_TO_RAD / dt : 0.0f;
      return;
    }
    if (type_ == NONE) return;
    float gz = rawGyroZ();
    if (isnan(gz)) return;
    if (still && fabsf(gz - bias_) < 2.0f) bias_ += 0.01f * (gz - bias_);  // slow re-learn
    rate_ = IMU_YAW_SIGN * (gz - bias_) * DEG_TO_RAD;
    if (dt > 0 && dt < 0.2f) yaw_ += rate_ * dt;
  }

  float yaw() const { return yaw_; }        // rad, CCW +, unwrapped, 0 at startup
  float rate() const { return rate_; }      // rad/s, CCW +
  bool ok() const { return type_ != NONE; }
  const char *name() const {
    switch (type_) {
      case BNO055: return "BNO055";
      case MPU6XXX: return "MPU6050/6500/9250";
      case ICM20948: return "ICM-20948";
      default: return "none";
    }
  }
  uint8_t address() const { return addr_; }

 private:
  uint8_t read8(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0xFF;
    if (Wire.requestFrom(addr, (uint8_t)1) != 1) return 0xFF;
    return Wire.read();
  }
  void write8(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
  }
  bool readBytes(uint8_t reg, uint8_t *buf, uint8_t n) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(addr_, n) != n) return false;
    for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
    return true;
  }
  float bnoHeading() {                      // EUL_Heading, 16 LSB per degree, little endian
    uint8_t b[2];
    if (!readBytes(0x1A, b, 2)) return NAN;
    return (int16_t)(b[0] | (b[1] << 8)) / 16.0f;
  }
  float rawGyroZ() {                        // deg/s, big endian
    uint8_t b[2];
    uint8_t reg = (type_ == MPU6XXX) ? 0x47 : 0x37;
    if (!readBytes(reg, b, 2)) return NAN;
    return (int16_t)((b[0] << 8) | b[1]) / lsb_per_dps_;
  }
  void calibrateBias(int samples) {
    float sum = 0.0f;
    int n = 0;
    for (int i = 0; i < samples; i++) {
      float g = rawGyroZ();
      if (!isnan(g)) { sum += g; n++; }
      delay(2);
    }
    bias_ = n ? sum / n : 0.0f;
  }

  Type type_ = NONE;
  uint8_t addr_ = 0;
  float lsb_per_dps_ = 65.5f;
  float bias_ = 0.0f;
  float yaw_ = 0.0f;
  float rate_ = 0.0f;
  float last_heading_ = 0.0f;
  uint32_t last_us_ = 0;
};
