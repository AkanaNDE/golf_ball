// Motor (Cytron MDD10A) and encoder (36GP-555, ESP32 PCNT hardware) access.
// Shared by the micro-ROS firmware (main.cpp) and the bench test (motor_test.cpp).
#pragma once

#include <Arduino.h>
#include <ESP32Encoder.h>

#include "config.h"

struct WheelConfig {
  int pwm_pin;
  int dir_pin;
  int enc_a;
  int enc_b;
  bool invert_motor;
  bool invert_encoder;
};

static const WheelConfig WHEEL_CONFIG[4] = {{M1_CONFIG}, {M2_CONFIG}, {M3_CONFIG}, {M4_CONFIG}};
static const char *const MOTOR_NAMES[4] = {"M1", "M2", "M3", "M4"};

static ESP32Encoder encoders[4];
static const int PWM_MAX = (1 << PWM_RESOLUTION_BITS) - 1;

inline void driveHwSetup() {
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  for (int i = 0; i < 4; i++) {
    const WheelConfig &c = WHEEL_CONFIG[i];
    pinMode(c.dir_pin, OUTPUT);
    digitalWrite(c.dir_pin, LOW);
    ledcSetup(i, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);  // LEDC channel i for motor i
    ledcAttachPin(c.pwm_pin, i);
    ledcWrite(i, 0);
#if USE_ENCODERS
    encoders[i].attachFullQuad(c.enc_a, c.enc_b);
    encoders[i].clearCount();
#endif
  }
}

// duty: -1.0 (full reverse) .. 1.0 (full forward)
inline void motorWrite(int i, float duty) {
  const WheelConfig &c = WHEEL_CONFIG[i];
  duty = constrain(duty, -1.0f, 1.0f);
  if (c.invert_motor) duty = -duty;
  digitalWrite(c.dir_pin, duty >= 0.0f ? HIGH : LOW);
  ledcWrite(i, (uint32_t)(fabsf(duty) * PWM_MAX));
}

inline void stopAllMotors() {
  for (int i = 0; i < 4; i++) motorWrite(i, 0.0f);
}

// Encoder count, positive = wheel turning forward
inline int64_t encoderCount(int i) {
#if USE_ENCODERS
  int64_t count = encoders[i].getCount();
  return WHEEL_CONFIG[i].invert_encoder ? -count : count;
#else
  (void)i;
  return 0;
#endif
}
