// Motor signal pin check: one pin at a time, measure it with a multimeter.
//   pio run -e pin_test -t upload && pio device monitor
//
// DIR pins: 3.3 V for 4 s (the motor does not move: its PWM stays 0).
// PWM pins: 20% duty for 4 s, a meter reads ~0.66 V; the motor may turn slowly,
//           which already proves the wire to the MDD10A works.
// Each pin also reads its own pad back: "SHORT?" means it is pulled to GND/another pin.
#include <Arduino.h>
#include <driver/gpio.h>

#include "drive_hw.h"

static const uint32_t STEP_MS = 4000;
static const float PWM_TEST_DUTY = 0.20f;

static void allLow() {
  for (int i = 0; i < 4; i++) {
    ledcDetachPin(WHEEL_CONFIG[i].pwm_pin);
    pinMode(WHEEL_CONFIG[i].pwm_pin, OUTPUT);
    digitalWrite(WHEEL_CONFIG[i].pwm_pin, LOW);
    pinMode(WHEEL_CONFIG[i].dir_pin, OUTPUT);
    digitalWrite(WHEEL_CONFIG[i].dir_pin, LOW);
  }
}

// Drive the pin HIGH with its input buffer on, so the real pad level can be read
static int driveHighAndReadBack(int pin) {
  gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT_OUTPUT);
  gpio_set_level((gpio_num_t)pin, 1);
  delay(5);
  return gpio_get_level((gpio_num_t)pin);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  allLow();
  Serial.println("\nMotor signal pin test: one pin at a time, 4 s each. Measure pin vs ESP32 GND.");
}

void loop() {
  for (int i = 0; i < 4; i++) {
    const WheelConfig &c = WHEEL_CONFIG[i];

    allLow();
    int pad = driveHighAndReadBack(c.dir_pin);
    Serial.printf("%s DIR  GPIO%-2d = HIGH 3.3 V   (pad reads %d%s)\n", MOTOR_NAMES[i], c.dir_pin,
                  pad, pad ? "" : "  SHORT? pin is pulled low");
    delay(STEP_MS);

    allLow();
    pad = driveHighAndReadBack(c.pwm_pin);  // short self-check before switching to PWM
    ledcSetup(i, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
    ledcAttachPin(c.pwm_pin, i);
    ledcWrite(i, (int)(PWM_TEST_DUTY * PWM_MAX));
    Serial.printf("%s PWM  GPIO%-2d = 20%% duty ~0.66 V (pad reads %d%s)\n", MOTOR_NAMES[i], c.pwm_pin,
                  pad, pad ? "" : "  SHORT? pin is pulled low");
    delay(STEP_MS);
    ledcWrite(i, 0);
  }
  allLow();
  Serial.println("----- all LOW, repeating -----");
  delay(2000);
}
