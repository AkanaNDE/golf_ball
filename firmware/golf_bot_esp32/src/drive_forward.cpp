// Simple drive test: all 4 motors forward at a fixed speed - no ROS.
//
// Upload:  pio run -e drive_forward -t upload
// Behaviour:
//   - waits START_DELAY_MS after power-on (time to step back)
//   - ramps all 4 wheels up to FORWARD_DUTY and keeps driving forward
//   - press the BOOT button (GPIO0) on the ESP32 to stop / start again
//   - prints each wheel's speed (rad/s) on the serial monitor every 500 ms

#include <Arduino.h>

#include "config.h"
#include "drive_hw.h"

#define FORWARD_DUTY    0.30f   // 30% PWM
#define START_DELAY_MS  3000    // wait before moving
#define RAMP_TIME_MS    1000    // 0 -> FORWARD_DUTY in 1 s
#define BOOT_BUTTON_PIN 0

bool running = true;
float duty = 0.0f;
uint32_t last_ramp_ms = 0;
uint32_t last_print_ms = 0;
int64_t last_counts[4] = {0, 0, 0, 0};

void setup() {
  Serial.begin(115200);
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED_PIN, OUTPUT);
  driveHwSetup();
  stopAllMotors();

  Serial.printf("\nDrive forward test: %d%% duty, starting in %d s (BOOT button = stop/start)\n",
                (int)(FORWARD_DUTY * 100), START_DELAY_MS / 1000);
  delay(START_DELAY_MS);
  last_ramp_ms = millis();
}

void loop() {
  // BOOT button toggles stop/start (simple debounce)
  static bool last_button = HIGH;
  bool button = digitalRead(BOOT_BUTTON_PIN);
  if (button == LOW && last_button == HIGH) {
    running = !running;
    Serial.println(running ? ">>> START" : ">>> STOP");
    delay(50);
  }
  last_button = button;

  // Ramp duty toward the target
  uint32_t now = millis();
  float step = FORWARD_DUTY * (now - last_ramp_ms) / (float)RAMP_TIME_MS;
  last_ramp_ms = now;
  float target = running ? FORWARD_DUTY : 0.0f;
  duty = (duty < target) ? min(duty + step, target) : max(duty - step, target);

  for (int i = 0; i < 4; i++) motorWrite(i, duty);
  digitalWrite(STATUS_LED_PIN, running ? HIGH : LOW);

  // Wheel speed print
  if (now - last_print_ms >= 500) {
    float dt = (now - last_print_ms) / 1000.0f;
    last_print_ms = now;
    Serial.printf("duty %3d%% | ", (int)(duty * 100));
    for (int i = 0; i < 4; i++) {
      int64_t count = encoderCount(i);
      float rad_s = ((float)(count - last_counts[i]) / ENCODER_TICKS_PER_REV) * TWO_PI / dt;
      last_counts[i] = count;
      Serial.printf("%s %6.2f rad/s  ", MOTOR_NAMES[i], rad_s);
    }
    Serial.println();
  }
  delay(10);
}
