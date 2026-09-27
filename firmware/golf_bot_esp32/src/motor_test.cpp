// Bench test for motors and encoders - no ROS needed.
// LIFT THE ROBOT so the wheels are off the ground before running any test.
//
// Upload:  pio run -e motor_test -t upload && pio device monitor
// Commands (type in the serial monitor, then Enter):
//   1..4   run M1..M4 forward for 2 s at the current duty
//   f      run all 4 motors forward for 2 s (every wheel should push the robot forward)
//   + / -  duty +10% / -10%  (default 30%)
//   m      max speed test: all forward at 100% for 3 s, prints rad/s -> MAX_WHEEL_SPEED
//   c      clear encoder counts (then turn a wheel exactly 1 rev by hand -> TICKS_PER_REV)
//   s      stop
// Encoder counts and wheel speed are printed every 250 ms.

#include <Arduino.h>

#include "config.h"
#include "drive_hw.h"

float duty = 0.3f;
int active_mask = 0;          // bit i = motor i running
float active_duty = 0.0f;
uint32_t run_until_ms = 0;
int64_t last_counts[4] = {0, 0, 0, 0};
uint32_t last_print_ms = 0;

void startRun(int mask, float d, uint32_t duration_ms) {
  stopAllMotors();
  active_mask = mask;
  active_duty = d;
  run_until_ms = millis() + duration_ms;
  for (int i = 0; i < 4; i++) {
    if (mask & (1 << i)) motorWrite(i, d);
  }
  Serial.printf("\n>>> run mask=0x%X duty=%d%% for %lu ms\n", mask, (int)(d * 100), duration_ms);
}

void stopRun() {
  stopAllMotors();
  if (active_mask) Serial.println(">>> stop");
  active_mask = 0;
}

void printHelp() {
  Serial.println();
  Serial.println("=== Golf bot motor/encoder test (LIFT THE WHEELS!) ===");
  Serial.println("1..4 = run M1..M4 forward 2 s | f = all forward | m = max speed test");
  Serial.println("+/- = duty +/-10%  | c = clear counts | s = stop | h = help");
  Serial.printf("Current duty: %d%%   TICKS_PER_REV (config): %.0f\n",
                (int)(duty * 100), (float)ENCODER_TICKS_PER_REV);
  Serial.println("Mapping: FL=M" + String(WHEEL_FL + 1) + " FR=M" + String(WHEEL_FR + 1) +
                 " RL=M" + String(WHEEL_RL + 1) + " RR=M" + String(WHEEL_RR + 1));
}

void setup() {
  Serial.begin(115200);
  driveHwSetup();
  stopAllMotors();
  delay(300);
  printHelp();
}

void loop() {
  // Commands
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case '1': case '2': case '3': case '4':
        startRun(1 << (c - '1'), duty, 2000);
        break;
      case 'f': startRun(0xF, duty, 2000); break;
      case 'm': startRun(0xF, 1.0f, 3000); break;
      case '+': duty = min(duty + 0.1f, 1.0f); Serial.printf("duty %d%%\n", (int)(duty * 100)); break;
      case '-': duty = max(duty - 0.1f, 0.1f); Serial.printf("duty %d%%\n", (int)(duty * 100)); break;
      case 'c':
        for (int i = 0; i < 4; i++) { encoders[i].clearCount(); last_counts[i] = 0; }
        Serial.println("counts cleared");
        break;
      case 's': stopRun(); break;
      case 'h': case '?': printHelp(); break;
      default: break;
    }
  }

  if (active_mask && (int32_t)(millis() - run_until_ms) >= 0) stopRun();

  // Status print
  uint32_t now = millis();
  if (now - last_print_ms >= 250) {
    float dt = (now - last_print_ms) / 1000.0f;
    last_print_ms = now;
    for (int i = 0; i < 4; i++) {
      int64_t count = encoderCount(i);
      float rad_s = ((float)(count - last_counts[i]) / ENCODER_TICKS_PER_REV) * TWO_PI / dt;
      last_counts[i] = count;
      Serial.printf("%s cnt=%7lld %6.2f rad/s | ", MOTOR_NAMES[i], (long long)count, rad_s);
    }
    Serial.println();
  }
}
