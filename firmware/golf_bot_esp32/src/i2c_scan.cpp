// I2C wiring check: line levels + address scan with SDA/SCL as configured and swapped.
//   pio run -e i2c_scan -t upload && pio device monitor
#include <Arduino.h>
#include <Wire.h>

#include "config.h"

static void scan(int sda, int scl) {
  pinMode(sda, INPUT);
  pinMode(scl, INPUT);
  delay(5);
  Serial.printf("\nSDA=GPIO%d SCL=GPIO%d  idle levels: SDA=%d SCL=%d (both must be 1: pull-ups present)\n",
                sda, scl, digitalRead(sda), digitalRead(scl));
  Wire.begin(sda, scl, 100000);
  Wire.setTimeOut(20);
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  device at 0x%02X%s\n", a,
                    a == 0x28 || a == 0x29 ? "  <- BNO055" :
                    a == 0x68 || a == 0x69 ? "  <- MPU/ICM" : "");
      found++;
    }
  }
  if (!found) Serial.println("  no device answered");
  Wire.end();
}

void setup() {
  Serial.begin(115200);
  delay(1500);   // BNO055 boot time
}

void loop() {
  Serial.println("\n===== I2C scan =====");
  scan(IMU_SDA_PIN, IMU_SCL_PIN);
  scan(IMU_SCL_PIN, IMU_SDA_PIN);  // in case SDA and SCL are swapped
  delay(3000);
}
