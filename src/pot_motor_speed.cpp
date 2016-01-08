#define DEBUG 1

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#include "printf.h"

// ================================================================
// ===                        CONSTANTS                         ===
// ================================================================
#define POTENT 3
int timer, old_timer;

void get_pot_motor_speed(int16_t * motor_accel, float angle, float angle_old, int16_t m1, int16_t m2) {
  int16_t val = map(analogRead(POTENT), 0, 1020, -15, 15);
#if DEBUG
  timer = millis();
  if (timer-old_timer > 1000) {
    prints("Potentiometer accel value: %d\n", val);
    old_timer = timer;
  }
#endif
  motor_accel[0] = val;
  motor_accel[1] = val;
}
