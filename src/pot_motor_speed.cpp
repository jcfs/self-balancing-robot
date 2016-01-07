#define DEBUG 1

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#include "printf.h"

// ================================================================
// ===                        CONSTANTS                         ===
// ================================================================
#define POTENT 3
int32_t timer = millis();
int32_t old_timer;

void get_pot_motor_speed(int16_t * motor_speed, float angle, float angle_old, int16_t m1, int16_t m2) {

  int val = map(analogRead(POTENT), 0, 1020, -500, 500);

#if DEBUG
  if (timer-old_timer > 1000) {
    noInterrupts();
    prints("Potentiometer speed value: %d\n", val);
    interrupts();
    old_timer = timer;
  }
  timer = millis();
#endif

  motor_speed[0] = val;
  motor_speed[1] = -val;
}
