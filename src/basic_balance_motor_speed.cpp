#define DEBUG 1

#include <Arduino.h>
#include <stdint.h>
#include "printf.h"
#include "basic_balance_motor_speed.h"

static int32_t basic_timer = millis();
static int32_t basic_old_timer;

//Basic speed calculator based on the angle measured
//if the angle is positive the speed is +max_speed
//if the angle is negative the speed is -max_speed
void get_basic_balance_motor_speed(int16_t * motor_speed, float angle, float angle_old, int16_t m1, int16_t m2) {
  int16_t speed = 500 * ((angle < 0) ? -1 : 1);

#if DEBUG
  if (basic_timer-basic_old_timer > 1000) {
    noInterrupts();
    prints("Speed value: %d\n", speed);
    interrupts();
    basic_old_timer = basic_timer;
  }
  basic_timer = millis();
#endif

  motor_speed[0] = speed;
  motor_speed[1] = -speed;
}
