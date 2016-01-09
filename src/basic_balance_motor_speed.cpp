#define DEBUG 1

#include <Arduino.h>
#include <stdint.h>
#include "printf.h"
#include "basic_balance_motor_speed.h"
#include "utils.h"

//Basic speed calculator based on the angle measured
//if the angle is positive the speed is +max_speed
//if the angle is negative the speed is -max_speed
void get_basic_balance_motor_speed(int16_t * motor_accel, float angle, float angle_old, int16_t m1, int16_t m2) {
  int16_t speed = 15 * ((angle < 0) ? -1 : 1);

#if DEBUG
  runEvery(1000) prints(__func__, "basic accel value: %d\n", speed);
#endif

  motor_accel[0] = speed;
  motor_accel[1] = speed;
}
