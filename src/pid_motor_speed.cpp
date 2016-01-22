#define DEBUG 0

#include <Arduino.h>
#include <stdint.h>
#include "PID_v1.h"
#include "printf.h"
#include "utils.h"

double pid_output_accel, pid_setpoint_angle, pid_input_angle;

// pid to obtain the desired speed given the current angle and the desired angle
PID pid_controller_speed(&pid_input_angle, &pid_output_accel, &pid_setpoint_angle, 2, 0, 0.2, DIRECT);

void setup_pid() {
  pid_controller_speed.SetMode(AUTOMATIC);
  pid_controller_speed.SetOutputLimits(-40, 40);
}

//  * Get the current angle from the MPU
//  * Feed the current angle to the speed PID to obtain the desired acceleration to get to the desired angle
//  * Adjust motor speeds accordingly
void get_pid_motor_speed(int16_t * motor_accel, float angle, float angle_old, int16_t m1, int16_t m2) {
  pid_setpoint_angle = 5; // for now we'll ignore the first pid computation
  pid_input_angle = angle;
  pid_controller_speed.Compute();

#if DEBUG
  runEvery(1000) {
    printsf(__func__, "Speed PID: setpoint: %f  input: %f output (acceleration): %f", pid_setpoint_angle, pid_input_angle, pid_output_accel);
  }
#endif

  motor_accel[0] = -(int16_t)pid_output_accel;
  motor_accel[1] = -(int16_t)pid_output_accel;
}
