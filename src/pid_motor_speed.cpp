#define DEBUG 1

#include <Arduino.h>
#include <stdint.h>
#include "PID_v1.h"
#include "printf.h"
#include "utils.h"
#include "constants.h"

double pid_output_accel, pid_setpoint_angle, pid_input_angle;

// pid to obtain the desired speed given the current angle and the desired angle
PID pid_controller_speed(&pid_input_angle, &pid_output_accel, &pid_setpoint_angle, KP, KI, KD, DIRECT);

void setup_pid() {
  pid_controller_speed.SetMode(AUTOMATIC);
  pid_controller_speed.SetOutputLimits(-MAX_ACCEL, MAX_ACCEL);
  pid_controller_speed.SetSampleTime(10); // 10ms
}

//  * Get the current angle from the MPU
//  * Feed the current angle to the speed PID to obtain the desired acceleration to get to the desired angle
//  * Adjust motor speeds accordingly
void get_pid_motor_speed(int16_t * motor_accel, float angle, float angle_old, int16_t m1, int16_t m2) {

  pid_setpoint_angle = 2; // for now we'll ignore the first pid computation
  pid_input_angle = angle;
  pid_controller_speed.Compute();

  motor_accel[0] = -(int16_t)pid_output_accel;
  motor_accel[1] = -(int16_t)pid_output_accel;

#if DEBUG
  runEvery(500) {
    printsf(__func__, "Speed PID: setpoint: %d  input: %d output (acceleration): %d\n", (int16_t)pid_setpoint_angle, (int16_t)pid_input_angle, (int16_t)pid_output_accel);
  }
#endif

}
