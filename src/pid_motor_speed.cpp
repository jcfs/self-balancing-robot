#define DEBUG 1

#include "PID_v1.h"
#include <stdint.h>
#include "printf.h"
#include <Arduino.h>

double pid_setpoint_angle, pid_input_angle, pid_output_angle;
double pid_setpoint_speed, pid_input_speed, pid_output_speed;

// pid to obtain the desired angle given the current speed and the target speed
PID pid_controller_angle(&pid_input_speed, &pid_output_angle, &pid_setpoint_speed, 1, 0.1, 0.1, DIRECT);
// pid to obtain the desired speed given the current angle and the desired angle
PID pid_controller_speed(&pid_input_angle, &pid_output_speed, &pid_setpoint_angle, 2, 0.1, 0.1, DIRECT);

//  * Get the current angle from the MPU
//  * Estimate the current speed
//  * Feed the current speed to the angle PID to obtain the desired angle to get to the input speed (zero for now
//  * Feed the current angle to the speed PID to obtain the desired speed to ge to the desired angle (obtained be
//  * Adjust motor speeds accordingly

void get_pid_motor_speed(int16_t * motor_speed, float angle, float angle_old, int16_t m1, int16_t m2) {
  // for now our target speed **ALWAYS** is zero
  pid_setpoint_speed = 0;
  // we need to predict our current speed for the inputs we have
  int32_t angular_velocity = (angle - angle_old) * 90;
  pid_input_speed = (m1 - m2) / 2 - (pid_input_speed-angular_velocity);

  pid_controller_angle.Compute();

#if DEBUG
  prints("Angle PID: setpoint: %f input: %f output (angle): %f\n", pid_setpoint_speed, pid_input_speed, pid_output_angle);
#endif

  // PID given the target angle and the current angle
  // outputs the target speed
  pid_setpoint_angle = pid_output_angle;
  pid_input_angle = angle;
  pid_controller_speed.Compute();

#if DEBUG
  prints("Speed PID: setpoint: %f  input: %f output (speed): %f", pid_setpoint_angle, pid_input_angle, pid_output_speed);
  prints("motor speed: %d\n", pid_output_speed);
#endif

  motor_speed[0] = (int16_t)pid_output_speed;
  motor_speed[1] = (int16_t)-pid_output_speed;
}
