#define DEBUG 0

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
  pid_controller_speed.SetTunings(KP, KI, KD);
  pid_setpoint_angle = -2; // for now we'll ignore the first pid computation
}

//  * Get the current angle from the MPU
//  * Feed the current angle to the speed PID to obtain the desired acceleration to get to the desired angle
//  * Adjust motor speeds accordingly
void get_pid_motor_speed(int16_t * motor_accel, float angle, float angle_old, int16_t m1, int16_t m2) {

  // manual tunning via serial
  if (Serial.available() > 0) {
    int incomingByte = Serial.read();

    Serial.println(incomingByte);
    //1 - KP += 0.1
    //2 - KP -= 0.1
    //3 - KI += 0.1
    //4 - KI -= 0.1

    if ((incomingByte - 48) == 1) KP += 0.01;
    if ((incomingByte - 48) == 2) KP -= 0.01;
    if ((incomingByte - 48) == 4) KI += 0.01;
    if ((incomingByte - 48) == 5) KI -= 0.01;
    if ((incomingByte - 48) == 7) KD += 0.01;
    if ((incomingByte - 48) == 8) KD -= 0.01;


    pid_controller_speed.SetTunings(KP, KI, KD);
    Serial.print("current values = ");
    Serial.print(KP);
    Serial.print(" ");

    Serial.print(KI);
    Serial.print(" ");
    Serial.println(KD);
  }

  pid_input_angle = angle;
  pid_controller_speed.Compute();

  motor_accel[0] = (int16_t)pid_output_accel;
  motor_accel[1] = (int16_t)pid_output_accel;

#if DEBUG
  runEvery(500) {
    printsf(__func__, "Speed PID: setpoint: %d  input: %d output (acceleration): %d\n", (int16_t)pid_setpoint_angle, (int16_t)pid_input_angle, (int16_t)pid_output_accel);
  }
#endif

}
