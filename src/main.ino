#define DEBUG     1

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "TimerOne.h"
#include "printf.h"
#include <stdio.h>

// Local includes
#include "pot_motor_speed.h"
#include "pid_motor_speed.h"
#include "basic_balance_motor_speed.h"

// ================================================================
// ===                        CONSTANTS                         ===
// ================================================================
#define MOTOR_1_STEP 8
#define MOTOR_1_DIR 9

#define MOTOR_2_STEP 12
#define MOTOR_2_DIR 13

#define MAX_SPEED 500
#define MAX_TARGET_ANGLE 12

// ================================================================
// ===                        Globals                           ===
// ================================================================
int16_t motor_1_speed;
int16_t motor_2_speed;
int16_t counter_motor;
uint8_t running_mode;

// MPU
MPU6050 mpu;

uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// ================================================================
// ===                      MPU FUNCTIONS                       ===
// ================================================================
float dmpGetPhi() {
  VectorFloat gravity;
  Quaternion q;
  float ypr[3];

  mpu.getFIFOBytes(fifoBuffer, 16); // We only read the quaternion
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.resetFIFO();  // We always reset FIFO
  float result = ( asin(-2*(q.x * q.z - q.w * q.y)) * 180/M_PI);

  return result;

}

// ================================================================
// ===                      MPU SETUP                           ===
// ================================================================
void setup_mpu(){
  mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setDLPFMode(MPU6050_DLPF_BW_20);  //10,20,42,98,188
  mpu.setRate(9);   // 0=1khz 1=500hz, 2=333hz, 3=250hz 4=200hz
  mpu.setSleepEnabled(false);
  uint8_t devStatus = mpu.dmpInitialize();

  prints("Dev status... %d\n", devStatus);

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  mpu.setDMPEnabled(true);
  mpuIntStatus = mpu.getIntStatus();
  packetSize = mpu.dmpGetFIFOPacketSize();

  // Yaw stablelizing

  prints("Waiting 20s for calibration\n");
  delay(20000);

}
// ================================================================
// ===                      Motors Functions                    ===
// ================================================================
void setMotorDirection(int pin, int motorSpeed) {
  digitalWrite(pin, motorSpeed > 0 ? HIGH : LOW);
}

void step(int motor){
  digitalWrite(motor, HIGH);
  delayMicroseconds(1);
  digitalWrite(motor, LOW);
}

void step_motors() {
  counter_motor++;

  int speed = -0.017*abs(motor_1_speed) + 9.5;

  if (counter_motor >= speed) {
    counter_motor = 0;
    step(MOTOR_1_STEP);
    step(MOTOR_2_STEP);
  }
}

// ======================================================
// Modules functions
// ========================
void (*function[24])(int16_t *, float, float, int16_t, int16_t);
uint16_t module_counter;

void register_module(void (*func)(int16_t *, float, float, int16_t, int16_t)) {
  function[module_counter++] = func;
}

// initialization functions
void setup() {
  // init comms
  Wire.begin();
  Serial.begin(115200);

  pinMode(MOTOR_1_STEP,OUTPUT);
  pinMode(MOTOR_1_DIR,OUTPUT);
  pinMode(MOTOR_2_STEP,OUTPUT);
  pinMode(MOTOR_2_DIR,OUTPUT);

  // setup MPU6050
  setup_mpu();

  // Set timer one frequency and call back function for interruption
  Timer1.initialize(60); // 25Khz
  Timer1.attachInterrupt(step_motors);

  register_module(get_pid_motor_speed);
  register_module(get_pot_motor_speed);
  register_module(get_basic_balance_motor_speed);

  prints("Select mode [0-%d]\n", module_counter-1);
  while(!Serial.available());
  running_mode = Serial.read() - '0';
  prints("Selected module: %d\n", running_mode);

}

float angle_adjusted;

// Main loop
void loop() {
  int16_t motor_speed[2];

  fifoCount =  mpu.getFIFOCount();

  if (fifoCount > 18) {
    float angle_adjusted_old = angle_adjusted;
    angle_adjusted = dmpGetPhi();
    // if we are in an unrecoverable position
    if (angle_adjusted > -15 && angle_adjusted < 15) {
      (*function[running_mode])(motor_speed, angle_adjusted, angle_adjusted_old, motor_1_speed, motor_2_speed);
      motor_1_speed = motor_speed[0];
      motor_2_speed = motor_speed[1];
    }

  }

#if DEBUG
  //prints("M1: %d M2: %d\n", motor_1_speed, motor_2_speed);
#endif

  setMotorDirection(MOTOR_1_DIR, motor_1_speed);
  setMotorDirection(MOTOR_2_DIR, motor_2_speed);
}
