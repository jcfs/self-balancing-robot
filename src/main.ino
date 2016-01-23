#include <stdio.h>

// Arduino libraries
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>

// Local includes
#include "printf.h"
#include "pot_motor_speed.h"
#include "pid_motor_speed.h"
#include "basic_balance_motor_speed.h"
#include "utils.h"

// ================================================================
// ===                        CONSTANTS                         ===
// ================================================================
#define DEBUG     1

#define MOTOR_1_STEP  6 
#define MOTOR_1_DIR   8

#define MOTOR_2_STEP  12
#define MOTOR_2_DIR   13

#define LED_PIN       3

// ================================================================
// ===                        Globals                           ===
// ================================================================
int16_t motor_1_speed;
int16_t motor_2_speed;
int16_t speed_period_m1;
int16_t speed_period_m2;
uint8_t running_mode;

// MPU
MPU6050 mpu;

// ================================================================
// ===                      MPU FUNCTIONS                       ===
// ================================================================
float dmpGetPhi() {
  uint8_t fifoBuffer[64]; // FIFO storage buffer
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
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)

  // run scl at 400khz
  TWSR = 0;
  TWBR = ((16000000L/400000L)-16)/2;
  TWCR = 1<<TWEN;

  mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setDLPFMode(MPU6050_DLPF_BW_20);  //10,20,42,98,188
  mpu.setRate(4);   // 0=1khz 1=500hz, 2=333hz, 3=250hz 4=200hz
  mpu.setSleepEnabled(false);
  uint8_t devStatus = mpu.dmpInitialize();
  prints("Dev status... %d\n", devStatus);

  if (devStatus) {
    digitalWrite(LED_PIN, HIGH);
    while(1);
  } 

  mpu.setDMPEnabled(true);

  int led_status = 1;
  // Yaw stablelizing
  for(uint8_t i = 0; i < 20; i++) {
    printsf(__func__, "Waiting %02d for calibration\r", 20-i);
    digitalWrite(LED_PIN, led_status);
    led_status = !led_status;
    delay(1000);
  }
  prints("\n");
  printsf(__func__, "Calibration done.");

}
// ================================================================
// ===                      Motors Functions                    ===
// ================================================================

// sets the motor direction pin accordingly to the motor speed
void setMotorDirection(int pin, int motorSpeed) {
  digitalWrite(pin, motorSpeed > 0 ? HIGH : LOW);
}

// pulse the motor one step
void step(int motor){
  digitalWrite(motor, HIGH);
  delayMicroseconds(1);
  digitalWrite(motor, LOW);
}

// interrupt function that is called every 25hkz (once every 40 microseconds)
ISR(TIMER1_COMPA_vect) {
  static int16_t counter_m1 = 0;
  static int16_t counter_m2 = 0;

  counter_m1++;
  counter_m2++;

  if (!speed_period_m1 && !speed_period_m2) return;

  if (counter_m1 >= speed_period_m1) {
    counter_m1 = 0;
    step(MOTOR_1_STEP);
  }

  if (counter_m2 >= speed_period_m2) {
    counter_m2 = 0;
    step(MOTOR_2_STEP);
  }
}
// Modules functions
static void (*function[24])(int16_t *, float, float, int16_t, int16_t);
static uint16_t module_counter;

static void register_module(void (*func)(int16_t *, float, float, int16_t, int16_t)) {
  function[module_counter++] = func;
}
//-----------------------------------------
// Timer Setup
// ----------------------------------------
// timer setup, frequency in milliseconds as argument - we are using a divider of 8
// resulting in a 2MHz on a 16MHz cpu
// 40 - 25Khz
void setup_timer(uint16_t freq) {
  TCCR1B &= ~(1<<WGM13);
  TCCR1B |=  (1<<WGM12);
  TCCR1A &= ~(1<<WGM11); 
  TCCR1A &= ~(1<<WGM10);

  // output mode = 00 (disconnected)
  TCCR1A &= ~(3<<COM1A0); 
  TCCR1A &= ~(3<<COM1B0); 

  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer on 16MHz CPU
  TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10);
  OCR1A = freq;   // 25Khz
  TCNT1 = 0;

  delay(3000);
  TIMSK1 |= (1<<OCIE1A);  
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
  pinMode(LED_PIN, OUTPUT);

  // setup MPU6050
  setup_mpu();
  // setup timer
  setup_timer(60);

  // pid setup
  setup_pid();
  // register existing modules
  register_module(get_pid_motor_speed);
  // register potetiometer module
  register_module(get_pot_motor_speed);
  // register basic balance module
  register_module(get_basic_balance_motor_speed);

  prints("Modules loaded\n");
  // load desired module
  prints("Select mode [0-%d]\n", module_counter-1);
//  while(!Serial.available());
//  running_mode = Serial.read() - '0';
  running_mode = 0;
  prints("Selected module: %d\n", running_mode);

}

float angle_adjusted;

// Main loop
void loop() {
  uint16_t fifoCount =  mpu.getFIFOCount();

  if (fifoCount >= 18) {
    float angle_adjusted_old = angle_adjusted;
    angle_adjusted = -dmpGetPhi();
    // if we are in an recoverable position
    if (angle_adjusted > -35 && angle_adjusted < 35) {
      int16_t motor_accel[2];
      // call the running module 
      (*function[running_mode])(motor_accel, angle_adjusted, angle_adjusted_old, motor_1_speed, motor_2_speed);

      // we integrate the acceleration
      motor_1_speed += motor_accel[0];
      motor_2_speed -= motor_accel[1];

      //constrain motor speed
      motor_1_speed = constrain(motor_1_speed, -500, 500);
      motor_2_speed = constrain(motor_2_speed, -500, 500);

      // calculate motor period by the function of f(x)=-0.017*x+10.5
      speed_period_m1 = -0.056*abs(motor_1_speed) + 30;
      speed_period_m2 = -0.056*abs(motor_2_speed) + 30;
    } else {
      // if it is an angle we can't recover we gg and stop the motors
      motor_1_speed = motor_2_speed = 0;
      speed_period_m1 = speed_period_m2 = 0;
    }
  }

#if DEBUG
  runEvery(1000) printsf(__func__, "M1: %d M2: %d angle: %d\n", motor_1_speed, motor_2_speed, (int)angle_adjusted);
#endif

  setMotorDirection(MOTOR_1_DIR, motor_1_speed);
  setMotorDirection(MOTOR_2_DIR, motor_2_speed);
}
