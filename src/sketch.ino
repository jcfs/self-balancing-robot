#define DEBUG 1

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "TimerOne.h"
#include "PID_v1.h"

// ================================================================
// ===                        CONSTANTS                         ===
// ================================================================
#define MOTOR_1_STEP 6
#define MOTOR_1_DIR 8

#define MOTOR_2_STEP 12
#define MOTOR_2_DIR 13

#define MAX_SPEED 7
#define MAX_ACCEL 7

#define ZERO_SPEED 65535
#define MAX_TARGET_ANGLE 12

float max_target_angle = MAX_TARGET_ANGLE;

// ================================================================
// ===                      MPU FUNCTIONS                       ===
// ================================================================

MPU6050 mpu;

uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer


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

  Serial.print("Dev status...");
  Serial.println(devStatus);

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  mpu.setDMPEnabled(true);
  mpuIntStatus = mpu.getIntStatus();
  packetSize = mpu.dmpGetFIFOPacketSize();

  // Yaw stablelizing

  Serial.println(F("Waiting 20s for calibration"));
  delay(20000);
  while(Serial.available()) Serial.read();
}


void motor_callback(int motor){
  digitalWrite(motor, HIGH);
  delayMicroseconds(1);
  digitalWrite(motor, LOW);
}

void setMotorDirection(int pin, int motorSpeed) {
  digitalWrite(pin, motorSpeed > 0 ? HIGH : LOW);
}

uint16_t motor_1_speed;
uint16_t motor_2_speed;
uint16_t counter_motor;

void callback() {
  counter_motor++;

  int speed = abs(1000 / motor_1_speed);

  if (counter_motor > speed) {
    counter_motor = 0;
    motor_callback(MOTOR_1_STEP);
    motor_callback(MOTOR_2_STEP);
  }
}


double pid_setpoint_angle, pid_input_angle, pid_output_angle;
double pid_setpoint_speed, pid_input_speed, pid_output_speed;

PID pid_controller_angle(&pid_input_speed, &pid_output_angle, &pid_setpoint_speed, 1, 0.1, 0.1, DIRECT);
PID pid_controller_speed(&pid_input_angle, &pid_output_speed, &pid_setpoint_angle, 2, 0.1, 0.1, DIRECT);

void setup() {
  // init comms
  Wire.begin();
  Serial.begin(115200);

  pinMode(MOTOR_1_STEP,OUTPUT);
  pinMode(MOTOR_1_DIR,OUTPUT);
  pinMode(MOTOR_2_STEP,OUTPUT);
  pinMode(MOTOR_1_DIR,OUTPUT);

  pinMode(inPin, INPUT);
  pinMode(outPin, OUTPUT);

  powerOn();
}

void powerOn() {
  setup_mpu();

  Timer1.initialize(60); // 25Khz
  Timer1.attachInterrupt(callback);

  pid_controller_angle.SetMode(AUTOMATIC);
  pid_controller_angle.SetOutputLimits(-max_target_angle, max_target_angle); 

  pid_controller_speed.SetMode(AUTOMATIC);
  pid_controller_speed.SetOutputLimits(-MAX_SPEED, MAX_SPEED); 
}

void loop() {
  float angle_adjusted;


  fifoCount =  mpu.getFIFOCount();
  if (fifoCount >= 18) {
    double angle_adjusted_old = angle_adjusted;
    angle_adjusted = dmpGetPhi();
    if (angle_adjusted > -40 && angle_adjusted < 40) {
      // for now our target speed is zero
      pid_setpoint_speed = 0;
      // we need to predict our current speed for the inputs we have
      int angular_velocity = (angle_adjusted - angle_adjusted_old) * 90;
      pid_input_speed = (motor_1_speed + motor_2_speed) / 2 - (pid_input_speed-angular_velocity);

      pid_controller_angle.Compute();

#if DEBUG
      Serial.print(angle_adjusted);
      Serial.print(" : ");
      Serial.print(pid_setpoint_angle);
      Serial.print(" : ");
      Serial.println(pid_output_angle);  
#endif

      // PID given the target angle and the current angle
      // outputs the target speed
      pid_setpoint_angle = pid_output_angle;
      pid_input_angle = angle_adjusted;
      pid_controller_speed.Compute();

      // motors 2 is reversed
      motor_1_speed = pid_output_speed;
      motor_2_speed = -pid_output_speed;

      setMotorDirection(MOTOR_1_DIR, motor_1_speed);
      setMotorDirection(MOTOR_2_DIR, motor_2_speed);

    }
  }
}
