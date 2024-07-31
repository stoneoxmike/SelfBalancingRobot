#include "Wire.h"
#include "I2Cdev.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "math.h"
#include <NewPing.h>

#define leftMotorPWMPin   A1
#define leftMotorDirPin   6
#define rightMotorPWMPin  A2
#define rightMotorDirPin  5

#define referenceRange 19.62
#define referenceLow -9.81
#define rawRangeX 19.47
#define rawRangeY 19.62
#define rawRangeZ 19.94
#define rawLowX -9.42
#define rawLowY -9.73
#define rawLowZ -10.04

#define gyroXOffset -0.06
#define gyroYOffset 0.03
#define gyroZOffset -0.02

// #define Kp  48
// #define Kd  .2
// #define Ki  99
#define Kp  112
#define Kd  0
#define Ki  112
#define sampleTime  0.005
#define targetAngle 1

long startTime;
long endTime;
double elapsedTime;

Adafruit_MPU6050 mpu;

float accX, accY, accZ, gyroY;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
volatile byte count=0;

hw_timer_t *Timer0_Cfg = NULL;
sensors_event_t a, g, temp;

void setMotors(int leftMotorSpeed, int rightMotorSpeed) {
  if(leftMotorSpeed >= 0) {
    analogWrite(leftMotorDirPin, leftMotorSpeed);
    digitalWrite(leftMotorPWMPin, LOW);
  }
  else {
    analogWrite(leftMotorPWMPin, -leftMotorSpeed);
    digitalWrite(leftMotorDirPin, LOW);
  }
  if(rightMotorSpeed >= 0) {
    analogWrite(rightMotorDirPin, rightMotorSpeed);
    digitalWrite(rightMotorPWMPin, LOW);
  }
  else {
    analogWrite(rightMotorPWMPin, -rightMotorSpeed);
    digitalWrite(rightMotorDirPin, LOW);
  }
}

// The ISR will be called every 5 milliseconds
//ISR(TIMER1_COMPA_vect)
// void IRAM_ATTR Timer0_ISR()
// {
//   // mpu.getEvent(&a, &g, &temp);
//   // accY = a.acceleration.y; //m/s^2
//   // accZ = a.acceleration.z;  
//   // gyroX = g.gyro.x; // rad/s

//   // calculate the angle of inclination
//   accAngle = atan2(accY, accZ)*RAD_TO_DEG;
//   gyroAngle = (float)(gyroX*RAD_TO_DEG)*sampleTime;
//   Serial.println(gyroAngle);
//   currentAngle = 0.9934*(prevAngle + gyroAngle) + 0.0066*(accAngle);
  
//   error = currentAngle - targetAngle;
//   errorSum = errorSum + error;  
//   errorSum = constrain(errorSum, -300, 300);
//   //calculate output from P, I and D values
//   motorPower = Kp*(error) + Ki*(errorSum)*sampleTime - Kd*(currentAngle-prevAngle)/sampleTime;
//   prevAngle = currentAngle;
//   // set motor power after constraining it
//   motorPower = constrain(motorPower, -255, 255);
//   setMotors(motorPower, motorPower);
//   // toggle the led on pin13 every second
//   count++;
//   if(count == 200)  {
//     count = 0;
//     digitalWrite(13, !digitalRead(13));
//   }
// }

// void init_PID() {
//   // Tout = Ticks * (Prescale/APB_CLK)
//   // given 240MHz frequency, 240 prescale this makes calculations easy
//   // 1 Tick is 1us
//   // init Timer0, 80 prescale division, count up
//   Timer0_Cfg = timerBegin(0, 240, true);
//   // attach ISR to Timer
//   timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
//   // set count to 5000 equalling 5ms, autoreload true
//   timerAlarmWrite(Timer0_Cfg, 5000, true);
//   timerAlarmEnable(Timer0_Cfg);
// }

void setup() {
  Serial.begin(115200);
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // set the motor control and PWM pins to output mode
  pinMode(leftMotorPWMPin, OUTPUT);
  pinMode(leftMotorDirPin, OUTPUT);
  pinMode(rightMotorPWMPin, OUTPUT);
  pinMode(rightMotorDirPin, OUTPUT);
  // set the status LED to output mode 
  pinMode(13, OUTPUT);
  // initialize the MPU6050 and set offset values
  // mpu.setYAccelOffset(1593);
  // mpu.setZAccelOffset(963);
  // mpu.setXGyroOffset(40);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  // initialize PID sampling loop
  // init_PID();
}

void loop() {
  // get time at beginning of loop in microseconds
  startTime = micros();

  // read acceleration and gyroscope values
  mpu.getEvent(&a, &g, &temp);
  accX = (((a.acceleration.x - rawLowX) * referenceRange) / rawRangeX) + referenceLow; //m/s^2
  accY = (((a.acceleration.y - rawLowY) * referenceRange) / rawRangeY) + referenceLow; //m/s^2
  accZ = (((-1 * a.acceleration.z - rawLowZ) * referenceRange) / rawRangeZ) + referenceLow;
  gyroY = g.gyro.y + gyroYOffset; // rad/s

  // calculate the angle of inclination
  accAngle = atan2(accX, accZ)*RAD_TO_DEG;
  gyroAngle = (float)(gyroY*RAD_TO_DEG)*elapsedTime;
  currentAngle = 0.9934*(prevAngle + gyroAngle) + 0.0066*(accAngle);
  
  // calculate error
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;  
  errorSum = constrain(errorSum, -300, 300);
  // Serial.print("AccX: ");
  // Serial.print(accX, 3);
  // Serial.print("    AccY: ");
  // Serial.print(accY, 3);
  // Serial.print("    AccZ: ");
  // Serial.print(accZ, 3);
  // Serial.print("    GyroY: ");
  // Serial.print(gyroY, 3);
  Serial.print("Current Angle: ");
  Serial.print(currentAngle, 3);
  Serial.print("   targetAngle: ");
  Serial.print(targetAngle);
  Serial.print("   error: ");
  Serial.println(error);

  // calculate output from P, I and D values
  motorPower = Kp*(error) + Ki*(errorSum)*elapsedTime - Kd*(currentAngle-prevAngle)/elapsedTime;
  prevAngle = currentAngle;

  // set motor power after constraining it
  motorPower = constrain(motorPower, -255, 255);
  setMotors(motorPower, motorPower);

  // toggle the led on pin13 every second
  count++;
  if(count == 200)  {
    count = 0;
    digitalWrite(13, !digitalRead(13));
  }

  // calculate elapsed time in seconds
  endTime = micros();
  elapsedTime = (double)(endTime - startTime) / 1000000;
  // Serial.println(elapsedTime);
}
