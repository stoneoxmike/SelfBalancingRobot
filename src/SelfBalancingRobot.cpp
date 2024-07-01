#include "Wire.h"
#include "I2Cdev.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "math.h"
#include <NewPing.h>

#define leftMotorPWMPin   A1
#define leftMotorDirPin   10
#define rightMotorPWMPin  A2
#define rightMotorDirPin  5

#define Kp  40
#define Kd  0.05
#define Ki  40
#define sampleTime  0.005
#define targetAngle -2.5

long startTime;
long endTime;
long elapsedTime;

Adafruit_MPU6050 mpu;

int16_t accY, accZ, gyroX;
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
void IRAM_ATTR Timer0_ISR()
{

}

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
  //Serial.begin(115200);
  // Try to initialize!
  if (!mpu.begin()) {
    //Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  //Serial.println("MPU6050 Found!");
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
  // initialize PID sampling loop
  //init_PID();
}

void loop() {
  // Serial.print(accY + " ");
  // Serial.print(accZ + " ");
  // Serial.println(gyroX);
  // read acceleration and gyroscope values
  startTime = micros();

  mpu.getEvent(&a, &g, &temp);
  accY = a.acceleration.y;
  accZ = a.acceleration.z;  
  gyroX = g.gyro.x;

  // calculate the angle of inclination
  accAngle = atan2(accY, accZ)*RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate*elapsedTime;  
  currentAngle = 0.9934*(prevAngle + gyroAngle) + 0.0066*(accAngle);
  
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;  
  errorSum = constrain(errorSum, -300, 300);
  //calculate output from P, I and D values
  motorPower = Kp*(error) + Ki*(errorSum)*elapsedTime - Kd*(currentAngle-prevAngle)/elapsedTime;
  prevAngle = currentAngle;
  // set motor power after constraining it
  motorPower = constrain(motorPower, -255, 255);
  setMotors(motorPower, motorPower);
  // toggle the led on pin13 every second
  // count++;
  // if(count == 200)  {
  //   count = 0;
  //   digitalWrite(13, HIGH);
  //   //Serial.println("ISR");
  // }

  endTime = micros();
  elapsedTime = endTime - startTime;
}
