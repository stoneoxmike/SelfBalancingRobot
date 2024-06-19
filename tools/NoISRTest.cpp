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

Adafruit_MPU6050 mpu;

int16_t accY, accZ, gyroX;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
volatile byte count=0;

hw_timer_t *Timer0_Cfg = NULL;
sensors_event_t a, g, temp;

void setMotors(int leftMotorSpeed, int rightMotorSpeed) {
  if(leftMotorSpeed >= 0) {
    analogWrite(leftMotorPWMPin, leftMotorSpeed);
    digitalWrite(leftMotorDirPin, LOW);
  }
  else {
    analogWrite(leftMotorPWMPin, 255 + leftMotorSpeed);
    digitalWrite(leftMotorDirPin, HIGH);
  }
  if(rightMotorSpeed >= 0) {
    analogWrite(rightMotorPWMPin, rightMotorSpeed);
    digitalWrite(rightMotorDirPin, LOW);
  }
  else {
    analogWrite(rightMotorPWMPin, 255 + rightMotorSpeed);
    digitalWrite(rightMotorDirPin, HIGH);
  }
}

/*
void init_PID() {  
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B    
  // set compare match register to set sample time 5ms
  OCR1A = 9999;    
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}
*/

// The ISR will be called every 5 milliseconds
//ISR(TIMER1_COMPA_vect)
void IRAM_ATTR Timer0_ISR()
{
  // calculate the angle of inclination
  accAngle = atan2(accY, accZ)*RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate*sampleTime;  
  currentAngle = 0.9934*(prevAngle + gyroAngle) + 0.0066*(accAngle);
  
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;  
  errorSum = constrain(errorSum, -300, 300);
  //calculate output from P, I and D values
  motorPower = Kp*(error) + Ki*(errorSum)*sampleTime - Kd*(currentAngle-prevAngle)/sampleTime;
  prevAngle = currentAngle;
  // set motor power after constraining it
  motorPower = constrain(motorPower, -255, 255);
  setMotors(motorPower, motorPower);
  // toggle the led on pin13 every second
  count++;
  if(count == 200)  {
    count = 0;
    digitalWrite(13, !digitalRead(13));
    //Serial.println("ISR");
  }
}

void init_PID() {
  // Tout = Ticks * (Prescale/APB_CLK)
  // given 240MHz frequency, 240 prescale this makes calculations easy
  // 1 Tick is 1us
  // init Timer0, 80 prescale division, count up
  Timer0_Cfg = timerBegin(0, 240, true);
  // attach ISR to Timer
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  // set count to 5000 equalling 5ms, autoreload true
  timerAlarmWrite(Timer0_Cfg, 5000, true);
  timerAlarmEnable(Timer0_Cfg);
}

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
  init_PID();
}

void loop() {
  // read acceleration and gyroscope values
  mpu.getEvent(&a, &g, &temp);
  accY = a.acceleration.y;
  accZ = a.acceleration.z;  
  gyroX = g.gyro.x;
}
