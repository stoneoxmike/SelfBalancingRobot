#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "I2Cdev.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "math.h"
#include <NewPing.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_MPU6050 mpu;


float xSum = 0;
float ySum = 0;
float zSum = 0;
int xCount = 0;
int yCount = 0;
int zCount = 0;
float xAverage = 0;
float yAverage = 0;
float zAverage = 0;


void setup(void) 
{
  Serial.begin(115200);
  Serial.println("MPU6050 Gyro Calibration"); 
  Serial.println("");
  
  /* Initialise the sensor */
  if(!mpu.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no MPU6050 detected ... Check your wiring!");
    while(1);
  }
}

void loop(void)
{
    Serial.println("Type key when ready..."); 
    while (!Serial.available()){}  // wait for a character
    
    /* Get a new sensor event */ 
    sensors_event_t accelEvent, gyroEvent, tempEvent;  
    for ( int i = 0; i<1000; i++) {
      mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);
      xSum += gyroEvent.gyro.x;
      ySum += gyroEvent.gyro.y;
      zSum += gyroEvent.gyro.z;
    }

    xAverage = xSum / 1000;
    yAverage = ySum / 1000;
    zAverage = zSum / 1000;
    
  
    Serial.print("Averages: "); Serial.print(xAverage); Serial.print("  ");Serial.print(yAverage); Serial.print("  "); Serial.print(zAverage); Serial.println();

    while (Serial.available())
    {
      Serial.read();  // clear the input buffer
    }
}