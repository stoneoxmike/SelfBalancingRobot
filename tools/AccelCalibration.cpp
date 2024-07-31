#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "I2Cdev.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "math.h"
#include <NewPing.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_MPU6050 mpu;


float AccelMinX = 0;
float AccelMaxX = 0;
float AccelMinY = 0;
float AccelMaxY = 0;
float AccelMinZ = 0;
float AccelMaxZ = 0;


void setup(void) 
{
  Serial.begin(115200);
  Serial.println("MPU6050 Accelerometer Calibration"); 
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
    mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);
    
    if (accelEvent.acceleration.x < AccelMinX) AccelMinX = accelEvent.acceleration.x;
    if (accelEvent.acceleration.x > AccelMaxX) AccelMaxX = accelEvent.acceleration.x;
    
    if (accelEvent.acceleration.y < AccelMinY) AccelMinY = accelEvent.acceleration.y;
    if (accelEvent.acceleration.y > AccelMaxY) AccelMaxY = accelEvent.acceleration.y;
  
    if (accelEvent.acceleration.z < AccelMinZ) AccelMinZ = accelEvent.acceleration.z;
    if (accelEvent.acceleration.z > AccelMaxZ) AccelMaxZ = accelEvent.acceleration.z;
  
    Serial.print("Accel Minimums: "); Serial.print(AccelMinX); Serial.print("  ");Serial.print(AccelMinY); Serial.print("  "); Serial.print(AccelMinZ); Serial.println();
    Serial.print("Accel Maximums: "); Serial.print(AccelMaxX); Serial.print("  ");Serial.print(AccelMaxY); Serial.print("  "); Serial.print(AccelMaxZ); Serial.println();
    Serial.print("Accel Ranges: "); Serial.print(AccelMaxX-AccelMinX); Serial.print("  ");Serial.print(AccelMaxY - AccelMinY); Serial.print("  "); Serial.print(AccelMaxZ-AccelMinZ); Serial.println();

    while (Serial.available())
    {
      Serial.read();  // clear the input buffer
    }
}