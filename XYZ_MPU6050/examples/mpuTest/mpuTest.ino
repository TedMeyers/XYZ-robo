/* XYZ_MPU6050 Test code
 by Ted Meyers  (4/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.

 This sketch tests the MPU6050 6dof IMU.  Open a terminal
 window at 115200 to see the results.
 It should print out Heading and YPR values.

 Uses Wire and I2Cdev libraries
*/

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include "I2Cdev.h"
#include "XYZ_MPU6050.h"


XYZ_MPU6050 mpu;

uint32_t out_time = 0;  // used to control display output rate
uint32_t up_time = 0;   // sensor update time



void setup(void) 
{ 
  mpu.setupI2C();

  TWBR = 12;  // set 400kHz mode @ 16MHz CPU or 200kHz mode @ 8MHz CPU

  // Setup for Master mode, pins 16/17, external pullups, 400kHz for Teensy 3.1
  //  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);

  // Setup serial connection
  delay(1000);
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("MPU6050 Sensor Test"));
  
  // Initialise the sensor
  while (mpu.setup())
  {
    Serial.println(F("No MPU6050 found"));
    delay(2000);
  }
  
  Serial.println(F("Calibrating, do not move..."));
  for (int i=0; i<1000; i++) {
    delay(10);
    mpu.updateYPR();
    if (i%50==0) Serial.print(".");    
  }
  Serial.println(F("Calibration complete"));

  mpu.updateYPR();
  float ho = mpu.setHeadingOffset();
  Serial.print("Heading Offset = "); Serial.println(ho);
}

void loop(void) 
{
  // Update frequently, so that the queue does not fill up
  if ((millis() - up_time) >= 20) {
    mpu.updateYPR();
    up_time = millis();
  }

 // Print out values
 if ((millis() - out_time) >= 200) {
    mpu.updateYPR();
    Serial.print("H: ");
    Serial.print(mpu.getHeading(), 3);
    Serial.print("  YPR: ");
    Serial.print(mpu.getYaw(), 2); Serial.print(", ");
    Serial.print(mpu.getPitch(), 2); Serial.print(", ");
    Serial.print(mpu.getRoll(), 2); Serial.print("  ");
    Serial.println();
  }
}
