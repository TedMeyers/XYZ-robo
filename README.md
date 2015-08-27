# XYZ-robo
Arduino robotics code (Libraries)

RoverRally: Library to assist in IMU(gyro) and wheel-encoder based rovers (built on an RC-car chassis)
XMPU6050: Library for the MPU6050 IMU (from https://github.com/jrowberg/i2cdevlib) included here for convenience
XYZ_MPU6050: Library used to interface with XMPU6050
XYZ_BNO055: Library for the Bosch BNO-055 IMU

I highly recommend the BNO-055 IMU (Adafruit makes a great breakout board), 
but the MPU6050 may also be used and is much more inexpensive (available as a breakout board on e-bay and other places).

Usage:
  1. Download the zip (with the "Download Zip" button).  
  2. Extract the libraries to your arduino library directory.
  3. Open the RoverRally.h file in a text editor and define one of USE_BNO or USE_MPU.
  4. Open Arduino IDE and then open the RoverRally/test sketch from Examples 
     (you will also need to choose USE_BNO or USE_MPU here).

You will need to choose an IMU (the gyro -- is what the RoverRally library uses from the IMU, to get heading).  Be sure
to define one of USE_BNO or USE_MPU in the RoverRally.h file. 
