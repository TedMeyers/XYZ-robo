# XYZ-robo
Arduino robotics code (Libraries)

- XYZ_Rover: Library to assist in IMU(gyro) and wheel-encoder based rovers (built on an RC-car chassis)
      Optionally uses the TiCoServer library
- XMPU6050: Library for the MPU6050 IMU (from https://github.com/jrowberg/i2cdevlib) included here for convenience
- XYZ_MPU6050: Library used to interface with XMPU6050.  NOTE: uses the stripped down MPU6050 library (see above).
- XYZ_BNO055: Library for the Bosch BNO-055 IMU
- XYZ_Single_Encoder - Library for using Single Wheel Encoders.  NOTE: uses the EnableInterrupts library
- XYZ_Quadrature_Encoder - Library for using Quadrature Wheel Encoders.  NOTE: uses the EnableInterrupts library

Other libraries optionally used:
- EnableInterrupts: https://github.com/GreyGnome/EnableInterrupt
- TiCoServo:        https://github.com/adafruit/Adafruit_TiCoServo
- I2cDevLib:        https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev
- MPU6050:          https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050

I highly recommend the BNO-055 IMU (Adafruit makes a great breakout board), 
but the MPU6050 may also be used and is much less expensive (available as a breakout board on e-bay and other places).

The Adafruit TiCoServo (Timer/Counter servos) library is (optionally) used by XYZ_Rover -- using the timer/counter limits
the number of servos and the specific servo pins, but is much more reliable when used with other code that uses 
interrupts, such as wheel encoders and I2C communications.

Usage:
  1. Download the zip (with the "Download Zip" button).  
  2. Extract the libraries to your arduino library directory.  (Steps 1 and 2 may be required for the other libs listed above)
  3. Open the XYZ_Rover.h file in a text editor and define one of USE_BNO or USE_MPU.
  4. Open Arduino IDE and then open the RoverRally/test sketch from Examples 
     (you will also need to choose USE_BNO or USE_MPU here).

You will need to choose an IMU (the gyro -- is what the RoverRally library uses from the IMU, to get heading).  Be sure to define one of USE_BNO or USE_MPU in the RoverRally.h file. 
