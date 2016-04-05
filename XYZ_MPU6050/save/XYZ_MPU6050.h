/* XYZ_MPU6050 Library
 by Ted Meyers  (5/19/2015)
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
*/
#ifndef __XYZ_MPU6050_H__
#define __XYZ_MPU6050_H__
#define MPU6050_INCLUDE_DMP_MOTIONAPPS20

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#define RAD_TO_DEG (57.295779513)

// class default I2C address is 0x68
#define MPU6050_ADDRESS_A (0x68)
#define MPU6050_ADDRESS_B (0x69)

class XYZ_MPU6050
{
  public:
  	XYZ_MPU6050();
  	void setupI2C();
  	uint8_t setup(uint8_t address=MPU6050_ADDRESS_A);
  	void setGyroOffsets(int, int, int);
  	void setAccelOffsets(int, int, int);
  	float *updateYPR();

    float getYaw();
    float getPitch();
    float getRoll();
    
    float setHeadingOffset();
    float getHeading();

  	float normalizeDeg360(float);
    float normalizeDeg180(float);
    float angleDegDiff(float, float);
    
    int normalizeDeg360I(int);
    int normalizeDeg180I(int);
    int angleDegDiffI(int, int);

  private:
  	XMPU6050 _mpu;
   //  uint8_t _address;
   //  uint16_t _packetSize;     // expected DMP packet size (default is 42 bytes)
   //  uint8_t _fifoBuffer[42];  // FIFO storage buffer for packetSize bytes
	  // float _ypr[3];            // [yaw, pitch, roll]   yaw/pitch/roll
   //  float _hdgOffset;         // Heading offset
};

#endif