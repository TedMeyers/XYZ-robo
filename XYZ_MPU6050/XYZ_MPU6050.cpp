/* XYZ_MPU6050 Library
 by Ted Meyers  (5/19/2015)
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
*/
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#define MPU6050_INCLUDE_DMP_MOTIONAPPS20

#include "helper_3dmath.h"
#include "XMPU6050.h"
#include "XYZ_MPU6050.h"


/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
XYZ_MPU6050::XYZ_MPU6050()
{
}

void XYZ_MPU6050::setupI2C() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
}

uint8_t XYZ_MPU6050::setup(uint8_t address) {
  _address = address;
  _mpu.initialize(_address);                  // initialize device

  int testConnect = _mpu.testConnection();    // verify connection
  if (!testConnect)  {
    return 127;
  }    
  uint8_t devStatus = _mpu.dmpInitialize();           // load and configure the DMP (0 = success, > 0 = error)

  if (devStatus == 0) {                               // make sure it worked (returns 0 if so)
    _mpu.setDMPEnabled(true);                         // turn on the DMP, now that it's ready
    _packetSize = _mpu.dmpGetFIFOPacketSize();        // get expected DMP packet size
    uint8_t mpuIntStatus = _mpu.getIntStatus();       // get INT_STATUS byte
    uint16_t fifoCount = _mpu.getFIFOCount();         // get current FIFO count
  }
  return devStatus;
}

void XYZ_MPU6050::setGyroOffsets(int16_t x, int16_t y, int16_t z) {
  _mpu.setXGyroOffset(x);
  _mpu.setYGyroOffset(y);
  _mpu.setZGyroOffset(z);	
}

void XYZ_MPU6050::setAccelOffsets(int16_t x, int16_t y, int16_t z) {
	_mpu.setXAccelOffset(x); 
  _mpu.setYAccelOffset(y); 
  _mpu.setZAccelOffset(z); 
}

float *XYZ_MPU6050::updateYPR() {
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorFloat gravity;    // [x, y, z]            gravity vector
  
  uint8_t mpuIntStatus = _mpu.getIntStatus();  //  get INT_STATUS byte from MPU
  uint16_t fifoCount = _mpu.getFIFOCount();    // get count of all bytes currently in FIFO

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 64) {
    _mpu.resetFIFO();  // reset so we can continue cleanly
      
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < _packetSize) fifoCount = _mpu.getFIFOCount();
      _mpu.getFIFOBytes(_fifoBuffer, _packetSize);  // read a packet from FIFO
      
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= _packetSize;

      // display Euler angles in degrees
      _mpu.dmpGetQuaternion(&q, _fifoBuffer);
      _mpu.dmpGetGravity(&gravity, &q);
      _mpu.dmpGetYawPitchRoll(_ypr, &q, &gravity);
   }
   return _ypr;
}

float XYZ_MPU6050::getYaw() {
  return normalizeDeg360(_ypr[0]*RAD_TO_DEG);
}
float XYZ_MPU6050::getPitch() {
  return normalizeDeg360(_ypr[1]*RAD_TO_DEG);
}
float XYZ_MPU6050::getRoll() {
  return normalizeDeg360(_ypr[2]*RAD_TO_DEG);
}

float XYZ_MPU6050::setHeadingOffset() {
  updateYPR();
  _hdgOffset = getYaw();
  return _hdgOffset;
}
float XYZ_MPU6050::getHeading() {
  return normalizeDeg360(getYaw() - _hdgOffset);
}


float XYZ_MPU6050::angleDegDiff(float headingA, float headingB) {
  headingA = normalizeDeg360(headingA);
  headingB = normalizeDeg360(headingB);
  float diff = normalizeDeg180(headingA - headingB);
  return diff;
}
float XYZ_MPU6050::normalizeDeg360(float heading) {
  if (heading < 0) heading += 360;
  if (heading < 0) heading += 360;
  if (heading > 360) heading -= 360;
  if (heading > 360) heading -= 360;
  return heading;
}
float XYZ_MPU6050::normalizeDeg180(float heading) {
  if (heading < -180) heading += 360;
  if (heading < -180) heading += 360;
  if (heading >  180) heading -= 360;
  if (heading >  180) heading -= 360;
  return heading;
}


int XYZ_MPU6050::angleDegDiffI(int headingA, int headingB) {
  headingA = normalizeDeg360I(headingA);
  headingB = normalizeDeg360I(headingB);
  int diff = normalizeDeg180I(headingA - headingB);
  return diff;
}
int XYZ_MPU6050::normalizeDeg360I(int heading) {
  if (heading < 0) heading += 360;
  if (heading < 0) heading += 360;
  if (heading > 360) heading -= 360;
  if (heading > 360) heading -= 360;
  return heading;
}
int XYZ_MPU6050::normalizeDeg180I(int heading) {
  if (heading < -180) heading += 360;
  if (heading < -180) heading += 360;
  if (heading >  180) heading -= 360;
  if (heading >  180) heading -= 360;
  return heading;
}