
/* BNO-055 library
 by Ted Meyers (4/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 This code is loosely based on the following (https://github.com/kriswiner/BNO-055):
 BNO055_MS5637_t3 Basic Example Code
 by: Kris Winer
 date: October 19, 2014

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.
*/
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include "Wire.h"
#include "XYZ_BNO055.h"

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
XYZ_BNO055::XYZ_BNO055()
{
  _address = BNO055_ADDRESS_A;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/
void XYZ_BNO055::setupI2C() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      TWBR = 12; // set 400kHz mode @ 16MHz CPU or 200kHz mode @ 8MHz CPU
      delay(2000);
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
}

bool XYZ_BNO055::setup(uint8_t address) {
  _address = address; 
  float gyroBias[3] = {0, 0, 0};   // Bias corrections for gyro
  float accelBias[3] = {0, 0, 0};  // Bias corrections for accelerometer 
  float magBias[3] = {0, 0, 0};    // Bias corrections for magnetometer
  
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = readByte(_address, BNO055_CHIP_ID);  // Read WHO_AM_I register for BNO055   
  byte d = readByte(_address, BNO055_ACC_ID);   // Read WHO_AM_I register for accelerometer
  byte e = readByte(_address, BNO055_MAG_ID);   // Read WHO_AM_I register for magnetometer
  byte f = readByte(_address, BNO055_GYRO_ID);  // Read WHO_AM_I register for LIS3MDL

  byte swlsb = readByte(_address, BNO055_SW_REV_ID_LSB);
  byte swmsb = readByte(_address, BNO055_SW_REV_ID_MSB);
  byte blid = readByte(_address, BNO055_BL_REV_ID);
  byte selftest = readByte(_address, BNO055_ST_RESULT);     // Check self-test results   
  init();        // Initialize the BNO055

  if (c != 0xA0) { // BNO055 WHO_AM_I should always be 0xA0
    return false;
  }
  return true;
}

void XYZ_BNO055::init() {
   uint8_t GPwrMode = NormalG;     // Gyro power mode
   uint8_t Gscale = GFS_250DPS;    // Gyro full scale
   uint8_t Gbw = GBW_23Hz;         // Gyro bandwidth
   uint8_t Ascale = AFS_2G;        // Accel full scale
   uint8_t APwrMode = NormalA;     // Accel power mode
   uint8_t Abw = ABW_31_25Hz;      // Accel bandwidth, accel sample rate divided by ABW_divx
   uint8_t MOpMode = HighAccuracy; // Select magnetometer perfomance mode
   uint8_t MPwrMode = Normal;      // Select magnetometer power mode
   uint8_t Modr = MODR_10Hz;       // Select magnetometer ODR when in BNO055 bypass mode
   uint8_t PWRMode = Normal;       // Select BNO055 power mode
   uint8_t OPRMode = IMU;         // specify operation mode for sensors

   // Select BNO055 config mode
   writeByte(_address, BNO055_OPR_MODE, CONFIGMODE);
   delay(25);
   writeByte(_address, BNO055_PAGE_ID, 0x01);                                   // Select page 1 to configure sensors
   writeByte(_address, BNO055_ACC_CONFIG, APwrMode << 5 | Abw << 2 | Ascale);   // Configure ACC
   writeByte(_address, BNO055_GYRO_CONFIG_0, Gbw << 3 | Gscale);                // Configure GYR
   writeByte(_address, BNO055_GYRO_CONFIG_1, GPwrMode);   
   writeByte(_address, BNO055_MAG_CONFIG, MPwrMode << 5 | MOpMode << 3 | Modr); // Configure MAG
   
   writeByte(_address, BNO055_PAGE_ID, 0x00);      // Select page 0 to read sensors
   writeByte(_address, BNO055_TEMP_SOURCE, 0x01);  // Select BNO055 gyro temperature source 
   writeByte(_address, BNO055_UNIT_SEL, 0x01);     // Select BNO055 sensor units (temperature in degrees C, rate in dps, accel in mg)
   writeByte(_address, BNO055_PWR_MODE, PWRMode);  // Select BNO055 system power mode
   writeByte(_address, BNO055_OPR_MODE, OPRMode);  // Select BNO055 system operation mode
}

void XYZ_BNO055::setMode(uint8_t mode) {
  writeByte(_address, BNO055_OPR_MODE, mode);
  delay(30);
}

uint8_t XYZ_BNO055::readCalibration(uint8_t *calstats) {
  // Check calibration status of the sensors
  uint8_t calstat = readByte(_address, BNO055_CALIB_STAT);
  calstats[0] = (0xC0 & calstat) >> 6;
  calstats[1] = (0x30 & calstat) >> 4;
  calstats[2] = (0x0C & calstat) >> 2;
  calstats[3] = (0x03 & calstat) >> 0;
  return calstat;
}

uint8_t XYZ_BNO055::readSysErr() {
  uint8_t syserr = readByte(_address, BNO055_SYS_ERR);
  return syserr;
}

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================
float XYZ_BNO055::readHeading() {
  return readYPRValue(0);
}
float XYZ_BNO055::readPitch() {
  return readYPRValue(1);
}
float XYZ_BNO055::readRoll() {
  return readYPRValue(2);
}
float XYZ_BNO055::readYPRValue(uint8_t i) {
  int16_t EulCount[3];    // Stores the 16-bit signed Euler angle output
  readEulData(EulCount);  // Read the x/y/z adc values 
  return (float)EulCount[i]/16.;
}
void XYZ_BNO055::readYPR(float *ypr) {
  int16_t EulCount[3];    // Stores the 16-bit signed Euler angle output
  readEulData(EulCount);  // Read the x/y/z adc values 
  
  // Calculate the Euler angles values in degrees
  ypr[0] = (float)EulCount[0]/16.;  
  ypr[2] = (float)EulCount[1]/16.;  
  ypr[1] = (float)EulCount[2]/16.;   
}


void XYZ_BNO055::readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(_address, BNO055_ACC_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;   // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}


void XYZ_BNO055::readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(_address, BNO055_GYR_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;   // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}

int8_t XYZ_BNO055::readGyroTempData()
{
  return readByte(_address, BNO055_TEMP);  // Read the two raw data registers sequentially into data array 
}

void XYZ_BNO055::readMagData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(_address, BNO055_MAG_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;   // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void XYZ_BNO055::readQuatData(int16_t * destination)
{
  uint8_t rawData[8];  // x/y/z gyro register data stored here
  readBytes(_address, BNO055_QUA_DATA_W_LSB, 8, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;   // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
  destination[3] = ((int16_t)rawData[7] << 8) | rawData[6] ;
}

void XYZ_BNO055::readEulData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
 readBytes(_address, BNO055_EUL_HEADING_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;   // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void XYZ_BNO055::readLIAData(int16_t * destination)
{
  uint8_t rawData[6];                                          // x/y/z gyro register data stored here
  readBytes(_address, BNO055_LIA_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;   // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void XYZ_BNO055::readGRVData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(_address, BNO055_GRV_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;   // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

// I2C read/write functions for the BNO055 sensor
void XYZ_BNO055::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  I2Cdev::writeByte(address, subAddress, data);

	// Wire.beginTransmission(address);  // Initialize the Tx buffer
	// Wire.write(subAddress);           // Put slave register address in Tx buffer
	// Wire.write(data);                 // Put data in Tx buffer
	// Wire.endTransmission();           // Send the Tx buffer
}

uint8_t XYZ_BNO055::readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data;
  I2Cdev::readByte(address, subAddress, &data);
  return data;

	// uint8_t data; // `data` will store the register data	 
	// Wire.beginTransmission(address);         // Initialize the Tx buffer
	// Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	// Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
	// Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
	// data = Wire.read();                      // Fill Rx buffer with result
	// return data;                            // Return data read from slave register
}

void XYZ_BNO055::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest)
{ 
  I2Cdev::readBytes(address, subAddress, count, dest);

	// Wire.beginTransmission(address);   // Initialize the Tx buffer
	// Wire.write(subAddress);            // Put slave register address in Tx buffer
	// Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
	// uint8_t i = 0;
	//  Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
	// while (Wire.available()) {
	//    dest[i++] = Wire.read(); 
	//  }         // Put read results in the Rx buffer
}
