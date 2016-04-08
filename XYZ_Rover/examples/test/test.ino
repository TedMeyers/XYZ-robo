/* BNO/MPU Rover Code
 by Ted Meyers (4/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.

 IMPORTANT: Define USE_MPU or USE_BNO below!
            and also in XYZ_Rover.h!
*/
// ----------------------------------------------------------
// THE FOLLOWING VALUES MAY NEED TO BE ADJUSTED FOR YOUR BOT!
// ----------------------------------------------------------
// Only define one of these... 
// IMPORTANT (REQUIRED): Change in xYZ_Rover.h ALSO!
// ----------------------------
#define IMU_TYPE 1
#if (IMU_TYPE == 0)
  #define USE_MPU
#else
  #define USE_BNO
#endif
// ----------------------------

// ----------------------------
// Quadrature vs Single Wheel Encoder
#define ENC_TYPE 1
#if (ENC_TYPE == 0)
  #define USE_SINGLE_ENC
#else
  #define USE_QUADRATURE_ENC
#endif
// ----------------------------

// These are the arduino digital I/O pins used:
// Wheel Encoder - 6,7 (0-7 required if using PORT D on EnableInterrupts library)
// Steering      -  9  (9 or 10 required if using TicoServo library)
// Throttle      - 10  (9 or 10 required if using TicoServo library)
// Button        - 11  (Can be any digital IO pin)
// LED           - 13  (13 is the on board led on arduino)
#define WE_A_PIN 6
#define WE_B_PIN 7
#define STR_PIN  9
#define THR_PIN 10
#define BTN_PIN 11
#define LED_PIN 13

// Wheel encoder distance (your value my be a lot different)
// This is a float type value
#define TICKS_PER_FOOT 27.1

// Car throttle speed
#define SPEED 30.0

// Throttle and steering values:
#define STEER_CENTER 89
#define THROTTLE_CENTER 1502
#define THROTTLE_MIN_FWD 73
#define THROTTLE_MIN_REV 185
// ---------------------------------------------------------------
//
// Serial out defines: uncomment to test
// It is a good idea to NOT print serial unless needed
// But, it can also be very helpful when troubleshooting
//
// Define SERIAL_WAIT when using a Leonardo,  to force
// the Leonardo to wait for a serial connection before
// continuing on.
//
#define SERIAL_OUT
#define PRINT_UPDATES
//#define SERIAL_WAIT
// ---------------------------------------------------------------

// ----------------------------
// I2C for IMU communications...
// ----------------------------
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include "I2Cdev.h"
#include <Servo.h> 

// ----------------------------
// Wheel Encoder Library
// ----------------------------
#ifdef USE_SINGLE_ENC
  #include "XYZ_SingleEncoder.h"
  XYZ_SingleEncoder myEnc(WE_A_PIN);                 // Pin 8 on portB
#else
  #include "XYZ_QuadratureEncoder.h"
  XYZ_QuadratureEncoder myEnc(WE_A_PIN, WE_B_PIN);  // Uses DIO pins 8 and 9 (on portB)
 #endif

// ----------------------------
// IMU headers
// ----------------------------
#ifdef USE_BNO
  #include "XYZ_BNO055.h"
  #define IMU_ADDR BNO055_ADDRESS_A
#endif
#ifdef USE_MPU
  #include "XYZ_MPU6050.h"
  #define IMU_ADDR MPU6050_ADDRESS_A
#endif
// ----------------------------

#include "XYZ_Rover.h"

// Global Objects...
XYZ_Rover myRover;              // The rover object (controls the rover)
uint32_t myStartTime = 0;        // Startup time (last reset)


void setup(void) { 
  //
  // Setup I2C (IMU/gyro interface)
  //
  Wire.begin();
  TWBR = 12;  // set 400kHz mode @ 16MHz CPU or 200kHz mode @ 8MHz CPU
  delay(2000);
  
  //
  // Setup Serial (for debugging)
  //  
  #ifdef SERIAL_OUT
    Serial.begin(115200);  // Setup serial connection
    #ifdef SERIAL_WAIT
      while (!Serial);     // For testing when using a Leonardo, waits for serial connection
                           // (DO NOT USE WHEN RUNNING -- will lock up here!)
    #endif
    Serial.println(F("XYZ_Rover Test begin..."));
  #endif

  //
  // Setup Rover
  //
  myRover.setSlowThrottlePercent(10.0);
  myRover.setThrottlePercent(20.0);  
  myRover.setUpdateCB(&updateCallback);
  myRover.setTickToDistanceConversion(TICKS_PER_FOOT);
  myRover.setSteeringValues(STEER_CENTER);
  myRover.setThrottleValues(THROTTLE_CENTER, THROTTLE_MIN_FWD, THROTTLE_MIN_REV);
  int test = myRover.setupRover(THR_PIN, STR_PIN, BTN_PIN, LED_PIN, IMU_ADDR);
  if (test == 0) {
    while (true) {
      #ifdef SERIAL_OUT
        Serial.println(F("No IMU found"));
      #endif
      digitalWrite(LED_PIN, HIGH);
      delay(500);
      digitalWrite(LED_PIN, LOW);
      delay(500);
    }   
  }
  #ifdef SERIAL_OUT
    Serial.println(F("XYZ_Rover Test Ready..."));
  #endif
  resetAll();
}

void loop(void) 
{
  // Don't start until button is pressed
  myRover.waitForButtonPress();
  
  #ifdef SERIAL_OUT
    Serial.println(F("XYZ_Rover Test Go!"));
  #endif
  
  resetAll();
  myRover.setSlowThrottlePercent(0.60*SPEED);
  myRover.setThrottlePercent(SPEED);
  drive();
}

// D_1, D_2 are distances in feet
// Speed is in throttle percent
#define D_1 20.0
#define D_2 10.0

// Drive a rectangle
void drive() {
  myRover.setModeRun();
  myRover.waitForDistance(D_1); 
  
  myRover.turnToLeft(90);
  myRover.waitForDistance(D_2);
  
  myRover.turnToLeft(90);
  myRover.waitForDistance(2*D_1);
  
  myRover.turnToLeft(90);
  myRover.waitForTicks(D_2);
  
  myRover.turnToLeft(90);
  myRover.waitForDistance(D_1);
  
  myRover.brakeStop(200);
  myRover.setModeWait();
}

// Update function -- does all the updating work when called
void updateCallback(int state) {
  static uint32_t updateTime = 0;   // Time of last update
  int32_t ticks = update_wheel_encoder();
  myRover.updateAllBasic();
  
  if ((millis() - updateTime) > 200) {
    updateTime = millis();
    myRover.updateCalibrationLED();
    uint8_t stats[4];
    myRover.readCalibrationStats(stats);

    #ifdef SERIAL_OUT
      #ifdef PRINT_UPDATES
        Serial.print(updateTime-myStartTime);
        Serial.print(F(" "));
        Serial.print(ticks);
        Serial.print(F(" "));
        Serial.print(myRover.getMarkedTick());
        #ifdef USE_BNO
          Serial.print(F(" "));
          Serial.print(stats[0]);
          Serial.print(stats[1]);
          Serial.print(stats[2]);
          Serial.print(stats[3]);
        #endif
        Serial.print(F(" "));
        Serial.println(myRover.getCurrentHeading());
      #endif
    #endif
  }
}

// Reset the rover, this is mostly about the wheel encoder ticks
void resetAll() {
  set_wheel_encoder(0);
  myRover.setTotalTicks(0);
  myRover.reset();
  myEnc.reset();
  myStartTime = millis();
}

// Update ticks in the rover
int32_t update_wheel_encoder() {
  int32_t ticks = myEnc.get_encoder_count();
  myRover.setTotalTicks(ticks);
  return ticks;
}

// Safely set the wheel encoder tick value
void set_wheel_encoder(int32_t ticks) {
  myEnc.set_encoder(ticks);
  myRover.setTotalTicks(ticks);
}

// Get the wheel encoder tick value safely
int32_t get_wheel_encoder() {
  int32_t ticks = myEnc.get_encoder_count();
  return ticks;
}
