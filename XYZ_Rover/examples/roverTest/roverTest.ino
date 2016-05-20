/* BNO/MPU Rover Code
 by Ted Meyers (4/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.

 This sketch runs a short rover test.

 * You will want to verify IMU type in XYZ_Rover.h, and you
   may need to set the correct address for your IMU.

 * Also in XYZ_Rover.h, set to desired: 
    USE_RX_MUX, USE_RX_AUX_DISABLE, USE_RX_SIGNAL_DISABLE

 * And in XYZ_Rover.h, set the TUNING PARAMETERS.
*/
// ------------------------------------------------------------
// THE FOLLOWING VALUES MAY NEED TO BE ADJUSTED FOR YOUR ROVER!
// ------------------------------------------------------------
// Only set if using the BNO IMU
// IMPORTANT (REQUIRED): Change IMU type in xYZ_Rover.h ALSO!
// #define USE_BNO
// ----------------------------

// These are the arduino digital I/O pins used:
// Wheel Encoder - 6,7 (0-7 required if using PORT D on EnableInterrupts library)
// Steering      - 10  (9 or 10 required if using TicoServo library)
// Throttle      -  9  (9 or 10 required if using TicoServo library)
// Button        -  8  (Can be any digital IO pin)
// LED           - 13  (13 is the on board led on arduino)

#define THR_IN_PIN  3
#define STR_IN_PIN  4
#define AUX_IN_PIN  5
#define WE_A_PIN    6
#define WE_B_PIN    7
#define THR_OUT_PIN 9
#define STR_OUT_PIN 10
#define BTN_PIN     8
#define LED_PIN     13

// Wheel encoder distance (your value my be a lot different)
// This is a float type value
#define TICKS_PER_FOOT 27.1

#define STEERING_CENTER -5
#define THROTTLE_CENTER 0

// Car throttle speed
#define SPEED 30.0

// This displays calibration status via the LED.
#define UPDATE_CALIBRATION

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

#include <I2Cdev.h>
#include <XYZ_Rover.h>
#include <XYZ_WheelEncoder.h>
#include <XYZ_Interrupts.h>


// Global Objects...
XYZ_Rover myRover;              // The rover object (controls the rover)
uint32_t myStartTime = 0;       // Startup time (last reset)


void setup(void) { 
  //
  // Setup I2C (IMU/gyro interface)
  //
  Wire.begin();
  TWBR = 12;  // set 400kHz mode @ 16MHz CPU or 200kHz mode @ 8MHz CPU
  delay(1000);
  
  //
  // Setup Serial (for debugging)
  //  
  #ifdef SERIAL_OUT
    Serial.begin(115200);  // Setup serial connection
    #ifdef SERIAL_WAIT
      while (!Serial);     // For testing when using a Leonardo, waits for serial connection
    #endif                 // (DO NOT USE WHEN NOT TESTING -- will lock up here (on leonardo)!)
    Serial.println(F("XYZ_Rover Test begin..."));
  #endif

  //
  // Setup Rover
  //
  myRover.getServos()->setup(THR_OUT_PIN, STR_OUT_PIN);
  myRover.getRx()->setup(THR_IN_PIN, STR_IN_PIN, AUX_IN_PIN); 
  myRover.getWheelEncoder()->setup(WE_A_PIN, WE_B_PIN); 
  myRover.getButton()->setup(BTN_PIN);
  myRover.getLED()->setup(LED_PIN);

  myRover.setUpdateCB(&updateCallback);
  myRover.setTickToFeetConversion(TICKS_PER_FOOT);
  myRover.setSteeringCenterAngle(STEERING_CENTER);
  myRover.setThrottleCenterPercent(THROTTLE_CENTER);

  int test = myRover.setupI2C_IMU(IMU_ADDR);
  if (test == 0) {
    while (true) {
      #ifdef SERIAL_OUT
        Serial.println(F("No IMU found"));
      #endif
      myRover.getLED()->setOn();
      delay(500);
      myRover.getLED()->setOff();
      delay(500);
    }   
  }

  #ifdef SERIAL_OUT
    Serial.print("IMU status: ");
    Serial.println(test);
    Serial.println(F("XYZ_Rover Test Ready..."));
  #endif

  resetAll();
}

void loop(void) 
{
  myStartTime = millis();
  
  // Don't start until button is pressed
  myRover.waitForButtonPress();
  myStartTime = millis();
  
  #ifdef SERIAL_OUT
    Serial.println(F("XYZ_Rover Test Go!"));
  #endif
  
  myRover.resetHeadings();
  myRover.setRoverModeHold();
  myRover.waitForTimeMs(200);
  myRover.setRoverModeRun();

  drive();
  
  myRover.setRoverModeHold();
}

// D_1, D_2 are distances in feet
// Speed is in throttle percent
#define D_1 20.0
#define D_2 10.0

// Drive a rectangle
void drive() {
  myRover.waitForDistanceFt(2*D_1); 
  
  myRover.turnToLeftDeg(90);
  myRover.waitForDistanceFt(D_2);
  
  myRover.turnToLeftDeg(90);
  myRover.waitForDistanceFt(3*D_1);
  
  myRover.turnToLeftDeg(90);
  myRover.waitForDistanceFt(D_2);
  
  myRover.turnToLeftDeg(90);
  myRover.waitForDistanceFt(D_1-10.0);  

  myRover.brakeStop(200);
}

// Update function called by the rover -- does all the updating work when called
void updateCallback(int state) {
  static uint32_t updateTime = 0;   // Time of last update
  myRover.updateAllBasic();

  if ((millis() - updateTime) > 200) {
    updateTime = millis();

    #ifdef UPDATE_CALIBRATION
      myRover.updateCalibrationLED();
    #endif

    #ifdef SERIAL_OUT
      #ifdef PRINT_UPDATES
        uint8_t mode = myRover.getRoverMode();
        int32_t ticks = myRover.getTotalTicks();
        int tdir = myRover.getServos()->getThrottleDirection();
        Serial.print(updateTime-myStartTime);
        Serial.print(F("s: "));
        Serial.print(state);
        Serial.print(F(" m: "));
        Serial.print(mode);
        Serial.print(F(" "));
        Serial.print(tdir);
        Serial.print(F(" "));
        Serial.print(ticks);
        Serial.print(F(" "));
        Serial.print(myRover.getMarkedTick());
        Serial.print(F(" "));
        Serial.print(myRover.getCurrentHeadingDeg());
        #ifdef UPDATE_CALIBRATION
          #ifdef USE_BNO
            uint8_t stats[4];
            myRover.readCalibrationStats(stats);
            Serial.print(F(" "));
            Serial.print(stats[0]);
            Serial.print(stats[1]);
            Serial.print(stats[2]);
            Serial.print(stats[3]);
          #endif
        #endif
        Serial.println(";");
      #endif
    #endif
  }
}

// Reset the rover, this is mostly about the wheel encoder ticks
void resetAll() {
  myRover.resetTotalTicks();
  myRover.reset();
  delay(100);
  myStartTime = millis();
}

