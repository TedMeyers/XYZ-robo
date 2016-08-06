/* Easy Rover Code
 by Ted Meyers (8/4/2016)
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

 * Set the encoder type in XYZ_WheelEncoder.h (SINGLE or QUADRATURE)
   Is SINGLE BY DEFAULT
*/

// ------------------------------------------------------------
// THE FOLLOWING VALUES MAY NEED TO BE ADJUSTED FOR YOUR ROVER!
// ------------------------------------------------------------

// These are the arduino digital I/O pins used:
// Wheel Encoder - 6,7 (0-7 required if using PORT D on EnableInterrupts library)
//                      Note: Just use pin 6 if using a single encoder
// Steering      - 10  (9 or 10 required if using TicoServo library)
// Throttle      -  9  (9 or 10 required if using TicoServo library)
// Button        -  8  (Can be any digital IO pin)
// LED           - 13  (13 is the on board led on arduino)

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
#define SPEED 30

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
//#define SERIAL_WAIT
// ---------------------------------------------------------------

#include <I2Cdev.h>
#include <XYZ_EasyRover.h>
#include <XYZ_WheelEncoder.h>
#include <XYZ_Interrupts.h>


// Global Objects...
XYZ_EasyRover myRover;          // The rover object (controls the rover)


void setup(void) { 
  setupI2C();    // For IMU
  setupSerial(); // For debugging
  setupRover();  // General rover setup
  setupIMU();    // IMU initialization
  setupRoverActions(); // Defines the drive path
}

void setupI2C(void) {
  //
  // Setup I2C (IMU/gyro interface)
  //
  Wire.begin();
  TWBR = 12;    // set 400kHz mode @ 16MHz CPU or 200kHz mode @ 8MHz CPU
  delay(1000);  // Delay 1 second to give wire time to begin
}
  
void setupSerial() {
  //
  // Setup Serial (for debugging)
  //  
  #ifdef SERIAL_OUT
    Serial.begin(115200);  // Setup serial connection

    #ifdef SERIAL_WAIT     // For testing when using a Leonardo, waits for serial connection
      while (!Serial);     // (DO NOT USE WHEN NOT TESTING -- will lock up here (on leonardo)!)
    #endif            
    Serial.println(F("XYZ_EasyRover Test begin..."));
  #endif
}

void setupRover() {
  //
  // Setup Rover servos, encoders, button, LED
  //

  // First set the various input/output pins... 
  //myRover.getWheelEncoder()->setup(WE_A_PIN, WE_B_PIN); 
  myRover.getWheelEncoder()->setup(WE_A_PIN);
  myRover.getButton()->setup(BTN_PIN);
  myRover.getLED()->setup(LED_PIN);
  myRover.getServos()->setup(THR_OUT_PIN, STR_OUT_PIN);

  // Set the center values...
  myRover.setSteeringCenterValue(STEERING_CENTER);
  myRover.setThrottleCenterValue(THROTTLE_CENTER);

  // Set turn to closest (or turn to left or turn to right)
  myRover.setClosestTurnMode();

  // Set zero steering and throttle
  myRover.setSteeringAngleDeg(0);
  myRover.setThrottlePercent(0);
}

void setupIMU() {
  //
  // Setup Rover IMU
  //
  int test = myRover.setupI2C_IMU(IMU_ADDR);
  if (test == 0) {
    while (true) {
      #ifdef SERIAL_OUT
        Serial.println(F("No IMU found"));
      #endif

      // Blink for error....
      myRover.getLED()->setOn();
      delay(500);
      myRover.getLED()->setOff();
      delay(500);
    }   
  } else {
    #ifdef SERIAL_OUT
      Serial.print("IMU status: ");
      Serial.println(test);
    #endif     
  }
}

void setupRoverActions() {
  // --------------------------------------------------------
  //
  // Set Rover Actions Here!
  //
  // Drive in a square (approx 22 feet), making left turns (CCW)
  //
  myRover.resetActions();
  myRover.addAction(600, 0.0);
  myRover.addAction(700, 270.0);
  myRover.addAction(700, 180.0);
  myRover.addAction(700, 90.0);
  // --------------------------------------------------------
}

//
// The main loop
//
void loop(void) 
{
  myRover.updateAll();  // Always call update!

  if (!myRover.getActionsEnabled()) {  // If actions are currently NOT running...
    myRover.setSteeringAngleDeg(0);    // Center steering
    myRover.setThrottlePercent(0);     // Disable throttle

    if (myRover.getButton()->waitForPress(1)) {   // Wait foruup to 1 msec for a button press
      #ifdef SERIAL_OUT
        Serial.println("--- RUNNING! ----");
      #endif
      myRover.restartActions();                   // If button pressed, restart actions and
      myRover.setThrottlePercent(SPEED);          // Enable throttle
    }                                             // Otherwise, just try again...
  }
}

