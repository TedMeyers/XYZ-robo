/* BNO/MPU Rover Code
 by Ted Meyers (5/19/2015)
 https://github.com/TedMeyers/XYZ-robo
 
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.

 IMPORTANT: Define USE_MPU or USE_BNO below!
            and also in RoverRally.h!
*/
// ----------------------------------------------------------
// THE FOLLOWING VALUES MAY NEED TO BE ADJUSTED FOR YOUR BOT!
// ----------------------------------------------------------
// Only define one of these...
// ----------------------------
#define USE_MPU
//#define USE_BNO
// ----------------------------

// Car throttle speed
#define SPEED 30.0

// Wheel encoder distance (your value my be a lot different)
#define TICKS_PER_FOOT 20

// Throttle and steering values:
#define STEER_CENTER 89
#define THROTTLE_CENTER 1502
#define THROTTLE_MIN_FWD 73
#define THROTTLE_MIN_REV 185

// These are the arduino digital I/O pins used:
// Throttle      - 4
// Steering      - 5
// Button        - 7
// Wheel Encoder - 11 - (This must match port - take care if changing)
//                      (See PinChangeInterrupt Library)
// LED           - 13
#define THR_PIN 4
#define STR_PIN 5
#define BTN_PIN 10
#define WE_PIN  11
#define LED_PIN 13
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
// PinChangeInterrupt Library
// Make sure the ports match the pins used!
// ----------------------------
#define NO_PORTC_PINCHANGES
#define NO_PORTD_PINCHANGES
#define NO_PIN_STATE
#define NO_PIN_NUMBER
#define DISABLE_PCINT_MULTI_SERVICE
#include <PinChangeInt.h>

// ----------------------------
// IMU headers
// ----------------------------
#ifdef USE_BNO
  #include "XYZ_BNO055.h"
  #define IMU_ADDR BNO055_ADDRESS_A
#endif
#ifdef USE_MPU
  #define MPU6050_INCLUDE_DMP_MOTIONAPPS20
  #include "helper_3dmath.h"
  #include "XMPU6050_6Axis_MotionApps20.h"
  #include "XMPU6050.h"
  #include "XYZ_MPU6050.h"
  #define IMU_ADDR MPU6050_ADDRESS_A
#endif
// ----------------------------

#include "RoverRally.h"

// Global Objects...
RoverRally myRover;              // The rover object (controls the rover)
volatile uint32_t _wheel_encoder_counter;    // The number of wheel ticks

void setup(void) { 
  //
  // Setup Wheel Encoder interrupt
  //
  pinMode(WE_PIN, INPUT_PULLUP);
  PCintPort::attachInterrupt(WE_PIN, wheel_encoder_tick, CHANGE);

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
    Serial.println(F("RoverRally Test begin..."));
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
    Serial.println(F("RoverRally Test Ready..."));
  #endif
  reset();
}

void loop(void) 
{
  // Don't start until button is pressed
  myRover.waitForButtonPress();
  
  #ifdef SERIAL_OUT
    Serial.println(F("RoverRally Test Go!"));
  #endif
  
  reset();
  myRover.setSlowThrottlePercent(0.60*SPEED);
  myRover.setThrottlePercent(SPEED);
  drive();
}

// D_1, D_2 are distances in feet
// Speed is in throttle percent
#define D_1 10.0
#define D_2 5.0

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
  uint32_t ticks = update_wheel_encoder();
  myRover.updateAllBasic();
  
  if ((millis() - updateTime) > 200) {
    updateTime = millis();
    myRover.updateCalibrationLED();
    uint8_t stats[4];
    myRover.readCalibrationStats(stats);

    #ifdef SERIAL_OUT
      #ifdef PRINT_UPDATES
        Serial.print(updateTime);
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

void reset() {
  set_wheel_encoder(0);
  myRover.setTotalTicks(0);
  myRover.reset();
}

// Update ticks in the rover
uint32_t update_wheel_encoder() {
  uint32_t ticks = get_wheel_encoder();
  myRover.setTotalTicks(ticks);
  return ticks;
}

// Safely set the wheel encoder tick value
void set_wheel_encoder(uint32_t ticks) {
  noInterrupts();
  _wheel_encoder_counter = ticks;
  interrupts();  
  myRover.setTotalTicks(ticks);
}

// Get the wheel encoder tick value safely
uint32_t get_wheel_encoder() {
  uint32_t ticks;
  noInterrupts();
  ticks = _wheel_encoder_counter;
  interrupts();
  return ticks;
}

// Encoder pin change callback
// Gets called when the wheel encoder pin changes
void wheel_encoder_tick() {
  _wheel_encoder_counter++;
}
