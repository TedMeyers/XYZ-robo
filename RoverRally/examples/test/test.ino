/* BNO/MPU Rover Code
 by Ted Meyers (5/19/2015)
 https://github.com/TedMeyers/XYZ-robo
 
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.

 IMPORTANT: Define USE_MPU or USE_BNO below!
            and also in RoverRally.h !
*/
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include "I2Cdev.h"

#include <Servo.h> 

// ----------------------------
// Only define one of these...
#define USE_MPU
//#define USE_BNO
// ----------------------------

#ifdef USE_BNO
  #include "XYZ_BNO055.h"
  #define IMU_ADDR BNO055_ADDRESS_A
#endif
#ifdef USE_MPU
  #include "helper_3dmath.h"
  #include "XMPU6050_6Axis_MotionApps20.h"
  #include "XMPU6050.h"
  #include "XYZ_MPU6050.h"
  #define IMU_ADDR MPU6050_ADDRESS_A
#endif
#include "RoverRally.h"

//#define PRINT_UPDATES

// These are the arduino digital I/O pins used:
#define THR_PIN 4
#define STR_PIN 5
#define BTN_PIN 7
#define LED_PIN 13

// Throttle and steering values:
#define STEER_CENTER 89
#define THROTTLE_CENTER 1502
#define THROTTLE_MIN_FWD 73
#define THROTTLE_MIN_REV 185

// Wheel encoder distance (your value my be a lot different)
#define TICKS_PER_FOOT 75


RoverRally myRover;
uint32_t myUpdateTime;

volatile uint32_t _wheel_encoder_counter;


void setup(void) { 
  pinMode(0, INPUT_PULLUP);
  attachInterrupt(0, wheel_encoder_tick, CHANGE);

  Wire.begin();
  delay(2000);
  TWBR = 12;  // set 400kHz mode @ 16MHz CPU or 200kHz mode @ 8MHz CPU

  Serial.begin(115200);  // Setup serial connection
  // while (!Serial);    // For testing using a Leonardo

  myUpdateTime = 0;

  myRover.setUpdateCB(&updateCallback);
  myRover.setTickToDistanceConversion(TICKS_PER_FOOT);
  myRover.setSteeringValues(STEER_CENTER);
  myRover.setThrottleValues(THROTTLE_CENTER, THROTTLE_MIN_FWD, THROTTLE_MIN_REV);
  int test = myRover.setupRover(THR_PIN, STR_PIN, BTN_PIN, LED_PIN, IMU_ADDR);
  if (test == 0) {
    while (true) {
      Serial.println(F("No IMU found"));
      digitalWrite(LED_PIN, HIGH);
      delay(500);
      digitalWrite(LED_PIN, LOW);
      delay(500);
    }   
  }

  reset();
}

void loop(void) 
{
  myRover.waitForButtonPress();
  reset();
  drive();
}

// t1, t2 are distances in feet
// Speed is in throttle percent
#define t1 10.0
#define t2 20.0
#define SPEED 10.0
void drive() {
  myRover.setModeRun();
  myRover.setThrottlePercent(SPEED);
  myRover.waitForDistance(t1); 
  
  myRover.turnToLeft(90);
  myRover.waitForDistance(t2);
  
  myRover.turnToLeft(90);
  myRover.waitForDistance(t2);
  
  myRover.turnToLeft(90);
  myRover.waitForDistance(t2);
  
  myRover.turnToLeft(90);
  myRover.waitForDistance(t1);
  
  myRover.brakeStop(200);
  myRover.setModeWait();
}

void updateCallback(int state) {
  uint32_t ticks = update_wheel_encoder();
  myRover.updateAllBasic();
  
  if ((millis() - myUpdateTime) > 200) {
    myUpdateTime = millis();
    myRover.updateCalibrationLED();
    uint8_t stats[4];
    myRover.readCalibrationStats(stats);

    #ifdef PRINT_UPDATES
      Serial.print(ticks);
      Serial.print(" ");
      Serial.print(stats[0]);
      Serial.print(stats[1]);
      Serial.print(stats[2]);
      Serial.print(stats[3]);
      Serial.print(" ");
      Serial.println(myRover.getCurrentHeading());
    #endif
  }
}

void reset() {
  set_wheel_encoder(0);
  myRover.reset();
}

uint32_t update_wheel_encoder() {
  uint32_t ticks = get_wheel_encoder();
  myRover.setTotalTicks(ticks);
  return ticks;
}

void set_wheel_encoder(uint32_t ticks) {
  noInterrupts();
  _wheel_encoder_counter = ticks;
  interrupts();  
  myRover.setTotalTicks(ticks);
}

uint32_t get_wheel_encoder() {
  uint32_t ticks;
  noInterrupts();
  ticks = _wheel_encoder_counter;
  interrupts();
  return ticks;
}

// Encoder pin change callback
void wheel_encoder_tick() {
  _wheel_encoder_counter++;
}
