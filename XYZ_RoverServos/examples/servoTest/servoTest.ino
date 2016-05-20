/* Rover Servos Code
 by Ted Meyers (4/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.

 This sketch runs some simple tests on the servos.
 Open a terminal window at 115200; some status stuff is printed.

 This library uses either the servo library or the TicoServo library.
 Servo library can use any pins, TiCoServo library uses timer 1, so
 is limited to pins 9 and 10 (on the Uno).

 Options (see .h file): SERVO_TYPE - 0 (standard servo) or 1 (TiCoServo)
*/

#include "XYZ_RoverServos.h"

// These are the arduino digital I/O pins used:
// Steering      - 10  (9 or 10 required if using TicoServo library)
// Throttle      -  9  (9 or 10 required if using TicoServo library)
#define STR_PIN 10
#define THR_PIN 9

// Throttle and steering values:
#define STEER_CENTER_MICROS 1500
#define THROTTLE_CENTER_MICROS 1500

// Global Objects...
XYZ_RoverServos myServos;  // The rover servos object (controls the servos)

void setup(void) { 
  Serial.begin(115200);
  //
  // Setup Rover
  //
  myServos.setup(THR_PIN, STR_PIN);
  delay(1000);
}

void loop(void) 
{
  test1();
  delay(1000);
}

int thr_pct = 100;
void test2() {
  myServos.setToCenterSteeringAngle();
  myServos.setToThrottlePercent(thr_pct);
  thr_pct -= 10;  
}

void test1() {
  myServos.setToThrottlePercent(2);
  myServos.setToSteeringAngle(40);
  Serial.println(myServos.getSteeringMicros());
  delay(2000);
  myServos.setToZeroThrottle();
  myServos.setToCenterSteeringAngle();
  Serial.println(myServos.getSteeringMicros());
  delay(2000);
  myServos.setToThrottlePercent(-2);
  myServos.setToSteeringAngle(-40);
  Serial.println(myServos.getSteeringMicros());
  delay(2000);
  myServos.setToZeroThrottle();
  myServos.setToCenterSteeringAngle();
  Serial.println(myServos.getSteeringMicros());
  delay(2000);  
}
