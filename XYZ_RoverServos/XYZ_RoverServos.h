/* XYZ Rover Servo Library
 by Ted Meyers (4/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.

 This library is used to control two hobby servo type 
 output signals (usually throttle and steering).

 The standard servo library may be used with any of the 
 arduino output pins, or the tico servo library may be
 used with arduino pins 9 and 10.  The advantage of
 using the tico library is that the signals are hardware
 controlled, so there will be no interference from
 other libraries, as might occur with the standard servo
 library.

 Set SERVO_TYPE to 0 to use the standard servo library,
 or set to 1 to use the TiCoServo library.

 Uses servo library: any pins, no peripherals
 OR 
 TiCoServor libary: Pins 9,10 (Uno) and Timer 1
*/
#ifndef __XYZ_ROVER_SERVO_H__
#define __XYZ_ROVER_SERVO_H__

// ----------------------------
// Servo library used;
//    0 == standard library,
//    1 == Tico Library (Tico is preferred)
// TiCo servos must be on pins 9,10 (on Uno)
#define SERVO_TYPE 1
 
#if (SERVO_TYPE == 0)
  #define USE_STD_SERVO
#else
  #define USE_TICO_SERVO
#endif
// ----------------------------

// ----------------------------
// Includes
// ----------------------------
#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif
 
#ifdef USE_TICO_SERVO
   #include "Adafruit_TiCoServo.h"
#else
   #include "Servo.h"
#endif
// ----------------------------


#define SERVO_MIN 1050
#define SERVO_MAX 1950

#define TH_SCALE 2
#define ST_SCALE 5
#define STEERING_EPS 10

#define DEFAULT_STEER_CENTER_MICROS 1500
#define DEFAULT_STEER_MIN_MICROS 1050
#define DEFAULT_STEER_MAX_MICROS 1950

#define DEFAULT_THROTTLE_CENTER_MICROS 1500
#define DEFAULT_THROTTLE_MIN_MICROS 1100
#define DEFAULT_THROTTLE_MAX_MICROS 1900
#define DEFAULT_THROTTLE_MIN_FWD_MICROS 1590
#define DEFAULT_THROTTLE_MIN_REV_MICROS 1410



class XYZ_RoverServos 
{
  public:
  	XYZ_RoverServos();
    
    void setup(uint8_t th_pin, uint8_t st_pin);
    void reset();

    void setSteeringValues(uint16_t center_us=DEFAULT_STEER_CENTER_MICROS, 
    	uint16_t min_us=DEFAULT_STEER_MIN_MICROS, uint16_t max_us=DEFAULT_STEER_MAX_MICROS);
    
    void setThrottleValues(uint16_t center_us=DEFAULT_THROTTLE_CENTER_MICROS, 
        uint16_t min_fwd_us=DEFAULT_THROTTLE_MIN_FWD_MICROS, uint16_t min_rev_us=DEFAULT_THROTTLE_MIN_REV_MICROS,
        uint16_t min_us=DEFAULT_THROTTLE_MIN_MICROS, uint16_t max_us=DEFAULT_THROTTLE_MAX_MICROS);

    uint16_t getThrottleMicros() { return _cur_throttle_us; }
    uint16_t getSteeringMicros() { return _cur_steering_us; }
    uint16_t getThrottleDirection() { return (_cur_throttle_us - _th_center_us); }
    uint16_t getSteeringDirection() { return (_cur_steering_us - _st_center_us); }

  	void setToZeroThrottle() { updateThrottle(_th_center_us); }
  	void setToCenterSteeringAngle() { updateSteering(_st_center_us); }

    void setToThrottlePercent(int percent) { updateThrottle(normalizeThrottle(percent)); }
    void setToSteeringAngle(int angle) { updateSteering(normalizeSteering(angle)); }

    void updateThrottle(uint16_t val_us);
    void updateSteering(uint16_t val_us);

  private:
    uint16_t normalizeSteering(int amount);
    uint16_t normalizeThrottle(int amount);

    #ifdef USE_TICO_SERVO
        Adafruit_TiCoServo _throttle_servo;
        Adafruit_TiCoServo _steering_servo;
    #else
        Servo _throttle_servo; 
        Servo _steering_servo;
    #endif

    uint16_t _th_center_us;
    uint16_t _th_min_fwd_us;
    uint16_t _th_min_rev_us;
    uint16_t _th_min_us;
    uint16_t _th_max_us;

    uint16_t _st_center_us;
    uint16_t _st_min_us;
    uint16_t _st_max_us;

    uint16_t _cur_throttle_us;    // The currently set throttle in microsecs
    uint16_t _cur_steering_us;    // The currently set steering in microsecs
};

#endif