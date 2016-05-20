/* XYZ Rover Servo Library
 by Ted Meyers (4/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.

 Options (see .h file): SERVO_TYPE - 0 (standard servo) or 1 (TiCoServo)
*/

#include "XYZ_RoverServos.h"

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
XYZ_RoverServos::XYZ_RoverServos()
{
  _cur_throttle_us = 0;
  _cur_steering_us = 0;
  setSteeringValues();
  setThrottleValues();
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/
void XYZ_RoverServos::setup(uint8_t th_pin, uint8_t st_pin) { 
  #ifdef USE_TICO_SERVO
    _throttle_servo.attach(th_pin, SERVO_MIN, SERVO_MAX);
    _steering_servo.attach(st_pin, SERVO_MIN, SERVO_MAX);
  #else
    _throttle_servo.attach(th_pin);
    _steering_servo.attach(st_pin);
  #endif

  _cur_throttle_us = DEFAULT_STEER_CENTER_MICROS;
  _cur_steering_us = DEFAULT_THROTTLE_CENTER_MICROS;
  _throttle_servo.write(_cur_throttle_us);
  _steering_servo.write(_cur_steering_us);
}

void XYZ_RoverServos::reset() {
  setToZeroThrottle();
  setToCenterSteeringAngle();
}

void XYZ_RoverServos::setSteeringValues(uint16_t center_us, uint16_t min_us, uint16_t max_us) {
  _st_center_us = center_us;
  _st_min_us = min_us;
  _st_max_us = max_us;
}

void XYZ_RoverServos::setThrottleValues(uint16_t center_us, 
  uint16_t min_fwd_us, uint16_t min_rev_us, uint16_t min_us, uint16_t max_us) {

  _th_center_us = center_us;
  _th_min_fwd_us = min_fwd_us;
  _th_min_rev_us = min_rev_us;
  _th_min_us = min_us;
  _th_max_us = max_us;
}


void XYZ_RoverServos::updateThrottle(uint16_t val_us) {
  if (_cur_throttle_us != val_us) {
    _cur_throttle_us = val_us;
    _throttle_servo.write(_cur_throttle_us);
  }
}
void XYZ_RoverServos::updateSteering(uint16_t val_us) {
  if (_cur_steering_us != val_us) {
    _cur_steering_us = val_us;
    _steering_servo.write(_cur_steering_us);
  }
}

uint16_t XYZ_RoverServos::normalizeThrottle(int percent) {
  uint16_t th_micros = _th_center_us;

  if (percent < 0) th_micros = _th_min_rev_us + ((percent+1) * TH_SCALE);
  else if (percent > 0) th_micros = _th_min_fwd_us + ((percent-1) * TH_SCALE);

  if (th_micros < _th_min_us) th_micros = _th_min_us;
  if (th_micros > _th_max_us) th_micros = _th_max_us;
  return th_micros;
}
uint16_t XYZ_RoverServos::normalizeSteering(int angle) {
  uint16_t st_micros = _st_center_us + (angle * ST_SCALE);

  if ((st_micros < _st_center_us) && (st_micros > (_st_center_us-STEERING_EPS))) 
    st_micros = _st_center_us-STEERING_EPS;
  if ((st_micros > _st_center_us) && (st_micros < (_st_center_us+STEERING_EPS))) 
    st_micros = _st_center_us+STEERING_EPS;

  if (st_micros < _st_min_us) st_micros = _st_min_us;
  if (st_micros > _st_max_us) st_micros = _st_max_us;

  return st_micros;
}
