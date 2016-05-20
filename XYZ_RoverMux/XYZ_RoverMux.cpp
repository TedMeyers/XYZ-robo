/* XYZ_RoverMux (multiplexer) Code
 by Ted Meyers (4/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.

 Options (see .h file): USE_DEFAULTS_FLAG, ENABLE_MUX_AUX 

 Uses XYZ_RxDecoder library and RoverServos library.
 These use EnableInterrupts and possibly TiCoServo (Timer1).
*/
#include "XYZ_RoverMux.h"

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
XYZ_RoverMux::XYZ_RoverMux()
{
  _cur_mode = NONE_MUX_MODE;
  _auto_throttle_percent = NO_SIGNAL;
  _auto_steering_angle = NO_SIGNAL;
}

int8_t XYZ_RoverMux::updateMuxMode()
{
  static uint8_t _s_cur_mux_mode = NONE_MUX_MODE;             // The current mux mode
  static uint8_t _s_next_mux_mode = NONE_MUX_MODE;            // The next mux mode
  static uint8_t _s_mode_count = 0;                           // The mode counter

  uint8_t t_mode = DEFAULT_MUX_MODE; 
  bool valid = _rx.checkSignal();
  if (valid) {
    #if (ENABLE_MUX_AUX)
      uint16_t aux_us = _rx.getAuxiliaryMicros();
      bool is_manual = (aux_us > DEFAULT_AUX_THRESH_US);
      if (DEFAULT_AUX_DIR_DOWN) {
        is_manual = !is_manual;
      }
      t_mode = (is_manual)?MANUAL_MUX_MODE:AUTO_MUX_MODE;
    #else
      t_mode = MANUAL_MUX_MODE;
    #endif
  } else {
    #if (USE_DEFAULTS_FLAG)
      t_mode = DEFAULT_MUX_MODE;
    #else
      t_mode = AUTO_MUX_MODE;
    #endif
  }
  _cur_mode = t_mode;

  // test the mode for consistency
  // The point here is to require a few identical readdings in
  // a row, before switching modes.
  if ((t_mode == _s_cur_mux_mode) || (t_mode != _s_next_mux_mode)) {
    _s_mode_count = 0;
  } else {
    if (_s_mode_count > 1)  {
      _s_mode_count = 0;
      _s_cur_mux_mode = t_mode;
      return t_mode;
    } else _s_mode_count++;
  }
  _s_next_mux_mode = t_mode;

  return (-1 * _s_cur_mux_mode);
}

void XYZ_RoverMux::setToThrottlePercent(int percent)
{
  if (_cur_mode == AUTO_MUX_MODE) {
      _servos.setToThrottlePercent(percent);
  }
  _auto_throttle_percent = percent;
}

void XYZ_RoverMux::setToSteeringAngle(int angle)
{
  if (_cur_mode == AUTO_MUX_MODE) {
      _servos.setToSteeringAngle(angle);
  }
  _auto_steering_angle = angle;
}

void XYZ_RoverMux::update()
{
  updateMuxMode();
  updateServos();  
}

void XYZ_RoverMux::updateServos(uint16_t auto_th_pct, uint16_t auto_st_ang)
{
  if (_cur_mode == DEFAULT_MUX_MODE) {
    setServoToDefaultValues();
  } else if (_cur_mode == MANUAL_MUX_MODE) {
    setServoToManualValues();
  } else if (_cur_mode == AUTO_MUX_MODE) {
    setServosToValues(auto_th_pct, auto_st_ang);
  }
}

void XYZ_RoverMux::setServoToDefaultValues()
{
  #if (DEFAULT_THROTTLE_USEC == NO_SIGNAL)
    _servos.setToZeroThrottle();
  #else
    _servos.updateThrottle(DEFAULT_THROTTLE_USEC);
  #endif

  #if (DEFAULT_STEERING_USEC == NO_SIGNAL)
    _servos.setToCenterSteeringAngle();
  #else
    _servos.updateSteering(DEFAULT_STEERING_USEC);
  #endif
}

void XYZ_RoverMux::setServoToManualValues()
{
  if (_rx.checkSignal()) {
    _servos.updateThrottle(_rx.getThrottleMicros());
    _servos.updateSteering(_rx.getSteeringMicros());
  }
}

void XYZ_RoverMux::setServosToValues(uint16_t th_pct, uint16_t st_ang)
{
  _auto_throttle_percent = th_pct;
  _auto_steering_angle = st_ang;
  _servos.setToThrottlePercent(th_pct);
  _servos.setToSteeringAngle(st_ang);
}

