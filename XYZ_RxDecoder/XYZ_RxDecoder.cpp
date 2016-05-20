/* XYZ_RxDecoder Code
 by Ted Meyers (4/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.

 Options (see .h file): RX_ENABLE_AUX - 0 or 1

 Uses XYZ_Interrupt, which uses the EnableInterrupts library.
*/
#include "XYZ_RxDecoder.h"

#define LIBCALL_ENABLEINTERRUPT
#include "XYZ_Interrupts.h"

volatile uint32_t _g_last_pulse_time_us = 0;    // For valid signal heartbeat check (in micros (us))

// The latest PWM duration (in micros (us)) for each channel
volatile uint16_t _g_throttle_duration_us = 0;
volatile uint16_t _g_steering_duration_us = 0;

#if (RX_ENABLE_AUX)
  volatile uint16_t _g_auxiliary_duration_us = 0;
#endif

// ----------------------------------------------------------------
// Interrupt Service Routines
// These record the start time of the pulse, then
// calculate the pulse duration when the pulse ends.
// ----------------------------------------------------------------
static XYZ_RxDecoder *_g_this;
void updateThrottleISR() {
  _g_this->updateThrottleDuration();
}
void updateSteeringISR() {
  _g_this->updateSteeringDuration();
}
#if (RX_ENABLE_AUX)
  void updateAuxISR() {
    _g_this->updateAuxiliaryDuration();
  }
#endif

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
XYZ_RxDecoder::XYZ_RxDecoder()
{
  _th_pin = RX_PIN_NOT_USED;
  _st_pin = RX_PIN_NOT_USED;
  #if (RX_ENABLE_AUX)
    _aux_pin = RX_PIN_NOT_USED;
  #endif
}

void XYZ_RxDecoder::setup(uint8_t th_pin, uint8_t st_pin, uint8_t aux_pin)
{
  _g_this = this;

  _th_pin = th_pin;
  _st_pin = st_pin;

  if (_th_pin != RX_PIN_NOT_USED) {
    pinMode(_th_pin, INPUT_PULLUP);
    enableInterrupt(_th_pin, updateThrottleISR, CHANGE);
  }
  if (_st_pin != RX_PIN_NOT_USED) {
    pinMode(_st_pin, INPUT_PULLUP);
    enableInterrupt(_st_pin, updateSteeringISR, CHANGE);
  }
  #if (RX_ENABLE_AUX)
    _aux_pin = aux_pin;
    if (_aux_pin != RX_PIN_NOT_USED) {
      pinMode(_aux_pin, INPUT_PULLUP);
      enableInterrupt(_aux_pin, updateAuxISR, CHANGE);
    }
  #endif
}

uint16_t XYZ_RxDecoder::getThrottleMicros()
{
	uint16_t usecs;
  	noInterrupts();
  	usecs = _g_throttle_duration_us;
    interrupts();
    return usecs;
}

uint16_t XYZ_RxDecoder::getSteeringMicros()
{
	uint16_t usecs;
  	noInterrupts();
  	usecs = _g_steering_duration_us;
    interrupts();
    return usecs;
}

#if (RX_ENABLE_AUX)
  uint16_t XYZ_RxDecoder::getAuxiliaryMicros()
  {
  	uint16_t usecs;
    	noInterrupts();
    	usecs = _g_auxiliary_duration_us;
      interrupts();
      return usecs;
  }
#endif

void XYZ_RxDecoder::updateThrottleDuration()
{
  static uint32_t th_start_time_us = 0;

  // Throttle has a failsafe in the Rx, so there are always pulses coming in,
  // skip setting the lastPulseTime
  uint32_t t_us = micros();  
  if (digitalRead(_th_pin) == HIGH) { 
    th_start_time_us = t_us;
  } else {
  	noInterrupts();
    _g_throttle_duration_us = (uint16_t)(t_us - th_start_time_us);
    interrupts();
  }
}

void XYZ_RxDecoder::updateSteeringDuration()
{
  static uint32_t st_start_time_us = 0;

  uint32_t t_us = micros();
  noInterrupts();
  _g_last_pulse_time_us = t_us;
  interrupts();  

  if (digitalRead(_st_pin) == HIGH) { 
    st_start_time_us = t_us;
  } else {
  	noInterrupts();
    _g_steering_duration_us = (uint16_t)(t_us - st_start_time_us);
    interrupts();
  }
}

#if (RX_ENABLE_AUX)
  void XYZ_RxDecoder::updateAuxiliaryDuration()
  {
    static uint32_t aux_start_time_us = 0;

    uint32_t t_us = micros();
    noInterrupts();
    _g_last_pulse_time_us = t_us;
    interrupts();

    if (digitalRead(_aux_pin) == HIGH) { 
      aux_start_time_us = t_us;
    } else {
    	noInterrupts();
      _g_auxiliary_duration_us = (uint16_t)(t_us - aux_start_time_us);
      interrupts();
    }
  }
#endif

// ----------------------------------------------------------------
// Check for a valid radio signal, on channels that are NOT
// failsafed (by the Rx), the PWM will stop when the signal is lost,
// so check for a long period of no PWM.
// ----------------------------------------------------------------
bool XYZ_RxDecoder::checkSignal() { 
  static bool _s_valid_signal_flag = false;        // Valid signal present
  static uint32_t lastCheck_ms = 0;

  // Don't want to do this too frequently.
  if ((millis() - lastCheck_ms) > 20) {
    lastCheck_ms = millis();
    uint32_t t_us = micros();
  
    // Don't allow the duration calculation to be interrupted (could cause corruption). 
    uint32_t dur_us; 
    noInterrupts();
    dur_us = (t_us - _g_last_pulse_time_us);
    interrupts();
    
    bool b = _s_valid_signal_flag;
    _s_valid_signal_flag = (dur_us < 70000);    // (70 milliseconds) -- Refresh should be every 20 ms

    if (!_s_valid_signal_flag && b) {
      noInterrupts();
      _g_throttle_duration_us = RX_NO_VALUE;
      _g_steering_duration_us = RX_NO_VALUE;
      #if (RX_ENABLE_AUX)      
        _g_auxiliary_duration_us = RX_NO_VALUE;
      #endif
      interrupts();      
    }
  }
  return _s_valid_signal_flag;
}