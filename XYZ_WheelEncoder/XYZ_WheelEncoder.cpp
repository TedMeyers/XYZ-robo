/* XYZ Wheel Encoder Library
 by Ted Meyers (4/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.

 Options (see .h file): ENCODER_TYPE - QUADRATURE or SINGLE

 Uses XYZ_Interrupt, which uses the EnableInterrupts library.
*/
#include "XYZ_WheelEncoder.h"

#define LIBCALL_ENABLEINTERRUPT
#include "XYZ_Interrupts.h"

volatile uint32_t _g_encoder_counter;    // The number of encoder ticks

// Called when the encoder pin changes
#if (ENCODER_TYPE == SINGLE)
  void encoder_tick() {
    noInterrupts();
    _g_encoder_counter++;
    interrupts();  
  }
#elif (ENCODER_TYPE == QUADRATURE)
  volatile uint8_t _g_enc_high_a_flag;
  volatile uint8_t _g_enc_high_b_flag;
  
  void encoder_tickA() {
    noInterrupts();
    if (_g_enc_high_a_flag) {
      _g_enc_high_a_flag = false;
      if (_g_enc_high_b_flag) _g_encoder_counter--;
      else _g_encoder_counter++;
    } else {
      _g_enc_high_a_flag = true;
      if (_g_enc_high_b_flag) _g_encoder_counter++;
      else _g_encoder_counter--;
    }
    interrupts();
  }

  // Called when the encoder pin changes
  void encoder_tickB() {
    noInterrupts();
    if (_g_enc_high_b_flag) {
      _g_enc_high_b_flag = false;
      if (_g_enc_high_a_flag) _g_encoder_counter++;
      else _g_encoder_counter--;
    } else {
      _g_enc_high_b_flag = true;
      if (_g_enc_high_a_flag) _g_encoder_counter--;
      else _g_encoder_counter++;
    }
    interrupts();  
  }
#endif

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
XYZ_WheelEncoder::XYZ_WheelEncoder()
{
  resetEncoderCount();
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/
// IMPORTANT: pin can only be any PORT D pin (0-7)
// UNLESS: you change the enable interrupts #defines above!
// This limitation exists because of memory saving assumptions that the
// interrupt is on port D.
#if (ENCODER_TYPE == SINGLE)
  void XYZ_WheelEncoder::setup(int pin)
  {
    pinMode(pin, INPUT_PULLUP);
    enableInterrupt(pin, encoder_tick, CHANGE);  
  }
#elif (ENCODER_TYPE == QUADRATURE)
  void XYZ_WheelEncoder::setup(int pin_a, int pin_b)
  {
      pinMode(pin_a, INPUT_PULLUP);
      _g_enc_high_a_flag = digitalRead(pin_a);
      enableInterrupt(pin_a, encoder_tickA, CHANGE);

      pinMode(pin_b, INPUT_PULLUP);
      _g_enc_high_b_flag = digitalRead(pin_b);
      enableInterrupt(pin_b, encoder_tickB, CHANGE);
  }
#endif

// Safely set the wheel encoder tick value
void XYZ_WheelEncoder::setEncoderCount(int32_t count) {
  noInterrupts();
  _g_encoder_counter = count;
  interrupts();  
}

// Get the wheel encoder tick value safely
int32_t XYZ_WheelEncoder::getEncoderCount() {
  int32_t count;
  noInterrupts();
  count = _g_encoder_counter;
  interrupts();
  return count;
}
