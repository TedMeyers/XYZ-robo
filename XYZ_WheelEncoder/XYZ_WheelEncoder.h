/* XYZ Wheel Encoder Library
 by Ted Meyers (4/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.

 This library reads signals from a wheel encoder.  There
 are two basic types of encoders, single and quadrature.  This
 library can handle either type, but the ENCODER_TYPE must 
 be set (below).

 The single encoder has one sensor that changes its value
 (high to low or low to high) for every turn (or part of turn)
 of the wheel.  It only tells if the wheel is turning, not the
 direction.

 A quadrature encoder has two sensors that are 90 degrees out of
 phase of each other.  This allows the sensor to measure the 
 number of turns (or parts of turns) of the wheel and the direction.
 It has double the resolution of the single encoder and also gives
 direction making it much more useful, if a bit more complicated.

 Options: ENCODER_TYPE - QUADRATURE or SINGLE

 Uses XYZ_Interrupt, which uses the EnableInterrupts library.
*/
#ifndef __XYZ_WHEEL_ENCODER_H__
#define __XYZ_WHEEL_ENCODER_H__

// ----------------------------
#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
// ----------------------------

#define QUADRATURE 0
#define SINGLE 1

//
// IMPORTANT: Set type to SINGLE -OR- QUADRATURE here!!
//
#define ENCODER_TYPE SINGLE

#if (ENCODER_TYPE == SINGLE)
  extern void encoder_tick();
#elif (ENCODER_TYPE == QUADRATURE)
  extern void encoder_tickA();
  extern void encoder_tickB();

  extern volatile uint8_t _g_enc_high_a_flag;
  extern volatile uint8_t _g_enc_high_b_flag;
#endif

extern volatile int32_t _g_encoder_counter;

class XYZ_WheelEncoder
{
  public:
	XYZ_WheelEncoder();

    #if (ENCODER_TYPE == SINGLE)
      void setup(int pin);
    #elif (ENCODER_TYPE == QUADRATURE)
      void setup(int pinA, int pinB);
      int getA() { return _g_enc_high_a_flag; }
      int getB() { return _g_enc_high_b_flag; }
    #endif

    void resetEncoderCount()  { setEncoderCount(0); }
    void setEncoderCount(int32_t count);
    int32_t getEncoderCount();
};

#endif