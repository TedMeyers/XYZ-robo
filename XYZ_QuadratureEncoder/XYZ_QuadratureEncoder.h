/* XYZ Wheel Encoder Library
 by Ted Meyers (4/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.
*/
#ifndef __XYZ_QUADRATURE_ENCODER_H__
#define __XYZ_QUADRATURE_ENCODER_H__

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif


// Non-portable way to read arduino pins 
#define READ_00 (PIND & B00000001)
#define READ_01 (PIND & B00000010) >> 1)
#define READ_02 (PIND & B00000100) >> 2)
#define READ_03 (PIND & B00001000) >> 3)
#define READ_04 (PIND & B00010000) >> 4)
#define READ_05 (PIND & B00100000) >> 5)
#define READ_06 (PIND & B01000000) >> 6)
#define READ_07 (PIND & B10000000) >> 7)

#define READ_08 (PINB & B00000001)
#define READ_09 ((PINB & B00000010) >> 1)
#define READ_10 ((PINB & B00000100) >> 2)
#define READ_11 ((PINB & B00001000) >> 3)
#define READ_12 ((PINB & B00010000) >> 4)
#define READ_13 ((PINB & B00100000) >> 5)

// Analog Pins A0-A7
#define READ_14 (PINC & B00000001)
#define READ_15 (PINC & B00000010) >> 1)
#define READ_16 (PINC & B00000100) >> 2)
#define READ_17 (PINC & B00001000) >> 3)
#define READ_18 (PINC & B00010000) >> 4)
#define READ_19 (PINC & B00100000) >> 5)
#define READ_20 (PINC & B01000000) >> 6)
#define READ_21 (PINC & B10000000) >> 7)

// Non-portable way to read arduino pins 
#define READ(port, mask, shift) ((port & (mask)) >> (shift))

void encoder_tickA();
void encoder_tickB();

class XYZ_QuadratureEncoder
{
  public:
    XYZ_QuadratureEncoder(int pinA, int pinB);

    int getA();
    int getB();

    void reset();
    void set_encoder(int32_t ticks);
    int32_t get_encoder_count();
};

#endif