/* XYZ Wheel Encoder Library
 by Ted Meyers (3/19/2016)
 https://github.com/TedMeyers/XYZ-robo

 license: Colaware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.
*/
#ifndef __XYZ_QUADRATURE_ENCODER_H__
#define __XYZ_QUADRATURE_ENCODER_H__

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#define READ_08 (PINB & B00000001)
#define READ_09 ((PINB & B00000010) >> 1)
#define READ_10 ((PINB & B00000100) >> 2)
#define READ_11 ((PINB & B00001000) >> 3)
#define READ_12 ((PINB & B00010000) >> 4)
#define READ_13 ((PINB & B00100000) >> 5)

#define ENC_PIN_A 8
#define ENC_PIN_B 12
#define READ_ENC_A READ_08
#define READ_ENC_B READ_12

void encoder_tickA();
void encoder_tickB();

class XYZ_QuadratureEncoder
{
  public:
    XYZ_QuadratureEncoder();

    int getA();
    int getB();

    void reset();
    void set_encoder(uint32_t ticks);
    uint32_t get_encoder_count();
};

#endif