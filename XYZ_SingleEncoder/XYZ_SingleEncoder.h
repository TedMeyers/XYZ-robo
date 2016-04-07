/* XYZ Wheel Encoder Library
 by Ted Meyers (4/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.
*/
#ifndef __XYZ_SINGLE_ENCODER_H__
#define __XYZ_SINGLE_ENCODER_H__

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

void encoder_tick();

class XYZ_SingleEncoder
{
  public:
    XYZ_SingleEncoder(int pin);

    void reset();
    void set_encoder(int32_t ticks);
    int32_t get_encoder_count();
};

#endif