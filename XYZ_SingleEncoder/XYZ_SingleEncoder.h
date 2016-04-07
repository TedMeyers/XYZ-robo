/* XYZ Wheel Encoder Library
 by Ted Meyers (3/19/2016)
 https://github.com/TedMeyers/XYZ-robo

 license: Colaware - Use this code however you'd like. If you 
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
    void set_encoder(uint32_t ticks);
    uint32_t get_encoder_count();
};

#endif