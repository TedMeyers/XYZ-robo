/* XYZ_RxDecoder Code
 by Ted Meyers (4/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.

 This library reads the input signals from a hobby RC receiver (rx).
 The values are put out in pulse widths between about 1000 and 2000
 microseconds.

 Set RX_ENABLE_TH to 0 to disable the THROTTLE channel (if not in use), 1 to enable.
 Set RX_ENABLE_ST to 0 to disable the STEERING channel (if not in use), 1 to enable.
 Set RX_ENABLE_AUX to 0 to disable the AUXILIARY channel (if not in use), 1 to enable.

 Uses XYZ_Interrupt, which uses the EnableInterrupts library.
*/
#ifndef _XYZ_RX_DECODER_H__
#define _XYZ_RX_DECODER_H__

// ----------------------------
#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
// ----------------------------

// Set to 0 to disable, 1 to enable the throttle channel
#define RX_ENABLE_TH 1

// Set to 0 to disable, 1 to enable the steerung channel
#define RX_ENABLE_ST 1

// Many RC transmitters do not have an auxiliary channel, so
// you may want to set to 0 to disable (and save some memory/code space).
#define RX_ENABLE_AUX 1

#define RX_PIN_NOT_USED 99
#define RX_NO_VALUE 0

class XYZ_RxDecoder
{
  public:
    XYZ_RxDecoder();

    void setup(uint8_t th_pin, uint8_t st_pin=RX_PIN_NOT_USED, uint8_t aux_pin=RX_PIN_NOT_USED);

    bool checkSignal();              // Get the signal valid status

    #if (RX_ENABLE_TH)
      uint8_t getThrottlePin() { return _th_pin; }
      uint16_t getThrottleMicros();
      void updateThrottleDuration();
    #endif
    #if (RX_ENABLE_ST)
      uint8_t getSteeringPin() { return _st_pin; }
      uint16_t getSteeringMicros();
      void updateSteeringDuration();
    #endif
    #if (RX_ENABLE_AUX)
      uint8_t getAuxiliaryPin() { return _aux_pin; }
      uint16_t getAuxiliaryMicros();
      void updateAuxiliaryDuration();
    #endif

  private:
    #if (RX_ENABLE_TH)
      uint8_t _th_pin;                // Pin number of throttle input
    #endif
    #if (RX_ENABLE_ST)
      uint8_t _st_pin;                // Pin number of steering input
    #endif
    #if (RX_ENABLE_AUX)
        uint8_t _aux_pin;           // Pin number of auxiliary input
    #endif
};

#endif