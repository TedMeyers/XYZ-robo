/* XYZ_RoverMux (Multiplexer) Code
 by Ted Meyers (4/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.

 A multiplexer is used to select between inputs and send the
 selected inputs to an output signal.  In this case it automatically
 selects the Rx (RC receiver) inputs or the rover code (Arduino)
 based on the existence of a radio signal (and the channel 3 value,
 if there is a channel 3) or sets failsafe values.

 This is a useful feature for testing your rover, as you can
 use your RC transmitter to take control back in case your
 rover goes haywire.

 Set USE_DEFAULTS_FLAG to configure the mux to set the signals when
 a valid RC signal is not present to the default values 
 (zero throttle, zero steering, manual) when true, or to rover (auto) 
 values when false.

 Set ENABLE_MUX_AUX to configure the mux to use (true) or ignore (false) 
 the auxiliary channel and just use the presence of a valid RC signal to
 switch to manual signals -- set this flag in conjunction with the 
 USE_DEFAULTS flag.

 USE_DEFAULTS_FLAG/ENABLE_MUX_AUX : TRUTH TABLE
 F/T - no signal enables auto or Use manual/auto switch (tx signal not req)
 T/T - no signal disables auto or Use manual/auto switch (tx signal required)
 F/F - no signal enables auto or Use manual (auto/manual on tx signal)
 T/F - no signal disables auto or Use manual (manual only on tx signal)

 Personally, I like F/T; for competition use: T/F

Uses XYZ_RxDecoder library and RoverServos library.
These use EnableInterrupts and possibly TiCoServo (Timer1).
*/
#ifndef _XYZ_ROVER_MUX_H__
#define _XYZ_ROVER_MUX_H__

// ----------------------------
#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
// ----------------------------
#include "XYZ_RxDecoder.h"
#include "XYZ_RoverServos.h"

// Set defaults for invalid signal when true (see above)
// or enable auto for invalid signal when false
#define USE_DEFAULTS_FLAG false

// Set to true to use manual switch
// Set to false to use valid signal (see above)
#define ENABLE_MUX_AUX true

#define NOT_USED 99
#define NO_SIGNAL 0

#define DEFAULT_AUX_DIR_DOWN false
#define DEFAULT_AUX_THRESH_US 1700 

#define NONE_MUX_MODE 0
#define DEFAULT_MUX_MODE 1
#define AUTO_MUX_MODE 2
#define MANUAL_MUX_MODE 3

// Setting to NO_SIGNAL (0) will zero the throttle or 
// center the steering
#define DEFAULT_THROTTLE_USEC NO_SIGNAL
#define DEFAULT_STEERING_USEC NO_SIGNAL


class XYZ_RoverMux
{
  public:
    XYZ_RoverMux();

    void reset() { _servos.reset(); }

    int8_t updateMuxMode();

    uint8_t getMuxMode() { return  _cur_mode; }
    bool isDefaultMode() { return (_cur_mode == DEFAULT_MUX_MODE);  }
    bool isManualMode() { return (_cur_mode == MANUAL_MUX_MODE);  }
    bool isAutoMode() { return (_cur_mode == AUTO_MUX_MODE);  }
    uint16_t getThrottlePercent() { return _auto_throttle_percent; }
    uint16_t getSteeringAngle() { return _auto_steering_angle; }

    XYZ_RoverServos *getServos() { return &_servos; }
    XYZ_RxDecoder *getRx() { return &_rx; };

    void setToThrottlePercent(int percent);
    void setToSteeringAngle(int angle);

    void update();
    void updateServos() { updateServos(_auto_throttle_percent, _auto_steering_angle); }
    void updateServos(uint16_t auto_th_pct, uint16_t auto_st_ang);

  private:
    void setServoToDefaultValues();
    void setServoToManualValues();
    void setServosToValues(uint16_t th_pct, uint16_t st_ang);

    XYZ_RoverServos _servos;           // The rover servos object (controls the servos)
    XYZ_RxDecoder _rx;

    int8_t _cur_mode;
    uint16_t _auto_throttle_percent;
    uint16_t _auto_steering_angle;
};

#endif