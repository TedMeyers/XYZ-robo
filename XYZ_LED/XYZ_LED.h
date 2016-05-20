/* XYZ_LED Code
 by Ted Meyers (4/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.

 This is a simple library that is used to control a LED, or
 any other digital output that turns on and off.
*/
#ifndef _XYZ_LED_H__
#define _XYZ_LED_H__

// ----------------------------
#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
// ----------------------------


class XYZ_LED
{
  public:
    XYZ_LED();

    void setup(uint8_t pin, bool is_high = false);

    uint8_t getPin() { return _led_pin; }
    bool isHigh() { return _led_high; }

    void set(bool is_high) { _led_high = is_high; write(is_high); }
    void setOn() {_led_high = true; write(true); }
    void setOff() {_led_high = false; write(false); }
    void toggle() { _led_high = !_led_high; update(); }

    bool update();
    void blink(uint32_t time_ms);

  private:
    void write() { digitalWrite(_led_pin, (_led_high)?HIGH:LOW); }
    void write(bool is_high) { digitalWrite(_led_pin, (is_high)?HIGH:LOW); }

    uint8_t _led_pin;                // Pin number of led
    bool _led_high;                  // Is the LED High
    uint32_t _blink_end_ms;          // End time for blink in millis
};

#endif