/* XYZ_Button Code
 by Ted Meyers (4/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.

 This is a simple library that is used to read a button,
 or any other digital input that turns on and off.  Note
 that there is code to automatically detect the type
 of button (normally open or normally closed), so this
 class may not work for all applications.  (Sometimes,
 you want to make the distinction).
*/
#ifndef _XYZ_BUTTON_H__
#define _XYZ_BUTTON_H__

// ----------------------------
#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
// ----------------------------


class XYZ_Button
{
  public:
    XYZ_Button();

    void setup(uint8_t pin);

    uint8_t getPin() { return _button_pin; }
    uint8_t read() { return digitalRead(_button_pin); }
    uint8_t getNormalButtonState() { return _normal_button_state; }
    void setNormalButtonState() { _normal_button_state = read(); }

    bool isPressed();                          // returns true if button is pressed (not normal state)
    bool waitForPress();                       // Wait without timeout
    bool waitForPress(uint32_t timeout_ms);    // wait with timout (duration)

  private:
    uint8_t _button_pin;                // Pin number of button
    uint8_t _normal_button_state;		// The non-pressed button state
};

#endif