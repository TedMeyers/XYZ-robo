/* Button Code
 by Ted Meyers (4/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.

*/
#include "XYZ_Button.h"

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
XYZ_Button::XYZ_Button()
{
	_button_pin = 0;
	_normal_button_state = 0;
}

void XYZ_Button::setup(uint8_t pin)
{
  _button_pin = pin;
  pinMode(_button_pin, INPUT_PULLUP);
  setNormalButtonState();
}

bool XYZ_Button::isPressed()
{
  uint8_t curBtn = digitalRead(_button_pin);
  return (curBtn != _normal_button_state);
}

bool XYZ_Button::waitForPress() 
{
	while (!isPressed());
	return true;
}

bool XYZ_Button::waitForPress(uint32_t timeout_ms)
{
	uint32_t time_ms = millis();
	uint32_t duration_ms = 0;
	bool is_pressed = false;
	while (!is_pressed && (duration_ms < timeout_ms)) {
		duration_ms = (millis() - time_ms);
		is_pressed = isPressed();
	}
	return is_pressed;
}

