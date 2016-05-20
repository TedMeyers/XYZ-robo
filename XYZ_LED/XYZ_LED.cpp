/* XYZ_LED Code
 by Ted Meyers (4/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.

*/
#include "XYZ_LED.h"


/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
XYZ_LED::XYZ_LED()
{
	_led_pin = 0;
	_led_high = false;
	_blink_end_ms = 0;
}

void XYZ_LED::setup(uint8_t pin, bool is_high)
{
  _led_pin = pin;
  _led_high = is_high;

  pinMode(_led_pin, OUTPUT);
  digitalWrite(_led_pin, LOW);

}

void XYZ_LED::blink(uint32_t time_ms)
{
	write(!_led_high);
	_blink_end_ms = millis() + time_ms;
}

bool XYZ_LED::update() {
	if (_blink_end_ms > 0) {
		if (millis() > _blink_end_ms) {
			_blink_end_ms = 0;
			write();
			return true;
		}
		return false;
	} else {
		write();
	}
	return true;
}
