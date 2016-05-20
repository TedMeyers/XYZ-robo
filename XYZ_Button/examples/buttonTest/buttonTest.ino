/* XYZ Button Library
 by Ted Meyers (4/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.

 This sketch runs some button tests.  Open your serial 
 terminal and set the rate to 115200 to see the results.
 Oh yeah, and hook up a button and press it when prompted.
*/

#include <XYZ_Button.h>

// DIO pin 8
#define BUTTON_PIN 8

XYZ_Button myButton;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println(F("XYZ_Button Test begin..."));
  myButton.setup(BUTTON_PIN);

  int pin = myButton.getPin();
  int norm = myButton.getNormalButtonState();

  Serial.print(F("   Button on pin: ")); Serial.println(pin);
  Serial.print(F("   Button normal state: ")); Serial.println(norm);
}

void loop() {
  Serial.println();
  delay(1000);

  Serial.println(F("Starting 10 second button press test..."));
  long t = millis();
  bool lastPress = myButton.isPressed();
  while ((millis() - t) < 10000)
  {
  	bool isPressed = myButton.isPressed();
  	if (isPressed != lastPress) {
  		Serial.print(F("Button pressed = ")); Serial.println(isPressed);
  	}
  	lastPress = isPressed;
  }
  Serial.println();

  delay(1000);
  Serial.println(F("Waiting 5 second for button press..."));
  bool b = myButton.waitForPress(5000);
  Serial.print(F("Button pressed = ")); Serial.println(b);
  Serial.println();

  delay(1000);
  Serial.println(F("Waiting for button press..."));
  b = myButton.waitForPress();
  Serial.print(F("Button pressed = ")); Serial.println(b);
}