/* XYZ LED Library
 by Ted Meyers (4/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.

 This sketch runs some LED tests.  Open a terminal window
 at 115200 to see the tests.

 Uses MsTimer2 library for setting a millisecond timer.
*/

#include <XYZ_LED.h>
#include <MsTimer2.h>

#define LED_PIN 13

XYZ_LED myLED;

void updateCB() {
  if (myLED.update()) MsTimer2::stop();
}

void setup() {
  Serial.begin(115200);
  while (!Serial)

  Serial.println(F("XYZ_LED Test begin..."));
  myLED.setup(LED_PIN);

  int pin = myLED.getPin();
  bool b = myLED.isHigh();

  Serial.print(F("   LED on pin: ")); Serial.println(pin);
  Serial.print(F("   LED state: ")); Serial.println(b);
}

void loop() {
  Serial.println(F("   LED on..."));
  myLED.set(true);
  delay(2000);
  Serial.println(F("   LED off..."));
  myLED.set(false);
  delay(2000);
  Serial.println(F("   LED toggle on..."));
  myLED.toggle();
  delay(2000);
  Serial.println(F("   LED toggle off..."));
  myLED.toggle();
  delay(2000);
  Serial.println(F("   LED fast blink..."));
  myLED.blink(500);

  // This part could be done with the timer2 library
  //while (!myLED.update()) { Serial.print("."); delay(50);}
  MsTimer2::set(10, &updateCB);
  MsTimer2::start();

  Serial.println();
  delay(2000);
  Serial.println(F("   LED faster blink..."));
  myLED.blink(100);

  // This part could be done with the timer2 library
  //while (!myLED.update()) { Serial.print("."); delay(20); }
  MsTimer2::start();

  Serial.println();
  delay(2000);
}