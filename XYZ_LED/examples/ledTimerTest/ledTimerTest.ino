/* LED Timer Test
 by Ted Meyers (4/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.

 This is an example of how to use a timer to blink an LED;
 obviously there are more interesting things to do.

 Uses MsTimer2 library for setting a millisecond timer.
*/

#include <XYZ_LED.h>
#include <MsTimer2.h>

#define LED_PIN 13

XYZ_LED myLED;

void updateCB() {
  myLED.toggle();
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  myLED.setup(LED_PIN);

  MsTimer2::set(50, &updateCB);
  MsTimer2::start();

  Serial.println(F("XYZ_Updater Test start..."));
}

void loop() {
  Serial.println("Wait...");
  delay(2000);
}