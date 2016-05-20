/* XYZ_RxDecoder Library
 by Ted Meyers (4/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.

 This sketch runs some Rx (Receiver PWM decoder) tests.  It
 reads the output from a hobby RC receiver and prints
 out the microsecond modulation values
 (typically 1000-2000 microseconds).

 Open a terminal window at 115200 to see the results.

 Options (see .h file): RX_ENABLE_AUX - 0 or 1

 Uses XYZ_Interrupt, which uses the EnableInterrupts library.
*/
#include <XYZ_RxDecoder.h>
#include <XYZ_Interrupts.h>

#define TH_PIN 3
#define ST_PIN 4
#define AUX_PIN 5

XYZ_RxDecoder myRx;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println(F("XYZ_RxDecoder Test begin..."));
  myRx.setup(TH_PIN, ST_PIN, AUX_PIN);

  int th = myRx.getThrottlePin();
  int st = myRx.getSteeringPin();
  int aux = myRx.getAuxiliaryPin();

  Serial.print(F("   Rx th  on pin: ")); Serial.println(th);
  Serial.print(F("   Rx st  on pin: ")); Serial.println(st);
  Serial.print(F("   Rx aux on pin: ")); Serial.println(aux);
}

void loop() {
  uint16_t th = myRx.getThrottleMicros();
  uint16_t st = myRx.getSteeringMicros();
  uint16_t aux = myRx.getAuxiliaryMicros();
  bool valid = myRx.checkSignal();

  Serial.print(F("  TH: ")); 
  Serial.print(th);
  Serial.print(F("  ST: ")); 
  Serial.print(st);
  Serial.print(F("  AUX: ")); 
  Serial.print(aux);
  Serial.print(F("  VALID: ")); 
  Serial.println(valid);


  delay(200);
}