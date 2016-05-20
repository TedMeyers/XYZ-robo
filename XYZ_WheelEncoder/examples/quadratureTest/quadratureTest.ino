/* XYZ Single Wheel Encoder Library
 by Ted Meyers (4/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.

 This sketch reads and prints out wheel encoder values.
 Open a serial terminal at 115200 to see the results.

 NOTE: Assumes the ENCODER_TYPE is set to QUADRATURE in XYZ_WheelEncoder.h
       Make sure that this is true or this sketch will not compile.

 Uses XYZ_Interrupt, which uses the EnableInterrupts library.
*/

#include <XYZ_WheelEncoder.h>
#include <XYZ_Interrupts.h>

// NOTE: On my setup there are about 27.1 ticks per foot,
// but your setup may be very different.

XYZ_WheelEncoder myEncoder;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  myEncoder.setup(6, 7);      // DIO pins 6 & 7 (on port D)
  myEncoder.resetEncoderCount();
}

void loop() {
  long val = myEncoder.getEncoderCount();
  Serial.print("Quadrature Encoder Count: ");
  Serial.println(val);
}