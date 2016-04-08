/* XYZ Wheel Encoder Library
 by Ted Meyers (4/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.
*/
#include <XYZ_QuadratureEncoder.h>

 // NOTE: On my setup there are about 54.0 ticks per foot,
 // but your setup may be very different.

XYZ_QuadratureEncoder enc(6, 7);  // Uses DIO pins 6 & 7 (on port D)

void setup() {
  Serial.begin(9600);
while (!Serial);
  enc.reset();
}

void loop() {
  long val = enc.get_encoder_count();
  Serial.print("Quadrature Encoder Count: ");
  Serial.println(val);
}