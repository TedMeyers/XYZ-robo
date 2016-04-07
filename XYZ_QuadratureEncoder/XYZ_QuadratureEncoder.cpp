/* XYZ Wheel Encoder Library
 by Ted Meyers (4/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.
*/
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include "XYZ_QuadratureEncoder.h"

// --------------------------------------------------------------------
// Enable Interrupts defines
//
// Save momory by assuming that only pins on port D are used (0-7)
// USE CARE!  This is non-portable!
//
#define EI_NOTPORTB
#define EI_NOTPORTC
#define EI_NOTEXTERNAL
// #define LIBCALL_ENABLEINTERRUPT
//
// Interrupts library
#include <EnableInterrupt.h>
// --------------------------------------------------------------------

uint8_t g_enc_highA;
uint8_t g_enc_highB;

volatile uint32_t g_encoder_counter;    // The number of encoder ticks

// Called when the encoder pin changes
void encoder_tickA() {
  noInterrupts();
  if (g_enc_highA) {
    g_enc_highA = false;
    if (g_enc_highB) g_encoder_counter--;
    else g_encoder_counter++;
  } else {
    g_enc_highA = true;
    if (g_enc_highB) g_encoder_counter++;
    else g_encoder_counter--;
  }
  interrupts();
}

// Called when the encoder pin changes
void encoder_tickB() {
  noInterrupts();
  if (g_enc_highB) {
    g_enc_highB = false;
    if (g_enc_highA) g_encoder_counter++;
    else g_encoder_counter--;
  } else {
    g_enc_highB = true;
    if (g_enc_highA) g_encoder_counter--;
    else g_encoder_counter++;
  }
  interrupts();  
}

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
// IMPORTANT: pin can only be any PORT D pin (0-7)
// UNLESS: you change the enable interrupts #defines above!
// This limitation exists because of memory saving assumptions that the
// interrupt is on port D.
XYZ_QuadratureEncoder::XYZ_QuadratureEncoder(int pinA, int pinB) {

  // Non-portable pin reading!
  //
  // uint8_t portA =  PINB;
  // uint8_t portB = PINB;
  // uint8_t maskA = 0;
  // uint8_t maskB = 0;
  // uint8_t shiftA = 0;
  // uint8_t shiftB = 0;

  // if (pinA ==  8) { maskA = B00000001; shiftA = 0; }
  // if (pinA ==  9) { maskA = B00000010; shiftA = 1; }
  // if (pinA == 10) { maskA = B00000100; shiftA = 2; }
  // if (pinA == 11) { maskA = B00001000; shiftA = 3; }
  // if (pinA == 12) { maskA = B00010000; shiftA = 4; }   
  // if (pinA == 13) { maskA = B00100000; shiftA = 5; }   

  // if (pinB ==  8) { maskB = B00000001; shiftB = 0; }
  // if (pinB ==  9) { maskB = B00000010; shiftB = 1; }
  // if (pinB == 10) { maskB = B00000100; shiftB = 2; }
  // if (pinB == 11) { maskB = B00001000; shiftB = 3; }
  // if (pinB == 12) { maskB = B00010000; shiftB = 4; }   
  // if (pinB == 13) { maskB = B00100000; shiftB = 5; }  

  // g_enc_highA = READ(portA, maskA, shiftA);
  // g_enc_highB = READ(portB, maskB, shiftB);

  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);

  g_enc_highA = digitalRead(pinA);
  g_enc_highB = digitalRead(pinB);

  enableInterrupt(pinA, encoder_tickA, CHANGE);
  enableInterrupt(pinB, encoder_tickB, CHANGE);

  reset();
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/
int XYZ_QuadratureEncoder::getA() { return g_enc_highA; }
int XYZ_QuadratureEncoder::getB() { return g_enc_highB; }

void XYZ_QuadratureEncoder::reset() {
  set_encoder(0);
}

// Safely set the wheel encoder tick value
void XYZ_QuadratureEncoder::set_encoder(int32_t ticks) {
  noInterrupts();
  g_encoder_counter = ticks;
  interrupts();  
}

// Get the wheel encoder tick value safely
int32_t XYZ_QuadratureEncoder::get_encoder_count() {
  int32_t count;
  noInterrupts();
  count = g_encoder_counter;
  interrupts();
  return count;
}

