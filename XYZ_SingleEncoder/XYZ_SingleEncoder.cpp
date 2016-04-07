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

#include "XYZ_SingleEncoder.h"

// --------------------------------------------------------------------
// Enable Interrupts defines
//
// Save momory by assuming that only pins on port D are used (pins 0-7)
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

volatile int32_t g_encoder_counter;    // The number of encoder ticks

// Called when the wheel encoder pin changes
void encoder_tick() {
  noInterrupts();
  g_encoder_counter++;
  interrupts();  
}

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
// IMPORTANT: pin can only be any PORT D pin (0-7)
// UNLESS: you change the enable interrupts #defines above!
// This limitation exists because of memory saving assumptions that the
// interrupt is on port D.
XYZ_SingleEncoder::XYZ_SingleEncoder(int pin)
{
  pinMode(pin, INPUT_PULLUP);
  enableInterrupt(pin, encoder_tick, CHANGE);

  reset();
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/
void XYZ_SingleEncoder::reset() {
  set_encoder(0);
}

// Safely set the wheel encoder tick value
void XYZ_SingleEncoder::set_encoder(int32_t ticks) {
  noInterrupts();
  g_encoder_counter = ticks;
  interrupts();  
}

// Get the wheel encoder tick value safely
int32_t XYZ_SingleEncoder::get_encoder_count() {
  uint32_t count;
  noInterrupts();
  count = g_encoder_counter;
  interrupts();
  return count;
}
