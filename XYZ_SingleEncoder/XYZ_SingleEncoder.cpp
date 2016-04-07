/* XYZ Wheel Encoder Library
 by Ted Meyers (3/19/2016)
 https://github.com/TedMeyers/XYZ-robo

 license: Colaware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.
*/
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include "XYZ_SingleEncoder.h"

// Save momory by assuming that only pins on port B are used (pins 8-13)
#define EI_NOTPORTC
#define EI_NOTPORTD
#define EI_NOTEXTERNAL
//#define LIBCALL_ENABLEINTERRUPT
#include <EnableInterrupt.h>

volatile uint32_t g_encoder_counter;    // The number of encoder ticks

// Called when the wheel encoder pin changes
void encoder_tick() {
  g_encoder_counter++;
}

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
// IMPORTANT: pin can only be any PORT B pin (8-12)
// This limitation exists because of memory saving assumptions that the
// interrupt is on port B.
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
void XYZ_SingleEncoder::set_encoder(uint32_t ticks) {
  noInterrupts();
  g_encoder_counter = ticks;
  interrupts();  
}

// Get the wheel encoder tick value safely
uint32_t XYZ_SingleEncoder::get_encoder_count() {
  uint32_t count;
  noInterrupts();
  count = g_encoder_counter;
  interrupts();
  return count;
}
