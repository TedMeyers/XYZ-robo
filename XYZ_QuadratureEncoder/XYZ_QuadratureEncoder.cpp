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

#include "XYZ_QuadratureEncoder.h"

// Save momory by assuming that only pins on port B are used (pins 8-13)
#define EI_NOTPORTC
#define EI_NOTPORTD
#define EI_NOTEXTERNAL
//#define LIBCALL_ENABLEINTERRUPT
#include <EnableInterrupt.h>

uint8_t g_enc_highA;
uint8_t g_enc_highB;

volatile uint32_t g_encoder_counter;    // The number of encoder ticks

// Called when the encoder pin changes
void encoder_tickA() {
  g_enc_highB = READ_ENC_B;
  if (g_enc_highA) {
    g_enc_highA = false;
    if (g_enc_highB) g_encoder_counter--;
    else g_encoder_counter++;
  } else {
    g_enc_highA = true;
    if (g_enc_highB) g_encoder_counter++;
    else g_encoder_counter--;
  }
}

// Called when the encoder pin changes
void encoder_tickB() {
  g_enc_highA = READ_ENC_A;
  if (g_enc_highB) {
    g_enc_highB = false;
    if (g_enc_highA) g_encoder_counter++;
    else g_encoder_counter--;
  } else {
    g_enc_highB = true;
    if (g_enc_highA) g_encoder_counter--;
    else g_encoder_counter++;
  }
}

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
XYZ_QuadratureEncoder::XYZ_QuadratureEncoder() {
  pinMode(ENC_PIN_A, INPUT_PULLUP);
  pinMode(ENC_PIN_B, INPUT_PULLUP);

  g_enc_highA = READ_ENC_A;
  g_enc_highB = READ_ENC_B;

  enableInterrupt(ENC_PIN_A, encoder_tickA, CHANGE);
  enableInterrupt(ENC_PIN_B, encoder_tickB, CHANGE);

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
void XYZ_QuadratureEncoder::set_encoder(uint32_t ticks) {
  noInterrupts();
  g_encoder_counter = ticks;
  interrupts();  
}

// Get the wheel encoder tick value safely
uint32_t XYZ_QuadratureEncoder::get_encoder_count() {
  uint32_t count;
  noInterrupts();
  count = g_encoder_counter;
  interrupts();
  return count;
}

