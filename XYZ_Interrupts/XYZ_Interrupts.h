/* XYZ_Interrupts
 by Ted Meyers (4/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.

 Note: this file can only be included in one source file!  Include
 it in a cpp file works best (not a .h file)

 Also, you need to include this file like this:
   #define LIBCALL_ENABLEINTERRUPT
   #include "XYZ_Interrupts.h"
 Do this with the #define in all but one place (probably your .ino file).

 This file defines the ports not used for interrupts (in order to
 optimize the code).
 
 It also includes the EnableInterrupts.h file.
*/

#ifndef _XYZ_INTERRUPTS_H__
#define _XYZ_INTERRUPTS_H__

// --------------------------------------------------------------------
// Enable Interrupts defines

// This causes the pin number to be stored in: arduinoInterruptedPin
// #define EI_ARDUINO_INTERRUPTED_PIN

// This enables some speed optimizations:
// #define NEEDFORSPEED

// Pins -----------------
// External Interrupts:
//   INT0 == Pin 2
//   INT1 == Pin 3
//
// PinChangeInterrupts:
//   PORTB == Pins 8-13
//   PORTC == Pins A0-A5 (analog)
//   PORTD == Pins 2-7
//
// Save memory by assuming that only pins on port D are used (pins 0-7)
// EI_NOTEXTERNAL assumes that pins 2 and 3 are not used
// USE CARE!  This is non-portable!
//
#define EI_NOTPORTB
#define EI_NOTPORTC
//#define EI_NOTPORTD

//#define EI_NOTINT0
//#define EI_NOTINT1
//#define EI_NOTEXTERNAL
//#define EI_NOTPINCHANGE
//
// Interrupts library
#include <EnableInterrupt.h>
// --------------------------------------------------------------------

#endif
