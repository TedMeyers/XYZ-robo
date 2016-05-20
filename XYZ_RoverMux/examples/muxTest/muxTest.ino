/* XYZ Rover Mux (multiplixer) Library
 by Ted Meyers (4/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.

 This sketch tests the rover mux.  Open a terminal window
 and set to 115200.  The Rx values will be printed out.

 The mux really has nothing to do with wheel encoders,
 but I added them in to test that interrupts work with them.
 You can take them out by removing the #define USE_WHEEL_ENCODERS line.

 This code assumes quadrature wheel encoders, you can use
 single encoders, but you will have to make a few changes.
 (see WheelEncoder library)

  Options (see .h file): USE_DEFAULTS_FLAG, ENABLE_MUX_AUX 

 Uses XYZ_Interrupt, which uses the EnableInterrupts library.
*/

#include <XYZ_RoverMux.h>
#include <XYZ_Interrupts.h>
#include <XYZ_LED.h>

#define USE_WHEEL_ENCODERS
#ifdef USE_WHEEL_ENCODERS
  #include <XYZ_WheelEncoder.h>
  #define ENC_A 6
  #define ENC_B 7
  XYZ_WheelEncoder myWheelEncoder;
#endif

#define TH_IN_PIN 3
#define ST_IN_PIN 4
#define AUX_IN_PIN 5

#define TH_OUT 9
#define ST_OUT 10

#define LED_PIN 13

XYZ_RoverMux myMux;
XYZ_LED myLed;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println(F("XYZ_RoverMux Test begin..."));

  myLed.setup(LED_PIN);
  myMux.getServos()->setup(TH_OUT, ST_OUT);
  myMux.getRx()->setup(TH_IN_PIN, ST_IN_PIN, AUX_IN_PIN);

  #ifdef USE_WHEEL_ENCODERS
    myWheelEncoder.setup(ENC_A, ENC_B); 
  #endif

  myMux.reset();
  myMux.updateServos(10, -20);
}

int cnt = 0;
void loop() {
  myMux.update();

  if ((cnt % 5000) == 0) {
    int8_t mode = myMux.getMuxMode();
    uint16_t vld = myMux.getRx()->checkSignal();
    uint16_t mth = myMux.getRx()->getThrottleMicros();
    uint16_t mst = myMux.getRx()->getSteeringMicros();
    uint16_t maux = myMux.getRx()->getAuxiliaryMicros();
    uint16_t th = myMux.getServos()->getThrottleMicros();
    uint16_t st = myMux.getServos()->getSteeringMicros();

    #ifdef USE_WHEEL_ENCODERS
      long encCnt = myWheelEncoder.getEncoderCount();
    #endif

    myLed.set(mode==-2);
    myLed.update();

    #ifdef USE_WHEEL_ENCODERS
      Serial.print("Enc: ");
      Serial.print(encCnt);
    #endif
    Serial.print(F("   TH: "));
    Serial.print(th);
    Serial.print(F(" / "));
    Serial.print(mth);
    Serial.print(F(" ST: "));
    Serial.print(st);
    Serial.print(F(" / "));
    Serial.print(mst);
    Serial.print(F(" AUX: "));
    Serial.print(maux);
    Serial.print(F(" VALID: "));
    Serial.print(vld);
    Serial.print(F("  MODE: ")); 
    Serial.println(mode);
  }
  cnt++;
}