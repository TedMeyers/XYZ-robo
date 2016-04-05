/* BNO-055 Test Code
 by Ted Meyers  (4/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.
*/
#include <Wire.h>
#include <XYZ_BNO055.h>

XYZ_BNO055 bno;

uint32_t out_time = 0;  // used to control display output rate
int myLed = 13;         // Pin definitions



void setup(void) 
{
  Wire.begin();  
  //TWBR = 12;  // 400 kbit/sec I2C speed for Pro Mini
  // Setup for Master mode, pins 16/17, external pullups, 400kHz for Teensy 3.1
  //  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);

  // Setup serial connection
  delay(2000);
  Serial.begin(115200);
  while (!Serial);  // For testing using a Leonardo
  Serial.println("BNO055 Sensor Test"); Serial.println("");
  
  // Initialise the sensor
  while (!bno.setup(BNO055_ADDRESS_B))
  {
    Serial.print("No BNO055 found");
    delay(1000);
  }
  Serial.print("BNO055 found");
  bno.setMode(XYZ_BNO055::NDOF);

  calibrate();

  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);
  delay(1000);
}

void loop(void) 
{
  uint32_t delt_t = millis() - out_time;

 if (delt_t > 100) { // update LCD once per half-second independent of read rate  
    uint8_t stats[4];
    uint8_t errstat = bno.readCalibration(stats);
    if (errstat == 0x01) {
      uint8_t syserr = bno.readSysErr();
      Serial.print("ERROR: ");  Serial.println(syserr);
      return;
    }

    float ypr[3];
    bno.readYPR(ypr);

    Serial.print("YPR: ");
    Serial.print(ypr[0], 2); Serial.print(", ");
    Serial.print(ypr[1], 2); Serial.print(", ");
    Serial.print(ypr[2], 2); Serial.print("  ");
    Serial.print("CAL: ");
    Serial.print(stats[0]); Serial.print(", ");
    Serial.print(stats[1]); Serial.print(", ");
    Serial.print(stats[2]); Serial.print(", ");
    Serial.print(stats[3]); Serial.print("  ");
    Serial.println();
     
    digitalWrite(myLed, !digitalRead(myLed));
    out_time = millis();
  }
}

void calibrate() {
  Serial.println("Cal: No=0, full=3");

  uint8_t stats[4];
  for (int i=0; i<60; i++) {
    bno.readCalibration(stats);
    if ((stats[0]==3) && (stats[1]==3) && (stats[3]==3)) {
      break;
    }
    Serial.print("  Sys "); Serial.print(stats[0]);
    Serial.print("  Gyr "); Serial.print(stats[1]);
    Serial.print("  Acc "); Serial.print(stats[2]);
    Serial.print("  Mag "); Serial.println(stats[3]);
    delay(500);
  }
}
