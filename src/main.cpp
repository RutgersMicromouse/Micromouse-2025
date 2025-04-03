#include <Arduino.h>
#include "ioexpander.h"
#include "imu.h"
#include "frontdist.h"
#include "motors.h"
#include "pidstraight.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  imuSetup();
  tofSetup();
  motorSetup();
  delay(5000);
  Serial.println("Hello setup!");
  pidForward(10);
}

void loop() {
  delay(2000);
  Serial.println("Hello main loop!");
}
