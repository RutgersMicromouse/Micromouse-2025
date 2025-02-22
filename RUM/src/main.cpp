#include <Arduino.h>
#include "ioexpander.h"
#include "imu.h"
#include "frontdist.h"
#include "motors.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  imuSetup();
  tofSetup();
  motorSetup();

}

void loop() {
  delay(500);
  // put your main code here, to run repeatedly:
  Serial.println(angle());
  setLeftPWM(800);
  Serial.println(encLeft.read());
}
