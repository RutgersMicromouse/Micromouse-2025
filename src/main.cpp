#include <Arduino.h>
#include "ioexpander.h"
#include "imu.h"
#include "frontdist.h"
#include "motors.h"
#include "pidstraight.h"
#include "pidrotate.h"
#include "firefighter.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(50000); // use 400 kHz I2C
  // Slowed 12C clock speed for the IMU to work
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); // turns the builtin LED on (active low)

  imuSetup();
  tofSetup();
  delay(500);
  motorSetup();
  delay(5000);
  Serial.println("Hello setup!");

  // Move to center of cell off of back wall alignment
  turnTo(180);

  // Switch options:  
  if(isFirefighter()) {
    Serial.println("Firefighter mode");
    init_GPIO();
    firefighterSetup();
    firefighterLoop();
  }


}

void loop() {
  delay(2000);
  //Serial.println("Hello main loop!");
  Serial.print(encLeft.read());
  Serial.print(" N ");
  Serial.println(encRight.read());


}
