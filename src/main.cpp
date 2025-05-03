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
  Wire.setClock(20000); // use 400 kHz I2C
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); // turns the builtin LED on (active low)

  imuSetup();
  tofSetup();
  delay(500);
  motorSetup();
  Serial.println("Hello setup!");

  // Switch options:  
  // if(isFirefighter()) {
  //   Serial.println("Firefighter mode");
  //   init_GPIO();
  //   firefighterSetup();
  //   firefighterLoop();
  // }

  pidForward(180);
  // pidForward(180);
  // pidForward(180);

  // turnTo(270);

  // pidForward(180);
  // pidForward(180);
  
  // turnTo(180);
  
  // pidForward(180);
  // pidForward(180);
  
  // turnTo(90);
  
  // pidForward(180);
  
  // turnTo(0);
  
  // pidForward(180);
  
}

void loop() {
  delay(2000);
  Serial.println("Hello main loop!");
  Serial.print(encLeft.read());
  Serial.print(" N ");
  Serial.println(encRight.read());

  // Serial.println(front());


}
