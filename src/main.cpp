#include <Arduino.h>
#include "ioexpander.h"
#include "imu.h"
#include "frontdist.h"
#include "motors.h"
#include "pidstraight.h"
#include "pidrotate.h"

#include "Flood.h"
#include "labyrinth.h"
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
  delay(50);
  motorSetup();
  
  // Switch options:
  if(isSpeedrun()) {
    Serial.println("Lightning McQueen mode");
    initialize(); // load switch should also be on
    speedrun();
    return;
  }
  if(isLabyrinth()) {
    Serial.println("Labyrinth mode");
    labyrinthLoop();
    return;    
  }
  if(isFirefighter()) {
    Serial.println("Firefighter mode");
    init_GPIO();
    firefighterSetup();
    firefighterLoop();
    return;
  } 

  // Default
  Serial.println("Exploration mode");
  initialize();
  runMaze('c');
}

void loop() {
  delay(2000);
  Serial.println("Hello main loop!");
  Serial.print(encLeft.read());
  Serial.print(" N ");
  Serial.println(encRight.read());

  // Serial.println(front());

}
