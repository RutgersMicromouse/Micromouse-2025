#include <Arduino.h>
#include "ioexpander.h"
#include "imu.h"
#include "frontdist.h"
#include "motors.h"
#include "pidstraight.h"
#include "pidrotate.h"
#include "firefighter.h"
#include "Flood.h"


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
  delay(50);
  motorSetup();
  delay(50);
  initialize();
  Serial.println("Hello setup!");

  // Switch options:  
  if(isSpeedrun()) {
    Serial.println("Lightning McQueen mode");
    speedrun();
    Serial.println("Kachow!");
  }

  else if(isLabyrinth()) {
    Serial.println("Labyrinth mode");
    //TODO: Labyrinth Setup and loop
  }

  else if(isFirefighter()) {
    Serial.println("Firefighter mode");
    init_GPIO();
    firefighterSetup();
    firefighterLoop();
  }
  else {
    Serial.println("Exploration mode");
    runMaze('c');
  }


}

void loop() {
  Serial.println("Big Chungus");
  delay(69420);
}
