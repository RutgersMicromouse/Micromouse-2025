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

  delay(8000); // Delay to open serial monitor

  imuSetup();
  tofSetup();
  delay(50);
  motorSetup();


  // Serial.println("Scanning I2C bus...");
  
  // for (byte address = 1; address < 127; address++) {
  //   Wire.beginTransmission(address);
  //   byte error = Wire.endTransmission();
  //   if (error == 0) {
  //     Serial.print("Found I2C device at 0x");
  //     Serial.println(address, HEX);
  //   }
  // }
  // Serial.println("Scan done.");

  
  // // // Switch options:
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

  // // Default
  Serial.println("Exploration mode");
  initialize();
  runMaze('c');

}

void loop() {
  delay(2000);
  Serial.println("Hello main loop!");
  // Serial.println(API::wallFront());
  // Serial.print(API::wallLeft());
  // Serial.print(" N ");
  // Serial.println(API::wallRight());

  // Serial.println(front());

}
