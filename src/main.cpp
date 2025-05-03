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
  Wire.setClock(20000); // use 400 kHz I2C
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); // turns the builtin LED on (active low)

  imuSetup();
  tofSetup();
  delay(50);
  motorSetup();
<<<<<<< HEAD
  Serial.println("Hello setup!");

  // Switch options:  
  // if(isFirefighter()) {
  //   Serial.println("Firefighter mode");
  //   init_GPIO();
  //   firefighterSetup();
  //   firefighterLoop();
  // }
=======
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
>>>>>>> 1bc8325012e5c1f686e9c83a921d1d748c516b1f

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
<<<<<<< HEAD
  delay(2000);
  Serial.println("Hello main loop!");
  Serial.print(encLeft.read());
  Serial.print(" N ");
  Serial.println(encRight.read());

  // Serial.println(front());


=======
  Serial.println("Big Chungus");
  delay(69420);
>>>>>>> 1bc8325012e5c1f686e9c83a921d1d748c516b1f
}
