#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "imu.h"
#include "distanceSensors.h"
#include "motors.h"
#include "pidRotate.h"
#include "pidStraight.h"
#include "API.h"
#include "Astar.h"
#include "floodfill.h"
#include "labyrinth.h"

int bl = 180;
const int HAND_THRESHOLD = 120;   // mm distance for hand detection
const int START_DELAY = 1500;     // 1.5 second delay
int labPin = 6;

bool armed = false;
unsigned long startTime = 0;

void setup() {
  Serial.begin(112500);
  pinMode(labPin, INPUT_PULLUP);
  Wire.begin(1, 2);

  delay(1000);

  setupDistanceSensors();
  Serial.println("finding IMU in main");
  Serial.println("Waiting for hand to start...");

  //Waiting for hand to come in to start the micromouse
  while(!armed) {
    
    int16_t frontDistance = checkFrontWall();
    if(frontDistance < HAND_THRESHOLD && frontDistance > 0) {
      armed = true;
      startTime = millis();
    }

    delay(50);
  }

  while(millis() - startTime < START_DELAY) {
    Serial.printf("Starting in %d ms\n", START_DELAY - (millis() - startTime));
    delay(100);
  }

  setupIMU();
  setupDistanceSensors();
  setupMotors();

  if(!digitalRead(labPin)) {
    Serial.println("LAYBRINTH");

    labyrinth();
  } else {
    Serial.println("MAZE");

    delay(1000);
    straight('N', 90);
    initialize_maze(15, 15, true); //Set to 15 15 for the maze center, 31 and 31
    floodfill();
    stopMotors();
    initialize_maze(1, 1, false);
    floodfill();
    delay(250);
  }
  
  // turnTo('E');
  // delay(100);
  // turnTo('S');
  // delay(100);
  // turnTo('W');
  // delay(100);
  // turnTo('N');
  
}

void loop() {

 // Serial.printf("%d \t %d\n", getLeftEncoder(), getRightEncoder());

}