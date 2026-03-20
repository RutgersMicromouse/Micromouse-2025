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

int bl = 160;
const int HAND_THRESHOLD = 120;   // mm distance for hand detection
const int START_DELAY = 1500;     // 1.5 second delay

bool armed = false;
unsigned long startTime = 0;

void setup() {
  Serial.begin(112500);
  
  Wire.begin(1, 2);

  Serial.println("finding IMU in main");
  setupIMU();
  setupDistanceSensors();
  setupMotors();

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

  setupDistanceSensors();

  //HALF BLOCK
  straight('N', 80);
  delay(500);
 
  initialize_maze(15, 15, true); //Set to 15 15 for the maze center, 31 and 31
  floodfill();
  initialize_maze(1, 1, false);
  floodfill();
  delay(250);

  turnTo('N');
  delay(100);
  moveRightMotor(-50);
  moveLeftMotor(-50);
  delay(1000);
  stopMotors();
  delay(1000);

  straight('N', 80);
  Serial.println("Running A Star");
  Astar(16, 16); //16 by 16 for the center
  
}

void loop() {

  //Serial.println(checkFrontWall());

}