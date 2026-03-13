#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "../lib/IMU/imu.h"
#include "../lib/distanceSensors/distanceSensors.h"
#include "../lib/Motors/motors.h"
#include "../lib/PIDMovement/pidRotate.h"
#include "../lib/PIDMovement/pidStraight.h"
#include "../lib/pathfinding/API.h"
#include "../lib/pathfinding/Astar.h"
#include "../lib/pathfinding/floodfill.h"

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
  straight('N', 80);
  delay(500);
 
  initialize_maze(5, 5, true);
  floodfill();
  initialize_maze(1, 1, false);
  floodfill();
  delay(1000);

  Astar(5, 5);
  
}

void loop() {

  //Serial.println(checkFrontWall());

}