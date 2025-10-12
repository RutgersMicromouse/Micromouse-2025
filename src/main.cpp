#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "../lib/IMU/imu.h"
#include "../lib/distanceSensors/distanceSensors.h"
#include "../lib/Motors/motors.h"
#include "../lib/PIDMovement/pidRotate.h"
#include "../lib/PIDMovement/pidStraight.h"
#include "../lib/pathfinding/floodfill.h"

#define LOAD 5  //LOAD
#define SAVE 6  //SAVE

void setup() {
  Serial.begin(112500);
  Wire.begin(1, 2);

  // Initialize switches as input with pullups

  delay(3000);
  setupIMU();
  setupDistanceSensors();
  setupMotors();
  setupSwitches();

  delay(1000);

  bool load = digitalRead(LOAD) == LOW;
  bool save = digitalRead(SAVE) == LOW;


  /*
    Work flow: 
      - Both switches high to save the maze
      - Middle switch being low, right switch being high means loading
      - Right and middle switch off is equal to turning everything off
  */

  if (load && !save) {
      Serial.println("Resetting maze...");
      initialize_maze(15,15,true);  // full reset
  } else if (!load && save) {
      Serial.println("Loading saved maze...");
      loadMaze();
  } else if (load && save) {
      Serial.println("Saving maze...");
      initialize_maze(15,15,true);
      floodfill();
      saveMaze();
  } else {
      Serial.println("Starting fresh without reset...");
      straight('N', 60);
      initialize_maze(15,15,true);
      floodfill();
      
  }

  //turnTo('E');



  //   initialize_maze(1,1,false);
  // floodfill();

  
}

void loop() {
  
}
