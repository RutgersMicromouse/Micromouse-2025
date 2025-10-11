#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "../lib/IMU/imu.h"
#include "../lib/distanceSensors/distanceSensors.h"
#include "../lib/Motors/motors.h"
#include "../lib/PIDMovement/pidRotate.h"
#include "../lib/PIDMovement/pidStraight.h"
#include "../lib/pathfinding/floodfill.h"

#define MODE 5

void setup() {
  Serial.begin(112500);
  Wire.begin(1, 2);

  delay(3000);
  setupIMU();
  setupDistanceSensors();
  setupMotors();
  //pinMode(MODE, INPUT_PULLUP);
  delay(1000);

  // if(digitalRead(MODE) == 1) {
  //   Serial.printf("Maze Mode");
  //   \
  // } else {
  //   Serial.printf("Labrinyth Mode");
  //   straight('N', 45);
  //   delay(500);
  //   initialize_maze(32,32,true);
  //   floodfill();
  //   initialize_maze(1,1,false);
  //   floodfill();

  // }

    straight('N', 60);
    delay(1000);

    initialize_maze(15,15,true);
    floodfill();
    initialize_maze(1,1,false);
    floodfill();

 




}

void loop() {

  //Serial.println(wallBrake());
  //Serial.printf("Right Encoder: %d  Left Encoder: %d\n", getRightEncoder(), getLeftEncoder());

   

}


