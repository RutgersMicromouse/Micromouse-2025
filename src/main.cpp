#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "../lib/IMU/imu.h"
#include "../lib/distanceSensors/distanceSensors.h"
#include "../lib/Motors/motors.h"
#include "../lib/PIDMovement/pidRotate.h"
#include "../lib/PIDMovement/pidStraight.h"
#include "../lib/pathfinding/floodfill.h"


//CURRENT PROBLEMS:
 //Veering left
  //Turns ARE WOBBLY WIGGLY WOOBBLY WIGGLY
void setup() {
  Serial.begin(112500);
  delay(1000);
  Wire.begin(1, 2);
  setupIMU();
  setupDistanceSensors();
  setupMotors();
  delay(1000);

  straight('N', 40);
  delay(1500);
  initialize_maze(15,15,true);
  floodfill();
  initialize_maze(1,1,false);
  floodfill();


}

void loop() {

  //Serial.println(getAngle());
 

 // Serial.printf("Right Encoder: %d  Left Encoder: %d\n", getRightEncoder(), getLeftEncoder());

   

}


