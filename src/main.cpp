#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "../lib/IMU/imu.h"
#include "../lib/distanceSensors/distanceSensors.h"
#include "../lib/Motors/motors.h"
#include "../lib/PIDMovement/pidRotate.h"
#include "../lib/PIDMovement/pidStraight.h"
#include "../lib/pathfinding/API.h"
// #include "../lib/pathfinding/Astar.h"
#include "../lib/pathfinding/floodfill.h"

int bl = 160;
void setup() {
  Serial.begin(112500);
  
  Wire.begin(1, 2);

  Serial.println("finding IMU in main");
  setupIMU();
  setupDistanceSensors();
  setupMotors();
  delay(1000);

  
//  straight('N', bl);
//  delay(100);

 initialize_maze(16, 16, true);
 floodfill();
 initialize_maze(1, 1, false);
 floodfill();


} 

void loop() {


//  Serial.printf("%d\n", checkFrontWall());


}


