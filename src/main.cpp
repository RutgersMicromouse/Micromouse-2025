#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "../lib/IMU/imu.h"
#include "../lib/distanceSensors/distanceSensors.h"
#include "../lib/Motors/motors.h"
#include "../lib/PIDMovement/pidRotate.h"
#include "../lib/PIDMovement/pidStraight.h"

void setup() {
  Serial.begin(112500);
  
  Wire.begin(1, 2);

  Serial.println("finding IMU in main");
  setupIMU();
  setupDistanceSensors();
  setupMotors();
  delay(1000);

  while(1) {
    straight('N', 160);

  }

} 

void loop() {


}


