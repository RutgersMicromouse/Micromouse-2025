#include <Arduino.h>
#include "../lib/Motors/Motors.h"
#include "../lib/DistanceSensors/distanceSensors.h"
#include "../lib/IMU/IMU.h"
#include "../lib/pidRotation/pidRotate.h"
#include "../lib/pidRotation/pidStraight.h"

void setup() {
  Serial.begin(9600);
  delay(1000);
  setupMotors();
  setupDistanceSensors();
  setupIMU();
  Wire.setClock(40000);
}

void loop() {
  // straight('N');
  // straight('N');
  // straight('N', 160);
  straight('N', 160);
  delay(1000);
  turnTo('E');
  
  delay(1000);

  while(1);

  // for(unsigned int i = 0; i < 255; i += 63){
  //   moveLeftMotor(i);
  //   moveRightMotor(i);
  //   delay(1000);
  // }
  // moveLeftMotor(0);
  // moveRightMotor(0);

  // delay(1000);
  // if(checkRightWall()) {
  //   Serial.println("right wall deteected");
  // } else {
  //   Serial.println("no detection");
  // } 

  // int16_t distance = checkFrontWall();
  // if(distance < 100) {
  //   Serial.println("Front wall detected");
  // } else {
  //   Serial.println(distance);
  // }
  //turnTo('E');

    
 // Serial.println(angle());

 
  
  
  


  
}
