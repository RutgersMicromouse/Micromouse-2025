#include <Arduino.h>
#include "../lib/Motors/Motors.h"
#include "../lib/DistanceSensors/distanceSensors.h"
#include "../lib/IMU/IMU.h"
#include "../lib/pidRotation/pidRotate.h"
#include "../lib/pidRotation/pidStraight.h"
#include "../pathfinding/floodfill.h"

void setup() {
  Serial.begin(9600);
  delay(1000);
  setupMotors();
  setupDistanceSensors();
  setupIMU();
  Wire.setClock(40000);
  delay(500);
  // while(1){
  //   straight('N', 160);
  // }
  // turnTo('W');
 
  straight('N', 80);
  delay(1000);
  initialize_maze(15,15,true);
  floodfill();
  initialize_maze(1,1,false);
  floodfill();
  // fprintf(stderr,"floodfill - nodes expanded: %d\n",floodfill_expand);
  //Astar(1,5);

}

void loop() {

  // straight('N', 65);
  // delayMicroseconds(10);
  // straight('N', 160);
  // delayMicroseconds(10);
  // straight('N', 160);
  // delayMicroseconds(10);
  // turnTo('W');
  // delayMicroseconds(10);
  // straight('W', 160);
  // delayMicroseconds(10);
  // turnTo('S');
  // delayMicroseconds(10);
  // straight('S', 160);
  // delayMicroseconds(10);
  // straight('S', 160);
  // delayMicroseconds(10);
  // turnTo('W');
  // delayMicroseconds(10);
  // straight('W', 160);

  // // straight('N', 160);
  // // straight('N', 160);
  // // turnTo('W');
  // // turnTo('S');
  // // straight('S', 160);
  // // straight('S', 160);
  // // straight('S', 160);
  // // straight('S', 160);
  

  // while(1);

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