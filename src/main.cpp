#include <Arduino.h>
#include "../lib/Motors/Motors.h"
#include "../lib/DistanceSensors/distanceSensors.h"
#include "../lib/IMU/IMU.h"
#include "../lib/pidRotation/pidRotate.h"

void setup() {
  Serial.begin(9600);
  setupMotors();
  setupDistanceSensors();
  setupIMU();
  

}

void loop() {
  for(unsigned int i = 0; i < 255; i += 63){
    moveLeftMotor(i,'N');
    moveRightMotor(i,'N');
    delay(1000);
  }
  moveLeftMotor(0,'N');
  moveRightMotor(0,'N');
  delay(1000);
  // Do nothing, motors keep running

  /* Distance Sensor Test
  if(checkRightWall()) {
    Serial.println("right wall deteected");
  } else {
    Serial.println("no detection");
  } 
  */

  /* TOF Testing 
  int16_t distance = checkFrontWall();
  if(distance < 100) {
    Serial.println("Front wall detected");
  } else {
    Serial.println(distance);
  }

  */
  
  /* IMU Testing
  //Serial.println(angle());
  */
  
  


  
}
