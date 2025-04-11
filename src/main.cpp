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
