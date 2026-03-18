#include "imu.h"

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setupIMU() {

  Serial.println("finding IMU in method");

  if (!bno.begin()) {
    Serial.println("No BNO055 detected");
   
  } 

  //bno.begin();

  Serial.println("IMU found!");
  
}

double getAngle() {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  return orientationData.orientation.x;
}
