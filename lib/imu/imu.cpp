#include "imu.h"

Adafruit_BNO055 bno;

void imuSetup() {
  bno = Adafruit_BNO055(55, 0x28);
  if (!bno.begin()) { 
    Serial.println("No BNO055 detected");
        while(1);
  }
    Serial.println("BNO055 detected");
  bno.setExtCrystalUse(true); 
}

double angle() {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  
  return orientationData.orientation.x;
}