#include "imu.h"

Adafruit_BNO055 bno;

void imuSetup() {
  bno = Adafruit_BNO055(55, 0x28);
  if (!bno.begin()) { 
        while(1);
  }
  bno.setExtCrystalUse(true);
}

double angle() {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  
  return orientationData.orientation.x;
}