#include <Arduino.h>

#include "motors.h"
#include "camera.h"


void setup() {
  
  Serial.begin(9600);
  motorSetup();
  // cameraSetup();
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
  
  setLeftPWM(400);
  


}

 



void loop() {
  
    // imageToAscii();
    delay(500); // 1 second delay


}