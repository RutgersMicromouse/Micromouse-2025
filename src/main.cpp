#include <Arduino.h>

#include "motors.h"
#include "camera.h"


void setup() {
  
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  motorSetup();
  cameraSetup();
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW); // Built-in LED is active-LOW

  


}

 



void loop() {
  followEdge();

    // imageToAscii();
    // delay(500);
    // digitalWrite(LED_BUILTIN, HIGH); 
    // delay(500);
    // digitalWrite(LED_BUILTIN, LOW); 


}