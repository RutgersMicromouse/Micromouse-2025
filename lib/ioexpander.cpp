#include "ioexpander.hpp"

bool is_saving(){
  return true;
}



bool is_load(){
  return false;
}



bool is_speedmode(){
  byte readValue = io_expander_read()
  return readValue;
}



bool is_labyrinth(){
  return readValue;
}



bool is_firefighter(){
return readValue;
}



bool left_wall(){
    return readValue;
}



bool right_wall(){
    return readValue;
}



byte io_expander_read() {
    byte readValue = 0xFF;
  
    Wire.requestFrom(IO_expander_addr, 1);
  
    if (Wire.available()) {
      readValue = Wire.read();
      readValue = readValue ^ 0xFF;  // xor so that 1 is on and 0 is off
  
      Serial.print("IO expander: ");
      // Print the 8 bits of the byte
      for (int i = 7; i >= 0; i--) {          // Start from the most significant bit (MSB)
        Serial.print(bitRead(readValue, i));  // Extract and print each bit
      }
      Serial.println("");
  
    } else {
      Serial.println("IO Expander read failed");
    }
    return readValue;
}
  