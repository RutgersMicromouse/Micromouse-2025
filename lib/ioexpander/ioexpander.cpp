#include "ioexpander.h"
#include <sidedist.h>

bool isSaving() {
    return bitRead(ioExpanderRead(), 1);
}

bool isLoad(){
    return bitRead(ioExpanderRead(), 2);
}

bool isSpeedrun(){
    return bitRead(ioExpanderRead(), 3);
}

bool isLabyrinth(){
    return bitRead(ioExpanderRead(), 4);
}

bool isFirefighter(){
    return bitRead(ioExpanderRead(), 5);
}

bool rightWall() {
    double dist = getRightSideDist();
    
    // If the sensor timed out OR the distance is greater than the 130mm threshold
    if (dist == -1 || dist > 10) {
        return false; // It's an opening
    }
    return true; // Wall detected
}

bool leftWall() {
    double dist = getLeftSideDist();
    
    // If the sensor timed out OR the distance is greater than the 130mm threshold
    if (dist == -1 || dist > 10) {
        return false; // It's an opening
    }
    return true; // Wall detected
}


byte ioExpanderRead() {
    byte readValue = 0xFF;

    Wire.requestFrom(IO_expander_addr, 1);
  
    if (Wire.available()) {
        readValue = Wire.read();
        readValue = readValue ^ 0xFF;  // xor so that 1 is on and 0 is off
        /*
                // Print the 8 bits of the byte
        for (int i = 7; i >= 0; i--) {          // Start from the most significant bit (MSB)
                }
                */

    } else {
          }
    return readValue;
}
