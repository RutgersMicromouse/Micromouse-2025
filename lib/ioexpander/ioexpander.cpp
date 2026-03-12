#include "ioexpander.h"

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

bool rightWall(){
    return bitRead(ioExpanderRead(), 6);
}
bool leftWall(){
    return bitRead(ioExpanderRead(), 7);
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
