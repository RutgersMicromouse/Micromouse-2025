#include "frontdist.h"

bool startUpcheck() {
    if (front() == -1) {
        Serial.println("Front ToF read timeout!");
    }

    bool started = false;

    while (!started) {
        delay(50);
        if ( abs(front()) <= 20) {
            started = true;
        }
        Serial.println("Distance: " + String(front()) + "mm. Waiting for wall to start...");
    }
    Serial.println("Wall detected in front, starting up!");
    delay(500);
    return true; // returns true if there is a wall within 150mm in front
}