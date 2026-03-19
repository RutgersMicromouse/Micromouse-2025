#include "save.h"
#include <Arduino.h>
#include <Preferences.h>

Preferences prefs;

void initialize_eeprom()
{
    /* set eeprom switch to input */
    pinMode(eepromswitch, INPUT);
}

void save_memory(void* source, uint32_t address,uint32_t size)
{
    prefs.begin("maze", false);
    prefs.putBytes("maze", source, size);
    prefs.end();
    Serial.println("Saved memory to EEPROM");
}

void load_memory(void* destination, uint32_t address,uint32_t size)
{
    prefs.begin("maze", false);
    prefs.getBytes("maze", destination, size);
    prefs.end();
    Serial.println("Loaded memory from EEPROM");
}