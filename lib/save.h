#ifndef SAVE_H
#define SAVE_H
#define eepromswitch 6

#include <stdint.h>
void initialize_eeprom();
void save_memory(void* source, uint32_t address,uint32_t size);
void load_memory(void* destination, uint32_t address,uint32_t size);

#endif // SAVE_H
