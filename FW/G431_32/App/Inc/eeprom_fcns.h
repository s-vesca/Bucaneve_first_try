#ifndef EEPROM_FCNS_H
#define EEPROM_FCNS_H

#include <stdint.h>

void eeprom_unlock();

uint8_t eeprom_erase();

uint8_t eeprom_read_params(uint32_t* _eeprom_params, uint8_t _n_params, uint16_t _start_addr);

uint8_t eeprom_write_params(uint32_t* _eeprom_params, uint8_t _n_params, uint16_t _start_addr);

#endif //EEPROM_FCNS_H