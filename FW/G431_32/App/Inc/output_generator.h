#ifndef OUTPUT_GENERATOR_H
#define OUTPUT_GENERATOR_H
#include "defines.h"
#include <stdint.h>

void output_generator_init(uint16_t _factor);
void read_adcs_output();
void calculate_outputs();
void update_out_pins();
uint8_t output_generator_set_cal_coeff(int32_t* _cal_coeff);

#endif //OUTPUT_GENERATOR_H