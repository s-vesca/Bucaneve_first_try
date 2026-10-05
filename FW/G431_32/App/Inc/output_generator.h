#ifndef OUTPUT_GENERATOR_H
#define OUTPUT_GENERATOR_H
#include "defines.h"
#include <stdint.h>

void output_generator_init(uint16_t _factor);
void read_adcs_output();
void calculate_outputs();
void output_generator_set_cal_coeff(cal_coeff_t _cal_coeff);

#endif //OUTPUT_GENERATOR_H