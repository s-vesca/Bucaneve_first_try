#ifndef OUTPUT_GENERATOR_H
#define OUTPUT_GENERATOR_H
#include "defines.h"
#include <stdint.h>

void output_generator_init(uint16_t _factor);
void __attribute__((section(".ccm_code")))read_adcs_output();
void __attribute__((section(".ccm_code")))calculate_outputs();
void __attribute__((section(".ccm_code")))update_out_pins();
void output_generator_set_cal_coeff(cal_coeff_t _cal_coeff);

#endif //OUTPUT_GENERATOR_H