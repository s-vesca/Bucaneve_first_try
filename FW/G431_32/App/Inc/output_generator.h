#ifndef OUTPUT_GENERATOR_H
#define OUTPUT_GENERATOR_H
#include <stdint.h>

void output_generator_init(uint16_t _factor);
__attribute__((section(".ccm_code")))
void read_adcs_output();
__attribute__((section(".ccm_code")))
void calculate_outputs();

#endif //OUTPUT_GENERATOR_H