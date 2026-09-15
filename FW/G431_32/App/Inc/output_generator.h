#ifndef OUTPUT_GENERATOR_H
#define OUTPUT_GENERATOR_H
#include <stdint.h>

void output_generator_init(uint16_t _factor);
void read_adcs();
void calculate_outputs();

#endif //OUTPUT_GENERATOR_H