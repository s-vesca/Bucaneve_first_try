#ifndef INPUT_CALIBRATION_H
#define INPUT_CALIBRATION_H

#include <stdint.h>
#include "defines.h"
struct input_calibration_private_vars_init_s
{
    //parameters
    uint16_t revolutions_todo;
};

void input_calibration_init();
uint8_t input_calibration_get_calibration_done();
void read_adcs_calibration();
void input_calibration_calculate_coefficients();
cal_coeff_t input_calibration_get_cal_coeff();

#endif //INPUT_CALIBRATION_H