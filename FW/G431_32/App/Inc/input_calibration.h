#ifndef INPUT_CALIBRATION_H
#define INPUT_CALIBRATION_H

#include <stdint.h>
#include "calib_coeffs.h"
#include "defines.h"

struct input_calibration_private_vars_init_s
{
    //parameters
    uint16_t revolutions_pre_acq;
    uint16_t revolutions_acq;
    int32_t  calib_coeff[CAL_COEFF_MAX_NUM];
};

void input_calibration_init();

//ADC JEOC ISR for calibration
void read_adcs_calibration();

uint8_t input_calibration_get_acquisition_done();
uint8_t input_calibration_calculate_coefficients();
uint8_t input_calibration_copy_cal_coeff(int32_t* _cal_coeff_cpy);

#endif //INPUT_CALIBRATION_H