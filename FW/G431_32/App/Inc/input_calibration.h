#ifndef INPUT_CALIBRATION_H
#define INPUT_CALIBRATION_H

#include <stdint.h>
#include "defines.h"

typedef enum ellipse_par_e
{
    ELLIPSE_PAR_A = 0,
    ELLIPSE_PAR_B,
    ELLIPSE_PAR_C,
    ELLIPSE_PAR_D,
    ELLIPSE_PAR_E,
    ELLIPSE_PAR_MAX_NUM
}ellipse_par_t;

struct input_calibration_private_vars_init_s
{
    //parameters
    uint16_t revolutions_pre_acq;
    uint16_t revolutions_todo;
};

void input_calibration_init();
uint8_t input_calibration_get_acquisition_done();
void read_adcs_calibration();
uint8_t input_calibration_calculate_coefficients();
cal_coeff_t input_calibration_get_cal_coeff();

#endif //INPUT_CALIBRATION_H