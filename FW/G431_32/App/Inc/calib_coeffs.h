#ifndef CALIB_COEFFS_H
#define CALIB_COEFFS_H

//------------------------------------------------------------------------------
//                    CALIBRATION PARAMETERS ENUM
//------------------------------------------------------------------------------
typedef enum cal_coeff_e
{
    CAL_COEFF_T00 = 0,
    CAL_COEFF_T01,
    CAL_COEFF_T10,
    CAL_COEFF_T11,
    CAL_COEFF_X0,
    CAL_COEFF_Y0,
    CAL_COEFF_MAX_NUM
}cal_coeff_t;

//------------------------------------------------------------------------------

//getter
int32_t* calib_coeff_get_ptr();

void calib_coeff_reset_to_defaults();

uint8_t calib_coeff_set(int32_t* _new_calib_coeffs);

uint8_t calib_coeff_copy(int32_t* _calib_coeffs_cpy);

#endif //CALIB_COEFFS_H