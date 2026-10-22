#include "calib_coeffs.h"
#include <sys/types.h>

//EEPROM addresses of calibration parameters
#define CALIB_COEFF_T00_ADDR        (0x0001)
#define CALIB_COEFF_T01_ADDR        (0x0002)
#define CALIB_COEFF_T10_ADDR        (0x0003)
#define CALIB_COEFF_T11_ADDR        (0x0004)
#define CALIB_COEFF_X0_ADDR         (0x0005)
#define CALIB_COEFF_Y0_ADDR         (0x0006)

//default values of calibration parameters
#define CALIB_COEFF_T00_DEFAULT     (1024)
#define CALIB_COEFF_T01_DEFAULT     (0)
#define CALIB_COEFF_T10_DEFAULT     (0)
#define CALIB_COEFF_T11_DEFAULT     (1024)
#define CALIB_COEFF_X0_DEFAULT      (2048)
#define CALIB_COEFF_Y0_DEFAULT      (2048)

uint16_t calib_coeffs_eeprom_addr[CAL_COEFF_MAX_NUM] = 
{
    CALIB_COEFF_T00_ADDR,
    CALIB_COEFF_T01_ADDR,
    CALIB_COEFF_T10_ADDR,
    CALIB_COEFF_T11_ADDR,
    CALIB_COEFF_X0_ADDR,
    CALIB_COEFF_Y0_ADDR
};

int32_t calib_coeffs_defaults[CAL_COEFF_MAX_NUM] = 
{
    CALIB_COEFF_T00_DEFAULT,
    CALIB_COEFF_T01_DEFAULT,
    CALIB_COEFF_T10_DEFAULT,
    CALIB_COEFF_T11_DEFAULT,
    CALIB_COEFF_X0_DEFAULT,
    CALIB_COEFF_Y0_DEFAULT
};

int32_t calib_coeffs[CAL_COEFF_MAX_NUM];

int32_t* calib_coeff_get_ptr()
{
    return calib_coeffs;
}

void calib_coeff_reset_to_defaults()
{
    uint8_t ii;

    for(ii = 0; ii < CAL_COEFF_MAX_NUM; ii++)
    {
        calib_coeffs[ii] = calib_coeffs_defaults[ii];
    }
    return;
}

uint8_t calib_coeff_set(int32_t* _new_calib_coeffs)
{
    uint8_t ii;
    uint8_t retVal = 0;

    if(NULL != _new_calib_coeffs)
    {
        for(ii = 0; ii < CAL_COEFF_MAX_NUM; ii++)
        {
            calib_coeffs[ii] = _new_calib_coeffs[ii];
        }
        retVal = 1;
    }
    return retVal;
}

uint8_t calib_coeff_copy(int32_t* _calib_coeffs_cpy)
{
    uint8_t retVal = 0;
    uint8_t ii;

    if(NULL != _calib_coeffs_cpy)
    {
        for(ii = 0; ii < CAL_COEFF_MAX_NUM; ii++)
        {
            calib_coeffs[ii] = _calib_coeffs_cpy[ii];
        }
        retVal = 1;
    }
    return retVal;
}

uint16_t calib_coeff_get_start_eeprom_addr()
{
    return calib_coeffs_eeprom_addr[0];
}

uint16_t calib_coeff_get_eeprom_addr(uint8_t _index)
{
    return calib_coeffs_eeprom_addr[_index];
}

int32_t  calib_coeff_get_default(uint8_t _index)
{
    return calib_coeffs_defaults[_index];
}