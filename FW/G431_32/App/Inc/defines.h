#ifndef DEFINES_H
#define DEFINES_H

#include <stdint.h>

#define DBG             (0)

#define PPR_OUT         (64)
#define POLE_PAIRS      (4)

//parameters for initial measure of supply voltage
#define VCC_ADC_N_AVG   (100)       //number of averaged samples
#define VCC_CAL_THR     (30.0f)//(30.0f)     //Threshold for entering calibration mode

#define SIGN(x)         ((x >  0) ? (2) : ((x < 0) ? (0) : 1))
#define ABS(x)          ((x >= 0) ? (x) : (-x))

//#define ELLIPSE_CAL_T00 (8843)
//#define ELLIPSE_CAL_T01 (-50)
//#define ELLIPSE_CAL_T10 (-50)
//#define ELLIPSE_CAL_T11 (8670)

#define ELLIPSE_CAL_T00 (4438)
#define ELLIPSE_CAL_T01 (-55)
#define ELLIPSE_CAL_T10 (-55)
#define ELLIPSE_CAL_T11 (4352)

#define ELLIPSE_CAL_T00_ADDR    (0x0001)
#define ELLIPSE_CAL_T01_ADDR    (0x0002)
#define ELLIPSE_CAL_T10_ADDR    (0x0003)
#define ELLIPSE_CAL_T11_ADDR    (0x0004)
#define ELLIPSE_CAL_X0_ADDR     (0x0005)
#define ELLIPSE_CAL_Y0_ADDR     (0x0006)

#define ELLIPSE_CAL_T00_DEFAULT (1024)
#define ELLIPSE_CAL_T01_DEFAULT (0)
#define ELLIPSE_CAL_T10_DEFAULT (0)
#define ELLIPSE_CAL_T11_DEFAULT (1024)
#define ELLIPSE_CAL_X0_DEFAULT  (0)
#define ELLIPSE_CAL_Y0_DEFAULT  (0)

typedef struct cal_coeff_s
{
    int32_t t00;
    int32_t t01;
    int32_t t10;
    int32_t t11;
    int32_t x0;
    int32_t y0;

}cal_coeff_t;

#endif //DEFINES_H