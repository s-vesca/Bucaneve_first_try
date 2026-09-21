#ifndef DEFINES_H
#define DEFINES_H

#define DBG             (1)

#define PPR_OUT         (64)
#define POLE_PAIRS      (4)

//parameters for initial measure of supply voltage
#define VCC_ADC_N_AVG   (100)       //number of averaged samples
#define VCC_CAL_THR     (8.0f)//(30.0f)     //Threshold for entering calibration mode

#define SIGN(x)         ((x > 0) ? (2) : ((x < 0) ? (0) : 1))

#endif //DEFINES_H
