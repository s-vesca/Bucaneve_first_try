#ifndef DEFINES_H
#define DEFINES_H

#include <stdint.h>

//------------------------------------------------------------------------------
//                             COMPILE OPTIONS
//------------------------------------------------------------------------------

#define DBG                         (0)
#define EEPROM_WRITE                (1)
#define EEPROM_ERASE                (0)
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//                       SENSOR GEOMETRY PARAMETERS
//------------------------------------------------------------------------------
#define PPR_OUT                     (64)
#define POLE_PAIRS                  (4)
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//                                  MACROS
//------------------------------------------------------------------------------

#define SIGN(x)                     ((x >  0) ? (2) : ((x < 0) ? (0) : 1))
#define SIGN_NOZ(x,y)               ((x >  0) ? (2) : ((x < 0) ? (0) : y))
#define ABS(x)                      ((x >= 0) ? (x) : (-x))
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//                         SUPPLY VOLTAGE MEASURE
//------------------------------------------------------------------------------
#define VCC_ADC_N_AVG               (100)       //number of averaged samples
#define VCC_CAL_THR                 (30.0f)     //Threshold for calibration mode
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//                            ANALOG PARAMETERS
//------------------------------------------------------------------------------
#define SINEWAVE_MID_VAL            (2048)      //DC component of sinewave 

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//                           SAMPLING FREQUENCY
//------------------------------------------------------------------------------
#define SYNC_TIM_ARR_CAL            (17000 - 1) //10kHz for calibration
#define SYNC_TIM_ARR_RUN            (567 - 1)   //300kHz for normal operation

//------------------------------------------------------------------------------
//                         AB GENERATION PARAMETERS
//------------------------------------------------------------------------------

#define SPEED_THR_HI_ONESHOT_MODE   (10)
#define SPEED_THR_LO_ONESHOT_MODE   (5)

#define CNT_THR_ONESHOT             (700)

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//                            CALIBRATION PARAMETERS
//------------------------------------------------------------------------------
#define N_TURNS_WAIT_CAL            (10)        //shaft turns before starting measures acquisition
#define N_TURNS_CAL                 (100)       //shaft turns for measures acquisition
#define CAL_CIRCLE_R                (10000.0f)  //circle radius

//calibration parameters used during debug for manual calibration
#define ELLIPSE_CAL_X0              (SINEWAVE_MID_VAL -219)
#define ELLIPSE_CAL_Y0              (SINEWAVE_MID_VAL -11) 
#define ELLIPSE_CAL_T00             (4355)
#define ELLIPSE_CAL_T01             (-375)
#define ELLIPSE_CAL_T10             (368)
#define ELLIPSE_CAL_T11             (4277)

//EEPROM addresses of calibration parameters
#define ELLIPSE_CAL_T00_ADDR        (0x0001)
#define ELLIPSE_CAL_T01_ADDR        (0x0002)
#define ELLIPSE_CAL_T10_ADDR        (0x0003)
#define ELLIPSE_CAL_T11_ADDR        (0x0004)
#define ELLIPSE_CAL_X0_ADDR         (0x0005)
#define ELLIPSE_CAL_Y0_ADDR         (0x0006)

//default values of calibration parameters
#define ELLIPSE_CAL_T00_DEFAULT     (1024)
#define ELLIPSE_CAL_T01_DEFAULT     (0)
#define ELLIPSE_CAL_T10_DEFAULT     (0)
#define ELLIPSE_CAL_T11_DEFAULT     (1024)
#define ELLIPSE_CAL_X0_DEFAULT      (2048)
#define ELLIPSE_CAL_Y0_DEFAULT      (2048)

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//                    CALIBRATION PARAMETERS STRUCTURE
//------------------------------------------------------------------------------
typedef struct cal_coeff_s
{
    int32_t t00;
    int32_t t01;
    int32_t t10;
    int32_t t11;
    int32_t x0;
    int32_t y0;

}cal_coeff_t;

//------------------------------------------------------------------------------

#endif //DEFINES_H