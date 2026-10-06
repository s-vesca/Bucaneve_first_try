#include <stdint.h>
#include "adc.h"
#include "stm32g431xx.h"
#include "input_calibration.h"
#include "system_status.h"
#include "defines.h"
#include "lin_sys5_solver.h"

#define     ARRAY_SIZE      (1000)

__attribute__((section(".ccm_data"),aligned(32)))
volatile int16_t adc_cos_dbg_cal;
__attribute__((section(".ccm_data"),aligned(32)))
volatile int16_t adc_sin_dbg_cal;

struct input_calibration_private_vars_s
{
    //parameters
    uint16_t revolutions_pre_acq;
    uint16_t revolutions_todo;

    //variables
    int16_t turns_cnt;
    
    //measures accumulators
    struct {
        int64_t x4;
        int64_t x3;
        int64_t x2;
        int64_t x;

        int64_t y4;
        int64_t y3;
        int64_t y2;
        int64_t y;

        int64_t x3y;
        int64_t x2y2;
        int64_t x2y;
        int64_t xy3;
        int64_t xy2;
        int64_t xy;
    }acc;

    //calibration coefficients
    cal_coeff_t cal_coeff;

    struct
    {
        uint8_t current;
        uint8_t prev;
    } quadrant;
    
    //flags
    uint8_t  calibration_done;
};

struct input_calibration_private_vars_s input_calibration_private_vars_v;

void input_calibration_init(struct input_calibration_private_vars_init_s input_calibration_private_vars_init_v)
{
    input_calibration_private_vars_v.revolutions_pre_acq    = input_calibration_private_vars_init_v.revolutions_pre_acq;
    input_calibration_private_vars_v.revolutions_todo       = input_calibration_private_vars_init_v.revolutions_todo;

    input_calibration_private_vars_v.turns_cnt              = 0;

    input_calibration_private_vars_v.calibration_done       = 0;

    //reset accumulators
    input_calibration_private_vars_v.acc.x4                 = 0;
    input_calibration_private_vars_v.acc.x3                 = 0;
    input_calibration_private_vars_v.acc.x2                 = 0;
    input_calibration_private_vars_v.acc.x                  = 0;
    input_calibration_private_vars_v.acc.y4                 = 0;
    input_calibration_private_vars_v.acc.y3                 = 0;
    input_calibration_private_vars_v.acc.y2                 = 0;
    input_calibration_private_vars_v.acc.y                  = 0;
    input_calibration_private_vars_v.acc.x3y                = 0;
    input_calibration_private_vars_v.acc.x2y2               = 0;
    input_calibration_private_vars_v.acc.x2y                = 0;
    input_calibration_private_vars_v.acc.xy3                = 0;
    input_calibration_private_vars_v.acc.xy2                = 0;
    input_calibration_private_vars_v.acc.xy                 = 0;

    input_calibration_private_vars_v.quadrant.current       = 0;
    input_calibration_private_vars_v.quadrant.prev          = 0;

    return;
}

uint8_t input_calibration_get_calibration_done()
{
    return input_calibration_private_vars_v.calibration_done;
}

void read_adcs_calibration()
{
    extern uint16_t cnt;
    int16_t    adc_data_reg_cos;
    int16_t    adc_data_reg_sin;

    if((ADC1->ISR & ADC_ISR_JEOC_Msk) && (ADC2->ISR & ADC_ISR_JEOC_Msk))
    {
        adc_data_reg_cos = (int16_t)((ADC1->JDR1));
        adc_data_reg_sin = (int16_t)((ADC2->JDR1));

        #if DBG == 1
        adc_cos_dbg_cal = (int16_t)(adc_data_reg_cos);
        adc_sin_dbg_cal = (int16_t)(adc_data_reg_sin);
        #endif

        //start acquiring after 10 turns
        if(ABS(input_calibration_private_vars_v.turns_cnt) > 10)
        {
            //update data accumulators
            input_calibration_private_vars_v.acc.x4   += (int64_t)adc_data_reg_cos * adc_data_reg_cos * adc_data_reg_cos * adc_data_reg_cos;
            input_calibration_private_vars_v.acc.x3   += (int64_t)adc_data_reg_cos * adc_data_reg_cos * adc_data_reg_cos;
            input_calibration_private_vars_v.acc.x2   += (int64_t)adc_data_reg_cos * adc_data_reg_cos;
            input_calibration_private_vars_v.acc.x    += (int64_t)adc_data_reg_cos;
            input_calibration_private_vars_v.acc.y4   += (int64_t)adc_data_reg_sin * adc_data_reg_sin * adc_data_reg_sin * adc_data_reg_sin;
            input_calibration_private_vars_v.acc.y3   += (int64_t)adc_data_reg_sin * adc_data_reg_sin * adc_data_reg_sin;
            input_calibration_private_vars_v.acc.y2   += (int64_t)adc_data_reg_sin * adc_data_reg_sin;
            input_calibration_private_vars_v.acc.y    += (int64_t)adc_data_reg_sin;
            input_calibration_private_vars_v.acc.x3y  += (int64_t)adc_data_reg_cos * adc_data_reg_cos * adc_data_reg_cos * adc_data_reg_sin;
            input_calibration_private_vars_v.acc.x2y2 += (int64_t)adc_data_reg_cos * adc_data_reg_cos * adc_data_reg_sin * adc_data_reg_sin;
            input_calibration_private_vars_v.acc.x2y  += (int64_t)adc_data_reg_cos * adc_data_reg_cos * adc_data_reg_sin;
            input_calibration_private_vars_v.acc.xy3  += (int64_t)adc_data_reg_cos * adc_data_reg_sin * adc_data_reg_sin * adc_data_reg_sin;
            input_calibration_private_vars_v.acc.xy2  += (int64_t)adc_data_reg_cos * adc_data_reg_sin * adc_data_reg_sin;
            input_calibration_private_vars_v.acc.xy   += (int64_t)adc_data_reg_cos * adc_data_reg_sin;
        }

        //check current quadrant and current dir
        if((adc_data_reg_cos > 0) && (adc_data_reg_sin > 0))
        {
            input_calibration_private_vars_v.quadrant.current = 0;
        }
        if((adc_data_reg_cos < 0) && (adc_data_reg_sin > 0))
        {
            input_calibration_private_vars_v.quadrant.current = 1;
        }
        if((adc_data_reg_cos < 0) && (adc_data_reg_sin < 0))
        {
            input_calibration_private_vars_v.quadrant.current = 2;
        }
        if((adc_data_reg_cos > 0) && (adc_data_reg_sin < 0))
        {
            input_calibration_private_vars_v.quadrant.current = 3;
        }

        //count turns
        if(input_calibration_private_vars_v.quadrant.current == 0 && 
           input_calibration_private_vars_v.quadrant.prev == 3)
        {
            input_calibration_private_vars_v.turns_cnt++;
        }
        else if(input_calibration_private_vars_v.quadrant.current == 3 && 
                input_calibration_private_vars_v.quadrant.prev == 0)
        {
            input_calibration_private_vars_v.turns_cnt--;
        }

        input_calibration_private_vars_v.quadrant.prev = input_calibration_private_vars_v.quadrant.current;

        //if turns completed
        if(ABS(input_calibration_private_vars_v.turns_cnt) >= input_calibration_private_vars_v.revolutions_todo + input_calibration_private_vars_v.revolutions_pre_acq)
        {
            //stop sampling timer
            TIM2->CR1 &= ~TIM_CR1_CEN_Msk;

            //cal_data_ready_flag = 1
            input_calibration_private_vars_v.calibration_done = 1;
        }

        //JEOC flag is cleared by Hardware when data is read
        cnt = TIM6->CNT;
    }
    return;
}

uint8_t input_calibration_calculate_coefficients()
{
    uint8_t retVal = 0;

    //build 5x5 linear system matrix
    int64_t sys_matrix[5][5] = 
    {
        {  
                input_calibration_private_vars_v.acc.x4,
                input_calibration_private_vars_v.acc.x3y,
                input_calibration_private_vars_v.acc.x2y2,
                input_calibration_private_vars_v.acc.x3,
                input_calibration_private_vars_v.acc.x2y
            },
        {
                input_calibration_private_vars_v.acc.x3y,
                input_calibration_private_vars_v.acc.x2y2,
                input_calibration_private_vars_v.acc.xy3,
                input_calibration_private_vars_v.acc.x2y,
                input_calibration_private_vars_v.acc.xy2
            },
        {
                input_calibration_private_vars_v.acc.x2y2,
                input_calibration_private_vars_v.acc.xy3,
                input_calibration_private_vars_v.acc.y4,
                input_calibration_private_vars_v.acc.xy2,
                input_calibration_private_vars_v.acc.y3
            },
        {
                input_calibration_private_vars_v.acc.x3,
                input_calibration_private_vars_v.acc.x2y,
                input_calibration_private_vars_v.acc.xy2,
                input_calibration_private_vars_v.acc.x2,
                input_calibration_private_vars_v.acc.y
            },
        {
                input_calibration_private_vars_v.acc.x2y,
                input_calibration_private_vars_v.acc.xy2,
                input_calibration_private_vars_v.acc.y3,
                input_calibration_private_vars_v.acc.xy,
                input_calibration_private_vars_v.acc.y2
            },
    };

    //build known values array
    int64_t known_vals[5] =
    {
        input_calibration_private_vars_v.acc.x2,
        input_calibration_private_vars_v.acc.xy,
        input_calibration_private_vars_v.acc.y2,
        input_calibration_private_vars_v.acc.x,
        input_calibration_private_vars_v.acc.y
    };

    float solutions[5] = {0.0f, 0.0f,0.0f,0.0f,0.0f};

    //call the solver
    retVal = lin_sys5_solve(sys_matrix, known_vals, solutions);

    if(retVal)
    {
        //calculate ellipse coefficients
    }

    return retVal;
}

cal_coeff_t input_calibration_get_cal_coeff()
{
    return input_calibration_private_vars_v.cal_coeff;
}