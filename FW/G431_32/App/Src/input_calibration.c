#include <stdint.h>
#include "adc.h"
#include "stm32g431xx.h"
#include "input_calibration.h"
#include "system_status.h"

#define     ARRAY_SIZE      (1000)

struct input_calibration_private_vars_s
{
    //parameters
    uint16_t revolutions_todo;

    //variables
    uint16_t turns_cnt;
    
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
    uint16_t ii;

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
    int16_t    adc_data_reg_cos;
    int16_t    adc_data_reg_sin;

    if((ADC1->ISR & ADC_ISR_JEOC_Msk) && (ADC2->ISR & ADC_ISR_JEOC_Msk))
    {
        adc_data_reg_cos = (int16_t)((ADC1->JDR1));
        adc_data_reg_sin = (int16_t)((ADC2->JDR1));

        //update data accumulators
        input_calibration_private_vars_v.acc.x4   += adc_data_reg_cos * adc_data_reg_cos * adc_data_reg_cos * adc_data_reg_cos;
        input_calibration_private_vars_v.acc.x3   += adc_data_reg_cos * adc_data_reg_cos * adc_data_reg_cos;
        input_calibration_private_vars_v.acc.x2   += adc_data_reg_cos * adc_data_reg_cos;
        input_calibration_private_vars_v.acc.x    += adc_data_reg_cos;
        input_calibration_private_vars_v.acc.y4   += adc_data_reg_sin * adc_data_reg_sin * adc_data_reg_sin * adc_data_reg_sin;
        input_calibration_private_vars_v.acc.y3   += adc_data_reg_sin * adc_data_reg_sin * adc_data_reg_sin;
        input_calibration_private_vars_v.acc.y2   += adc_data_reg_sin * adc_data_reg_sin;
        input_calibration_private_vars_v.acc.y    += adc_data_reg_sin;
        input_calibration_private_vars_v.acc.x3y  += adc_data_reg_cos * adc_data_reg_cos * adc_data_reg_cos * adc_data_reg_sin;
        input_calibration_private_vars_v.acc.x2y2 += adc_data_reg_cos * adc_data_reg_cos * adc_data_reg_sin * adc_data_reg_sin;
        input_calibration_private_vars_v.acc.x2y  += adc_data_reg_cos * adc_data_reg_cos * adc_data_reg_sin;
        input_calibration_private_vars_v.acc.xy3  += adc_data_reg_cos * adc_data_reg_sin * adc_data_reg_sin * adc_data_reg_sin;
        input_calibration_private_vars_v.acc.xy2  += adc_data_reg_cos * adc_data_reg_sin * adc_data_reg_sin;
        input_calibration_private_vars_v.acc.xy   += adc_data_reg_cos * adc_data_reg_sin;

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
           input_calibration_private_vars_v.quadrant.current == 3)
        {
            input_calibration_private_vars_v.turns_cnt++;
        }
        else if(input_calibration_private_vars_v.quadrant.current == 3 && 
                input_calibration_private_vars_v.quadrant.current == 0)
        {
            input_calibration_private_vars_v.turns_cnt--;
        }

        input_calibration_private_vars_v.quadrant.prev = input_calibration_private_vars_v.quadrant.current;

        //if turns completed
        if(input_calibration_private_vars_v.turns_cnt >= input_calibration_private_vars_v.revolutions_todo)
        {
            //stop sampling timer
            TIM2->CR1 &= ~TIM_CR1_CEN_Msk;

            //cal_data_ready_flag = 1
            input_calibration_private_vars_v.calibration_done = 1;
        }

        //JEOC flag is cleared by Hardware when data is read
    }
    return;
}