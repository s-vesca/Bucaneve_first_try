#include <stdint.h>
#include "adc.h"
#include "stm32g431xx.h"
#include "input_calibration.h"
#include "system_status.h"

#define     ARRAY_SIZE      (1000)

struct input_calibration_private_vars_s
{
    //parameters
    uint16_t n_pts_per_turn;
    uint16_t n_turns;

    //data
    uint16_t pts_cnt;
    uint16_t turns_cnt;
    float    cos_data_array[ARRAY_SIZE];
    float    sin_data_array[ARRAY_SIZE];

    //flags
    uint8_t  enough_data;
};

struct input_calibration_private_vars_s input_calibration_private_vars_v;

void input_calibration_init(struct input_calibration_private_vars_init_s input_calibration_private_vars_init_v)
{
    uint16_t ii;

    input_calibration_private_vars_v.n_pts_per_turn = input_calibration_private_vars_init_v.n_pts_per_turn;
    input_calibration_private_vars_v.n_turns = input_calibration_private_vars_init_v.n_turns;

    input_calibration_private_vars_v.pts_cnt = 0;

    for(ii = 0; ii < ARRAY_SIZE; ii++)
    {
        input_calibration_private_vars_v.cos_data_array[ii] = 0.0f;
        input_calibration_private_vars_v.sin_data_array[ii] = 0.0f;
    }

    input_calibration_private_vars_v.enough_data = 0;

    return;
}

void read_adcs_calibration()
{
    int32_t    adc_data_reg_cos;
    int32_t    adc_data_reg_sin;
    float      adc_cos_volt;
    float      adc_sin_volt;

    if((ADC1->ISR & ADC_ISR_JEOC_Msk) && (ADC2->ISR & ADC_ISR_JEOC_Msk))
    {
        adc_data_reg_cos = (int32_t)(ADC1->JDR1);
        adc_data_reg_sin = (int32_t)(ADC2->JDR1);

        adc_cos_volt = adc_data_reg_cos * 3.3f / 4096.0f;
        adc_sin_volt = adc_data_reg_sin * 3.3f / 4096.0f;

        if(SYSTEM_STATUS_CALIBRATION == get_system_status())
        {
            input_calibration_private_vars_v.cos_data_array[input_calibration_private_vars_v.pts_cnt] = adc_cos_volt;
            input_calibration_private_vars_v.sin_data_array[input_calibration_private_vars_v.pts_cnt] = adc_sin_volt;

            input_calibration_private_vars_v.pts_cnt++;
        }
        
        if(input_calibration_private_vars_v.pts_cnt >= ARRAY_SIZE)
        {
            //stop sampling
            TIM2->CR1 &= ~TIM_CR1_CEN_Msk;

            //set status calibration done
            set_system_status(SYSTEM_STATUS_CALIBRATION_DONE);
        }
    }
    return;
}