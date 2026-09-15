#include "output_generator.h"
#include "stm32g431xx.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_cordic.h"
#include "defines.h"
#include "stm32g4xx_ll_gpio.h"
#include <stdint.h>
#include "recorder.h"

#define MAX_FACTOR (32767)
#define MAX_SHIFT  (16)

//------------------------------------------------------------------------------
//                              Debug Variables
//------------------------------------------------------------------------------

volatile int8_t dir_dbg;
volatile int16_t phase_dbg;
volatile int16_t ph_diff_dbg;
volatile int16_t speed_dbg;
volatile int32_t filt_state_dbg;

volatile int16_t adc_cos_dbg;
volatile int16_t adc_sin_dbg;
volatile int16_t adc_cos_filt_dbg;
volatile int16_t adc_sin_filt_dbg;

volatile uint16_t cnt;

//------------------------------------------------------------------------------
//                              Code Variables
//------------------------------------------------------------------------------

uint8_t shift_pos = 0xFF;

uint32_t gray_lut[4] = 
{
  GPIO_BSRR_BR12_Msk | GPIO_BSRR_BR11_Msk,
  GPIO_BSRR_BR12_Msk | GPIO_BSRR_BS11_Msk,
  GPIO_BSRR_BS12_Msk | GPIO_BSRR_BS11_Msk,
  GPIO_BSRR_BS12_Msk | GPIO_BSRR_BR11_Msk
};

void output_generator_init(uint16_t _factor)
{
    //check factor is lower than maximum
    if(_factor < MAX_FACTOR)
    {
        shift_pos = 2;

        while((1 << (shift_pos - 1)) <= _factor) 
        {
            shift_pos++;
        }

        shift_pos = MAX_SHIFT - shift_pos;
    }
}

void read_adcs()
{
    int32_t    adc_data_reg_cos;
    int32_t    adc_data_reg_sin;
    int32_t    adc_data_reg_cos_filt;
    int32_t    adc_data_reg_cos_filt2;
    int32_t    adc_data_reg_sin_filt;

    int32_t    adc_cos_filt_diff;
    int32_t    adc_cos_filt2_diff;
    int32_t    adc_sin_filt_diff;

    static int32_t adc_cos_filt_state;
    static int32_t adc_cos_filt2_state;
    static int32_t adc_sin_filt_state;

    TIM6->CNT = 0;

    if(LL_ADC_IsActiveFlag_JEOC(ADC1) && LL_ADC_IsActiveFlag_JEOC(ADC2))
    {
        adc_data_reg_cos = (int32_t)(ADC1->JDR1) << 16;
        adc_data_reg_sin = (int32_t)(ADC2->JDR1) << 16;

        adc_cos_filt_diff = (int32_t)(adc_data_reg_cos) - adc_cos_filt_state;
        adc_cos_filt_state += (adc_cos_filt_diff >> 1) - (adc_cos_filt_diff >> 5) - (adc_cos_filt_diff >> 7);
        adc_data_reg_cos_filt = (int32_t)(adc_cos_filt_state);
        
        adc_cos_filt2_diff = (int32_t)(adc_data_reg_cos_filt) - adc_cos_filt2_state;
        adc_cos_filt2_state += (adc_cos_filt2_diff >> 1) - (adc_cos_filt2_diff >> 5) - (adc_cos_filt2_diff >> 7);
        adc_data_reg_cos_filt2 = (int32_t)(adc_cos_filt2_state);

        adc_sin_filt_diff = (int32_t)(adc_data_reg_sin) - adc_sin_filt_state;
        adc_sin_filt_state += (adc_sin_filt_diff >> 2) + (adc_sin_filt_diff >> 6) + (adc_sin_filt_diff >> 9);
        adc_data_reg_sin_filt = (int32_t)(adc_sin_filt_state);

        adc_cos_dbg = (int16_t)(adc_data_reg_cos >> 16);
        adc_sin_dbg = (int16_t)(adc_data_reg_sin >> 16);
        adc_cos_filt_dbg = (int16_t)(adc_data_reg_cos_filt2 >> 16);
        adc_sin_filt_dbg = (int16_t)(adc_data_reg_sin_filt >> 16);

        LL_CORDIC_WriteData(CORDIC, ((uint32_t)(uint16_t)adc_data_reg_sin_filt << 16) | (uint32_t)(uint16_t)adc_data_reg_cos_filt);

        //clear flags
        LL_ADC_ClearFlag_EOC(ADC1);
        LL_ADC_ClearFlag_EOC(ADC2);
    }
}

void calculate_outputs()
{
    uint32_t    rdata_reg;
    int16_t     phase;
    int16_t     ph_diff;
    int16_t     speed;
    int8_t      dir;
    uint8_t     state_tmp;
    
    static uint8_t  state_out = 0;
    static uint8_t  first_iteration = 1;
    static int16_t  phase_prev = 0;
    static int32_t  filt_state = 0;

    //check that isr comes from cordic data ready flag
    if(LL_CORDIC_IsActiveFlag_RRDY(CORDIC))
    {        
        //check that shift_pos variable is initialized
        if(shift_pos < MAX_SHIFT)
        {
            //------------------ Read Phase -----------------------
            rdata_reg = LL_CORDIC_ReadData(CORDIC);

            //keep least significative 16bits
            phase = (int16_t)(rdata_reg & 0xFFFF);
            phase_dbg = phase;

            //--------------- Calculate Speed ---------------------
            if(first_iteration)
            {
              phase_prev = phase; //avoid speed glitches at startup
              first_iteration = 0;
            }

            //Fixed Point Subtraction --> Automatic wrapping 
            ph_diff = phase - phase_prev;
            ph_diff_dbg = ph_diff;

            //------------------------------------------------------------------
            //                          Filter
            //------------------------------------------------------------------
            //
            //                        1 - 1023/1024
            //           H(z) = ------------------------------
            //                     1  -  (1023/1024) z^-1
            //
            // alpha = 1023/1024
            // tau = Ts/(1-alpha) = 5.12ms approx
            // fs = 31 Hz approx
            //------------------------------------------------------------------

            //speed, measured as a fraction of the sampling frequency (Q1.15 format)
            filt_state += ph_diff - (filt_state >> 10);
            speed = (int16_t)(filt_state >> 10);

            filt_state_dbg = filt_state;
            speed_dbg = speed;

            //direcrtion 
            dir = (int8_t)(SIGN(speed));
            dir_dbg = dir;

            //state is given by the 2 LSBs of the phase variable after 
            //the correct shift
            state_tmp = ((phase >> shift_pos) & 0x0003);

            // check that direction matches state variaton and add 1/4 step hysteresis 
            if(((dir ==  1) &&  ((state_tmp > state_out) || ((state_tmp == 0x00) && (state_out == 0x03))) && (((phase >> (shift_pos-2)) & 0x0003) > 0x0001)) ||
               ((dir == -1) &&  ((state_tmp < state_out) || ((state_tmp == 0x03) && (state_out == 0x00))) && (((phase >> (shift_pos-2)) & 0x0003) < 0x0003)))
            {
                //update output only if condition matches
                state_out = state_tmp;
            }

            //convert to grey code and write outputs
            LL_GPIO_WriteReg(GPIOA, BSRR , (uint32_t)(gray_lut[state_out]));

            phase_prev = phase;
            cnt = TIM6->CNT;
            recorderStep();
        }
            //flag is cleared by HW

            
    }
    return;
}