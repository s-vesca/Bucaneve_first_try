#include "output_generator.h"
#include "stm32g431xx.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_cordic.h"
#include "defines.h"
#include "stm32g4xx_ll_gpio.h"
#include <stdint.h>

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

//------------------------------------------------------------------------------
//                              Code Variables
//------------------------------------------------------------------------------

uint8_t shift_pos = 0xFF;

__attribute__((section(".ccm_rodata")))
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

void calculate_outputs()
{
    uint32_t    rdata_reg;
    int16_t     phase;
    int16_t     ph_diff;
    int16_t     speed;
    int8_t      dir;
    uint8_t     state_tmp;
    float       phase_float;
    //uint16_t    adc_data_reg_cos;
    //uint16_t    adc_data_reg_sin;
    
    //__attribute__((section(".ccm_data"),aligned(32)))
    static uint8_t  state_out = 0;
    //__attribute__((section(".ccm_data"),aligned(32)))
    static uint8_t  first_iteration = 1;
    //__attribute__((section(".ccm_data"),aligned(32)))
    static int16_t  phase_prev = 0;
    //__attribute__((section(".ccm_data"),aligned(32)))
    static int32_t  filt_state = 0;

    //__attribute__((section(".ccm_data"),aligned(32)))
    //static uint8_t cnt_trace = 0;

    GPIOA->ODR |= GPIO_PIN_10;

    //check that isr comes from cordic data ready flag
    //if(LL_CORDIC_IsActiveFlag_RRDY(CORDIC))
    if(CORDIC->CSR & CORDIC_CSR_RRDY_Msk)
    {
        //adc_data_reg_cos = LL_ADC_REG_ReadMultiConversionData32(ADC12_COMMON, LL_ADC_MULTI_MASTER);
        //adc_data_reg_sin = LL_ADC_REG_ReadMultiConversionData32(ADC12_COMMON, LL_ADC_MULTI_SLAVE);

        //adc_cos_dbg = adc_data_reg_cos;
        //adc_sin_dbg = adc_data_reg_sin;
        
        //check that shift_pos variable is initialized
        if(shift_pos < MAX_SHIFT)
        {
            //------------------ Read Phase -----------------------
            //rdata_reg = LL_CORDIC_ReadData(CORDIC);
            rdata_reg = CORDIC->RDATA;

            //keep least significative 16bits
            phase = (int16_t)(rdata_reg & 0xFFFF);
            //phase_dbg = phase;

            //--------------- Calculate Speed ---------------------
            if(first_iteration)
            {
              phase_prev = phase; //avoid speed glitches at startup
              first_iteration = 0;
            }

            //Fixed Point Subtraction --> Automatic wrapping 
            ph_diff = phase - phase_prev;
            //ph_diff_dbg = ph_diff;

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

            //filt_state_dbg = filt_state;
            //speed_dbg = speed;

#if (TRACE_EN == 1)
            switch(cnt_trace)
            {
                case(0): {ITM->PORT[0].u16 = adc_data_reg_cos;break;}
                case(1): {ITM->PORT[1].u16 = adc_data_reg_sin;break;}
                case(2): {ITM->PORT[2].u16 = phase;break;}
                case(3): {ITM->PORT[3].u16 = speed;break;}
            }

            cnt_trace = ((cnt_trace + 1) > 3) ? (0) : (cnt_trace + 1);
#endif
            //direcrtion 
            dir = (int8_t)(SIGN(speed));
            //dir_dbg = dir;

            //state is given by the 2 LSBs of the phase variable after 
            //the correct shift
            state_tmp = ((phase >> shift_pos) & 0x0003);

            // check that direction matches state variaton and add 1/4 step hysteresis 
            if(((dir ==  1) &&  ((state_tmp > state_out) || ((state_tmp == 0x00) && (state_out == 0x03))) && (((phase >> (shift_pos-2)) & 0x0003) > 0x0002)) ||
               ((dir == -1) &&  ((state_tmp < state_out) || ((state_tmp == 0x03) && (state_out == 0x00))) && (((phase >> (shift_pos-2)) & 0x0003) < 0x0001)))
            {
                //update output only if condition matches
                state_out = state_tmp;
            }

            //convert to grey code and write outputs
            //LL_GPIO_WriteReg(GPIOA, BSRR , (uint32_t)(gray_lut[state_out]));
            GPIOA->BSRR = (uint32_t)(gray_lut[state_out]);

            phase_prev = phase;
        }
            //flag is cleared by HW
    }
    GPIOA->ODR &= ~GPIO_PIN_10;
    return;
}