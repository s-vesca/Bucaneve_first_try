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
__attribute__((section(".ccm_data"),aligned(32)))
volatile int8_t dir_dbg;
__attribute__((section(".ccm_data"),aligned(32)))
volatile int16_t phase_dbg;
__attribute__((section(".ccm_data"),aligned(32)))
volatile int16_t ph_diff_dbg;
__attribute__((section(".ccm_data"),aligned(32)))
volatile int16_t speed_dbg;
__attribute__((section(".ccm_data"),aligned(32)))
volatile int32_t filt_state_dbg;

__attribute__((section(".ccm_data"),aligned(32)))
volatile int16_t adc_cos_dbg;
__attribute__((section(".ccm_data"),aligned(32)))
volatile int16_t adc_sin_dbg;
__attribute__((section(".ccm_data"),aligned(32)))
volatile int16_t adc_cos_filt_dbg;
__attribute__((section(".ccm_data"),aligned(32)))
volatile int16_t adc_sin_filt_dbg;

__attribute__((section(".ccm_data"),aligned(32)))
volatile uint16_t cnt;

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

//next_state[actual_dir][actual_state][proposed_next_state]
const uint8_t next_state_lut[3][4][4] = 
{
    //negative dir
    {  
        {0,0,0,3},
        {0,1,1,1},
        {2,1,2,2},
        {3,3,2,3}
    },
    //0 speed
    {
        {0,0,0,0},
        {1,1,1,1},
        {2,2,2,2},
        {3,3,3,3}
    },
    //positive dir
    {
        {0,1,0,0},
        {1,1,2,1},
        {2,2,2,3},
        {0,3,3,3}
    },
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
    //int32_t    adc_data_reg_cos_filt2;
    int32_t    adc_data_reg_sin_filt;

    int32_t    adc_cos_filt_diff;
    //int32_t    adc_cos_filt2_diff;
    int32_t    adc_sin_filt_diff;

    __attribute__((section(".ccm_data"),aligned(32)))
    static int32_t adc_cos_filt_state;
    __attribute__((section(".ccm_data"),aligned(32)))
    static int32_t adc_cos_filt2_state;
    __attribute__((section(".ccm_data"),aligned(32)))
    static int32_t adc_sin_filt_state;
    
    //TIM6->CNT = 0;
    //GPIOA->ODR |= GPIO_PIN_10;

    //Check that both ADCs have finished their injected conversion
    if((ADC1->ISR & ADC_ISR_JEOC_Msk) && (ADC2->ISR & ADC_ISR_JEOC_Msk))
    {
        adc_data_reg_cos = (int32_t)(ADC1->JDR1) << 16;
        adc_data_reg_sin = (int32_t)(ADC2->JDR1) << 16;

        adc_cos_filt_diff = (int32_t)(adc_data_reg_cos) - adc_cos_filt_state;
        adc_cos_filt_state += (adc_cos_filt_diff >> 4) - (adc_cos_filt_diff >> 9) + (adc_cos_filt_diff >> 11);
        adc_data_reg_cos_filt = (int32_t)(adc_cos_filt_state);
        
        //adc_cos_filt2_diff = (int32_t)(adc_data_reg_cos_filt) - adc_cos_filt2_state;
        //adc_cos_filt2_state += (adc_cos_filt2_diff >> 1) - (adc_cos_filt2_diff >> 5) - (adc_cos_filt2_diff >> 7);
        //adc_data_reg_cos_filt2 = (int32_t)(adc_cos_filt2_state);

        adc_sin_filt_diff = (int32_t)(adc_data_reg_sin) - adc_sin_filt_state;
        adc_sin_filt_state += (adc_sin_filt_diff >> 4) - (adc_sin_filt_diff >> 9) + (adc_sin_filt_diff >> 11);
        adc_data_reg_sin_filt = (int32_t)(adc_sin_filt_state);

        #if DBG == 1
        adc_cos_dbg = (int16_t)(adc_data_reg_cos >> 16);
        adc_sin_dbg = (int16_t)(adc_data_reg_sin >> 16);
        adc_cos_filt_dbg = (int16_t)(adc_data_reg_cos_filt >> 16);
        adc_sin_filt_dbg = (int16_t)(adc_data_reg_sin_filt >> 16);
        #endif

        CORDIC->WDATA = ((uint32_t)(uint16_t)((int16_t)(adc_data_reg_sin_filt >> 16) << 4) << 16) | (uint32_t)(uint16_t)((int16_t)(adc_data_reg_cos_filt >> 16) << 4);

        //JEOC flags are cleared by reading JDRx data registers
    }

    //GPIOA->ODR &= ~GPIO_PIN_10;
    return;
    
}

void calculate_outputs()
{
    uint32_t    rdata_reg;
    int16_t     phase;
    int16_t     ph_diff;
    int16_t     speed;
    int8_t      dir;
    uint8_t     state_tmp;
    uint8_t     state4b_tmp;
    
    __attribute__((section(".ccm_data"),aligned(32)))
    static uint8_t  state_out = 0;
    __attribute__((section(".ccm_data"),aligned(32)))
    static uint8_t  first_iteration = 1;
    __attribute__((section(".ccm_data"),aligned(32)))
    static int16_t  phase_prev = 0;
    __attribute__((section(".ccm_data"),aligned(32)))
    static int32_t  filt_state = 0;

    //GPIOA->ODR |= GPIO_PIN_10;

    //check that isr comes from cordic data ready flag
    if(CORDIC->CSR & CORDIC_CSR_RRDY_Msk)
    {        
        //check that shift_pos variable is initialized
        if(shift_pos < MAX_SHIFT)
        {
            //------------------ Read Phase -----------------------
            rdata_reg = CORDIC->RDATA;
    
            //keep least significative 16bits
            phase = (int16_t)(rdata_reg & 0xFFFF);
            #if DBG == 1
            phase_dbg = phase;
            #endif

            //--------------- Calculate Speed ---------------------
            if(first_iteration)
            {
              phase_prev = phase; //avoid speed glitches at startup
              first_iteration = 0;
            }

            //Fixed Point Subtraction --> Automatic wrapping 
            ph_diff = phase - phase_prev;
            #if DBG == 1
            ph_diff_dbg = ph_diff;
            #endif

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
            
            #if DBG == 1
            filt_state_dbg = filt_state;
            speed_dbg = speed;
            #endif

            //direcrtion 
            dir = (int8_t)(SIGN(speed));

            #if DBG == 1
            dir_dbg = dir;
            #endif

            //state is given by the 2 LSBs of the phase variable after 
            //the correct shift
            state_tmp = ((phase >> shift_pos) & 0x0003);

            // check that direction matches state variaton and add 1/4 step hysteresis 
            state_out = next_state_lut[dir][state_out][state_tmp];

            //convert to grey code and write outputs
            //LL_GPIO_WriteReg(GPIOA, BSRR , (uint32_t)(gray_lut[state_out]));
            GPIOA->BSRR = (uint32_t)(gray_lut[state_out]);

            phase_prev = phase;
            cnt = TIM6->CNT;
            recorderStep();
        }
        //RRDY flag is cleared by HW when reading RDATA register     
    }

    GPIOA->ODR &= ~GPIO_PIN_10;
    return;
}
