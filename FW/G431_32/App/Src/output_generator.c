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
volatile int32_t ph_diff_dbg;
__attribute__((section(".ccm_data"),aligned(32)))
volatile int16_t speed_dbg;
__attribute__((section(".ccm_data"),aligned(32)))
volatile int32_t filt_state_dbg;
__attribute__((section(".ccm_data"),aligned(32)))
volatile uint32_t time_to_next_state_dbg;

__attribute__((section(".ccm_data"),aligned(32)))
volatile int16_t adc_cos_dbg;
__attribute__((section(".ccm_data"),aligned(32)))
volatile int16_t adc_sin_dbg;
__attribute__((section(".ccm_data"),aligned(32)))
volatile int16_t adc_cos_filt_dbg;
__attribute__((section(".ccm_data"),aligned(32)))
volatile int16_t adc_sin_filt_dbg;

__attribute__((section(".ccm_data"),aligned(32)))
volatile int16_t cos_comp_dbg;
__attribute__((section(".ccm_data"),aligned(32)))
volatile int16_t sin_comp_dbg;

__attribute__((section(".ccm_data"),aligned(32)))
volatile uint16_t cnt;

//------------------------------------------------------------------------------
//                              Code Variables
//------------------------------------------------------------------------------
__attribute__((section(".ccm_data"),aligned(32)))
volatile uint8_t shift_pos = 0xFF;

__attribute__((section(".ccm_data"),aligned(32)))
volatile uint8_t next_state = 0;

__attribute__((section(".ccm_data"),aligned(32)))
volatile uint8_t pulse_fired = 0;

__attribute__((section(".ccm_data"),aligned(32)))
volatile uint8_t dir = 0;

__attribute__((section(".ccm_data"),aligned(32)))
volatile int16_t     speed;

__attribute__((section(".ccm_data"),aligned(32)))
volatile cal_coeff_t cal_coeff_v = {
    .t00    = ELLIPSE_CAL_T00_DEFAULT,
    .t01    = ELLIPSE_CAL_T01_DEFAULT,
    .t10    = ELLIPSE_CAL_T10_DEFAULT,
    .t11    = ELLIPSE_CAL_T11_DEFAULT,
    .x0     = ELLIPSE_CAL_X0_DEFAULT,
    .y0     = ELLIPSE_CAL_Y0_DEFAULT,
};

__attribute__((section(".ccm_rodata")))
const uint32_t gray_lut[4] = 
{
  GPIO_BSRR_BR12_Msk | GPIO_BSRR_BR11_Msk,
  GPIO_BSRR_BR12_Msk | GPIO_BSRR_BS11_Msk,
  GPIO_BSRR_BS12_Msk | GPIO_BSRR_BS11_Msk,
  GPIO_BSRR_BS12_Msk | GPIO_BSRR_BR11_Msk
};

//state_lut[actual_dir][actual_state][proposed_next_state]
__attribute__((section(".ccm_rodata")))
const uint8_t state_lut[3][4][4] = 
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
    }
}; 

//next_state[actual_dir][actual_state]
__attribute__((section(".ccm_rodata")))
const uint8_t next_state_lut[3][4] = 
{
    //negative dir
    {3,0,1,2},
    //0 speed
    {0,1,2,3},
    //positive dir
    {1,2,3,0}
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

    return;
}


void read_adcs_output()
{
    int32_t    adc_data_reg_cos;
    int32_t    adc_data_reg_sin;
    int32_t    adc_data_reg_cos_filt;
    int32_t    adc_data_reg_sin_filt;

    int32_t    adc_cos_filt_diff;
    int32_t    adc_sin_filt_diff;
    int32_t    cos_comp;
    int32_t    sin_comp;

    int16_t    cos_q16;
    int16_t    sin_q16;
    int16_t    ph_diff;

    int32_t    mag_sq;

    __attribute__((section(".ccm_data"),aligned(32)))
    static int32_t adc_cos_filt_state=0;
    __attribute__((section(".ccm_data"),aligned(32)))
    static int32_t adc_sin_filt_state=0;

    __attribute__((section(".ccm_data"),aligned(32)))
    static int32_t cos_prev=0;
    __attribute__((section(".ccm_data"),aligned(32)))
    static int32_t sin_prev=0;
    __attribute__((section(".ccm_data"),aligned(32)))
    static uint8_t dir_prev=0;

    __attribute__((section(".ccm_data"),aligned(32)))
    static int32_t  speed_filt_state = 0;
    
    //TIM6->CNT = 0;
    //GPIOA->ODR |= GPIO_PIN_10;

    //Check that both ADCs have finished their injected conversion
    if((ADC1->ISR & ADC_ISR_JEOC_Msk) && (ADC2->ISR & ADC_ISR_JEOC_Msk))
    {
        adc_data_reg_cos = (int32_t)(ADC1->JDR1 - cal_coeff_v.x0) << 16;
        adc_data_reg_sin = (int32_t)(ADC2->JDR1 - cal_coeff_v.y0) << 16;

        cos_comp = ((adc_data_reg_cos>>10) * cal_coeff_v.t00 + (adc_data_reg_sin>>10) * cal_coeff_v.t01);
        sin_comp = ((adc_data_reg_cos>>10) * cal_coeff_v.t10 + (adc_data_reg_sin>>10) * cal_coeff_v.t11);

        adc_cos_filt_diff = (int32_t)(cos_comp) - adc_cos_filt_state;
        adc_cos_filt_state += (adc_cos_filt_diff >> 6) - (adc_cos_filt_diff >> 8) + (adc_cos_filt_diff >> 10); //fc = 600Hz @ 300kHz Fs
        //adc_cos_filt_state += (adc_cos_filt_diff >> 4) - (adc_cos_filt_diff >> 9) + (adc_cos_filt_diff >> 11); //fc = 3kHz @ 300kHz Fs
        //adc_cos_filt_state += (adc_cos_filt_diff >> 1) - (adc_cos_filt_diff >> 5) - (adc_cos_filt_diff >> 7);
        adc_data_reg_cos_filt = (int32_t)(adc_cos_filt_state);

        adc_sin_filt_diff = (int32_t)(sin_comp) - adc_sin_filt_state;
        adc_sin_filt_state += (adc_sin_filt_diff >> 6) - (adc_sin_filt_diff >> 8) + (adc_sin_filt_diff >> 10); //fc = 600Hz @ 300kHz Fs
        //adc_sin_filt_state += (adc_sin_filt_diff >> 4) - (adc_sin_filt_diff >> 9) + (adc_sin_filt_diff >> 11); //fc = 3kHz @ 300kHz Fs
        //adc_sin_filt_state += (adc_sin_filt_diff >> 1) - (adc_sin_filt_diff >> 5) - (adc_sin_filt_diff >> 7);
        adc_data_reg_sin_filt = (int32_t)(adc_sin_filt_state);
        
        #if DBG == 1
        adc_cos_dbg = (int16_t)(adc_data_reg_cos >> 16);
        adc_sin_dbg = (int16_t)(adc_data_reg_sin >> 16);
        adc_cos_filt_dbg = (int16_t)(adc_data_reg_cos_filt >> 16);
        adc_sin_filt_dbg = (int16_t)(adc_data_reg_sin_filt >> 16);
        cos_comp_dbg = (int16_t)(cos_comp >> 16);
        sin_comp_dbg = (int16_t)(sin_comp >> 16);
        ITM->PORT[0].u32 = *(uint32_t*)&cos_comp_dbg;
        #endif

        cos_q16 = (int16_t)(adc_data_reg_cos_filt >> 16);
        sin_q16 = (int16_t)(adc_data_reg_sin_filt >> 16);

        //CORDIC->WDATA = ((uint32_t)(uint16_t)((int16_t)(adc_data_reg_sin_filt >> 16) << 4) << 16) | (uint32_t)(uint16_t)((int16_t)(adc_data_reg_cos_filt >> 16) << 4);
        CORDIC->WDATA = ((uint32_t)(uint16_t)(sin_q16) << 16) | (uint32_t)(uint16_t)(cos_q16);

        mag_sq = (((int32_t)cos_q16*cos_q16) + ((int32_t)sin_q16*sin_q16)) >> 14 ;
        if(!mag_sq)
        {
            mag_sq = 1;
        }

        //ph_diff ~= sin(ph_diff) if ph_diff --> 0
        ph_diff = (int16_t)((((int32_t)(sin_q16) * (int32_t)(cos_prev)) - ((int32_t)(cos_q16) * (int32_t)(sin_prev))) / (mag_sq));
        
        #if DBG
        ph_diff_dbg = ph_diff;
        #endif

        //speed, measured as a fraction of the sampling frequency (Q1.15 format)
        speed_filt_state += ph_diff - (speed_filt_state >> 10);
        speed = (int16_t)(speed_filt_state >> 10);

        cos_prev = cos_q16;
        sin_prev = sin_q16;

        dir = SIGN_NOZ(speed, dir_prev);

        //JEOC flags are cleared by reading JDRx data registers
    }

    //GPIOA->ODR &= ~GPIO_PIN_10;
    return;
    
}

void calculate_outputs()
{
    uint32_t    rdata_reg;
    int16_t     phase;
    //int8_t      dir;
    uint8_t     state_tmp;
    //int16_t     delta_next_state;
    int16_t     delta_next_state_pos;
    int16_t     delta_next_state_neg;
    uint8_t     oneshot_mode;
    
    __attribute__((section(".ccm_data"),aligned(32)))
    static uint8_t  state_out = 0;
    __attribute__((section(".ccm_data"),aligned(32)))
    static uint8_t  first_iteration = 1;
    __attribute__((section(".ccm_data"),aligned(32)))
    static int32_t  filt_state = 0;
    __attribute__((section(".ccm_data"),aligned(32)))
    static uint16_t mask;
    __attribute__((section(".ccm_data"),aligned(32)))
    static uint32_t time_to_next_state = 0;
    __attribute__((section(".ccm_data"),aligned(32)))
    static int8_t  dir_prev = 0;

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
              //phase_prev = phase; //avoid speed glitches at startup
              mask = 1;
              for(uint8_t ii = 0; ii < shift_pos-1; ii++)
              {
                mask <<= 1;
                mask |= 0x0001;
              }
              pulse_fired = 0;
              first_iteration = 0;
              
            }

            //Fixed Point Subtraction --> Automatic wrapping 
            //ph_diff1 = phase - phase_prev;
            #if DBG == 1
            ph_diff_dbg = ph_diff1;
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

            //sign(sin(Delta_theta))
            

            #if DBG == 1
            dir_dbg = dir;
            #endif

            delta_next_state_pos = (1 << shift_pos) - (phase & mask);
            delta_next_state_neg = -(phase & mask);

            next_state = next_state_lut[dir][state_out];

            //state is given by the 2 LSBs of the phase variable after 
            //the correct shift
            state_tmp = ((phase >> shift_pos) & 0x0003);

            // check that direction matches state variaton 
            state_out = state_lut[dir][state_out][state_tmp];

            if(10 <= speed)
            {
                time_to_next_state = (uint32_t)((int32_t)(delta_next_state_pos * 1700) / (int32_t)speed);
                oneshot_mode = 1;
            }
            else if(-10 >= speed) 
            {
                time_to_next_state = (uint32_t)((int32_t)(delta_next_state_neg * 1700) / (int32_t)speed);
                oneshot_mode = 1;
            }
            else if((-5 < speed) && (5 > speed))
            {
                time_to_next_state = 0;
                oneshot_mode = 0;
            } 

            #if DBG == 1
            time_to_next_state_dbg = time_to_next_state;
            #endif

            if(time_to_next_state && (time_to_next_state < 2100) && !pulse_fired)
            {
                //start oneshot timer
                TIM4->CCR1 = time_to_next_state;
                TIM4->CNT = 0;
                TIM4->ARR   = time_to_next_state;
                TIM4->CR1 |= TIM_CR1_CEN_Msk;
            
                pulse_fired = 1;
            }

            if(!oneshot_mode)
            {
                GPIOA->BSRR = (uint32_t)(gray_lut[state_out]);
            } 

            //phase_prev = phase;
            dir_prev = dir;
            
            #if DBG
            recorderStep();
            #endif
            //ITM->PORT[1].u32 = *(uint32_t*)&sin_comp_dbg;
            //ITM->PORT[3].u16 = *(uint16_t*)&speed;
            cnt = TIM6->CNT;
        }
        //RRDY flag is cleared by HW when reading RDATA register     
    }

    //GPIOA->ODR &= ~GPIO_PIN_10;
    return;
}

void output_generator_set_cal_coeff(cal_coeff_t _cal_coeff)
{
    cal_coeff_v = _cal_coeff;
    return;
}

void update_out_pins()
{
    if(TIM4->SR & TIM_SR_CC1IF_Msk)
    {
        GPIOA->BSRR = (uint32_t)(gray_lut[next_state]);

        pulse_fired = 0;

        //clear flag
        TIM4->SR &= ~TIM_SR_CC1IF_Msk;
    }
    return;
}