#include "output_generator.h"
#include "calib_coeffs.h"
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
//                  output_generator private Variables
//------------------------------------------------------------------------------
__attribute__((section(".ccm_data"),aligned(32)))
static uint8_t shift_pos = 0xFF;

__attribute__((section(".ccm_data"),aligned(32)))
static uint8_t next_state = 0;

__attribute__((section(".ccm_data"),aligned(32)))
static uint8_t pulse_fired = 0;

__attribute__((section(".ccm_data"),aligned(32)))
static uint8_t dir = 0;

__attribute__((section(".ccm_data"),aligned(32)))
static int16_t speed;

__attribute__((section(".ccm_data"),aligned(32)))
static int32_t cal_coeff_v[CAL_COEFF_MAX_NUM] = 
{
    ELLIPSE_CAL_T00_DEFAULT,
    ELLIPSE_CAL_T01_DEFAULT,
    ELLIPSE_CAL_T10_DEFAULT,
    ELLIPSE_CAL_T11_DEFAULT,
    ELLIPSE_CAL_X0_DEFAULT,
    ELLIPSE_CAL_Y0_DEFAULT,
};

//------------------------------------------------------------------------------
//                              Lookup Tables
//------------------------------------------------------------------------------

__attribute__((section(".ccm_rodata")))
const uint32_t gpio_gray_lut[4] = 
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

    //------------------------- Init Oneshot timer -----------------------------
    
    //enable TIM4 CH1
    TIM4->CCER |= TIM_CCER_CC1E;

    //enable TIM4 CH1 IRQ
    TIM4->DIER |= TIM_DIER_CC1IE_Msk;

    //configure TIM4 in One Pulse mode 
    TIM4->CR1 |= TIM_CR1_OPM_Msk;

    //--------------------------------------------------------------------------

    return;
}


void read_adcs_output()
{
    //--------- Static Vars (CCM_SRAM for constant access latency) -------------
    __attribute__((section(".ccm_data"),aligned(32)))
    static int32_t adc_cos_filt_state=0;
    __attribute__((section(".ccm_data"),aligned(32)))
    static int32_t adc_sin_filt_state=0;
    __attribute__((section(".ccm_data"),aligned(32)))
    static int32_t speed_filt_state = 0;
    __attribute__((section(".ccm_data"),aligned(32)))
    static int32_t cos_prev=0;
    __attribute__((section(".ccm_data"),aligned(32)))
    static int32_t sin_prev=0;
    __attribute__((section(".ccm_data"),aligned(32)))
    static uint8_t dir_prev=0;

    //---------------------- Local Vars (CPU registers) ------------------------
    int32_t    adc_reg_cos;
    int32_t    adc_reg_sin;
    int32_t    adc_cos_filt;
    int32_t    adc_sin_filt;

    int32_t    adc_cos_filt_diff;
    int32_t    adc_sin_filt_diff;
    int32_t    adc_cos_comp;
    int32_t    adc_sin_comp;

    int16_t    cos_q16;
    int16_t    sin_q16;
    int16_t    ph_diff;

    int32_t    mag_sq;

    //--------------------------------------------------------------------------
    
    GPIOA->BSRR |= GPIO_PIN_10;

    //Check that both ADCs have finished their injected conversion
    if((ADC1->ISR & ADC_ISR_JEOC_Msk) && (ADC2->ISR & ADC_ISR_JEOC_Msk))
    {
        //GPIOA->BSRR |= GPIO_PIN_10; //Pull GPIO UP for Debug 

        //read ADCs registers, remove offset and shift them to run 32 bits filters
        adc_reg_cos = (int32_t)(ADC1->JDR1 - cal_coeff_v[CAL_COEFF_X0]) << 16;
        adc_reg_sin = (int32_t)(ADC2->JDR1 - cal_coeff_v[CAL_COEFF_Y0]) << 16;

        //----------------------------------------------------------------------
        //          Ellipse compensation matrix multiplication
        //----------------------------------------------------------------------
        //
        //   [adc_cos_comp] _ [cal_coeff_v.t00   cal_coeff_v.t01] [adc_reg_cos]
        //   |            | _ |                                 | |           |
        //   [adc_sin_comp]   [cal_coeff_v.t10   cal_coeff_v.t11] [adc_reg_sin]
        //----------------------------------------------------------------------

        adc_cos_comp = ((adc_reg_cos>>10) * cal_coeff_v[CAL_COEFF_T00] + (adc_reg_sin>>10) * cal_coeff_v[CAL_COEFF_T01]);
        adc_sin_comp = ((adc_reg_cos>>10) * cal_coeff_v[CAL_COEFF_T10] + (adc_reg_sin>>10) * cal_coeff_v[CAL_COEFF_T11]);

        //----------------------------------------------------------------------
        //                          Sin Cos Filter
        //----------------------------------------------------------------------
        //
        //                        1 - 1011/1024
        //           H(z) = ------------------------------
        //                     1  -  (1011/1024) z^-1
        //
        // alpha = 1011/1024
        // tau = Ts/(1-alpha) = 263us approx
        // fc = 605 Hz approx
        //----------------------------------------------------------------------
        adc_cos_filt_diff = (int32_t)(adc_cos_comp) - adc_cos_filt_state;
        adc_cos_filt_state += (adc_cos_filt_diff >> 6) - (adc_cos_filt_diff >> 8) + (adc_cos_filt_diff >> 10);      //fc = 600Hz @ 300kHz Fs
        //adc_cos_filt_state += (adc_cos_filt_diff >> 4) - (adc_cos_filt_diff >> 9) + (adc_cos_filt_diff >> 11);    //fc = 3kHz @ 300kHz Fs
        adc_cos_filt = (int32_t)(adc_cos_filt_state);

        adc_sin_filt_diff = (int32_t)(adc_sin_comp) - adc_sin_filt_state;
        adc_sin_filt_state += (adc_sin_filt_diff >> 6) - (adc_sin_filt_diff >> 8) + (adc_sin_filt_diff >> 10);      //fc = 600Hz @ 300kHz Fs
        //adc_sin_filt_state += (adc_sin_filt_diff >> 4) - (adc_sin_filt_diff >> 9) + (adc_sin_filt_diff >> 11);    //fc = 3kHz @ 300kHz Fs
        adc_sin_filt = (int32_t)(adc_sin_filt_state);
        
        #if DBG == 1
        adc_cos_dbg = (int16_t)(adc_reg_cos >> 16);
        adc_sin_dbg = (int16_t)(adc_reg_sin >> 16);
        adc_cos_filt_dbg = (int16_t)(adc_cos_filt >> 16);
        adc_sin_filt_dbg = (int16_t)(adc_sin_filt >> 16);
        cos_comp_dbg = (int16_t)(adc_cos_comp >> 16);
        sin_comp_dbg = (int16_t)(adc_sin_comp >> 16);
        ITM->PORT[0].u32 = *(uint32_t*)&cos_comp_dbg;
        #endif

        //return to 16 bit precision
        cos_q16 = (int16_t)(adc_cos_filt >> 16);
        sin_q16 = (int16_t)(adc_sin_filt >> 16);

        //load cordic 
        CORDIC->WDATA = ((uint32_t)(uint16_t)(sin_q16) << 16) | (uint32_t)(uint16_t)(cos_q16);

        //calculate vector magnitude, prepare it for normalization division
        mag_sq = (((int32_t)cos_q16*cos_q16) + ((int32_t)sin_q16*sin_q16)) >> 14 ;
        if(!mag_sq)
        {
            //avoid division by 0
            mag_sq = 1;
        }

        //------------------------ Speed calculation ---------------------------
        // Approximate speed from sin and cos measures to reduce cpu load after 
        // phase calculation

        //small angles approximation
        //ph_diff ~= sin(ph_diff) if ph_diff --> 0
        ph_diff = (int16_t)((((int32_t)(sin_q16) * (int32_t)(cos_prev)) - ((int32_t)(cos_q16) * (int32_t)(sin_prev))) / (mag_sq));
        
        #if DBG
        ph_diff_dbg = ph_diff;
        #endif

        //----------------------------------------------------------------------
        //                          Speed Filter
        //----------------------------------------------------------------------
        //
        //                        1 - 1023/1024
        //           H(z) = ------------------------------
        //                     1  -  (1023/1024) z^-1
        //
        // alpha = 1023/1024
        // tau = Ts/(1-alpha) = 3.4ms approx
        // fc = 47 Hz approx
        //----------------------------------------------------------------------

        //speed, measured as a fraction of the sampling frequency (Q1.15 format)
        speed_filt_state += ph_diff - (speed_filt_state >> 10);
        speed = (int16_t)(speed_filt_state >> 10);

        cos_prev = cos_q16;
        sin_prev = sin_q16;

        dir = SIGN_NOZ(speed, dir_prev);

        dir_prev = dir;

        #if DBG == 1
        dir_dbg = dir;
        #endif

        ITM->PORT[3].u16 = *(uint16_t*)&speed;
        ITM->PORT[4].u8 = *(uint8_t*)&dir;

        //JEOC flags are cleared by reading JDRx data registers
    }

    //GPIOA->BSRR |= GPIO_PIN_10 << 16;
    return;
    
}

void calculate_outputs()
{
    //--------- Static Vars (CCM_SRAM for constant access latency) -------------

    __attribute__((section(".ccm_data"),aligned(32)))
    static uint8_t      oneshot_mode = 0;
    __attribute__((section(".ccm_data"),aligned(32)))
    static uint8_t      state_out = 0;
    __attribute__((section(".ccm_data"),aligned(32)))
    static uint8_t      first_iteration = 1;
    __attribute__((section(".ccm_data"),aligned(32)))
    static uint16_t     mask_delta_state;
    __attribute__((section(".ccm_data"),aligned(32)))
    static uint32_t     time_to_next_state = 0;

    //---------------------- Local Vars (CPU registers) ------------------------

    uint32_t            rdata_reg;
    int16_t             phase;
    uint8_t             state_tmp;
    int16_t             delta_next_state_pos;
    int16_t             delta_next_state_neg;

    //--------------------------------------------------------------------------

    //GPIOA->BSRR |= GPIO_PIN_10; //Pull GPIO UP for Debug 

    //check that isr comes from cordic data ready flag and shift_pos variable has been initialized
    if((CORDIC->CSR & CORDIC_CSR_RRDY_Msk) && (shift_pos < MAX_SHIFT))
    {       
        //------------------ Read Phase -----------------------
        rdata_reg = CORDIC->RDATA;
    
        //keep least significative 16bits
        phase = (int16_t)(rdata_reg & 0xFFFF);
        #if DBG == 1
        phase_dbg = phase;
        #endif

        //----------------------- Initialize variables -------------------------
        if(first_iteration)
        {
          mask_delta_state = 1;
          for(uint8_t ii = 0; ii < shift_pos-1; ii++)
          {
            mask_delta_state <<= 1;
            mask_delta_state |= 0x0001;
          }
          pulse_fired = 0;
          first_iteration = 0;
        }

        //state is given by the 2 LSBs of the phase variable after 
        //the correct shift
        state_tmp = ((phase >> shift_pos) & 0x0003);

        //AB_state FSM:
        //state_out is changed if state_tmp matches the state following state_out with current direction
        state_out = state_lut[dir][state_out][state_tmp];

        //calculate next state to be used in oneshot mode
        next_state = next_state_lut[dir][state_out];

        //the angle difference to next state is given by the LSBs after the state bits
        delta_next_state_pos = (1 << shift_pos) - (phase & mask_delta_state);   //counting up
        delta_next_state_neg = -(phase & mask_delta_state);                     //counting down

        //apply hysteresys on speed to choose between oneshot and normal mode
        if(SPEED_THR_HI_ONESHOT_MODE <= speed)
        {
            //calculate time to next state when positive speed
            time_to_next_state = (uint32_t)((int32_t)(delta_next_state_pos * (SYNC_TIM_ARR_RUN+1)) / (int32_t)speed);
            oneshot_mode = 1;
        }
        else if(-SPEED_THR_HI_ONESHOT_MODE >= speed) 
        {
            //calculate time to next state when negative speed
            time_to_next_state = (uint32_t)((int32_t)(delta_next_state_neg * (SYNC_TIM_ARR_RUN+1)) / (int32_t)speed);
            oneshot_mode = 1;
        }
        else if((-SPEED_THR_LO_ONESHOT_MODE < speed) && (SPEED_THR_LO_ONESHOT_MODE > speed))
        {
            //disable oneshot mode
            time_to_next_state = 0;
            oneshot_mode = 0;
        }

        #if DBG == 1
        time_to_next_state_dbg = time_to_next_state;
        #endif

        //when time_to_next_state is less than the next sampling instant and the pulse has not been fired yet
        if((0 != time_to_next_state) && (time_to_next_state < CNT_THR_ONESHOT) && !pulse_fired)
        {
            //start oneshot timer
            TIM4->CCR1  = time_to_next_state;
            TIM4->ARR   = time_to_next_state;
            TIM4->CNT   = 0;
            TIM4->CR1  |= TIM_CR1_CEN_Msk;
        
            pulse_fired = 1;
        }

        //if oneshot mode disabled (low speed)
        if(!oneshot_mode)
        {
            //directly update AB GPIO state
            GPIOA->BSRR = (uint32_t)(gpio_gray_lut[state_out]);
        } 
            
        #if DBG
        recorderStep();
        #endif

        //Send variables out on SWO Trace (uncomment only 1)
        //ITM->PORT[1].u32 = *(uint32_t*)&sin_comp_dbg;
        //ITM->PORT[2].u16 = *(uint16_t*)&phase;
        //ITM->PORT[5].u8 = *(uint8_t*)&state_out;
        
        //----- RRDY flag is cleared by HW when reading RDATA register ---------     
    }
    GPIOA->BSRR |= GPIO_PIN_10 << 16;
    return;
}

void output_generator_set_cal_coeff(int32_t* _cal_coeff)
{
    uint8_t ii;

    for(ii = 0; ii < CAL_COEFF_MAX_NUM; ii++)
    {
        cal_coeff_v[ii] = _cal_coeff[ii];
    }
    return;
}

void update_out_pins()
{
    //check that IRQ comes from the TIM4 CH1 COMPARE
    if(TIM4->SR & TIM_SR_CC1IF_Msk)
    {
        //update AB GPIO state
        GPIOA->BSRR = (uint32_t)(gpio_gray_lut[next_state]);

        //reset the flag
        pulse_fired = 0;

        //clear ISR flag
        TIM4->SR &= ~TIM_SR_CC1IF_Msk;
    }
    return;
}