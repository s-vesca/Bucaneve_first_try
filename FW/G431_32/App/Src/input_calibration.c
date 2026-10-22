#include <math.h>
#include <stdint.h>
#include "adc.h"
#include "calib_coeffs.h"
#include "stm32g431xx.h"
#include "input_calibration.h"
#include "system_status.h"
#include "defines.h"
#include "lin_sys5_solver.h"
#include "sync_timer.h"

//ellipse equation parameters
typedef enum ellipse_par_e
{
    ELLIPSE_PAR_A = 0,
    ELLIPSE_PAR_B,
    ELLIPSE_PAR_C,
    ELLIPSE_PAR_D,
    ELLIPSE_PAR_E,
    ELLIPSE_PAR_MAX_NUM
}ellipse_par_t;

__attribute__((section(".ccm_data"),aligned(32)))
volatile int16_t adc_cos_dbg_cal;
__attribute__((section(".ccm_data"),aligned(32)))
volatile int16_t adc_sin_dbg_cal;

struct input_calibration_private_vars_s
{
    //parameters
    uint16_t revolutions_pre_acq;   //sine periods before starting acquisition
    uint16_t revolutions_acq;       //sine periods acquired for calibration

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
    int32_t cal_coeff[CAL_COEFF_MAX_NUM];

    struct
    {
        uint8_t current;
        uint8_t prev;
    } quadrant;
    
    //flags
    uint8_t  acquisition_done;
};

struct input_calibration_private_vars_s input_calibration_private_vars_v;

//------------------------------------------------------------------------------
//                     Private functions prototypes
//------------------------------------------------------------------------------
uint8_t input_calibration_calculate_comp_coeff(float* solutions);

//------------------------------------------------------------------------------

void input_calibration_init(struct input_calibration_private_vars_init_s _input_calibration_private_vars_init_v)
{
    uint8_t ii;

    input_calibration_private_vars_v.revolutions_pre_acq    = _input_calibration_private_vars_init_v.revolutions_pre_acq;
    input_calibration_private_vars_v.revolutions_acq        = _input_calibration_private_vars_init_v.revolutions_acq;

    input_calibration_private_vars_v.turns_cnt              = 0;

    input_calibration_private_vars_v.acquisition_done       = 0;

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

    for(ii = 0; ii < CAL_COEFF_MAX_NUM; ii++)
    {
        input_calibration_private_vars_v.cal_coeff[ii] = _input_calibration_private_vars_init_v.calib_coeff[ii];
    }
    
    return;
}

void read_adcs_calibration()
{
    int16_t     adc_data_reg_cos;
    int16_t     adc_data_reg_sin;
    int64_t     tmp;
    int64_t     acc_prev;

    //check that both ADC1 and ADC2 have finished injected conversion
    if((ADC1->ISR & ADC_ISR_JEOC_Msk) && (ADC2->ISR & ADC_ISR_JEOC_Msk))
    {
        //read adc values
        adc_data_reg_cos = (int16_t)((ADC1->JDR1)) - SINEWAVE_MID_VAL;
        adc_data_reg_sin = (int16_t)((ADC2->JDR1)) - SINEWAVE_MID_VAL;

        #if DBG == 1
        adc_cos_dbg_cal = (int16_t)(adc_data_reg_cos);
        adc_sin_dbg_cal = (int16_t)(adc_data_reg_sin);
        #endif

        //start acquiring after 10 turns
        if(ABS(input_calibration_private_vars_v.turns_cnt) > 10)
        {
            //------------------------------------------------------------------
            //                   Update data accumulators
            //                      cos = x; sin = y
            //------------------------------------------------------------------

            //---------------------------- x^4 ---------------------------------
            
            tmp = (int64_t)adc_data_reg_cos * adc_data_reg_cos * adc_data_reg_cos * adc_data_reg_cos;
            acc_prev = input_calibration_private_vars_v.acc.x4;
            input_calibration_private_vars_v.acc.x4   = tmp + acc_prev;
            if((tmp > 0 && acc_prev > 0 && input_calibration_private_vars_v.acc.x4 < 0) ||
               (tmp < 0 && acc_prev < 0 && input_calibration_private_vars_v.acc.x4 > 0))
            {
                //overflow occurred
                
                //stop_sampling
                sync_timer_stop();

                //error state
                set_system_status(SYSTEM_STATUS_ERROR);
            }

            //---------------------------- x^3 ---------------------------------

            tmp = (int64_t)adc_data_reg_cos * adc_data_reg_cos * adc_data_reg_cos;
            acc_prev = input_calibration_private_vars_v.acc.x3;
            input_calibration_private_vars_v.acc.x3   = tmp + acc_prev;
            if((tmp > 0 && acc_prev > 0 && input_calibration_private_vars_v.acc.x3 < 0) ||
               (tmp < 0 && acc_prev < 0 && input_calibration_private_vars_v.acc.x3 > 0))
            {
                //overflow occurred
                
                //stop_sampling
                sync_timer_stop();

                //error state
                set_system_status(SYSTEM_STATUS_ERROR);
            }

            //---------------------------- x^2 ---------------------------------

            tmp = (int64_t)adc_data_reg_cos * adc_data_reg_cos;
            acc_prev = input_calibration_private_vars_v.acc.x2;
            input_calibration_private_vars_v.acc.x2   = tmp + acc_prev;
            if((tmp > 0 && acc_prev > 0 && input_calibration_private_vars_v.acc.x2 < 0) ||
               (tmp < 0 && acc_prev < 0 && input_calibration_private_vars_v.acc.x2 > 0))
            {
                //overflow occurred
                
                //stop_sampling
                sync_timer_stop();

                //error state
                set_system_status(SYSTEM_STATUS_ERROR);
            }

            //----------------------------- x ----------------------------------

            tmp = (int64_t)adc_data_reg_cos;
            acc_prev = input_calibration_private_vars_v.acc.x;
            input_calibration_private_vars_v.acc.x    = tmp + acc_prev;
            if((tmp > 0 && acc_prev > 0 && input_calibration_private_vars_v.acc.x < 0) ||
               (tmp < 0 && acc_prev < 0 && input_calibration_private_vars_v.acc.x > 0))
            {
                //overflow occurred
                
                //stop_sampling
                sync_timer_stop();

                //error state
                set_system_status(SYSTEM_STATUS_ERROR);
            }

            //---------------------------- y^4 ---------------------------------
            tmp = (int64_t)adc_data_reg_sin * adc_data_reg_sin * adc_data_reg_sin * adc_data_reg_sin;
            acc_prev = input_calibration_private_vars_v.acc.y4;
            input_calibration_private_vars_v.acc.y4    = tmp + acc_prev;
            if((tmp > 0 && acc_prev > 0 && input_calibration_private_vars_v.acc.y4 < 0) ||
               (tmp < 0 && acc_prev < 0 && input_calibration_private_vars_v.acc.y4 > 0))
            {
                //overflow occurred
                
                //stop_sampling
                sync_timer_stop();

                //error state
                set_system_status(SYSTEM_STATUS_ERROR);
            }

            //---------------------------- y^3 ---------------------------------

            tmp = (int64_t)adc_data_reg_sin * adc_data_reg_sin * adc_data_reg_sin;
            acc_prev = input_calibration_private_vars_v.acc.y3;
            input_calibration_private_vars_v.acc.y3    = tmp + acc_prev;
            if((tmp > 0 && acc_prev > 0 && input_calibration_private_vars_v.acc.y3 < 0) ||
               (tmp < 0 && acc_prev < 0 && input_calibration_private_vars_v.acc.y3 > 0))
            {
                //overflow occurred
                
                //stop_sampling
                sync_timer_stop();

                //error state
                set_system_status(SYSTEM_STATUS_ERROR);
            }

            //---------------------------- y^2 ---------------------------------

            tmp = (int64_t)adc_data_reg_sin * adc_data_reg_sin;
            acc_prev = input_calibration_private_vars_v.acc.y2;
            input_calibration_private_vars_v.acc.y2    = tmp + acc_prev;
            if((tmp > 0 && acc_prev > 0 && input_calibration_private_vars_v.acc.y2 < 0) ||
               (tmp < 0 && acc_prev < 0 && input_calibration_private_vars_v.acc.y2 > 0))
            {
                //overflow occurred
                
                //stop_sampling
                sync_timer_stop();

                //error state
                set_system_status(SYSTEM_STATUS_ERROR);
            }

            //----------------------------- y ----------------------------------

            tmp = (int64_t)adc_data_reg_sin;
            acc_prev = input_calibration_private_vars_v.acc.y;
            input_calibration_private_vars_v.acc.y    = tmp + acc_prev;
            if((tmp > 0 && acc_prev > 0 && input_calibration_private_vars_v.acc.y < 0) ||
               (tmp < 0 && acc_prev < 0 && input_calibration_private_vars_v.acc.y > 0))
            {
                //overflow occurred
                
                //stop_sampling
                sync_timer_stop();

                //error state
                set_system_status(SYSTEM_STATUS_ERROR);
            }

            //--------------------------- x^3*y --------------------------------

            tmp = (int64_t)adc_data_reg_cos * adc_data_reg_cos * adc_data_reg_cos * adc_data_reg_sin;
            acc_prev = input_calibration_private_vars_v.acc.x3y;
            input_calibration_private_vars_v.acc.x3y    = tmp + acc_prev;
            if((tmp > 0 && acc_prev > 0 && input_calibration_private_vars_v.acc.x3y < 0) ||
               (tmp < 0 && acc_prev < 0 && input_calibration_private_vars_v.acc.x3y > 0))
            {
                //overflow occurred
                
                //stop_sampling
                sync_timer_stop();

                //error state
                set_system_status(SYSTEM_STATUS_ERROR);
            }

            //-------------------------- x^2*y^2 -------------------------------

            tmp = (int64_t)adc_data_reg_cos * adc_data_reg_cos * adc_data_reg_sin * adc_data_reg_sin;
            acc_prev = input_calibration_private_vars_v.acc.x2y2;
            input_calibration_private_vars_v.acc.x2y2    = tmp + acc_prev;
            if((tmp > 0 && acc_prev > 0 && input_calibration_private_vars_v.acc.x2y2 < 0) ||
               (tmp < 0 && acc_prev < 0 && input_calibration_private_vars_v.acc.x2y2 > 0))
            {
                //overflow occurred
                
                //stop_sampling
                sync_timer_stop();

                //error state
                set_system_status(SYSTEM_STATUS_ERROR);
            }

            //--------------------------- x^2*y --------------------------------

            tmp = (int64_t)adc_data_reg_cos * adc_data_reg_cos * adc_data_reg_sin;
            acc_prev = input_calibration_private_vars_v.acc.x2y;
            input_calibration_private_vars_v.acc.x2y    = tmp + acc_prev;
            if((tmp > 0 && acc_prev > 0 && input_calibration_private_vars_v.acc.x2y < 0) ||
               (tmp < 0 && acc_prev < 0 && input_calibration_private_vars_v.acc.x2y > 0))
            {
                //overflow occurred
                
                //stop_sampling
                sync_timer_stop();

                //error state
                set_system_status(SYSTEM_STATUS_ERROR);
            }

            //--------------------------- x*y^3 --------------------------------

            tmp = (int64_t)adc_data_reg_cos * adc_data_reg_sin * adc_data_reg_sin * adc_data_reg_sin;
            acc_prev = input_calibration_private_vars_v.acc.xy3;
            input_calibration_private_vars_v.acc.xy3    = tmp + acc_prev;
            if((tmp > 0 && acc_prev > 0 && input_calibration_private_vars_v.acc.xy3 < 0) ||
               (tmp < 0 && acc_prev < 0 && input_calibration_private_vars_v.acc.xy3 > 0))
            {
                //overflow occurred
                
                //stop_sampling
                sync_timer_stop();

                //error state
                set_system_status(SYSTEM_STATUS_ERROR);
            }

            //--------------------------- x*y^2 --------------------------------

            tmp = (int64_t)adc_data_reg_cos * adc_data_reg_sin * adc_data_reg_sin;
            acc_prev = input_calibration_private_vars_v.acc.xy2;
            input_calibration_private_vars_v.acc.xy2    = tmp + acc_prev;
            if((tmp > 0 && acc_prev > 0 && input_calibration_private_vars_v.acc.xy2 < 0) ||
               (tmp < 0 && acc_prev < 0 && input_calibration_private_vars_v.acc.xy2 > 0))
            {
                //overflow occurred
                
                //stop_sampling
                sync_timer_stop();

                //error state
                set_system_status(SYSTEM_STATUS_ERROR);
            }

            //---------------------------- x*y ---------------------------------

            tmp = (int64_t)adc_data_reg_cos * adc_data_reg_sin;
            acc_prev = input_calibration_private_vars_v.acc.xy;
            input_calibration_private_vars_v.acc.xy    = tmp + acc_prev;
            if((tmp > 0 && acc_prev > 0 && input_calibration_private_vars_v.acc.xy < 0) ||
               (tmp < 0 && acc_prev < 0 && input_calibration_private_vars_v.acc.xy > 0))
            {
                //overflow occurred
                
                //stop_sampling
                sync_timer_stop();

                //error state
                set_system_status(SYSTEM_STATUS_ERROR);
            }

        }

        //----------------------------------------------------------------------
        //                       Find current quadrant
        //----------------------------------------------------------------------
        //
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!! WARNING !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //                    An assumption has been made:
        // "Sin and Cos signals are centered around approx. 0V differential"
        // Otherwise the parameter SINEWAVE_MID_VAL should be tuned
        //----------------------------------------------------------------------

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

        //----------------------------------------------------------------------
        //                            Count turns
        //----------------------------------------------------------------------
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
        if(ABS(input_calibration_private_vars_v.turns_cnt) >= input_calibration_private_vars_v.revolutions_acq + input_calibration_private_vars_v.revolutions_pre_acq)
        {
            //stop sampling timer
            sync_timer_stop();

            //cal_data_ready_flag = 1
            input_calibration_private_vars_v.acquisition_done = 1;
        }

        //JEOC flag is cleared by Hardware when data is read
    }
    return;
}

uint8_t input_calibration_get_acquisition_done()
{
    return input_calibration_private_vars_v.acquisition_done;
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
        //calculate compensation coefficients
        retVal = input_calibration_calculate_comp_coeff(solutions);
    }

    return retVal;
}


//------------------------------------------------------------------------------
//                            Private functions
//------------------------------------------------------------------------------

uint8_t input_calibration_calculate_comp_coeff(float* solutions)
{
    uint8_t retVal = 0;
    float x0,y0,fn,delta,s1,s2,sx,sy,a,b,th,r00,r01,r10,r11,t00,t01,t10,t11;

    if((NULL != solutions))
    {
        if(0.0f != solutions[ELLIPSE_PAR_A] && 0.0f != solutions[ELLIPSE_PAR_B] && 
           0.0f != (solutions[ELLIPSE_PAR_B] * solutions[ELLIPSE_PAR_B] - 4.0f * solutions[ELLIPSE_PAR_A] * solutions[ELLIPSE_PAR_C]))
        {
            x0 = (2.0f * solutions[ELLIPSE_PAR_C] * solutions[ELLIPSE_PAR_D] - solutions[ELLIPSE_PAR_B] * solutions[ELLIPSE_PAR_E]) / 
                 (solutions[ELLIPSE_PAR_B] * solutions[ELLIPSE_PAR_B] - 4.0f * solutions[ELLIPSE_PAR_A] * solutions[ELLIPSE_PAR_C]);

            y0 = (2.0f * solutions[ELLIPSE_PAR_A] * solutions[ELLIPSE_PAR_E] - solutions[ELLIPSE_PAR_B] * solutions[ELLIPSE_PAR_D]) / 
                 (solutions[ELLIPSE_PAR_B] * solutions[ELLIPSE_PAR_B] - 4.0f * solutions[ELLIPSE_PAR_A] * solutions[ELLIPSE_PAR_C]);

            //Ellipse known term (F)
            fn = solutions[ELLIPSE_PAR_A] * x0 * x0 + solutions[ELLIPSE_PAR_B] * x0 * y0 + solutions[ELLIPSE_PAR_C] * y0 * y0 + 
                 solutions[ELLIPSE_PAR_D] * x0 + solutions[ELLIPSE_PAR_E] * y0 - 1.0f;

            //Eigenvalues 
            delta = solutions[ELLIPSE_PAR_A] * solutions[ELLIPSE_PAR_A] - 2 * solutions[ELLIPSE_PAR_A] * solutions[ELLIPSE_PAR_C] +
                    solutions[ELLIPSE_PAR_C] * solutions[ELLIPSE_PAR_C] + solutions[ELLIPSE_PAR_B] * solutions[ELLIPSE_PAR_B];
            if(delta > 0.0f)
            {
                s1 = ((solutions[ELLIPSE_PAR_A] + solutions[ELLIPSE_PAR_C]) + sqrtf(delta)) / 2.0f; //greater eigenvalue
                s2 = ((solutions[ELLIPSE_PAR_A] + solutions[ELLIPSE_PAR_C]) - sqrtf(delta)) / 2.0f; //smaller eigenvalue

                if(solutions[ELLIPSE_PAR_A] > solutions[ELLIPSE_PAR_C])
                {
                    //x axis eigenvalue is bigger
                    sx = s1;
                    sy = s2;
                }
                else 
                {
                    //y axis eigenvalue is bigger
                    sx = s2;
                    sy = s1;
                }
                
                if(0.0f != s1 && 0.0f != s2)
                {
                    if(((- fn / s1) > 0.0f) && ((- fn / s2) > 0.0f))
                    {
                        //ellipse axis half-length
                        a = sqrtf(- fn / sx);
                        b = sqrtf(- fn / sy);

                        //ellipse orientation
                        th  = 0.5f*atan2f(solutions[ELLIPSE_PAR_B], solutions[ELLIPSE_PAR_A] - solutions[ELLIPSE_PAR_C]);
                        r00 =  cosf(th);
                        r01 = -sinf(th);
                        r10 =  sinf(th);
                        r11 =  cosf(th);

                        //compensation parameters
                        t00 = 512.0f * CAL_CIRCLE_R * r00 / a;
                        t01 = 512.0f * CAL_CIRCLE_R * r10 / a;
                        t10 = 512.0f * CAL_CIRCLE_R * r01 / b;
                        t11 = 512.0f * CAL_CIRCLE_R * r11 / b;

                        //write compensation coefficients to structure
                        input_calibration_private_vars_v.cal_coeff[CAL_COEFF_X0]  = x0 + SINEWAVE_MID_VAL;
                        input_calibration_private_vars_v.cal_coeff[CAL_COEFF_Y0]  = y0 + SINEWAVE_MID_VAL;
                        input_calibration_private_vars_v.cal_coeff[CAL_COEFF_T00] = t00;
                        input_calibration_private_vars_v.cal_coeff[CAL_COEFF_T01] = t01;
                        input_calibration_private_vars_v.cal_coeff[CAL_COEFF_T10] = t10;
                        input_calibration_private_vars_v.cal_coeff[CAL_COEFF_T11] = t11;

                        retVal = 1;
                    }
                    else 
                    {
                        retVal = 0;
                    }
                }
                else
                {
                    retVal = 0;
                }
            }
            else 
            {
                retVal = 0;
            }
        }
    }

    return retVal;
}

uint8_t input_calibration_copy_cal_coeff(int32_t* _cal_coeff_cpy)
{
    uint8_t retVal = 0;
    uint8_t ii;

    //check that array pointer is valid
    if(NULL != _cal_coeff_cpy)
    {
        for(ii = 0; ii < CAL_COEFF_MAX_NUM; ii++)
        {
            _cal_coeff_cpy[ii] = input_calibration_private_vars_v.cal_coeff[ii];
        }
        retVal = 1;
    }
    return retVal;
}