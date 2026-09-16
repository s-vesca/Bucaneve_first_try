#include "led_handler.h"
#include "main.h"
#include "stm32g4xx_ll_gpio.h"
#include "system_status.h"
#include "system_status.h"

static uint16_t led_state_ton_ms[SYSTEM_STATUS_MAX_NUM] = 
{
    125,        //    SYSTEM_STATUS_CALIBRATION,
    250,        //    SYSTEM_STATUS_CALIBRATION_DONE,
    500,        //    SYSTEM_STATUS_RUNNING,
    0xFFFF      //    SYSTEM_STATUS_ERROR,
};

uint32_t call_freq_khz;

uint8_t led_handler_init(uint32_t _freq_khz)
{
    uint8_t retVal = 0;

    call_freq_khz = _freq_khz;
    retVal = 1;

    return retVal;
}

void blink_led()
{
    system_status_t sys_stat;
    static system_status_t sys_stat_prev = SYSTEM_STATUS_ERROR;
    static uint32_t cnt = 0;
    static uint8_t led_stat = 0;

    if(LL_TIM_IsActiveFlag_UPDATE(TIM7))
    {
        sys_stat = get_system_status();

        if(SYSTEM_STATUS_ERROR != sys_stat)
        {
            if(sys_stat_prev != sys_stat)
            {
                cnt = 0;
            }
            else 
            {
                cnt++;
            }

            if(led_state_ton_ms[sys_stat] * call_freq_khz <= cnt)
            {
                //toggle leds
                if(led_stat)
                {
                    LL_GPIO_SetOutputPin(DBG_LED1_GPIO_Port, DBG_LED1_Pin);
                    LL_GPIO_ResetOutputPin(DBG_LED2_GPIO_Port, DBG_LED2_Pin);
                    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);
                    led_stat = 0;
                }
                else 
                {
                    LL_GPIO_SetOutputPin(DBG_LED2_GPIO_Port, DBG_LED2_Pin);
                    LL_GPIO_ResetOutputPin(DBG_LED1_GPIO_Port, DBG_LED1_Pin);
                    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
                    led_stat = 1;
                }
                cnt=0;
                //LL_GPIO_ResetOutputPin(ERR_LED_GPIO_Port, ERR_LED_Pin);
            }
        }
        else 
        {
            LL_GPIO_ResetOutputPin(DBG_LED1_GPIO_Port, DBG_LED1_Pin);
            LL_GPIO_ResetOutputPin(DBG_LED2_GPIO_Port, DBG_LED2_Pin);
            //LL_GPIO_SetOutputPin(ERR_LED_GPIO_Port, ERR_LED_Pin);
        }
         

        sys_stat_prev = sys_stat;

        LL_TIM_ClearFlag_UPDATE(TIM7);
    }

    return;
}