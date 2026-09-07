#include "led_handler.h"
#include "main.h"
#include "stm32g4xx_ll_gpio.h"
#include "system_status.h"
#include "system_status.h"

static uint16_t led_state_ton_ms[SYSTEM_STATUS_MAX_NUM] = 
{
    500, //    SYSTEM_STATUS_IDLE,
    250, //    SYSTEM_STATUS_RUNNING,
    -1   //    SYSTEM_STATUS_ERROR,
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
    static system_status_t sys_stat;
    static system_status_t sys_stat_prev = SYSTEM_STATUS_IDLE;
    static uint32_t cnt = 0;
    static uint8_t led_stat = 0;

    sys_stat = get_system_status();

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
    } 

    sys_stat_prev = sys_stat;
    return;
}