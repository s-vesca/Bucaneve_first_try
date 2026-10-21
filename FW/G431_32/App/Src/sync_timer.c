#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_gpio.h"
#include "defines.h"
#include "sync_timer.h"

void sync_timer_init(uint8_t _irq_en)
{
  LL_TIM_SetCounter(TIM2, 0);

  if(_irq_en) {LL_TIM_EnableIT_UPDATE(TIM2);}
  else        {LL_TIM_DisableIT_UPDATE(TIM2);}
  
  LL_TIM_ClearFlag_UPDATE(TIM2);
  return;
}

void sync_timer_start(uint8_t _calibration)
{
  if(_calibration)
  {
    //10kHz
    LL_TIM_SetAutoReload(TIM2, SYNC_TIM_ARR_CAL);
  }
  else 
  {
    #if (DBG)
    //50kHz
    LL_TIM_SetAutoReload(TIM2, 3400);
    #else
    //300kHz
    LL_TIM_SetAutoReload(TIM2, SYNC_TIM_ARR_RUN);
    #endif
  }
  LL_TIM_EnableCounter(TIM2);
  return;
}

void sync_timer_stop()
{
  LL_TIM_DisableCounter(TIM2);
  return;
}

void TIM2_irq_handler()
{
  if(TIM2->SR & TIM_SR_UIF_Msk)
  {
    //GPIOA->ODR |= GPIO_PIN_10;
    TIM6->CNT = 0;
    LL_TIM_ClearFlag_UPDATE(TIM2);
  }
  return;
}