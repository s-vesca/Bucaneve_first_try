#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_adc.h"
#include "adc_fcns.h"
#include "defines.h"
#include <stdint.h>

void adcs_init()
{
  //---------------------------- calibrate ADC1 --------------------------------
  if(LL_ADC_IsEnabled(ADC1) != RESET)
  {
    LL_ADC_Disable(ADC1);
    while(LL_ADC_IsEnabled(ADC1) != RESET);
  }

  LL_ADC_EnableInternalRegulator(ADC1);
  HAL_Delay(10);
  LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
  while (LL_ADC_IsCalibrationOnGoing(ADC1));

  //---------------------------- calibrate ADC2 --------------------------------
  if(LL_ADC_IsEnabled(ADC2) != RESET)
  {
    LL_ADC_Disable(ADC2);
    while(LL_ADC_IsEnabled(ADC2) != RESET);
  }

  LL_ADC_EnableInternalRegulator(ADC2);
  HAL_Delay(10);
  LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED);
  while (LL_ADC_IsCalibrationOnGoing(ADC2));


  //------------------------------ Enable ADC1 ---------------------------------
  LL_ADC_ClearFlag_ADRDY(ADC1);
  LL_ADC_Enable(ADC1);
  while(!LL_ADC_IsActiveFlag_ADRDY(ADC1));
  LL_ADC_ClearFlag_ADRDY(ADC1);

  //------------------------------ Enable ADC2 ---------------------------------
  LL_ADC_ClearFlag_ADRDY(ADC2);
  LL_ADC_Enable(ADC2);
  while(!LL_ADC_IsActiveFlag_ADRDY(ADC2));
  LL_ADC_ClearFlag_ADRDY(ADC2);

  //clear flags
  LL_ADC_ClearFlag_JEOC(ADC1);
  LL_ADC_ClearFlag_JEOC(ADC2);

  //Enable JEOC interrupt for ADC1 (both ADCs will have finished the conversion)
  ADC1->IER |= ADC_IER_JEOCIE;

  return;
}

void adcs_en_injected_conversion()
{
    //Enable injected conversions
    ADC1->CR |= ADC_CR_JADSTART;
    ADC2->CR |= ADC_CR_JADSTART;

    return;
}

float adc_read_supply_voltage()
{
  uint16_t ii;
  uint32_t adc_vcc_acc = 0;

  for(ii = 0; ii < VCC_ADC_N_AVG; ii++)
  {
    ADC1->CR |= ADC_CR_ADEN_Msk;
    ADC1->CR |= ADC_CR_ADSTART_Msk;
    while(!(ADC1->ISR & ADC_ISR_EOC_Msk));
    adc_vcc_acc += (uint16_t)(ADC1->DR & 0x0FFF);
    HAL_Delay(1);
  }

  return (float)adc_vcc_acc / (float)(VCC_ADC_N_AVG) * ADC_CNT_TO_SUPPLY_VOLT;
}