#include "stm32g4xx_ll_adc.h"
#include "adc_fcns.h"

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