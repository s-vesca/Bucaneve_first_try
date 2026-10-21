
#include "stm32g4xx_ll_cordic.h"
#include "cordic_fcns.h"

void cordic_init()
{
  //1 32bit Write OP with (16bit ARG1 | 16bit ARG2) 
  LL_CORDIC_SetInSize(CORDIC, LL_CORDIC_INSIZE_16BITS);
  LL_CORDIC_SetNbWrite(CORDIC, LL_CORDIC_NBWRITE_1);

  //1 32bit Read OP with (16bit MOD | 16bit PHASE)
  LL_CORDIC_SetOutSize(CORDIC, LL_CORDIC_OUTSIZE_16BITS);
  LL_CORDIC_SetNbRead(CORDIC, LL_CORDIC_NBREAD_1);

  //24 Approximation cycles
  LL_CORDIC_SetPrecision(CORDIC, LL_CORDIC_PRECISION_6CYCLES);
  
  //OPERATION: PHASE
  LL_CORDIC_SetFunction(CORDIC, LL_CORDIC_FUNCTION_PHASE);

  //read rdata register to eventually clear interrupt flag
  uint32_t rdata = CORDIC->RDATA;

  LL_CORDIC_EnableIT(CORDIC);
  
  return;
}