/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "cordic.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "system_status.h"
#include "led_handler.h"
#include "output_generator.h"
#include "stm32g4xx_it.h"
#include <stdint.h>
#include "defines.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NVIC_ISR_NUMBER (118)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//extern used because variables are declared in the linker file
extern uint32_t Reset_Handler;
extern uint32_t _estack;

__attribute__((aligned(0x200)))
volatile uint32_t isr_vec[NVIC_ISR_NUMBER];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void TIM2_irq_handler();
void adc_sync_timer_init(uint8_t _irq_en);
void adc_sync_timer_start();
void adc_sync_timer_stop();

void relocate_isr_table();

void adcs_init_normal_mode();
void adc_en_conv();

void cordic_init();

void dma_init();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  //----------------------------------------------------------------------------
  //                          RAM initialization
  //----------------------------------------------------------------------------

  //----------------------------------------------------------------------------
  //                        ISR functions relocation
  //----------------------------------------------------------------------------

  relocate_isr_table();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CORDIC_Init();
  MX_TIM2_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  //----------------------------------------------------------------------------
  //                          Peripherals init
  //----------------------------------------------------------------------------
  
  //led handler timer
  led_handler_init(10);
  TIM7->DIER |= TIM_DIER_UIE_Msk;
  TIM7->CR1  |= TIM_CR1_CEN_Msk;

  //-------------------------------- ADCs --------------------------------------
  adcs_init_normal_mode();
  adc_en_conv();

  //------------------------------- CORDIC -------------------------------------
  cordic_init();

  //-------------------------------- DMA ---------------------------------------
  dma_init();

  //---------------------------- ADC sync timer --------------------------------
  adc_sync_timer_init(0);

  //----------------------------------------------------------------------------
  //                              Code init
  //----------------------------------------------------------------------------
  output_generator_init(PPR_OUT/POLE_PAIRS);

  //----------------------------------------------------------------------------
  //                             Start Counting
  //----------------------------------------------------------------------------
  set_system_status(SYSTEM_STATUS_RUNNING);
  adc_sync_timer_start();
  //----------------------------------------------------------------------------

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void relocate_isr_table()
{
  //cortex-M4 IRQs
  isr_vec[0] 							              = (uint32_t)(&_estack);       	//address -16 to end of stack
  isr_vec[1] 							              = (uint32_t)(&Reset_Handler); 	//address -15 to end of stack
  isr_vec[16 + NonMaskableInt_IRQn]  	  = (uint32_t)(NMI_Handler);
  isr_vec[16 + HardFault_IRQn]          = (uint32_t)(HardFault_Handler); //Hard fault is irq # -13
  isr_vec[16 + MemoryManagement_IRQn]  	= (uint32_t)(MemManage_Handler);
  isr_vec[16 + BusFault_IRQn]  			    = (uint32_t)(BusFault_Handler);
  isr_vec[16 + UsageFault_IRQn]  		    = (uint32_t)(UsageFault_Handler);
  isr_vec[16 + SVCall_IRQn]  			      = (uint32_t)(SVC_Handler);
  isr_vec[16 + DebugMonitor_IRQn]  		  = (uint32_t)(DebugMon_Handler);
  isr_vec[16 + PendSV_IRQn]  			      = (uint32_t)(PendSV_Handler);
  isr_vec[16 + SysTick_IRQn] 			      = (uint32_t)(SysTick_Handler);

  //peripheral IRQs
  isr_vec[16 + TIM2_IRQn]      			    = (uint32_t)(TIM2_irq_handler);
  isr_vec[16 + TIM7_IRQn]      			    = (uint32_t)(blink_led);
  isr_vec[16 + CORDIC_IRQn]      			  = (uint32_t)(calculate_outputs);

  //relocate interrupt vector table
  SCB->VTOR = (uint32_t)(isr_vec);

  __DSB();
  __ISB();

  return;
}

void adcs_init_normal_mode()
{
  ADC1->OFR1 -= 863;
  ADC2->OFR1 += 24;
  //ADC1
  LL_ADC_ClearFlag_ADRDY(ADC1);
  LL_ADC_Enable(ADC1);
  while(!LL_ADC_IsActiveFlag_ADRDY(ADC1));
  LL_ADC_ClearFlag_ADRDY(ADC1);

  //ADC1->ISR |= ADC_ISR_ADRDY;
  //ADC1->CR  |= ADC_CR_ADEN;
  //while(!ADC1->ISR & ADC_ISR_ADRDY);
  //ADC1->ISR |= ADC_ISR_ADRDY;

  //ADC2
  LL_ADC_ClearFlag_ADRDY(ADC2);
  LL_ADC_Enable(ADC2);
  while(!LL_ADC_IsActiveFlag_ADRDY(ADC2));
  LL_ADC_ClearFlag_ADRDY(ADC2);
  
  //ADC2->ISR |= ADC_ISR_ADRDY;
  //ADC2->CR  |= ADC_CR_ADEN;
  //while(!ADC2->ISR & ADC_ISR_ADRDY);
  //ADC2->ISR |= ADC_ISR_ADRDY;

  //clear flags
  LL_ADC_ClearFlag_EOC(ADC1);
  LL_ADC_ClearFlag_EOC(ADC2);
  LL_ADC_ClearFlag_OVR(ADC1);
  LL_ADC_ClearFlag_OVR(ADC2);
  //ADC1->ISR |= ADC_ISR_EOC_Msk;
  //ADC2->ISR |= ADC_ISR_EOC_Msk;

  //Enable JEOC interrupt for ADC1 (both ADCs will have finished the conversion)
  //ADC1->IER |= ADC_IER_EOCIE;

  //Setup adc common to generate dma requests for 32bit-word
  LL_ADC_SetMultiDMATransfer(ADC12_COMMON, LL_ADC_MULTI_REG_DMA_UNLMT_RES12_10B);

  return;
}

void adc_en_conv()
{
  ADC1->CR |= ADC_CR_ADSTART_Msk;
  return;
}

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

  LL_CORDIC_EnableIT(CORDIC);
  
  return;
}

void dma_init()
{
  //Set DMA MUX request channel to ADC1
  LL_DMAMUX_SetRequestID(DMAMUX1, LL_DMAMUX_CHANNEL_0, LL_DMAMUX_REQ_ADC1);

  // From 32bit to 32bit transfer
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_WORD);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_WORD);

  //disable pointers increment
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_NOINCREMENT);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

  //Peripheral (ADCs) to Memory (CORDIC memory mapped address)
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  //Circular mode for continuous transfer
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);

  //Very High priority (not needed, only 1 transfer)
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_VERYHIGH);

  //Number of data to transfer. In circular mode it is auto-reloaded after counter reaches zero.
  //It has to be NOT 0 to allow DMA transfers
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 1);

  //set FROM ADDRESS to ADC12_COMMON CDR(Common Data Register)
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&(ADC12_COMMON->CDR));

  //set TO ADDRESS to CORDIC WDATA (Write Data register)
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&(CORDIC->WDATA));

  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

  return;
}

void adc_sync_timer_init(uint8_t _irq_en)
{
  LL_TIM_SetCounter(TIM2, 0);

  if(_irq_en) {LL_TIM_EnableIT_UPDATE(TIM2);}
  else        {LL_TIM_DisableIT_UPDATE(TIM2);}
  
  return;
}

void adc_sync_timer_start()
{
  LL_TIM_EnableCounter(TIM2);
  return;
}

void adc_sync_timer_stop()
{
  LL_TIM_DisableCounter(TIM2);
  return;
}

void TIM2_irq_handler()
{
  return;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
