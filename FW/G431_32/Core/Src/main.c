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
#include "crc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
#include "stm32g4xx_it.h"

#include "defines.h"
#include "eeprom_emul.h"

#include "sync_timer.h"
#include "cordic_fcns.h"
#include "adc_fcns.h"

#include "system_status.h"
#include "led_handler.h"
#include "output_generator.h"
#include "input_calibration.h"
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
extern uint16_t cnt;
float exec_time_us;

//extern used because variables are declared in the linker script
extern uint32_t Reset_Handler;
extern uint32_t _estack, _sidata, _sdata, _edata, _si_ccm_sram_code, _sccm_sram_code,
                _eccm_sram_code, _si_ccm_sram_data, _sccm_sram_data, _eccm_sram_data, 
                _si_ccm_sram_rodata, _sccm_sram_rodata, _eccm_sram_rodata, _sbss, 
                _ebss;

__attribute__((section(".ccm_data")))
__attribute__((aligned(0x200)))
volatile uint32_t isr_vec[NVIC_ISR_NUMBER];

uint16_t adc_vcc_val;
uint16_t adc_vcc_cnt;
uint32_t adc_vcc_acc;
float vcc;

uint32_t eeprom_init_vals[6];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void relocate_isr_table();
void set_adc_irq_ptr(uint8_t calibration);
void init_ram();

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
  uint16_t ii;
  uint8_t cal_status;
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

  init_ram();

  //----------------------------------------------------------------------------
  //                        ISR functions relocation
  //----------------------------------------------------------------------------

  relocate_isr_table(); 

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CORDIC_Init();
  MX_TIM2_Init();
  MX_TIM7_Init();
  MX_TIM6_Init();
  MX_CRC_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  //----------------------------------------------------------------------------
  //                             EEPROM init
  //----------------------------------------------------------------------------
  HAL_FLASH_Unlock();

  #if EEPROM_ERASE
  EE_Status eeprom_status = EE_Init(EE_FORCED_ERASE);
  if (eeprom_status != EE_OK)
  {
  	Error_Handler();
  }
  
  //write default values to NVM variables
  eeprom_status = EE_WriteVariable32bits(ELLIPSE_CAL_T00_ADDR, ELLIPSE_CAL_T00_DEFAULT);
  if(EE_OK != eeprom_status  && EE_CLEANUP_REQUIRED != eeprom_status)
  {
    Error_Handler();
  }
  eeprom_status = EE_WriteVariable32bits(ELLIPSE_CAL_T01_ADDR, ELLIPSE_CAL_T01_DEFAULT);
  if(EE_OK != eeprom_status  && EE_CLEANUP_REQUIRED != eeprom_status)
  {
    Error_Handler();
  }
  eeprom_status = EE_WriteVariable32bits(ELLIPSE_CAL_T10_ADDR, ELLIPSE_CAL_T10_DEFAULT);
  if(EE_OK != eeprom_status  && EE_CLEANUP_REQUIRED != eeprom_status)
  {
    Error_Handler();
  }
  eeprom_status = EE_WriteVariable32bits(ELLIPSE_CAL_T11_ADDR, ELLIPSE_CAL_T11_DEFAULT);
  if(EE_OK != eeprom_status  && EE_CLEANUP_REQUIRED != eeprom_status)
  {
    Error_Handler();
  }
  eeprom_status = EE_WriteVariable32bits(ELLIPSE_CAL_X0_ADDR, ELLIPSE_CAL_X0_DEFAULT);
  if(EE_OK != eeprom_status  && EE_CLEANUP_REQUIRED != eeprom_status)
  {
    Error_Handler();
  }
  eeprom_status = EE_WriteVariable32bits(ELLIPSE_CAL_Y0_ADDR, ELLIPSE_CAL_Y0_DEFAULT);
  if(EE_OK != eeprom_status  && EE_CLEANUP_REQUIRED != eeprom_status)
  {
    Error_Handler();
  }

  if(EE_CLEANUP_REQUIRED == eeprom_status)
  {
    eeprom_status = EE_CleanUp();
    if(EE_OK != eeprom_status)
    {
      Error_Handler();
    }
  }

  #else

  EE_Status eeprom_status = EE_Init(EE_CONDITIONAL_ERASE);
  if (eeprom_status != EE_OK)
  {
  	Error_Handler();
  }

  for(ii = 0; ii < 6; ii++)
  {
    eeprom_status = EE_ReadVariable32bits(ELLIPSE_CAL_T00_ADDR + ii, &eeprom_init_vals[ii]);
    if(EE_OK != eeprom_status)
    {
      Error_Handler();
    }
  }

  #endif
  
  //----------------------------------------------------------------------------
  //                          Peripherals init
  //----------------------------------------------------------------------------
  
  //led handler timer
  led_handler_init(10);

  //-------------------------------- ADCs --------------------------------------
  adcs_init();

  //------------------------------- CORDIC -------------------------------------
  cordic_init();

  //---------------------------- ADC sync timer --------------------------------
  sync_timer_init(1);

  //----------------------------------------------------------------------------
  //                              Code init
  //----------------------------------------------------------------------------
  output_generator_init(PPR_OUT/POLE_PAIRS);
  
  struct input_calibration_private_vars_init_s input_calibration_private_vars_init_v = 
  {
    .revolutions_pre_acq  = N_TURNS_WAIT_CAL,
    .revolutions_todo     = N_TURNS_CAL
  };

  input_calibration_init(input_calibration_private_vars_init_v);

  //----------------------------------------------------------------------------
  //                          Read supply voltage
  //----------------------------------------------------------------------------

  //wait for voltage to settle (200ms)
  HAL_Delay(200);

  adc_vcc_acc = 0;
  for(ii = 0; ii < VCC_ADC_N_AVG; ii++)
  {
    ADC1->CR |= ADC_CR_ADEN_Msk;
    ADC1->CR |= ADC_CR_ADSTART_Msk;
    while(!(ADC1->ISR & ADC_ISR_EOC_Msk));
    adc_vcc_acc += (uint16_t)(ADC1->DR & 0x0FFF);
    HAL_Delay(1);
  }

  adc_vcc_cnt = (uint16_t)(adc_vcc_acc / VCC_ADC_N_AVG);
  vcc = (float)adc_vcc_cnt * 3.3f / 4096.0f / 0.0909f;

  //if supply voltage under the threshold
  if(vcc < VCC_CAL_THR)
  {
    //--------------------- begin normal operation -----------------------------
    cal_coeff_t tmp =
    {
      eeprom_init_vals[0],
      eeprom_init_vals[1],
      eeprom_init_vals[2],
      eeprom_init_vals[3],
      eeprom_init_vals[4],
      eeprom_init_vals[5],
    };

    output_generator_set_cal_coeff(tmp);

    set_system_status(SYSTEM_STATUS_RUNNING);
    
    //point ADC_IRQ to "read_adcs_output" function
    set_adc_irq_ptr(0);

    //start sampling
    sync_timer_start(0);

    //enable TIM4 CH1
    TIM4->CCER |= TIM_CCER_CC1E;

    //enable TIM4 CH1 ISR
    TIM4->DIER |= TIM_DIER_CC1IE_Msk;
  }
  else 
  {
    //-------------------- begin calibration operation -------------------------

    set_system_status(SYSTEM_STATUS_CALIBRATION);

    //point ADC_IRQ to "read_adcs_calibration" function
    set_adc_irq_ptr(1);

    //start sampling
    sync_timer_start(1);
  }

  adcs_en_injected_conversion();

  //----------------------------------------------------------------------------
  //                            Enable Led Timer
  //----------------------------------------------------------------------------
  TIM7->DIER |= TIM_DIER_UIE_Msk;
  TIM7->CR1  |= TIM_CR1_CEN_Msk;
  
  //----------------------------------------------------------------------------

  TIM6->CR1 |= TIM_CR1_CEN_Msk;

  //----------------------------------------------------------------------------
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  while (1)
  {
    exec_time_us = (float)(cnt)/(170.0f);

    if(SYSTEM_STATUS_CALIBRATION == get_system_status() && input_calibration_get_acquisition_done())
    {
      //calculate compensation coefficients
      cal_status = input_calibration_calculate_coefficients();
      if(cal_status)
      {
#if (EEPROM_WRITE && EEPROM_ERASE == 0)
        //store coefficients inside NVM parameters
        eeprom_status = EE_WriteVariable32bits(ELLIPSE_CAL_T00_ADDR, input_calibration_get_cal_coeff().t00);
        if(EE_OK != eeprom_status  && EE_CLEANUP_REQUIRED != eeprom_status)
        {
          Error_Handler();
        }
        eeprom_status = EE_WriteVariable32bits(ELLIPSE_CAL_T01_ADDR, input_calibration_get_cal_coeff().t01);
        if(EE_OK != eeprom_status  && EE_CLEANUP_REQUIRED != eeprom_status)
        {
          Error_Handler();
        }
        eeprom_status = EE_WriteVariable32bits(ELLIPSE_CAL_T10_ADDR, input_calibration_get_cal_coeff().t10);
        if(EE_OK != eeprom_status  && EE_CLEANUP_REQUIRED != eeprom_status)
        {
          Error_Handler();
        }
        eeprom_status = EE_WriteVariable32bits(ELLIPSE_CAL_T11_ADDR, input_calibration_get_cal_coeff().t11);
        if(EE_OK != eeprom_status  && EE_CLEANUP_REQUIRED != eeprom_status)
        {
          Error_Handler();
        }
        eeprom_status = EE_WriteVariable32bits(ELLIPSE_CAL_X0_ADDR, input_calibration_get_cal_coeff().x0);
        if(EE_OK != eeprom_status  && EE_CLEANUP_REQUIRED != eeprom_status)
        {
          Error_Handler();
        }
        eeprom_status = EE_WriteVariable32bits(ELLIPSE_CAL_Y0_ADDR, input_calibration_get_cal_coeff().y0);
        if(EE_OK != eeprom_status  && EE_CLEANUP_REQUIRED != eeprom_status)
        {
          Error_Handler();
        }
      
        if(EE_CLEANUP_REQUIRED == eeprom_status)
        {
          eeprom_status = EE_CleanUp();
          if(EE_OK != eeprom_status)
          {
            Error_Handler();
          }
        }
#endif
        set_system_status(SYSTEM_STATUS_CALIBRATION_DONE);
      }
      else 
      {
        set_system_status(SYSTEM_STATUS_ERROR);
      }
    }
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

  __disable_irq();

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
  isr_vec[16 + ADC1_2_IRQn]      			  = (uint32_t)(read_adcs_output);
  isr_vec[16 + TIM4_IRQn]      			    = (uint32_t)(update_out_pins);
  
  //relocate interrupt vector table
  SCB->VTOR = (uint32_t)(isr_vec);

  __DSB();
  __ISB();

  __enable_irq();

  return;
}

void init_ram()
{
  //copy memory regions to RAM
  memcpy(&_sdata,&_sidata,((void*)(&_edata)-(void*)(&_sdata))); //initialized variables
  memcpy(&_sccm_sram_code,&_si_ccm_sram_code,((void*)(&_eccm_sram_code)-(void*)(&_sccm_sram_code))); //fast code
  memcpy(&_sccm_sram_data,&_si_ccm_sram_data,((void*)(&_eccm_sram_data)-(void*)(&_sccm_sram_data))); //fast code variables
  memcpy(&_sccm_sram_rodata,&_si_ccm_sram_rodata,((void*)(&_eccm_sram_rodata)-(void*)(&_sccm_sram_rodata))); //fast code constants

  //reset uninitialized variables
  memset(&_sbss,0,((void*)(&_ebss)-(void*)(&_sbss)));
  return;
}

void set_adc_irq_ptr(uint8_t calibration)
{
  __disable_irq();
  if(calibration)
  {
    isr_vec[16 + ADC1_2_IRQn] = (uint32_t)(read_adcs_calibration);
  }
  else 
  {
    isr_vec[16 + ADC1_2_IRQn] = (uint32_t)(read_adcs_output);
  }

  __enable_irq();
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
