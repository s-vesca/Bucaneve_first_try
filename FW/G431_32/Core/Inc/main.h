/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_cordic.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TIM7_PSC 0
#define TIM7_ARR 16999
#define ADC_OFFSET_DEFAULT 2048
#define TIM2_ARR 213
#define COS_P_Pin LL_GPIO_PIN_0
#define COS_P_GPIO_Port GPIOA
#define COS_N_Pin LL_GPIO_PIN_1
#define COS_N_GPIO_Port GPIOA
#define VDC_SENSE_Pin LL_GPIO_PIN_3
#define VDC_SENSE_GPIO_Port GPIOA
#define SIN_P_Pin LL_GPIO_PIN_6
#define SIN_P_GPIO_Port GPIOA
#define SIN_N_Pin LL_GPIO_PIN_7
#define SIN_N_GPIO_Port GPIOA
#define DBG_LED2_Pin LL_GPIO_PIN_8
#define DBG_LED2_GPIO_Port GPIOA
#define DBG_LED1_Pin LL_GPIO_PIN_9
#define DBG_LED1_GPIO_Port GPIOA
#define ERR_LED_Pin LL_GPIO_PIN_10
#define ERR_LED_GPIO_Port GPIOA
#define INT_A_Pin LL_GPIO_PIN_11
#define INT_A_GPIO_Port GPIOA
#define INT_B_Pin LL_GPIO_PIN_12
#define INT_B_GPIO_Port GPIOA
#define SWDIO_Pin LL_GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin LL_GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SWO_Pin LL_GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
