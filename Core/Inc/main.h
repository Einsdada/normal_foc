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
#define Encoder_A_Pin GPIO_PIN_0
#define Encoder_A_GPIO_Port GPIOA
#define Encoder_B_Pin GPIO_PIN_1
#define Encoder_B_GPIO_Port GPIOA
#define Encoder_mode_Pin GPIO_PIN_2
#define Encoder_mode_GPIO_Port GPIOA
#define Encoder_push_Pin GPIO_PIN_3
#define Encoder_push_GPIO_Port GPIOA
#define Encoder_HALL_V_Pin GPIO_PIN_4
#define Encoder_HALL_V_GPIO_Port GPIOA
#define Encoder_Z_Pin GPIO_PIN_5
#define Encoder_Z_GPIO_Port GPIOA
#define Encoder_HALL_U_Pin GPIO_PIN_6
#define Encoder_HALL_U_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_7
#define LED_GPIO_Port GPIOA
#define Encoder_ANALOG_Pin GPIO_PIN_4
#define Encoder_ANALOG_GPIO_Port GPIOC
#define Encoder_HALL_W_Pin GPIO_PIN_0
#define Encoder_HALL_W_GPIO_Port GPIOB
#define SOA_Pin GPIO_PIN_1
#define SOA_GPIO_Port GPIOB
#define V_Bus_Pin GPIO_PIN_2
#define V_Bus_GPIO_Port GPIOB
#define SOB_Pin GPIO_PIN_11
#define SOB_GPIO_Port GPIOB
#define SOC_Pin GPIO_PIN_12
#define SOC_GPIO_Port GPIOB
#define Motor_PWM_AN_Pin GPIO_PIN_13
#define Motor_PWM_AN_GPIO_Port GPIOB
#define Motor_PWM_BN_Pin GPIO_PIN_14
#define Motor_PWM_BN_GPIO_Port GPIOB
#define Motor_PWM_CN_Pin GPIO_PIN_15
#define Motor_PWM_CN_GPIO_Port GPIOB
#define Motro_fault_Pin GPIO_PIN_6
#define Motro_fault_GPIO_Port GPIOC
#define Motor_PWM_A_Pin GPIO_PIN_8
#define Motor_PWM_A_GPIO_Port GPIOA
#define Motor_PWM_B_Pin GPIO_PIN_9
#define Motor_PWM_B_GPIO_Port GPIOA
#define Motor_PWM_C_Pin GPIO_PIN_10
#define Motor_PWM_C_GPIO_Port GPIOA
#define Motor_sleep_Pin GPIO_PIN_11
#define Motor_sleep_GPIO_Port GPIOA
#define Motor_mode_Pin GPIO_PIN_12
#define Motor_mode_GPIO_Port GPIOA
#define Encoder_CSN_Pin GPIO_PIN_15
#define Encoder_CSN_GPIO_Port GPIOA
#define Encoder_CLK_Pin GPIO_PIN_10
#define Encoder_CLK_GPIO_Port GPIOC
#define Encoder_DO_Pin GPIO_PIN_11
#define Encoder_DO_GPIO_Port GPIOC
#define TEXT3_Pin GPIO_PIN_3
#define TEXT3_GPIO_Port GPIOB
#define TEXT2_Pin GPIO_PIN_4
#define TEXT2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
