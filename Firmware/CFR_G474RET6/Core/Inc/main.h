/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define PwrEn_Pin GPIO_PIN_13
#define PwrEn_GPIO_Port GPIOC
#define ENC_A_Pin GPIO_PIN_1
#define ENC_A_GPIO_Port GPIOC
#define ENC_B_Pin GPIO_PIN_2
#define ENC_B_GPIO_Port GPIOC
#define ENC_Z_Pin GPIO_PIN_3
#define ENC_Z_GPIO_Port GPIOC
#define U_Imes_Pin GPIO_PIN_1
#define U_Imes_GPIO_Port GPIOA
#define IC_PWM_U_H_Pin GPIO_PIN_2
#define IC_PWM_U_H_GPIO_Port GPIOA
#define IC_PWM_U_L_Pin GPIO_PIN_3
#define IC_PWM_U_L_GPIO_Port GPIOA
#define USR_BTN_2_Pin GPIO_PIN_4
#define USR_BTN_2_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_10
#define VCP_RX_GPIO_Port GPIOB
#define VCP_TX_Pin GPIO_PIN_11
#define VCP_TX_GPIO_Port GPIOB
#define IC_PWM_V_L_Pin GPIO_PIN_7
#define IC_PWM_V_L_GPIO_Port GPIOC
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define USR_LED_4_Pin GPIO_PIN_15
#define USR_LED_4_GPIO_Port GPIOA
#define USR_LED_3_Pin GPIO_PIN_10
#define USR_LED_3_GPIO_Port GPIOC
#define USR_LED_2_Pin GPIO_PIN_11
#define USR_LED_2_GPIO_Port GPIOC
#define USR_LED_1_Pin GPIO_PIN_12
#define USR_LED_1_GPIO_Port GPIOC
#define USR_BTN_1_Pin GPIO_PIN_2
#define USR_BTN_1_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define ISO_PWM_Fan_Pin GPIO_PIN_4
#define ISO_PWM_Fan_GPIO_Port GPIOB
#define USR_BTN_3_Pin GPIO_PIN_6
#define USR_BTN_3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */