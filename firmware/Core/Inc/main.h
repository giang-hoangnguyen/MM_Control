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
#include "stm32f4xx_hal.h"

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
#define LED_MCU_Pin GPIO_PIN_5
#define LED_MCU_GPIO_Port GPIOE
#define STEP_DIR4_Pin GPIO_PIN_0
#define STEP_DIR4_GPIO_Port GPIOC
#define STEP_DIR3_Pin GPIO_PIN_1
#define STEP_DIR3_GPIO_Port GPIOC
#define STEP_DIR2_Pin GPIO_PIN_2
#define STEP_DIR2_GPIO_Port GPIOC
#define STEP_DIR1_Pin GPIO_PIN_3
#define STEP_DIR1_GPIO_Port GPIOC
#define STEP_ENABLE_Pin GPIO_PIN_5
#define STEP_ENABLE_GPIO_Port GPIOC
#define STEP_1_Pin GPIO_PIN_9
#define STEP_1_GPIO_Port GPIOE
#define STEP_2_Pin GPIO_PIN_11
#define STEP_2_GPIO_Port GPIOE
#define STEP_3_Pin GPIO_PIN_13
#define STEP_3_GPIO_Port GPIOE
#define STEP_4_Pin GPIO_PIN_14
#define STEP_4_GPIO_Port GPIOE
#define DC_F1_Pin GPIO_PIN_8
#define DC_F1_GPIO_Port GPIOD
#define DC_F2_Pin GPIO_PIN_9
#define DC_F2_GPIO_Port GPIOD
#define DC_F3_Pin GPIO_PIN_10
#define DC_F3_GPIO_Port GPIOD
#define DC_F4_Pin GPIO_PIN_11
#define DC_F4_GPIO_Port GPIOD
#define DC_11_Pin GPIO_PIN_12
#define DC_11_GPIO_Port GPIOD
#define DC_12_Pin GPIO_PIN_13
#define DC_12_GPIO_Port GPIOD
#define DC_21_Pin GPIO_PIN_14
#define DC_21_GPIO_Port GPIOD
#define DC_22_Pin GPIO_PIN_15
#define DC_22_GPIO_Port GPIOD
#define DC_31_Pin GPIO_PIN_6
#define DC_31_GPIO_Port GPIOC
#define DC_32_Pin GPIO_PIN_7
#define DC_32_GPIO_Port GPIOC
#define DC_41_Pin GPIO_PIN_8
#define DC_41_GPIO_Port GPIOC
#define DC_42_Pin GPIO_PIN_9
#define DC_42_GPIO_Port GPIOC
#define SERVO_TX_Pin GPIO_PIN_9
#define SERVO_TX_GPIO_Port GPIOA
#define SPI3_CS1_Pin GPIO_PIN_2
#define SPI3_CS1_GPIO_Port GPIOD
#define SPI3_CS2_Pin GPIO_PIN_3
#define SPI3_CS2_GPIO_Port GPIOD
#define SPI3_CS1D4_Pin GPIO_PIN_4
#define SPI3_CS1D4_GPIO_Port GPIOD
#define SPI1_CS1_Pin GPIO_PIN_8
#define SPI1_CS1_GPIO_Port GPIOB
#define SPI1_CS2_Pin GPIO_PIN_9
#define SPI1_CS2_GPIO_Port GPIOB
#define SPI1_CS3_Pin GPIO_PIN_0
#define SPI1_CS3_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
