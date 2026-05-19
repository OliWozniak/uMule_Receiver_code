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
#define HC_SR_04_Pin GPIO_PIN_14
#define HC_SR_04_GPIO_Port GPIOC
#define UART_TX_Ex_LiDaR_Pin GPIO_PIN_2
#define UART_TX_Ex_LiDaR_GPIO_Port GPIOA
#define UART_RX_Ex_LiDaR_Pin GPIO_PIN_3
#define UART_RX_Ex_LiDaR_GPIO_Port GPIOA
#define UART_TX_Ex_ESP_Pin GPIO_PIN_4
#define UART_TX_Ex_ESP_GPIO_Port GPIOC
#define UART_RX_Ex_ESP_Pin GPIO_PIN_5
#define UART_RX_Ex_ESP_GPIO_Port GPIOC
#define RS_485_FC_Pin GPIO_PIN_2
#define RS_485_FC_GPIO_Port GPIOB
#define RS_485_TX_Pin GPIO_PIN_10
#define RS_485_TX_GPIO_Port GPIOB
#define RS_485_RX_Pin GPIO_PIN_11
#define RS_485_RX_GPIO_Port GPIOB
#define DBG_LED0_Pin GPIO_PIN_3
#define DBG_LED0_GPIO_Port GPIOB
#define DBG_LED1_Pin GPIO_PIN_4
#define DBG_LED1_GPIO_Port GPIOB
#define DBG_LED2_Pin GPIO_PIN_5
#define DBG_LED2_GPIO_Port GPIOB
#define HC_SR_01_Pin GPIO_PIN_6
#define HC_SR_01_GPIO_Port GPIOB
#define HC_SR_02_Pin GPIO_PIN_7
#define HC_SR_02_GPIO_Port GPIOB
#define HC_SR_03_Pin GPIO_PIN_8
#define HC_SR_03_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
