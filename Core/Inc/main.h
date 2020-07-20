/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OLED_SCL_Pin GPIO_PIN_13
#define OLED_SCL_GPIO_Port GPIOC
#define OLED_SDA_Pin GPIO_PIN_14
#define OLED_SDA_GPIO_Port GPIOC
#define OLED_RES_Pin GPIO_PIN_15
#define OLED_RES_GPIO_Port GPIOC
#define OLED_DC_Pin GPIO_PIN_0
#define OLED_DC_GPIO_Port GPIOC
#define PS2_DO_Pin GPIO_PIN_1
#define PS2_DO_GPIO_Port GPIOC
#define PS2_DI_Pin GPIO_PIN_2
#define PS2_DI_GPIO_Port GPIOC
#define PS2_CS_Pin GPIO_PIN_3
#define PS2_CS_GPIO_Port GPIOC
#define ENCD_A_Pin GPIO_PIN_0
#define ENCD_A_GPIO_Port GPIOA
#define ENCD_B_Pin GPIO_PIN_1
#define ENCD_B_GPIO_Port GPIOA
#define WIFI_BLE_UART2_TX_Pin GPIO_PIN_2
#define WIFI_BLE_UART2_TX_GPIO_Port GPIOA
#define WIFI_BLE_UART2_RX_Pin GPIO_PIN_3
#define WIFI_BLE_UART2_RX_GPIO_Port GPIOA
#define PS2_CLK_Pin GPIO_PIN_4
#define PS2_CLK_GPIO_Port GPIOA
#define USER_ADC_Pin GPIO_PIN_5
#define USER_ADC_GPIO_Port GPIOA
#define ENCB_A_Pin GPIO_PIN_6
#define ENCB_A_GPIO_Port GPIOA
#define ENCB_B_Pin GPIO_PIN_7
#define ENCB_B_GPIO_Port GPIOA
#define DIR_A_Pin GPIO_PIN_4
#define DIR_A_GPIO_Port GPIOC
#define DIR_C_Pin GPIO_PIN_5
#define DIR_C_GPIO_Port GPIOC
#define DIR_B_Pin GPIO_PIN_0
#define DIR_B_GPIO_Port GPIOB
#define DIR_D_Pin GPIO_PIN_1
#define DIR_D_GPIO_Port GPIOB
#define IMU_I2C2_SCL_Pin GPIO_PIN_10
#define IMU_I2C2_SCL_GPIO_Port GPIOB
#define IMU_I2C2_SDA_Pin GPIO_PIN_11
#define IMU_I2C2_SDA_GPIO_Port GPIOB
#define USER_SW_Pin GPIO_PIN_12
#define USER_SW_GPIO_Port GPIOB
#define USER_LED_Pin GPIO_PIN_13
#define USER_LED_GPIO_Port GPIOB
#define USER_BUTTON_Pin GPIO_PIN_14
#define USER_BUTTON_GPIO_Port GPIOB
#define IMU_INT_EXTI15_Pin GPIO_PIN_15
#define IMU_INT_EXTI15_GPIO_Port GPIOB
#define IMU_INT_EXTI15_EXTI_IRQn EXTI15_10_IRQn
#define PWMA_Pin GPIO_PIN_6
#define PWMA_GPIO_Port GPIOC
#define PWMB_Pin GPIO_PIN_7
#define PWMB_GPIO_Port GPIOC
#define PWMC_Pin GPIO_PIN_8
#define PWMC_GPIO_Port GPIOC
#define PWMD_Pin GPIO_PIN_9
#define PWMD_GPIO_Port GPIOC
#define CH340_UART1_TX_Pin GPIO_PIN_9
#define CH340_UART1_TX_GPIO_Port GPIOA
#define CH340_UART1_RX_Pin GPIO_PIN_10
#define CH340_UART1_RX_GPIO_Port GPIOA
#define ENCA_A_Pin GPIO_PIN_15
#define ENCA_A_GPIO_Port GPIOA
#define USER_UART3_TX_Pin GPIO_PIN_10
#define USER_UART3_TX_GPIO_Port GPIOC
#define USER_UART3_RX_Pin GPIO_PIN_11
#define USER_UART3_RX_GPIO_Port GPIOC
#define ENCA_B_Pin GPIO_PIN_3
#define ENCA_B_GPIO_Port GPIOB
#define ENCC_A_Pin GPIO_PIN_6
#define ENCC_A_GPIO_Port GPIOB
#define ENCC_B_Pin GPIO_PIN_7
#define ENCC_B_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
