/**
  ******************************************************************************
  * @file   :	pwm.h
  * @author :	HienND
  * @version:	v1_00 
  * @date   : 27/05/2020 
  * @brief  :	Header file for pwm.c module.
  ******************************************************************************
  ******************************************************************************  
  */
  /*
  History
  ------------------------------------------------------------------------------
  version  		author			date		    description
  ------------------------------------------------------------------------------
  v1.00			  HienND			27/05/2020
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PWM_H
#define __PWM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
#include "main.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/
void SetPwmLEFT(int16_t udk);
void SetPwmRIGHT(int16_t udk);
void PWM_Start_LEFT(TIM_HandleTypeDef *htim, uint32_t Channel);
void PWM_Start_RIGHT(TIM_HandleTypeDef *htim, uint32_t Channel);
/* Private defines -----------------------------------------------------------*/

/* Initialization and de-initialization functions ----------------------------*/
/* IO operation functions ----------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __H */

/******************* (C) COPYRIGHT 2020 hiennd *****END OF FILE****/
