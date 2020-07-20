/**
  ******************************************************************************
  * @file   :	motor.h
  * @author :	HienND
  * @version:	v1_00
  * @date   : 27/05/2020
  * @brief  :	Header file for motor.c module.
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
#ifndef __DRIVER_AGV_INC_ESTIMATION_H_
#define __DRIVER_AGV_INC_ESTIMATION_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
#include "main.h"
#include "robot.h"
#include "uart.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Initialization and de-initialization functions ----------------------------*/
/* IO operation functions ----------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
#define MAX_DUTY_PWM_EST		1000	//for Estimation
/* Private functions ---------------------------------------------------------*/
int16_t PulseGeneratorSignalPWM(int16_t maxPulse, uint16_t periodTimeSecond);
int16_t SinGeneratorSignalPWM(uint16_t stepTimeS, uint8_t inverse);
void Estimation(ROBOT_HandleTypeDef *hRobot);

#ifdef __cplusplus
}
#endif

#endif /* __H */

/******************* (C) COPYRIGHT 2020 hiennd *****END OF FILE****/

