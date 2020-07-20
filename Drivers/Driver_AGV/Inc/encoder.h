/**
  ******************************************************************************
  * @file   :	encoder.h
  * @author :	HienND
  * @version:	v1_00 
  * @date   : 27/05/2020 
  * @brief  :	Header file for encoder.c module.
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
#ifndef __ENCODER_H
#define __ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
#include "main.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/
double GetSpeed_LEFT(void);
double GetSpeed_RIGHT(void);

void EncoderStart_LEFT(TIM_HandleTypeDef *htim);
void EncoderStart_RIGHT(TIM_HandleTypeDef *htim);
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

