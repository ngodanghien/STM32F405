/**
  ******************************************************************************
  * @file   :	str.h
  * @author :	HienND
  * @version:	v1_00
  * @date   :   Jun 24, 2020
  * @brief  :	Header file for motor.c module.
  ******************************************************************************
  ******************************************************************************
  */
  /*
  History
  ------------------------------------------------------------------------------
  version  		author			date		    description
  ------------------------------------------------------------------------------
  v1.00			  HienND	   Jun 24, 2020
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRIVER_AGV_INC_STR_H_
#define __DRIVER_AGV_INC_STR_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
//#include "main.h"
/* Exported types ------------------------------------------------------------*/
typedef struct __STR_HandleTypeDef
{
	//ex
	double Value;
}STR_HandleTypeDef;
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
/* Private functions ---------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __H */

/******************* (C) COPYRIGHT 2020 hiennd *****END OF FILE****/

