/**
  ******************************************************************************
  * @file   :	control.h
  * @author :	HienND
  * @version:	v1_00
  * @date   : 	Jun 24, 2020
  * @brief  :	Header file for control.c module.
  ******************************************************************************
  ******************************************************************************
  */
  /*
  History
  ------------------------------------------------------------------------------
  version  		author			date		    description
  ------------------------------------------------------------------------------
  v1.00			  HienND			Jun 24, 2020
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRIVER_AGV_INC_CONTROL_H_
#define __DRIVER_AGV_INC_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
#include "pid.h"
#include "str.h"
/* Exported types ------------------------------------------------------------*/
typedef struct __CONTROL_HandleTypeDef{
	PID_HandleTypeDef 	*cPID;
	//Fuzzy_HandleTypeDef *cFuzzy;
	STR_HandleTypeDef	*cSTR;

	//function callbacks
//	void (* InitControl)(void);
//	void (* RUN)(void);
}CONTROL_HandleTypeDef;

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

#endif /* DRIVER_AGV_INC_CONTROL_H_ */

/******************* (C) COPYRIGHT 2020 hiennd *****END OF FILE****/

