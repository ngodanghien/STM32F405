/**
  ******************************************************************************
  * @file   :	sys.h
  * @author :	HienND
  * @version:	v1_00
  * @date   : 	Jun 24, 2020
  * @brief  :	Header file for motor.c module.
  ******************************************************************************
  ******************************************************************************
  */
  /*
  History
  ------------------------------------------------------------------------------
  version  		author			date		    description
  ------------------------------------------------------------------------------
  v1.00			  HienND		Jun 24, 2020
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRIVER_AGV_INC_SYS_H_
#define __DRIVER_AGV_INC_SYS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
#include "main.h"
/* Exported types ------------------------------------------------------------*/
typedef enum
{
  DRV_OK       = 0x00U,
  DRV_ERROR    = 0x01U,
  DRV_BUSY     = 0x02U,
  DRV_TIMEOUT  = 0x03U
} DRV_StatusTypeDef;	//DRV = DRIVER

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
//#define USER_LED_Pin GPIO_PIN_13
/* Exported functions prototypes ---------------------------------------------*/
void Error_DriverHandler(void);
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

