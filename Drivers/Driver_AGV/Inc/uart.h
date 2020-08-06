/**
  ******************************************************************************
  * @file   :	uart.h
  * @author :	HienND
  * @version:	v1_00
  * @date   : 	Jun 19, 2020
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
#ifndef DRIVER_AGV_INC_UART_H_
#define DRIVER_AGV_INC_UART_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
#include "main.h"
#include "uart.h"
#include "stdio.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/
void UartTX_Float(const float *arrTx, int lengthF);
void UartTX_Double(const double *arrTx, int lengthD);

void UartRX_Float(float *result, uint8_t *buffRx, int lengthF);
void UartTX_Float_aHeader(const float *arrTx, int lengthF);
void UartRX_aFloat(float *result, uint8_t *buffRx, int lengthF);
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

