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
#ifndef __DRIVER_AGV_INC_PID_H_
#define __DRIVER_AGV_INC_PID_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
#include "motor.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/**
	* @brief  PID Model Structure definition
*/
typedef struct {
	double Kp; /*!< Kp	: Proportional > */
	double Ki; /*!< Ki	: Integral > */
	double Kd; /*!< Kd	: Derivative > */
}PID_ModelTypeDef;

typedef struct {
	double pPart; 						/*!<  > */
	double iPart; 						/*!<  > */
	double dPart; 						/*!<  > */
	double preDpart;					/*!< > */
	double err;				//setpoint-getspeed
}PID_PartTypeDef;

typedef struct {
	int16_t pwm;			/*!< udk: PWM (-1000 -> +1000)> */
	double getspeed;		//rad/s
	double setpoint;		//rad/s
}PID_ParaTypeDef;

typedef struct {
	PID_ModelTypeDef	gain;		/*!< > */
	PID_PartTypeDef		part; 	/*!< > */
	PID_ParaTypeDef		para;
}PID_InitTypeDef;

typedef struct {
	PID_InitTypeDef mLeft; 		/*!< Control Motor LEFT > */
	PID_InitTypeDef mRight; 	/*!< Control Motor RIGHT > */
}PID_HandleTypeDef;
/* Exported functions prototypes ---------------------------------------------*/
void PID_Init(PID_HandleTypeDef *pid);
void PID_Run(PID_HandleTypeDef *pid);
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

