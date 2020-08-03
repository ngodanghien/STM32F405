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
#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
#include "encoder.h"
#include "pwm.h"
/* Exported types ------------------------------------------------------------*/

/**
	* @brief  MOTOR Model Structure definition
*/
/*!< ENC_ViewTypeDef: Xem tốc độ Encoder của từng bánh xe ở các chế độ khác nhau > */
typedef struct {
	double RADs;	/*!< rad/s > */
	double RPM;		/*!< rpm > */
	double Hz;		/*!< Hz > */
}ENC_ViewTypeDef;
typedef struct {
	void (* Start)(TIM_HandleTypeDef *htim);
	double (* GetSpeed)(void); //double GetSpeed_LEFT(void) vs double GetSpeed_RIGHT(void)
}Encoder_InitTypeDef;

typedef struct {
	//TIM_HandleTypeDef timPwm; /*!<  > */
	/*!<  > */
	void (* Start)(TIM_HandleTypeDef *htim, uint32_t Channel);
	void (* SetPwm) (int16_t udk);
}PWM_InitTypeDef;

typedef struct {
	int16_t	cur_pwm;	//1000=100%
	double	cur_speed;	//rad/s
	ENC_ViewTypeDef view;	/*!< Xem kết quả của Encoder > */
}Para_InitTyeDef;
typedef struct {
	Encoder_InitTypeDef Encoder; 	/*!< Init Encoder> */
	PWM_InitTypeDef PWM; 			/*!< Init PWM > */
	Para_InitTyeDef para;
}Motor_InitTypeDef;

typedef struct __Motor_HandleTypeDef
{
	Motor_InitTypeDef mLEFT; 		/*!<  > */
	Motor_InitTypeDef mRIGHT;		/*!<  > */
}Motor_HandleTypeDef;



/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define PI						3.1415926535897932384626433832795
#define PPI						6.28318530717959		//2*PI
#define TIME_SAMPE 				0.005	//S
#define RADIUS_WHEEL			0.0625 	//Robot wheel radius = 0.125/2 = 0.0625 (m)
#define D2WHEEL					0.37	//D2W = L = distance 2 wheel = 0.37 (m)
#define MAX_PWM					999

/* Giá trị được TEST lúc pin xạc đầy, ngày 23/06/2020 */
#define MAX_RADs_LEFT_UP			39.0
#define MAX_RADs_LEFT_DOWN			-39.5
#define MAX_RADs_RIGHT_UP			38.0
#define MAX_RADs_RIGHT_DOWN			-40.5
//Đã khai báo bên: robot.h --> để đây xem cho tiện.
//#define MAX_ROBOT_VELOCITY_UP		2.4		// m/s
//#define MAX_ROBOT_VELOCITY_DOWN	-2.4	// m/s
//#define MAX_ROBOT_ANGULAR_UP 		13.6	// rad/s
//#define MAX_ROBOT_ANGULAR_DOWN	-13.6	// rad/s
/* Exported functions prototypes ---------------------------------------------*/
void Motor_Init(Motor_HandleTypeDef *hMotor);
void MOTOR_SetPWM(Motor_HandleTypeDef *hMotor);
void MOTOR_GetSpeed(Motor_HandleTypeDef *hMotor);
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

