/**
  ******************************************************************************
  * @file   :	robot.h
  * @author :	HienND
  * @version:	v1_00
  * @date   : 	Jun 22, 2020
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
#ifndef __DRIVER_AGV_INC_ROBOT_H_
#define __DRIVER_AGV_INC_ROBOT_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
//#include "main.h"
#include "sys.h"
#include "motor.h"
#include "uart.h"
#include "pid.h"
/* Exported types ------------------------------------------------------------*/

/**
  * @brief ROBOT Init Structure definition
  */
typedef struct
{
	double set_wheel_velocity;	//(m/s): Vận tốc dài của bánh xe tính được từ robot_velocity
	double cur_wheel_velocity;	//(m/s): Vận tốc dài của bánh xe đo được !

	double set_wheel_angular;	//rad/s: Chính bằng: cur_setpoint (tốc độ góc mong muốn)
	double cur_wheel_angular; 	//rad/s: Chính bằng:  getspeed = tốc độ góc của bánh xe đo được.

	int16_t cur_pwm;			//Giá trị PWM đang được băm xuống cầu H
}ROBOT_ParaViewTypeDef;

typedef struct
{
	double set_robot_velocity;		/*!< v - tangential velocity (m/s) : Tốc độ dài (m/s) > */
	double set_robot_angular; 		/*!< w - angular velocity (rad/s) : tốc độ góc (rad/s)> */

	double cur_robot_velocity;		/*!< v - tangential velocity (m/s) : Tốc độ dài (m/s) > */
	double cur_robot_angular; 		/*!< w - angular velocity (rad/s) : tốc độ góc (rad/s)> */

	ROBOT_ParaViewTypeDef wheelLeft;
	ROBOT_ParaViewTypeDef wheelRight;

} ROBOT_InitTypeDef;
/**
  * @brief  ROBOT handle Structure definition
  */
typedef struct __ROBOT_HandleTypeDef
{
	ROBOT_InitTypeDef              	Value;			/*!<   >*/
	Motor_HandleTypeDef				*hMotor;		/*!< Motor LEFT + RIGHT  */
	//CONTROL_HandleTypeDef			*hControl;		/*!< Controller Robot : PID, STR, Fuzzy ...  >*/
	PID_HandleTypeDef				*hPID;
	//USE_HAL_UART_REGISTER_CALLBACKS
	void (* InitMotor)(struct __Motor_HandleTypeDef *hMotor); //ref: void Motor_Init(Motor_HandleTypeDef *hMotor);
	void (* GetSpeed)(struct __Motor_HandleTypeDef *hMotor); //ref: void MOTOR_GetSpeed(Motor_HandleTypeDef *hMotor);
	void (* SetSpeed)(struct __Motor_HandleTypeDef *hMotor); //ref: void MOTOR_Run(Motor_HandleTypeDef *hMotor);
	//void (* )(struct __ROBOT_HandleTypeDef *hRobot);
}ROBOT_HandleTypeDef;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define __ROBOT_LINK_HANDLE(__HANDLE_PARENT__, __PPP_FIELD__, __HANDLE__)          	\
                        do{                                                      	\
                              (__HANDLE_PARENT__)->__PPP_FIELD__ = &(__HANDLE__);	\
                          } while(0U)

#define ROBOT_ERROR_NONE             	 0x00000000U   /*!< No error            */

/* Exported functions prototypes ---------------------------------------------*/
DRV_StatusTypeDef ROBOT_Init(ROBOT_HandleTypeDef *hRobot);
void ROBOT_SetSpeed(ROBOT_HandleTypeDef *hRobot);
void ROBOT_GetSpeed(ROBOT_HandleTypeDef *hRobot);
	//Dùng cho các mục đích điều khiển set PWM trực tiếp !
void ROBOT_SetPWMtoMotor(ROBOT_HandleTypeDef *hRobot, int16_t pwmLeft, int16_t pwmRight);
void ROBOT_CONTROL_PID_Init(ROBOT_HandleTypeDef *hRobot);
void ROBOT_CONTROL_PID_Run(ROBOT_HandleTypeDef *hRobot);
/* Private defines -----------------------------------------------------------*/

/* Initialization and de-initialization functions ----------------------------*/
/* IO operation functions ----------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
#define MAX_ROBOT_VELOCITY_UP		2.4		// m/s
#define MAX_ROBOT_VELOCITY_DOWN		-2.4	// m/s
#define MAX_ROBOT_ANGULAR_UP 		13.6	// rad/s
#define MAX_ROBOT_ANGULAR_DOWN	 	-13.6	// rad/s
/* Private functions ---------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_AGV_INC_ROBOT_H_ */

/******************* (C) COPYRIGHT 2020 hiennd *****END OF FILE****/

