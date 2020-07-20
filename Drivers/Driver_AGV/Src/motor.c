/**
  ******************************************************************************
  * @file    :motor.c
  * @author  :HienND
  * @version :v1_00
  * @date    :27/05/2020
	* @brief   :
  *          Board H-Brigde use FET (+IC): BTS79xx (796x,797x ..) 
  *          Link: https://item.taobao.com/item.htm?spm=a1z10.5-c-s.w4002-15726392046.36.472c7b0b2IjhmD&id=551092190133  
  *          Logic: The same with L298N
  *           
  ******************************************************************************
	 @History
  ------------------------------------------------------------------------------
  version  		author			date		      description
  ------------------------------------------------------------------------------
  v1.00			  HienND			27/05/2020
	------------------------------------------------------------------------------
**/	

/* Includes ------------------------------------------------------------------*/
#include "motor.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void MOTOR_InitRegisterCallbacks(Motor_HandleTypeDef *hMotor);
void MOTOR_Start(Motor_HandleTypeDef *hMotor);	//Start of HAL_Lib
void MOTOR_ViewENC(Motor_HandleTypeDef *hMotor);
/* External variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void Motor_Init(Motor_HandleTypeDef *hMotor);
void MOTOR_GetSpeed(Motor_HandleTypeDef *hMotor);
void MOTOR_SetPWM(Motor_HandleTypeDef *hMotor);
/**		[PASSED]
  * @brief  Init Driver Motor
  * @param  Motor_HandleTypeDef *hMotor
  * @retval void
  */
void Motor_Init(Motor_HandleTypeDef *hMotor)
{
	MOTOR_InitRegisterCallbacks(hMotor);
	MOTOR_Start(hMotor);
	MOTOR_SetPWM(hMotor);
	MOTOR_GetSpeed(hMotor);	//Demo view
}
/**		[PASSED]
  * @brief  Initialize the callbacks to their default values.
  * @param  Motor handle.
  * @retval none
  */
void MOTOR_InitRegisterCallbacks(Motor_HandleTypeDef *hMotor)
{
  /* Init the MOTOR Callback settings */
	hMotor->mLEFT.Encoder.Start 	= EncoderStart_LEFT; //ref EncoderStart_LEFT(TIM_HandleTypeDef *htim)
	hMotor->mLEFT.Encoder.GetSpeed 	= GetSpeed_LEFT;	//ref double GetSpeed_LEFT(void);
	hMotor->mLEFT.PWM.Start 		= PWM_Start_LEFT;
	hMotor->mLEFT.PWM.SetPwm 		= SetPwmLEFT;

	hMotor->mRIGHT.Encoder.Start 	= EncoderStart_RIGHT;
	hMotor->mRIGHT.Encoder.GetSpeed = GetSpeed_RIGHT;
	hMotor->mRIGHT.PWM.Start 		= PWM_Start_RIGHT;
	hMotor->mRIGHT.PWM.SetPwm 		= SetPwmRIGHT;
}
/**		[PASSED]
  * @brief  MOTOR_Start	: Init ban đầu cho PWM và Encoder; Init Msp đang để mặc định từ STM32CubeMx
  * @param  Motor_HandleTypeDef *hMotor
  * @retval void
  */
void MOTOR_Start(Motor_HandleTypeDef *hMotor)
{
	hMotor->mLEFT.Encoder.Start(&htim2);		//HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);	//Motor A (Left)
	hMotor->mRIGHT.Encoder.Start(&htim3);	//HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);	//Motor B (Right)
	// -- For PWM Start---
	hMotor->mLEFT.PWM.Start(&htim8, TIM_CHANNEL_1);	//HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	hMotor->mRIGHT.PWM.Start(&htim8, TIM_CHANNEL_2);	//HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	//-- Init Motor STOP ----
	hMotor->mLEFT.PWM.SetPwm(0);
	hMotor->mRIGHT.PWM.SetPwm(0);
}
/**		[PASSED]
  * @brief  MOTOR_ViewENC
  * @param  Motor_HandleTypeDef *hMotor
  * @retval void
  */
void MOTOR_ViewENC(Motor_HandleTypeDef *hMotor)
{
	hMotor->mLEFT.para.view.RADs	= hMotor->mLEFT.para.cur_speed; 		//Cần update .get_speed trước.
	hMotor->mLEFT.para.view.Hz		= hMotor->mLEFT.para.view.RADs/(2*PI);	//Hz = rad/2pi
	hMotor->mLEFT.para.view.RPM		= hMotor->mLEFT.para.view.Hz*60; 		//(RPM)
	//
	hMotor->mRIGHT.para.view.RADs	= hMotor->mRIGHT.para.cur_speed; 		//Cần update .get_speed trước.
	hMotor->mRIGHT.para.view.Hz		= hMotor->mRIGHT.para.view.RADs/(2*PI);	//Hz = rad/2pi
	hMotor->mRIGHT.para.view.RPM	= hMotor->mRIGHT.para.view.Hz*60; 		//(RPM)
}
/**		[PASSED]
  * @brief  MOTOR_GetSpeed	: Tốc độ động cơ, được tính với thời gian lấy mẫu 5ms
  * @param  Motor_HandleTypeDef *hMotor
  * @retval void
  */
void MOTOR_GetSpeed(Motor_HandleTypeDef *hMotor)
{
	hMotor->mLEFT.para.cur_speed	= hMotor->mLEFT.Encoder.GetSpeed();
	hMotor->mRIGHT.para.cur_speed	= hMotor->mRIGHT.Encoder.GetSpeed();
	//Update view.
	MOTOR_ViewENC(hMotor);
}
/**		[PASSED]
  * @brief  MOTOR_SetPWM	: Motor chạy giá trị pwm được lưu trong .para.cur_pwm
  * @param  Motor_HandleTypeDef *hMotor
  * @retval void
  */
void MOTOR_SetPWM(Motor_HandleTypeDef *hMotor)
{
	hMotor->mLEFT.PWM.SetPwm(hMotor->mLEFT.para.cur_pwm);
	hMotor->mRIGHT.PWM.SetPwm(hMotor->mRIGHT.para.cur_pwm);
}
/******************* (C) COPYRIGHT 2020 hiennd *****END OF FILE****/

