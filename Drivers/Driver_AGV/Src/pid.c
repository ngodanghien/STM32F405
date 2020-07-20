/**
  ******************************************************************************
  * @file    :pid.c
  * @author  :HienND
  * @version :v1_00
  * @date    :27/05/2020
	* @brief   :
  *
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
#include "pid.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void PID_Init(PID_HandleTypeDef *pid);
void PID_InitRegisterCallbacks(PID_HandleTypeDef *pid);
//void PID_Contol(PID_HandleTypeDef *hMotor, double refL, double refR);
/* External variables --------------------------------------------------------*/
extern PID_HandleTypeDef cPID;
/* Exported functions --------------------------------------------------------*/

void PID_GainInit(PID_HandleTypeDef *pid)
{
	//MotorLEFT									//  Rise time	: 0.46s
	pid->mLeft.gain.Kp = 7.91171858465162; //4.38148531613524; 		// 	Setting time: 0.825s
	pid->mLeft.gain.Ki = 500;//256.583282731306; //122.740679932892; 		// 	Overshoot	:	0%
	pid->mLeft.gain.Kd = 0.0181756509445584;//0.0101865840407575;	//	Gain margin	: 36.7 dB (271%)

	pid->mLeft.part.pPart		= 0;
	pid->mLeft.part.iPart 		= 0;
	pid->mLeft.part.dPart 		= 0;
	pid->mLeft.part.preDpart 	= 0;

	//MotorRIGHT
	pid->mRight.gain.Kp = 7.91171858465162;//4.46609358003201;
	pid->mRight.gain.Ki = 500;//124.506858089363;
	pid->mRight.gain.Kd = 0.0181756509445584;//0.0103870660870215;	//overshot

	pid->mRight.part.pPart 		= 0;
	pid->mRight.part.iPart 		= 0;
	pid->mRight.part.dPart 		= 0;
	pid->mRight.part.preDpart 	= 0;
}
/**
  * @brief  Init parametter for PID_Control
  * @param  PID_HandleTypeDef *pid
  * @retval void
  */
void PID_Init(PID_HandleTypeDef *pid)
{
	PID_InitRegisterCallbacks(pid);
	PID_GainInit(pid);
}
void PID_InitRegisterCallbacks(PID_HandleTypeDef *pid)
{
	// TBD !!
}
/**
  * @brief  PID_Control
  * 	- Lấy giá trị tốc độ tai: 	hPID->mLeft.para.getspeed;
  * 	- Set PWM ở tại			:	hPID->mRight.para.pwm;
  * @param  PID_HandleTypeDef *pid; refL: setpoint MotorL; refR: setpoint MotorR
  * @retval void
  */
void PID_Run(PID_HandleTypeDef *pid)	// rad/s
{
	static double Ts = 0.005;		/*!< Time sampel (second), ex: 0.005 > */
	/*
		1. Read Encoder (rad/s)
		2. Calc PID Control => udk (pwm)
		3. Set PWM (Duty), -1000 - 1000
	*/
	//Check Limit
	if 			(pid->mLeft.para.setpoint > MAX_RADs_LEFT_UP) 		pid->mLeft.para.setpoint = MAX_RADs_LEFT_UP;
	else if (pid->mLeft.para.setpoint < MAX_RADs_LEFT_DOWN) 	pid->mLeft.para.setpoint = MAX_RADs_LEFT_DOWN;

	if 			(pid->mRight.para.setpoint > MAX_RADs_RIGHT_UP) 		pid->mRight.para.setpoint = MAX_RADs_RIGHT_UP;
	else if (pid->mRight.para.setpoint < MAX_RADs_RIGHT_DOWN) 	pid->mRight.para.setpoint = MAX_RADs_RIGHT_DOWN;

	//MotorLEFT
	pid->mLeft.part.err = (pid->mLeft.para.setpoint - pid->mLeft.para.getspeed);

	pid->mLeft.part.pPart = 		pid->mLeft.gain.Kp*pid->mLeft.part.err;	// = Kp*E
	pid->mLeft.part.iPart += 		0.5*Ts*pid->mLeft.gain.Ki*pid->mLeft.part.err; //+= 0.5*Ts*Ki*E
	double temp_dPart1 = 				pid->mLeft.gain.Kd/Ts*pid->mLeft.part.err;
	pid->mLeft.part.dPart = 		temp_dPart1 - pid->mLeft.part.preDpart;
	pid->mLeft.part.preDpart = 	temp_dPart1; //pid->mLeft.gain.Kd/pid->mLeft.Ts*err1;
	pid->mLeft.para.pwm = pid->mLeft.part.pPart + pid->mLeft.part.iPart + pid->mLeft.part.dPart;

	if (pid->mLeft.para.pwm > MAX_PWM) 	pid->mLeft.para.pwm = MAX_PWM;
	else
		if (pid->mLeft.para.pwm < -MAX_PWM) pid->mLeft.para.pwm = -MAX_PWM;

	//Motor RIGHT
	pid->mRight.part.err = (pid->mRight.para.setpoint - pid->mRight.para.getspeed);

	pid->mRight.part.pPart = 		pid->mRight.gain.Kp*pid->mRight.part.err;	// = Kp*E
	pid->mRight.part.iPart += 	0.5*Ts*pid->mRight.gain.Ki*pid->mRight.part.err; //+= 0.5*Ts*Ki*E
	double temp_dPart2 = 				pid->mRight.gain.Kd/Ts*pid->mRight.part.err;
	pid->mRight.part.dPart = 		temp_dPart2 - pid->mRight.part.preDpart;
	pid->mRight.part.preDpart = temp_dPart2; //pid->mRight.gain.Kd/pid->mRight.Ts*err1;
	pid->mRight.para.pwm = pid->mRight.part.pPart + pid->mRight.part.iPart + pid->mRight.part.dPart;

	if (pid->mRight.para.pwm > MAX_PWM) 	pid->mRight.para.pwm = MAX_PWM;
	else
		if (pid->mRight.para.pwm < -MAX_PWM) pid->mRight.para.pwm = -MAX_PWM;
}
/******************* (C) COPYRIGHT 2020 hiennd *****END OF FILE****/


