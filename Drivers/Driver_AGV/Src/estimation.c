/**
  ******************************************************************************
  * @file    :pwm.c
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
#include "estimation.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int16_t PulseGeneratorSignalPWM(int16_t maxPulse, uint16_t periodTimeSecond);
int16_t SinGeneratorSignalPWM(uint16_t stepTimeS, uint8_t inverse);
/* Private function prototypes -----------------------------------------------*/
/* External variables --------------------------------------------------------*/
extern uint16_t nCountTick1ms;
/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Main Estimation
  * @param  uint16_t stepTimeS, uint8_t inverse
  * @retval void
  */
int max_pwm_duty 	= 250; 	// -999 to +999
int time_duty	= 2; 	// seconds
//Estimation(&robotAGV); -> (hàm thay thế):  Test_Set_PWM_For_Estimation(&robotAGV);
void Estimation(ROBOT_HandleTypeDef *hRobot)
{
	//0. Tính toán PWM dựa vào: PulseGeneratorEstimation
	//1. Set pwm đã tính toán đc ở trên, và tính Speed Encoder
	//2. Gửi toàn bộ data lên PC theo stt (float32) = pwm,speed,speed
	static float parameter[3] = {0};	//pwm,speed,speed
	static int16_t pwm = 0;
	//static int timeSendWiFi = 0;
	//
	if (nCountTick1ms >= 5)
	{
		nCountTick1ms = 0; //reset

		// ----Code here (5ms)
		//pwm = SinGeneratorSignalPWM(1,1);
		//pwm = PulseGeneratorSignalPWM(max_pwm_duty,time_duty);	//900|2
		// Nếu chỉ sử dụng mỗi Estimation() thì thời gian trong này chính xác: 5ms
		//GPIOB->ODR ^= USER_LED_Pin; //LED = PB13 (Toggle)
		pwm = max_pwm_duty;
		//ROBOT_GetSpeed(hRobot);	//TIM7 lam viec nay roi !
		ROBOT_SetPWMtoMotor(hRobot, pwm, pwm); //
		//
		parameter[0] = pwm;
		parameter[1] = hRobot->Value.wheelLeft.cur_wheel_angular;
		parameter[2] = hRobot->Value.wheelRight.cur_wheel_angular;
		//
		UartTX_Float(parameter,3);
		//
		//using WiFi ..... !!!
		/*
		if (++timeSendWiFi >= 10)	//20*5 ==== 100ms = OK [PASSED]
		{
			timeSendWiFi = 0;
			UartTX_Float(parameter,3);
			GPIOB->ODR ^= USER_LED_Pin; //LED = PB13 (Toggle)
		} */
	}
}
/**
  * @brief  DiscretePulseGenerator function FOR Estimation
  * @param
  * 	uint16_t stepTimeS	: (Second) bước nhảy 1s 1 lần/ mỗi lần 200 pwm
  * 	uint8_t inverse		: (inverse = 1 = Đảo chiều động cơ), tín hiệu hình SINx
  * @retval int16_t (pwm)
  */
int16_t SinGeneratorSignalPWM(uint16_t stepTimeS, uint8_t inverse)
{
  /* DiscretePulseGenerator: '<Root>/Pulse Generator' */
  static uint16_t clockTickCounter = 0;
  static uint16_t stepPwm = 200;	//step
  static uint16_t index = 0;
  static uint8_t bInverse = 0;
  static int16_t setpoint = 0;

  // step = 5ms -> 200 lần = 200*5 = 1000ms = 1s ==> Gọi đến 200 lần thì index mới tăng lên 1.
  if (++clockTickCounter >= stepTimeS*200) {	//200 //Ở đây đang tính cho 5m/step
	  clockTickCounter = 0; //reset
      //Update
	  //  0   1   2   3   4   5    6    7    8    9   10
	  // 200 400 600 800 1000 800  600  400  200  00 ---
      if (index < 5) {// 0->4
    	  setpoint += stepPwm ; //inc +200
      }
      else	{ //5->9
    	  setpoint -= stepPwm ; //dec -200
      }
      if (setpoint>MAX_DUTY_PWM_EST) setpoint = MAX_DUTY_PWM_EST;

      if (++index >= 10) {
    	index = 0; //reset
    	bInverse = !bInverse; //toggle
      }
  }	//end-IF
  //
  // inverse = 1 => Tín hiệu hình SIN, else hình Tam giác.
  if (inverse) {
	  if (bInverse) {
		  return (-setpoint);
	  }
  }
  return setpoint;
}
/**
  * @brief  PulseGenerator: Tạo xung vuông
  * @param
  * 	int16_t maxPulse: 			Giá trị PWM cao nhất mong muốn
  * 	uint16_t periodTimeSecond: 	Thời gian (giây) đảo xung
  * @retval int16_t (pwm)
  */
int16_t PulseGeneratorSignalPWM(int16_t maxPulse, uint16_t periodTimeSecond)
{
	/* DiscretePulseGenerator: '<Root>/Pulse Generator' */
	static int16_t clockTickCounter = 0;
	int16_t pulse = (clockTickCounter < periodTimeSecond*100) && (clockTickCounter >=0) ? maxPulse : 0;
	if (clockTickCounter >= periodTimeSecond*200-1) {
    clockTickCounter = 0;
	} else {
		clockTickCounter++;
	}
	return pulse;
}

/******************* (C) COPYRIGHT 2020 hiennd *****END OF FILE****/

