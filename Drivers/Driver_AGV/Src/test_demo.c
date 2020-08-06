/**
  ******************************************************************************
  * @file    :test_demo.c
  * @author  :HienND
  * @version :v1_00
  * @date    :Aug 3, 2020
  * @brief   :
  *
  ******************************************************************************
**/
/* Includes ------------------------------------------------------------------*/
#include "test_demo.h"
#include "sys.h"
#include "delay.h"
#include "uart.h"
#include "robot.h"
#include "estimation.h"
/* Private variables ---------------------------------------------------------*/
static float parameter[10];
static float v_target_robot = 0, w_target_robot = 0;
// Note: Biến static vẫn debug bình thường như biến global.
static float target_w_wheel;
static int time_period;
static int16_t set_pwm_duty = 250; //(-999 to +999)
/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim7;
extern float yaw_imu;
extern float ax, ay, az, gx, gy, gz, pitch, yaw, roll;
extern volatile float q0, q1, q2, q3;
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/
void Test_Odom_IMU(ROBOT_HandleTypeDef *hRobot)
{
	parameter[0] = hRobot->Value.cur_robot_velocity;	//v = m/s
	parameter[1] = hRobot->Value.cur_robot_angular; 	//w = rad/s
	//
	parameter[2] = hRobot->Odom.odom_pose[0];	//x
	parameter[3] = hRobot->Odom.odom_pose[1];	//y
	parameter[4] = hRobot->Odom.odom_pose[2];	//yaw_mcu = rad
	parameter[5] = yaw_imu*(PI/180);	//convert to rad

	parameter[6] = hRobot->Value.wheelLeft.cur_wheel_angular;	//wL = rad/s
	parameter[7] = hRobot->Value.wheelRight.cur_wheel_angular; 	//wR = rad/s

	//UartTX_Float(parameter,8);
	//UartTX_Float_aHeader(parameter,8);
	//Thời gian đẩy data lên đang quá nhiều. giảm xuống
	static int timeSampleTs = 0;
	if (++timeSampleTs  >= 4) // 20ms = 50Hz = 200/4
	{
		timeSampleTs = 0;
		//UartTX_Float_aHeader(parameter,8);
		UartTX_Float(parameter,8);
	}
}
//add:
extern float dataFloat[3];	//v_Robot, w_Robot, du tru
void Test_Run_Robot(ROBOT_HandleTypeDef *hRobot)
{
	//add
	v_target_robot = dataFloat[0];
	w_target_robot = dataFloat[1];
	//
	ComputeAngularVelocity(hRobot,v_target_robot,w_target_robot); //v=0.(m/s) | w=0.(rad/s)
}
extern volatile int16_t nCountTick1ms;
volatile uint32_t time_pre = 0;
volatile uint32_t time_delta = 0;
volatile uint32_t nCountTx_Run_Test = 0;
//OK == PASSED. 05/08/2020
void Test_Robot_Run_OneCircle(ROBOT_HandleTypeDef *hRobot)
{
	//1. Reset
	ComputeAngularVelocity(hRobot,0,0);
	hRobot->Odom.odom_pose[0] = 0.0;	//x
	hRobot->Odom.odom_pose[1] = 0.0;	//y
	hRobot->Odom.odom_pose[2] = 0.0;	//yaw_mcu = rad
	//2. quay đủ 1 vòng
	float v = 0.5, w = 0.5; //v=0.5|w=0.5 -> R = ~~2m
	float timeOneCircle = (2*PI/w)*1000; //ms = 12,566.37

	//timeOneCircle = 25*1000; //cho quay hơn 2 vòng chút.

	time_pre = HAL_GetTick();
	nCountTick1ms = 0;
	ComputeAngularVelocity(hRobot,v,w);
	nCountTx_Run_Test = 0;
	while(HAL_GetTick() - time_pre <= timeOneCircle)
	{
		//wait();
		if (nCountTick1ms >= 5) {	// 5ms == 200 Hz
			nCountTick1ms = 0; //reset
			Test_Odom_IMU(hRobot);
			nCountTx_Run_Test++;
		}
	}
	time_delta = HAL_GetTick() - time_pre;
	ComputeAngularVelocity(hRobot,0,0);
	Test_Odom_IMU(hRobot);
	Error_Handler();
}
//OK == PASSED. 05/08/2020
void Test_Robot_Run_Square(ROBOT_HandleTypeDef *hRobot)
{
	//1. Reset
	ComputeAngularVelocity(hRobot,0,0);
	hRobot->Odom.odom_pose[0] = 0.0;	//x
	hRobot->Odom.odom_pose[1] = 0.0;	//y
	hRobot->Odom.odom_pose[2] = 0.0;	//yaw_mcu = rad

	int timeSendTx = 0;
	float v = 0.5, w = 0.785398163397448;
	static int32_t nCount = 0;
	float timeSquare = 24*1000; //24s

	nCountTx_Run_Test = 0;
	time_pre = HAL_GetTick();
	nCountTick1ms = 0;
	nCount = 0;

	//
	while(HAL_GetTick() - time_pre <= timeSquare)
	{
		if (nCountTick1ms >= 1) { nCountTick1ms = 0; //reset
			nCount++;
			if (++timeSendTx >= 5) { timeSendTx = 0;
				Test_Odom_IMU(hRobot);
				nCountTx_Run_Test++;
			}
		}
		//
		if (nCount < 4*1000)	//4s
		{
			ComputeAngularVelocity(hRobot,v,0);
		}
		//
		if ((nCount >= 4*1000) && (nCount < 6*1000))	//4s-6s
		{
			ComputeAngularVelocity(hRobot,0,w);
		}
		if ((nCount >= 6*1000) && (nCount < 10*1000))	//4s-6s
		{
			ComputeAngularVelocity(hRobot,v,0);
		}
		if ((nCount >= 10*1000) && (nCount < 12*1000))	//4s-6s
		{
			ComputeAngularVelocity(hRobot,0,w);
		}
		if ((nCount >= 12*1000) && (nCount < 16*1000))	//4s-6s
		{
			ComputeAngularVelocity(hRobot,v,0);
		}
		if ((nCount >= 16*1000) && (nCount < 18*1000))	//4s-6s
		{
			ComputeAngularVelocity(hRobot,0,w);
		}
		if ((nCount >= 18*1000) && (nCount < 22*1000))	//4s-6s
		{
			ComputeAngularVelocity(hRobot,v,0);
		}
		//
		if (nCount >= 22*1000)
			ComputeAngularVelocity(hRobot,0,0);
	}
	time_delta = HAL_GetTick()  - time_pre;
	ComputeAngularVelocity(hRobot,0,0);
	Test_Odom_IMU(hRobot);
	Error_Handler();
}
//?
void Test_Robot_Run_Num8(ROBOT_HandleTypeDef *hRobot)
{
	//1. Reset
	ComputeAngularVelocity(hRobot,0,0);
	hRobot->Odom.odom_pose[0] = 0.0;	//x
	hRobot->Odom.odom_pose[1] = 0.0;	//y
	hRobot->Odom.odom_pose[2] = 0.0;	//yaw_mcu = rad

	int timeSendTx = 0;
	float v = 0.5, w = 0.5;
	static int32_t nCount = 0;
	float timeNum8 = 28*1000; //28s

	nCountTx_Run_Test = 0;
	time_pre = HAL_GetTick();
	nCountTick1ms = 0;
	nCount = 0;

	while(HAL_GetTick() - time_pre <= timeNum8)
	{
		if (nCountTick1ms >= 1) { nCountTick1ms = 0; //reset
			nCount++;
			if (++timeSendTx >= 5) { timeSendTx = 0;
				Test_Odom_IMU(hRobot);
				nCountTx_Run_Test++;
			}
		}
		//1
		if (nCount < 6.283*1000)	//
		{
			ComputeAngularVelocity(hRobot,v,w);
		}
		//2
		if ((nCount >= 6.283*1000) && (nCount < 18.85*1000))
		{
			ComputeAngularVelocity(hRobot,v,-w);
		}
		//3
		if ((nCount >= 18.85*1000) && (nCount < 25*1000))
		{
			ComputeAngularVelocity(hRobot,v,w);
		}
		//4
		if (nCount >= 25*1000)
			ComputeAngularVelocity(hRobot,0,0);
	}
	time_delta = HAL_GetTick()  - time_pre;
	ComputeAngularVelocity(hRobot,0,0);
	Test_Odom_IMU(hRobot);
	Error_Handler();
}
//Kiểm tra đáp ứng tốc độ góc (rad/s) của 2 bánh xe.
void Test_Response_Angular_Velocity_Wheel(ROBOT_HandleTypeDef *hRobot)
{
	float wLR = PulseGeneratorSignalPWM(target_w_wheel, time_period);	//ex: || 10.0 (rad/s) || 10s ||
	//Đặt 02 giá trị này, thì bộ PID đã tự động nhận và tính toán điều khiển.
	hRobot->Value.wheelLeft.set_wheel_angular = wLR;	//rad/s
	hRobot->Value.wheelRight.set_wheel_angular = wLR;
	//
	parameter[0] = wLR;
	parameter[1] = hRobot->Value.wheelLeft.cur_wheel_angular;
	parameter[2] = hRobot->Value.wheelRight.cur_wheel_angular;
	//Gửi về Matlab
	UartTX_Float(parameter,3);
}
/*
 * Dùng cho việc gửi: pwm, nhận về tốc độ 2 2 bánh xe, để ước lượng hàm truyền rời rạc Gz trên Matlab
 */
void Test_Set_PWM_For_Estimation(ROBOT_HandleTypeDef *hRobot)
{
	//ROBOT_GetSpeed(hRobot);	//TIM7 lam viec nay roi !
	ROBOT_SetPWMtoMotor(hRobot, set_pwm_duty, set_pwm_duty); //
	//
	parameter[0] = set_pwm_duty;
	parameter[1] = hRobot->Value.wheelLeft.cur_wheel_angular;
	parameter[2] = hRobot->Value.wheelRight.cur_wheel_angular;
	//
	UartTX_Float(parameter,3);
}
void Test_Get_IMU_Raw_RPY()
{
	//Send Data to Matlab
	parameter[0] = ax;
	parameter[1] = ay;
	parameter[2] = az;
	parameter[3] = gx;
	parameter[4] = gy;
	parameter[5] = gz;
	parameter[6] = roll;
	parameter[7] = pitch;
	parameter[8] = yaw;	// độ - degree - 0-360 độ
	//
	UartTX_Float(parameter, 9);//Maltab
}
