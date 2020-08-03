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
extern float yaw_imu;
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/
void Test_Odom_IMU(ROBOT_HandleTypeDef *hRobot)
{
	parameter[0] = hRobot->Value.cur_robot_velocity;	//v = m/s
	parameter[1] = hRobot->Value.cur_robot_angular; 	//w = rad/s
	//
	parameter[2] = hRobot->Odom.odom_pose[0];	//x
	parameter[3] = hRobot->Odom.odom_pose[1];	//y
	parameter[4] = hRobot->Odom.odom_pose[2];	//yaw_mcu
	parameter[5] = yaw_imu;	//yaw_imu
	UartTX_Float(parameter,6);
}
void Test_Run_Robot(ROBOT_HandleTypeDef *hRobot)
{
	ComputeAngularVelocity(hRobot,v_target_robot,w_target_robot); //v=0.(m/s) | w=0.(rad/s)
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

