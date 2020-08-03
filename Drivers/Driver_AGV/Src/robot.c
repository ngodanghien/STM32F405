	/**
  ******************************************************************************
  * @file    :robot.c
  * @author  :HienND
  * @version :v1_00
  * @date    :Jun 22, 2020
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
#include "robot.h"
#include <math.h>
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
DRV_StatusTypeDef ROBOT_Init(ROBOT_HandleTypeDef *hRobot);
void ROBOT_InitRegisterCallbacks(ROBOT_HandleTypeDef *hRobot);
void ROBOT_SetPWMtoMotor(ROBOT_HandleTypeDef *hRobot, int16_t pwmLeft, int16_t pwmRight);
void ROBOT_SetSpeed(ROBOT_HandleTypeDef *hRobot);
void ROBOT_GetSpeed(ROBOT_HandleTypeDef *hRobot);
void ROBOT_CONTROL_PID_Init(ROBOT_HandleTypeDef *hRobot);
void ROBOT_CONTROL_PID_Run(ROBOT_HandleTypeDef *hRobot);
void ComputeAngularVelocity(ROBOT_HandleTypeDef *hRobot, double target_v, double target_w);
/* External variables --------------------------------------------------------*/
extern int16_t nCountTick1ms;
extern Motor_HandleTypeDef motorAGV;
extern PID_HandleTypeDef cPID;
/* Exported functions --------------------------------------------------------*/

/**
  * @brief  void
  * @param  void
  * @retval void
  */
DRV_StatusTypeDef ROBOT_Init(ROBOT_HandleTypeDef *hRobot)
{
	/* Check the UART handle allocation */
	if (hRobot == NULL)
	{
		return DRV_ERROR;
	}
	//Phải được gán pointer trước hàm: ROBOT_InitRegisterCallbacks()
	__ROBOT_LINK_HANDLE(hRobot,hMotor,motorAGV);	//hRobot->hMotor = &motorAGV;
	/* Initialize the callbacks to their default values */
	ROBOT_InitRegisterCallbacks(hRobot); //Phải được gọi trước: hRobot->MotorInit(hRobot->hMotor)
	/* Init Motor LEFT + RIGHT */
	Motor_Init(hRobot->hMotor);	//Motor_Init(&motorAGV);
	/* Có thể để Init đồng thời các bộ điều khiển ở đây : PID, STR, Fuzzy ...
	 * Việc sử dụng bộ điều khiển nào do robot.c quyết định.
	 * */
	//PID_Init(hRobot->hPID);
	return DRV_OK;
}
/**	[PASSED]
  * @brief  Initialize the callbacks to their default values.
  * 	Ở các bộ điều khiển (PID,STR...) cần đọc speed trước khi đưa ra pwm
  * 	-> Nên cần phải tách GetSpeed và SetPWM riêng ra.
  * @param  ROBOT handle.
  * @retval none
  */
void ROBOT_InitRegisterCallbacks(ROBOT_HandleTypeDef *hRobot)
{
	/* Init the ROBOT Callback settings */
	hRobot->SetSpeed			= MOTOR_SetPWM;		//[PASSED]
		//ex:hRobot->SetSpeed(hRobot->hMotor);
	hRobot->GetSpeed			= MOTOR_GetSpeed;	//[PASSED]
		//ex:hRobot->GetSpeed(hRobot->hMotor);
}
/**
  * @brief  ROBOT_SetSpeed
  * 	Đặt tốc độ động cơ (tốc độ góc rad/s, tốc độ dài m/s)
  * @param  ROBOT_HandleTypeDef *hRobot
  * @retval void
  */
void ROBOT_SetSpeed(ROBOT_HandleTypeDef *hRobot)
{
	//Tham số đầu vào sẽ là tốc độ dài và tốc độ quay của Robot
	//Tính toán quy đổi sang vr và vl
	//Sau khi đã tính toán vận tốc vr,vl
	ROBOT_SetPWMtoMotor(hRobot, 300, 300); //PASSED
}
void ROBOT_GetOdom(ROBOT_HandleTypeDef *hRobot)
{
	/* Update Odometry...(trước khi tính toán vận tốc mới) */
	double v = hRobot->Value.cur_robot_velocity;	//m/s
	double w = hRobot->Value.cur_robot_angular; 	//rad/s
	double theta = hRobot->Odom.odom_pose[2];
	//
	hRobot->Odom.odom_pose[0] 	+= v*TIME_SAMPE*cos(theta + 0.5*w*TIME_SAMPE);
	hRobot->Odom.odom_pose[1] 	+= v*TIME_SAMPE*sin(theta + 0.5*w*TIME_SAMPE);
	hRobot->Odom.odom_pose[2] 	+= w*TIME_SAMPE;
	//Giới hạn góc quay từ: 0--> 360 độ (quay trái); -0 đến -360 độ (quay phải).
	if (hRobot->Odom.odom_pose[2] >= PPI) 	hRobot->Odom.odom_pose[2] -= PPI; //PPI=2*PI()
	if (hRobot->Odom.odom_pose[2] <= -PPI)	hRobot->Odom.odom_pose[2] += PPI;
	//
	hRobot->Odom.odom_vel[0]	= v;
	hRobot->Odom.odom_vel[1]	= 0.0;
	hRobot->Odom.odom_vel[2]	= w;

}
void ROBOT_GetSpeed(ROBOT_HandleTypeDef *hRobot)
{
	/* Get Speed from ENC Left + Right */
	hRobot->GetSpeed(hRobot->hMotor);
	//Update Odometry
	ROBOT_GetOdom(hRobot);
	/* Update hRobot */
	hRobot->Value.wheelLeft.cur_wheel_angular 	= hRobot->hMotor->mLEFT.para.cur_speed;
	hRobot->Value.wheelRight.cur_wheel_angular 	= hRobot->hMotor->mRIGHT.para.cur_speed;
	hRobot->Value.wheelLeft.cur_wheel_velocity 	= hRobot->Value.wheelLeft.cur_wheel_angular*RADIUS_WHEEL;
	hRobot->Value.wheelRight.cur_wheel_velocity = hRobot->Value.wheelRight.cur_wheel_angular*RADIUS_WHEEL;
}
/**
  * @brief  ROBOT_SetPWMtoMotor		[PASSED]
  * 	Dùng để set giá trị PWM cho 02 động cơ (dùng bất cứ ở đâu)
  * @param  ROBOT_HandleTypeDef *hRobot, int16_t pwmLeft, int16_t pwmRight
  * @retval void
  */
void ROBOT_SetPWMtoMotor(ROBOT_HandleTypeDef *hRobot, int16_t pwmLeft, int16_t pwmRight)
{
	static double vL,vR;
	//check Limit LEFT
	if (pwmLeft > MAX_PWM) 	pwmLeft = MAX_PWM;
	else if (pwmLeft < -MAX_PWM) pwmLeft = -MAX_PWM;
	//check Limit RIGHT
	if (pwmRight > MAX_PWM) 	pwmRight = MAX_PWM;
	else if (pwmRight < -MAX_PWM) pwmRight = -MAX_PWM;
	// Update
	hRobot->Value.wheelLeft.cur_pwm = pwmLeft;
	hRobot->Value.wheelRight.cur_pwm = pwmRight;

	hRobot->hMotor->mLEFT.para.cur_pwm 	= hRobot->Value.wheelLeft.cur_pwm;//cur_pwm_Left;
	hRobot->hMotor->mRIGHT.para.cur_pwm = hRobot->Value.wheelRight.cur_pwm;//cur_pwm_Right;
	/* Set PWM for Motor Left + Right */
	hRobot->SetSpeed(hRobot->hMotor);	//MOTOR_SetPWM(hRobot->hMotor);
	//Chú ý: Phải đặt các thông số pwm cho .para.cur_pwm trước khi chạy: MOTOR_SetPWM()
	//Update tốc độ dài (m/s) và tốc độ góc của robot, http://moorerobots.com/blog/post/4
	// velocity = (vl+vr)/2		(m/s) ; with v = w*RW
	// angular	= (vr-vl)/D2W 	(rad/s)
	//------------------[PASSED] 25/6/2020----------------------
	vL = hRobot->Value.wheelLeft.cur_wheel_angular*RADIUS_WHEEL; //cur_Speed_Left*RADIUS_WHEEL;
	vR = hRobot->Value.wheelRight.cur_wheel_angular*RADIUS_WHEEL; //cur_Speed_Right*RADIUS_WHEEL;
	hRobot->Value.cur_robot_velocity 	= 0.5*( vL + vR);
	hRobot->Value.cur_robot_angular 	= (vR - vL)/D2WHEEL;
	//----------------------------------------------------------
}

//Thêm phần bộ điều khiển vào đây: 24/6/2020
void ROBOT_CONTROL_PID_Init(ROBOT_HandleTypeDef *hRobot)
{
	__ROBOT_LINK_HANDLE(hRobot,hPID,cPID);
	PID_Init(hRobot->hPID);
}

//int16_t PulseGeneratorSignalPWM(int16_t maxPulse, uint16_t periodTimeSecond); //demo
//int16_t SinGeneratorSignalPWM(uint16_t stepTimeS, uint8_t inverse);//
//[TIME-RUN]: 0.5uS
void ROBOT_CONTROL_PID_Run(ROBOT_HandleTypeDef *hRobot)
{
	//static float parameter[4] = {0};	//setpoint1,2,speed1,2

	//1. Lấy tốc độ hiện tại đưa vào bộ điều khiển
	//GPIOB->ODR ^= USER_LED_Pin; //check xem đúng 5ms step ko?
	//Để vào đây thì ko chính xác 5ms cho 1 lần đọc Encoder -> cho time7 gọi hàm này.
	ROBOT_GetSpeed(hRobot);	//Update Odometry !
	hRobot->hPID->mLeft.para.getspeed 	= hRobot->Value.wheelLeft.cur_wheel_angular; //cur_Speed_Left;
	hRobot->hPID->mRight.para.getspeed 	= hRobot->Value.wheelRight.cur_wheel_angular; //cur_Speed_Right;
	hRobot->hPID->mLeft.para.setpoint	= hRobot->Value.wheelLeft.set_wheel_angular;
	hRobot->hPID->mRight.para.setpoint	= hRobot->Value.wheelRight.set_wheel_angular;
	//2. Tính toán giá trị PID
	PID_Run(hRobot->hPID);
	//Set PWM cho động cơ : Các thông số về tốc độ (v,w) đang được update trong này.../
	ROBOT_SetPWMtoMotor(hRobot, hRobot->hPID->mLeft.para.pwm, hRobot->hPID->mRight.para.pwm);

	//Send data to PC
	//		parameter[0] = hRobot->Value.wheelLeft.set_wheel_angular;//setpoint_Left;
	//		parameter[1] = hRobot->Value.wheelLeft.cur_wheel_angular; //cur_Speed_Left;
	//		parameter[2] = hRobot->Value.wheelRight.set_wheel_angular; //setpoint_Right;
	//		parameter[3] = hRobot->Value.wheelRight.cur_wheel_angular;//cur_Speed_Right;

	//UartTX_Float(parameter,4);
	//}
}
/*  Compute angular velocity target
	# v = wR
	# v - tangential velocity (m/s)
	# w - angular velocity (rad/s)
	# R - radius of wheel (m)
	angular_vel = tangent_vel / self.R;
 */
void ComputeAngularVelocity(ROBOT_HandleTypeDef *hRobot, double target_v, double target_w)	//robotAGV || m/s || rad/s
{
	double vl, vr, wr, wl; 	//vận tốc tính toán cho từng bánh xe trái(l), phải (r)
	// Check Limit
	if 		(target_v > MAX_ROBOT_VELOCITY_UP) 		target_v = MAX_ROBOT_VELOCITY_UP;
	else if (target_v < MAX_ROBOT_VELOCITY_DOWN)	target_v = MAX_ROBOT_VELOCITY_DOWN;

	if 		(target_w > MAX_ROBOT_ANGULAR_UP) 		target_w = MAX_ROBOT_ANGULAR_UP;
	else if (target_w < MAX_ROBOT_ANGULAR_DOWN)	target_w = MAX_ROBOT_ANGULAR_DOWN;

	/* Convert (linear & angular) of Robot to 2 wheel (ms/s) */
	//1. Chuyển về tốc độ dài (m/s) của 2 bánh xe trái và phải
	vr = 0.5*(2.0*target_v + target_w*D2WHEEL); //D2W = L = distance 2 wheel
	vl = 0.5*(2.0*target_v - target_w*D2WHEEL); //D2W = Khoảng cách giữa 2 bánh xe (m)
	/* Convert form m/s -> rad/s */
	//2. Chuyển từ tốc độ m/s của bánh xe về tốc độ góc (rad/s)

	wl = vl/RADIUS_WHEEL;		//WHEEL_RADIUS: Bán kính bánh xe (m)
	wr = vr/RADIUS_WHEEL; 		//RW: robot wheel radius

	//3. Giới hạn cơ khí của bánh xe (Tốc độ quay maximum)
	if 		(wl > MAX_RADs_LEFT_UP) 		wl = MAX_RADs_LEFT_UP;
	else if (wl < MAX_RADs_LEFT_DOWN) 		wl = MAX_RADs_LEFT_DOWN;
	//
	if 		(wr > MAX_RADs_RIGHT_UP) 		wr = MAX_RADs_RIGHT_UP;
	else if (wr < MAX_RADs_RIGHT_DOWN) 		wr = MAX_RADs_RIGHT_DOWN;
	//4. Cập nhập các giá trị vl,wr đến robot trước khi thực hiện bộ điều khiển PID, STR, Fuzzy ....
	hRobot->Value.set_robot_velocity 	= target_v;	//view
	hRobot->Value.set_robot_angular		= target_w;	//view
	hRobot->Value.wheelLeft.set_wheel_velocity	= vl;
	hRobot->Value.wheelRight.set_wheel_velocity	= vl;
	hRobot->Value.wheelLeft.set_wheel_angular 	= wl;	//control wheel (rad/s)
	hRobot->Value.wheelRight.set_wheel_angular 	= wr;	//control
	//5. Thực hiện bộ điều khiển !
}
/******************* (C) COPYRIGHT 2020 hiennd *****END OF FILE****/

