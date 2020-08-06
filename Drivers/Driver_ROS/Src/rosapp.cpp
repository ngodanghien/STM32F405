/**
 ******************************************************************************
 * @file    :mainpp.c
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
  v1.0			HienND			27/05/2020
  v1.1			HienND			25/06/2020
  v1.2			HienND			14/07/2020
  	  	  1. Để đơn giản cho trên PC (Linux) thì bên dưới MCU sẽ tính toán luôn TF gửi lên ROS
  	  	  2. Nhận (SUB) mỗi bản tin về tốc độ: /cmd_vel
  	  	  3. Truyền (PUB) các bản tin về: /odom ; /imu (rpy và imu_raw)
  	  	  4. * Note: Hết sức chú ý đến thời gian truyền lên PC, gói tin cho dữ liệu được truyền theo Format của ROS
  	  	  	  - Nếu data dài thì thời gian truyền sẽ quá thời gian lấy mẫu ...Vì vậy, đề 1 vòng For cho truyền lần lượt với Hz cố định (đảm bảo đã gửi xong)
  	  	  	  - Thời gian này tầm 100ms là được (khác với step của hệ thống)./
	------------------------------------------------------------------------------
 **/
/* Includes SYS------------------------------------------------------------------*/
#include <math.h>
#include <ros.h>
#include <rosapp.h>

/* Includes ROS------------------------------------------------------------------*/
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include <geometry_msgs/Twist.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>

#include <sensor_msgs/Imu.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint16_t nCountTickROS=0;
/* External variables --------------------------------------------------------*/
extern ROBOT_HandleTypeDef robotAGV;
/* ROS variables -------------------------------------------------------------*/
ros::NodeHandle nh;			// ROS nodehandle

/* ROS function prototypes Callback -------------------------------------------*/
void sub__cmd_vel__callback(const geometry_msgs::Twist &msg);

/* ROS Publisher variables ---------------------------------------------------*/
std_msgs::Float32 rwheel_angular_vel;
ros::Publisher rpub_wheel_angular_vel_enc("rwheel_angular_vel_enc", &rwheel_angular_vel);

nav_msgs::Odometry odom;
ros::Publisher pub_odom("odom",&odom); //topic_name + Messenger

geometry_msgs::TransformStamped t;

tf::TransformBroadcaster odom_broadcaster;

sensor_msgs::Imu imu;
ros::Publisher pub_imu("imu", &imu);

geometry_msgs::Vector3 imu_rpy;	//store roll|pitch|yaw
ros::Publisher pub_imu_rpy("imu_rpy", &imu_rpy);

/* ROS Subscriber variables ---------------------------------------------------*/
ros::Subscriber<geometry_msgs::Twist> sub__cmd_vel("cmd_vel", &sub__cmd_vel__callback);

/* Private function prototypes -----------------------------------------------*/
geometry_msgs::Quaternion YawToQuaternion(double yaw);
void OdomPublisher();
void pub_tf();
void pub_odometry();
void pub_IMU();
void pub_IMU_rpy();

/* External variables --------------------------------------------------------*/
#define Size_pData	13
extern uint8_t pData[10];
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

float dataFloat[3] = {0,0,0};	//v_Robot, w_Robot, duphong`
/* HAL_UART functions --------------------------------------------------------*/
//[PASSED] : Đã Test với Raspi3B+ ở các tốc độ: 115200; 256000;	OK
// 512000bps: PC làm việc tốt, Raspi3B+ xử lý ko kịp.
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	//nh.getHardware()->flush();
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance==USART1)
	{
		//nh.getHardware()->reset_rbuf();
		//UartRX_Float(float *result, uint8_t *buffRx, int lengthF)
		UartRX_Float(dataFloat, pData, 3); //3 float
		HAL_UART_Receive_DMA(&huart1, (uint8_t *)pData, Size_pData);
		return;
	}
	//
	if(huart->Instance==USART2)
	{
		//UartRX_Float(dataFloat, pData, 2);
		//HAL_UART_Receive_DMA(&huart2, (uint8_t *)pData, 10);
		return;
	}
	if(huart->Instance==USART3)
	{
		//UartRX_Float(dataFloat, pData, 2);
		//HAL_UART_Receive_DMA(&huart3, (uint8_t *)pData, 10);
		return;
	}
}

/* Mains functions ----------------------------------------------------------*/
/**
 * @brief  setup for ROSserial
 * 1. ROS nodehandle Init
 * 2. Publisher (Đăng ký toàn bộ các Publisher trước khi .publish - bắt buộc)
 * 3. Subcrible (Đăng ký toàn bộ các Subcrible với function callback)
 * @param  void
 * @retval void
 */
void ROS_Setup(void)
{
	// ROS nodehandle initialization and topic registration
	/* Start serial, initialize buffers */
	nh.initNode();	//ko có TX lên PC
		nh.loginfo("[MCU] ROS nodehandle initialization and topic registration---------OK");	//~6.4mS
	HAL_Delay(1);

	/* Register a new publisher */
	if (nh.advertise(pub_odom))	//ko có TX lên PC
	{
		nh.loginfo("[MCU] ROS registration topic: pub_odom ----------------------------OK");	//~6.4ms
	}
	HAL_Delay(10);

//	if (nh.advertise(pub_imu))	//ko có TX lên PC
//	{
//		nh.loginfo("[MCU] ROS registration topic: pub_imu -----------------------------OK");	//~6.4ms
//	}
	HAL_Delay(10);

//	if (nh.advertise(pub_imu_rpy))	//ko có TX lên PC
//	{
//		nh.loginfo("[MCU] ROS registration topic: pub_imu_rpy -------------------------OK");	//~6.4ms
//	}
	HAL_Delay(10);

	/* Register a new subscriber */
	// Đối với Subscribe chỉ việc đăng ký callbacks là đủ,
	// Tốc độ truyền phụ thuộc vào PC gửi xuống (đơn vị Hz)
	nh.subscribe(sub__cmd_vel);
	HAL_Delay(100);

	/************  broadcast tf  *************/
//	odom_broadcaster.init(nh);	//bắt buộc phải init trước khi sử dụng
//	HAL_Delay(10);
//		nh.loginfo("[MCU] ROS registration topic: odom_broadcaster(TF)-----------------OK");	//~6.4ms
//	HAL_Delay(10);
}
/**
 * @brief  ROS_Loop : Chỉ dùng để truyền lên PC (Publisher)
 * @param  void
 * @retval void
 * Bắt buộc phải gọi hàm liên tục: nh.spinOnce();
 * Publisher.bublisher("TopicName",&Msg): Để gửi 01 Msg với Topic == TopicName
 * nCountTickROS >= 20 ==> 1/20 = 50Hz; max(nNumCountPubs) = 3 => 50Hz/3 = 16.67 Hz (cho 1 bản tin Pub)
 */
void ROS_Loop(void)
{
	static int nNumCountPubs = 0;
	// Handle all communications and callbacks.
	nh.spinOnce();	//Luôn được gọi liên tục để phục vụ ROSserial (bao gồm nhận Subcrible)
	// Publisher:
	if (nCountTickROS >= 40)
	{
		nCountTickROS = 0; //reset , 1/40/2 = 12.5Hz
		//Code Here !
		//GPIOB->ODR ^= GPIO_PIN_13; //Toggle LED ROS

		switch(nNumCountPubs)
		{
		case 0:
			pub_odometry();	// Hết 28.2mS với 722 bytes ở baudrate = 256000bps.
			//GPIOB->ODR ^= GPIO_PIN_13; //Toggle LED ROS
			nNumCountPubs++;
			//break;
		case 1:
			//anything - Final
			//pub_tf();			// Hết 	3.9mS với 101 bytes ở baudrate = 256000bps.
			//HAL_Delay(1);		// Note: Delay sẽ ko có tác dụng ở đây, vì sử dụng DMA để truyền.
			//pub_IMU();			// Hết 12,5mS với 320 Bytes ở baudrate = 256000bps.
			//pub_IMU_rpy();		// Hết 1.24mS với  32 Bytes ở baudrate = 256000bps.
			nNumCountPubs++;// = 0; 	//reset
			//break;
		case 2:
			//anything - Final
			//pub_tf();			// Hết 	3.9mS với 101 bytes ở baudrate = 256000bps.
			//HAL_Delay(1);		// Note: Delay sẽ ko có tác dụng ở đây, vì sử dụng DMA để truyền.
			//pub_IMU();			// Hết 12,5mS với 320 Bytes ở baudrate = 256000bps.
			//pub_IMU_rpy();		// Hết 1.24mS với  32 Bytes ở baudrate = 256000bps.
			nNumCountPubs = 0; 	//reset
			break;
		}
	}
}
/**	Hết 28.2mS với 722 bytes ở baudrate = 256000bps.
 * @brief  loop
 * @param  void
 * @retval void
 */
void pub_odometry()
{
	odom.header.stamp = nh.now();
	odom.header.frame_id = "odom";	//ko được bỏ dấu /
	//set the position
	odom.pose.pose.position.x = robotAGV.Odom.odom_pose[0];
	odom.pose.pose.position.y = robotAGV.Odom.odom_pose[1];
	odom.pose.pose.position.z = 0.0;
	//odom.pose.pose.orientation = YawToQuaternion(hRobot->Odom.odom_pose[2]);
	odom.pose.pose.orientation = tf::createQuaternionFromYaw(robotAGV.Odom.odom_pose[2]);
	//set the velocity
	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x 	= robotAGV.Odom.odom_vel[0];//vx;
	odom.twist.twist.linear.y 	= 0;//robotAGV.Odom.odom_vel[1];//vy=0.0;
	odom.twist.twist.angular.z 	= robotAGV.Odom.odom_vel[2];//vth;
	//publish the message
	pub_odom.publish(&odom);

}
/* Hết 	3.9mS với 101 bytes ở baudrate = 256000bps.
 * TF Tree xuất hiện: odom -> base_link
 * ref: http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF
 */
void pub_tf()		// = OK
{
	//ref: http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF
	//ref: http://wiki.ros.org/tf/Tutorials
	//since all odometry is 6DOF we'll need a quaternion created from yaw
	double x = 0.0, y = 0.0, theta = 0.0;
	//geometry_msgs::Quaternion odom_quat = YawToQuaternion(theta);
	//first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = nh.now();
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_footprint"; //TODO base_link

	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = tf::createQuaternionFromYaw(theta); //yaw
	//send the transform
	odom_broadcaster.sendTransform(odom_trans);
}
/** Hết 12.5mS với 320 Bytes ở baudrate = 256000bps.
 * @brief  loop
 * @param  void
 * @retval void
 */
void pub_IMU()
{
	double gyro[3] = {1,2,3}, acc[3] = {4,5,6};
	imu.angular_velocity.x = gyro[0];
	imu.angular_velocity.y = gyro[1];
	imu.angular_velocity.z = gyro[2];
	imu.linear_acceleration.x = acc[0];
	imu.linear_acceleration.y = acc[1];
	imu.linear_acceleration.z = acc[2];
	pub_imu.publish(&imu);
}
/** Hết 1.24mS với 32 Bytes ở baudrate = 256000bps.
 * @brief  pub_IMU_rpy
 * @param  void
 * @retval void
 */
void pub_IMU_rpy()
{
	imu_rpy.x = 0.123;
	imu_rpy.y = 0.456;
	imu_rpy.z = 0.789;
	pub_imu_rpy.publish(&imu_rpy);
}
/* CallBack Functions ----------------------------------------------*/
/**
 * @brief  sub__cmd_vel__callback
 * 	Nhận tốc độ robot từ topic: cmd_vel, chuyển đổi sang tốc độ góc (rad/s) cho từng bánh xe
 * @param  geometry_msgs::Twist
 * @retval void
 */
//TODO Check Again wL, wR
void sub__cmd_vel__callback(const geometry_msgs::Twist &msg)
{
	double target_v, target_w;	//vận tốc setpoint của Robot
	double vl, vr, wr, wl; 	//vận tốc tính toán cho từng bánh xe trái(l), phải (r)
	//0. Tốc độ của Robot (tốc độ di chuyển thẳng và quay)
	target_v 	= msg.linear.x;			//	m/s
	target_w 	= msg.angular.z;		//	rad/s
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
	/*  Compute angular velocity target
    	# v = wR
    	# v - tangential velocity (m/s)
    	# w - angular velocity (rad/s)
    	# R - radius of wheel (m)
    	angular_vel = tangent_vel / self.R;
	 */
	wl = vl/RADIUS_WHEEL;		//WHEEL_RADIUS: Bán kính bánh xe (m)
	wr = vr/RADIUS_WHEEL; 		//RW: robot wheel radius

	//3. Giới hạn cơ khí của bánh xe (Tốc độ quay maximum)
	if 		(wl > MAX_RADs_LEFT_UP) 		wl = MAX_RADs_LEFT_UP;
	else if (wl < MAX_RADs_LEFT_DOWN) 		wl = MAX_RADs_LEFT_DOWN;
	//
	if 		(wr > MAX_RADs_RIGHT_UP) 		wr = MAX_RADs_RIGHT_UP;
	else if (wr < MAX_RADs_RIGHT_DOWN) 		wr = MAX_RADs_RIGHT_DOWN;
	//4. Cập nhập các giá trị vl,wr đến robot trước khi thực hiện bộ điều khiển PID, STR, Fuzzy ....
	robotAGV.Value.set_robot_velocity 	= target_v;	//view
	robotAGV.Value.set_robot_angular	= target_w;	//view
	robotAGV.Value.wheelLeft.set_wheel_velocity	= vl;
	robotAGV.Value.wheelRight.set_wheel_velocity	= vl;
	robotAGV.Value.wheelLeft.set_wheel_angular 	= wl;	//control wheel (rad/s)
	robotAGV.Value.wheelRight.set_wheel_angular = wr;	//control
	//5. Thực hiện bộ điều khiển !
}

/** Không dùng đến các góc: Pitch, Roll
 * @brief  YawToQuaternion
 * @param  double yaw
 * @retval geometry_msgs::Quaternion
 */
//geometry_msgs::Quaternion ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
geometry_msgs::Quaternion YawToQuaternion(double yaw) // yaw (Z)
{
	//ref: //https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	//Abbreviations for the various angular functions
	double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cp = 1;//cos(pitch * 0.5);
	double sp = 0;//sin(pitch * 0.5);
	double cr = 1;//cos(roll * 0.5);
	double sr = 0;//sin(roll * 0.5);

	geometry_msgs::Quaternion q;
	q.w = cr * cp * cy + sr * sp * sy;
	q.x = sr * cp * cy - cr * sp * sy;
	q.y = cr * sp * cy + sr * cp * sy;
	q.z = cr * cp * sy - sr * sp * cy;

	return q;
}

