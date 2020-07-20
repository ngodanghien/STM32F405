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
/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <rosapp.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h> //demo
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

//#include <geometry_msgs/Quaternion.h>
//#include "geometry_msgs/PointStamped.h"
//#include "geometry_msgs/Vector3Stamped.h"
//#include "geometry_msgs/QuaternionStamped.h"
//#include "geometry_msgs/TransformStamped.h"
//#include "geometry_msgs/PoseStamped.h"

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>

#include <sensor_msgs/Imu.h>



#include "motor.h"
#include "robot.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint16_t nCountTickROS=0;
/* External variables --------------------------------------------------------*/
extern ROBOT_HandleTypeDef robotAGV;
/* ROS variables -------------------------------------------------------------*/
ros::NodeHandle nh;			// ROS nodehandle

//std_msgs::Float32 lwheel_angular_vel;
//std_msgs::Float32 rwheel_angular_vel;
//
//ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

sensor_msgs::Imu imu;
nav_msgs::Odometry odom_pub;


tf::TransformBroadcaster odom_broadcaster;

/* ROS function prototypes Callback -------------------------------------------*/
void sub__cmd_vel__callback(const geometry_msgs::Twist &msg);
geometry_msgs::Quaternion YawToQuaternion(double yaw);
void OdomPublisher();
//void odometry_pub();
/* ROS Publisher variables ---------------------------------------------------*/
//ros::Publisher lpub_wheel_angular_vel_enc("lwheel_angular_vel_enc", &lwheel_angular_vel);
//ros::Publisher rpub_wheel_angular_vel_enc("rwheel_angular_vel_enc", &rwheel_angular_vel);

ros::Publisher pub_imu("imu", &imu);
ros::Publisher pub_odom("odom",&odom_pub); //topic_name + Messenger

std_msgs::Float32MultiArray	para; //demo
float viewROS[6] = {0};	//demo
//
ros::Publisher pubRobotPara("robotAGVpara",&para);

/* ROS Subscriber variables ---------------------------------------------------*/
ros::Subscriber<geometry_msgs::Twist> sub__cmd_vel("cmd_vel", &sub__cmd_vel__callback);

/* Private function prototypes -----------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* HAL_UART functions --------------------------------------------------------*/
//[PASSED] : Đã Test với Raspi3B+ ở các tốc độ: 115200; 256000;	OK
// 512000bps: PC làm việc tốt, Raspi3B+ xử lý ko kịp.
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	nh.getHardware()->reset_rbuf();
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
	nh.initNode();

	/* Register a new publisher */
	//	nh.advertise(lpub_wheel_angular_vel_enc);
	//	nh.advertise(rpub_wheel_angular_vel_enc);
	nh.advertise(pub_odom);
	//nh.loginfo("[MCU] Advertise OK-----------");

	/* Register a new subscriber */
	// Đối với Subscribe chỉ việc đăng ký callbacks là đủ,
	// Tốc độ truyền phụ thuộc vào PC gửi xuống (đơn vị Hz)
	nh.subscribe(sub__cmd_vel);

	/************  broadcast tf  *************/
	odom_broadcaster.init(nh);
	//Để send Tf đến ROS
	//odom_broadcaster.sendTransform(tfStamp);



}
/**
 * @brief  loop
 * @param  void
 * @retval void
 */
void ROS_Loop(void)
{
	static int nNumCountPubs = 0;
	// Handle all communications and callbacks.
	nh.spinOnce();	//Luôn được gọi liên tục để phục vụ ROSserial (bao gồm nhận Subcrible)
	// Publisher Only : Chỉ có truyền lên PC (Publisher)
	if (nCountTickROS >= 50) { nCountTickROS = 0; //reset // 20 Hz = 1/20 = 50ms
	//Code Here !

	// Publish message to be transmitted.
	//	if (nNumCountPubs++ == 0)
	//	{
	//		lwheel_angular_vel.data = (float)robotAGV.Value.wheelLeft.cur_wheel_velocity;
	//		lpub_wheel_angular_vel_enc.publish(&lwheel_angular_vel);
	//	}
	//	else
	//	{
	//		nNumCountPubs = 0; //reset
	//		rwheel_angular_vel.data = (float)robotAGV.Value.wheelRight.cur_wheel_velocity;
	//		rpub_wheel_angular_vel_enc.publish(&rwheel_angular_vel);
	//	}
	//  	  para.layout.data_offset = 0;
	//  	  para.layout.dim->stride = 123;
	//Update thông tin lên cho ROS xem (xem sự thay đổi khi thay đổi tốc độ)
	//PubDemoView();
	//
	//pubRobotPara.publish(&para);
	//
	//GPIOB->ODR ^= GPIO_PIN_13; //Toggle LED ROS

	//New code !! Chia thành các lần truyền ...
	switch(nNumCountPubs)
	{
	case 0:
		//broadcaster.sendTransform(tfStamp);
		//odometry_pub();
		OdomPublisher();
		nNumCountPubs++;
		break;
	case 1:
		//anything - Final
		nNumCountPubs = 0; //reset
		break;
	}
	}
}
void pub_odometry()
{
	//ref: D:\STUDY\DeCuong-LVThS-NEW\ROS Pi3B\Home\src\slam_robot\src\my_slam_robot_diff_odom\diffdrive_odom.py
	//	 odom_msg = Odometry()
	//	    odom_msg.header.stamp = self.time_prev_update
	//	    odom_msg.header.frame_id = self.frame_id
	//	    odom_msg.child_frame_id = self.child_frame_id
	//	    odom_msg.pose.pose.position = Point(pose['x'], pose['y'], 0)
	//	    odom_msg.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,pose['th']))
	//	#    P = numpy.mat(numpy.diag([0.0]*3)) # Dummy covariance
	//	#    odom_msg.pose.covariance = tuple(P.ravel().tolist())
	//	    # odom.pose.covariance[0]  = 0.01;
	//	    # odom.pose.covariance[7]  = 0.01;
	//	    # odom.pose.covariance[14] = 99999;
	//	    # odom.pose.covariance[21] = 99999;
	//	    # odom.pose.covariance[28] = 99999;
	//	    # odom.pose.covariance[35] = 0.01;
	//
	//	    # odom.child_frame_id = "base_footprint";
	//	    # odom.twist.twist.linear.x = vx;
	//	    # odom.twist.twist.linear.y = 0;
	//	    # odom.twist.twist.angular.z = w;
	//	    # odom.twist.covariance = odom.pose.covariance;
	//	    self.odom_pub.publish(odom_msg)

	//compute odometry in a typical way given the velocities of the robot
	//since all odometry is 6DOF we'll need a quaternion created from yaw
	//	double x,y,theta;
	//	x = 0; y =0;; theta =0;
	//geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

	//ref: http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom

	geometry_msgs::Point pose;
	pose.x = 2.0;
	pose.y = 3.0;
	pose.z = 1;
	//next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;

	odom.header.stamp = nh.now();
	odom.header.frame_id = "odom";
	//set the position
	odom.pose.pose.position.x = 0;//x;
	odom.pose.pose.position.y = 0;//y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = YawToQuaternion(1.0);
	//set the velocity
	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x = 0;//vx;
	odom.twist.twist.linear.y = 0;//vy;
	odom.twist.twist.angular.z = 0;//vth;
	//publish the message
	//odom_pub.publish(&odom);

}
void pub_tf()
{
	/*	self.tf_broadcaster.sendTransform( \
				(pose['x'], pose['y'], 0), \
				tf.transformations.quaternion_from_euler(0,0,pose['th']), \
				self.time_prev_update, \
				self.child_frame_id, \
				self.frame_id \
	) */
	//since all odometry is 6DOF we'll need a quaternion created from yaw
	double x = 2.0, y = 3.0, theta = 0.1;
	geometry_msgs::Quaternion odom_quat = YawToQuaternion(theta);
	//first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = nh.now();
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;	//
	//send the transform
	odom_broadcaster.sendTransform(odom_trans);
}
/* Odometry Functions ----------------------------------------------*/
void OdomPublisher()
{
	//ref: http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
	//ref: ref http://moorerobots.com/blog/post/5?fbclid=IwAR1qnJ5xJERBM6K_v51F8yDjFIzCXEAbo71GJ6sNwJ-OquP3gmXfPHQD8L8
	//* self.update() : Bao gồm 02 thành phần, Odometry và TF
	// 1. : self.pub_odometry(self.pose)
	// 2. : self.pub_tf(self.pose)
	pub_odometry();
	pub_tf();
}
/* CallBack Functions ----------------------------------------------*/
/**
 * @brief  sub__cmd_vel__callback
 * 	Nhận tốc độ robot từ topic: cmd_vel, chuyển đổi sang tốc độ góc (rad/s) cho từng bánh xe
 * @param  geometry_msgs::Twist
 * @retval void
 */
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

//void Note()
//{
//	nh.logdebug("Debug Statement");
//	nh.loginfo("Program info");
//	nh.logwarn("Warnings.");
//	nh.logerror("Errors..");
//	nh.logfatal("Fatalities!");
//}

//geometry_msgs::Quaternion ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
geometry_msgs::Quaternion YawToQuaternion(double yaw) // yaw (Z)
{
	//ref: //https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	// Abbreviations for the various angular functions
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
//https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
//geometry_msgs::Transform::
//EulerAngles ToEulerAngles(Quaternion q) {
//    EulerAngles angles;
//
//    // roll (x-axis rotation)
//    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
//    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
//    angles.roll = std::atan2(sinr_cosp, cosr_cosp);
//
//    // pitch (y-axis rotation)
//    double sinp = 2 * (q.w * q.y - q.z * q.x);
//    if (std::abs(sinp) >= 1)
//        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
//    else
//        angles.pitch = std::asin(sinp);
//
//    // yaw (z-axis rotation)
//    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
//    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
//    angles.yaw = std::atan2(siny_cosp, cosy_cosp);
//
//    return angles;
//}

