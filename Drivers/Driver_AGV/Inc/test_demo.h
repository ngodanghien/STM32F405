/**
  ******************************************************************************
  * @file   :	test_demo.h
  * @author :	HienND
  * @version:	v1_00
  * @date   : 	Aug 3, 2020
  * @brief  :	Header file for test_demo.c module.
  ******************************************************************************/

#ifndef DRIVER_AGV_INC_TEST_DEMO_H_
#define DRIVER_AGV_INC_TEST_DEMO_H_

/* Private includes ----------------------------------------------------------*/
#include "robot.h"
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/
void Test_Odom_IMU(ROBOT_HandleTypeDef *hRobot);
void Test_Run_Robot(ROBOT_HandleTypeDef *hRobot);;
void Test_Response_Angular_Velocity_Wheel(ROBOT_HandleTypeDef *hRobot);
void Test_Set_PWM_For_Estimation(ROBOT_HandleTypeDef *hRobot);
void Test_Get_IMU_Raw_RPY();

/* Private defines -----------------------------------------------------------*/

/* Initialization and de-initialization functions ----------------------------*/
/* IO operation functions ----------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#endif /* DRIVER_AGV_INC_TEST_DEMO_H_ */
