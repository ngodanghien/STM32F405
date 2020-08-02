 /**
   ******************************************************************************
   * @file   :	rosapp.h
   * @author :	HienND
   * @version:	v1_00
   * @date   :  Jun 25, 2020
   * @brief  :	Header file for motor.c module.
   ******************************************************************************
   ******************************************************************************
   */
   /*
   History
   ------------------------------------------------------------------------------
   version  		author			date		    description
   ------------------------------------------------------------------------------
   v1.00			HienND			Jun 25, 2020
   */
 /* Define to prevent recursive inclusion -------------------------------------*/
 #ifndef __ROS_APP_H
 #define __ROS_APP_H

 #ifdef __cplusplus
 extern "C" {
 #endif

 /* Private includes ----------------------------------------------------------*/
 //#include "main.h"
#include "motor.h"
#include "robot.h"
 /* Exported types ------------------------------------------------------------*/
 /* Exported constants --------------------------------------------------------*/
 /* Exported macro ------------------------------------------------------------*/
#define WHEEL_RADIUS                    0.033     // meter
#define wheel_seperation_ 				0.160;
#define LEFT                            0
#define RIGHT                           1

#define DEG2RAD(x)                      (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                      (x * 57.2957795131)  // *180/PI


 /* Exported functions prototypes ---------------------------------------------*/
 void ROS_Setup(void);
 void ROS_Loop(void);
 /* Private defines -----------------------------------------------------------*/

 /* Initialization and de-initialization functions ----------------------------*/
 /* IO operation functions ----------------------------------------------------*/

 /* Private types -------------------------------------------------------------*/
 /* Private variables ---------------------------------------------------------*/
 /* Private constants ---------------------------------------------------------*/
 /* Private macros ------------------------------------------------------------*/
 /* Private functions ---------------------------------------------------------*/

 #ifdef __cplusplus
 }
 #endif

 #endif /* __H */

 /******************* (C) COPYRIGHT 2020 hiennd *****END OF FILE****/

