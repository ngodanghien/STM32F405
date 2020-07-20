/**
  ******************************************************************************
  * @file   :	motor.h
  * @author :	HienND
  * @version:	v1_00 
  * @date   : 27/05/2020 
  * @brief  :	Header file for motor.c module.
  ******************************************************************************
  ******************************************************************************  
  */
  /*
  History
  ------------------------------------------------------------------------------
  version  		author			date		    description
  ------------------------------------------------------------------------------
  v1.00			  HienND			27/05/2020
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PSTWO_H
#define __PSTWO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
#include "gpio.h"
#include "delay.h"
//#include "sys.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define DI   PCin(2)           //PC2  Input

#define DO_H PCout(1)=1        //Command high
#define DO_L PCout(1)=0        //Command bit low

#define CS_H PCout(3)=1       //CS high
#define CS_L PCout(3)=0       //CS low

#define CLK_H PAout(4)=1      //Clock up
#define CLK_L PAout(4)=0      //Clock down

//These are our button constants
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2          9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_GREEN       13
#define PSB_RED         14
#define PSB_BLUE        15
#define PSB_PINK        16

#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15
#define PSB_SQUARE      16

//#define WHAMMY_BAR		8

//These are stick values
#define PSS_RX 5                //Right joystick X axis data
#define PSS_RY 6
#define PSS_LX 7
#define PSS_LY 8

/* Exported variables ------------------------------------------------------------*/
extern u8 Data[9];
extern u16 MASK[16];
extern u16 Handkey;

/* Initialization and de-initialization functions ----------------------------*/
/* IO operation functions ----------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void PS2_Init(void);
u8 PS2_RedLight(void);   //Determine whether it is red light mode
void PS2_ReadData(void); //Read handle data
void PS2_Cmd(u8 CMD);		  //Send commands to the handle
u8 PS2_DataKey(void);		  //Key value reading
u8 PS2_AnologData(u8 button); //Get an analog of a rocker
void PS2_ClearData(void);	  //Clear data buffer
void PS2_Vibration(u8 motor1, u8 motor2);//Vibration setting motor1 0xFF on, other off, motor2 0x40~0xFF

void PS2_EnterConfing(void);	 //Enter configuration
void PS2_TurnOnAnalogMode(void); //Send analog
void PS2_VibrationMode(void);    //Vibration settings
void PS2_ExitConfing(void);	     //Complete configuration
void PS2_SetInit(void);		     //Configuration initialization
void PS2_Receive (void);

#ifdef __cplusplus
}
#endif

#endif /* __H */

/******************* (C) COPYRIGHT 2020 hiennd *****END OF FILE****/






