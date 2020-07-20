/**
  ******************************************************************************
  * @file    :pstwo.c
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
#include "pstwo.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DELAY_TIME  delay_us(5) 
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
u16 Handkey;			// The key value is read and stored at zero.
u8 Comd[2]={0x01,0x42};	//Start command. Request data
u8 Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //Data storage array

u16 MASK[]={
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1 ,
    PSB_GREEN,
    PSB_RED,
    PSB_BLUE,
    PSB_PINK
};	//Key value and key description

/* Private function prototypes -----------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
  * @brief  void
  * @param  void
  * @retval void
  */
void PS2_Init(void)
{
//  GPIO_InitTypeDef  GPIO_InitStructure;
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC, ENABLE);//
//  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;// Corresponding IO port
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;/
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);
//	GPIO_SetBits(GPIOA,GPIO_Pin_4);//GPIOA4 is set high, the light is off
//	
//	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1|GPIO_Pin_3;// Corresponding IO port
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_Init(GPIOC, &GPIO_InitStructure);
//	GPIO_SetBits(GPIOC, GPIO_Pin_1|GPIO_Pin_3);	//GPIOC1 C3 is set high, the light is off
//	
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //Corresponding pin
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP
//  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/**
  * @brief  void PS2_Cmd
  * @param  u8 CMD
  * @retval void
  */
void PS2_Cmd(u8 CMD)
{
	volatile u16 ref=0x01;
	Data[1] = 0;
	for(ref=0x01;ref<0x0100;ref<<=1)
	{
		if(ref&CMD)
		{
			DO_H;                   //Output one control bit
		}
		else DO_L;

		CLK_H;                        //Clock up
		DELAY_TIME;
		CLK_L;
		DELAY_TIME;
		CLK_H;
		if(DI)
			Data[1] = ref|Data[1];
	}
	delay_us(16);
}
/**
  * @brief  PS2_RedLight
		- Determine whether it is red light mode, 0x41=simulate green light, 0x73=simulate red light
  * @param  void
  * @retval u8: Return value; 0, red light mode; Other, other modes
  */
u8 PS2_RedLight(void)
{
	CS_L;
	PS2_Cmd(Comd[0]);  //Start command
	PS2_Cmd(Comd[1]);  //Request data
	CS_H;
	if( Data[1] == 0X73)   return 0 ;
	else return 1;

}
/**
  * @brief  Read handle data
  * @param  void
  * @retval void
  */
void PS2_ReadData(void)
{
	volatile u8 byte=0;
	volatile u16 ref=0x01;
	CS_L;
	PS2_Cmd(Comd[0]);  //Start command
	PS2_Cmd(Comd[1]);  //Request data
	for(byte=2;byte<9;byte++)          //Start accepting data
	{
		for(ref=0x01;ref<0x100;ref<<=1)
		{
			CLK_H;
			DELAY_TIME;
			CLK_L;
			DELAY_TIME;
			CLK_H;
		      if(DI)
		      Data[byte] = ref|Data[byte];
		}
        delay_us(16);
	}
	CS_H;
}


/**
  * @brief  PS2_DataKey
	- Process the read data of PS2, only process the key part
	- Only one button is pressed when pressed, 0 is not pressed
  * @param  void
  * @retval void
  */
u8 PS2_DataKey()
{
	u8 index;

	PS2_ClearData();
	PS2_ReadData();

	Handkey=(Data[4]<<8)|Data[3];     //These are the 16 keys. Pressed to 0, not pressed to 1
	for(index=0;index<16;index++)
	{	    
		if((Handkey&(1<<(MASK[index]-1)))==0)
		return index+1;
	}
	return 0;          //No buttons pressed
}


/**
  * @brief  Get the analog value of a 0 ~256
  * @param  u8 button
  * @retval u8
  */
u8 PS2_AnologData(u8 button)
{
	return Data[button];
}

/**
  * @brief  PS2_ClearData
  * @param  void
  * @retval void
  */
void PS2_ClearData()
{
	u8 a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}

/**
  * @brief  Handle vibration function			
  * @param  - motor1: Right small vibration motor 0x00 off, others on
			- motor2: Large vibration motor on the left side 0x40~0xFF Motor on, the greater the value, the greater the vibration
  * @retval void
  */
  
void PS2_Vibration(u8 motor1, u8 motor2)
{
	CS_L;
	delay_us(16);
    PS2_Cmd(0x01);  //Start command
	PS2_Cmd(0x42);  //Request data
	PS2_Cmd(0X00);
	PS2_Cmd(motor1);
	PS2_Cmd(motor2);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);  
}
/**
  * @brief  PS2_ShortPoll
  * @param  void
  * @retval void
  */
void PS2_ShortPoll(void)
{
	CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x42);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x00);
	CS_H;
	delay_us(16);	
}
/**
  * @brief  Enter configuration
  * @param  void
  * @retval void
  */
void PS2_EnterConfing(void)
{
    CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01);
	PS2_Cmd(0x00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);
}
/**
  * @brief  Send mode settings
  * @param  void
  * @retval void
  */
void PS2_TurnOnAnalogMode(void)
{
	CS_L;
	PS2_Cmd(0x01);  
	PS2_Cmd(0x44);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01); //analog=0x01;digital=0x00  Software setting sending mode
	PS2_Cmd(0x03); //Ox03 Latch setting, that is, cannot be pressed ¡°MODE¡± Setting mode
				   //0xEE does not latch the software setting, you can set the mode by pressing the "MODE" button.
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);
}
/**
  * @brief  Vibration settings
  * @param  void
  * @retval void
  */
void PS2_VibrationMode(void)
{
	CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x4D);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0X01);
	CS_H;
	delay_us(16);	
}
/**
  * @brief  Complete and save the configuration
  * @param  void
  * @retval void
  */
void PS2_ExitConfing(void)
{
    CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	CS_H;
	delay_us(16);
}
/**
  * @brief  Controller configuration initialization
  * @param  void
  * @retval void
  */
void PS2_SetInit(void)
{
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_EnterConfing();		//Enter configuration mode
	PS2_TurnOnAnalogMode();	//"Traffic light" configuration mode, and choose whether to save
	//PS2_VibrationMode();	//Turn on vibration mode
	PS2_ExitConfing();		//Complete and save the configuration
}
/**
  * @brief  Read handle information
  * @param  void
  * @retval void
  */
//example demo.
u8 PS2_ON_Flag=0;
int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY; 
void PS2_Receive (void)
{
	if(PS2_ON_Flag)
		{
		PS2_LX=PS2_AnologData(PSS_LX);
		PS2_LY=PS2_AnologData(PSS_LY);
		PS2_RX=PS2_AnologData(PSS_RX);
		PS2_RY=PS2_AnologData(PSS_RY);
		}
		PS2_KEY=PS2_DataKey();
}
/******************* (C) COPYRIGHT 2020 hiennd *****END OF FILE****/
















