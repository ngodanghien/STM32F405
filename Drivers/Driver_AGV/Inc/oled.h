#ifndef __OLED_H
#define __OLED_H			  	 
/**************************************************************************
**************************************************************************/


/* Private includes ----------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

//-----------------OLED�˿ڶ���---------------- 
#define OLED_RST_Clr() PCout(15)=0   //RST = PC15
#define OLED_RST_Set() PCout(15)=1   //RST

#define OLED_RS_Clr() PCout(0)=0    //DC	= PC0
#define OLED_RS_Set() PCout(0)=1    //DC

#define OLED_SCLK_Clr()  PCout(13)=0  //SCL = PC13
#define OLED_SCLK_Set()  PCout(13)=1   //SCL

#define OLED_SDIN_Clr()  PCout(14)=0   //SDA	= PC14
#define OLED_SDIN_Set()  PCout(14)=1   //SDA

#define OLED_CMD  0	//д����
#define OLED_DATA 1	//д����


void OLED_WR_Byte(u8 dat,u8 cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);		   				   		    
void OLED_Init(void);
void OLED_Clear(void);

void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode);
void OLED_ShowNumber(u8 x,u8 y,u32 num,u8 len,u8 size);
void OLED_ShowString(u8 x,u8 y,const char *p);


//demo
void oled_show(void);;

#endif  

