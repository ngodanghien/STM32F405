#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"  	 
#include "delay.h"
  /**************************************************************************
**************************************************************************/		   
u8 OLED_GRAM[128][8];	 
void OLED_Refresh_Gram(void)
{
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //����ҳ��ַ��0~7��
		OLED_WR_Byte (0x00,OLED_CMD);      //������ʾλ�á��е͵�ַ
		OLED_WR_Byte (0x10,OLED_CMD);      //������ʾλ�á��иߵ�ַ   
		for(n=0;n<128;n++)OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA); 
	}   
}

//��OLEDд��һ���ֽڡ�
//dat:Ҫд�������/����
//cmd:����/�����־ 0,��ʾ����;1,��ʾ����;
void OLED_WR_Byte(u8 dat,u8 cmd)
{	
	u8 i;			  
	if(cmd)
	  OLED_RS_Set();
	else 
	  OLED_RS_Clr();		  
	for(i=0;i<8;i++)
	{			  
		OLED_SCLK_Clr();
		if(dat&0x80)
		   OLED_SDIN_Set();
		else 
		   OLED_SDIN_Clr();
		OLED_SCLK_Set();
		dat<<=1;   
	}				 		  
	OLED_RS_Set();   	  
} 

	  	  
//����OLED��ʾ    
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC����
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//�ر�OLED��ʾ     
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC����
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}		   			 
//��������,������,������Ļ�Ǻ�ɫ��!��û����һ��!!!	  
void OLED_Clear(void)  
{  
	u8 i,n;  
	for(i=0;i<8;i++)for(n=0;n<128;n++)OLED_GRAM[n][i]=0X00;  
	OLED_Refresh_Gram();//������ʾ
}
//���� 
//x:0~127
//y:0~63
//t:1 ��� 0,���				   
void OLED_DrawPoint(u8 x,u8 y,u8 t)
{
	u8 pos,bx,temp=0;
	if(x>127||y>63)return;//������Χ��.
	pos=7-y/8;
	bx=y%8;
	temp=1<<(7-bx);
	if(t)OLED_GRAM[x][pos]|=temp;
	else OLED_GRAM[x][pos]&=~temp;	    
}

//��ָ��λ����ʾһ���ַ�,���������ַ�
//x:0~127
//y:0~63
//mode:0,������ʾ;1,������ʾ				 
//size:ѡ������ 16/12 
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode)
{      			    
	u8 temp,t,t1;
	u8 y0=y;
	chr=chr-' ';//�õ�ƫ�ƺ��ֵ				   
    for(t=0;t<size;t++)
    {   
		if(size==12)temp=oled_asc2_1206[chr][t];  //����1206����
		else temp=oled_asc2_1608[chr][t];		 //����1608���� 	                          
        for(t1=0;t1<8;t1++)
		{
			if(temp&0x80)OLED_DrawPoint(x,y,mode);
			else OLED_DrawPoint(x,y,!mode);
			temp<<=1;
			y++;
			if((y-y0)==size)
			{
				y=y0;
				x++;
				break;
			}
		}  	 
    }          
}
//m^n����
u32 oled_pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}				  
//��ʾ2������
//x,y :�������	 
//len :���ֵ�λ��
//size:�����С
//mode:ģʽ	0,���ģʽ;1,����ģʽ
//num:��ֵ(0~4294967295);	 		  
void OLED_ShowNumber(u8 x,u8 y,u32 num,u8 len,u8 size)
{         	
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size/2)*t,y,' ',size,1);
				continue;
			}else enshow=1; 
		 	 
		}
	 	OLED_ShowChar(x+(size/2)*t,y,temp+'0',size,1); 
	}
} 
//��ʾ�ַ���
//x,y:�������  
//*p:�ַ�����ʼ��ַ
//��16����
void OLED_ShowString(u8 x,u8 y,const char *p)
{
#define MAX_CHAR_POSX 122
#define MAX_CHAR_POSY 58          
    while(*p!='\0')
    {       
        if(x>MAX_CHAR_POSX){x=0;y+=16;}
        if(y>MAX_CHAR_POSY){y=x=0;OLED_Clear();}
        OLED_ShowChar(x,y,*p,12,1);	 
        x+=8;
        p++;
    }  
}	   
//��ʼ��OLED					    
void OLED_Init(void)
{
/* 	
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC  , ENABLE);  //ʹ��GPIOC
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR  , ENABLE);    //PWRʹ��
	PWR_BackupAccessCmd(ENABLE);         //�����޸�RTC �ͺ󱸼Ĵ���  
	RCC_LSEConfig(RCC_LSE_OFF);       //�ر��ⲿ�����ⲿʱ���źŹ��� ��PC13 PC14 PC15 �ſ��Ե���ͨIO�á�
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 |GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;  //PC0 PC13 PC14 PC15
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;   //���
	GPIO_InitStructure.GPIO_OType =GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_2MHz;  //2M
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//�����趨������ʼ��GPIO
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIOC->BSRRH = GPIO_Pin_0;
	GPIOC->BSRRH = GPIO_Pin_13;
	GPIOC->BSRRH = GPIO_Pin_14;
	GPIOC->BSRRH = GPIO_Pin_15;
	*/
	OLED_RST_Clr();
	delay_ms(100);
	OLED_RST_Set(); 					  
	OLED_WR_Byte(0xAE,OLED_CMD); //�ر���ʾ
	OLED_WR_Byte(0xD5,OLED_CMD); //����ʱ�ӷ�Ƶ����,��Ƶ��
	OLED_WR_Byte(80,OLED_CMD);   //[3:0],��Ƶ����;[7:4],��Ƶ��
	OLED_WR_Byte(0xA8,OLED_CMD); //��������·��
	OLED_WR_Byte(0X3F,OLED_CMD); //Ĭ��0X3F(1/64) 
	OLED_WR_Byte(0xD3,OLED_CMD); //������ʾƫ��
	OLED_WR_Byte(0X00,OLED_CMD); //Ĭ��Ϊ0
	OLED_WR_Byte(0x40,OLED_CMD); //������ʾ��ʼ�� [5:0],����.
													    
	OLED_WR_Byte(0x8D,OLED_CMD); //��ɱ�����
	OLED_WR_Byte(0x14,OLED_CMD); //bit2������/�ر�
	OLED_WR_Byte(0x20,OLED_CMD); //�����ڴ��ַģʽ
	OLED_WR_Byte(0x02,OLED_CMD); //[1:0],00���е�ַģʽ;01���е�ַģʽ;10,ҳ��ַģʽ;Ĭ��10;
	OLED_WR_Byte(0xA1,OLED_CMD); //���ض�������,bit0:0,0->0;1,0->127;
	OLED_WR_Byte(0xC0,OLED_CMD); //����COMɨ�跽��;bit3:0,��ͨģʽ;1,�ض���ģʽ COM[N-1]->COM0;N:����·��
	OLED_WR_Byte(0xDA,OLED_CMD); //����COMӲ����������
	OLED_WR_Byte(0x12,OLED_CMD); //[5:4]����
		 
	OLED_WR_Byte(0x81,OLED_CMD); //�Աȶ�����
	OLED_WR_Byte(0xEF,OLED_CMD); //1~255;Ĭ��0X7F (��������,Խ��Խ��)
	OLED_WR_Byte(0xD9,OLED_CMD); //����Ԥ�������
	OLED_WR_Byte(0xf1,OLED_CMD); //[3:0],PHASE 1;[7:4],PHASE 2;
	OLED_WR_Byte(0xDB,OLED_CMD); //����VCOMH ��ѹ����
	OLED_WR_Byte(0x30,OLED_CMD); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

	OLED_WR_Byte(0xA4,OLED_CMD); //ȫ����ʾ����;bit0:1,����;0,�ر�;(����/����)
	OLED_WR_Byte(0xA6,OLED_CMD); //������ʾ��ʽ;bit0:1,������ʾ;0,������ʾ	    						   
	OLED_WR_Byte(0xAF,OLED_CMD); //������ʾ	 
	OLED_Clear();
}  

//demo:
int Encoder_A,Encoder_B; 
long int Target_A,Target_B;
float Pitch,Roll,Yaw,Move_X,Move_Y,Move_Z;
int Voltage;    
u8 Run_Flag=0;  

void oled_show(void)
{
	//=============��1����ʾ3��Ƕ�===============//	
	OLED_ShowString(0,0,"X:");
	if(Pitch<0)		OLED_ShowNumber(15,0,Pitch+360,3,12);
	else					OLED_ShowNumber(15,0,Pitch,3,12);	
		 
	OLED_ShowString(40,0,"Y:");
	if(Roll<0)		OLED_ShowNumber(55,0,Roll+360,3,12);
	else					OLED_ShowNumber(55,0,Roll,3,12);	

	 OLED_ShowString(80,0,"Z:");
	if(Yaw<0)		  OLED_ShowNumber(95,0,Yaw+360,3,12);
	else					OLED_ShowNumber(95,0,Yaw,3,12);		


	//=============��ʾ���A��״̬=======================//	
	if( Target_A<0)		    OLED_ShowString(00,10,"-"),
												OLED_ShowNumber(15,10,-Target_A,6,12);
	else                 	OLED_ShowString(0,10,"+"),
												OLED_ShowNumber(15,10, Target_A,6,12); 
	
	if( Encoder_A<0)		  OLED_ShowString(60,10,"-"),
												OLED_ShowNumber(75,10,-Encoder_A,6,12);
	else                 	OLED_ShowString(60,10,"+"),
												OLED_ShowNumber(75,10, Encoder_A,6,12);
	//=============��ʾ���B��״̬=======================//	
		if( Target_B<0)		  OLED_ShowString(00,20,"-"),
												OLED_ShowNumber(15,20,-Target_B,6,12);
	else                 	OLED_ShowString(0,20,"+"),
												OLED_ShowNumber(15,20, Target_B,6,12); 
		
	if( Encoder_B<0)		  OLED_ShowString(60,20,"-"),
												OLED_ShowNumber(75,20,-Encoder_B,6,12);
	else                 	OLED_ShowString(60,20,"+"),
												OLED_ShowNumber(75,20, Encoder_B,6,12);
// 		//=============��ʾ���C��״̬=======================//	
//		  if( Target_C<0)		  OLED_ShowString(00,30,"-"),
//		                      OLED_ShowNumber(15,30,-Target_C,6,12);
//		else                 	OLED_ShowString(0,30,"+"),
//		                      OLED_ShowNumber(15,30, Target_C,6,12); 
//		  
//		if( Encoder_C<0)		  OLED_ShowString(60,30,"-"),
//		                      OLED_ShowNumber(75,30,-Encoder_C,6,12);
//		else                 	OLED_ShowString(60,30,"+"),
//		                      OLED_ShowNumber(75,30, Encoder_C,6,12);	
//		//=============��ʾ���D��״̬=======================//	
//		  if( Target_D<0)	  	OLED_ShowString(00,40,"-"),
//		                      OLED_ShowNumber(15,40,-Target_D,6,12);
//		else                 	OLED_ShowString(0,40,"+"),
//		                      OLED_ShowNumber(15,40, Target_D,6,12); 
//		
//		if( Encoder_D<0)		    OLED_ShowString(60,40,"-"),
//		                      OLED_ShowNumber(75,40,-Encoder_D,6,12);
//		else                 	OLED_ShowString(60,40,"+"),
//		                      OLED_ShowNumber(75,40, Encoder_D,6,12);

	//=============��������ʾ��ѹ=======================//
	if(Run_Flag==0)       OLED_ShowString(00,50,"VELOCITY");
												OLED_ShowString(88,50,".");
												OLED_ShowString(110,50,"V");
												OLED_ShowNumber(75,50,Voltage/100,2,12);
												OLED_ShowNumber(98,50,Voltage%100,2,12);
	 if(Voltage%100<10) 	OLED_ShowNumber(92,50,0,2,12);
	//=============ˢ��=======================//
	OLED_Refresh_Gram();		
}




