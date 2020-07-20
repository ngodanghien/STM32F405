/**
  ******************************************************************************
  * @file   :	i2cIO.h
  * @author :	HienND
  * @version:	v1_00 
  * @date   : 27/05/2020 
  * @brief  :	Header file for i2cIO.c module.
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
#ifndef __I2CIO_H
#define __I2CIO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
//#include "main.h"
#include "stm32f4xx.h"
#include "gpio.h"
#include "delay.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define SDA_IN()  {GPIOB->MODER&=~(3<<(11*2));GPIOB->MODER|=0<<11*2;}	//PB9 Input
#define SDA_OUT() {GPIOB->MODER&=~(3<<(11*2));GPIOB->MODER|=1<<11*2;} 	//PB9 Output
 
#define IIC_SCL    PBout(10) //SCL
#define IIC_SDA    PBout(11) //SDA	 
#define READ_SDA   PBin(11)  //ReadSDA 


/* Initialization and de-initialization functions ----------------------------*/
void IIC_Init(void);
/* Exported functions prototypes ---------------------------------------------*/
int IIC_Start(void);
void IIC_Stop(void);
void IIC_Send_Byte(u8 txd);
u8 IIC_Read_Byte(unsigned char ack);
int IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);

/* IO operation functions ----------------------------------------------------*/
void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	 
unsigned char I2C_Readkey(unsigned char I2C_Addr);

unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr);
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);

u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data);
u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data);
u8 IICwriteBit(u8 dev,u8 reg,u8 bitNum,u8 data);
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data);

int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

/* Private defines -----------------------------------------------------------*/
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

