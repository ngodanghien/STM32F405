/**
 ******************************************************************************
 * @file    :i2cIO.c
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
#include "i2cIO.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
 * @brief  IIC_Init : Đã Test --> PASSED
 * @param  void		: Delay(1) chạy được ở 200kHz
 * @retval void
 */
void IIC_Init(void)
{			
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOB_CLK_ENABLE();
	/**I2C2 GPIO Configuration
	    PB10     ------> I2C2_SCL
	    PB11     ------> I2C2_SDA
	 */
	GPIO_InitStruct.Pin = IMU_I2C2_SCL_Pin|IMU_I2C2_SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	//
	IIC_SCL=1;
	IIC_SDA=1;
}
int IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA=1;
	if(!READ_SDA)return 0;	
	IIC_SCL=1;
	delay_us(1);
	IIC_SDA=0;//START:when CLK is high,DATA change form high to low
	if(READ_SDA)return 0;
	delay_us(1);
	IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
	return 1;
}
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
	delay_us(1);
	IIC_SCL=1; 
	IIC_SDA=1;//����I2C���߽����ź�
	delay_us(1);							   	
}
int IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	IIC_SDA=1;
	delay_us(1);	   
	IIC_SCL=1;
	delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			IIC_Stop();
			return 0;
		}
		delay_us(1);
	}
	IIC_SCL=0;//ʱ�����0 	   
	return 1;  
}
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;
}
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;
}
void IIC_Send_Byte(u8 txd)
{                        
	u8 t;
	SDA_OUT(); 	    
	IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
	for(t=0;t<8;t++)
	{
		IIC_SDA=(txd&0x80)>>7;
		txd<<=1;
		delay_us(1);   
		IIC_SCL=1;
		delay_us(1); 
		IIC_SCL=0;	
		delay_us(1);
	}
} 
/**
 * @brief  bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
 * @param  void
 * @retval void
 */
int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
	int i;
	if (!IIC_Start())
		return 1;
	IIC_Send_Byte(addr << 1 );
	if (!IIC_Wait_Ack()) {
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(reg);
	IIC_Wait_Ack();
	for (i = 0; i < len; i++) {
		IIC_Send_Byte(data[i]);
		if (!IIC_Wait_Ack()) {
			IIC_Stop();
			return 0;
		}
	}
	IIC_Stop();
	return 0;
}
/**
 * @brief  bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
 * @param  void
 * @retval void
 */
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	if (!IIC_Start())
		return 1;
	IIC_Send_Byte(addr << 1);
	if (!IIC_Wait_Ack()) {
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(reg);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte((addr << 1)+1);
	IIC_Wait_Ack();
	while (len) {
		if (len == 1)
			*buf = IIC_Read_Byte(0);
		else
			*buf = IIC_Read_Byte(1);
		buf++;
		len--;
	}
	IIC_Stop();
	return 0;
}
/**
 * @brief  u8 IIC_Read_Byte(unsigned char ack)
 * @param  void
 * @retval void
 */
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
	for(i=0;i<8;i++ )
	{
		IIC_SCL=0;
		delay_us(2);
		IIC_SCL=1;
		receive<<=1;
		if(READ_SDA)receive++;
		delay_us(2); 
	}
	if (ack)
		IIC_Ack(); //����ACK
	else
		IIC_NAck();//����nACK
	return receive;
}
/**
 * @brief  unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
 * @param  void
 * @retval void
 */
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
{
	unsigned char res=0;

	IIC_Start();	
	IIC_Send_Byte(I2C_Addr);	   //����д����
	res++;
	IIC_Wait_Ack();
	IIC_Send_Byte(addr); res++;  //���͵�ַ
	IIC_Wait_Ack();	  
	//IIC_Stop();//����һ��ֹͣ����	
	IIC_Start();
	IIC_Send_Byte(I2C_Addr+1); res++;          //�������ģʽ			   
	IIC_Wait_Ack();
	res=IIC_Read_Byte(0);	   
	IIC_Stop();//����һ��ֹͣ����

	return res;
}
/**
 * @brief  u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
 * @param  void
 * @retval void
 */
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data){
	u8 count = 0;

	IIC_Start();
	IIC_Send_Byte(dev);	   //����д����
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //���͵�ַ
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(dev+1);  //�������ģʽ	
	IIC_Wait_Ack();

	for(count=0;count<length;count++){

		if(count!=length-1)data[count]=IIC_Read_Byte(1);  //��ACK�Ķ�����
		else  data[count]=IIC_Read_Byte(0);	 //���һ���ֽ�NACK
	}
	IIC_Stop();//����һ��ֹͣ����
	return count;
}

/**
 * @brief  u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
 * @param  void
 * @retval void
 */
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data){

	u8 count = 0;
	IIC_Start();
	IIC_Send_Byte(dev);	   //����д����
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //���͵�ַ
	IIC_Wait_Ack();
	for(count=0;count<length;count++){
		IIC_Send_Byte(data[count]); 
		IIC_Wait_Ack(); 
	}
	IIC_Stop();//����һ��ֹͣ����

	return 1; //status == 0;
}
/**
 * @brief
 * @param  void
 * @retval void
 */
/**
 * @brief  u8 IICreadByte(u8 dev, u8 reg, u8 *data)
 * @param  void
 * @retval void
 */
u8 IICreadByte(u8 dev, u8 reg, u8 *data){
	*data=I2C_ReadOneByte(dev, reg);
	return 1;
}

/**
 * @brief  unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
 * @param  void
 * @retval void
 */
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data){
	return IICwriteBytes(dev, reg, 1, &data);
}
/**
 * @brief  u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
 * @param  void
 * @retval void
 */
u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
{

	u8 b;
	if (IICreadByte(dev, reg, &b) != 0) {
		u8 mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
		data <<= (8 - length);
		data >>= (7 - bitStart);
		b &= mask;
		b |= data;
		return IICwriteByte(dev, reg, b);
	} else {
		return 0;
	}
}
/**
 * @brief  u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
 * @param  void
 * @retval void
 */
u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data){
	u8 b;
	IICreadByte(dev, reg, &b);
	b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	return IICwriteByte(dev, reg, b);
}

/******************* (C) COPYRIGHT 2020 hiennd *****END OF FILE****/

