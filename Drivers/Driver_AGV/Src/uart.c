/**
  ******************************************************************************
  * @file    :uart.c
  * @author  :HienND
  * @version :v1_00
  * @date    :19/06/2020
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
#include "uart.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
float txBuffFload[10];	//bat buoc phai dung global.
double txBuffDouble[10];
/* Private function prototypes -----------------------------------------------*/
/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart2;					//UART

/* Exported functions --------------------------------------------------------*/

/* Những tham số cần gửi lên để PC nhận dạng (cũng như vẽ đồ thị)
 * 1. Setpoint (Left + Right)				= 2*4byte(float)
 * 2. Giá trị PWM hiện tại (Left + Right)	= 2*4byte(float)
 * 3. Tốc độ Rad/s hiện tại (Left + Right)	= 2*4byte(float)
 * ----------------------------------------------------------Sum = 8x3 = 24 bytes
 */
//#define MAX_LEN_RX_FLOAT 	20
//float result[MAX_LEN_RX_FLOAT] = {0};	//global
/** [PASSED] [04/08/2020]
  * @brief  UartTX_Float
  * @param  float *arrTx, int lengthF
  * @retval void
  * ref: https://www.microchip.com/forums/m590535.aspx
  */
void UartRX_Float(float *result, uint8_t *buffRx, int lengthF)
{
	//0. ex: 0.5f = 0x0000003f (thứ tự lưu trong buffer) = litle-endian = MCU
	//buffRx[0] = 'S' 83  == header
	if (buffRx[0] == 'S')
	{
		uint8_t *pUint8 = buffRx+1;		//Cộng thêm 1 để trừ header 'S' ra.
		for (int i=0; i<lengthF;i++)	//số float cần xử lý (ex:2)
		{
			result[i] = *(float *)pUint8;	//*(float *)&buffRx;
			pUint8 += 4;
		}
	}
	return ;
}
/**
  * @brief  UartTX_Float
  * @param  float *arrTx, int lengthF
  * @retval void
  */
void UartTX_Float(const float *arrTx, int lengthF)
{
	//0. Convert length Float -> length Byte + 2 byte "\r\n"
	int length = lengthF*4 + 2;
	//1. copy float *arrTx to txbuff
	for (int i=0;i<lengthF;i++) txBuffFload[i] = arrTx[i]; //Dư 01 float (4byte) để lưu "\r\n"
	//2.Add "\r\n" into terminal.
	*((uint32_t*)txBuffFload + lengthF) = 0x00000A0D;
	//3. Send Tx buffer
	//HAL_UART_DMAStop(_huart);
	//HAL_UART_Transmit_DMA(&huart2,(uint8_t*)txBuffFload,length);	//OK = UART
	HAL_UART_Transmit_DMA(&huart2,(uint8_t*)txBuffFload,length-1); //end = '\r'
}
/**
  * @brief  UartTX_Double
  * @param  double *arrTx, int lengthD
  * @retval void
  */
void UartTX_Double(const double *arrTx, int lengthD)
{
	//0. Convert length Double -> length Byte + 2 byte "\r\n"
	int length = lengthD*8 + 2;
	//1. copy float *arrTx to txbuff
	for (int i=0;i<lengthD;i++) txBuffDouble[i] = arrTx[i]; //Dư 01 float (4byte) để lưu "\r\n"
	//2.Add "\r\n" into terminal.
	*((uint32_t*)txBuffDouble + lengthD*2) = 0x00000A0D;
	//3. Send Tx buffer
	//HAL_UART_DMAStop(_huart);
	HAL_UART_Transmit_DMA(&huart2,(uint8_t*)txBuffDouble,length);
}

/**
  * @brief  UartTX_Float
  * @param  float *arrTx, int lengthF
  * @retval void
  */
void UartWiFiTX_Float(const float *arrTx, int lengthF)
{
	//0. Convert length Float -> length Byte + 2 byte "\r\n"
	int length = lengthF*4;
	//1. copy float *arrTx to txbuff
	for (int i=0;i<lengthF;i++) txBuffFload[i] = arrTx[i]; //Dư 01 float (4byte) để lưu "\r\n"
	//2.Add "\r\n" into terminal.
	*((uint32_t*)txBuffFload + lengthF) = 0x00000A0D;
	//3. Send Tx buffer
	HAL_UART_Transmit_DMA(&huart2,(uint8_t*)txBuffFload,length);	//WiFi =  OK
}
/******************* (C) COPYRIGHT 2020 hiennd *****END OF FILE****/


