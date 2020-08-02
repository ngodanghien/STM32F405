/**
  ******************************************************************************
  * @file    :delay.c
  * @author  :HienND
  * @version :v1_00
  * @date    :27/05/2020
	* @brief   : 
  *          + Add: TIM6->RCR = 9; Because Max TIM6->PSC = 0xFFFF
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
#include "delay.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void delay_us(uint16_t delay);	//1.25us = 400kHz
void delay_ms(uint16_t period);
/* External variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
  * @brief  delay_us (micro second)
  * @param  uint16_t us < 0xFFFF
  * @retval void
  */
//void delay_us(uint16_t period)
//{
//	//TIM6,7 APB1, max 84 MHz
//	__HAL_RCC_TIM6_CLK_ENABLE();
//
//	TIM6->PSC = 83;		// clk = SystemCoreClock / 4 / (PSC+1) *2 = 1MHz
//	TIM6->ARR = period-1;
//	TIM6->CNT = 0;
//	TIM6->EGR = 1;		// update registers;
//
//	TIM6->SR  = 0;		// clear overflow flag
//	TIM6->CR1 = 1;		// enable Timer6
//
//	while (!TIM6->SR);
//
//	TIM6->CR1 = 0;		// stop Timer6
//
//	__HAL_RCC_TIM6_CLK_DISABLE();
//}
/**
  * @brief  delay_ms (mili second)
  * PASSED, Note: real delay_ms = 0.1ms, so, period = period*10
  * @param  uint16_t ms
  * @retval void
  */
void delay_ms(uint16_t period)
{
	__HAL_RCC_TIM6_CLK_ENABLE();	// TIM_CLK = 84Mhz
	TIM6->PSC = 8399;		// Update_event = TIM_CLK/((PSC + 1)*(ARR + 1)) = 10kHz = 0.1ms
	// 0xFFFF = 6553.5 ms = 6.5s
	TIM6->ARR = period*10-1; //convert 0.1ms --> 1ms <=> *10;
	TIM6->CNT = 0;
	TIM6->EGR = 1;		// update registers;

	TIM6->SR  = 0;		// clear overflow flag
	TIM6->CR1 = 1;		// enable Timer6

	while (!TIM6->SR);
	
	TIM6->CR1 = 0;		// stop Timer6
	__HAL_RCC_TIM6_CLK_DISABLE();
}

/**	[PASSED] *14 = 1.25us ; Gía trị delay càng lớn càng chính xác
  * @brief  delay cycles us
  * @retval void
  */
void delay_us(uint16_t delay)
{
	//delay *= 14; //[OK ] 168/12;
	//delay *= 6;		//Dưới 1us, ở tốc độ này: I2C mềm chạy đúng: 500kHz
	delay *= 10;		//307kHz
	while(--delay);
}
/******************* (C) COPYRIGHT 2020 hiennd *****END OF FILE****/

