/**
  ******************************************************************************
  * @file    :pwm.c
  * @author  :HienND
  * @version :v1_00
  * @date    :27/05/2020
	* @brief   :
  *          Board H-Brigde use FET (+IC): BTS79xx (796x,797x ..) 
  *          Link: https://item.taobao.com/item.htm?spm=a1z10.5-c-s.w4002-15726392046.36.472c7b0b2IjhmD&id=551092190133  
  *          Logic: The same with L298N
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
#include "pwm.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void SetPwmLEFT(int16_t udk);
void SetPwmRIGHT(int16_t udk);
/* External variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Set PWM at TIM8 (Motor LEFT)
  * @param  Int16_t udk (PWM): 0 - 999 (0-100%)
  * @retval void
  */
void SetPwmLEFT(int16_t udk)
{
	if (udk > 0) {	// UP = PWM active LOW
		TIM8->CCR1 = udk;
		TIM8->CCER |= TIM_CCER_CC1P;
		GPIOC->ODR |= (GPIO_PIN_4); 	//Set
	}
	else if (udk < 0){	//DOWN = PWM active HIGH
		TIM8->CCR1 = -udk;
		TIM8->CCER &= ~TIM_CCER_CC1P;
		GPIOC->ODR &= ~(GPIO_PIN_4); 	//Reset
	}
	else { //Motor STOP
		TIM8->CCR1 = 0;
		TIM8->CCER |= TIM_CCER_CC1P;
		GPIOC->ODR |= (GPIO_PIN_4); 	//Set
	}
}
/**
  * @brief  Set PWM at TIM8 (Motor RIGHT)
  * @param  Int16_t udk (PWM): 0 - 999 (0-100%)
  * @retval void
  */
void SetPwmRIGHT(int16_t udk)
{
	if (udk > 0) {	// UP = PWM active HIGH
		TIM8->CCR2 = udk;
		TIM8->CCER &= ~TIM_CCER_CC2P;
		GPIOB->ODR &= ~(GPIO_PIN_0); //Reset
	}
	else if (udk < 0){	//DOWN 
		TIM8->CCR2 = -udk;
		TIM8->CCER |= TIM_CCER_CC2P;
		GPIOB->ODR |= (GPIO_PIN_0);
	}
	else { //Motor STOP
		TIM8->CCR2 = 0;
		TIM8->CCER &= ~TIM_CCER_CC2P;
		GPIOB->ODR &= ~(GPIO_PIN_0); //Reset
	}
}

/**
  * @brief  Start PWM at TIM8 (Motor LEFT)
  * @param  TIM_HandleTypeDef htim
  * @retval void
  */
void PWM_Start_LEFT(TIM_HandleTypeDef *htim, uint32_t Channel) //htim8
{
	HAL_TIM_PWM_Start(htim, Channel); //TIM_CHANNEL_1);
}
/**
  * @brief  Start PWM at TIM8 (Motor RIGHT)
  * @param  TIM_HandleTypeDef htim
  * @retval void
  */
void PWM_Start_RIGHT(TIM_HandleTypeDef *htim, uint32_t Channel)
{
	HAL_TIM_PWM_Start(htim, Channel); //TIM_CHANNEL_2);
}



/******************* (C) COPYRIGHT 2020 hiennd *****END OF FILE****/
