/**
  ******************************************************************************
  * @file    :encoder.c
  * @author  :HienND
  * @version :v1_00
  * @date    :27/05/2020
	* @brief   :
  *          Encoder A|B: 500 pulse/round	
  *          Motor Ratio: 1:27 
  ******************************************************************************
	 @History
  ------------------------------------------------------------------------------
  version  		author			date		      description
  ------------------------------------------------------------------------------
  v1.00			  HienND			27/05/2020
	------------------------------------------------------------------------------
**/	
/* Includes ------------------------------------------------------------------*/
#include "encoder.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define PI								3.1415926535897932384626433832795
#define ENC_RESOLUTION 		500	//Encoder Resolution
#define ENC_MODE					4		//Mode Encoder Interface
#define MOTOR_RATIO				27 	//gear ratio motor

/* Private macro -------------------------------------------------------------*/
#define ENC_PPR	(ENC_REOLUTION*ENC_MODE*MOTOR_RATIO)	//Pulse per Round = 500*4*27=54000
#define ENC_GET (2*PI/ENC_PPR/TIME_SAMPE)								// 0.0232710566932577	(rad/s)
/* Private variables ---------------------------------------------------------*/
double encGetResolution = 0.0232710566932577;//(2*PI/ENC_PPR/TIME_SAMPE); //Speed (rad/s) = dp*2*PI()/(ENC_RELSOLUTION*ENC_MODE*MOTOR_RATIO)/TIME_SAMPE		
/* Private function prototypes -----------------------------------------------*/
double GetSpeed_LEFT(void);
double GetSpeed_RIGHT(void);
/* External variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Get speed (rad/s) from Encoder Interface (TIM2)
  * @param  void
  * @retval double speed Motor (rad/s): >0 : Forward ; <0: Reverse
  */
double GetSpeed_LEFT(void)
{
	static int32_t 	p = 0, p_pre = 0, dp = 0;
	
	p = (int32_t)TIM2->CNT;
	dp = p - p_pre; //+/- (speed)
	if (dp > 32768)
		dp -= 65536;
	else if (dp < -32768)
		dp += 65536;
	p_pre = p;
	
	return (encGetResolution*dp);	//rad/s
}

/**
  * @brief  Get speed (rad/s) from Encoder Interface (TIM3)
  * @param  void
  * @retval double speed Motor (rad/s): >0 : Forward ; <0: Reverse
  */
double GetSpeed_RIGHT(void)
{
	static int32_t 	p = 0, p_pre = 0, dp = 0;
	
	p = (int32_t)TIM3->CNT;
	dp = p - p_pre; //+/- (speed)
	if (dp > 32768)
		dp -= 65536;
	else if (dp < -32768)
		dp += 65536;
	p_pre = p;
	
	return (encGetResolution*dp);	//rad/s
}
/**
  * @brief  Start Encoder (TIM2)
  * @param  void
  * @retval void
  */
void EncoderStart_LEFT(TIM_HandleTypeDef *htim) //htim2
{
	HAL_TIM_Encoder_Start(htim,TIM_CHANNEL_ALL);			//htim2: Motor A (Left)
}
/**
  * @brief  Start Encoder (TIM3)
  * @param  void
  * @retval void
  */
void EncoderStart_RIGHT(TIM_HandleTypeDef *htim) //htim3
{
	HAL_TIM_Encoder_Start(htim,TIM_CHANNEL_ALL);	//htim3: Motor B (Right)
}
/**
* @brief TIM_Encoder MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_encoder: TIM_Encoder handle pointer
* @retval None
*/
void HAL_TIM_Encoder_MspInit_1(TIM_HandleTypeDef* htim_encoder)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim_encoder->Instance==TIM2)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM2 GPIO Configuration    
    PA15     ------> TIM2_CH1
    PB3     ------> TIM2_CH2 
    */
    GPIO_InitStruct.Pin = ENCA_A_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(ENCA_A_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ENCA_B_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(ENCA_B_GPIO_Port, &GPIO_InitStruct);
  }
  else if(htim_encoder->Instance==TIM3)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM3 GPIO Configuration    
    PA6     ------> TIM3_CH1
    PA7     ------> TIM3_CH2 
    */
    GPIO_InitStruct.Pin = ENCB_A_Pin|ENCB_B_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  else if(htim_encoder->Instance==TIM4)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM4 GPIO Configuration    
    PB6     ------> TIM4_CH1
    PB7     ------> TIM4_CH2 
    */
    GPIO_InitStruct.Pin = ENCC_A_Pin|ENCC_B_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
  else if(htim_encoder->Instance==TIM5)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM5_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM5 GPIO Configuration    
    PA0-WKUP     ------> TIM5_CH1
    PA1     ------> TIM5_CH2 
    */
    GPIO_InitStruct.Pin = ENCD_A_Pin|ENCD_B_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }

}
/**
* @brief TIM_Encoder MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim_encoder: TIM_Encoder handle pointer
* @retval None
*/
void HAL_TIM_Encoder_MspDeInit_1(TIM_HandleTypeDef* htim_encoder)
{
  if(htim_encoder->Instance==TIM2)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();
  
    /**TIM2 GPIO Configuration    
    PA15     ------> TIM2_CH1
    PB3     ------> TIM2_CH2 
    */
    HAL_GPIO_DeInit(ENCA_A_GPIO_Port, ENCA_A_Pin);
    HAL_GPIO_DeInit(ENCA_B_GPIO_Port, ENCA_B_Pin);
  }
  else if(htim_encoder->Instance==TIM3)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
  
    /**TIM3 GPIO Configuration    
    PA6     ------> TIM3_CH1
    PA7     ------> TIM3_CH2 
    */
    HAL_GPIO_DeInit(GPIOA, ENCB_A_Pin|ENCB_B_Pin);
  }
  else if(htim_encoder->Instance==TIM4)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM4_CLK_DISABLE();
  
    /**TIM4 GPIO Configuration    
    PB6     ------> TIM4_CH1
    PB7     ------> TIM4_CH2 
    */
    HAL_GPIO_DeInit(GPIOB, ENCC_A_Pin|ENCC_B_Pin);
  }
  else if(htim_encoder->Instance==TIM5)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM5_CLK_DISABLE();
  
    /**TIM5 GPIO Configuration    
    PA0-WKUP     ------> TIM5_CH1
    PA1     ------> TIM5_CH2 
    */
    HAL_GPIO_DeInit(GPIOA, ENCD_A_Pin|ENCD_B_Pin);
  }

}
/******************* (C) COPYRIGHT 2020 hiennd *****END OF FILE****/
