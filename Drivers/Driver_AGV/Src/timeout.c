/**
  ******************************************************************************
  * @file    :timeout.c
  * @author  :HienND
  * @version :v1_00
  * @date    :Jun 25, 2020
	* @brief   :
  *
  *
  ******************************************************************************
	 @History
  ------------------------------------------------------------------------------
  version  		author			date		      description
  ------------------------------------------------------------------------------
  v1.00			HienND			Jun 25, 2020
	------------------------------------------------------------------------------
**/

/* Includes ------------------------------------------------------------------*/
#include "timeout.h"
#include "main.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
  * @brief  void
  * @param  void
  * @retval void
  */
void TO_ISR()
{

}
void TO_Init()
{
	uint32_t Tickstart = 0;
	uint32_t Timeout = 200;
	if((HAL_GetTick()-Tickstart) > Timeout)
	{
	    //return HAL_TIMEOUT;
	}
}
/******************* (C) COPYRIGHT 2020 hiennd *****END OF FILE****/



