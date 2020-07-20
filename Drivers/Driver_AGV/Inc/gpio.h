/**
  ******************************************************************************
  * @file   :	gpio.h
  * @author :	HienND
  * @version:	v1_00 
  * @date   : 27/05/2020 
  * @brief  :	Header file for gpio.c module.
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
#ifndef __GPIO_H
#define __GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
#include "stm32f4xx.h"
	
/* Exported types ------------------------------------------------------------*/
typedef unsigned          char u8;
typedef unsigned short     int u16;
typedef unsigned           int u32;
	
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define GPIOA_ODR  			(*(volatile uint32_t *)GPIOA_ODR_Addr)
#define GPIOB_ODR  			(*(volatile uint32_t *)GPIOB_ODR_Addr)
	
#define PA_SET(pin)			(GPIOA_ODR |= 1UL << (uint32_t)pin)			// Choose a pin number from 0 to 15
#define PA_CLEAR(pin)		(GPIOA_ODR &= ~(1UL << (uint32_t)pin))	// ex: PA_CLEAR(13)
#define PB_SET(pin)			(GPIOB_ODR |= 1UL << (uint32_t)pin)			// ex: PB_SET(13)
#define PB_CLEAR(pin)		(GPIOB_ODR &= ~(1UL << (uint32_t)pin))	// Choose a pin number from 0 to 15
#define PB_TOGGLE(pin)			(GPIOB_ODR ^= 1UL << (uint32_t)pin)

#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))  

#define GPIOA_ODR_Addr    (GPIOA_BASE+20) //0x40020014
#define GPIOB_ODR_Addr    (GPIOB_BASE+20) //0x40020414 
#define GPIOC_ODR_Addr    (GPIOC_BASE+20) //0x40020814 
#define GPIOD_ODR_Addr    (GPIOD_BASE+20) //0x40020C14 
#define GPIOE_ODR_Addr    (GPIOE_BASE+20) //0x40021014 
#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40021414    
#define GPIOG_ODR_Addr    (GPIOG_BASE+20) //0x40021814   
#define GPIOH_ODR_Addr    (GPIOH_BASE+20) //0x40021C14    
#define GPIOI_ODR_Addr    (GPIOI_BASE+20) //0x40022014 

#define GPIOA_IDR_Addr    (GPIOA_BASE+16) //0x40020010 
#define GPIOB_IDR_Addr    (GPIOB_BASE+16) //0x40020410 
#define GPIOC_IDR_Addr    (GPIOC_BASE+16) //0x40020810 
#define GPIOD_IDR_Addr    (GPIOD_BASE+16) //0x40020C10 
#define GPIOE_IDR_Addr    (GPIOE_BASE+16) //0x40021010 
#define GPIOF_IDR_Addr    (GPIOF_BASE+16) //0x40021410 
#define GPIOG_IDR_Addr    (GPIOG_BASE+16) //0x40021810 
#define GPIOH_IDR_Addr    (GPIOH_BASE+16) //0x40021C10 
#define GPIOI_IDR_Addr    (GPIOI_BASE+16) //0x40022010 

#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)
#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)
#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)
#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)
#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)

#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)

/* Exported functions prototypes ---------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
	
/* Initialization and de-initialization functions ----------------------------*/
/* IO operation functions ----------------------------------------------------*/

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

