/* 
*   BSD 2-Clause License
*   Copyright (c) 2022, LiteEMF
*   All rights reserved.
*   This software component is licensed by LiteEMF under BSD 2-Clause license,
*   the "License"; You may not use this file except in compliance with the
*   License. You may obtain a copy of the License at:
*       opensource.org/licenses/BSD-2-Clause
* 
*/

/************************************************************************************************************
**	Description:	
************************************************************************************************************/
#include  "api/api_gpio.h"

/******************************************************************************************************
** Defined
*******************************************************************************************************/
/* GPIO Bit-Banding Macro Definition */
#define BITBAND(adr, number)  (((adr) & 0xF0000000)+0x2000000+(((adr) & 0xFFFFF)<<5)+((number)<<2)) 
#define MEM_ADR(adr)  *((volatile unsigned long  *)(adr)) 
#define BIT_ADR(adr, number)  MEM_ADR(BITBAND(adr, number)) 

/* GPIO Output Address Mapping */
#define GPIOA_ODR_Adr    (GPIOA_BASE+12) 						//0x4001080C 
#define GPIOB_ODR_Adr    (GPIOB_BASE+12) 						//0x40010C0C 
#define GPIOC_ODR_Adr    (GPIOC_BASE+12) 						//0x4001100C 
#define GPIOD_ODR_Adr    (GPIOD_BASE+12) 						//0x4001140C 
    
/* GPIO Input Address Mapping */
#define GPIOA_IDR_Adr    (GPIOA_BASE+8) 						//0x40010808 
#define GPIOB_IDR_Adr    (GPIOB_BASE+8) 						//0x40010C08 
#define GPIOC_IDR_Adr    (GPIOC_BASE+8) 						//0x40011008 
#define GPIOD_IDR_Adr    (GPIOD_BASE+8) 						//0x40011408 
/* GPIO Output */
#define PAout(n)   BIT_ADR(GPIOA_ODR_Adr,n) 
#define PBout(n)   BIT_ADR(GPIOB_ODR_Adr,n)  
#define PCout(n)   BIT_ADR(GPIOC_ODR_Adr,n)
#define PDout(n)   BIT_ADR(GPIOD_ODR_Adr,n) 

/* GPIO Input */
#define PAin(n)    BIT_ADR(GPIOA_IDR_Adr,n)   
#define PBin(n)    BIT_ADR(GPIOB_IDR_Adr,n)     
#define PCin(n)    BIT_ADR(GPIOC_IDR_Adr,n)  
#define PDin(n)    BIT_ADR(GPIOD_IDR_Adr,n)  
/******************************************************************************************************
**	static Parameters
*******************************************************************************************************/

/******************************************************************************************************
**	public Parameters
*******************************************************************************************************/

/*****************************************************************************************************
**	static Function
******************************************************************************************************/

/*****************************************************************************************************
**  Function
******************************************************************************************************/

/*******************************************************************
** Parameters:		
** Returns:	
** Description:		
*******************************************************************/
void hal_gpio_mode(pin_t pin, uint8_t mode)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = 0x01<<(pin & 0X0F);              
    GPIO_InitStructure.GPIO_Mode = (GPIOMode_TypeDef)mode;           
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;

    switch(pin>>4){
        case 0:
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
            GPIO_Init(GPIOA, &GPIO_InitStructure);
            break;
        case 1:
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
            GPIO_Init(GPIOB, &GPIO_InitStructure);
            break;
        case 2:
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
            GPIO_Init(GPIOC, &GPIO_InitStructure);
            break;
        case 3:
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);
            GPIO_Init(GPIOD, &GPIO_InitStructure);
            break;

        default :break;
	}
}
void hal_gpio_dir(pin_t pin, pin_dir_t dir, pin_pull_t pull)
{
	if(PIN_OUT == dir){
		if(PIN_PULL_OD == pull){
			hal_gpio_mode(pin,GPIO_Mode_Out_OD);
		}else{
			hal_gpio_mode(pin,GPIO_Mode_Out_PP);
		}
	}else{
		switch(pull){
		case PIN_PULLNONE:
			hal_gpio_mode(pin,GPIO_Mode_IN_FLOATING);
			break;
		case PIN_PULLUP:
			hal_gpio_mode(pin,GPIO_Mode_IPU);
			break;
		case PIN_PULLDOWN:
			hal_gpio_mode(pin,GPIO_Mode_IPD);
			break;			
		}
	}
}
uint32_t hal_gpio_in(pin_t pin)
{
    uint32_t value = 0;
	switch(pin>>4){
        case 0:
            value = PAin(pin&0X0F);
            break;
        case 1:
            value = PBin(pin&0X0F);
            break;
        case 2:
            value = PCin(pin&0X0F);
            break;
        case 3:
            value = PDin(pin&0X0F);
            break;
        default :break;
	}

	if(value) value = 1;

	return value;
}
void hal_gpio_out(pin_t pin, uint8_t value)
{
	switch(pin>>4){
        case 0:
            PAout(pin&0X0F) = value;
            break;
        case 1:
            PBout(pin&0X0F) = value;
            break;
        case 2:
            PCout(pin&0X0F) = value;
            break;
        case 3:
            PDout(pin&0X0F) = value;
            break;
        default :break;
	}
}








