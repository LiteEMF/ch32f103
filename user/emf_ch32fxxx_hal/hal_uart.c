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
#include "hw_config.h"
#include "hw_board.h"
#ifdef HW_UART_MAP

#include  "api/api_uart.h"
#include  "api/api_gpio.h"

#include  "api/api_log.h"
/******************************************************************************************************
** Defined
*******************************************************************************************************/
        // #define HW_UART_MAP { \
        //  {PA_09, (pin_t)PIN_NULL,    0,  0,      (uint32_t)USART1,   0},	\
		// 	{PA_02, PA_03,              0,  0X100,  (uint32_t)USART2,   VAL2FLD(UART_PRI,1)},	\
		// }

/******************************************************************************************************
**	static Parameters
*******************************************************************************************************/

/******************************************************************************************************
**	public Parameters
*******************************************************************************************************/

/*****************************************************************************************************
**	static Function
******************************************************************************************************/
void uart_periph_en (USART_TypeDef *puart,FunctionalState enable)
{
    switch((uint32_t)puart){
    case (uint32_t)USART1:
        RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1, enable );
        break;
    case (uint32_t)USART2:
        RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART2, enable );
        break;
    case (uint32_t)USART3:
        RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART3, enable );
        break;
    }
}

static uint32_t get_uart_irqn (USART_TypeDef *puart)
{
    uint32_t APBPeriph = 0;
    
	switch((uint32_t)puart){
    case (uint32_t)USART1:
        APBPeriph = USART1_IRQn;
        break;
    case (uint32_t)USART2:
        APBPeriph = USART2_IRQn;
        break;
    case (uint32_t)USART3:
        APBPeriph = USART3_IRQn;
        break;
    }

    return APBPeriph;
}



uint32_t get_uart_id (USART_TypeDef *puart)
{
    uint8_t i;

    for(i=0; i<m_uart_num; i++){
        if(m_uart_map[i].peripheral == (uint32_t)puart){
            return i;
        }
    }

    return (uint32_t)ID_NULL;
}



void USART1_IRQHandler(void)
{
    volatile uint8_t val;
    uint8_t id;
    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)		//接收完一个字节
    {
        id = get_uart_id(USART1);
        //USART_ClearITPendingBit(TRANSPORT_UART_PORT, USART_IT_RXNE);
        val = USART_ReceiveData(USART1);
        api_uart_rx_hook(id,(uint8_t*)&val,1);
    }
    if(USART_GetFlagStatus(USART1, USART_FLAG_ORE) == SET)
    {
        loge("uoe\n");  //串口硬件缓冲buf溢出 上一个字节还未处理  又收到第二个字节
        //USART_ClearFlag(TRANSPORT_UART_PORT, USART_FLAG_ORE); //清除溢出中断
        USART_ReceiveData(USART1);
    }
}


void USART2_IRQHandler(void)
{
    volatile uint8_t val;
    uint8_t id;

    if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)		//接收完一个字节
    {
        id = get_uart_id(USART2);
        //USART_ClearITPendingBit(TRANSPORT_UART_PORT, USART_IT_RXNE);
        val = USART_ReceiveData(USART2);
        api_uart_rx_hook(id,(uint8_t*)&val,1);
    }
    if(USART_GetFlagStatus(USART2, USART_FLAG_ORE) == SET)
    {
        loge("uoe\n");  //串口硬件缓冲buf溢出 上一个字节还未处理  又收到第二个字节
        //USART_ClearFlag(TRANSPORT_UART_PORT, USART_FLAG_ORE); //清除溢出中断
        USART_ReceiveData(USART2);
    }
}


void USART3_IRQHandler(void)
{
    volatile uint8_t val;
    uint8_t id;

    if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)		//接收完一个字节
    {
        id = get_uart_id(USART3);
        //USART_ClearITPendingBit(TRANSPORT_UART_PORT, USART_IT_RXNE);
        val = USART_ReceiveData(USART3);
        api_uart_rx_hook(id,(uint8_t*)&val,1);
    }
    if(USART_GetFlagStatus(USART3, USART_FLAG_ORE) == SET)
    {
        loge("uoe\n");  //串口硬件缓冲buf溢出 上一个字节还未处理  又收到第二个字节
        //USART_ClearFlag(TRANSPORT_UART_PORT, USART_FLAG_ORE); //清除溢出中断
        USART_ReceiveData(USART3);
    }
}

/*****************************************************************************************************
**  Function
******************************************************************************************************/

/*******************************************************************
** Parameters:		
** Returns:	
** Description:		
*******************************************************************/
bool hal_uart_set_baud(uint8_t id, uint32_t baud)
{
	return false;
}
bool hal_uart_tx(uint8_t id,void *buf,uint16_t len)
{
    uint16_t i;
    USART_TypeDef *puart = (USART_TypeDef *)m_uart_map[id].peripheral;

    for(i=0; i<len; i++){
        while (USART_GetFlagStatus(puart, USART_FLAG_TC) == RESET);
        USART_SendData(puart, (uint16_t)( *((uint8_t*)buf + i) ) );
    }

	return true;
}




bool hal_uart_init(uint8_t id,uint32_t baudrate)
{
    USART_InitTypeDef USART_InitStructure = {0};
    NVIC_InitTypeDef  NVIC_InitStructure = {0};
    USART_TypeDef *puart = (USART_TypeDef *)m_uart_map[id].peripheral;
    uint32_t irq_type = get_uart_irqn(puart);

    uart_periph_en(puart,ENABLE);

	api_gpio_mode(m_uart_map[id].tx, GPIO_Mode_AF_PP);
    if((pin_t)PIN_NULL != m_uart_map[id].rx){
	    api_gpio_mode(m_uart_map[id].rx, GPIO_Mode_IPU);
    }
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1, ENABLE );
	
	USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx;
	if((pin_t)PIN_NULL != m_uart_map[id].rx){
    	USART_InitStructure.USART_Mode |= USART_Mode_Rx;
	}

    USART_Init( puart, &USART_InitStructure );
    // puart->STATR = 0x00C0;
    
    if((pin_t)PIN_NULL != m_uart_map[id].rx){
        uint8_t priority = UART_PRI_ATT(id);
        if(0 == priority) priority = 1;

        USART_ITConfig( puart, USART_IT_RXNE, ENABLE );
        NVIC_InitStructure.NVIC_IRQChannel = irq_type;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = priority;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init( &NVIC_InitStructure );
    }

    USART_Cmd( puart, ENABLE );

	return false;
}

bool hal_uart_deinit(uint8_t id)
{
	return false;
}

#endif

