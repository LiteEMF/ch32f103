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
#if	defined HW_TIMER_MAP && API_TIMER_ENABLE
#include  "api/api_timer.h"

/******************************************************************************************************
** Defined
*******************************************************************************************************/
        // #define HW_TIMER_MAP { \
        //     {(uintptr_t)TIM3,VAL2FLD(TIMER_FREQ,1000)}, \
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

static void timer_periph_en (TIM_TypeDef *ptimer, FunctionalState enable)
{
	switch((uint32_t)ptimer){
    case (uint32_t)TIM1:
        RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM1, enable );
		break;
    case (uint32_t)TIM2:
        RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, enable );
		break;
    case (uint32_t)TIM3:
        RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM3, enable );
		break;
    case (uint32_t)TIM4:
        RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM4, enable );
		break;
    }
}

static uint32_t get_timer_irqn (TIM_TypeDef *ptimer)
{
    uint32_t APBPeriph = 0;
    
	switch((uint32_t)ptimer){
    case (uint32_t)TIM1:
        APBPeriph = TIM1_UP_IRQn;
		break;
    case (uint32_t)TIM2:
        APBPeriph = TIM2_IRQn;
		break;
    case (uint32_t)TIM3:
        APBPeriph = TIM3_IRQn;
		break;
    case (uint32_t)TIM4:
        APBPeriph = TIM4_IRQn;
		break;
    }

    return APBPeriph;
}

uint32_t get_timer_id (TIM_TypeDef *ptimer)
{
    uint8_t i;

    for(i=0; i<m_timer_num; i++){
        if(m_timer_map[i].peripheral == (uint32_t)ptimer){
            return i;
        }
    }

    return ID_NULL;
}

    
#if API_TIMER_SUPPORT & BIT(1)
void TIM1_UP_IRQHandler(void)
{
	api_timer_hook(get_timer_id(TIM1));
	TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
}
#endif

#if API_TIMER_SUPPORT & BIT(2)
void TIM2_IRQHandler(void)
{
	api_timer_hook(get_timer_id(TIM2));
    TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
}
#endif

#if API_TIMER_SUPPORT & BIT(3)
void TIM3_IRQHandler(void)
{
	api_timer_hook(get_timer_id(TIM3));
    TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
}
#endif

#if API_TIMER_SUPPORT & BIT(4)
void TIM4_IRQHandler(void)
{
	api_timer_hook(get_timer_id(TIM4));
    TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
}
#endif

/*****************************************************************************************************
**  Function
******************************************************************************************************/





/*******************************************************************
** Parameters:		
** Returns:	
** Description:		
*******************************************************************/
bool hal_timer_init(uint8_t id,uint32_t us)
{
	NVIC_InitTypeDef NVIC_InitStructure = {0};
	TIM_TypeDef *timer = (TIM_TypeDef*)m_timer_map[id].peripheral;
	uint32_t freq = TIMER_FREQ_ATT(id);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};

    timer_periph_en(timer,ENABLE);

	TIM_TimeBaseInitStructure.TIM_Period = 1000000 / freq;	//set pwm freq
	TIM_TimeBaseInitStructure.TIM_Prescaler = 72-1;			//72M  per CK_CNT 1Mhz	
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( timer, &TIM_TimeBaseInitStructure );

	TIM_ITConfig(timer,TIM_IT_Update,ENABLE );

    NVIC_InitStructure.NVIC_IRQChannel = get_timer_irqn(timer);
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = TIMER_PRI_ATT(id);;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init( &NVIC_InitStructure );

    TIM_Cmd( timer, ENABLE );

	return true;
}



bool hal_timer_deinit(uint8_t id)
{
	TIM_TypeDef *timer = (TIM_TypeDef*)m_timer_map[id].peripheral;
	TIM_Cmd( timer, DISABLE );
	TIM_DeInit(timer);
	return false;
}

#endif

