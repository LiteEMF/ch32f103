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
#ifdef HW_PWM_MAP


#include  "api/api_pwm.h"
#include  "api/api_gpio.h"

#include  "api/api_log.h"
/******************************************************************************************************
** Defined
*******************************************************************************************************/
		// #define HW_PWM_MAP {\
        //     {PB_01, (uintptr_t)TIM3, VAL2FLD(PWM_CH,4)|VAL2FLD(PWM_FREQ,1000)|VAL2FLD(PWM_ACTIVE,0)},\
        //     {PB_00, (uintptr_t)TIM3, VAL2FLD(PWM_CH,3)|VAL2FLD(PWM_FREQ,1000)|VAL2FLD(PWM_ACTIVE,0)},\
        //     {PA_00, (uintptr_t)TIM2, VAL2FLD(PWM_CH,1)|VAL2FLD(PWM_FREQ,1000)|VAL2FLD(PWM_ACTIVE,0)},\
        //     {PA_01, (uintptr_t)TIM2, VAL2FLD(PWM_CH,2)|VAL2FLD(PWM_FREQ,1000)|VAL2FLD(PWM_ACTIVE,0)},\
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
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,enable);
		break;
    case (uint32_t)TIM2:
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,enable);
		break;
    case (uint32_t)TIM3:
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,enable);
		break;
    case (uint32_t)TIM4:
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,enable);
		break;
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
bool hal_pwm_set_duty(uint16_t id, uint8_t duty)
{
	TIM_TypeDef *timer = (TIM_TypeDef*)m_pwm_map[id].peripheral;
	uint32_t period  = timer->ATRLR;
    period = period * duty / 255;

	switch(PWM_CH_ATT(id)){
	case 1:
	    timer->CH1CVR = period;
		break;
	case 2:
		timer->CH2CVR = period;
		break;
	case 3:
		timer->CH3CVR = period;
		break;
	case 4:
		timer->CH4CVR = period;
		break;    
	}
	// logd("period=%d, duyt=%d\n",timer->ATRLR,period );
	return true;
}


/*******************************************************************
** Parameters:	PWM_FREQ_ATT: 1hz ~ 93Khz
** Returns:	
** Description:	注意相同TIMER, 最多支持4个通道, 必须设置相同的PWM频率!!!
*******************************************************************/
bool hal_pwm_init(uint16_t id, uint8_t duty)
{
	TIM_TypeDef *timer = (TIM_TypeDef*)m_pwm_map[id].peripheral;
	uint32_t freq = PWM_FREQ_ATT(id);

	if(PWM_CH_ATT(id) > 4) return false;

	if(0 == freq) freq = PWM_FREQ_DEFAULT;

    TIM_OCInitTypeDef TIM_OCInitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};

    timer_periph_en( timer, ENABLE );

	api_gpio_mode(m_pwm_map[id].pin, GPIO_Mode_AF_PP);
	
	if(freq < 300){	
		TIM_TimeBaseInitStructure.TIM_Period = 100000 / freq;				//set pwm freq
		TIM_TimeBaseInitStructure.TIM_Prescaler = 720-1;					//72M  per CK_CNT 100k	
	}else{
		TIM_TimeBaseInitStructure.TIM_Period = 24000000 / freq;				//set pwm freq
		TIM_TimeBaseInitStructure.TIM_Prescaler = 3-1;						//72M  per CK_CNT 24Mhz
	}
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( timer, &TIM_TimeBaseInitStructure );


	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;		//边沿对齐
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = TIM_TimeBaseInitStructure.TIM_Period * duty / 255;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	switch(PWM_CH_ATT(id)){
	case 1:
		TIM_OC1Init( timer, &TIM_OCInitStructure );
		break;
	case 2:
		TIM_OC2Init( timer, &TIM_OCInitStructure );
		break;
	case 3:
		TIM_OC3Init( timer, &TIM_OCInitStructure );
		break;
	case 4:
		TIM_OC4Init( timer, &TIM_OCInitStructure );
		break;    
	}
	
	// logd("pwm ch=%d\n",PWM_CH_ATT(id));
	// logd("psr = %d, period=%d, duyt=%d\n",TIM_TimeBaseInitStructure.TIM_Prescaler,TIM_TimeBaseInitStructure.TIM_Period,TIM_OCInitStructure.TIM_Pulse );
    TIM_CtrlPWMOutputs( timer, ENABLE );
    TIM_OC1PreloadConfig( timer, TIM_OCPreload_Disable );
    TIM_ARRPreloadConfig( timer, ENABLE );
    TIM_Cmd( timer, ENABLE );

	return true;
}
bool hal_pwm_deinit(uint16_t id)
{
	TIM_TypeDef *timer = (TIM_TypeDef*)m_pwm_map[id].peripheral;

	TIM_Cmd( timer, DISABLE );

	return true;
}

#endif





