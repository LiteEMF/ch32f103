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


#ifndef _hw_board_h
#define _hw_board_h
#include "utils/emf_typedef.h" 
#include "ch32f10x.h"

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************************************************************
** Defined
*******************************************************************************************************/
#if PROJECT_KM



#elif PROJECT_GAMEPAD
	#if GAMEPAD_C1
		// uart
		#define HW_UART_MAP { \
            {PA_09, (pin_t)PIN_NULL,    0,  0,      (uint32_t)USART1,   0},	\
			{PA_02, PA_03,              0,  0X100,  (uint32_t)USART2,   VAL2FLD(UART_PRI,1)},	\
		}

		//KEY
        #define KEY_A_GPIO 			PC_12
        #define KEY_B_GPIO 			PB_09 
        #define KEY_X_GPIO 			PB_04
        #define KEY_Y_GPIO 			PC_09
        #define KEY_L1_GPIO			PD_02
        #define KEY_R1_GPIO			PA_08
        #define KEY_UP_GPIO			PC_11
        #define KEY_DOWN_GPIO		PA_14
        #define KEY_LEFT_GPIO		PC_10
        #define KEY_RIGHT_GPIO		PA_15
        #define KEY_L3_GPIO			PA_13 
        #define KEY_R3_GPIO 		PC_04 
        #define KEY_SELECT_GPIO 	PB_03
        #define KEY_START_GPIO 		PB_08
        #define KEY_HOME_GPIO 		PB_05
        #define KEY_M1_GPIO			PC_08
        #define KEY_M2_GPIO			PA_05
        #define KEY_M5_GPIO			PC_00 //FN
        #define KEY_M6_GPIO			PC_15 //SCREENSHOT

		//IIC
        #define IMU_ISR_READ_ENABLE		1
        #define IIC_ISR_DMA_ENABLE		1
        #define IIC_ISR_SUPPORT			BIT(2)	/*BIT(2)*/
		#define HW_IIC_MAP {	\
			{PB_10,PB_11,(pin_t)PIN_NULL,(uint32_t)I2C2,VAL2FLD(IIC_BADU,400)},	\
		}


        //SPI
        #define SPI_ISR_SUPPORT		    0	/*BIT(2)*/
        #define HW_SPI_HOST_MAP {	\
			{PB_13,PB_15,(pin_t)PIN_NULL,PB_12,(uint32_t)SPI2,VAL2FLD(SPI_BADU,1000)},	\
		}
 
		#define APP_STICK_ACTIVE {{true, true},{false, false}}
		#define APP_TRIGGER_ACTIVE {false, false}

        #define ADC_LX_ID		0
        #define ADC_LY_ID		1
        #define ADC_RX_ID		2
        #define ADC_RY_ID		3
        #define ADC_L2_ID		4
        #define ADC_R2_ID		5
        #define HW_ADC_MAP      {\
            {PC_01  , 0UL,VAL2FLD(ADC_CHANNEL,ADC_Channel_11)},\
            {PC_02  , 0UL,VAL2FLD(ADC_CHANNEL,ADC_Channel_12)},\
            {(pin_t)PIN_NULL  , 0UL,VAL2FLD(ADC_CHANNEL,ADC_Channel_7)},\
            {(pin_t)PIN_NULL  , 0UL,VAL2FLD(ADC_CHANNEL,ADC_Channel_6)},\
            {PC_03  , 0UL,VAL2FLD(ADC_CHANNEL,ADC_Channel_13)},\
            {PA_04  , 0UL,VAL2FLD(ADC_CHANNEL,ADC_Channel_4)}\
        }

        // #define HW_PWM_MAP {\
        //     {PB_01, (uintptr_t)TIM3, VAL2FLD(PWM_CH,4)|VAL2FLD(PWM_FREQ,1000)|VAL2FLD(PWM_ACTIVE,0)},\
        //     {PB_00, (uintptr_t)TIM3, VAL2FLD(PWM_CH,3)|VAL2FLD(PWM_FREQ,1000)|VAL2FLD(PWM_ACTIVE,0)},\
        //     {PA_00, (uintptr_t)TIM2, VAL2FLD(PWM_CH,1)|VAL2FLD(PWM_FREQ,1000)|VAL2FLD(PWM_ACTIVE,0)},\
        //     {PA_01, (uintptr_t)TIM2, VAL2FLD(PWM_CH,2)|VAL2FLD(PWM_FREQ,1000)|VAL2FLD(PWM_ACTIVE,0)},\
        // }

	#endif
#endif




#ifdef __cplusplus
}
#endif
#endif





