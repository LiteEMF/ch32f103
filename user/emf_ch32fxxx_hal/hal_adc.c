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
#ifdef HW_ADC_MAP

#include  "api/api_adc.h"
#include  "api/api_gpio.h"
#include  "utils/emf_utils.h"
/******************************************************************************************************
** Defined
*******************************************************************************************************/
		// #define ADC_LX_ID		0
        // #define ADC_LY_ID		1
        // #define ADC_RX_ID		2
        // #define ADC_RY_ID		3
        // #define ADC_L2_ID		4
        // #define ADC_R2_ID		5
        // #define HW_ADC_MAP      {\
        //     {PC_01  , 0UL,VAL2FLD(ADC_CH,ADC_Channel_11)},\
        //     {PC_02  , 0UL,VAL2FLD(ADC_CH,ADC_Channel_12)},\
        //     {PA_07  , 0UL,VAL2FLD(ADC_CH,ADC_Channel_7)},\
        //     {PA_06  , 0UL,VAL2FLD(ADC_CH,ADC_Channel_6)},\
        //     {PC_03  , 0UL,VAL2FLD(ADC_CH,ADC_Channel_13)},\
        //     {PA_04  , 0UL,VAL2FLD(ADC_CH,ADC_Channel_4)}\
        // }
/******************************************************************************************************
**	static Parameters
*******************************************************************************************************/
/* Global Variable */
int16_t Calibrattion_Val = 0;
uint16_t *dma_adc_bufp = NULL;
/******************************************************************************************************
**	public Parameters
*******************************************************************************************************/

/*****************************************************************************************************
**	static Function
******************************************************************************************************/

static void DMA_Rx_Init( DMA_Channel_TypeDef* DMA_CHx)
{
	DMA_InitTypeDef DMA_InitStructure={0};

	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_DMA1, ENABLE );
	
	DMA_DeInit(DMA_CHx);	
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->RDATAR;;	
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)dma_adc_bufp;	
    DMA_InitStructure.DMA_BufferSize = m_adc_num; 
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;	
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;	
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;	
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;	
	DMA_Init( DMA_CHx, &DMA_InitStructure );	
}

/*********************************************************************
 * @fn      Get_ConversionVal
 *
 * @brief   Get Conversion Value.
 *
 * @param   val - Sampling value
 *
 * @return  val+Calibrattion_Val - Conversion Value.
 **********************************************************************/
u16 Get_ConversionVal( int16_t val )
{
    if( ( val + Calibrattion_Val ) < 0 ){
        return 0;
    }else if((Calibrattion_Val + val) > 4095 || val==4095){
        return 4095;
    }
    return ( val + Calibrattion_Val );
}
/*****************************************************************************************************
**  Function
******************************************************************************************************/

/*******************************************************************
** Parameters:		
** Returns:	
** Description:		
*******************************************************************/
uint16_t hal_adc_to_voltage(uint16_t adc)
{	
	uint16_t adj_adc;

	adj_adc = Get_ConversionVal(adc);
	return adj_adc * ADC_REF_MV / ADC_RES_MAX;
}

bool hal_adc_value(uint8_t id, uint16_t* valp)
{
	*valp = dma_adc_bufp[id];
	return true;
}

bool hal_adc_start_scan(void)
{
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		
	return true;
}

bool hal_adc_init(void)
{
	uint8_t id;

	ADC_InitTypeDef ADC_InitStructure={0}; 

	if(NULL == dma_adc_bufp){					//only malloc by init
		dma_adc_bufp = emf_malloc(m_adc_num*sizeof(uint16_t));
	}

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE );	 
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);	  		//72M / 8 = 9M
                    
	ADC_DeInit(ADC1);  
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;	
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;		
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	
	ADC_InitStructure.ADC_NbrOfChannel = m_adc_num;	

	for(id=0; id < m_adc_num; id++){
		api_gpio_mode(m_adc_map[id].pin, GPIO_Mode_AIN);
		//相当于采样一个通道约等于27us 
		ADC_RegularChannelConfig(ADC1,ADC_CH_ATT(id) ,id+1, ADC_SampleTime_239Cycles5);  	//设置规则通道 
	}

	ADC_Init(ADC1, &ADC_InitStructure);	 

	ADC_DMACmd(ADC1, ENABLE);  
	ADC_Cmd(ADC1, ENABLE);

	ADC_ResetCalibration(ADC1);	
	while(ADC_GetResetCalibrationStatus(ADC1));		
	ADC_StartCalibration(ADC1);	 
	while(ADC_GetCalibrationStatus(ADC1));
	Calibrattion_Val = Get_CalibrationValue(ADC1);

	DMA_Rx_Init( DMA1_Channel1);  
	DMA_Cmd( DMA1_Channel1, ENABLE );	

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	return true;
}
bool hal_adc_deinit(void)
{
	DMA_Cmd( DMA1_Channel1, DISABLE );
	ADC_Cmd(ADC1, DISABLE);
	ADC_DMACmd(ADC1, DISABLE);  

	ADC_DeInit(ADC1);  
	DMA_DeInit(DMA1_Channel1);

	return true;
}

#endif

