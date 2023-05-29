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
SPI1 : TX DMA3, RX DMA2
SPI2 : TX DMA5, RX DMA4
************************************************************************************************************/
#include "hw_config.h"
#include "hw_board.h"
#ifdef HW_SPI_HOST_MAP

#include  "api/api_spi_host.h"
#include  "api/api_gpio.h"
#include  "utils/emf_utils.h"

#include  "api/api_log.h"
/******************************************************************************************************
** Defined
*******************************************************************************************************/
        // #define HW_SPI_HOST_MAP {	\
		// 	{PB_13,PB_15,PB_14,(pin_t)PIN_NULL,(uint32_t)SPI1,VAL2FLD(SPI_BADU,1000)},	\
		// }


#define WAIT_CONDITION(x, del)	\
do{\
	uint32_t d = del;\
	while( (x) && --d);\
	if(0 == d){\
        api_spi_host_isr_hook(spi_current_id,ERROR_TIMEOUT);\
        if(0 == (spi_current_id & 0X80)) emf_free(spi_tx_buf);\
        spi_current_id = ID_NULL;\
		return false;\
	}\
}while(0)


#define WAIT_TIME  (2000)
/******************************************************************************************************
**	static Parameters
*******************************************************************************************************/

#if SPI_ISR_SUPPORT
volatile uint8_t spi_current_id = ID_NULL;				//bit7: 0: write, 1:read
volatile uint8_t spi_current_err = 0;
uint8_t *spi_tx_buf = NULL;
#endif

/******************************************************************************************************
**	public Parameters
*******************************************************************************************************/

/*****************************************************************************************************
**	static Function
******************************************************************************************************/
static void spi_periph_en (SPI_TypeDef *pspi, FunctionalState enable)
{
    switch((uint32_t)pspi){
    case (uint32_t)SPI1:
		RCC_APB2PeriphClockCmd( RCC_APB2Periph_SPI1, enable );
		break;
    case (uint32_t)SPI2:
		RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI2, enable );
		break;
    }
}
#if SPI_ISR_SUPPORT
static uint32_t get_spi_irqn (SPI_TypeDef *pspi)
{
    uint32_t APBPeriph = 0;
    
	switch((uint32_t)pspi){
    case (uint32_t)SPI1:
        APBPeriph = SPI1_IRQn;
        break;
    case (uint32_t)SPI2:
        APBPeriph = SPI2_IRQn;
        break;
    }

    return APBPeriph;
}

static uint32_t get_spi_dma_irqn(SPI_TypeDef *pspi, pin_dir_t dir)
{
    uint32_t APBPeriph = 0;
    
	switch((uint32_t)pspi){
    case (uint32_t)SPI1:
		if(PIN_OUT == dir){
			APBPeriph = DMA1_Channel3_IRQn;
		}else{
			APBPeriph = DMA1_Channel2_IRQn;
		}
		break;
    case (uint32_t)SPI2:
		if(PIN_OUT == dir){
			APBPeriph = DMA1_Channel5_IRQn;
		}else{
			APBPeriph = DMA1_Channel4_IRQn;
		}
		break;
    }

    return APBPeriph;
}

static DMA_Channel_TypeDef* get_spi_dma(SPI_TypeDef *pspi, pin_dir_t dir)
{
    DMA_Channel_TypeDef* APBPeriph = 0;
    
	switch((uint32_t)pspi){
    case (uint32_t)SPI1:
		if(PIN_OUT == dir){
			APBPeriph = DMA1_Channel3;
		}else{
			APBPeriph = DMA1_Channel2;
		}
		break;
    case (uint32_t)SPI2:
		if(PIN_OUT == dir){
			APBPeriph = DMA1_Channel5;
		}else{
			APBPeriph = DMA1_Channel4;
		}
		break;
    }

    return APBPeriph;
}


/*********************************************************************
 * @fn      DMA_Tx_Init
 *
 * @brief   Initializes the DMAy Channelx configuration.
 *
 * @param   DMA_CHx - x can be 1 to 7.
 *          ppadr - Peripheral base address.
 *          memadr - Memory base address.
 *          bufsize - DMA channel buffer size.
 *
 * @return  none
 */
static void DMA_Tx_Init( DMA_Channel_TypeDef *DMA_CHx, u32 ppadr, u32 memadr, u16 bufsize )
{
    DMA_InitTypeDef DMA_InitStructure = {0};
	NVIC_InitTypeDef NVIC_InitStructure = {0};
    SPI_TypeDef *spi = (SPI_TypeDef*)m_spi_map[spi_current_id&0x7f].peripheral;

    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_DMA1, ENABLE );

    DMA_DeInit( DMA_CHx );

    DMA_InitStructure.DMA_PeripheralBaseAddr = ppadr;
    DMA_InitStructure.DMA_MemoryBaseAddr = memadr;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = bufsize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init( DMA_CHx, &DMA_InitStructure );

    NVIC_InitStructure.NVIC_IRQChannel = get_spi_dma_irqn(spi,PIN_OUT);
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init( &NVIC_InitStructure );

    DMA_ITConfig( DMA_CHx, DMA_IT_TC | DMA_IT_TE, ENABLE );
}

/*********************************************************************
 * @fn      DMA_Rx_Init
 *
 * @brief   Initializes the SPI1 DMA Channelx configuration.
 *
 * @param   DMA_CHx - x can be 1 to 7.
 *          ppadr - Peripheral base address.
 *          memadr - Memory base address.
 *          bufsize - DMA channel buffer size.
 *
 * @return  none
 */
static void DMA_Rx_Init( DMA_Channel_TypeDef *DMA_CHx, u32 ppadr, u32 memadr, u16 bufsize )
{
    DMA_InitTypeDef DMA_InitStructure = {0};
	NVIC_InitTypeDef NVIC_InitStructure = {0};
    SPI_TypeDef *spi = (SPI_TypeDef*)m_spi_map[spi_current_id&0x7f].peripheral;

    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_DMA1, ENABLE );

    DMA_DeInit( DMA_CHx );

    DMA_InitStructure.DMA_PeripheralBaseAddr = ppadr;
    DMA_InitStructure.DMA_MemoryBaseAddr = memadr;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = bufsize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init( DMA_CHx, &DMA_InitStructure );

	NVIC_InitStructure.NVIC_IRQChannel = get_spi_dma_irqn(spi,PIN_IN);
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init( &NVIC_InitStructure );

    DMA_ITConfig( DMA_CHx, DMA_IT_TC | DMA_IT_TE, ENABLE );
}





#if SPI_ISR_SUPPORT & BIT(1)

void SPI1_IRQHandler(void)
{
    if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_TXE)==SET){
        if(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY )==RESET){  //over
            SPI_I2S_ITConfig(SPI1,SPI_I2S_IT_TXE, DISABLE);         //DISABLE SPI interrupt
            SPI_I2S_ClearITPendingBit( SPI1, SPI_I2S_IT_TXE);

            api_spi_host_isr_hook(spi_current_id,ERROR_SUCCESS);
		    spi_current_id = ID_NULL;
		    spi_current_err = ERROR_SUCCESS;
            emf_free(spi_tx_buf);
        }
    }
}
/*********************************************************************
 * @fn      DMA1_Channel3_IRQHandler
 *
 * @brief   This function handles DMA1 channel6 exception.
 *
 * @return  none
 */
void DMA1_Channel3_IRQHandler()
{
    if( DMA_GetITStatus( DMA1_IT_TC3 ) != RESET ){
        DMA_Cmd( DMA1_Channel3, DISABLE );
        DMA_ClearITPendingBit( DMA1_IT_TC3 );

		SPI_I2S_ITConfig(SPI1,SPI_I2S_IT_TXE, ENABLE);  //ENABLE SPI interrupt
    }

	if( DMA_GetITStatus( DMA1_IT_TE3 ) != RESET ){
        DMA_Cmd( DMA1_Channel3, DISABLE );
        DMA_ClearITPendingBit( DMA1_IT_TE3 );

		api_spi_host_isr_hook(spi_current_id,ERROR_FAILE);
		spi_current_id = ID_NULL;
		spi_current_err = ERROR_FAILE;
    }

    emf_free(spi_tx_buf);
}

/*********************************************************************
 * @fn      DMA1_Channel2_IRQHandler
 *
 * @brief   This function handles DMA1 channel6 exception.
 *
 * @return  none
 */
void DMA1_Channel2_IRQHandler()
{
    if( DMA_GetITStatus( DMA1_IT_TC2 ) != RESET ){
        DMA_Cmd( DMA1_Channel2, DISABLE );
        DMA_ClearITPendingBit( DMA1_IT_TC2 );

		api_spi_host_isr_hook(spi_current_id,ERROR_SUCCESS);
		spi_current_id = ID_NULL;
		spi_current_err = ERROR_SUCCESS;
    }

	if( DMA_GetITStatus( DMA1_IT_TE2 ) != RESET ){
        DMA_Cmd( DMA1_Channel2, DISABLE );
        DMA_ClearITPendingBit( DMA1_IT_TE2 );

		api_spi_host_isr_hook(spi_current_id,ERROR_FAILE);
		spi_current_id = ID_NULL;
		spi_current_err = ERROR_FAILE;
    }

    emf_free(spi_tx_buf);
}
#endif

#if SPI_ISR_SUPPORT & BIT(2)
void SPI2_IRQHandler(void)
{
    if(SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_TXE)==SET){
        if(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY )==RESET){  //over
            SPI_I2S_ITConfig(SPI2,SPI_I2S_IT_TXE, DISABLE);         //DISABLE SPI interrupt
            SPI_I2S_ClearITPendingBit( SPI2, SPI_I2S_IT_TXE);

            api_spi_host_isr_hook(spi_current_id,ERROR_SUCCESS);
		    spi_current_id = ID_NULL;
		    spi_current_err = ERROR_SUCCESS;
            emf_free(spi_tx_buf);
        }
    }
}

void DMA1_Channel5_IRQHandler()
{
    if( DMA_GetITStatus( DMA1_IT_TC5 ) != RESET ){
        DMA_Cmd( DMA1_Channel5, DISABLE );
        DMA_ClearITPendingBit( DMA1_IT_TC5 );

		SPI_I2S_ITConfig(SPI2,SPI_I2S_IT_TXE, ENABLE);  //ENABLE SPI interrupt
    }

	if( DMA_GetITStatus( DMA1_IT_TE5 ) != RESET ){
        DMA_Cmd( DMA1_Channel5, DISABLE );
        DMA_ClearITPendingBit( DMA1_IT_TE5 );

		api_spi_host_isr_hook(spi_current_id,ERROR_FAILE);
		spi_current_id = ID_NULL;
		spi_current_err = ERROR_FAILE;
    }

    emf_free(spi_tx_buf);
}
/*********************************************************************
 * @fn      DMA1_Channel2_IRQHandler
 *
 * @brief   This function handles DMA1 channel6 exception.
 *
 * @return  none
 */
void DMA1_Channel4_IRQHandler()
{
    if( DMA_GetITStatus( DMA1_IT_TC4 ) != RESET ){
        DMA_Cmd( DMA1_Channel4, DISABLE );
        DMA_ClearITPendingBit( DMA1_IT_TC4 );

		api_spi_host_isr_hook(spi_current_id,ERROR_SUCCESS);
		spi_current_id = ID_NULL;
		spi_current_err = ERROR_SUCCESS;
    }

	if( DMA_GetITStatus( DMA1_IT_TE4 ) != RESET ){
        DMA_Cmd( DMA1_Channel4, DISABLE );
        DMA_ClearITPendingBit( DMA1_IT_TE4 );

		api_spi_host_isr_hook(spi_current_id,ERROR_FAILE);
		spi_current_id = ID_NULL;
		spi_current_err = ERROR_FAILE;
    }

    emf_free(spi_tx_buf);
}
#endif
#endif


/*********************************************************************
 * @fn      SPI1_ReadWriteByte
 *
 * @brief   SPI1 read or write one byte.
 *
 * @param   TxData - write one byte data.
 *
 * @return  Read one byte data.
 */
uint8_t SPI1_ReadWriteByte(SPI_TypeDef *pspi, uint8_t TxData )
{
    uint8_t i = 0;

    while( SPI_I2S_GetFlagStatus( pspi, SPI_I2S_FLAG_TXE ) == RESET ){
        i++;
        if( i > 200 ){
            return 0;
        }
    }

    SPI_I2S_SendData( pspi, TxData );
    i = 0;

    while( SPI_I2S_GetFlagStatus( pspi, SPI_I2S_FLAG_RXNE ) == RESET ){
        i++;
        if( i > 200 ){
            return 0;
        }
    }

    return SPI_I2S_ReceiveData( pspi );
}
/*****************************************************************************************************
**  Function
******************************************************************************************************/

/*******************************************************************
** Parameters:		
** Returns:	
** Description:		
*******************************************************************/
bool hal_spi_host_write(uint8_t id,uint16_t addr, uint8_t * buf, uint16_t len)
{
	bool ret = true;

    #if SPI_ISR_SUPPORT
        ret = hal_spi_host_isr_write(id, addr, buf, len);

        WAIT_CONDITION(ID_NULL != spi_current_id, WAIT_TIME * len);
        if( ERROR_SUCCESS != spi_current_err) ret = false;
    #else
        uint16_t i;
        SPI_TypeDef *spi = (SPI_TypeDef*)m_spi_map[id].peripheral;
        if(0XFF00 & addr){
            SPI1_ReadWriteByte(spi, addr>>8 );
        }
        SPI1_ReadWriteByte(spi, addr );
        for(i=0; i<len; i++){
            SPI1_ReadWriteByte(spi, buf[i] );
        }
    #endif

	return ret;
}

bool hal_spi_host_read(uint8_t id,uint16_t addr, uint8_t * buf, uint16_t len)
{
	bool ret = true;

    #if SPI_ISR_SUPPORT
        ret = hal_spi_host_isr_read(id,addr, buf, len);

        WAIT_CONDITION(ID_NULL != spi_current_id, WAIT_TIME * len);
        if( ERROR_SUCCESS != spi_current_err) ret = false;
    #else
        uint16_t i;
        SPI_TypeDef *spi = (SPI_TypeDef*)m_spi_map[id].peripheral;

        if(0XFF00 & addr){
            SPI1_ReadWriteByte(spi, addr>>8 );
        }
        SPI1_ReadWriteByte(spi, addr );

        for(i=0; i<len; i++){
            buf[i] = SPI1_ReadWriteByte(spi, 0xFF );
        }
    #endif

	return ret;
}

bool hal_spi_host_isr_write(uint8_t id,uint16_t addr, uint8_t * buf, uint16_t len)
{
    #if SPI_ISR_SUPPORT
	SPI_TypeDef *spi = (SPI_TypeDef*)m_spi_map[id].peripheral;
    DMA_Channel_TypeDef *dma = get_spi_dma(spi,PIN_OUT);
    uint16_t spi_buf_len;

    spi_buf_len = len + 2;
    if(NULL != spi_tx_buf) emf_free(spi_tx_buf);       //防止内存未释放
    spi_tx_buf = emf_malloc(spi_buf_len);
    if(NULL == spi_tx_buf) return false;

    if(0XFF00 & addr){
        spi_tx_buf[0] = addr>>8;
        spi_tx_buf[1] = addr & 0XFF;
        memcpy(spi_tx_buf+2,buf,len);
    }else{
        spi_tx_buf[0] = addr & 0XFF;
        memcpy(spi_tx_buf+1,buf,len);
        spi_buf_len--;
    }

	DMA_Tx_Init( dma, ( uint32_t )&spi->DATAR, ( uint32_t )spi_tx_buf, spi_buf_len );
	spi_current_id = id;
    DMA_Cmd( dma, ENABLE );
    #endif

	return true;
}
bool hal_spi_host_isr_read(uint8_t id,uint16_t addr, uint8_t * buf, uint16_t len)
{
    #if SPI_ISR_SUPPORT
	SPI_TypeDef *spi = (SPI_TypeDef*)m_spi_map[id].peripheral;
    DMA_Channel_TypeDef *dma = get_spi_dma(spi,PIN_IN);

    //先写地址
    if(0XFF00 & addr){
        SPI1_ReadWriteByte(spi, addr>>8 );
    }
    SPI1_ReadWriteByte(spi, addr );     

	DMA_Rx_Init( dma, ( uint32_t )&spi->DATAR, ( uint32_t )buf, len );
	spi_current_id = 0x80 | id;
    DMA_Cmd( dma, ENABLE );
    #endif

	return true;
}






//注意不是任意波特率都支持
bool hal_spi_host_init(uint8_t id)
{
	SPI_TypeDef *spi = (SPI_TypeDef*)m_spi_map[id].peripheral;
    SPI_InitTypeDef SPI_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};
    RCC_ClocksTypeDef RCC_ClocksStatus;
    uint32_t pclk;
    uint8_t scaler;

    #if SPI_ISR_SUPPORT
    spi_tx_buf = NULL;
    spi_current_id = 0;
    #endif

	spi_periph_en( spi, ENABLE );

	api_gpio_mode(m_spi_map[id].clk, GPIO_Mode_AF_PP);
	api_gpio_mode(m_spi_map[id].mosi, GPIO_Mode_AF_PP);
	api_gpio_mode(m_spi_map[id].miso, GPIO_Mode_IN_FLOATING);

    RCC_GetClocksFreq( &RCC_ClocksStatus );
    if(SPI1 == spi){
        pclk = RCC_ClocksStatus.PCLK2_Frequency;
    }else{
        pclk = RCC_ClocksStatus.PCLK1_Frequency;
    }
    
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;

    scaler = pclk / (SPI_BADU_ATT(id) * 1000);
    if(scaler >= 256){
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
    }else if(scaler >= 128){
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
    }else if(scaler >= 64){
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;  
    }else if(scaler >= 32){
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32; 
    }else if(scaler >= 16){
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; 
    }else if(scaler >= 8){
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; 
    }else if(scaler >= 4){
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; 
    }else{
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; 
    }
    logd("spi sca=%d, pclk=%d, badu=%d\n", scaler, pclk, SPI_BADU_ATT(id));

    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init( spi, &SPI_InitStructure );

    #if SPI_ISR_SUPPORT
	if((pin_t)PIN_NULL != m_spi_map[id].mosi){
        logd("SPI_I2S_DMACmd miso tx en\n");
    	SPI_I2S_DMACmd( spi, SPI_I2S_DMAReq_Tx, ENABLE );
	}
	if((pin_t)PIN_NULL != m_spi_map[id].miso){
   	 	SPI_I2S_DMACmd( spi, SPI_I2S_DMAReq_Rx, ENABLE );
	}

    SPI_I2S_ITConfig(spi,SPI_I2S_IT_TXE, DISABLE);
	NVIC_InitStructure.NVIC_IRQChannel = get_spi_irqn(spi);
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    #endif

    SPI_Cmd( spi, ENABLE );

    UNUSED_VARIABLE(NVIC_InitStructure);
	return true;
}
bool hal_spi_host_deinit(uint8_t id)
{
	SPI_TypeDef *spi = (SPI_TypeDef*)m_spi_map[id].peripheral;
	SPI_Cmd( spi, DISABLE );

	return true;
}

#endif






