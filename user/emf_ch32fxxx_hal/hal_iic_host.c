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

I2C1的TX是通道6，RX是通道7
I2C2的TX是通道4，RX是通道5

************************************************************************************************************/
#include "hw_config.h"
#include "hw_board.h"
#ifdef HW_IIC_MAP

#include  "api/api_iic_host.h"
#include  "api/api_gpio.h"
#include  "utils/emf_utils.h"

#include  "api/api_log.h"
/******************************************************************************************************
** Defined
*******************************************************************************************************/
		// #define HW_IIC_MAP {	\
		// 	{PB_10,PB_11,(pin_t)PIN_NULL,(uint32_t)I2C1,VAL2FLD(IIC_BADU,400)},	\
		// }



#define WAIT_CONDITION(x, del)	\
do{\
	uint32_t d = del;\
	while( (x) && --d);\
	if(0 == d){\
		api_iic_host_isr_hook(iic_current_id,ERROR_TIMEOUT);\
		if(iic_current_id & 0X80) emf_free(iic_tx_buf);\
		iic_current_id = ID_NULL;\
		return false;\
	}\
}while(0)


#define WAIT_TIME  (2000)



/******************************************************************************************************
**	static Parameters
*******************************************************************************************************/
#if IIC_ISR_SUPPORT
volatile uint8_t iic_current_id = ID_NULL;				//bit7: 0: write, 1:read
volatile uint8_t iic_current_err = 0;
uint8_t *iic_tx_buf = NULL;

#if !IIC_ISR_DMA_ENABLE
typedef enum {
    I2C_SEND_ADDRESS_FIRST = 0,
    I2C_CLEAR_ADDRESS_FLAG_FIRST,
    I2C_TRANSMIT_WRITE_READ_ADD,
    I2C_SEND_ADDRESS_SECOND,
    I2C_CLEAR_ADDRESS_FLAG_SECOND,
    I2C_TRANSMIT_DATA,
    I2C_STOP,
} i2c_process_enum;

i2c_process_enum i2c_read_process = I2C_SEND_ADDRESS_FIRST;
i2c_process_enum i2c_write_process = I2C_SEND_ADDRESS_FIRST;

volatile uint8_t 	   *iic_buf = NULL;	
volatile uint16_t      iic_dev_addr;
volatile uint16_t      iic_addr;
volatile uint16_t      i2c_nbytes;
volatile uint8_t       i2c_process_flag = 0;
#endif
#endif


/******************************************************************************************************
**	public Parameters
*******************************************************************************************************/

/*****************************************************************************************************
**	static Function
******************************************************************************************************/
static void iic_periph_en (I2C_TypeDef *piic, FunctionalState enable)
{
	switch((uint32_t)piic){
    case (uint32_t)I2C1:
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,enable);
		break;
    case (uint32_t)I2C2:
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,enable);
		break;
    }
}
#if IIC_ISR_SUPPORT
static uint32_t get_iic_irqn (I2C_TypeDef *piic)
{
    uint32_t APBPeriph = 0;
    
	switch((uint32_t)piic){
    case (uint32_t)I2C1:
        APBPeriph = I2C1_EV_IRQn;
        break;
    case (uint32_t)I2C2:
        APBPeriph = I2C2_EV_IRQn;
        break;
    }

    return APBPeriph;
}

#if IIC_ISR_DMA_ENABLE
static uint32_t get_iic_dma_irqn(I2C_TypeDef *piic, pin_dir_t dir)
{
    uint32_t APBPeriph = 0;
    
	switch((uint32_t)piic){
    case (uint32_t)I2C1:
		if(PIN_OUT == dir){
			APBPeriph = DMA1_Channel6_IRQn;
		}else{
			APBPeriph = DMA1_Channel7_IRQn;
		}
		break;
    case (uint32_t)I2C2:
		if(PIN_OUT == dir){
			APBPeriph = DMA1_Channel4_IRQn;
		}else{
			APBPeriph = DMA1_Channel5_IRQn;
		}
		break;
    }

    return APBPeriph;
}

static DMA_Channel_TypeDef* get_iic_dma(I2C_TypeDef *piic, pin_dir_t dir)
{
    DMA_Channel_TypeDef* APBPeriph = 0;
    
	switch((uint32_t)piic){
    case (uint32_t)I2C1:
		if(PIN_OUT == dir){
			APBPeriph = DMA1_Channel6;
		}else{
			APBPeriph = DMA1_Channel7;
		}
		break;
    case (uint32_t)I2C2:
		if(PIN_OUT == dir){
			APBPeriph = DMA1_Channel4;
		}else{
			APBPeriph = DMA1_Channel5;
		}
		break;
    }

    return APBPeriph;
}


/*********************************************************************
 * @fn      DMA_Tx_Init
 *
 * @brief   Initializes the SPI1 DMAy Channelx configuration.
 *
 * @param   DMA_CHx -
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
	I2C_TypeDef *iic = (I2C_TypeDef*)m_iic_map[iic_current_id&0X7F].peripheral;

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
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init( DMA_CHx, &DMA_InitStructure );

    NVIC_InitStructure.NVIC_IRQChannel = get_iic_dma_irqn(iic,PIN_OUT);
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init( &NVIC_InitStructure );

    DMA_ITConfig( DMA_CHx, DMA_IT_TC | DMA_IT_TE, ENABLE );
}

/*********************************************************************
 * @fn      DMA_Rx_Init
 *
 * @brief   Initializes the SPI1 DMAy Channelx configuration.
 *
 * @param   DMA_CHx -
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
	I2C_TypeDef *iic = (I2C_TypeDef*)m_iic_map[iic_current_id&0X7F].peripheral;

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
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init( DMA_CHx, &DMA_InitStructure );

	NVIC_InitStructure.NVIC_IRQChannel = get_iic_dma_irqn(iic,PIN_IN);
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init( &NVIC_InitStructure );

    DMA_ITConfig( DMA_CHx, DMA_IT_TC | DMA_IT_TE, ENABLE );
}
#endif




void i2c_event_irq_handler(I2C_TypeDef *I2Cx)
{
#if IIC_ISR_DMA_ENABLE
     if(I2C_GetITStatus(I2Cx, I2C_IT_TXE)) {
        /* the master sends a stop condition to I2C bus */
        I2C_GenerateSTOP(I2Cx,ENABLE);

        /* disable the I2CX interrupt */
        I2C_ITConfig(I2Cx, I2C_IT_ERR,DISABLE);
        I2C_ITConfig(I2Cx, I2C_IT_BUF,DISABLE);
        I2C_ITConfig(I2Cx, I2C_IT_EVT,DISABLE);

		api_iic_host_isr_hook(iic_current_id,ERROR_SUCCESS);
		iic_current_id = ID_NULL;
		iic_current_err = ERROR_SUCCESS;
		emf_free(iic_tx_buf);
    }
#else
	if(RESET == i2c_process_flag) {
        switch(i2c_write_process) {
        case I2C_SEND_ADDRESS_FIRST:
            if(I2C_GetITStatus(I2Cx, I2C_IT_SB)) {
                /* send slave address */
                I2C_Send7bitAddress(I2Cx, iic_dev_addr, I2C_Direction_Transmitter);
                i2c_write_process = I2C_CLEAR_ADDRESS_FLAG_FIRST;
            }
            break;
        case I2C_CLEAR_ADDRESS_FLAG_FIRST:
            if(I2C_GetITStatus(I2Cx, I2C_IT_ADDR)) {
                /*clear ADDR bit */
                I2C_GetFlagStatus(I2Cx, I2C_FLAG_ADDR);
                I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY);
                i2c_write_process = I2C_TRANSMIT_WRITE_READ_ADD;
            }
            break;
        case I2C_TRANSMIT_WRITE_READ_ADD:
            if(I2C_GetITStatus(I2Cx, I2C_IT_TXE)) {
                I2C_SendData(I2Cx, iic_addr);
                /* wait until BTF bit is set */
				while(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF));
                i2c_write_process = I2C_TRANSMIT_DATA;
            }
            break;
        case I2C_TRANSMIT_DATA:
            if(I2C_GetITStatus(I2Cx, I2C_IT_TXE)) {
                /* the master sends a data byte */
                I2C_SendData(I2Cx, *iic_buf++);
                i2c_nbytes--;
                if(RESET == i2c_nbytes) {
                    i2c_write_process = I2C_STOP;
                }
            }
            break;
        case I2C_STOP:
            /* the master sends a stop condition to I2C bus */
            I2C_GenerateSTOP(I2Cx,ENABLE);
            /* disable the I2Cx interrupt */
            I2C_ITConfig(I2Cx, I2C_IT_ERR,DISABLE);
            I2C_ITConfig(I2Cx, I2C_IT_BUF,DISABLE);
            I2C_ITConfig(I2Cx, I2C_IT_EVT,DISABLE);
            i2c_write_process = I2C_SEND_ADDRESS_FIRST;

			api_iic_host_isr_hook(iic_current_id,ERROR_SUCCESS);
			iic_current_id = ID_NULL;
			iic_current_err = ERROR_SUCCESS;
			emf_free((void*)iic_tx_buf);
            break;
        default:
            break;
        }
    } else if(SET == i2c_process_flag) {
        switch(i2c_read_process) {
        case I2C_SEND_ADDRESS_FIRST:
            // logd("STAR1=%x,CTLR1=%x)",I2Cx->STAR1,I2Cx->CTLR1);
            if(I2C_GetITStatus(I2Cx, I2C_IT_SB)) {
                /* send slave address */
                I2C_Send7bitAddress(I2Cx, iic_dev_addr, I2C_Direction_Transmitter);
                i2c_read_process = I2C_CLEAR_ADDRESS_FLAG_FIRST;
            }
            break;
        case I2C_CLEAR_ADDRESS_FLAG_FIRST:
            if(I2C_GetITStatus(I2Cx, I2C_IT_ADDR)) {
                /*clear ADDR bit */
                I2C_GetFlagStatus(I2Cx, I2C_FLAG_ADDR);
                I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY);
                i2c_read_process = I2C_TRANSMIT_WRITE_READ_ADD;
            }
            break;
        case I2C_TRANSMIT_WRITE_READ_ADD:
            if(I2C_GetITStatus(I2Cx, I2C_IT_TXE)) {
                I2C_SendData(I2Cx, iic_addr);
                /* wait until BTF bit is set */
				while(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF));
                /* send a start condition to I2C bus */
                I2C_GenerateSTART(I2Cx,ENABLE);
                i2c_read_process = I2C_SEND_ADDRESS_SECOND;
            }
            break;
        case I2C_SEND_ADDRESS_SECOND:
            if(I2C_GetITStatus(I2Cx, I2C_IT_SB)) {
                I2C_Send7bitAddress(I2Cx, iic_dev_addr, I2C_Direction_Receiver);
				if(1 == i2c_nbytes) {
					/* disable acknowledge */
                    I2C_AcknowledgeConfig(I2Cx,DISABLE);
				}else if(2 == i2c_nbytes) {
					I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Next);
				}
				
                i2c_read_process = I2C_CLEAR_ADDRESS_FLAG_SECOND;
            }
            break;
        case I2C_CLEAR_ADDRESS_FLAG_SECOND:
            if(I2C_GetITStatus(I2Cx, I2C_IT_ADDR)) {
                /*clear ADDR bit */
                I2C_GetFlagStatus(I2Cx, I2C_FLAG_ADDR);
                I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY);
                if((1 == i2c_nbytes) || (2 == i2c_nbytes)) {
                    /* clear the ACKEN before the ADDR is cleared */
                    I2C_AcknowledgeConfig(I2Cx,DISABLE);
                }
                i2c_read_process = I2C_TRANSMIT_DATA;
            }
            break;
        case I2C_TRANSMIT_DATA:
            if(I2C_GetITStatus(I2Cx, I2C_IT_RXNE)) {
                if(i2c_nbytes > 0) {
                    if( i2c_nbytes == 3 ) {
                        /* wait until BTF bit is set */
                        while(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF));
                        /* disable acknowledge */
                        I2C_AcknowledgeConfig(I2Cx,DISABLE);
                    }
                    /* read a byte from the EEPROM */
                    *iic_buf = I2C_ReceiveData(I2Cx);
                    iic_buf++;
                    i2c_nbytes--;
                    if(i2c_nbytes == 0) {
                        i2c_read_process = I2C_SEND_ADDRESS_FIRST;
                        /* the master sends a stop condition to I2C bus */
                        I2C_GenerateSTOP(I2Cx,ENABLE);
                        /* disable the I2CX interrupt */
                        I2C_ITConfig(I2Cx, I2C_IT_ERR,DISABLE);
                        I2C_ITConfig(I2Cx, I2C_IT_BUF,DISABLE);
                        I2C_ITConfig(I2Cx, I2C_IT_EVT,DISABLE);
                        i2c_process_flag = RESET;

						api_iic_host_isr_hook(iic_current_id,ERROR_SUCCESS);
						iic_current_id = ID_NULL;
						iic_current_err = ERROR_SUCCESS;
                    }
                }
            }
            break;
        default:
            break;
        }
    }
#endif
}

void i2c_error_irq_handler(I2C_TypeDef *I2Cx)
{
    /* no acknowledge received */
    if(I2C_GetITStatus(I2Cx, I2C_IT_AF)) {
        I2C_ClearITPendingBit(I2Cx, I2C_IT_AF);
    }

    /* SMBus alert */
    if(I2C_GetITStatus(I2Cx, I2C_IT_SMBALERT)) {
        I2C_ClearITPendingBit(I2Cx, I2C_IT_SMBALERT);
    }

    /* bus timeout in SMBus mode */
    if(I2C_GetITStatus(I2Cx, I2C_IT_TIMEOUT)) {
        I2C_ClearITPendingBit(I2Cx, I2C_IT_TIMEOUT);
    }

    /* over-run or under-run when SCL stretch is disabled */
    if(I2C_GetITStatus(I2Cx, I2C_IT_OVR)) {
        I2C_ClearITPendingBit(I2Cx, I2C_IT_OVR);
    }

    /* arbitration lost */
    if(I2C_GetITStatus(I2Cx, I2C_IT_ARLO)) {
        I2C_ClearITPendingBit(I2Cx, I2C_IT_ARLO);
    }

    /* bus error */
    if(I2C_GetITStatus(I2Cx, I2C_IT_BERR)) {
        I2C_ClearITPendingBit(I2Cx, I2C_IT_BERR);
    }

    /* CRC value doesn't match */
    if(I2C_GetITStatus(I2Cx, I2C_IT_PECERR)) {
        I2C_ClearITPendingBit(I2Cx, I2C_IT_PECERR);
    }

    I2C_GenerateSTOP(I2Cx,ENABLE);
    /* disable the error interrupt */
    I2C_ITConfig(I2Cx, I2C_IT_ERR,DISABLE);
    I2C_ITConfig(I2Cx, I2C_IT_BUF,DISABLE);
    I2C_ITConfig(I2Cx, I2C_IT_EVT,DISABLE);

	api_iic_host_isr_hook(iic_current_id,ERROR_FAILE);
	iic_current_id = ID_NULL;
	iic_current_err = ERROR_SUCCESS;
	emf_free((void*)iic_tx_buf);
	logd("irqerr\n");
}

#if IIC_ISR_SUPPORT & BIT(1)
void I2C1_EV_IRQHandler(void)
{
    i2c_event_irq_handler(I2C1);
}


void I2C1_ER_IRQHandler(void)
{
    i2c_error_irq_handler(I2C1);
}
#if IIC_ISR_DMA_ENABLE
/*********************************************************************
 * @fn      DMA1_Channel6_IRQHandler
 *
 * @brief   This function handles DMA1 channel6 exception.
 *
 * @return  none
 */
void DMA1_Channel6_IRQHandler()
{
    if( DMA_GetITStatus( DMA1_IT_TC6 ) != RESET ){
        DMA_Cmd( DMA1_Channel6, DISABLE );
        DMA_ClearITPendingBit( DMA1_IT_TC6 );

		I2C_ITConfig(I2C1, I2C_IT_ERR,ENABLE);
        I2C_ITConfig(I2C1, I2C_IT_EVT,ENABLE);
    }

	if( DMA_GetITStatus( DMA1_IT_TE6 ) != RESET ){
        DMA_Cmd( DMA1_Channel6, DISABLE );
        DMA_ClearITPendingBit( DMA1_IT_TE6 );
		I2C_GenerateSTOP( I2C1, ENABLE );

		api_iic_host_isr_hook(iic_current_id,ERROR_FAILE);
		iic_current_id = ID_NULL;
		iic_current_err = ERROR_FAILE;
    }
}

void DMA1_Channel7_IRQHandler()
{
    if( DMA_GetITStatus( DMA1_IT_TC7 ) != RESET ){
        DMA_Cmd( DMA1_Channel7, DISABLE );
        DMA_ClearITPendingBit( DMA1_IT_TC7 );
		I2C_GenerateSTOP( I2C1, ENABLE );

		api_iic_host_isr_hook(iic_current_id,ERROR_SUCCESS);
		iic_current_id = ID_NULL;
		iic_current_err = ERROR_SUCCESS;
    }

	if( DMA_GetITStatus( DMA1_IT_TE7 ) != RESET ){
        DMA_Cmd( DMA1_Channel7, DISABLE );
        DMA_ClearITPendingBit( DMA1_IT_TE7 );
		I2C_GenerateSTOP( I2C1, ENABLE );

		api_iic_host_isr_hook(iic_current_id,ERROR_FAILE);
		iic_current_id = ID_NULL;
		iic_current_err = ERROR_FAILE;
    }
}
#endif
#endif


#if IIC_ISR_SUPPORT & BIT(2)
//void I2C2_EV_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void I2C2_ER_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void I2C2_EV_IRQHandler(void)
{
    i2c_event_irq_handler(I2C2);
}

void I2C2_ER_IRQHandler(void)
{
    i2c_error_irq_handler(I2C2);
}

#if IIC_ISR_DMA_ENABLE
/*********************************************************************
 * @fn      DMA1_Channel6_IRQHandler
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

		I2C_ITConfig(I2C2, I2C_IT_ERR,ENABLE);
        I2C_ITConfig(I2C2, I2C_IT_EVT,ENABLE);;
    }

	if( DMA_GetITStatus( DMA1_IT_TE4 ) != RESET ){
        DMA_Cmd( DMA1_Channel4, DISABLE );
        DMA_ClearITPendingBit( DMA1_IT_TE4 );
		
		I2C_GenerateSTOP( I2C2, ENABLE );
		api_iic_host_isr_hook(iic_current_id,ERROR_FAILE);
		iic_current_id = ID_NULL;
		iic_current_err = ERROR_FAILE;
		logd("dma4e\n");
    }
}


void DMA1_Channel5_IRQHandler()
{
    if( DMA_GetITStatus( DMA1_IT_TC5 ) != RESET ){
        DMA_Cmd( DMA1_Channel5, DISABLE );
        DMA_ClearITPendingBit( DMA1_IT_TC5 );
		I2C_GenerateSTOP( I2C2, ENABLE );

		api_iic_host_isr_hook(iic_current_id,ERROR_SUCCESS);
		iic_current_id = ID_NULL;
		iic_current_err = ERROR_SUCCESS;
    }

	if( DMA_GetITStatus( DMA1_IT_TE5 ) != RESET ){
        DMA_Cmd( DMA1_Channel5, DISABLE );
        DMA_ClearITPendingBit( DMA1_IT_TE5 );
		I2C_GenerateSTOP( I2C2, ENABLE );

		api_iic_host_isr_hook(iic_current_id,ERROR_FAILE);
		iic_current_id = ID_NULL;
		iic_current_err = ERROR_FAILE;
		logd("dma5e\n");
    }
}
#endif
#endif

#endif

/*****************************************************************************************************
**  Function
******************************************************************************************************/


/*******************************************************************
** Parameters:		
** Returns:	
** Description:		
*******************************************************************/
bool hal_iic_write(uint8_t id,uint8_t dev_addr,uint16_t addr, uint8_t const *buf, uint16_t len)
{
	bool ret = true;

	#if IIC_ISR_SUPPORT
		uint32_t d = WAIT_TIME*len;

		ret = hal_iic_isr_write(id, dev_addr, addr, buf, len);

		#if IIC_ISR_DMA_ENABLE
		while( (ID_NULL != iic_current_id) && --d);
		#else
		while( (i2c_nbytes > 0) && --d);
		#endif
		if( ERROR_SUCCESS != iic_current_err) ret = false;

		if(0 == d){
			api_iic_host_isr_hook(iic_current_id,ERROR_TIMEOUT);
			emf_free(iic_tx_buf);
			iic_current_id = ID_NULL;
			ret = false;
		}
	#endif

	return ret;
}

bool hal_iic_read(uint8_t id,uint8_t dev_addr,uint16_t addr, uint8_t* buf, uint16_t len)
{
	bool ret = true;
	#if IIC_ISR_SUPPORT
		ret = hal_iic_isr_read(id, dev_addr, addr, buf, len);

		#if IIC_ISR_DMA_ENABLE
		WAIT_CONDITION(ID_NULL != iic_current_id, WAIT_TIME * len);
		if( ERROR_SUCCESS != iic_current_err) ret = false;
		#else
		WAIT_CONDITION((i2c_nbytes > 0), WAIT_TIME * len);
		#endif
	#endif
	return ret;
}




/*******************************************************************
** Parameters:
** Returns:	
** Description:	设备和寄存器地址使用while等待,这部分后面可以考虑结合中断方式来实现	
*******************************************************************/
bool hal_iic_isr_write(uint8_t id,uint8_t dev_addr,uint16_t addr, uint8_t const *buf, uint16_t len)
{	
	I2C_TypeDef *iic = (I2C_TypeDef*)m_iic_map[id].peripheral;
	#if IIC_ISR_SUPPORT
		#if IIC_ISR_DMA_ENABLE
		DMA_Channel_TypeDef *dma = get_iic_dma(iic,PIN_OUT);
		iic_current_id = id;
		WAIT_CONDITION(I2C_GetFlagStatus( iic, I2C_FLAG_BUSY ) != RESET, WAIT_TIME);

		I2C_GenerateSTART( iic, ENABLE );
		WAIT_CONDITION( !I2C_CheckEvent( iic, I2C_EVENT_MASTER_MODE_SELECT ),  WAIT_TIME);

		I2C_Send7bitAddress( iic, dev_addr, I2C_Direction_Transmitter );
		WAIT_CONDITION( !I2C_CheckEvent( iic, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ), WAIT_TIME );

		if(addr>>8){
			I2C_SendData( iic, (uint8_t)(addr>>8) );   
			WAIT_CONDITION( !I2C_CheckEvent( iic, I2C_EVENT_MASTER_BYTE_TRANSMITTED ), WAIT_TIME );
		}
		I2C_SendData( iic, (uint8_t)(addr&0xFF) );  
		WAIT_CONDITION( !I2C_CheckEvent( iic, I2C_EVENT_MASTER_BYTE_TRANSMITTED ), WAIT_TIME );

		if(iic_tx_buf) emf_free(iic_tx_buf); 	//防止内存出错
		iic_tx_buf = emf_malloc(len);
		if(NULL == iic_tx_buf) return false;

		memcpy(iic_tx_buf, buf,len);
		DMA_Tx_Init( dma, ( uint32_t )&iic->DATAR, ( uint32_t )iic_tx_buf, len );
		DMA_Cmd( dma, ENABLE );
		#else
		iic_current_id = id;
		iic_dev_addr = dev_addr;
		iic_addr = addr;

		if(iic_tx_buf) emf_free((void*)iic_tx_buf); 	//防止内存出错
		iic_tx_buf = emf_malloc(len);
		if(NULL == iic_tx_buf) return false;
		memcpy(iic_tx_buf, buf,len);
		iic_buf = iic_tx_buf;
		i2c_nbytes = len;
		
		i2c_process_flag = 0;
		/* enable the I2C0 interrupt */
		I2C_ITConfig(iic, I2C_IT_ERR,ENABLE);
		I2C_ITConfig(iic, I2C_IT_BUF,ENABLE);
		I2C_ITConfig(iic, I2C_IT_EVT,ENABLE);
		/* the master waits until the I2C bus is idle */
		WAIT_CONDITION(I2C_GetFlagStatus(iic, I2C_FLAG_BUSY),WAIT_TIME);

		/* the master sends a start condition to I2C bus */
		I2C_GenerateSTART(iic,ENABLE);
		#endif
	#endif
	
	UNUSED_VARIABLE(iic);
	return true;
} 

/*******************************************************************
** Parameters:	buf: 必须是全局变量	
** Returns:	
** Description:	设备和寄存器地址使用while等待, 这部分后面可以考虑结合中断方式来实现
*******************************************************************/
bool hal_iic_isr_read(uint8_t id,uint8_t dev_addr,uint16_t addr, uint8_t* buf, uint16_t len)
{
	I2C_TypeDef *iic = (I2C_TypeDef*)m_iic_map[id].peripheral;
	#if IIC_ISR_SUPPORT
		#if IIC_ISR_DMA_ENABLE
		DMA_Channel_TypeDef *dma = get_iic_dma(iic,PIN_IN);

		iic_current_id = 0X80 | id;
		WAIT_CONDITION( I2C_GetFlagStatus( iic, I2C_FLAG_BUSY ) != RESET , WAIT_TIME);

		I2C_GenerateSTART( iic, ENABLE );
		WAIT_CONDITION( !I2C_CheckEvent( iic, I2C_EVENT_MASTER_MODE_SELECT ), WAIT_TIME );

		I2C_Send7bitAddress( iic, dev_addr, I2C_Direction_Transmitter );
		WAIT_CONDITION( !I2C_CheckEvent( iic, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ), WAIT_TIME );

		if(addr>>8){
			I2C_SendData( iic, (uint8_t)(addr>>8) );   
			WAIT_CONDITION( !I2C_CheckEvent( iic, I2C_EVENT_MASTER_BYTE_TRANSMITTED ), WAIT_TIME );
		}
		I2C_SendData( iic, (uint8_t)(addr&0xFF) );  
		WAIT_CONDITION( !I2C_CheckEvent( iic, I2C_EVENT_MASTER_BYTE_TRANSMITTED ), WAIT_TIME );
			
		I2C_GenerateSTART( iic, ENABLE );	
		WAIT_CONDITION( !I2C_CheckEvent( iic, I2C_EVENT_MASTER_MODE_SELECT ), WAIT_TIME );

		I2C_Send7bitAddress( iic, dev_addr, I2C_Direction_Receiver );
		WAIT_CONDITION( !I2C_CheckEvent( iic, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED ), WAIT_TIME );

		DMA_Rx_Init( dma, ( uint32_t )&iic->DATAR, ( uint32_t )buf, len );
		DMA_Cmd( dma, ENABLE );
		#else

		iic_current_id = 0X80 | id;
		iic_dev_addr = dev_addr;
		iic_addr = addr;
		iic_buf = buf;
		i2c_nbytes = len;

		i2c_process_flag = SET;
		/* enable the I2C interrupt */
		I2C_ITConfig(iic, I2C_IT_ERR,ENABLE);
		I2C_ITConfig(iic, I2C_IT_BUF,ENABLE);
		I2C_ITConfig(iic, I2C_IT_EVT,ENABLE);
		I2C_AcknowledgeConfig(iic,ENABLE);
        I2C_NACKPositionConfig(iic, I2C_NACKPosition_Current);

		/* wait until I2C bus is idle */
		WAIT_CONDITION(I2C_GetFlagStatus(iic, I2C_FLAG_BUSY), WAIT_TIME );

		/* send a start condition to I2C bus */
		I2C_GenerateSTART(iic,ENABLE);
		#endif
	#endif
		
	UNUSED_VARIABLE(iic);
	return true;
}

bool hal_iic_init(uint8_t id)
{	
	I2C_InitTypeDef I2C_InitTSturcture = {0};
	NVIC_InitTypeDef NVIC_InitStructure = {0};
	I2C_TypeDef *iic = (I2C_TypeDef*)m_iic_map[id].peripheral;

	api_gpio_mode(m_iic_map[id].clk, GPIO_Mode_AF_OD);
	api_gpio_mode(m_iic_map[id].sda, GPIO_Mode_AF_OD);

    iic_periph_en( iic, ENABLE );

    I2C_InitTSturcture.I2C_ClockSpeed = IIC_BADU_ATT(id)*1000;
    I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
    I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_16_9;
    I2C_InitTSturcture.I2C_OwnAddress1 = 0;
    I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
    I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init( iic, &I2C_InitTSturcture );

	#if IIC_ISR_SUPPORT
	iic_current_id = 0;
	iic_tx_buf = NULL;

	#if IIC_ISR_DMA_ENABLE
    I2C_DMACmd( iic, ENABLE );
	I2C_DMALastTransferCmd( iic, ENABLE );
	#endif

	/* disable the error interrupt */
    I2C_ITConfig(iic, I2C_IT_ERR,DISABLE);
    I2C_ITConfig(iic, I2C_IT_BUF,DISABLE);
    I2C_ITConfig(iic, I2C_IT_EVT,DISABLE);

	NVIC_InitStructure.NVIC_IRQChannel = get_iic_irqn(iic);
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = get_iic_irqn(iic)+1;	//I2Cn_ER_IRQn
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	#endif

    I2C_Cmd( iic, ENABLE );
    I2C_AcknowledgeConfig( iic, ENABLE );

	UNUSED_VARIABLE(NVIC_InitStructure);
	return true;
}
bool hal_iic_deinit(uint8_t id)
{
	I2C_TypeDef *iic = (I2C_TypeDef*)m_iic_map[id].peripheral;

	I2C_Cmd( iic, DISABLE );
	return true;
}


#endif





