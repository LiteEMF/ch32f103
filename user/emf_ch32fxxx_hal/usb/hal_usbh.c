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
#if (API_USBH_BIT_ENABLE)   
#include  "api/usb/usb_typedef.h"
#include  "api/usb/host/usbh.h"
#include  "api/api_tick.h"
#include  "api/api_system.h"

#include "api/api_log.h"

/******************************************************************************************************
** Defined
*******************************************************************************************************/
#define USBH_ID 	(0<<4)

    
/* USB Communication Status Code */
#define ERR_SUCCESS                 0x00
#define ERR_USB_CONNECT             0x15
#define ERR_USB_DISCON              0x16
#define ERR_USB_BUF_OVER            0x17
#define ERR_USB_DISK_ERR            0x1F
#define ERR_USB_TRANSFER            0x20
#define ERR_USB_UNSUPPORT           0xFB
#define ERR_USB_UNAVAILABLE         0xFC
#define ERR_USB_UNKNOWN             0xFE
                
/* USB Communication Time */
#define DEF_BUS_RESET_TIME          11          // USB bus reset time
#define DEF_RE_ATTACH_TIMEOUT       100         // Wait for the USB device to reconnect after reset, 100mS timeout
#define DEF_WAIT_USB_TOUT_200US     1000
#define DEF_CTRL_TRANS_TIMEOVER_CNT 60000       // Control transmission delay timing


/******************************************************************************************************
**	public Parameters
*******************************************************************************************************/


/******************************************************************************************************
**	static Parameters
*******************************************************************************************************/
#if !defined(__C51__)
__ALIGN(4) uint8x_t  USBHD_RX_Buf[ 0x40];  // IN, must even address
__ALIGN(4) uint8x_t  USBHD_TX_Buf[ 0x40 ];  // OUT, must even address
#else
uint8x_t  USBHD_RX_Buf[ 0x40] _at_ 0x0000 ;  // IN, must even address, 错开usbd buf
uint8x_t  USBHD_TX_Buf[ 0x40 ] _at_ (0x0040) ;  // OUT, must even address
#endif

/*****************************************************************************************************
**	static Function
******************************************************************************************************/

/*********************************************************************
 * @fn      USBHD_RCC_Init
 *
 * @brief   Set USB clock.
 *          Note: If the SystemCoreClock is selected as the USB clock source, 
 *          only the frequency specified below can be used.
 *
 * @return  none
 */
void USBHD_RCC_Init( void )
{
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE );
    EXTEN->EXTEN_CTR |= EXTEN_USBHD_IO_EN;
    
    if( SystemCoreClock == 72000000 )
    {
        RCC_USBCLKConfig( RCC_USBCLKSource_PLLCLK_1Div5 );
    }        
    else if( SystemCoreClock == 48000000 )
    {
        RCC_USBCLKConfig( RCC_USBCLKSource_PLLCLK_Div1 );
    }
    
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_USBHD, ENABLE );
}

/*********************************************************************
 * @fn      USBHD_Host_Init
 *
 * @brief   Initialize USB host.
 *
 * @return  none
 */
void USBHD_Host_Init( FunctionalState sta )
{
    if( sta )
    {
        /* Reset USB module */
        R8_USB_CTRL = RB_UC_RESET_SIE | RB_UC_CLR_ALL;
        delay_us( 10 );
        R8_USB_CTRL = 0;
        
        /* Initialize USB host configuration */
        R8_USB_CTRL = RB_UC_HOST_MODE | RB_UC_INT_BUSY | RB_UC_DMA_EN;
        R8_UH_EP_MOD = RB_UH_EP_TX_EN | RB_UH_EP_RX_EN;
        R16_UH_RX_DMA = (uint16_t)(uint32_t)USBHD_RX_Buf;
        R16_UH_TX_DMA = (uint16_t)(uint32_t)USBHD_TX_Buf;
    }
    else
    {
        R8_USB_CTRL = RB_UC_RESET_SIE | RB_UC_CLR_ALL;
        delay_us( 10 );
        R8_USB_CTRL = 0;
    }
}
/*********************************************************************
 * @fn      USBH_Detect
 *
 * @brief   Detecting equipment plugging and unplugging On Fs Port
 *
 * @return  none
 */
void USBHDH_CheckRootHubPortStatus( void )
{
    if( UIF_DETECT ){ 				// Check that there is a device connection or disconnection event on the port
        USB_INT_FG = RB_UIF_DETECT; 			// Clear flag

        if( USB_MIS_ST & RB_UMS_DEV_ATTACH ){ 	// Check that there is a device connection to the port
			usbh_det_event(USBH_ID, 1);
        } else{ 								// Check that there is no device connection to the port
            usbh_det_event(USBH_ID, 0);
		}
    }
}
/*********************************************************************
 * @fn      USBHDH_CheckPortStatus
 *
 * @brief   Check the enable status of the USB port.
 *          Note: This bit is automatically cleared when the device is disconnected.  
 *
 * @return  The current enable status of the port.
 */
uint8_t USBHDH_CheckRootHubPortEnable( void )
{
    return ( R8_UHOST_CTRL & RB_UH_PORT_EN );
}
/*********************************************************************
 * @fn      USBHDH_CheckDevSpeed
 *
 * @brief   Check the speed of the USB port.
 *
 * @return  Low speed or full speed
 */
uint8_t USBHDH_CheckRootHubPortSpeed( void )
{
    uint8_t speed;

    speed = ( R8_USB_MIS_ST & RB_UMS_DM_LEVEL? USB_SPEED_LOW : USB_SPEED_FULL );

    return speed;
}

/*********************************************************************
 * @fn      USBH_SetSelfAddr
 *
 * @brief   Set the USB device address.
 *
 * @para    addr: USB device address.
 *
 * @return  none
 */
void USBHDH_SetSelfAddr( uint8_t addr )
{
    R8_USB_DEV_AD = ( R8_USB_DEV_AD & RB_UDA_GP_BIT ) | ( addr & MASK_USB_ADDR );
}

/*********************************************************************
 * @fn      USBH_SetSelfSpeed
 *
 * @brief   Set USB speed.
 *
 * @para    speed: USB speed.
 *
 * @return  none
 */
void USBHDH_SetSelfSpeed(usb_speed_t speed)
{
    if (TUSB_SPEED_FULL == speed) {       //full speed
		R8_USB_CTRL &= ~RB_UC_LOW_SPEED;
		R8_UHOST_CTRL &= ~RB_UH_LOW_SPEED;
		R8_UH_SETUP &= ~RB_UH_PRE_PID_EN; 
    } else {                //low speed
		R8_USB_CTRL |= RB_UC_LOW_SPEED;
		R8_UHOST_CTRL |= RB_UH_LOW_SPEED;
		R8_UH_SETUP |= RB_UH_PRE_PID_EN;
    }
}

/*********************************************************************
 * @fn      USBH_ResetRootHubPort
 *
 * @brief   Reset USB host port.
 *
 * @para    index: USB host port.
 *          mod: Reset host port operating mode.
 *               0=reset and wait end, 1=begin reset, 2=end reset
 *
 * @return  none
 */
void USBHDH_ResetRootHubPort(uint8_t mode)
{
    USBHDH_SetSelfAddr(0x00);
    USBHDH_SetSelfSpeed(TUSB_SPEED_FULL);
    if (mode <= 1) {
        R8_UHOST_CTRL = ( R8_UHOST_CTRL & ~RB_UH_LOW_SPEED ) | RB_UH_BUS_RESET;
    }
    if (mode == 0) {
        delay_ms(11); // Reset time from 10mS to 20mS
    }
    if (mode != 1) {
        R8_UHOST_CTRL &= ~RB_UH_BUS_RESET;
    }
    delay_ms(2);

    if (R8_USB_INT_FG & RB_UIF_DETECT) {
		if (R8_USB_MIS_ST & RB_UMS_DEV_ATTACH) {
			R8_USB_INT_FG = RB_UIF_DETECT;
		}
    }
}

/*********************************************************************
 * @fn      USBH_EnableRootHubPort
 *
 * @brief   Enable host port.
 *
 * @para    index: USB host port.
 *
 * @return  none
 */
uint8_t USBHDH_EnableRootHubPort(uint8_t* pspeed)
{
    if (R8_USB_MIS_ST & RB_UMS_DEV_ATTACH) {
        if (USBHDH_CheckRootHubPortEnable() == 0x00) {
            *pspeed = USBHDH_CheckRootHubPortSpeed();
            if (*pspeed == TUSB_SPEED_LOW) {
                USBHDH_SetSelfSpeed(TUSB_SPEED_LOW);
            }
        }
		R8_UHOST_CTRL |= RB_UH_PORT_EN;
		R8_UH_SETUP |= RB_UH_SOF_EN;

        
		return ERR_SUCCESS;
    }

    return ERR_USB_DISCON;
}

/*********************************************************************
 * @fn      USBH_Transact
 *
 * @brief   Perform USB transaction.
 *
 * @para    endp_pid: Endpoint number.
 *          tog:
 *          timeout: Timeout time.
 *
 * @return  none
 */
uint8_t USBHDH_Transact( uint8_t endp_pid, uint8_t endp_tog, uint32_t timeout )
{
    uint8_t   r, trans_rerty;
    uint16_t  i;

    R8_UH_RX_CTRL = R8_UH_TX_CTRL = endp_tog;
    trans_rerty = 0;
    do
    {
        R8_UH_EP_PID = endp_pid; // Specify token PID and endpoint number
        R8_USB_INT_FG = RB_UIF_TRANSFER; // Allow transmission
        for( i = DEF_WAIT_USB_TOUT_200US; ( i != 0 ) && ( ( R8_USB_INT_FG & RB_UIF_TRANSFER ) == 0 ); i-- )
        {
            delay_us( 40 );
        }
        R8_UH_EP_PID = 0x00; // Stop USB transfer

        if( ( R8_USB_INT_FG & RB_UIF_TRANSFER ) == 0 )
        {
            return ERR_USB_UNKNOWN;
        }
        else
        {
            /* Complete transfer */
            if( R8_USB_INT_ST & RB_UIS_TOG_OK )
            {
                return ERR_SUCCESS;
            }
            r = R8_USB_INT_ST & MASK_UIS_H_RES; // USB device answer status

            if( r == USB_PID_STALL )
            {
                return ( r | ERR_USB_TRANSFER );
            }
            if( r == USB_PID_NAK )
            {
                if( timeout == 0 )
                {
                    return ( r | ERR_USB_TRANSFER );
                }
                if( timeout < 0xFFFF )
                {
                    timeout--;
                }
                --trans_rerty;
            }
            else switch ( endp_pid >> 4 )
            {
                case USB_PID_SETUP:
                case USB_PID_OUT:
                    if( r )
                    {
                        return ( r | ERR_USB_TRANSFER );
                    }
                    break; 
                case USB_PID_IN:
                    if( ( r == USB_PID_DATA0 ) && ( r == USB_PID_DATA1 ) ) 
                    {  
                        ;
                    }  
                    else if( r ) 
                    {
                        return ( r | ERR_USB_TRANSFER );
                    }
                    break;
                default:
                    return ERR_USB_UNKNOWN;
            }
        }
        delay_us( 15 );
        if( R8_USB_INT_ST & RB_UIF_DETECT )
        {
            delay_us( 200 );
            if( USBHDH_CheckRootHubPortEnable( ) == 0 )
            {
                return ERR_USB_DISCON; // USB device disconnect event
            }
        }
    }while( ++trans_rerty < 10 );

    return ERR_USB_TRANSFER;  // Reply timeout
}

/*********************************************************************
 * @fn      USBH_CtrlTransfer
 *
 * @brief
 *
 * @return  none
 */
uint8_t USBHDH_CtrlTransfer( uint8_t ep0_size,  USB_SETUP_REQ *preq, uint8_t *pbuf, uint16_t *plen )
{
    uint8_t  s;
    uint16_t rem_len, rx_len, rx_cnt, tx_cnt;

    delay_us( 100 );
    
    if( plen )
    {
        *plen = 0;
    }
    R8_UH_TX_LEN = sizeof( USB_SETUP_REQ );
    memcpy(USBHD_TX_Buf,preq,sizeof(USB_SETUP_REQ));
    s = USBHDH_Transact( USB_PID_SETUP << 4 | 0x00, 0x00, DEF_CTRL_TRANS_TIMEOVER_CNT );  // SETUP stage, 200mS timeout
    if( s != ERR_SUCCESS )
    {
        return s;
    }
    R8_UH_RX_CTRL = R8_UH_TX_CTRL = RB_UH_R_TOG | RB_UH_R_AUTO_TOG | RB_UH_T_TOG | RB_UH_T_AUTO_TOG;  // Default DATA1
    R8_UH_TX_LEN = 0x01; // The default no-data state stage is IN
    rem_len = preq->wLength;
    if( rem_len && pbuf )
    {
        if( preq->bRequestType & USB_REQ_TYP_IN )
        {
            /* Receive data */
            while( rem_len )
            {
                delay_us( 100 );
                s = USBHDH_Transact( USB_PID_IN << 4 | 0x00, R8_UH_RX_CTRL, DEF_CTRL_TRANS_TIMEOVER_CNT );  // IN
                if( s != ERR_SUCCESS )
                {
                    return s;
                }
                rx_len = R8_USB_RX_LEN < rem_len ? R8_USB_RX_LEN : rem_len;
                rem_len -= rx_len;
                if( plen )
                {
                    *plen += rx_len; // The total length of the actual successful transmission and reception
                }
                for( rx_cnt = 0; rx_cnt != rx_len; rx_cnt++ )
                {
                    *pbuf = USBHD_RX_Buf[ rx_cnt ];
                    pbuf++;
                }

                if( R8_USB_RX_LEN == 0 || ( R8_USB_RX_LEN & ( ep0_size - 1 ) ) )
                {
                    break; // Short package
                }
            }
            R8_UH_TX_LEN = 0x00; // Status stage is OUT
        }
        else
        {
            /* Send data */
            while( rem_len )
            {
                delay_us( 100 );
                R8_UH_TX_LEN = rem_len >= ep0_size ? ep0_size : rem_len;
                for( tx_cnt = 0; tx_cnt != R8_UH_TX_LEN; tx_cnt++ )
                {
                    USBHD_TX_Buf[ tx_cnt ] = *pbuf;
                    pbuf++;
                }
                s = USBHDH_Transact( USB_PID_OUT << 4 | 0x00, R8_UH_TX_CTRL, DEF_CTRL_TRANS_TIMEOVER_CNT ); // OUT
                if( s != ERR_SUCCESS )
                {
                    return( s );
                }
                rem_len -= R8_UH_TX_LEN;
                if( plen )
                {
                    *plen += R8_UH_TX_LEN; // The total length of the actual successful transmission and reception
                }
            }
        }
    }
    delay_us( 100 );
    s = USBHDH_Transact( ( R8_UH_TX_LEN ? USB_PID_IN << 4 | 0x00: USB_PID_OUT << 4 | 0x00 ), RB_UH_R_TOG | RB_UH_T_TOG, DEF_CTRL_TRANS_TIMEOVER_CNT );  /* STATUS�׶� */
    if( s != ERR_SUCCESS )
    {
        return s;
    }
    if( R8_UH_TX_LEN == 0 )
    {
        return ERR_SUCCESS;
    }
    if( R8_USB_RX_LEN == 0 )
    {
        return ERR_SUCCESS;
    }
    return ERR_USB_BUF_OVER;
}



/*********************************************************************
 * @fn      USBFSH_GetEndpData
 *
 * @brief   Get data from USB device input endpoint.
 *
 * @para    endp_num: Endpoint number.
 *          endp_tog: Endpoint toggle.
 *          *pbuf: Data Buffer.
 *          *plen: Data length.
 *
 * @return  The result of getting data.
 */
uint8_t USBHDH_GetEndpData( uint8_t endp_num, uint8_t *pendp_tog, uint8_t *pbuf, uint16_t *plen )
{
    uint8_t  s;
    
    s = USBHDH_Transact( ( USB_PID_IN << 4 ) | endp_num, *pendp_tog, 0 );
    if( s == ERR_SUCCESS )
    {
        *plen = R8_USB_RX_LEN;
        memcpy( pbuf, USBHD_RX_Buf, *plen );
        
        *pendp_tog ^= RB_UH_R_TOG;
    }
    
    return s;
}

/*********************************************************************
 * @fn      USBHDH_SendEndpData
 *
 * @brief   Send data to the USB device output endpoint.
 *
 * @para    endp_num: Endpoint number
 *          endp_tog: Endpoint toggle
 *          *pbuf: Data Buffer
 *          *plen: Data length
 *
 * @return  The result of sending data.
 */
uint8_t USBHDH_SendEndpData( uint8_t endp_num, uint8_t *pendp_tog, uint8_t *pbuf, uint16_t len )
{
    uint8_t  s;
    
    memcpy( USBHD_TX_Buf, pbuf, len );
    R8_UH_TX_LEN = len;
    
    s = USBHDH_Transact( ( USB_PID_OUT << 4 ) | endp_num, *pendp_tog, 0 );
    if( s == ERR_SUCCESS )  
    {
        *pendp_tog ^= RB_UH_T_TOG;
    }
    
    return s;
}

/*****************************************************************************************************
**  Function
******************************************************************************************************/
/*******************************************************************
** Parameters:		
** Returns:	
** Description:		
*******************************************************************/
error_t hal_usbh_set_status(uint8_t id,usb_state_t usb_sta)
{
	return ERROR_SUCCESS;
}
/*******************************************************************
** Parameters:		
** Returns:	
** Description:		
*******************************************************************/
error_t hal_usbh_port_en(uint8_t id,uint8_t en, usb_speed_t* pspeed)
{
    uint8_t i,s;
	if(en){
        for( i = 0, s = 0; i < DEF_RE_ATTACH_TIMEOUT; i++ ){
            if( USBHDH_EnableRootHubPort((uint8_t*)pspeed ) == ERR_SUCCESS ){
                i = 0;
                s++;
                if( s > 6 ){
                    break;
                }
            }
            delay_ms( 1 );
        }
        if(0 == i){
            return ERROR_SUCCESS;
        }else{
            return ERROR_DISCON;
        }

	}else{
		R8_UHOST_CTRL &= ~RB_UH_PORT_EN;
		return ERROR_SUCCESS;
	}

}

error_t hal_usbh_port_reset(uint8_t id, uint8_t reset_ms)
{
	USBHDH_ResetRootHubPort(0);
	return ERROR_SUCCESS;
}

error_t hal_usbh_set_speed(uint8_t id, usb_speed_t speed)
{
	USBHDH_SetSelfSpeed(speed);
    
    if (id & 0X0F){                         // 通过外部HUB与低速USB设备通讯,设置root hub状态
        R8_UHOST_CTRL &= ~RB_UH_LOW_SPEED;  //这里简单处理默认hub 为full speed
    }
	return ERROR_SUCCESS;
}
error_t hal_usbh_set_addr(uint8_t id,uint8_t addr)
{
	USBHDH_SetSelfAddr( addr );
	return ERROR_SUCCESS;
}
error_t hal_usbh_endp_unregister(uint8_t id,usb_endp_t *endpp)
{
	return ERROR_SUCCESS;
}
error_t hal_usbh_endp_register(uint8_t id,usb_endp_t *endpp)
{
	return ERROR_SUCCESS;
}
error_t hal_usbh_ctrl_transfer( uint8_t id, usb_control_request_t* preq,uint8_t* buf, uint16_t* plen)
{
    error_t err;
    usbh_dev_t* pdev = get_usbh_dev(id);

    err = (error_t)USBHDH_CtrlTransfer( pdev->endp0_mtu,  (USB_SETUP_REQ *)preq, buf, plen );

    err =  err ?  ERROR_FAILE: ERROR_SUCCESS;
	return err;
}

error_t hal_usbh_in(uint8_t id, usb_endp_t *endpp, uint8_t* buf,uint16_t* plen,uint16_t timeout_ms)
{
    error_t err;
    uint8_t tog = endpp->sync? RB_UH_R_TOG : 0;
    uint8_t endp_num = endpp->addr;

    err = (error_t)USBHDH_GetEndpData(endp_num, &tog, buf, plen );
    endpp->sync = BOOL_SET(tog);

    err =  err ?  ERROR_FAILE: ERROR_SUCCESS;
	return err;
}
error_t hal_usbh_out(uint8_t id, usb_endp_t *endpp, uint8_t* buf, uint16_t len)
{
    error_t err;
    uint8_t tog = endpp->sync? RB_UH_T_TOG : 0;
    uint8_t endp_num = endpp->addr;

    err = (error_t)USBHDH_SendEndpData( endp_num, &tog, buf, len );
    err =  err ?  ERROR_FAILE: ERROR_SUCCESS;

	return err;
}
error_t hal_usbh_driver_init(uint8_t id)
{
    logd("hal_usbh_driver_init\n");
    // RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE );
	USBHD_RCC_Init(  );
	USBHD_Host_Init( ENABLE );
	return ERROR_SUCCESS;
}
error_t hal_usbh_driver_deinit(uint8_t id)
{
	USBHD_Host_Init( DISABLE );
	return ERROR_SUCCESS;
}
void hal_usbh_driver_task(uint32_t dt_ms)
{
	USBHDH_CheckRootHubPortStatus();
}

#endif











