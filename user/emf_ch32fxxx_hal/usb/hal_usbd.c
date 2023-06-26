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

 //ch32f10xx 有两个USB, 通过CH32_USBD_ID 选择使用哪一个usb硬件
#include "usb/hal_usbd.h"

#if (BIT(0) == API_USBD_BIT_ENABLE)     

#define USBD_ID         0

/************************************************************************************************************
**	Description:
************************************************************************************************************/
#include "api/usb/device/usbd.h"
#include "api/usb/usb_typedef.h"
#include "api/api_tick.h"
#include "hw_config.h"

#include "api/api_log.h"

/******************************************************************************************************
** Defined
*******************************************************************************************************/
#define Fullspeed
/******************************************************************************************************
**	public Parameters
*******************************************************************************************************/

/******************************************************************************************************
**	static Parameters
*******************************************************************************************************/

#if !defined(__C51__)
__ALIGN(4) uint8x_t Ep0Buffer[(64 + 2) + (64 + 2) * 2]; // 端点0+4 OUT&IN缓冲区，必须是偶地址
__ALIGN(4) uint8x_t Ep1Buffer[(64 + 2) * 2]; // 端点1 IN/OUT
__ALIGN(4) uint8x_t Ep2Buffer[(64 + 2) * 2]; // 端点2 IN/OUT
__ALIGN(4) uint8x_t Ep3Buffer[(64 + 2) * 2]; // 端点3 IN/OUT
#else
uint8x_t Ep0Buffer[(64 + 2) + (64 + 2) * 2] _at_(0x0080); // 端点0+4 OUT&IN缓冲区，必须是偶地址
uint8x_t Ep1Buffer[(64 + 2) * 2] _at_(0x80 + (64 + 2) + (64 + 2) * 2); // 端点1 IN/OUT
uint8x_t Ep2Buffer[(64 + 2) * 2] _at_(0x80 + (64 + 2) + (64 + 2) * 4); // 端点2 IN/OUT
uint8x_t Ep3Buffer[(64 + 2) * 2] _at_(0x80 + (64 + 2) + (64 + 2) * 6); // 端点3 IN/OUT
#endif

/*****************************************************************************************************
**	static Function
******************************************************************************************************/

/*********************************************************************
 * @fn      USBHD_RCC_Init
 *
 * @brief   Set USB clock.
 *
 * @return  none
 */
void USBHD_RCC_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
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




/*******************************************************************************
* Function Name  : DeviceInterrupt()
* Description    : CH559USB中断处理函数
*******************************************************************************/
void USBHD_IRQHandler( void )
{
    uint16_t len;
    uint8_t ep;
    uint8_t  intst;

    intst = USB_INT_ST;
    ep = intst & MASK_UIS_ENDP;

    if (UIF_TRANSFER) { // USB传输完成标志
        switch (intst & MASK_UIS_TOKEN) {

        case UIS_TOKEN_IN:
            usbd_endp_in_event(USBD_ID, USB_DIR_IN_MASK | ep);
            if(0 == ep){
                UEP0_CTRL ^= bUEP_T_TOG; // 同步标志位翻转
            }else if(4 == ep){
                UEP4_CTRL ^= bUEP_T_TOG; // 同步标志位翻转
            }
            break;
        case UIS_TOKEN_OUT:
            if (U_TOG_OK) {                     // 不同步的数据包将丢弃
                usbd_endp_out_event(USBD_ID, ep, USB_RX_LEN);
                if(0 == ep){
                    UEP0_CTRL ^= bUEP_R_TOG; // 同步标志位翻转
                }else if(4 == ep){
                    UEP4_CTRL ^= bUEP_R_TOG; // 同步标志位翻转
                }
            }
            break;
        case UIS_TOKEN_SETUP: // SETUP
            UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_NAK;
            len = USB_RX_LEN;
            if (len == sizeof(usb_control_request_t)) {
                UEP0_T_LEN = 0;
                usbd_setup_event(USBD_ID, (usb_control_request_t*)Ep0Buffer , sizeof(usb_control_request_t));
            } else {
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL; // STALL
            }
            break;
        default:
            break;
        }
        USB_INT_FG = RB_UIF_TRANSFER;
    } else if (UIF_BUS_RST){
        /* usb reset interrupt processing */
        usbd_reset_event(USBD_ID);
        USB_INT_FG |= RB_UIF_BUS_RST;
    } else if (UIF_SUSPEND){
        usbd_suspend_event(USBD_ID);
        R8_USB_INT_FG = RB_UIF_SUSPEND;
    } else { 
        USB_INT_FG = 0XFF;
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
error_t hal_usbd_endp_dma_init(uint8_t id)
{
	return ERROR_SUCCESS;
}
error_t hal_usbd_endp_open(uint8_t id, usb_endp_t* pendp)
{
    if (USBD_ID != id) return ERROR_FAILE;

    uint8d_t mode = 0, ctrl = 0;
    // logd("enp init %d %d\n",(uint16_t)(endp), (uint16_t)(in_out));

    if (0 == pendp->addr) {
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK; // OUT事务返回ACK，IN事务返回NAK
        return ERROR_SUCCESS;
    }

    if (pendp->dir) {
        mode |= bUEP2_TX_EN;
        ctrl |= UEP_T_RES_NAK;
    } else {
        mode |= bUEP2_RX_EN;
        ctrl |= UEP_R_RES_ACK;
    }
    if ((pendp->addr == 1) || (pendp->addr == 3)) {
        mode <<= 4;
    }

    switch (pendp->addr) {
    case 1:
        UEP4_1_MOD |= mode;
        UEP1_CTRL = bUEP_AUTO_TOG | ctrl; // 端点1自动翻转同步标志位，IN事务返回NAK，OUT返回ACK
        break;
    case 2:
        UEP2_3_MOD |= mode;
        UEP2_CTRL = bUEP_AUTO_TOG | ctrl; // 端点2自动翻转同步标志位，IN事务返回NAK，OUT返回ACK
        break;
    case 3:
        UEP2_3_MOD |= mode;
        UEP3_CTRL = bUEP_AUTO_TOG | ctrl; // 端点3自动翻转同步标志位，IN事务返回NAK，OUT返回ACK
        break;
    case 4:
        UEP4_1_MOD |= mode; 
        UEP4_CTRL = ctrl;               // 端点4不支持自动翻转同步标志位 
        break;
    }

    return ERROR_SUCCESS;
}

error_t hal_usbd_endp_close(uint8_t id, uint8_t ep)
{
    uint8_t ep_addr = ep & 0x7f;

    if (USBD_ID != id) return ERROR_FAILE;

    if (0 == ep_addr) {
        return ERROR_SUCCESS;
    }

    switch (ep_addr) {
    case 1:
        UEP4_1_MOD &= ~(bUEP1_TX_EN | bUEP1_RX_EN | bUEP1_BUF_MOD);
        break;
    case 2:
        UEP2_3_MOD &= ~(bUEP2_TX_EN | bUEP2_RX_EN | bUEP2_BUF_MOD);
        break;
    case 3:
        UEP2_3_MOD &= ~(bUEP3_TX_EN | bUEP3_RX_EN | bUEP3_BUF_MOD);
        break;
    case 4:
        UEP4_1_MOD &= ~(bUEP4_TX_EN | bUEP4_RX_EN);     //端点4 dma buf和端点0 共用
        break;
    }

    return ERROR_SUCCESS;
}
error_t hal_usbd_endp_ack(uint8_t id, uint8_t ep, uint16_t len)
{
    if (USBD_ID != id) return ERROR_FAILE;

    switch (ep) {
    case 0x80:
        UEP0_T_LEN = len;
		UEP0_CTRL = (UEP0_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
        break;
    case 0x81:
        UEP1_T_LEN = len;
		UEP1_CTRL = (UEP1_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
        break;
    case 0x82:
        UEP2_T_LEN = len;
        UEP2_CTRL = (UEP2_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
        break;
    case 0x83:
        UEP3_T_LEN = len;
        UEP3_CTRL = (UEP3_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
        break;
    case 0x84:
        UEP4_T_LEN = len;
        UEP4_CTRL = (UEP4_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
        break;
        
    case 0x00:
        UEP0_CTRL = UEP0_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;  //重新开启接收
        break;
    case 0x01:
        UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;  //重新开启接收
        break;
    case 0x02:
        UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;  //重新开启接收
        break;
    case 0x03:
        UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;  //重新开启接收
        break;
    case 0x04:
        UEP4_CTRL = UEP4_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;  //重新开启接收
        break;
    default:
        break;
    }
	return ERROR_SUCCESS;
}
error_t hal_usbd_endp_nak(uint8_t id, uint8_t ep)
{
    if (USBD_ID != id) return ERROR_FAILE;
   
     // 关闭接收,等待数据处理
    switch (ep) {
    case 0x00:
        UEP0_CTRL = (UEP0_CTRL & ~MASK_UEP_R_RES) | UEP_R_RES_NAK;
        break;
    case 0x80:
        UEP0_T_LEN = 0;
        UEP0_CTRL = (UEP0_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
        break;
    case 0x81:
        UEP1_T_LEN = 0;
        UEP1_CTRL = UEP1_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; // 默认应答NAK
        break;
    case 0x82:
        UEP2_T_LEN = 0;
        UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; // 默认应答NAK
        break;
    case 0x83:
        UEP3_T_LEN = 0;
        UEP3_CTRL = UEP3_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; // 默认应答NAK
        break;
    case 0x84:
        UEP4_T_LEN = 0;
        UEP4_CTRL = UEP4_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; // 默认应答NAK
        break;
    case 0x01:
        UEP1_CTRL = UEP1_CTRL & ~MASK_UEP_R_RES | UEP_R_RES_NAK; /* 设置端点2 OUT NAK */
        break;
    case 0x02:
        UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_R_RES | UEP_R_RES_NAK; /* 设置端点2 OUT NAK */
        break;
    case 0x03:
        UEP3_CTRL = UEP3_CTRL & ~MASK_UEP_R_RES | UEP_R_RES_NAK; /* 设置端点2 OUT NAK */
        break;
    case 0x04:
        UEP4_CTRL = UEP4_CTRL & ~MASK_UEP_R_RES | UEP_R_RES_NAK; /* 设置端点2 OUT NAK */
        break;
    default:
        break;
    }
    return ERROR_SUCCESS;
}
       



error_t hal_usbd_clear_endp_stall(uint8_t id, uint8_t ep)
{
    if (USBD_ID != id) return ERROR_FAILE;

    switch (ep) {
    case 0x00:
    case 0x80:
        UEP0_T_LEN = 0; // 虽然尚未到状态阶段，但是提前预置上传0长度数据包以防主机提前进入状态阶段
        UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK; // 默认数据包是DATA1,返回应答ACK
        break;
    case 0x81:
        UEP1_CTRL = UEP1_CTRL & ~(bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
        break;
    case 0x82:
        UEP2_CTRL = UEP2_CTRL & ~(bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
        break;
    case 0x83:
        UEP3_CTRL = UEP3_CTRL & ~(bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
        break;
    case 0x84:
        UEP4_CTRL = UEP4_CTRL & ~(bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
        break;
    case 0x01:
        UEP1_CTRL = UEP1_CTRL & ~(bUEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
        break;
    case 0x02:
        UEP2_CTRL = UEP2_CTRL & ~(bUEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
        break;
    case 0x03:
        UEP3_CTRL = UEP3_CTRL & ~(bUEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
        break;
    case 0x04:
        UEP4_CTRL = UEP4_CTRL & ~(bUEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
        break;
    default:
        break;
    }
    return ERROR_SUCCESS;
}
error_t hal_usbd_endp_stall(uint8_t id, uint8_t ep)
{
    if (USBD_ID != id) return ERROR_FAILE;

    switch (ep) {
    case 0x00:
    case 0x80:
        UEP0_T_LEN = 0;
        UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL; // STALL
        break;
    case 0x81:
        UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; /* 设置端点1 IN STALL */
        break;
    case 0x82:
        UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; /* 设置端点2 IN STALL */
        break;
    case 0x83:
        UEP3_CTRL = UEP3_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; /* 设置端点1 IN STALL */
        break;
    case 0x84:
        UEP4_CTRL = UEP4_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; /* 设置端点1 IN STALL */
        break;
    case 0x01:
        UEP1_CTRL = UEP1_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL; /* 设置端点2 OUT Stall */
        break;
    case 0x02:
        UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL; /* 设置端点2 OUT Stall */
        break;
    case 0x03:
        UEP3_CTRL = UEP3_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL; /* 设置端点2 OUT Stall */
        break;
    case 0x04:
        UEP4_CTRL = UEP4_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL; /* 设置端点2 OUT Stall */
        break;
    default:
        break;
    }

    return ERROR_SUCCESS;
}


uint8_t* hal_usbd_get_endp_buffer(uint8_t id, uint8_t ep)
{
    uint8_t *pbuf;
    uint8_t ep_addr = ep & 0x7f;

    if (USBD_ID != id) return NULL;

    switch (ep_addr) {
    case 0:
        return Ep0Buffer;
    case 1:
        pbuf = Ep1Buffer;
        if((ep & 0x80) && (UEP4_1_MOD & bUEP1_TX_EN) && (UEP4_1_MOD & bUEP1_RX_EN)){
            pbuf += 64;
        }

        break;
    case 2:
        pbuf = Ep2Buffer;
        if((ep & 0x80) && (UEP2_3_MOD & bUEP2_TX_EN) && (UEP2_3_MOD & bUEP2_RX_EN)){
            pbuf += 64;
        }
        break;
    case 3:
        pbuf = Ep3Buffer;
        if((ep & 0x80) && (UEP2_3_MOD & bUEP3_TX_EN) && (UEP2_3_MOD & bUEP3_RX_EN)){
            pbuf += 64;
        }
        break;
    case 4:
        pbuf = Ep0Buffer + USBD_ENDP0_MTU;
        if((ep & 0x80) && (UEP4_1_MOD & bUEP4_TX_EN) && (UEP4_1_MOD & bUEP4_RX_EN)){
            pbuf += 64;
        }
        break;
    default:
        pbuf = NULL;
        break;
    }

    return pbuf;
}



/*******************************************************************
** Parameters: buf: ch32f103平台端点0时参数无效,使用usbd_req_t中数据发送
** Returns:
** Description: 注意: 端点0 发送需要处理 usbd_req_t,usbd_free_setup_buffer释放空间
//TODO 后面优化这一部分代码
    调试记录:
        1. 设置nak后, in消息不会进中断,只有有效传送才会进中断
        2. 设置地址必须在ack 请求后再进行地址设置
*******************************************************************/
error_t hal_usbd_in(uint8_t id, uint8_t ep, uint8_t* buf, uint16_t len)
{
    error_t err = ERROR_FAILE;
    uint8_t ep_addr = ep & 0x7f;
    uint16_t send_len;
    uint8_t* endp_buf = hal_usbd_get_endp_buffer(id, USB_DIR_IN_MASK | ep);

    if (USBD_ID != id) return ERROR_FAILE;

    if(0 == ep_addr){
        usbd_req_t *preq = usbd_get_req(id);

        if(preq->setup_index <= preq->setup_len){
            send_len = preq->setup_len - preq->setup_index;
            send_len = (send_len >= USBD_ENDP0_MTU) ? USBD_ENDP0_MTU : send_len; //本次传输长度
            memcpy(Ep0Buffer, (void*)(preq->setup_buf+preq->setup_index), send_len);						          //加载上传数据
            preq->setup_index += send_len;

            err = hal_usbd_endp_ack(id, ep, send_len);
            if((preq->setup_index == preq->setup_len)){
                if(USBD_ENDP0_MTU != send_len){             //判断发送最后一包数据
                    usbd_free_setup_buffer(preq);           //发送完成释放内存
                    hal_usbd_endp_ack(id, 0x00, 0);         //开始接收
                }
            }
        }else{
            return ERROR_FAILE;
        }
    }else{
        memcpy(endp_buf, buf, len);
        err = hal_usbd_endp_ack(id, ep, len);
    }

    return err;
}

error_t hal_usbd_out(uint8_t id, uint8_t ep, uint8_t* buf, uint16_t* plen)
{
    uint8_t* p;

    if (USBD_ID != id) return ERROR_FAILE;

    p = hal_usbd_get_endp_buffer(id, ep);
    if(NULL != buf) memcpy(buf, p,*plen);
    hal_usbd_endp_ack(id, ep, 0);

    return ERROR_SUCCESS;
}
error_t hal_usbd_reset(uint8_t id)
{
    return hal_usbd_init(id);
}
error_t hal_usbd_set_address(uint8_t id, uint8_t address)
{
    if (USBD_ID != id) return ERROR_FAILE;

    USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | address;
    return ERROR_SUCCESS;
}

/*********************************************************************
 * @fn      USBHD_Device_Endp_Init
 *
 * @brief   Initializes USB device endpoints.
 *
 * @return  none
 */
void USBHD_Device_Endp_Init( void )
{
    /* Initiate DMA address, please modify them according to your project. */
    R16_UEP0_DMA = (uint16_t)(uint32_t)Ep0Buffer; // 端点0+4数据传输地址
    R16_UEP1_DMA = (uint16_t)(uint32_t)Ep1Buffer; // 端点1数据传输地址
    R16_UEP2_DMA = (uint16_t)(uint32_t)Ep2Buffer; // 端点2数据传输地址
    R16_UEP3_DMA = (uint16_t)(uint32_t)Ep3Buffer; // 端点3数据传输地址
    UEP0_T_LEN = 0;
    UEP1_T_LEN = 0; // 预使用发送长度一定要清空
    UEP2_T_LEN = 0; // 预使用发送长度一定要清空
    UEP3_T_LEN = 0; // 预使用发送长度一定要清空
    UEP4_T_LEN = 0; // 预使用发送长度一定要清空

    /* End-points initial states */
    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
}

error_t hal_usbd_init(uint8_t id)
{
    if (USBD_ID != id) return ERROR_FAILE;
    logd("hal_usbd_init\n");

    USBHD_RCC_Init();

    R8_USB_CTRL = RB_UC_RESET_SIE | RB_UC_CLR_ALL;
    delay_us( 20 );
    R8_USB_CTRL = 0x00; // 先设定模式,取消 RB_UC_CLR_ALL
    USB_INT_EN = RB_UIE_SUSPEND | RB_UIE_BUS_RST | RB_UIE_TRANSFER;
    R8_USB_CTRL = RB_UC_DEV_PU_EN | RB_UC_INT_BUSY | RB_UC_DMA_EN;
    USB_DEV_AD = 0x00; // 设备地址初始化
    USBHD_Device_Endp_Init( );
    R8_UDEV_CTRL = RB_UD_PD_DIS | RB_UD_PORT_EN;
    NVIC_EnableIRQ(USBHD_IRQn);

    return ERROR_SUCCESS;
}
error_t hal_usbd_deinit(uint8_t id)
{
    if (USBD_ID != id) return ERROR_FAILE;

    R8_USB_CTRL = RB_UC_RESET_SIE | RB_UC_CLR_ALL;
    delay_us( 10 );
    R8_USB_CTRL = 0x00; // 先设定模式,取消 RB_UC_CLR_ALL
    UDEV_CTRL = RB_UD_PD_DIS; // 禁止DP/DM下拉电阻
    NVIC_DisableIRQ(USBHD_IRQn);
    return ERROR_SUCCESS;
}

#endif

