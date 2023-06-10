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
#include "usb/hal_usbd.h"
#if (BIT(1) == API_USBD_BIT_ENABLE)     

#define USBD_ID         1

#include "api/usb/device/usbd.h"
#include "api/usb/usb_typedef.h"
#include "api/api_tick.h"
#include "hw_config.h"

#include "usb_lib.h"
#include "ch32f10x.h"

#include "api/api_log.h"




/******************************************************************************************************
** Defined
*******************************************************************************************************/
#define Fullspeed

#define USBD_ENDP_SIZE			64

/* Buffer Description Table */
/* buffer table base address */
/* buffer table base address */
#define BTABLE_ADDRESS      (0x00)

/* EP0  */
/* rx/tx buffer base address */
#define ENDP0_RXADDR        (0x40)
#define ENDP0_TXADDR        (0x80)


/* ISTR events */
/* IMR_MSK */
/* mask defining which events has to be handled */
/* by the device application software */
#define IMR_MSK (CNTR_CTRM  | CNTR_WKUPM | CNTR_SUSPM | CNTR_ERRM  | CNTR_SOFM \
                 | CNTR_ESOFM | CNTR_RESETM )


/******************************************************************************************************
**	public Parameters
*******************************************************************************************************/

/******************************************************************************************************
**	static Parameters
*******************************************************************************************************/
typedef enum _RESUME_STATE
{
  RESUME_EXTERNAL,
  RESUME_INTERNAL,
  RESUME_LATER,
  RESUME_WAIT,
  RESUME_START,
  RESUME_ON,
  RESUME_OFF,
  RESUME_ESOF
} RESUME_STATE;
struct
{
  __IO RESUME_STATE eState;
  __IO uint8_t bESOFcnt;
}ResumeS;


uint16_t  wInterrupt_Mask;
__IO bool fSuspendEnabled = true;  
__IO uint32_t remotewakeupon=0;

uint16_t Ep0RxBlks;
/* Private variables */
__IO uint16_t wIstr;  
__IO uint8_t bIntPackSOF = 0;  
__IO uint32_t esof_counter =0;
__IO uint32_t wCNTR=0;

volatile uint16_t 	usb_dma_address;            //max512 
/*****************************************************************************************************
**	static Function
******************************************************************************************************/

/*********************************************************************
 * @fn      Set_USBConfig
 *
 * @brief   Set_USBConfig.
 *
 * @return  none
 */
void Set_USBConfig()
{
  if (SystemCoreClock == 72000000) {
      RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
  } else if (SystemCoreClock == 48000000) {
      RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_Div1);
  }
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
}

/*********************************************************************
 * @fn      USB_Interrupts_Config
 *
 * @brief   Configrate USB interrupt.
 *
 * @return  none
 */
void USB_Interrupts_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  EXTI_ClearITPendingBit(EXTI_Line18);
  EXTI_InitStructure.EXTI_Line = EXTI_Line18;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = USBWakeUp_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_Init(&NVIC_InitStructure);
}

/*********************************************************************
 * @fn      USB_Port_Set
 *
 * @brief   Set USB IO port.
 *
 * @param   NewState - DISABLE or ENABLE.
 *          Pin_In_IPU - Enables or Disables intenal pull-up resistance.
 *
 * @return  none
 */
void USB_Port_Set(FunctionalState NewState, FunctionalState Pin_In_IPU)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  if (NewState) {
      _SetCNTR(_GetCNTR() & (~(1 << 1)));
      GPIOA->CFGHR &= 0XFFF00FFF;
      GPIOA->OUTDR &= ~(3 << 11); // PA11/12=0
      GPIOA->CFGHR |= 0X00088000; // IPD
  } else {
      _SetCNTR(_GetCNTR() | (1 << 1));
      GPIOA->CFGHR &= 0XFFF00FFF;
      GPIOA->OUTDR &= ~(3 << 11); // PA11/12=0
      GPIOA->CFGHR |= 0X00033000; // LOW
  }

  if (Pin_In_IPU)
      (EXTEN->EXTEN_CTR) |= EXTEN_USBD_PU_EN;
  else
      (EXTEN->EXTEN_CTR) &= ~EXTEN_USBD_PU_EN;
}

/*********************************************************************
 * @fn      USB_SIL_Init
 *
 * @brief   Initialize the USB Device IP and the Endpoint 0.
 *
 * @return  Status.
 */
uint32_t USB_SIL_Init(void)
{
  _SetISTR(0);
  wInterrupt_Mask = IMR_MSK;
  _SetCNTR(wInterrupt_Mask);
	
  return 0;
}



    /*********************************************************************
 * @fn      SetDeviceAddress..
 *
 * @brief   Set the device and all the used Endpoints addresses.
 *
 * @param   Val: device address.
 *
 * @return  none
 */	
void SetDeviceAddress(uint8_t Val)
{
  uint32_t i;

  for (i = 0; i < USBD_ENDP_NUM; i++)
  {
    _SetEPAddress((uint8_t)i, (uint8_t)i);
  } 
	
  _SetDADDR(Val | DADDR_EF); 
}

/*********************************************************************
 * @fn      Enter_LowPowerMode
 *
 * @brief   Enter low power mode.
 *
 * @return  none
 */
void Enter_LowPowerMode(void)
{
  printf("usb enter low power mode\r\n");
}

/*******************************************************************************
 * Function Name  : PowerOn
 * Description    : Enable power on.
 * Input          : None.
 * Return         : USB_SUCCESS.
 *******************************************************************************/
bool PowerOn(void)
{
  uint16_t wRegVal;

  wRegVal = CNTR_FRES;
  _SetCNTR(wRegVal);
  wInterrupt_Mask = 0;
  _SetCNTR(wInterrupt_Mask);
  _SetISTR(0);
  wInterrupt_Mask = CNTR_RESETM | CNTR_SUSPM | CNTR_WKUPM;
  _SetCNTR(wInterrupt_Mask);

  return true;
}

/*******************************************************************************
 * Function Name  : PowerOff
 * Description    : handles switch-off conditions
 * Input          : None.
 * Return         : USB_SUCCESS.
 *******************************************************************************/
bool PowerOff()
{
  _SetCNTR(CNTR_FRES);
  _SetISTR(0);
  _SetCNTR(CNTR_FRES + CNTR_PDWN);

  return true;
}

/*******************************************************************************
 * Function Name  : Suspend
 * Description    : sets suspend mode operating conditions
 * Input          : None.
 * Return         : USB_SUCCESS.
 *******************************************************************************/
void Suspend(void)
{
    __IO uint32_t EP[8];
    uint32_t i = 0;
    uint16_t wCNTR;
    __IO uint32_t savePWR_CR = 0;

    wCNTR = _GetCNTR();
    for (i = 0; i < 8; i++)
        EP[i] = _GetENDPOINT(i);

    wCNTR |= CNTR_RESETM;
    _SetCNTR(wCNTR);

    wCNTR |= CNTR_FRES;
    _SetCNTR(wCNTR);

    wCNTR &= ~CNTR_FRES;
    _SetCNTR(wCNTR);

    while ((_GetISTR() & ISTR_RESET) == 0)
        ;

    _SetISTR((uint16_t)CLR_RESET);

    for (i = 0; i < 8; i++)
        _SetENDPOINT(i, EP[i]);

    wCNTR |= CNTR_FSUSP;
    _SetCNTR(wCNTR);

    wCNTR = _GetCNTR();
    wCNTR |= CNTR_LPMODE;
    _SetCNTR(wCNTR);

    Enter_LowPowerMode();
}

/*******************************************************************************
 * Function Name  : Resume_Init
 * Description    : Handles wake-up restoring normal operations
 * Input          : None.
 * Return         : USB_SUCCESS.
 *******************************************************************************/
void Resume_Init(void)
{
    uint16_t wCNTR;

    wCNTR = _GetCNTR();
    wCNTR &= (~CNTR_LPMODE);
    _SetCNTR(wCNTR);
    _SetCNTR(IMR_MSK);
}

/*******************************************************************************
 * Function Name  : Resume
 * Description    : This is the state machine handling resume operations and
 *                 timing sequence. The control is based on the Resume structure
 *                 variables and on the ESOF interrupt calling this subroutine
 *                 without changing machine state.
 * Input          : a state machine value (RESUME_STATE)
 *                  RESUME_ESOF doesn't change ResumeS.eState allowing
 *                  decrementing of the ESOF counter in different states.
 * Return         : None.
 *******************************************************************************/
void Resume(RESUME_STATE eResumeSetVal)
{
    uint16_t wCNTR;

    if (eResumeSetVal != RESUME_ESOF) {
        ResumeS.eState = eResumeSetVal;
    }

    switch (ResumeS.eState) {
    case RESUME_EXTERNAL:
        if (remotewakeupon == 0) {
            Resume_Init();
            ResumeS.eState = RESUME_OFF;
        } else {
            ResumeS.eState = RESUME_ON;
        }
        break;

    case RESUME_INTERNAL:
        Resume_Init();
        ResumeS.eState = RESUME_START;
        remotewakeupon = 1;
        break;

    case RESUME_LATER:
        ResumeS.bESOFcnt = 2;
        ResumeS.eState = RESUME_WAIT;
        break;

    case RESUME_WAIT:
        ResumeS.bESOFcnt--;
        if (ResumeS.bESOFcnt == 0)
            ResumeS.eState = RESUME_START;
        break;

    case RESUME_START:
        wCNTR = _GetCNTR();
        wCNTR |= CNTR_RESUME;
        _SetCNTR(wCNTR);
        ResumeS.eState = RESUME_ON;
        ResumeS.bESOFcnt = 10;
        break;

    case RESUME_ON:
        ResumeS.bESOFcnt--;
        if (ResumeS.bESOFcnt == 0) {
            wCNTR = _GetCNTR();
            wCNTR &= (~CNTR_RESUME);
            _SetCNTR(wCNTR);
            ResumeS.eState = RESUME_OFF;
            remotewakeupon = 0;
        }
        break;

    case RESUME_OFF:

    case RESUME_ESOF:

    default:
        ResumeS.eState = RESUME_OFF;
        break;
    }
}

/*******************************************************************************
 * Function Name  : CTR_LP.
 * Description    : Low priority Endpoint Correct Transfer interrupt's service
 *                  routine.
 * Input          : None.
 * Return         : None.
 *******************************************************************************/
void CTR_LP(void)
{
    __IO uint16_t wIstr;
    __IO uint16_t wEPVal = 0;
    __IO uint16_t len;
    __IO uint8_t EPindex;

    while (((wIstr = _GetISTR()) & ISTR_CTR) != 0) {
        EPindex = (uint8_t)(wIstr & ISTR_EP_ID);

        if (EPindex == 0) {
            _SetEPRxTxStatus(ENDP0, EP_RX_NAK, EP_TX_NAK);
			
            if ((wIstr & ISTR_DIR) == 0){ // IN
                _ClearEP_CTR_TX(ENDP0);
                usbd_endp_in_event(USBD_ID, USB_DIR_IN_MASK | EPindex);
                // SetEPRxCount(ENDP0, USBD_ENDP0_MTU);
                return;
            } else {
                wEPVal = _GetENDPOINT(ENDP0);

                if ((wEPVal & EP_SETUP) != 0){  // EP_SETUP
                    usb_control_request_t req;
                    
                    _ClearEP_CTR_RX(ENDP0);
                    PMAToUserBufferCopy((uint8_t*)&req, GetEPRxAddr(ENDP0), 8);
                    usbd_setup_event(USBD_ID, &req , sizeof(usb_control_request_t));
                    return;
                } else if ((wEPVal & EP_CTR_RX) != 0){ // out
                    _ClearEP_CTR_RX(ENDP0);
                    len = GetEPRxCount(0);
                    usbd_endp_out_event(USBD_ID, EPindex, len);
                    return;
                } else {
                    wEPVal = _GetENDPOINT(ENDP0);
                    logd("i %x %x\n", wIstr, wEPVal);
                }
            }
        } else {
            wEPVal = _GetENDPOINT(EPindex);
            if ((wEPVal & EP_CTR_RX) != 0) {            //endp rx isr
                len = GetEPRxCount(EPindex);
                usbd_endp_out_event(USBD_ID, EPindex, len);
                _ClearEP_CTR_RX(EPindex);
            }

            if ((wEPVal & EP_CTR_TX) != 0) {           //endp tx isr
                usbd_endp_in_event(USBD_ID, USB_DIR_IN_MASK | EPindex);
                _ClearEP_CTR_TX(EPindex);
            }
        }
    }
}

/*********************************************************************
 * @fn      USB_Istr
 *
 * @brief   ISTR events interrupt service routine
 *
 * @return  none
 */
void USB_Istr(void)
{
    uint32_t i = 0;
    __IO uint32_t EP[8];

    if ((*_pEPRxCount(0) & 0xFC00) != Ep0RxBlks) {
        *_pEPRxCount(0) |= (Ep0RxBlks & 0xFC00);
    }
    wIstr = _GetISTR();
    #if (IMR_MSK & ISTR_SOF)
    if (wIstr & ISTR_SOF & wInterrupt_Mask) {
        _SetISTR((uint16_t)CLR_SOF);
        bIntPackSOF++;

        #ifdef SOF_CALLBACK
        SOF_Callback();
        #endif
    }
    #endif

    #if (IMR_MSK & ISTR_CTR)
    if (wIstr & ISTR_CTR & wInterrupt_Mask) {
        CTR_LP();
    #ifdef CTR_CALLBACK
        CTR_Callback();
    #endif
    }
    #endif

    #if (IMR_MSK & ISTR_RESET)
    if (wIstr & ISTR_RESET & wInterrupt_Mask) {
        _SetISTR((uint16_t)CLR_RESET);
        usbd_reset_event(USBD_ID);
    #ifdef RESET_CALLBACK
        RESET_Callback();
    #endif
    }
    #endif

    #if (IMR_MSK & ISTR_DOVR)
    if (wIstr & ISTR_DOVR & wInterrupt_Mask) {
        _SetISTR((uint16_t)CLR_DOVR);
    #ifdef DOVR_CALLBACK
        DOVR_Callback();
    #endif
    }
    #endif

    #if (IMR_MSK & ISTR_ERR)
    if (wIstr & ISTR_ERR & wInterrupt_Mask) {
        _SetISTR((uint16_t)CLR_ERR);
    #ifdef ERR_CALLBACK
        ERR_Callback();
    #endif
    }
    #endif

    #if (IMR_MSK & ISTR_WKUP)
    if (wIstr & ISTR_WKUP & wInterrupt_Mask) {
        _SetISTR((uint16_t)CLR_WKUP);
        Resume(RESUME_EXTERNAL);
        #ifdef WKUP_CALLBACK
        WKUP_Callback();
        #endif
    }
    #endif
    #if (IMR_MSK & ISTR_SUSP)
    if (wIstr & ISTR_SUSP & wInterrupt_Mask) {
        if (fSuspendEnabled) {
            Suspend();
        } else {
            Resume(RESUME_LATER);
        }
        _SetISTR((uint16_t)CLR_SUSP);
        #ifdef SUSP_CALLBACK
        SUSP_Callback();
        #endif

        usbd_suspend_event(USBD_ID);
    }
    #endif

    #if (IMR_MSK & ISTR_ESOF)
    if (wIstr & ISTR_ESOF & wInterrupt_Mask) {
        _SetISTR((uint16_t)CLR_ESOF);

        if ((_GetFNR() & FNR_RXDP) != 0) {
            esof_counter++;

            if ((esof_counter > 3) && ((_GetCNTR() & CNTR_FSUSP) == 0)) {

                wCNTR = _GetCNTR();

                for (i = 0; i < 8; i++)
                    EP[i] = _GetENDPOINT(i);

                wCNTR |= CNTR_FRES;
                _SetCNTR(wCNTR);

                wCNTR &= ~CNTR_FRES;
                _SetCNTR(wCNTR);

                while ((_GetISTR() & ISTR_RESET) == 0)
                    ;

                _SetISTR((uint16_t)CLR_RESET);

                for (i = 0; i < 8; i++)
                    _SetENDPOINT(i, EP[i]);

                esof_counter = 0;
            }
        } else {
            esof_counter = 0;
        }

        Resume(RESUME_ESOF);

        #ifdef ESOF_CALLBACK
        ESOF_Callback();
        #endif
    }
    #endif
} /* USB_Istr */


/*********************************************************************
 * @fn      USBWakeUp_IRQHandler
 *
 * @brief   This function handles USB wake up exception.
 *
 * @return  none
 */
void USBWakeUp_IRQHandler(void)
{
    EXTI_ClearITPendingBit(EXTI_Line18);
}

/*********************************************************************
 * @fn      USBWakeUp_IRQHandler
 *
 * @brief   This function handles USB exception.
 *
 * @return  none
 */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    USB_Istr();
}

/*****************************************************************************************************
**  Function
******************************************************************************************************/

error_t hal_usbd_endp_dma_init(uint8_t id)
{
    usb_dma_address = ENDP0_TXADDR + USBD_ENDP0_MTU;
	return ERROR_SUCCESS;
}

/*******************************************************************
** Parameters:
** Returns:
** Description: 注意: 不要多次打开相同端点
    usbd 使用的是硬件 fifo 最大fifo 512(USB基地址占用了底64byte(没有完全用完)
    因为双缓冲切换缓冲区的管理机制需要用到所有4个缓冲区描述表的表项，
    分别用来表示每个方向上的两个缓冲区的地址指针和缓冲区大小，
	因此用来实现双缓冲批量端点的USB_EpnR寄存器必需配置为单向。
	所以只需要设定STAT_RX位(作为双缓冲批量接收端点)或者STAT_TX位(作为双缓冲批量发送端点)。
	如果需要一个双向的双缓冲批量端点，则须使用两个USB_EpnR寄存器。
	
	同步传送必须是双buf, 所以同步传送一定是单独端点IN/out, 这里不使用音频双buf 设置为同一个buf
*******************************************************************/
error_t hal_usbd_endp_open(uint8_t id, usb_endp_t* pendp)
{
    uint16_t endp_add = usb_dma_address;

    if (USBD_ID != id) return ERROR_FAILE;

    if (0 == pendp->addr) {
        SetBTABLE(BTABLE_ADDRESS);
        SetEPType(ENDP0, EP_CONTROL);
		SetEPTxStatus(ENDP0, EP_TX_STALL);
		SetEPRxAddr(ENDP0, ENDP0_RXADDR);
		SetEPTxAddr(ENDP0, ENDP0_TXADDR);
		Clear_Status_Out(ENDP0);
		SetEPRxCount(ENDP0, USBD_ENDP0_MTU);
		SetEPRxValid(ENDP0);
        _ClearDTOG_RX(ENDP0);
        _ClearDTOG_TX(ENDP0);
        SetDeviceAddress(0);
    }else if (pendp->dir) {
        switch(pendp->type){
            case USB_ENDP_TYPE_CTRL:
                SetEPType(pendp->addr, EP_CONTROL);
            break;
            case USB_ENDP_TYPE_ISOCH:		//设置为同一个buf
                SetEPType(pendp->addr, EP_ISOCHRONOUS);
                SetEPRxAddr(pendp->addr, endp_add);
                SetEPRxCount(pendp->addr, pendp->mtu);
            break;
            case USB_ENDP_TYPE_BULK:
                SetEPType(pendp->addr, EP_BULK);
            break;
            case USB_ENDP_TYPE_INTER:
                SetEPType(pendp->addr, EP_INTERRUPT);
            break;
        }
        
        SetEPTxAddr(pendp->addr, endp_add);
        SetEPTxStatus(pendp->addr, EP_TX_NAK);
        _ClearDTOG_TX(pendp->addr);
        endp_add += pendp->mtu;
    } else {
        switch(pendp->type){
            case USB_ENDP_TYPE_CTRL:
                SetEPType(pendp->addr, EP_CONTROL);
            break;
            case USB_ENDP_TYPE_ISOCH:		//设置为同一个buf
                SetEPType(pendp->addr, EP_ISOCHRONOUS);
                SetEPTxAddr(pendp->addr, endp_add);
            break;
            case USB_ENDP_TYPE_BULK:
                SetEPType(pendp->addr, EP_BULK);
            break;
            case USB_ENDP_TYPE_INTER:
                SetEPType(pendp->addr, EP_INTERRUPT);
            break;
        }
        
        SetEPRxAddr(pendp->addr, endp_add);
        SetEPRxCount(pendp->addr, pendp->mtu);
        SetEPRxStatus(pendp->addr, EP_RX_VALID);
        _ClearDTOG_RX(pendp->addr);
        SetEPRxValid(pendp->addr);
        endp_add += pendp->mtu;
    }
    usb_dma_address = endp_add;
    logd("set ep%x addr=%x\n", ((uint8_t)pendp->dir << 7) | pendp->addr , usb_dma_address);

    if(usb_dma_address > 512){
        loge_r("usbd endp set over %d\n", usb_dma_address);
    }
    return ERROR_SUCCESS;
}

error_t hal_usbd_endp_close(uint8_t id, uint8_t ep)
{
    uint8_t ep_addr = ep & 0x7f;
    if (USBD_ID != id) return ERROR_FAILE;

    if (0 == ep_addr) return ERROR_FAILE;

    SetEPTxStatus(ep_addr, EP_TX_DIS);
    SetEPRxStatus(ep_addr, EP_RX_DIS);

    return ERROR_SUCCESS;
}


/*******************************************************************************
* Function Name  : 
* Description    : USB设备设置端点STALL
* Input          : endp: 最高位表示输入输出	
* Output         : None
* Return         : None
*******************************************************************************/
error_t hal_usbd_endp_ack(uint8_t id, uint8_t ep, uint16_t len)
{
    uint8_t ep_addr = ep & 0x7f;
    if (USBD_ID != id) return ERROR_FAILE;
   
	if (ep & 0x80){
        SetEPTxCount(ep_addr, len);
	    SetEPTxValid(ep_addr);	
    }else{
        SetEPRxValid(ep_addr);
    }

    return ERROR_SUCCESS;
}

error_t hal_usbd_endp_nak(uint8_t id, uint8_t ep)
{
    uint8_t ep_addr = ep & 0x7f;
    if (USBD_ID != id) return ERROR_FAILE;
   
	if (ep & 0x80){
        _SetEPTxStatus(ep_addr, EP_TX_NAK);
    }else{
        _SetEPRxStatus(ep_addr, EP_RX_NAK);
    }

    return ERROR_SUCCESS;
}


error_t hal_usbd_clear_endp_stall(uint8_t id, uint8_t ep)
{
    uint8_t ep_addr = ep & 0x7f;

    if (USBD_ID != id) return ERROR_FAILE;

    if (ep & 0x80) {
        ClearDTOG_TX(ep_addr);
        _SetEPTxStatus(ep_addr, EP_TX_VALID);
    } else {
        if (0 == ep_addr) {
            SetEPRxCount(ep_addr, USBD_ENDP0_MTU);
            _SetEPRxStatus(ep_addr, EP_RX_VALID);
        } else {
            ClearDTOG_RX(ep_addr);
            _SetEPRxStatus(ep_addr, EP_RX_VALID);
        }
    }

    return ERROR_SUCCESS;
}
error_t hal_usbd_endp_stall(uint8_t id, uint8_t ep)
{
    uint8_t ep_addr = ep & 0x7f;

    if (USBD_ID != id) return ERROR_FAILE;

    if (0 == ep_addr) {
        _SetEPRxTxStatus(ENDP0, EP_RX_STALL, EP_TX_STALL);
    } else {
        if (ep & 0x80) {
            _SetEPTxStatus(ep_addr, EP_TX_STALL);
        } else {
            _SetEPRxStatus(ep_addr, EP_RX_STALL);
        }
    }

    return ERROR_SUCCESS;
}


uint8_t* hal_usbd_get_endp_buffer(uint8_t id, uint8_t ep)
{
    uint8_t ep_addr = ep & 0x7f;
    uint8_t *pbuf = NULL;

    if (USBD_ID != id) return NULL;

    if(0X80 & ep){
        pbuf = (uint8_t *)(GetEPTxAddr(ep_addr) * 2 + PMAAddr);
    }else{
        pbuf = (uint8_t *)(GetEPRxAddr(ep_addr) * 2 + PMAAddr);
    }

    return pbuf;
}



/*******************************************************************
** Parameters:
** Returns:
** Description: 注意: 端点0 发送需要处理usbd_req_t
*******************************************************************/
error_t hal_usbd_in(uint8_t id, uint8_t ep, uint8_t* buf, uint16_t len)
{
    error_t err = ERROR_FAILE;
    uint8_t ep_addr = ep & 0x7f;
    uint16_t send_len;

    if (USBD_ID != id) return ERROR_FAILE;

    if(0 == ep_addr){
        usbd_req_t *preq = usbd_get_req(id);

        if(preq->setup_index <= preq->setup_len){
            send_len = preq->setup_len - preq->setup_index;
            send_len = (send_len >= USBD_ENDP0_MTU) ? USBD_ENDP0_MTU : send_len; //本次传输长度
            UserToPMABufferCopy((void*)(preq->setup_buf+preq->setup_index), GetEPTxAddr(ENDP0), send_len);               //加载上传数据	
            preq->setup_index += send_len;

            err = hal_usbd_endp_ack(id, ep, send_len);

            if(preq->setup_index == preq->setup_len){
                if(USBD_ENDP0_MTU != send_len){             //判断发送最后一包数据
                    usbd_free_setup_buffer(preq);           //发送完成释放内存
                    hal_usbd_endp_ack(id, 0x00, 0);         //设置RX ACK
                }
            }
        }else{
            return ERROR_FAILE;
        }
    }else{

        UserToPMABufferCopy(buf, GetEPTxAddr(ep_addr), len);
        err = hal_usbd_endp_ack(id, ep, len);
    }

    return err;
}

error_t hal_usbd_out(uint8_t id, uint8_t ep, uint8_t* buf, uint16_t* plen)
{
    uint8_t ep_addr = ep & 0x7f;
    if (USBD_ID != id) return ERROR_FAILE;

    if(NULL != buf) PMAToUserBufferCopy(buf, GetEPRxAddr(ep_addr), *plen);
    hal_usbd_endp_ack(id, ep, 0);         //设置RX ACK

    return ERROR_SUCCESS;
}
error_t hal_usbd_reset(uint8_t id)
{
    return hal_usbd_init(id);
}
error_t hal_usbd_set_address(uint8_t id, uint8_t address)
{
    uint32_t i;
    if (USBD_ID != id) return ERROR_FAILE;

    for (i = 0; i < 8; i++){			//必须设置全部地址防止出错
        _SetEPAddress((uint8_t)i, (uint8_t)i);
    } 
	
    _SetDADDR(address | DADDR_EF); 

    return ERROR_SUCCESS;
}



error_t hal_usbd_init(uint8_t id)
{
    uint8_t	i;

    if (USBD_ID != id) return ERROR_FAILE;

    usb_dma_address = ENDP0_TXADDR + USBD_ENDP0_MTU;                    //clear dma address 

    Set_USBConfig();  
    //usb init
    PowerOn();

    for (i=0;i<USBD_ENDP_NUM;i++){
        _SetENDPOINT(i,_GetENDPOINT(i) & 0x7F7F & EPREG_MASK);//all clear
        SetEPTxCount(i,0);		//clean tx len
    }
    _SetISTR((uint16_t)0x00FF);//all clear
    USB_SIL_Init();
    USB_Port_Set(DISABLE, DISABLE);		
	delay_ms(10);
	USB_Port_Set(ENABLE, ENABLE);	
 	USB_Interrupts_Config();  


    return ERROR_SUCCESS;
}
error_t hal_usbd_deinit(uint8_t id)
{
    if (USBD_ID != id) return ERROR_FAILE;

    USB_Port_Set(DISABLE, DISABLE);

    return ERROR_SUCCESS;
}
#endif

