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


#ifndef _hal_usbd_h
#define _hal_usbd_h
#include "emf_typedef.h" 
#include "ch32f10x_usb.h"
#include "ch32f10x_rcc.h"
#include "hw_config.h"


#ifdef __cplusplus
extern "C" {
#endif


/******************************************************************************************************
** Defined
*******************************************************************************************************/
//for regest sync
#ifndef UEP0_CTRL
#define UEP0_CTRL	R8_UEP0_CTRL
#define UEP1_CTRL	R8_UEP1_CTRL
#define UEP2_CTRL	R8_UEP2_CTRL
#define UEP3_CTRL	R8_UEP3_CTRL
#define UEP4_CTRL	R8_UEP4_CTRL
#define UEP4_1_MOD	R8_UEP4_1_MOD
#define UEP2_3_MOD	R8_UEP2_3_MOD
#define bUEP1_BUF_MOD	RB_UEP1_BUF_MOD	
#define bUEP2_BUF_MOD	RB_UEP2_BUF_MOD 
#define bUEP3_BUF_MOD	RB_UEP3_BUF_MOD	
#define bUEP_R_TOG        RB_UEP_R_TOG      // expected data toggle flag of USB endpoint X receiving (OUT): 0=DATA0, 1=DATA1
#define bUEP_T_TOG        RB_UEP_T_TOG      // prepared data toggle flag of USB endpoint X transmittal (IN): 0=DATA0, 1=DATA1
#define bUEP_AUTO_TOG     RB_UEP_AUTO_TOG      // enable automatic toggle after successful transfer completion on endpoint 1/2/3: 0=manual toggle, 1=automatic toggle
#define bUEP_R_RES1       RB_UEP_R_RES1      // handshake response type high bit for USB endpoint X receiving (OUT)
#define bUEP_R_RES0       RB_UEP_R_RES0      // handshake response type low bit for USB endpoint X receiving (OUT)
#define bUEP1_TX_EN		RB_UEP1_TX_EN
#define bUEP2_TX_EN		RB_UEP2_TX_EN	
#define bUEP3_TX_EN		RB_UEP3_TX_EN
#define bUEP4_TX_EN		RB_UEP4_TX_EN	
#define bUEP1_RX_EN		RB_UEP1_RX_EN	
#define bUEP2_RX_EN		RB_UEP2_RX_EN
#define bUEP3_RX_EN		RB_UEP3_RX_EN	
#define bUEP4_RX_EN		RB_UEP4_RX_EN	
#define USB_RX_LEN		R8_USB_RX_LEN
#define UEP0_T_LEN      R8_UEP0_T_LEN        // endpoint 0 transmittal length
#define UEP1_T_LEN		R8_UEP1_T_LEN
#define UEP2_T_LEN		R8_UEP2_T_LEN
#define UEP3_T_LEN		R8_UEP3_T_LEN
#define UEP4_T_LEN		R8_UEP4_T_LEN
#define USB_INT_ST        R8_USB_INT_ST         // ReadOnly: USB interrupt status
#define bUIS_IS_NAK       RB_UIS_IS_NAK      // ReadOnly: indicate current USB transfer is NAK received for USB device mode
#define bUIS_TOG_OK       RB_UIS_TOG_OK      // ReadOnly: indicate current USB transfer toggle is OK
#define bUIS_TOKEN1       RB_UIS_TOKEN1      // ReadOnly: current token PID code bit 1 received for USB device mode
#define bUIS_TOKEN0       RB_UIS_TOKEN0      // ReadOnly: current token PID code bit 0 received for USB device mode
#define	USB_MIS_ST		R8_USB_MIS_ST
#define USB_INT_EN      R8_USB_INT_EN         // USB interrupt enable
#define USB_INT_FG      R8_USB_INT_FG         // USB interrupt flag	
#define U_IS_NAK       (R8_USB_INT_FG&RB_U_IS_NAK) // ReadOnly: indicate current USB transfer is NAK received
#define U_TOG_OK       (R8_USB_INT_FG&RB_U_TOG_OK)
#define U_SIE_FREE     (R8_USB_INT_FG&RB_U_SIE_FREE) // ReadOnly: indicate USB SIE free status
#define UIF_FIFO_OV    (R8_USB_INT_FG&RB_UIF_FIFO_OV) // FIFO overflow interrupt flag for USB, direct bit address clear or write 1 to clear
#define UIF_HST_SOF    (R8_USB_INT_FG&RB_UIF_HST_SOF) // host SOF timer interrupt flag for USB host, direct bit address clear or write 1 to clear
#define UIF_SUSPEND    (R8_USB_INT_FG&RB_UIF_SUSPEND) // USB suspend or resume event interrupt flag, direct bit address clear or write 1 to clear
#define UIF_TRANSFER   (R8_USB_INT_FG&RB_UIF_TRANSFER) // USB transfer completion interrupt flag, direct bit address clear or write 1 to clear
#define UIF_DETECT     (R8_USB_INT_FG&RB_UIF_DETECT) // device detected event interrupt flag for USB host mode, direct bit address clear or write 1 to clear
#define UIF_BUS_RST    (R8_USB_INT_FG&RB_UIF_BUS_RST) // bus reset event interrupt flag for USB device mode, direct bit address clear or write 1 to clear


#define USB_DEV_AD        R8_USB_DEV_AD        // USB device address, lower 7 bits for USB device address
#define bUDA_GP_BIT       RB_UDA_GP_BIT      // general purpose bit
#define bUIE_HST_SOF      RB_UIE_HST_SOF

#define bUIE_DEV_SOF      RB_UIE_DEV_SOF
#define bUIE_DEV_NAK      RB_UIE_DEV_NAK
#define bUIE_FIFO_OV      RB_UIE_FIFO_OV
#define bUIE_HST_SOF      RB_UIE_HST_SOF      // enable interrupt for host SOF timer action for USB host mode
#define bUIE_SUSPEND      RB_UIE_SUSPEND      // enable interrupt for USB suspend or resume event
#define bUIE_TRANSFER     RB_UIE_TRANSFER      // enable interrupt for USB transfer completion
#define bUIE_DETECT       RB_UIE_DETECT      // enable interrupt for USB device detected event for USB host mode
#define bUIE_BUS_RST      RB_UIE_BUS_RST      // enable interrupt for USB bus reset event for USB device mode

#define bUMS_BUS_RESET    RB_UMS_BUS_RESET      // ReadOnly: indicate USB bus reset status
#define bUMS_SUSPEND      RB_UMS_SUSPEND      // ReadOnly: indicate USB suspend status
#define bUMS_DM_LEVEL     RB_UMS_DM_LEVEL      // ReadOnly: indicate UDM level saved at device attached to USB host
#define bUMS_DEV_ATTACH   RB_UMS_DEV_ATTACH      // ReadOnly: indicate device attached status on USB host
#define bUMS_SOF_PRES     RB_UMS_SOF_PRES      // ReadOnly: indicate host SOF timer presage status
#define bUMS_SOF_ACT      RB_UMS_SOF_ACT      // ReadOnly: indicate host SOF timer action status for USB host
#define bUMS_SIE_FREE     RB_UMS_SIE_FREE      // ReadOnly: indicate USB SIE free status
#define bUMS_R_FIFO_RDY   RB_UMS_R_FIFO_RDY      // ReadOnly: indicate USB receiving FIFO ready status (not empty)
#define bUH_R_TOG         0x80      // expected data toggle flag of host receiving (IN): 0=DATA0, 1=DATA1
#define bUH_R_AUTO_TOG    RB_UH_R_AUTO_TOG      // enable automatic toggle after successful transfer completion: 0=manual toggle, 1=automatic toggle
#define bUH_R_RES         RB_UH_R_RES      // prepared handshake response type for host receiving (IN): 0=ACK (ready), 1=no response, time out to device, for isochronous transactions
#define bUCC_PD_MOD       RB_UTCC_GP_BIT      // USB UCC1/UCC2 power delivery BMC output mode enable: 0=disable, 1=enable PD output mode
#define bUCC2_PD_EN       RB_UCC2_PD_EN      // USB UCC2 5.1K pulldown resistance: 0=disable, 1=enable pulldown
#define bUCC2_PU1_EN      RB_UCC2_PU1_EN      // USB UCC2 pullup resistance control high bit
#define bUCC2_PU0_EN      RB_UCC2_PU0_EN      // USB UCC2 pullup resistance control low bit
#define bVBUS_PD_EN       RB_VBUS_PD_EN      // USB VBUS 10K pulldown resistance: 0=disable, 1=enable pullup
#define bUCC1_PD_EN       RB_UCC1_PD_EN      // USB UCC1 5.1K pulldown resistance: 0=disable, 1=enable pulldown
#define bUCC1_PU1_EN      RB_UCC1_PU1_EN      // USB UCC1 pullup resistance control high bit
#define bUCC1_PU0_EN      RB_UCC1_PU0_EN      // USB UCC1 pullup resistance control low bit
#define UDEV_CTRL         R8_UDEV_CTRL         // USB device physical port control
#define bUD_PD_DIS        RB_UD_PD_DIS      // disable USB UDP/UDM pulldown resistance: 0=enable pulldown, 1=disable
#define bUD_DP_PIN        RB_UD_DP_PIN      // ReadOnly: indicate current UDP pin level
#define bUD_DM_PIN        RB_UD_DM_PIN      // ReadOnly: indicate current UDM pin level
#define bUD_LOW_SPEED     RB_UD_LOW_SPEED      // enable USB physical port low speed: 0=full speed, 1=low speed
#define bUD_GP_BIT        RB_UD_GP_BIT      // general purpose bit
#define bUD_PORT_EN       RB_UD_PORT_EN      // enable USB physical port I/O: 0=disable, 1=enable


#define UHOST_CTRL        R8_UHOST_CTRL
#define bUH_PD_DIS        RB_UH_PD_DIS      // disable USB UDP/UDM pulldown resistance: 0=enable pulldown, 1=disable
#define bUH_DP_PIN        RB_UH_DP_PIN      // ReadOnly: indicate current UDP pin level
#define bUH_DM_PIN        RB_UH_DM_PIN      // ReadOnly: indicate current UDM pin level
#define bUH_LOW_SPEED     RB_UH_LOW_SPEED      // enable USB port low speed: 0=full speed, 1=low speed
#define bUH_BUS_RESET     RB_UH_BUS_RESET      // control USB bus reset: 0=normal, 1=force bus reset
#define bUH_PORT_EN       RB_UH_PORT_EN      // enable USB port: 0=disable, 1=enable port, automatic disabled if USB device detached
#define UH_TX_LEN         UEP3_T_LEN
#define UH_EP_MOD         R8_UH_EP_MOD
#define bUH_EP_TX_EN      RB_UH_EP_TX_EN      // enable USB host OUT endpoint transmittal
#define bUH_EP_TBUF_MOD   RB_UH_EP_TBUF_MOD      // buffer mode of USB host OUT endpoint
#define bUH_EP_RX_EN      RB_UH_EP_RX_EN      // enable USB host IN endpoint receiving
#define bUH_EP_RBUF_MOD   RB_UH_EP_RBUF_MOD      // buffer mode of USB host IN endpoint
#define UH_RX_DMA         R16_UH_RX_DMA
#define UH_RX_DMA_L       R16_UH_TX_DMA
#define UH_EP_PID         R8_UH_EP_PID
#define UH_TX_CTRL        UEP3_CTRL
#define bUH_T_TOG         RB_UH_T_TOG      // prepared data toggle flag of host transmittal (SETUP/OUT): 0=DATA0, 1=DATA1
#define bUH_T_AUTO_TOG    RB_UH_T_AUTO_TOG      // enable automatic toggle after successful transfer completion: 0=manual toggle, 1=automatic toggle
#define bUH_T_RES         RB_UH_T_RES      // expected handshake response type for host transmittal (SETUP/OUT): 0=ACK (ready), 1=no response, time out from device, for isochronous transactions
#define UH_SETUP          R8_UH_SETUP
#define bUH_PRE_PID_EN    RB_UH_PRE_PID_EN      // USB host PRE PID enable for low speed device via hub
#define bUH_SOF_EN        RB_UH_SOF_EN      // USB host automatic SOF enable
#define UH_RX_CTRL        UEP2_CTRL
#define bUEP_T_RES1       RB_UEP_T_RES1      // handshake response type high bit for USB endpoint X transmittal (IN)
#define bUEP_T_RES0       RB_UEP_T_RES0      // handshake response type low bit for USB endpoint X transmittal (IN)
#define bUC_HOST_MODE     RB_UC_HOST_MODE      // enable USB host mode: 0=device mode, 1=host mode
#define bUC_LOW_SPEED     RB_UC_LOW_SPEED      // enable USB low speed: 0=full speed, 1=low speed
#define bUC_DEV_PU_EN     RB_UC_DEV_PU_EN      // USB device enable and internal pullup resistance enable
#define bUC_SYS_CTRL1     RB_UC_SYS_CTRL1      // USB system control high bit
#define bUC_SYS_CTRL0     RB_UC_SYS_CTRL0      // USB system control low bit
#define bUC_INT_BUSY      RB_UC_INT_BUSY      // enable automatic responding busy for device mode or automatic pause for host mode during interrupt flag UIF_TRANSFER valid
#define bUC_RESET_SIE     RB_UC_RESET_SIE      // force reset USB SIE, need software clear
#define bUC_CLR_ALL       RB_UC_CLR_ALL      // force clear FIFO and count of USB
#define bUC_DMA_EN        RB_UC_DMA_EN      // DMA enable and DMA interrupt enable for USB
#endif

#if (BIT(1) == API_USBD_BIT_ENABLE)   
#define USBD_ENDP_NUM			8
#endif

/******************************************************************************************************
**	Parameters
*******************************************************************************************************/



/*****************************************************************************************************
**  Function
******************************************************************************************************/

#ifdef __cplusplus
}
#endif
#endif





