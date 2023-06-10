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
#if GAMEPAD1	
#include "app/emf.h"
#include "app/app_km.h"

#include "api/api_log.h"

/******************************************************************************************************
** Defined
*******************************************************************************************************/

/******************************************************************************************************
**	public Parameters
*******************************************************************************************************/


/******************************************************************************************************
**	static Parameters
*******************************************************************************************************/

/*****************************************************************************************************
**	static Function
******************************************************************************************************/

/*****************************************************************************************************
**  Function
******************************************************************************************************/


const uint8_t led_channel[] = {0, 1, 2, 3, 4, 5, 10, 11, 12};
bool rgb_driver_show(uint8_t* frame, uint8_t size)
{
	bool ret = false;
    uint8_t brightness;
	uint8_t i;
    
    // logd("show:");dumpd(frame,size);
    // emf_mem_stats();
	for(i=0; i<size; i++){
        brightness = remap(frame[i], 0, 255, 0, 63);
        #ifdef HW_SPI_HOST_MAP
        ret = api_spi_host_write(0,led_channel[i], &brightness, 1);
        #endif
	}

	return ret;
}
bool rgb_driver_init(void)
{
	return true;
}
bool rgb_driver_deinit(void)
{
	return true;
}



void hw_user_vender_init(void)
{
    uint8_t id;

	for(id=0; id<USBD_NUM; id++){
		m_usbd_types[id] = USBD_TYPE_SUPPORT;
        m_usbd_hid_types[id] = USBD_HID_SUPPORT;
	}

    
    

    logd("call hw_user_vender_init ok\n" );

}
void user_vender_init(void)	
{
    uint8_t i;
    logd("call user_vender_init ok\n" );


    api_gpio_dir(PB_00, PIN_OUT,PIN_PULLNONE);
    // app_rumble_set_duty(RUMBLE_L, 0X80, 1000);
    // app_rumble_set_duty(RUMBLE_R, 0X80, 1000);
	
	#if APP_RGB_ENABLE
    for(i=0; i<APP_RGB_NUMS; i++){
        app_rgb_set_blink(i, Color_White, BLINK_SLOW);
    }
	#endif
	
}
void user_vender_deinit(void)			//关机前deinit
{
}

void user_vender_handler(void)
{
	//rx uart
    static timer_t timer;
    app_fifo_t *fifop = api_uart_get_rx_fifo(1);
    uint8_t c;
	command_rx_t rx;
	static uint8_t s_cmd_buf[UART_CMD_MTU];
	static uint8_t s_cmd_len = 0;
    
	memset(&rx, 0, sizeof(rx));

    while(ERROR_SUCCESS == app_fifo_get(fifop, &c)){
        // logd("%x",c);
		// if(api_command_rx_byte(&rx, UART_CMD_MTU, c, s_cmd_buf, &s_cmd_len)){
		// 	logd("uart cmd %d:",rx.len); dumpd(rx.pcmd, rx.len);
		// 	command_rx_free(&rx);
		// }
    }

    // // adc test 
    // static timer_t adc_times = 0;
    // if(m_systick - adc_times >= 10){
    //     adc_times = m_systick;
    //     logd("adc:%4d,%4d,%4d,%4d,%4d,%4d\n",
	// 	api_adc_value(0),api_adc_value(1),api_adc_value(2),api_adc_value(3),api_adc_value(4),api_adc_value(5));
    // }


    //use test

    if(m_systick - timer >= 3000){
        timer = m_systick;

        uint8_t usb_id = 1;

        usbd_dev_t *pdev = usbd_get_dev(usb_id);

        logd("pdev->ready=%d\n",pdev->ready);
        if(pdev->ready){
            static kb_t kb={KB_REPORT_ID,0};
            static mouse_t mouse={MOUSE_REPORT_ID,0};

            if(kb.key[0]){
                kb.key[0] = 0;
            }else{
                kb.key[0] = KB_CAP_LOCK;
            }
            trp_handle_t handle={TR_USBD, usb_id, U16(DEF_DEV_TYPE_HID,DEF_HID_TYPE_KB)};
            api_transport_tx(&handle,&kb,sizeof(kb));


            if(mouse.x >= 0){
                mouse.x = -10;
            }else{
                mouse.x = 10;
            }
            handle.index = U16(DEF_DEV_TYPE_HID,DEF_HID_TYPE_MOUSE);
            api_transport_tx(&handle,&mouse,sizeof(mouse));
        }
    }

}

#endif
