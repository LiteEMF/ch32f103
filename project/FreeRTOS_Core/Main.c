/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2019/10/15
 * Description        : Main program body.
 *********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "debug.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "LiteEMF/app/emf.h"

#include "api/api_log.h"


/* Global define */
#define MAIN_TASK_PRIO     3
#define MAIN_STK_SIZE      256

/* Global Variable */
TaskHandle_t MainTask_Handler;



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

    logd("call hw_user_vender_init ok\n" );

}
void user_vender_init(void)	
{
    uint8_t i;
    logd("call user_vender_init ok\n" );


    api_gpio_dir(PB_00, PIN_OUT,PIN_PULLNONE);
    // app_rumble_set_duty(RUMBLE_L, 0X80, 1000);
    // app_rumble_set_duty(RUMBLE_R, 0X80, 1000);

    for(i=0; i<APP_RGB_NUMS; i++){
        app_rgb_set_blink(i, Color_White, BLINK_SLOW);
    }

}
void user_vender_deinit(void)			//关机前deinit
{
}

void user_vender_handler(void)
{
	//rx uart
    app_fifo_t *fifop = api_uart_get_rx_fifo(1);
    uint8_t c;
	command_rx_t rx;
	static uint8_t s_cmd_buf[UART_CMD_MTU];
	static uint8_t s_cmd_len = 0;
    
	memset(&rx, 0, sizeof(rx));

    while(ERROR_SUCCESS == app_fifo_get(fifop, &c)){
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

}

void timerCallback(TimerHandle_t xTimer)
{
    // static bool on;
    // api_gpio_out(PB_00,on);on = !on;

    vTaskResume(MainTask_Handler);
}

void vApplicationIdleHook(void) 
{
    api_wdt_feed();
}

void fifo_state(void)
{
    app_fifo_t* fifop = api_uart_get_rx_fifo(1);

    logd("fifo:%d %d %d %d\n",
        fifop->buf_size_max,
        fifop->read_pos,
        fifop->write_pos,
        fifop->fifo_stu);
}

/*********************************************************************
 * @fn      task1_task
 *
 * @brief   task1 program.
 *
 * @param  *pvParameters - Parameters point of task1
 *
 * @return  none
 */
void main_task(void *pvParameters)
{
    while(1)
    {
        api_gpio_out(PB_00,1);
        m_task_tick10us += 100;
        emf_handler(0);
        api_gpio_out(PB_00,0);
        vTaskSuspend(NULL); // 挂起任务
    }
}




/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main( void )
{
    emf_mem_init();
	#ifdef HW_UART_MAP
	api_uart_init(UART_DEBUG_ID);			
	#endif

    RCC_APB1PeriphClockCmd( RCC_APB1Periph_PWR, ENABLE );
    logd("CSR=%x",PWR->CSR);
    if( PWR_GetFlagStatus( PWR_FLAG_WU ) == SET ){
        logd( "\n Standby wake up reset \n" );
    }else{
        logd( "\n Power on reset \n" );
    }
    logd("CSR=%x",PWR->CSR);

    logi(" \n ");
    logi("\n===================================================\n" );
    logi( "SystemClk:%d\n", SystemCoreClock );
    logi("FreeRTOS Kernel Version:%s\n",tskKERNEL_VERSION_NUMBER);

    emf_api_init();
	emf_init();

    logd("add task..\n");
    /* create two task */
    xTaskCreate((TaskFunction_t )main_task,
                        (const char*    )"TaskMain",
                        (uint16_t       )MAIN_STK_SIZE,
                        (void*          )NULL,
                        (UBaseType_t    )MAIN_TASK_PRIO,
                        (TaskHandle_t*  )&MainTask_Handler);


    // 创建定时器
    TimerHandle_t timerHandle = xTimerCreate("Timer", pdMS_TO_TICKS(1), pdTRUE, NULL, timerCallback);
    // 启动定时器
    xTimerStart(timerHandle, 0);

    vTaskStartScheduler();

    while(1)
    {
        logi("shouldn't run at here!!\n");
    }
}



