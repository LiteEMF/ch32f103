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
        // api_gpio_out(PB_00,1);

        m_task_tick10us += 100;
        emf_handler(0);

        // api_gpio_out(PB_00,0);
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



