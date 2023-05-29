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
#include  "api/api_pm.h"

#include  "api/api_log.h"
/******************************************************************************************************
** Defined
*******************************************************************************************************/

/*****************************************************************************************************
**  Function
******************************************************************************************************/

/*******************************************************************
** Parameters:		
** Returns:	
** Description:		
*******************************************************************/
void hal_weakup_init(void)
{
}

pm_reson_t hal_get_reset_reson(void)
{
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_PWR, ENABLE );
    if( PWR_GetFlagStatus( PWR_FLAG_WU ) == SET ){
        logd( "\n Standby wake up reset \n" );
		return PM_RESON_SOFT;
    }else{
        logd( "\n Power on reset \n" );
		return PM_RESON_POR;
    }
}
void hal_boot(uint8_t index)
{ 
    NVIC_SystemReset();
}
void hal_reset(void)
{
	NVIC_SystemReset();
}
void hal_sleep(void)
{
	PWR_EnterSTOPMode(PWR_Regulator_LowPower,PWR_STOPEntry_WFI);
}










