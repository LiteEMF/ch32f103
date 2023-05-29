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
#if API_WDT_ENABLE

#include  "api/api_wdt.h"

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
//hal

void hal_wdt_feed(void)
{
	IWDG_ReloadCounter();	//Feed dog
}

/*********************************************************************
 * @fn      IWDG_Init
 *
 * @brief   Initializes IWDG.
 *
 * @param   IWDG_Prescaler: specifies the IWDG Prescaler value.
 *            IWDG_Prescaler_4 - IWDG prescaler set to 4.
 *            IWDG_Prescaler_8 - IWDG prescaler set to 8.
 *            IWDG_Prescaler_16 - IWDG prescaler set to 16.
 *            IWDG_Prescaler_32 - IWDG prescaler set to 32.
 *            IWDG_Prescaler_64 - IWDG prescaler set to 64.
 *            IWDG_Prescaler_128 - IWDG prescaler set to 128.
 *            IWDG_Prescaler_256 - IWDG prescaler set to 256.
 *          Reload: specifies the IWDG Reload value.
 *            This parameter must be a number between 0 and 0x0FFF.
 */
bool hal_wdt_init(uint32_t ms)
{
	IWDG_WriteAccessCmd( IWDG_WriteAccess_Enable );

	if(API_WDT_TIME > 3200){
		IWDG_SetPrescaler( IWDG_Prescaler_128 );				// 40Khz to 1.25Khz
		IWDG_SetReload( API_WDT_TIME * 312UL / 1000 );			//max 0X0FFF
	}else{
		IWDG_SetPrescaler( IWDG_Prescaler_32 );	// 40Khz to 1.25Khz
		IWDG_SetReload( API_WDT_TIME * 1250UL / 1000 );		//max 0X0FFF
	}

    IWDG_ReloadCounter();
    IWDG_Enable();

	return true;
}
bool hal_wdt_deinit(void)
{
	return false;
}

#endif



