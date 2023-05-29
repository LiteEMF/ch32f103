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
#include  "api/api_tick.h"

/******************************************************************************************************
** Defined
*******************************************************************************************************/

/******************************************************************************************************
**	static Parameters
*******************************************************************************************************/

/******************************************************************************************************
**	public Parameters
*******************************************************************************************************/

/*****************************************************************************************************
**	static Function
******************************************************************************************************/

/*****************************************************************************************************
**  Function
******************************************************************************************************/

/*******************************************************************
** Parameters:		
** Returns:	
** Description:	MIN 410ns, io 翻转误差80ns	
*******************************************************************/
void hal_delay_ns(uint32_t ns)
{
	volatile uint32_t t = ns;
	if(ns <= 410){			//判断 + return 410 ns
        return ;
    }else{					//判断 + 运算 + return = 690
        t = (ns - 410) / 70;
    }

	while ( t ) {			//一个循环约70ns	
		-- t;
	}
}
void hal_delay_us(uint32_t us)
{
	hal_delay_ns(1000*us);
}
void hal_delay_ms(uint32_t ms)
{
	hal_delay_us(1000*ms);
}
void hal_tick_init(void)
{	
}







