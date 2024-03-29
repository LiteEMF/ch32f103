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


#ifndef _hal_flash_h
#define _hal_flash_h
#include "emf_typedef.h" 

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************************************************************
** Defined
*******************************************************************************************************/
#ifndef API_FLASH_PAGE_NUM
#define API_FLASH_PAGE_NUM		2
#endif
#ifndef API_FLASH_PAGE_SIZE
#define API_FLASH_PAGE_SIZE		0X80
#endif
#ifndef API_FLASH_ADDRESS
#define API_FLASH_ADDRESS		(PAGE_WRITE_END_ADDR - API_FLASH_PAGE_SIZE*API_FLASH_PAGE_NUM)
#endif

#define PAGE_WRITE_END_ADDR    ((uint32_t)0x08010000) /* End at 63K */
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





