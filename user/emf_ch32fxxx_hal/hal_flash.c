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
#include  "api/api_flash.h"

/******************************************************************************************************
** Defined
*******************************************************************************************************/

/*****************************************************************************************************
**  Function
******************************************************************************************************/

/*******************************************************************
** Parameters: offset must mul 0x80 
** Returns:	
** Description:	Program 128Byte
*******************************************************************/
bool hal_flash_write(uint16_t offset,uint8_t *buf,uint16_t len)
{
	uint8_t i;
	uint32_t addr;
	uint32_t dat[4];

	if(offset & 0XFF80) return false;
	addr = API_FLASH_ADDRESS+offset;
	
	FLASH_BufReset();

	for(i=0; i<len; i+=0x10){

		if(0X10 >= len - i){
			memcpy(dat, buf+i, 0x10);
		}else{
			memset(dat, 0xFF, 0x10);
			memcpy(dat, buf+i, len - i);
		}
		FLASH_BufLoad(addr + i, dat[0], dat[1], dat[2], dat[3]);
	}
	
	FLASH_ProgramPage_Fast(addr);

	return true;
}

/*******************************************************************
** Parameters: offset must mul 0x04
** Returns:	
** Description:	read world
*******************************************************************/
bool hal_flash_read(uint16_t offset,uint8_t *buf,uint16_t len)
{
	uint32_t i;
	uint32_t read_buf;

	if(offset & 0XFFFC) return false;

	for(i=0; i<len; i+=4){
		read_buf =  *( uint32_t * )( (API_FLASH_ADDRESS+offset) + i );
		memcpy(buf+i, &read_buf, MIN(4, len - i));
	}

	return true;
}
bool hal_flash_erase(uint16_t offset)
{
	FLASH_ErasePage_Fast( API_FLASH_ADDRESS+offset );
	return true;
}
bool hal_flash_init(void)
{
	return true;
}










