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
#include "utils/mem/emf_mem.h"
#include "FreeRTOS.h"


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
void* hal_malloc(uint32_t size)
{
	return pvPortMalloc(size);
}

/*******************************************************************
** Parameters:		
** Returns:	
** Description:		
*******************************************************************/
void hal_free(void* p)
{
	vPortFree(p);
}

/*******************************************************************
** Parameters:		
** Returns:	
** Description:		
*******************************************************************/
void hal_mem_stats(void)
{
	HeapStats_t xHeapStats;

	vPortGetHeapStats(&xHeapStats);

	logd("emf mem stats heap free size=%d\n",xHeapStats.xAvailableHeapSpaceInBytes);
	logd("	free size: largest=%d,smallest=%d, num=%d\n",xHeapStats.xSizeOfLargestFreeBlockInBytes,xHeapStats.xSizeOfSmallestFreeBlockInBytes,xHeapStats.xNumberOfFreeBlocks);
	logd("	malloc times=%d, free times=%d\n",xHeapStats.xNumberOfSuccessfulAllocations,xHeapStats.xNumberOfSuccessfulFrees);
}
