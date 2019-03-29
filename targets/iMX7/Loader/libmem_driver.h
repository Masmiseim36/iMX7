#ifndef _LIBMEM_DRIVER_H_
#define _LIBMEM_DRIVER_H_

#include "libmem.h"
#include "fsl_device_registers.h"

enum FlashParmeter
{
	FLASH_PAGE_SIZE     = 256U,
	FLASH_SECTORE_SIZE  = 4096U,
	FLASH_SIZE          = 0x00800000U,
	FLASH_START_ADDRESS = 0xC0000000	// 0x40000000
};


int Libmem_InitializeDriver (void);
const char * const Libmem_GetErrorString (int Error);

#endif	// _LIBMEM_DRIVER_H_
