#include <libmem.h>
#include <libmem_loader.h>
#include <libmem_flm_driver.h>
#include <stdio.h>

#include "board.h"
#include "fsl_qspi_edma.h"
#include "fsl_dmamux.h"
#include "fsl_qspi.h"
#include "fsl_common.h"
#include "fsl_clock.h"
#include "pin_mux.h"
#include "libmem_driver.h"

extern uint8_t __SRAM_segment_start__;
extern uint8_t __SRAM_segment_used_end__;
extern uint8_t __SRAM_segment_end__;


int main (uint32_t flags, uint32_t param)
{
	uint32_t fosc = flags & LIBMEM_RPC_LOADER_FLAG_PARAM ? param : 100000;

	BOARD_InitPins ();
	BOARD_BootClockRUN ();
	CLOCK_SetIpSrcDiv (kCLOCK_Qspi, kCLOCK_IpSrcSystem, 3, 0);

    int res = Libmem_InitializeDriver ();	// Register iMX7 internal FLASH driver

	#ifdef DEBUG
	{
		// Tetscode
		static uint32_t buffer[512];
		memset (buffer, 0, sizeof(buffer));
		for (int i=0; i<sizeof(buffer)/sizeof(buffer[0]); i++)
			buffer[i] = (uint32_t)(((uint32_t *)buffer) + i);

		uint8_t *erase_start;
		size_t erase_size;
		res = libmem_erase ((uint8_t *)FLASH_START_ADDRESS, sizeof(buffer), &erase_start, &erase_size);
		res = libmem_write ((uint8_t *)FLASH_START_ADDRESS, (uint8_t *)buffer, sizeof(buffer));
		res = libmem_flush ();
	}
	#endif

	// Start loader
	if (res == LIBMEM_STATUS_SUCCESS)
		res = libmem_rpc_loader_start (&__SRAM_segment_used_end__, &__SRAM_segment_end__ - 1);

	// Terminate loader and return error String if an Error occured
    if (res == LIBMEM_STATUS_SUCCESS)
		libmem_rpc_loader_exit (res, NULL);
	else
	{
		char ErrorString[64];
		sprintf (ErrorString, "Error %s occurred\r\n", Libmem_GetErrorString (res));
		libmem_rpc_loader_exit (res, ErrorString);
	}

	return 0;
}
