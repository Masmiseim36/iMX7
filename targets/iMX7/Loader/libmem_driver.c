#include "libmem_driver.h"
#include "fsl_device_registers.h"
#include "fsl_qspi_edma.h"
#include "fsl_dmamux.h"
#include "fsl_qspi.h"
#include "fsl_common.h"
#include "fsl_clock.h"
#include <string.h>

static libmem_driver_paged_write_ctrlblk_t paged_write_ctrlblk;
static uint32_t s_clk;

static void callback (QuadSPI_Type *base, qspi_edma_handle_t *handle, status_t status, void *userData);
volatile bool isFinished = false;
static libmem_driver_handle_t FlashHandle;
static qspi_edma_handle_t qspiHandle = {0};

// Check if serial flash erase or program finished.
void CheckIfFinished (QuadSPI_Type *qspi)
{
	uint32_t val = 0;
	// Check WIP bit
	do
	{
		while (QSPI_GetStatusFlags (qspi) & kQSPI_Busy)
			;
		QSPI_ClearFifo             (qspi, kQSPI_RxFifo);
		QSPI_ExecuteIPCommand      (qspi, 12U);
		while (QSPI_GetStatusFlags (qspi) & kQSPI_Busy)
			;
		val = *(volatile uint32_t *)(FSL_FEATURE_QSPI_ARDB_BASE);
		// Clear ARDB area
		QSPI_ClearErrorFlag        (qspi, kQSPI_RxBufferDrain);
	}
	while (val & 0x1);
}

static void cmd_write_enable(void)
{
	while (QSPI_GetStatusFlags (QuadSPI0) & kQSPI_Busy)
		;
	QSPI_ExecuteIPCommand (QuadSPI0, 4U);
}

static int EraseSector (libmem_driver_handle_t *h, libmem_sector_info_t *si)
{
	while (QSPI_GetStatusFlags (QuadSPI0) & kQSPI_Busy)
		;

	QSPI_ClearFifo           (QuadSPI0, kQSPI_TxFifo);
	QSPI_SetIPCommandAddress (QuadSPI0, (uint32_t)si->start);
	cmd_write_enable         ();
	QSPI_ExecuteIPCommand    (QuadSPI0, 28U);
	CheckIfFinished          (QuadSPI0);

	return LIBMEM_STATUS_SUCCESS;
}

static int ProgramPage (libmem_driver_handle_t *h, uint8_t *dest_addr, const uint8_t *src_addr)
{
	qspi_transfer_t xfer = {0};

	xfer.data     = (uint32_t *)src_addr;
	xfer.dataSize = FLASH_PAGE_SIZE;
	while (QSPI_GetStatusFlags(QuadSPI0) & kQSPI_Busy)
		;

	QSPI_ClearFifo(QuadSPI0, kQSPI_TxFifo);
	QSPI_SetIPCommandAddress   (QuadSPI0, (uint32_t)dest_addr);
	QSPI_SetIPCommandSize      (QuadSPI0, FLASH_PAGE_SIZE);
	cmd_write_enable();
	while (QSPI_GetStatusFlags (QuadSPI0) & kQSPI_Busy)
		;

	QSPI_TransferSendEDMA (QuadSPI0, &qspiHandle, &xfer);	// Use EDMA transfer
	QSPI_ExecuteIPCommand (QuadSPI0, 16U);					// Execute the programe page command
	while (isFinished != true)								// Wait for EDMA transfer finished
		;
	CheckIfFinished       (QuadSPI0);						//  Wait until flash finished program
	while (QSPI_GetStatusFlags (QuadSPI0) & (kQSPI_Busy | kQSPI_IPAccess))
		;
	QSPI_SoftwareReset    (QuadSPI0);
	while (QSPI_GetStatusFlags (QuadSPI0) & (kQSPI_Busy | kQSPI_IPAccess))
		;

	return LIBMEM_STATUS_SUCCESS;
}

static int libmem_ProgramPage (libmem_driver_handle_t *h, uint8_t *dest, const uint8_t *src, size_t size)
{
  return libmem_driver_paged_write (h, dest, src, size, &paged_write_ctrlblk);
}

static int libmem_EraseSector(libmem_driver_handle_t *h, uint8_t *start, size_t size, uint8_t **erase_start, size_t *erase_size)
{
  return libmem_foreach_sector_in_range (h, start, size, EraseSector, erase_start, erase_size);
}

static int libmem_Flush (libmem_driver_handle_t *h)
{
	int res = libmem_driver_paged_write_flush (h, &paged_write_ctrlblk);
	return res;
}

static int libmem_Read (libmem_driver_handle_t *h, uint8_t *dest, const uint8_t *src, size_t size)
{
	if (size)
		memcpy (dest, src, size);
	return LIBMEM_STATUS_SUCCESS;
}

static uint32_t libmem_CRC32 (libmem_driver_handle_t *h, const uint8_t *start, size_t size, uint32_t crc)
{
	crc = libmem_crc32_direct (start, size, crc);
	return crc;
}

static const libmem_driver_functions_t driver_functions =
{
	libmem_ProgramPage,
	0,
	libmem_EraseSector,
	0,
	0,
	libmem_Flush
};

static const libmem_ext_driver_functions_t ext_driver_functions =
{
	0,
	libmem_Read,
	libmem_CRC32
};


uint32_t lut[FSL_FEATURE_QSPI_LUT_DEPTH] =
{
	/* Seq0 :Quad Read */
	/* CMD:        0xEB - Quad Read, Single pad */
	/* ADDR:       0x18 - 24bit address, Quad pads */
	/* DUMMY:      0x06 - 6 clock cyles, Quad pads */
	/* READ:       0x80 - Read 128 bytes, Quad pads */
	/* JUMP_ON_CS: 0 */
	[0] = 0x0A1804EB,
	[1] = 0x1E800E06,
	[2] = 0x2400,

	/* Seq1: Write Enable */
	/* CMD:      0x06 - Write Enable, Single pad */
	[4] = 0x406,

	/* Seq2: Erase All */
	/* CMD:    0x60 - Erase All chip, Single pad */
	[8] = 0x460,

	/* Seq3: Read Status */
	/* CMD:    0x05 - Read Status, single pad */
	/* READ:   0x01 - Read 1 byte */
	[12] = 0x1c010405,

	/* Seq4: Page Program */
	/* CMD:    0x02 - Page Program, Single pad */
	/* ADDR:   0x18 - 24bit address, Single pad */
	/* WRITE:  0x80 - Write 128 bytes at one pass, Single pad */
	[16] = 0x08180402,
	[17] = 0x2080,

	/* Seq5: Write Register */
	/* CMD:    0x01 - Write Status Register, single pad */
	/* WRITE:  0x01 - Write 1 byte of data, single pad */
	[20] = 0x20010401,

	/* Seq6: Read Config Register */
	/* CMD:  0x05 - Read Config register, single pad */
	/* READ: 0x01 - Read 1 byte */
	[24] = 0x1c010405,

	/* Seq7: Erase Sector */
	/* CMD:  0x20 - Sector Erase, single pad */
	/* ADDR: 0x18 - 24 bit address, single pad */
	[28] = 0x08180420,

	/* Seq8: Dummy */
	/* CMD:    0xFF - Dummy command, used to force SPI flash to exit continuous read mode */
	[32] = 0x4FF,

	/* Seq9: Fast Single read */
	/* CMD:        0x0B - Fast Read, Single Pad */
	/* ADDR:       0x18 - 24bit address, Single Pad */
	/* DUMMY:      0x08 - 8 clock cyles, Single Pad */
	/* READ:       0x80 - Read 128 bytes, Single Pad */
	/* JUMP_ON_CS: 0 */
	[36] = 0x0818040B,
	[37] = 0x1C800C08,
	[38] = 0x2400,

	/* Seq10: Fast Dual read */
	/* CMD:        0x3B - Dual Read, Single Pad */
	/* ADDR:       0x18 - 24bit address, Single Pad */
	/* DUMMY:      0x08 - 8 clock cyles, Single Pad */
	/* READ:       0x80 - Read 128 bytes, Dual pads */
	/* JUMP_ON_CS: 0 */
	[40] = 0x0818043B,
	[41] = 0x1D800C08,
	[42] = 0x2400,

	/* Match MISRA rule */
	[63] = 0
};

qspi_flash_config_t single_config =
{
	.flashA1Size = FLASH_SIZE, /* 4MB */
	.flashA2Size = 0,
	#if defined(FSL_FEATURE_QSPI_SUPPORT_PARALLEL_MODE) && (FSL_FEATURE_QSPI_SUPPORT_PARALLEL_MODE)
		.flashB1Size = FLASH_SIZE,
		.flashB2Size = 0,
	#endif
	.lookuptable =
	{
		/* Seq0 :Quad Read */
		/* CMD:        0xEB - Quad Read, Single pad */
		/* ADDR:       0x18 - 24bit address, Quad pads */
		/* DUMMY:      0x06 - 6 clock cyles, Quad pads */
		/* READ:       0x80 - Read 128 bytes, Quad pads */
		/* JUMP_ON_CS: 0 */
		[0] = 0x0A1804EB,
		[1] = 0x1E800E06,
		[2] = 0x2400,

		/* Seq1: Write Enable */
		/* CMD:      0x06 - Write Enable, Single pad */
		[4] = 0x406,

		/* Seq2: Erase All */
		/* CMD:    0x60 - Erase All chip, Single pad */
		[8] = 0x460,

		/* Seq3: Read Status */
		/* CMD:    0x05 - Read Status, single pad */
		/* READ:   0x01 - Read 1 byte */
		[12] = 0x1c010405,

		/* Seq4: Page Program */
		/* CMD:    0x02 - Page Program, Single pad */
		/* ADDR:   0x18 - 24bit address, Single pad */
		/* WRITE:  0x80 - Write 128 bytes at one pass, Single pad */
		[16] = 0x08180402,
		[17] = 0x2080,

		/* Seq5: Write Register */
		/* CMD:    0x01 - Write Status Register, single pad */
		/* WRITE:  0x01 - Write 1 byte of data, single pad */
		[20] = 0x20010401,

		/* Seq6: Read Config Register */
		/* CMD:  0x05 - Read Config register, single pad */
		/* READ: 0x01 - Read 1 byte */
		[24] = 0x1c010405,

		/* Seq7: Erase Sector */
		/* CMD:  0x20 - Sector Erase, single pad */
		/* ADDR: 0x18 - 24 bit address, single pad */
		[28] = 0x08180420,

		/* Seq8: Dummy */
		/* CMD:    0xFF - Dummy command, used to force SPI flash to exit continuous read mode */
		[32] = 0x4FF,

		/* Seq9: Fast Single read */
		/* CMD:        0x0B - Fast Read, Single Pad */
		/* ADDR:       0x18 - 24bit address, Single Pad */
		/* DUMMY:      0x08 - 8 clock cyles, Single Pad */
		/* READ:       0x80 - Read 128 bytes, Single Pad */
		/* JUMP_ON_CS: 0 */
		[36] = 0x0818040B,
		[37] = 0x1C800C08,
		[38] = 0x2400,

		/* Seq10: Fast Dual read */
		/* CMD:        0x3B - Dual Read, Single Pad */
		/* ADDR:       0x18 - 24bit address, Single Pad */
		/* DUMMY:      0x08 - 8 clock cyles, Single Pad */
		/* READ:       0x80 - Read 128 bytes, Dual pads */
		/* JUMP_ON_CS: 0 */
		[40] = 0x0818043B,
		[41] = 0x1D800C08,
		[42] = 0x2400,

		/* Match MISRA rule */
		[63] = 0
	},
	.dataHoldTime      = 0,
	.CSHoldTime        = 0,
	.CSSetupTime       = 0,
	.cloumnspace       = 0,
	.dataLearnValue    = 0,
	.endian            = kQSPI_64LittleEndian,
	.enableWordAddress = false
};

static const libmem_geometry_t geometry[] =
{
	{0x800, 0x1000},
	{0, 0} 
};


int Libmem_InitializeDriver (void)
{
	// Init DMAMUX
	DMAMUX_Init          (DMA_CH_MUX0);
	DMAMUX_SetSource     (DMA_CH_MUX0, 16, kDmaRequestMux0QSPITx);
	DMAMUX_EnableChannel (DMA_CH_MUX0, 16);

	// Init EDMA
	// edmaConfig.enableRoundRobinArbitration = false;
	// edmaConfig.enableHaltOnError = true;
	// edmaConfig.enableContinuousLinkMode = false;
	// edmaConfig.enableDebugMode = false;
    edma_config_t edmaConfig = {0};
	EDMA_GetDefaultConfig (&edmaConfig);
	EDMA_Init             (DMA0, &edmaConfig);
    static edma_handle_t    dmaHandle = {0};
	EDMA_CreateHandle     (&dmaHandle, DMA0, 16);

	//Get QSPI default settings and configure the qspi
    qspi_config_t config     = {0};
	QSPI_GetDefaultQspiConfig (&config);

	// Set AHB buffer size for reading data through AHB bus
	config.AHBbufferSize[3] = FLASH_PAGE_SIZE;
	QSPI_Init (QuadSPI0, &config, CLOCK_GetIpFreq (kCLOCK_Qspi));

	// Copy the LUT table
	memcpy(single_config.lookuptable, lut, sizeof(uint32_t) * FSL_FEATURE_QSPI_LUT_DEPTH);

	// According to serial flash feature to configure flash settings
	QSPI_SetFlashConfig (QuadSPI0, &single_config);
	QSPI_TransferTxCreateHandleEDMA (QuadSPI0, &qspiHandle, callback, NULL, &dmaHandle);

	// Enable Quad mode
	uint32_t val[4] = {0x40U, 0, 0, 0};	// 0x40U --> Enable Quad Command
	while (QSPI_GetStatusFlags(QuadSPI0) & kQSPI_Busy)
		;
	QSPI_SetIPCommandAddress (QuadSPI0, FSL_FEATURE_QSPI_AMBA_BASE);
	QSPI_ClearFifo           (QuadSPI0, kQSPI_TxFifo);					// Clear Tx FIFO
	// Write enable command:
	while (QSPI_GetStatusFlags(QuadSPI0) & kQSPI_Busy)
		;
	QSPI_ExecuteIPCommand    (QuadSPI0, 4U);
	QSPI_WriteBlocking       (QuadSPI0, val, 16U);						// Write data into TX FIFO, needs to write at least 16 bytes of data
	QSPI_ExecuteIPCommand    (QuadSPI0, 20);							// Set seq id, write register
	CheckIfFinished          (QuadSPI0);								// Wait until finished


	static uint8_t write_buffer[FLASH_PAGE_SIZE];
//	libmem_register_driver (&FlashHandle, (uint8_t *)FLASH_START_ADDRESS, FLASH_SIZE, geometry, 0, &driver_functions, &ext_driver_functions);
	libmem_register_driver (&FlashHandle, (uint8_t *)FLASH_START_ADDRESS, FLASH_SIZE, geometry, 0, &driver_functions, NULL);
	int result = libmem_driver_paged_write_init (&paged_write_ctrlblk, write_buffer, FLASH_PAGE_SIZE, ProgramPage, 4, LIBMEM_DRIVER_PAGED_WRITE_OPTION_DISABLE_PAGE_PRELOAD);
	if (result != LIBMEM_STATUS_SUCCESS)
		return result;

	return LIBMEM_STATUS_SUCCESS;
}

static void callback(QuadSPI_Type *base, qspi_edma_handle_t *handle, status_t status, void *userData)
{
    isFinished = true;
}


//libmem_driver_handle_t flash1_handle;
//    static uint8_t write_buffer[FLASH_SIZE];
//	int res = libmem_register_flm_driver (&flash1_handle, (uint8_t *)FLASH_START_ADDRESS, FLASH_SIZE, geometry, write_buffer, sizeof(write_buffer), fosc);
//
//int libmem_register_flm_driver (libmem_driver_handle_t *h,uint8_t *start, size_t size, const libmem_geometry_t *geometry, uint8_t *page_buffer, size_t page_size, uint32_t clk)
//{
//	libmem_register_driver (h, start, size, geometry, 0, &driver_functions, NULL);
//	int result = libmem_driver_paged_write_init (&paged_write_ctrlblk, page_buffer, page_size, ProgramPage, 4, LIBMEM_DRIVER_PAGED_WRITE_OPTION_DISABLE_PAGE_PRELOAD);
//    if (result != LIBMEM_STATUS_SUCCESS)
//		return result;
//	s_clk = clk;
//	return LIBMEM_STATUS_SUCCESS;
//}
//
//int libmem_register_flm_driver_with_verify (libmem_driver_handle_t *h,uint8_t *start, size_t size, const libmem_geometry_t *geometry, uint8_t *page_buffer, size_t page_size, uint32_t clk)
//{
//
//}


const char * const Libmem_GetErrorString (int Error)
{
	switch (Error)
	{
		// error codes
		case LIBMEM_STATUS_ERROR:
			return "Generic Error";
		case LIBMEM_STATUS_TIMEOUT:
			return "Timeout Error";
		case LIBMEM_STATUS_LOCKED:
			return "Lock Error";
		case LIBMEM_STATUS_NOT_IMPLEMENTED:
			return "Not Implemented Error";
		case LIBMEM_STATUS_GEOMETRY_REGION_OVERFLOW:
			return "Geometry Region Overflow Error";
		case LIBMEM_STATUS_NO_DRIVER:
			return "No Driver Error";
		case LIBMEM_STATUS_CFI_ERROR:
			return "CFI Error";
		case LIBMEM_STATUS_INVALID_RANGE:
			return "Invalid Range Error";
		case LIBMEM_STATUS_INVALID_PARAMETER:
			return "Invalid parameter Error";
		case LIBMEM_STATUS_INVALID_WIDTH:
			return "Invalid Widdth Error";
		case LIBMEM_STATUS_INVALID_DEVICE:
			return "Invalid Device Error";
//		case LIBMEM_CFI_CMDSET_NONE:
//			return "Invalid CFI command set number";
		
		// status Codes
//        case LIBMEM_STATUS_SUCCESS:
//			return "Success";
//		case LIBMEM_CFI_CMDSET_INTEL_EXTENDED:
//			return "Intel extended command set";
//		case LIBMEM_CFI_CMDSET_AMD_STANDARD:
//			return "AMD standard command set";
//		case LIBMEM_CFI_CMDSET_INTEL_STANDARD:
//			return "Intel standard command set";
//		case LIBMEM_CFI_CMDSET_AMD_EXTENDED:
//			return "AMD extended command set";
//		case LIBMEM_CFI_CMDSET_WINBOND_STANDARD:
//			return "Winbond standard command set";
//		case LIBMEM_CFI_CMDSET_MITSUBISHI_STANDARD:
//			return "Mitsubishi standard command set";
//		case LIBMEM_CFI_CMDSET_MITSUBISHI_EXTENDED:
//			return "Mitsubishi extended command set";
//		case LIBMEM_CFI_CMDSET_SST_PAGE_WRITE:
//			return "SST page write command set";
		default:
			return "Unknown Error";
	}
}