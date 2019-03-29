/***********************************************************************/
/*  This file is part of the ARM Toolchain package                     */
/*  Copyright (c) 2016 Keil - An ARM Company. All rights reserved.     */
/***********************************************************************/
/*                                                                     */
/*  FlashDev.C:  Flash Programming Functions                           */
/*                                                                     */
/***********************************************************************/

#include "../FlashOS.H"        // FlashOS Structures              
#include "fsl_qspi.h"

/* 
   Mandatory Flash Programming Functions (Called by FlashOS):
                int Init        (unsigned long adr,   // Initialize Flash
                                 unsigned long clk,
                                 unsigned long fnc);
                int UnInit      (unsigned long fnc);  // De-initialize Flash
                int EraseSector (unsigned long adr);  // Erase Sector Function
                int ProgramPage (unsigned long adr,   // Program Page Function
                                 unsigned long sz,
                                 unsigned char *buf);

   Optional  Flash Programming Functions (Called by FlashOS):
                int BlankCheck  (unsigned long adr,   // Blank Check
                                 unsigned long sz,
                                 unsigned char pat);
                int EraseChip   (void);               // Erase complete Device
      unsigned long Verify      (unsigned long adr,   // Verify Function
                                 unsigned long sz,
                                 unsigned char *buf);

       - BlanckCheck  is necessary if Flash space is not mapped into CPU memory space
       - Verify       is necessary if Flash space is not mapped into CPU memory space
       - if EraseChip is not provided than EraseSector for all sectors is called
*/

#define DEV_START_ADDR_A    0x60000000

static struct spi_slave *bus = NULL;
static unsigned long dev_start_adr = 0;

/*  Initialize Flash Programming Functions
 *    Parameter:      adr:  Device Base Address
 *                    clk:  Clock Frequency (Hz)
 *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */
int Init (unsigned long adr, unsigned long clk, unsigned long fnc) {
  
  IOMUXC_SW_MUX_CTL_PAD_EPDC_DATA00 = 0x00000002;
  IOMUXC_SW_MUX_CTL_PAD_EPDC_DATA01 = 0x00000002;
  IOMUXC_SW_MUX_CTL_PAD_EPDC_DATA02 = 0x00000002;
  IOMUXC_SW_MUX_CTL_PAD_EPDC_DATA03 = 0x00000002;
  IOMUXC_SW_MUX_CTL_PAD_EPDC_DATA04 = 0x00000002;
  IOMUXC_SW_MUX_CTL_PAD_EPDC_DATA05 = 0x00000002;
  IOMUXC_SW_MUX_CTL_PAD_EPDC_DATA06 = 0x00000002;                                    
  
  if (adr == DEV_START_ADDR_A) bus = spi_setup_slave(0,0,clk,0);
  
  if (bus==NULL) return 1;
  
  dev_start_adr = adr;

  uint8_t idcode[5];  
  uint8_t cmd = CMD_READ_ID;
  if (spi_flash_read_write(bus, (uint8_t*)&cmd, 1, NULL, idcode, sizeof(idcode))) return 1;

  return 0;                                  /* Finished without Errors */
}


/*  De-Initialize Flash Programming Functions
 *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int UnInit (unsigned long fnc) {
  bus = NULL;
  dev_start_adr = 0;
  return 0;                                  // Finished without Errors
}


/*  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 */
int EraseSector (unsigned long adr) {
  uint8_t cmd[4];
  
  cmd[0] = CMD_WRITE_ENABLE;
  if (spi_flash_read_write(bus, (uint8_t*)&cmd, 1, NULL, NULL, 0)) return 1;
  
  cmd[0] = CMD_ERASE_4K;
  spi_flash_addr(adr - dev_start_adr, cmd); 
  if (spi_flash_read_write(bus, (uint8_t*)&cmd, 4, NULL, NULL, 0)) return 1;
  
  uint8_t status = 1;
  cmd[0] = CMD_READ_STATUS;
  while (status)
    if (spi_flash_read_write(bus, (uint8_t*)&cmd , 1, NULL, &status, 1)) return 1;
  
  cmd[0] = CMD_WRITE_DISABLE;
  if (spi_flash_read_write(bus, (uint8_t*)&cmd, 1, NULL, NULL, 0)) return 1;
  
  return (0);
}


/*  Program Page in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
 */

int ProgramPage (unsigned long adr, unsigned long sz, unsigned char *buf) {
  uint8_t cmd[4];

  cmd[0] = CMD_WRITE_ENABLE;
  if (spi_flash_read_write(bus, (uint8_t*)&cmd, 1, NULL, NULL, 0)) return 1;
  
  cmd[0] = CMD_PAGE_PROGRAM;
  spi_flash_addr(adr - dev_start_adr, cmd);  
  if (spi_flash_read_write(bus, (uint8_t*)&cmd, 4, buf, NULL, sz)) return 1;
  
  uint8_t status = 1;
  cmd[0] = CMD_READ_STATUS;
  while (status)
    if (spi_flash_read_write(bus, (uint8_t*)&cmd , 1, NULL, &status, 1)) return 1;
  
  cmd[0] = CMD_WRITE_DISABLE;
  if (spi_flash_read_write(bus, (uint8_t*)&cmd, 1, NULL, NULL, 0)) return 1;
  
  return (0);
}
