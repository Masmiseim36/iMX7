/***********************************************************************/
/*  This file is part of the ARM Toolchain package                     */
/*  Copyright (c) 2016 Keil - An ARM Company. All rights reserved.     */
/***********************************************************************/
/*                                                                     */
/*  FlashDev.C:  Device Description                                    */
/*                                                                     */
/***********************************************************************/

#include "../FlashOS.H"        // FlashOS Structures

#ifdef QSPI1

#ifdef QSPI1A_32MB
struct FlashDevice const FlashDevice = {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "i.MX7Dual QSPI1A 32MB",    // Device Name 
   EXTSPI,                     // Device Type
   0x60000000,                 // Device Start Address
   0x02000000,                 // Device Size (32MB)
   256,                        // Programming Page Size
   0,                          // Reserved, must be 0
   0xFF,                       // Initial Content of Erased Memory
   500,                        // Program Page Timeout 500 mSec
   3000,                       // Erase Sector Timeout 3000 mSec

// Specify Size and Address of Sectors
   0x1000, 0,                  // sectors are 4 KB
   SECTOR_END
};
#endif

#endif
