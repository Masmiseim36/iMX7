/*
 * Copyright 2013-2014 Freescale Semiconductor, Inc.
 *
 * Register definitions for Freescale QSPI
 *
 * SPDX-License-Identifier:	GPL-2.0+
 *
 * Modifications Copyright (c) 2016 ARM Germany GmbH. All rights reserved.
 */

#ifndef _FSL_QSPI_H_
#define _FSL_QSPI_H_

#include <stddef.h>
#include <string.h>
#include <stdint.h>

#ifdef QSPI1
#define CONFIG_QSPI_BASE            0x30BB0000
#endif 

#ifdef QSPI2
#define CONFIG_QSPI_BASE            0x30BB4000
#endif 
    
#define CONFIG_QSPI_MEMMAP_BASE	    0x60000000

#define SPI_XFER_BEGIN		          0x01	/* Assert CS before transfer */
#define SPI_XFER_END		            0x02	/* Deassert CS after transfer */
            
#define SPI_DEFAULT_WORDLEN         8

#define QUADSPI_AHBMAP_BANK_MAXSIZE 0x04000000

#define IOMUXC_BASE                 0x30330000
	
// QSPI2 pins
#define IOMUXC_SW_MUX_CTL_PAD_EPDC_DATA00 (*(volatile unsigned long *)(IOMUXC_BASE + 0x034))
#define IOMUXC_SW_MUX_CTL_PAD_EPDC_DATA01 (*(volatile unsigned long *)(IOMUXC_BASE + 0x038))
#define IOMUXC_SW_MUX_CTL_PAD_EPDC_DATA02 (*(volatile unsigned long *)(IOMUXC_BASE + 0x03C))
#define IOMUXC_SW_MUX_CTL_PAD_EPDC_DATA03 (*(volatile unsigned long *)(IOMUXC_BASE + 0x040))
#define IOMUXC_SW_MUX_CTL_PAD_EPDC_DATA04 (*(volatile unsigned long *)(IOMUXC_BASE + 0x044))
#define IOMUXC_SW_MUX_CTL_PAD_EPDC_DATA05 (*(volatile unsigned long *)(IOMUXC_BASE + 0x048))
#define IOMUXC_SW_MUX_CTL_PAD_EPDC_DATA06 (*(volatile unsigned long *)(IOMUXC_BASE + 0x04C))


#define QSPI_IPCR_SEQID_SHIFT		24
#define QSPI_IPCR_SEQID_MASK		(0xf << QSPI_IPCR_SEQID_SHIFT)

#define QSPI_MCR_END_CFD_SHIFT		2
#define QSPI_MCR_END_CFD_MASK		(3 << QSPI_MCR_END_CFD_SHIFT)
#ifdef CONFIG_SYS_FSL_QSPI_AHB
/* AHB needs 64bit operation */
#define QSPI_MCR_END_CFD_LE		(3 << QSPI_MCR_END_CFD_SHIFT)
#else
#define QSPI_MCR_END_CFD_LE		(1 << QSPI_MCR_END_CFD_SHIFT)
#endif
#define QSPI_MCR_DDR_EN_SHIFT		7
#define QSPI_MCR_DDR_EN_MASK		(1 << QSPI_MCR_DDR_EN_SHIFT)
#define QSPI_MCR_CLR_RXF_SHIFT		10
#define QSPI_MCR_CLR_RXF_MASK		(1 << QSPI_MCR_CLR_RXF_SHIFT)
#define QSPI_MCR_CLR_TXF_SHIFT		11
#define QSPI_MCR_CLR_TXF_MASK		(1 << QSPI_MCR_CLR_TXF_SHIFT)
#define QSPI_MCR_MDIS_SHIFT		14
#define QSPI_MCR_MDIS_MASK		(1 << QSPI_MCR_MDIS_SHIFT)
#define QSPI_MCR_RESERVED_SHIFT		16
#define QSPI_MCR_RESERVED_MASK		(0xf << QSPI_MCR_RESERVED_SHIFT)
#define QSPI_MCR_SWRSTHD_SHIFT		1
#define QSPI_MCR_SWRSTHD_MASK		(1 << QSPI_MCR_SWRSTHD_SHIFT)
#define QSPI_MCR_SWRSTSD_SHIFT		0
#define QSPI_MCR_SWRSTSD_MASK		(1 << QSPI_MCR_SWRSTSD_SHIFT)

#define QSPI_SMPR_HSENA_SHIFT		0
#define QSPI_SMPR_HSENA_MASK		(1 << QSPI_SMPR_HSENA_SHIFT)
#define QSPI_SMPR_FSPHS_SHIFT		5
#define QSPI_SMPR_FSPHS_MASK		(1 << QSPI_SMPR_FSPHS_SHIFT)
#define QSPI_SMPR_FSDLY_SHIFT		6
#define QSPI_SMPR_FSDLY_MASK		(1 << QSPI_SMPR_FSDLY_SHIFT)
#define QSPI_SMPR_DDRSMP_SHIFT		16
#define QSPI_SMPR_DDRSMP_MASK		(7 << QSPI_SMPR_DDRSMP_SHIFT)

#define QSPI_BUFXCR_INVALID_MSTRID	0xe
#define QSPI_BUF3CR_ALLMST_SHIFT	31
#define QSPI_BUF3CR_ALLMST_MASK		(1 << QSPI_BUF3CR_ALLMST_SHIFT)
#define QSPI_BUF3CR_ADATSZ_SHIFT	8
#define QSPI_BUF3CR_ADATSZ_MASK		(0xFF << QSPI_BUF3CR_ADATSZ_SHIFT)

#define QSPI_BFGENCR_SEQID_SHIFT	12
#define QSPI_BFGENCR_SEQID_MASK		(0xf << QSPI_BFGENCR_SEQID_SHIFT)
#define QSPI_BFGENCR_PAR_EN_SHIFT	16
#define QSPI_BFGENCR_PAR_EN_MASK	(1 << QSPI_BFGENCR_PAR_EN_SHIFT)

#define QSPI_RBSR_RDBFL_SHIFT		8
#define QSPI_RBSR_RDBFL_MASK		(0x3f << QSPI_RBSR_RDBFL_SHIFT)

#define QSPI_RBCT_RXBRD_SHIFT		8
#define QSPI_RBCT_RXBRD_USEIPS		(1 << QSPI_RBCT_RXBRD_SHIFT)

#define QSPI_SR_BUSY_SHIFT		0
#define QSPI_SR_BUSY_MASK		(1 << QSPI_SR_BUSY_SHIFT)

#define QSPI_LCKCR_LOCK			0x1
#define QSPI_LCKCR_UNLOCK		0x2

#define LUT_KEY_VALUE			0x5af05af0

#define OPRND0_SHIFT			0
#define OPRND0(x)			((x) << OPRND0_SHIFT)
#define PAD0_SHIFT			8
#define PAD0(x)				((x) << PAD0_SHIFT)
#define INSTR0_SHIFT			10
#define INSTR0(x)			((x) << INSTR0_SHIFT)
#define OPRND1_SHIFT			16
#define OPRND1(x)			((x) << OPRND1_SHIFT)
#define PAD1_SHIFT			24
#define PAD1(x)				((x) << PAD1_SHIFT)
#define INSTR1_SHIFT			26
#define INSTR1(x)			((x) << INSTR1_SHIFT)

#define LUT_CMD				1
#define LUT_ADDR			2
#define LUT_DUMMY			3
#define LUT_READ			7
#define LUT_WRITE			8

#define LUT_PAD1			0
#define LUT_PAD2			1
#define LUT_PAD4			2

#define ADDR24BIT			0x18
#define ADDR32BIT			0x20

/* The registers */
#define QUADSPI_MCR			0x00
#define QUADSPI_MCR_RESERVED_SHIFT	16
#define QUADSPI_MCR_RESERVED_MASK	(0xF << QUADSPI_MCR_RESERVED_SHIFT)
#define QUADSPI_MCR_MDIS_SHIFT		14
#define QUADSPI_MCR_MDIS_MASK		(1 << QUADSPI_MCR_MDIS_SHIFT)
#define QUADSPI_MCR_CLR_TXF_SHIFT	11
#define QUADSPI_MCR_CLR_TXF_MASK	(1 << QUADSPI_MCR_CLR_TXF_SHIFT)
#define QUADSPI_MCR_CLR_RXF_SHIFT	10
#define QUADSPI_MCR_CLR_RXF_MASK	(1 << QUADSPI_MCR_CLR_RXF_SHIFT)
#define QUADSPI_MCR_DDR_EN_SHIFT	7
#define QUADSPI_MCR_DDR_EN_MASK		(1 << QUADSPI_MCR_DDR_EN_SHIFT)
#define QUADSPI_MCR_END_CFG_SHIFT   2
#define QUADSPI_MCR_END_CFG_MASK    (3 << QUADSPI_MCR_END_CFG_SHIFT)
#define QUADSPI_MCR_SWRSTHD_SHIFT	1
#define QUADSPI_MCR_SWRSTHD_MASK	(1 << QUADSPI_MCR_SWRSTHD_SHIFT)
#define QUADSPI_MCR_SWRSTSD_SHIFT	0
#define QUADSPI_MCR_SWRSTSD_MASK	(1 << QUADSPI_MCR_SWRSTSD_SHIFT)

#define QUADSPI_IPCR			0x08
#define QUADSPI_IPCR_SEQID_SHIFT	24
#define QUADSPI_IPCR_SEQID_MASK		(0xF << QUADSPI_IPCR_SEQID_SHIFT)

#define QUADSPI_BUF0CR			0x10
#define QUADSPI_BUF1CR			0x14
#define QUADSPI_BUF2CR			0x18
#define QUADSPI_BUFXCR_INVALID_MSTRID	0xe

#define QUADSPI_BUF3CR			0x1c
#define QUADSPI_BUF3CR_ALLMST_SHIFT	31
#define QUADSPI_BUF3CR_ALLMST_MASK  (1U << QUADSPI_BUF3CR_ALLMST_SHIFT)
#define QUADSPI_BUF3CR_ADATSZ_SHIFT	8
#define QUADSPI_BUF3CR_ADATSZ_MASK	(0xFF << QUADSPI_BUF3CR_ADATSZ_SHIFT)

#define QUADSPI_BFGENCR			0x20
#define QUADSPI_BFGENCR_PAR_EN_SHIFT	16
#define QUADSPI_BFGENCR_PAR_EN_MASK	(1 << (QUADSPI_BFGENCR_PAR_EN_SHIFT))
#define QUADSPI_BFGENCR_SEQID_SHIFT	12
#define QUADSPI_BFGENCR_SEQID_MASK	(0xF << QUADSPI_BFGENCR_SEQID_SHIFT)

#define QUADSPI_BUF0IND			0x30
#define QUADSPI_BUF1IND			0x34
#define QUADSPI_BUF2IND			0x38
#define QUADSPI_SFAR			0x100

#define QUADSPI_SMPR			0x108
#define QUADSPI_SMPR_DDRSMP_SHIFT	16
#define QUADSPI_SMPR_DDRSMP_MASK	(7 << QUADSPI_SMPR_DDRSMP_SHIFT)
#define QUADSPI_SMPR_FSDLY_SHIFT	6
#define QUADSPI_SMPR_FSDLY_MASK		(1 << QUADSPI_SMPR_FSDLY_SHIFT)
#define QUADSPI_SMPR_FSPHS_SHIFT	5
#define QUADSPI_SMPR_FSPHS_MASK		(1 << QUADSPI_SMPR_FSPHS_SHIFT)
#define QUADSPI_SMPR_HSENA_SHIFT	0
#define QUADSPI_SMPR_HSENA_MASK		(1 << QUADSPI_SMPR_HSENA_SHIFT)

#define QUADSPI_RBSR			0x10c
#define QUADSPI_RBSR_RDBFL_SHIFT	8
#define QUADSPI_RBSR_RDBFL_MASK		(0x3F << QUADSPI_RBSR_RDBFL_SHIFT)

#define QUADSPI_RBCT			0x110
#define QUADSPI_RBCT_WMRK_MASK		0x1F
#define QUADSPI_RBCT_RXBRD_SHIFT	8
#define QUADSPI_RBCT_RXBRD_USEIPS	(0x1 << QUADSPI_RBCT_RXBRD_SHIFT)

#define QUADSPI_TBSR			0x150
#define QUADSPI_TBDR			0x154
#define QUADSPI_SR			0x15c
#define QUADSPI_SR_IP_ACC_SHIFT     1
#define QUADSPI_SR_IP_ACC_MASK      (0x1 << QUADSPI_SR_IP_ACC_SHIFT)
#define QUADSPI_SR_AHB_ACC_SHIFT    2
#define QUADSPI_SR_AHB_ACC_MASK     (0x1 << QUADSPI_SR_AHB_ACC_SHIFT)


#define QUADSPI_FR			0x160
#define QUADSPI_FR_TFF_MASK		0x1

#define QUADSPI_SFA1AD			0x180
#define QUADSPI_SFA2AD			0x184
#define QUADSPI_SFB1AD			0x188
#define QUADSPI_SFB2AD			0x18c
#define QUADSPI_RBDR			0x200

#define QUADSPI_LUTKEY			0x300
#define QUADSPI_LUTKEY_VALUE		0x5AF05AF0

#define QUADSPI_LCKCR			0x304
#define QUADSPI_LCKER_LOCK		0x1
#define QUADSPI_LCKER_UNLOCK		0x2

#define QUADSPI_RSER			0x164
#define QUADSPI_RSER_TFIE		(0x1 << 0)

#define QUADSPI_LUT_BASE		0x310

/*
 * The definition of the LUT register shows below:
 *
 *  ---------------------------------------------------
 *  | INSTR1 | PAD1 | OPRND1 | INSTR0 | PAD0 | OPRND0 |
 *  ---------------------------------------------------
 */
#define OPRND0_SHIFT		0
#define PAD0_SHIFT		8
#define INSTR0_SHIFT		10
#define OPRND1_SHIFT		16

/* Instruction set for the LUT register. */
#define LUT_STOP		0
#define LUT_CMD			1
#define LUT_ADDR		2
#define LUT_DUMMY		3
#define LUT_MODE		4
#define LUT_MODE2		5
#define LUT_MODE4		6
#define LUT_READ		7
#define LUT_WRITE		8
#define LUT_JMP_ON_CS		9
#define LUT_ADDR_DDR		10
#define LUT_MODE_DDR		11
#define LUT_MODE2_DDR		12
#define LUT_MODE4_DDR		13
#define LUT_READ_DDR		14
#define LUT_WRITE_DDR		15
#define LUT_DATA_LEARN		16

/*
 * The PAD definitions for LUT register.
 *
 * The pad stands for the lines number of IO[0:3].
 * For example, the Quad read need four IO lines, so you should
 * set LUT_PAD4 which means we use four IO lines.
 */
#define LUT_PAD1		0
#define LUT_PAD2		1
#define LUT_PAD4		2

/* Oprands for the LUT register. */
#define ADDR24BIT		0x18
#define ADDR32BIT		0x20

/* Macros for constructing the LUT register. */
#define LUT0(ins, pad, opr)						\
		(((opr) << OPRND0_SHIFT) | ((LUT_##pad) << PAD0_SHIFT) | \
		((LUT_##ins) << INSTR0_SHIFT))

#define LUT1(ins, pad, opr)	(LUT0(ins, pad, opr) << OPRND1_SHIFT)

/* other macros for LUT register. */
#define QUADSPI_LUT(x)          (QUADSPI_LUT_BASE + (x) * 4)
#define QUADSPI_LUT_NUM		64

/* SEQID -- we can have 16 seqids at most. */
#define SEQID_QUAD_READ		0
#define SEQID_WREN		1
#define SEQID_FAST_READ		2
#define SEQID_RDSR		3
#define SEQID_SE		4
#define SEQID_CHIP_ERASE	5
#define SEQID_PP		6
#define SEQID_RDID		7
#define SEQID_WRSR		8
#define SEQID_RDCR		9
#define SEQID_DDR_QUAD_READ	10
#define SEQID_BE_4K		11
#ifdef CONFIG_SPI_FLASH_BAR
#define SEQID_BRRD		12
#define SEQID_BRWR		13
#define SEQID_RDEAR		14
#define SEQID_WREAR		15
#endif

/* Flash opcodes. */
#define	OPCODE_WREN		0x06	/* Write enable */
#define	OPCODE_RDSR		0x05	/* Read status register */
#define	OPCODE_WRSR		0x01	/* Write status register 1 byte */
#define	OPCODE_NORM_READ	0x03	/* Read data bytes (low frequency) */
#define	OPCODE_FAST_READ	0x0b	/* Read data bytes (high frequency) */
#define	OPCODE_QUAD_READ        0x6b    /* Read data bytes */
#define	OPCODE_DDR_QUAD_READ	0x6d    /* Read data bytes in DDR mode*/
#define	OPCODE_PP		0x02	/* Page program (up to 256 bytes) */
#define	OPCODE_BE_4K		0x20	/* Erase 4KiB block */
#define	OPCODE_BE_4K_PMC	0xd7	/* Erase 4KiB block on PMC chips */
#define	OPCODE_BE_32K		0x52	/* Erase 32KiB block */
#define	OPCODE_CHIP_ERASE	0xc7	/* Erase whole flash chip */
#define	OPCODE_SE		0xd8	/* Sector erase (usually 64KiB) */
#define	OPCODE_RDID		0x9f	/* Read JEDEC ID */
#define	OPCODE_RDCR             0x35    /* Read configuration register */

/* 4-byte address opcodes - used on Spansion and some Macronix flashes. */
#define	OPCODE_NORM_READ_4B	0x13	/* Read data bytes (low frequency) */
#define	OPCODE_FAST_READ_4B	0x0c	/* Read data bytes (high frequency) */
#define	OPCODE_QUAD_READ_4B	0x6c    /* Read data bytes */
#define	OPCODE_PP_4B		0x12	/* Page program (up to 256 bytes) */
#define	OPCODE_SE_4B		0xdc	/* Sector erase (usually 64KiB) */

/* Used for SST flashes only. */
#define	OPCODE_BP		0x02	/* Byte program */
#define	OPCODE_WRDI		0x04	/* Write disable */
#define	OPCODE_AAI_WP		0xad	/* Auto address increment word program */

/* Used for Macronix and Winbond flashes. */
#define	OPCODE_EN4B		0xb7	/* Enter 4-byte mode */
#define	OPCODE_EX4B		0xe9	/* Exit 4-byte mode */

/* Used for Micron, winbond and Macronix flashes */
#define	OPCODE_WREAR		0xc5	/* EAR register write */
#define	OPCODE_RDEAR		0xc8	/* EAR reigster read */

/* Used for Spansion flashes only. */
#define	OPCODE_BRRD		0x16	/* Bank register read */
#define	OPCODE_BRWR		0x17	/* Bank register write */

/* Status Register bits. */
#define	SR_WIP			1	/* Write in progress */
#define	SR_WEL			2	/* Write enable latch */
/* meaning of other SR_* bits may differ between vendors */
#define	SR_BP0			4	/* Block protect 0 */
#define	SR_BP1			8	/* Block protect 1 */
#define	SR_BP2			0x10	/* Block protect 2 */
#define	SR_SRWD			0x80	/* SR write protect */

#define SR_QUAD_EN_MX           0x40    /* Macronix Quad I/O */

/* Configuration Register bits. */
#define CR_QUAD_EN_SPAN		0x2     /* Spansion Quad I/O */

/* Endianess Configuration */
#define BE_64	0x00
#define LE_32	0x01
#define BE_32	0x02
#define LE_64	0x03


/* Erase commands */
#define CMD_ERASE_2K			0x50
#define CMD_ERASE_4K			0x20
#define CMD_ERASE_32K			0x52
#define CMD_ERASE_CHIP		0xc7
#define CMD_ERASE_64K			0xd8

/* Write commands */
#define CMD_WRITE_STATUS		0x01
#define CMD_PAGE_PROGRAM		0x02
#define CMD_WRITE_DISABLE		0x04
#ifdef CONFIG_SPI_FLASH_ATMEL
#define CMD_READ_STATUS			0xd7
#else
#define CMD_READ_STATUS			0x05
#endif
#define CMD_QUAD_PAGE_PROGRAM		0x32
#define CMD_READ_STATUS1		0x35
#define CMD_WRITE_ENABLE		0x06
#define CMD_READ_CONFIG		0x35
#define CMD_FLAG_STATUS		0x70

/* Read commands */
#define CMD_READ_ARRAY_SLOW		0x03
#define CMD_READ_ARRAY_FAST		0x0b
#define CMD_READ_DUAL_OUTPUT_FAST	0x3b
#define CMD_READ_DUAL_IO_FAST		0xbb
#define CMD_READ_QUAD_OUTPUT_FAST	0x6b
#define CMD_READ_QUAD_IO_FAST		0xeb
#define CMD_READ_ID			0x9f

enum fsl_qspi_devtype {
	FSL_QUADSPI_VYBRID,
	FSL_QUADSPI_IMX6SX,
};

struct fsl_qspi_devtype_data {
	enum fsl_qspi_devtype devtype;
	int rxfifo;
	int txfifo;
};

struct spi_slave {
	unsigned int bus;
	unsigned int cs;
	uint8_t op_mode_rx;
	uint8_t op_mode_tx;
	unsigned int wordlen;
	unsigned int max_write_size;
	void *memory_map;
	uint8_t option;
	uint8_t flags;
};

struct fsl_qspi {
	struct spi_slave slave;
	uint32_t max_khz;
	uint32_t mode;
	uint32_t iobase;
	uint32_t ahb_base; /* Used when read from AHB bus */
	uint32_t bank_memmap_phy[4];
	struct fsl_qspi_devtype_data *devtype_data;
};
  
struct spi_slave *spi_setup_slave(unsigned int bus, unsigned int cs,
		unsigned int max_hz, unsigned int mode);

void spi_free_slave(struct spi_slave *slave);

int  spi_xfer(struct spi_slave *slave, unsigned int bitlen, const void *dout,
		void *din, unsigned long flags);

void *spi_do_alloc_slave(int offset, int size, unsigned int bus,
			 unsigned int cs);

void spi_flash_addr(uint32_t addr, uint8_t *cmd);

int spi_flash_read_write(struct spi_slave *spi,
				uint8_t *cmd, size_t cmd_len,
				const uint8_t *data_out, uint8_t *data_in,
				size_t data_len);
        
#define spi_alloc_slave(_struct, bus, cs) \
	spi_do_alloc_slave(offsetof(_struct, slave), \
			    sizeof(_struct), bus, cs)

#define container_of(ptr, type, member) \
    (type *)((char *)(ptr) - offsetof(type, member))
      
#define readw(addr) (*(volatile uint16_t *) (addr))
#define readl(addr) (*(volatile uint32_t *) (addr))
#define writew(b,addr) ((*(volatile uint16_t *) (addr)) = (b))
#define writel(b,addr) ((*(volatile uint32_t *) (addr)) = (b))
          
#endif /* _FSL_QSPI_H_ */
