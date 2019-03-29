/*
 * Freescale QuadSPI driver.
 *
 * Copyright (C) 2014-2015 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Modifications Copyright (c) 2016 ARM Germany GmbH. All rights reserved.
 */
#include "fsl_qspi.h"

static inline void fsl_qspi_unlock_lut(struct fsl_qspi *q)
{
	writel(QUADSPI_LUTKEY_VALUE, q->iobase + QUADSPI_LUTKEY);
	writel(QUADSPI_LCKER_UNLOCK, q->iobase + QUADSPI_LCKCR);
}

static inline void fsl_qspi_lock_lut(struct fsl_qspi *q)
{
	writel(QUADSPI_LUTKEY_VALUE, q->iobase + QUADSPI_LUTKEY);
	writel(QUADSPI_LCKER_LOCK, q->iobase + QUADSPI_LCKCR);
}

static void fsl_qspi_init_lut(struct fsl_qspi *q)
{
	uint32_t base = q->iobase;
	int rxfifo = q->devtype_data->rxfifo;
	uint32_t lut_base;
	uint8_t cmd, addrlen, dummy;
	int i;

	fsl_qspi_unlock_lut(q);

	/* Clear all the LUT table */
	for (i = 0; i < QUADSPI_LUT_NUM; i++)
		writel(0, base + QUADSPI_LUT_BASE + i * 4);

	/* Quad Read */
	lut_base = SEQID_QUAD_READ * 4;

	/* U-boot SPI flash only support 24bits address*/
	cmd = OPCODE_QUAD_READ;
	addrlen = ADDR24BIT;
	dummy = 8;

	writel(LUT0(CMD, PAD1, cmd) | LUT1(ADDR, PAD1, addrlen),
			base + QUADSPI_LUT(lut_base));
	writel(LUT0(DUMMY, PAD1, dummy) | LUT1(READ, PAD4, rxfifo),
			base + QUADSPI_LUT(lut_base + 1));

	/* Write enable */
	lut_base = SEQID_WREN * 4;
	writel(LUT0(CMD, PAD1, OPCODE_WREN), base + QUADSPI_LUT(lut_base));

	/* Fast Read */
	lut_base = SEQID_FAST_READ * 4;
	cmd = OPCODE_FAST_READ;
	addrlen = ADDR24BIT;
	dummy = 8;

	writel(LUT0(CMD, PAD1, cmd) | LUT1(ADDR, PAD1, addrlen),
			base + QUADSPI_LUT(lut_base));
	writel(LUT0(DUMMY, PAD1, dummy) | LUT1(READ, PAD1, rxfifo),
			base + QUADSPI_LUT(lut_base + 1));

	/* Page Program */
	lut_base = SEQID_PP * 4;
	cmd = OPCODE_PP;
	addrlen = ADDR24BIT;

	writel(LUT0(CMD, PAD1, cmd) | LUT1(ADDR, PAD1, addrlen),
			base + QUADSPI_LUT(lut_base));
	writel(LUT0(WRITE, PAD1, 0), base + QUADSPI_LUT(lut_base + 1));

	/* Read Status */
	lut_base = SEQID_RDSR * 4;
	writel(LUT0(CMD, PAD1, OPCODE_RDSR) | LUT1(READ, PAD1, 0x1),
			base + QUADSPI_LUT(lut_base));

	/* Erase a sector */
	lut_base = SEQID_SE * 4;
	cmd = OPCODE_SE;
	addrlen = ADDR24BIT;

	writel(LUT0(CMD, PAD1, cmd) | LUT1(ADDR, PAD1, addrlen),
			base + QUADSPI_LUT(lut_base));

	/* Erase the whole chip */
	lut_base = SEQID_CHIP_ERASE * 4;
	writel(LUT0(CMD, PAD1, OPCODE_CHIP_ERASE),
			base + QUADSPI_LUT(lut_base));

	/* READ ID */
	lut_base = SEQID_RDID * 4;
	writel(LUT0(CMD, PAD1, OPCODE_RDID) | LUT1(READ, PAD1, 0x8),
			base + QUADSPI_LUT(lut_base));

	/* Write Register */
	lut_base = SEQID_WRSR * 4;
	writel(LUT0(CMD, PAD1, OPCODE_WRSR) | LUT1(WRITE, PAD1, 0x2),
			base + QUADSPI_LUT(lut_base));

	/* Read Configuration Register */
	lut_base = SEQID_RDCR * 4;
	writel(LUT0(CMD, PAD1, OPCODE_RDCR) | LUT1(READ, PAD1, 0x1),
			base + QUADSPI_LUT(lut_base));

	/* DDR QUAD Read */
	lut_base = SEQID_DDR_QUAD_READ * 4;
	cmd = OPCODE_DDR_QUAD_READ;
	addrlen = ADDR24BIT;
	dummy = 6;

	writel(LUT0(CMD, PAD1, cmd) | LUT1(ADDR_DDR, PAD1, addrlen),
			base + QUADSPI_LUT(lut_base));
	writel(LUT0(DUMMY, PAD1, dummy) | LUT1(READ_DDR, PAD4, rxfifo),
			base + QUADSPI_LUT(lut_base + 1));
	writel(LUT0(JMP_ON_CS, PAD1, 0),
			base + QUADSPI_LUT(lut_base + 2));

	/* SUB SECTOR 4K ERASE */
	lut_base = SEQID_BE_4K * 4;
	cmd = OPCODE_BE_4K;
	addrlen = ADDR24BIT;

	writel(LUT0(CMD, PAD1, cmd) | LUT1(ADDR, PAD1, addrlen),
			base + QUADSPI_LUT(lut_base));

#ifdef CONFIG_SPI_FLASH_BAR
	/*
	 * BRRD BRWR RDEAR WREAR are all supported, because it is hard to
	 * dynamically check whether to set BRRD BRWR or RDEAR WREAR.
	 */
	lut_base = SEQID_BRRD * 4;
	cmd = OPCODE_BRRD;
	writel(LUT0(CMD, PAD1, cmd) | LUT1(READ, PAD1, 0x1),
	       base + QUADSPI_LUT(lut_base));

	lut_base = SEQID_BRWR * 4;
	cmd = OPCODE_BRWR;
	writel(LUT0(CMD, PAD1, cmd) | LUT1(WRITE, PAD1, 0x1),
	       base + QUADSPI_LUT(lut_base));

	lut_base = SEQID_RDEAR * 4;
	cmd = OPCODE_RDEAR;
	writel(LUT0(CMD, PAD1, cmd) | LUT1(READ, PAD1, 0x1),
	       base + QUADSPI_LUT(lut_base));

	lut_base = SEQID_WREAR * 4;
	cmd = OPCODE_WREAR;
	writel(LUT0(CMD, PAD1, cmd) | LUT1(WRITE, PAD1, 0x1),
	       base + QUADSPI_LUT(lut_base));
#endif

	fsl_qspi_lock_lut(q);
}

/* Enable DDR Read Mode*/
static void fsl_enable_ddr_mode(struct fsl_qspi *q)
{
	uint32_t base = q->iobase;
	uint32_t reg, reg2;

	reg = readl(base + QUADSPI_MCR);
	/* Firstly, disable the module */
	writel(reg | QUADSPI_MCR_MDIS_MASK, base + QUADSPI_MCR);

	/* Set the Sampling Register for DDR */
	reg2 = readl(base + QUADSPI_SMPR);
	reg2 &= ~QUADSPI_SMPR_DDRSMP_MASK;
	reg2 |= (2 << QUADSPI_SMPR_DDRSMP_SHIFT);
	writel(reg2, base + QUADSPI_SMPR);

	/* Enable the module again (enable the DDR too) */
	reg |= QUADSPI_MCR_DDR_EN_MASK;
	reg |= (1 << 29); /* enable bit 29 for imx6sx */

	writel(reg, base + QUADSPI_MCR);
}

/*
 * There are two different ways to read out the data from the flash:
 *  the "IP Command Read" and the "AHB Command Read".
 *
 * The IC guy suggests we use the "AHB Command Read" which is faster
 * then the "IP Command Read". (What's more is that there is a bug in
 * the "IP Command Read" in the Vybrid.)
 *
 * After we set up the registers for the "AHB Command Read", we can use
 * the memcpy to read the data directly. A "missed" access to the buffer
 * causes the controller to clear the buffer, and use the sequence pointed
 * by the QUADSPI_BFGENCR[SEQID] to initiate a read from the flash.
 */
static void fsl_qspi_init_abh_read(struct fsl_qspi *q)
{
	uint32_t base = q->iobase;

	/* Map the SPI NOR to accessiable address, arrage max space for each bank*/
	writel(q->bank_memmap_phy[0] + QUADSPI_AHBMAP_BANK_MAXSIZE,
		base + QUADSPI_SFA1AD);
	writel(q->bank_memmap_phy[1] + QUADSPI_AHBMAP_BANK_MAXSIZE,
		base + QUADSPI_SFA2AD);
	writel(q->bank_memmap_phy[2] + QUADSPI_AHBMAP_BANK_MAXSIZE,
		base + QUADSPI_SFB1AD);
	writel(q->bank_memmap_phy[3] + QUADSPI_AHBMAP_BANK_MAXSIZE,
		base + QUADSPI_SFB2AD);

	/* AHB configuration for access buffer 0/1/2 .*/
	writel(QUADSPI_BUFXCR_INVALID_MSTRID, base + QUADSPI_BUF0CR);
	writel(QUADSPI_BUFXCR_INVALID_MSTRID, base + QUADSPI_BUF1CR);
	writel(QUADSPI_BUFXCR_INVALID_MSTRID, base + QUADSPI_BUF2CR);
	writel((uint32_t)QUADSPI_BUF3CR_ALLMST_MASK | (0x80 << QUADSPI_BUF3CR_ADATSZ_SHIFT),
			base + QUADSPI_BUF3CR);

	/* We only use the buffer3 */
	writel(0, base + QUADSPI_BUF0IND);
	writel(0, base + QUADSPI_BUF1IND);
	writel(0, base + QUADSPI_BUF2IND);

	/* Set the default lut sequence for AHB Read. */
	writel(SEQID_FAST_READ << QUADSPI_BFGENCR_SEQID_SHIFT,
		base + QUADSPI_BFGENCR);

	/*Enable DDR Mode*/
	fsl_enable_ddr_mode(q);
}

static int fsl_qspi_init(struct fsl_qspi *q)
{
	uint32_t base = q->iobase;
	uint32_t reg;
	void *ptr;
  
  //ptr = malloc(sizeof(struct fsl_qspi_devtype_data));  
  static struct fsl_qspi_devtype_data f;
  ptr = (void*)&f;

	if (!ptr) {
		//puts("FSL_QSPI: per-type data not allocated !\n");
		return 1;
	}
	q->devtype_data = ptr;
	q->devtype_data->rxfifo = 128;
	q->devtype_data->txfifo = 512;

	/* init the LUT table */
	fsl_qspi_init_lut(q);

	/* Disable the module */
	writel(QUADSPI_MCR_MDIS_MASK | QUADSPI_MCR_RESERVED_MASK,
			base + QUADSPI_MCR);

	reg = readl(base + QUADSPI_SMPR);
	writel(reg & ~(QUADSPI_SMPR_FSDLY_MASK
			| QUADSPI_SMPR_FSPHS_MASK
			| QUADSPI_SMPR_HSENA_MASK
			| QUADSPI_SMPR_DDRSMP_MASK), base + QUADSPI_SMPR);

	/* Enable the module */
	writel(QUADSPI_MCR_RESERVED_MASK | LE_64 << QUADSPI_MCR_END_CFG_SHIFT,
		base + QUADSPI_MCR);

	/* We do not enable the interrupt */

	/* init for AHB read */
	fsl_qspi_init_abh_read(q);

	/*
	 * High level code use page_size and max_write_size to calculate
	 * the number of bytes that should be programmed once.
	 */
	q->slave.max_write_size = q->devtype_data->txfifo;

	return 0;
}

struct spi_slave *spi_setup_slave(unsigned int bus, unsigned int cs,
		unsigned int max_hz, unsigned int mode)
{
	struct fsl_qspi *q;
	int ret;

	if (bus > 1) {
		//puts("FSL_QSPI: Not a valid bus !\n");
		return NULL;
	}

	if (cs > 1) {
		//puts("FSL_QSPI: Not a valid cs !\n");
		return NULL;
	}
           
  q = spi_alloc_slave(struct fsl_qspi, bus, cs);
	if (!q) {
		//puts("FSL_QSPI: SPI Slave not allocated !\n");
		return NULL;
	}

	q->iobase = CONFIG_QSPI_BASE;  
	q->bank_memmap_phy[0] = CONFIG_QSPI_MEMMAP_BASE;
	q->bank_memmap_phy[1] = q->bank_memmap_phy[0] + QUADSPI_AHBMAP_BANK_MAXSIZE;
	q->bank_memmap_phy[2] = q->bank_memmap_phy[1] + QUADSPI_AHBMAP_BANK_MAXSIZE;
	q->bank_memmap_phy[3] = q->bank_memmap_phy[2] + QUADSPI_AHBMAP_BANK_MAXSIZE;

	/* Init the QuadSPI controller */
	ret = fsl_qspi_init(q);
	if (ret) {
		//puts("FSL_QSPI: init failed!\n");
		return NULL;
	}

	return &q->slave;
}

/*
void spi_free_slave(struct spi_slave *slave)
{
	struct fsl_qspi *q;

	q = container_of(slave, struct fsl_qspi, slave);
	free(q->devtype_data);
	free(q);
}
*/

/* Get the SEQID for the command */
static int fsl_qspi_get_seqid(struct fsl_qspi *q, uint8_t cmd)
{
	switch (cmd) {
	case OPCODE_QUAD_READ:
	case OPCODE_QUAD_READ_4B:
		return SEQID_QUAD_READ;
	case OPCODE_FAST_READ:
	case OPCODE_FAST_READ_4B:
		return SEQID_FAST_READ;
	case OPCODE_WREN:
		return SEQID_WREN;
	case OPCODE_RDSR:
		return SEQID_RDSR;
	case OPCODE_SE:
		return SEQID_SE;
	case OPCODE_CHIP_ERASE:
		return SEQID_CHIP_ERASE;
	case OPCODE_PP:
	case OPCODE_PP_4B:
		return SEQID_PP;
	case OPCODE_RDID:
		return SEQID_RDID;
	case OPCODE_WRSR:
		return SEQID_WRSR;
	case OPCODE_RDCR:
		return SEQID_RDCR;
	case OPCODE_DDR_QUAD_READ:
		return SEQID_DDR_QUAD_READ;
	case OPCODE_BE_4K:
		return SEQID_BE_4K;
#ifdef CONFIG_SPI_FLASH_BAR
	case OPCODE_BRRD:
		return SEQID_BRRD;
	case OPCODE_BRWR:
		return SEQID_BRWR;
	case OPCODE_RDEAR:
		return SEQID_RDEAR;
	case OPCODE_WREAR:
		return SEQID_WREAR;
#endif
	default:
		break;
	}
	return -1;
}

/* return 1 on success */
static int fsl_qspi_wait_to_complete(struct fsl_qspi *q)
{
	uint32_t base = q->iobase;
	uint32_t reg;

	/*printf("QuadSPI: poll the busy bit\n");*/
	while (1) {
		reg = readl(base + QUADSPI_SR);
		if (reg & 1)
			continue;
		else
			return 1;
	}
}

/*
 * If we have changed the content of the flash by writing or erasing,
 * we need to invalidate the AHB buffer. If we do not do so, we may read out
 * the wrong data. The spec tells us reset the AHB domain and Serial Flash
 * domain at the same time.
 */
static inline void fsl_qspi_invalid(struct fsl_qspi *q)
{
    uint32_t reg;

    reg = readl(q->iobase + QUADSPI_MCR);
    reg |= QUADSPI_MCR_SWRSTHD_MASK | QUADSPI_MCR_SWRSTSD_MASK;
    writel(reg, q->iobase + QUADSPI_MCR);

    /*
     * The minimum delay : 1 AHB + 2 SFCK clocks.
     * Delay 1 us is enough.
     */
    //udelay(1);
    __nop();
    __nop();
    __nop();

    reg &= ~(QUADSPI_MCR_SWRSTHD_MASK | QUADSPI_MCR_SWRSTSD_MASK);
    writel(reg, q->iobase + QUADSPI_MCR);
}

static int fsl_qspi_runcmd(struct fsl_qspi *q, uint8_t cmd, unsigned int addr, int len)
{
	uint32_t base = q->iobase;
	int seqid;
	uint32_t reg, reg2;
	int err;
	int bank_id;

	/* check the SR first, wait previous cmd completed*/
	do {
		reg2 = readl(base + QUADSPI_SR);
		if (reg2 & (QUADSPI_SR_IP_ACC_MASK | QUADSPI_SR_AHB_ACC_MASK)) {
			//udelay(1);
			//printf("The controller is busy, 0x%x\n", reg2);
			continue;
		}
		break;
	} while (1);

	/* save the reg */
	reg = readl(base + QUADSPI_MCR);

	/* get the bank index */
	bank_id = ((q->slave.bus) << 1) + (q->slave.cs);

	writel(q->bank_memmap_phy[bank_id] + addr, base + QUADSPI_SFAR);
	writel(QUADSPI_RBCT_WMRK_MASK | QUADSPI_RBCT_RXBRD_USEIPS,
			base + QUADSPI_RBCT);
	writel(reg | QUADSPI_MCR_CLR_RXF_MASK, base + QUADSPI_MCR);

	/* trigger the LUT now */
	seqid = fsl_qspi_get_seqid(q, cmd);
	writel((seqid << QUADSPI_IPCR_SEQID_SHIFT) | len, base + QUADSPI_IPCR);

	/* Wait until completed */
	err = fsl_qspi_wait_to_complete(q);
	if (!err)
		err = -1;
	else
		err = 0;

	/* restore the MCR */
	writel(reg, base + QUADSPI_MCR);

	/* After switch BANK, AHB buffer should also be invalid. */
	if ((OPCODE_SE == cmd) || (OPCODE_PP == cmd) ||
	    (OPCODE_BE_4K == cmd) || (OPCODE_WREAR == cmd) ||
	    (OPCODE_BRWR == cmd))
		fsl_qspi_invalid(q);
	return err;
}

/*
 * An IC bug makes us to re-arrange the 32-bit data.
 * The following chips, such as IMX6SLX, have fixed this bug.
 */
static inline uint32_t fsl_qspi_endian_xchg(struct fsl_qspi *q, uint32_t a)
{
	return a;
}

/* Read out the data from the AHB buffer. */
static void fsl_qspi_ahb_read(struct fsl_qspi *q,
	unsigned int addr, int len, uint8_t *rxbuf)
{
	int bank_id;

	/* get the bank index */
	bank_id = ((q->slave.bus) << 1) + (q->slave.cs);

	/* Read out the data directly from the AHB buffer.*/
	memcpy(rxbuf, (uint8_t *)(q->bank_memmap_phy[bank_id] + addr), len);
}

/* Read out the data from the QUADSPI_RBDR buffer registers. */
static void fsl_qspi_ip_read(struct fsl_qspi *q, int len, uint8_t *rxbuf)
{
	uint32_t tmp;
	int i = 0;

	while (len > 0) {
		tmp = readl(q->iobase + QUADSPI_RBDR + i * 4);
		tmp = fsl_qspi_endian_xchg(q, tmp);

		if (len >= 4) {
			memcpy(rxbuf, &tmp, 4);
			rxbuf += 4;
		} else {
			memcpy(rxbuf, &tmp, len);
			break;
		}

		len -= 4;
		i++;
	}
}

/* Write data to the QUADSPI_TBDR buffer registers. */
static void fsl_qspi_write_data(struct fsl_qspi *q, int len, uint8_t* txbuf)
{
	uint32_t tmp;
	uint32_t t1, t2;
	int j;

	/* clear the TX FIFO. */
	tmp = readl(q->iobase + QUADSPI_MCR);
	writel(tmp | QUADSPI_MCR_CLR_TXF_MASK, q->iobase + QUADSPI_MCR);

	/* fill the TX data to the FIFO */
	t2 = len % 4;
	t1 = len >> 2; /* 4 Bytes aligned */

	for (j = 0; j < t1; j++) {
		memcpy(&tmp, txbuf, 4);
		tmp = fsl_qspi_endian_xchg(q, tmp);
		writel(tmp, q->iobase + QUADSPI_TBDR);
		txbuf += 4;
	}

	if (t2) {
		tmp = 0;
		memcpy(&tmp, txbuf, t2);
		tmp = fsl_qspi_endian_xchg(q, tmp);
		writel(tmp, q->iobase + QUADSPI_TBDR);
	}

#if defined(CONFIG_MX7D) || defined(CONFIG_MX6UL)
	uint32_t t3;
	/* iMX7D and MX6UL TXFIFO must be at least 16 bytes*/
	t3 = t1 + ((t2 + 3) >> 2);
	for (; t3 < 4; t3++)
		writel(0, q->iobase + QUADSPI_TBDR);
#endif

}
      
/* see the spi_flash_read_write() */
int  spi_xfer(struct spi_slave *slave, unsigned int bitlen, const void *dout,
		void *din, unsigned long flags)
{
	struct fsl_qspi *q = container_of(slave, struct fsl_qspi, slave);
	int len = bitlen / 8;
	int ret = 0;
	uint8_t *buf;
	static uint8_t opcode;
	static unsigned int addr;

	if (!opcode && (flags & SPI_XFER_BEGIN)) {
		/* spi_xfer for cmd phase */
		buf = (uint8_t *)dout;
		opcode = buf[0];
		if (len > 1)
			addr = buf[1] << 16 | buf[2] << 8 | buf[3];

		/* if transfer cmd only */
		if (flags & SPI_XFER_END)
			ret = fsl_qspi_runcmd(q, opcode, addr, 0);

	} else if (opcode) {
		/* spi_xfer for data phase */
		if (din) {
			/* read*/
			buf = (uint8_t *)din;
			if (OPCODE_FAST_READ == opcode) {
				fsl_qspi_ahb_read(q, addr, len, buf);
			} else {
				ret = fsl_qspi_runcmd(q, opcode, addr, len);
				if (!ret)
					fsl_qspi_ip_read(q, len, buf);
			}
		} else if (dout) {
			/* write data, prepare data first */
			buf = (uint8_t *)dout;
			fsl_qspi_write_data(q, len, buf);
			/* then run page program cmd */
			ret = fsl_qspi_runcmd(q, opcode, addr, len);
		}
	}

	if (ret || (flags & SPI_XFER_END)) {
		opcode = 0;
		addr = 0;
	}

	return ret;
}

/* spi_do_alloc_slave */
void *spi_do_alloc_slave(int offset, int size, unsigned int bus,
			 unsigned int cs)
{
	struct spi_slave *slave;
	void *ptr;

  //ptr = malloc(size);
  static struct fsl_qspi f;
  ptr = (void*)&f;
  
	if (ptr) {
		memset(ptr, '\0', size);
		slave = (struct spi_slave *)(ptr);
		slave->bus = bus;
		slave->cs = cs;
		slave->wordlen = SPI_DEFAULT_WORDLEN;
	}

	return ptr;
}

/* spi_flash_addr */
void spi_flash_addr(uint32_t addr, uint8_t *cmd)
{
	/* cmd[0] is actual command */
	cmd[1] = addr >> 16;
	cmd[2] = addr >> 8;
	cmd[3] = addr >> 0;
}

/* spi_flash_read_write */
int spi_flash_read_write(struct spi_slave *spi,
				uint8_t *cmd, size_t cmd_len,
				const uint8_t *data_out, uint8_t *data_in,
				size_t data_len)
{
	unsigned long flags = SPI_XFER_BEGIN;
	int ret;

	if (data_len == 0) flags |= SPI_XFER_END;
  
	ret = spi_xfer(spi, cmd_len * 8, cmd, NULL, flags);
	if ((!ret)&&(data_len != 0)) {
		ret = spi_xfer(spi, data_len * 8, data_out, data_in, SPI_XFER_END);
	}

	return ret;
}

