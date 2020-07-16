/*
 * SPDX-License-Identifier: GPL
 * Copyright 2020 ZGOO, Inc.
 * ZGOO SPI controller driver (master mode only)
 * Author: ZGOO, Inc.
 * zgoo@zgoo.com
 *
 * Based on spi_altera.c, which is:
 * Copyright (C) 2008 Thomas Chou <thomas@wytron.com.tw>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
 


#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
//#include <linux/spi/spi_bitbang.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/io.h>

#include <linux/printk.h>

#define ZGOO_SPI_NAME "zgoo_spi"

#define ZG_SPI_RX_0          0x00
#define ZG_SPI_RX_1          0x04
#define ZG_SPI_RX_2          0x08
#define ZG_SPI_RX_3          0x0C
#define ZG_SPI_TX_0          0x00
#define ZG_SPI_TX_1          0x04
#define ZG_SPI_TX_2          0x08
#define ZG_SPI_TX_3          0x0C                       
#define ZG_SPI_CTRL          0x10               /*control and status*/
#define ZG_SPI_DIVIDER       0x14               /*divider*/
#define ZG_SPI_SS            0x18               /*slave select*/

#define ZG_SPI_CTRL_RESERVED    0x80
#define ZG_SPI_CTRL_GO_BSY      0x100           /*set 1,to start transfer.auto clear after transfet finished*/
#define ZG_SPI_CTRL_RX_NEG      0x200           /*set 1,miso signal is latched on the falling edge of a sclk*/
#define ZG_SPI_CTRL_TX_NEG      0x400           /*set 1,mosi signal is changed on the falling edge of a sclk*/
#define ZG_SPI_CTRL_LSB         0x800           /*set 1,LSB(TXL[0]) sent/received first. */
#define ZG_SPI_CTRL_IE          0x1000          /*set 1,interrupt active after a transfer is finished*/
#define ZG_SPI_CTRL_ASS         0x2000          /*set 1,ss signals automatically*/


struct zgoo_spi {
	void __iomem *regs;
	int irq;
	int len;
	int count;
	int bytes_per_word;
	unsigned long imr;
	const unsigned char *tx;
	unsigned char *rx;
};

static inline struct zgoo_spi *zgoo_spi_to_hw(struct spi_device *sdev)
{
	return spi_master_get_devdata(sdev->master);
}

static void zgoo_spi_set_cs(struct spi_device *spi, bool is_high)
{
	struct zgoo_spi *hw = zgoo_spi_to_hw(spi);

	if (is_high) {
		/*hw->imr &= ~ZG_SPI_CTRL_ASS;*/
		/*writel(hw->imr, hw->regs + ZG_SPI_CTRL);*/
		writel(0, hw->regs + ZG_SPI_SS);
	} else {
		writel(BIT(spi->chip_select), hw->regs + ZG_SPI_SS);
		/*hw->imr |= ZG_SPI_CTRL_ASS;*/
		/*writel(hw->imr, hw->regs + ZG_SPI_CTRL);*/
	}
}

static void zgoo_spi_tx_word(struct zgoo_spi *hw)
{
	unsigned int txd = 0;

	if (hw->tx) {
		switch (hw->bytes_per_word) {
		case 1:
			txd = hw->tx[hw->count];
			break;
		case 2:
			txd = (hw->tx[hw->count * 2]
				| (hw->tx[hw->count * 2 + 1] << 8));
			break;
		}
	}

	writel(txd, hw->regs + ZG_SPI_TX_0);
}

static void zgoo_spi_rx_word(struct zgoo_spi *hw)
{
	unsigned int rxd;

	rxd = readl(hw->regs + ZG_SPI_RX_0);
	if (hw->rx) {
		switch (hw->bytes_per_word) {
		case 1:
			hw->rx[hw->count] = rxd;
			break;
		case 2:
			hw->rx[hw->count * 2] = rxd;
			hw->rx[hw->count * 2 + 1] = rxd >> 8;
			break;
		}
	}

	hw->count++;
}

static int zgoo_spi_txrx(struct spi_master *master,
	struct spi_device *spi, struct spi_transfer *t)
{
	struct zgoo_spi *hw = spi_master_get_devdata(master);

	hw->tx = t->tx_buf;
	hw->rx = t->rx_buf;
	hw->count = 0;
	hw->bytes_per_word = DIV_ROUND_UP(t->bits_per_word, 8);
	hw->len = t->len / hw->bytes_per_word;

	
	while (hw->count < hw->len) {
		zgoo_spi_tx_word(hw);

		while (!(readl(hw->regs + ZG_SPI_CTRL) &                  
			 ZG_SPI_CTRL_GO_BSY))
			cpu_relax();

		zgoo_spi_rx_word(hw);
	}
	spi_finalize_current_transfer(master);


	return t->len;
}

static int zgoo_spi_probe(struct platform_device *pdev)
{
	struct zgoo_spi *hw;
	struct spi_master *master;
	struct resource *res;
	int err = -ENODEV;

	pr_info("start probe zg-spi@pr_info");
	pr_err("start probe zg-spi@pr_err");

	master = spi_alloc_master(&pdev->dev, sizeof(struct zgoo_spi));
	if (!master)
		return err;

	/* setup the master state. */
	master->bus_num = pdev->id;
	master->num_chipselect = 16;
	master->mode_bits = SPI_CS_HIGH;
	master->bits_per_word_mask = SPI_BPW_RANGE_MASK(1, 16);
	master->dev.of_node = pdev->dev.of_node;
	master->transfer_one = zgoo_spi_txrx;
	master->set_cs = zgoo_spi_set_cs;

	hw = spi_master_get_devdata(master);

	/* find and map our resources */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hw->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(hw->regs)) {
		err = PTR_ERR(hw->regs);
		goto exit;
	}
	/* program defaults into the registers */
	writel(0x01, hw->regs + ZG_SPI_DIVIDER);
	writel(0x2708, hw->regs + ZG_SPI_CTRL);
	/*hw->imr = 0;		// disable spi interrupts */
	/*writel(hw->imr, hw->regs + ZG_SPI_CTRL) & ZG_SPI_CTRL_IE;*/
    
	pr_err("probe done@pr_err");


	if (readl(hw->regs + ZG_SPI_CTRL) & ZG_SPI_CTRL_GO_BSY)
		readl(hw->regs + ZG_SPI_RX_0);	/* flush rxdata */
	/* irq is optional */
    /*
	hw->irq = platform_get_irq(pdev, 0);
	if (hw->irq >= 0) {
		err = devm_request_irq(&pdev->dev, hw->irq, zgoo_spi_irq, 0,
				       pdev->name, master);
		if (err)
			goto exit;
	}
	err = devm_spi_register_master(&pdev->dev, master);
	if (err)
		goto exit;
	dev_info(&pdev->dev, "regs %p, irq %d\n", hw->regs, hw->irq);
    */
	return 0;
exit:
	spi_master_put(master);
	return err;
}

#ifdef CONFIG_OF
static const struct of_device_id zgoo_spi_match[] = {
	{ .compatible = "ZGOO,zg-spi-1.0", },
	{ .compatible = "zgoo,zg-spi-1.0", },
	{},
};
MODULE_DEVICE_TABLE(of, zgoo_spi_match);
#endif /* CONFIG_OF */

static struct platform_driver zgoo_spi_driver = {
	.probe = zgoo_spi_probe,
	.driver = {
		.name = ZGOO_SPI_NAME,
		.pm = NULL,
		.of_match_table = of_match_ptr(zgoo_spi_match),
	},
};
module_platform_driver(zgoo_spi_driver);

MODULE_AUTHOR("ZGOO <zgoo@zgoo.com>");
MODULE_DESCRIPTION("ZGOO SPI driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" ZGOO_SPI_NAME);
