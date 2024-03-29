/*
 * drivers/dma/fsl-qdma.c
 *
 * Copyright 2014-2015 Freescale Semiconductor, Inc.
 *
 * Driver for the Freescale qDMA engine with legacy mode.
 * This module can be found on Freescale LS SoCs.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include "virt-dma.h"

#define FSL_QDMA_DMR		0x0
#define FSL_QDMA_DSR_P		0x4

#define FSL_QDMA_DSR_M		0x10004
#define FSL_QDMA_DLMR		0x10100
#define FSL_QDMA_DLSR		0x10104
#define FSL_QDMA_DLSATR		0x10110
#define FSL_QDMA_DLSAR		0x10114
#define FSL_QDMA_DLDATR		0x10118
#define FSL_QDMA_DLDAR		0x1011c
#define FSL_QDMA_DLBCR		0x10120
#define FSL_QDMA_DLESAD		0x10148
#define FSL_QDMA_DLEDAD		0x1014c

#define FSL_QDMA_DLMR_CS	0x1
#define FSL_QDMA_DLMR_EOSIE	0x200
#define FSL_QDMA_DLMR_EIE	0x40
#define FSL_QDMA_DLSR_TE	0x80
#define FSL_QDMA_DLSR_CH	0x20
#define FSL_QDMA_DLSR_PE	0x10
#define FSL_QDMA_DLSR_CB	0x4
#define FSL_QDMA_DLSR_EOSI	0x2

#define FSL_QDMA_SRTTYPE_R_N	0x40000

struct fsl_qdma_tcd {
	u64	saddr;
	u32	nbytes;
	u64	daddr;
};

struct fsl_qdma_chan_config {
	enum dma_transfer_direction	dir;
	enum dma_slave_buswidth		addr_width;
	u32				burst;
	u32				attr;
};

struct fsl_qdma_desc {
	struct virt_dma_desc		vdesc;
	struct fsl_qdma_chan		*qchan;
	struct fsl_qdma_tcd		tcd;
};

struct fsl_qdma_chan {
	struct virt_dma_chan		vchan;
	struct fsl_qdma_desc		*desc;
	enum dma_status			status;
	u32				slave_id;
	struct fsl_qdma_engine		*qdma;
};

struct fsl_qdma_engine {
	struct dma_device	dma_dev;
	void __iomem		*membase;
	u32			n_chans;
	struct mutex            fsl_qdma_mutex;
	int			controller_irq;
	int			err_irq;
	struct fsl_qdma_chan	chans[];

};

static u32 qdma_readl(struct fsl_qdma_engine *qdma, void __iomem *addr)
{
	return ioread32(addr);
}

static void qdma_writel(struct fsl_qdma_engine *qdma, u32 val,
						void __iomem *addr)
{
	iowrite32(val, addr);
}

static struct fsl_qdma_chan *to_fsl_qdma_chan(struct dma_chan *chan)
{
	return container_of(chan, struct fsl_qdma_chan, vchan.chan);
}

static struct fsl_qdma_desc *to_fsl_qdma_desc(struct virt_dma_desc *vd)
{
	return container_of(vd, struct fsl_qdma_desc, vdesc);
}

static int fsl_qdma_alloc_chan_resources(struct dma_chan *chan)
{
	/*
	 * In QDMA mode, We don't need to do anything.
	 */
	return 0;
}

static void fsl_qdma_free_chan_resources(struct dma_chan *chan)
{
	struct fsl_qdma_chan *fsl_chan = to_fsl_qdma_chan(chan);
	unsigned long flags;
	LIST_HEAD(head);

	spin_lock_irqsave(&fsl_chan->vchan.lock, flags);
	vchan_get_all_descriptors(&fsl_chan->vchan, &head);
	spin_unlock_irqrestore(&fsl_chan->vchan.lock, flags);

	vchan_dma_desc_free_list(&fsl_chan->vchan, &head);
}

static void fsl_qdma_set_tcd_params(struct fsl_qdma_chan *fsl_chan,
					u64 src, u64 dst, u32 nbytes)
{
	void __iomem *addr = fsl_chan->qdma->membase;
	u32 reg;

	/*
	 * Source address.
	 * Represents address bits 31-0 of a 49-bit source address.
	 */
	qdma_writel(fsl_chan->qdma, (u32)src, addr + FSL_QDMA_DLSAR);
	/*
	 * Source address.
	 * Represents address bits 47-32 of a 49-bit source address.
	 */
	reg = qdma_readl(fsl_chan->qdma, addr + FSL_QDMA_DLSATR);
	reg |= (u16)(src >> 32) & 0xffff;
	reg |= FSL_QDMA_SRTTYPE_R_N;
	qdma_writel(fsl_chan->qdma, reg, addr + FSL_QDMA_DLSATR);
	/*
	 * Source address.
	 * Represents address bits 48 of a 49-bit source address.
	 */
	reg = qdma_readl(fsl_chan->qdma, addr + FSL_QDMA_DLESAD);
	reg |= (src >> 48) & 0x1;
	qdma_writel(fsl_chan->qdma, reg, addr + FSL_QDMA_DLESAD);

	/*
	 * Destination address.
	 * Represents address bits 31-0 of a 49-bit destination address.
	 */
	qdma_writel(fsl_chan->qdma, (u32)dst, addr + FSL_QDMA_DLDAR);
	/*
	 * Destination address.
	 * Represents address bits 47-32 of a 49-bit destination address.
	 */
	reg = qdma_readl(fsl_chan->qdma, addr + FSL_QDMA_DLDATR);
	reg |= (u16)(dst >> 32) & 0xffff;
	reg |= FSL_QDMA_SRTTYPE_R_N;
	qdma_writel(fsl_chan->qdma, reg, addr + FSL_QDMA_DLDATR);
	/*
	 * Destination address.
	 * Represents address bits 48 of a 49-bit destination address.
	 */
	reg = qdma_readl(fsl_chan->qdma, addr + FSL_QDMA_DLEDAD);
	reg |= (dst >> 48) & 0x1;
	qdma_writel(fsl_chan->qdma, reg, addr + FSL_QDMA_DLEDAD);

	/*
	 * Byte count.
	 * Contains the number of bytes to transfer.
	 */
	qdma_writel(fsl_chan->qdma, nbytes, addr + FSL_QDMA_DLBCR);
}

static int fsl_qdma_reg_init(struct fsl_qdma_engine *fsl_qdma)
{
	u32 reg;

	reg = qdma_readl(fsl_qdma, fsl_qdma->membase + FSL_QDMA_DLMR);
	reg |= FSL_QDMA_DLMR_EOSIE;
	reg |= FSL_QDMA_DLMR_EIE;
	qdma_writel(fsl_qdma, reg, fsl_qdma->membase + FSL_QDMA_DLMR);
	return 0;
}

static void fsl_qdma_enable_request(struct fsl_qdma_chan *fsl_chan)
{
	void __iomem *addr = fsl_chan->qdma->membase;
	u32 reg;

	reg = qdma_readl(fsl_chan->qdma, addr + FSL_QDMA_DLMR);
	reg |= FSL_QDMA_DLMR_CS;
	qdma_writel(fsl_chan->qdma, reg, addr + FSL_QDMA_DLMR);
}

static struct fsl_qdma_desc *fsl_qdma_alloc_desc(struct fsl_qdma_chan *fsl_chan)
{
	struct fsl_qdma_desc *fsl_desc;

	fsl_desc = kzalloc(sizeof(*fsl_desc), GFP_NOWAIT);

	if (!fsl_desc)
		return NULL;

	fsl_desc->qchan = fsl_chan;

	return fsl_desc;
}

static void fsl_qdma_free_desc(struct virt_dma_desc *vdesc)
{
	struct fsl_qdma_desc *fsl_desc;

	fsl_desc = to_fsl_qdma_desc(vdesc);
	kfree(fsl_desc);
}

static void fsl_qdma_enqueue_desc(struct fsl_qdma_chan *fsl_chan)
{
	struct fsl_qdma_tcd *tcd;
	struct virt_dma_desc *vdesc;

	vdesc = vchan_next_desc(&fsl_chan->vchan);
	if (!vdesc)
		return;

	fsl_chan->desc = to_fsl_qdma_desc(vdesc);
	tcd = &fsl_chan->desc->tcd;
	fsl_qdma_set_tcd_params(fsl_chan, tcd->saddr, tcd->daddr, tcd->nbytes);
	fsl_qdma_enable_request(fsl_chan);
	fsl_chan->status = DMA_IN_PROGRESS;
}

static void fsl_qdma_issue_pending(struct dma_chan *chan)
{
	struct fsl_qdma_chan *fsl_chan = to_fsl_qdma_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&fsl_chan->vchan.lock, flags);

	if (vchan_issue_pending(&fsl_chan->vchan) && !fsl_chan->desc)
		fsl_qdma_enqueue_desc(fsl_chan);

	spin_unlock_irqrestore(&fsl_chan->vchan.lock, flags);
}

static struct dma_async_tx_descriptor *
fsl_qdma_prep_memcpy(struct dma_chan *chan, dma_addr_t dst,
		dma_addr_t src, size_t len, unsigned long flags)
{
	struct fsl_qdma_chan *fsl_chan = to_fsl_qdma_chan(chan);
	struct fsl_qdma_desc *fsl_desc;
	struct fsl_qdma_tcd *tcd;

	fsl_desc = fsl_qdma_alloc_desc(fsl_chan);
	if (!fsl_desc)
		return NULL;

	tcd = &fsl_desc->tcd;
	tcd->saddr = (u64)src;
	tcd->nbytes = (u32)len;
	tcd->daddr = (u64)dst;

	return vchan_tx_prep(&fsl_chan->vchan, &fsl_desc->vdesc, flags);
}

static int fsl_qdma_control(struct dma_chan *chan, enum dma_ctrl_cmd cmd,
		unsigned long arg)
{
	return -ENXIO;
}

static enum dma_status fsl_qdma_tx_status(struct dma_chan *chan,
		dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	return dma_cookie_status(chan, cookie, txstate);
}

static irqreturn_t fsl_qdma_controller_handler(int irq, void *dev_id)
{
	struct fsl_qdma_engine *fsl_qdma = dev_id;
	struct fsl_qdma_chan *fsl_chan = &fsl_qdma->chans[0];
	void __iomem *addr = fsl_qdma->membase;
	u32 reg;

	reg = qdma_readl(fsl_chan->qdma, addr + FSL_QDMA_DLMR);
	reg &= ~FSL_QDMA_DLMR_CS;
	qdma_writel(fsl_chan->qdma, reg, addr + FSL_QDMA_DLMR);

	reg = qdma_readl(fsl_qdma, addr + FSL_QDMA_DLSR);

	if (reg & FSL_QDMA_DLSR_EOSI) {
		spin_lock(&fsl_chan->vchan.lock);
		list_del(&fsl_chan->desc->vdesc.node);
		vchan_cookie_complete(&fsl_chan->desc->vdesc);
		fsl_chan->desc = NULL;
		fsl_chan->status = DMA_COMPLETE;
		spin_unlock(&fsl_chan->vchan.lock);
		fsl_qdma_enqueue_desc(fsl_chan);
	}

	/* Don't clean TE and PE bit if they are set. */
	reg &= ~FSL_QDMA_DLSR_TE & ~FSL_QDMA_DLSR_PE;
	qdma_writel(fsl_qdma, reg, addr + FSL_QDMA_DLSR);

	return IRQ_HANDLED;
}

static irqreturn_t fsl_qdma_controller_handler_err(int irq, void *dev_id)
{
	struct fsl_qdma_engine *fsl_qdma = dev_id;
	u32 reg;

	reg = qdma_readl(fsl_qdma, fsl_qdma->membase + FSL_QDMA_DLSR);

	if (reg & FSL_QDMA_DLSR_TE) {
		dev_err(fsl_qdma->dma_dev.dev,
			"Transfer error. Check your address please!\n");
	}

	if (reg & FSL_QDMA_DLSR_PE) {
		dev_err(fsl_qdma->dma_dev.dev,
			"Programming error. Check your setting please!\n");
	}

	/* Don't clean EOSI bit if it's set. */
	reg &= ~FSL_QDMA_DLSR_EOSI;
	qdma_writel(fsl_qdma, reg, fsl_qdma->membase + FSL_QDMA_DLSR);

	return IRQ_HANDLED;
}

static int fsl_qdma_irq_init(struct platform_device *pdev,
					struct fsl_qdma_engine *fsl_qdma)
{
	int ret;

	fsl_qdma->controller_irq = platform_get_irq_byname(pdev,
							"qdma-tx");
	if (fsl_qdma->controller_irq < 0) {
		dev_err(&pdev->dev, "Can't get qdma controller irq.\n");
		return fsl_qdma->controller_irq;
	}

	ret = devm_request_irq(&pdev->dev, fsl_qdma->controller_irq,
			fsl_qdma_controller_handler, 0, "qDMA controller",
								fsl_qdma);
	if (ret) {
		dev_err(&pdev->dev, "Can't register qDMA controller IRQ.\n");
		return  ret;
	}

	fsl_qdma->err_irq = platform_get_irq_byname(pdev,
							"qdma-err");
	if (fsl_qdma->err_irq < 0) {
		dev_err(&pdev->dev, "Can't get qdma err irq.\n");
		return fsl_qdma->err_irq;
	}

	ret = devm_request_irq(&pdev->dev, fsl_qdma->err_irq,
			fsl_qdma_controller_handler_err, 0, "qDMA err",
								fsl_qdma);
	if (ret) {
		dev_err(&pdev->dev, "Can't register qDMA err IRQ.\n");
		return  ret;
	}
	return 0;
}

static int fsl_qdma_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct fsl_qdma_engine *fsl_qdma;
	struct fsl_qdma_chan *fsl_chan;
	struct resource *res;
	unsigned int len, chans;
	int ret, i;

	ret = of_property_read_u32(np, "channels", &chans);
	if (ret) {
		dev_err(&pdev->dev, "Can't get channels.\n");
		return ret;
	}

	len = sizeof(*fsl_qdma) + sizeof(*fsl_chan) * chans;
	fsl_qdma = devm_kzalloc(&pdev->dev, len, GFP_KERNEL);
	if (!fsl_qdma)
		return -ENOMEM;

	fsl_qdma->n_chans = chans;
	mutex_init(&fsl_qdma->fsl_qdma_mutex);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	fsl_qdma->membase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(fsl_qdma->membase))
		return PTR_ERR(fsl_qdma->membase);

	ret = fsl_qdma_irq_init(pdev, fsl_qdma);
	if (ret)
		return ret;

	INIT_LIST_HEAD(&fsl_qdma->dma_dev.channels);
	for (i = 0; i < fsl_qdma->n_chans; i++) {
		struct fsl_qdma_chan *fsl_chan = &fsl_qdma->chans[i];

		fsl_chan->qdma = fsl_qdma;
		fsl_chan->vchan.desc_free = fsl_qdma_free_desc;
		vchan_init(&fsl_chan->vchan, &fsl_qdma->dma_dev);
	}

	dma_cap_set(DMA_PRIVATE, fsl_qdma->dma_dev.cap_mask);
	dma_cap_set(DMA_SLAVE, fsl_qdma->dma_dev.cap_mask);
	dma_cap_set(DMA_MEMCPY, fsl_qdma->dma_dev.cap_mask);

	fsl_qdma->dma_dev.dev = &pdev->dev;
	fsl_qdma->dma_dev.device_alloc_chan_resources
		= fsl_qdma_alloc_chan_resources;
	fsl_qdma->dma_dev.device_free_chan_resources
		= fsl_qdma_free_chan_resources;
	fsl_qdma->dma_dev.device_tx_status = fsl_qdma_tx_status;
	fsl_qdma->dma_dev.device_control = fsl_qdma_control;
	fsl_qdma->dma_dev.device_prep_dma_memcpy = fsl_qdma_prep_memcpy;
	fsl_qdma->dma_dev.device_issue_pending = fsl_qdma_issue_pending;

	platform_set_drvdata(pdev, fsl_qdma);

	ret = dma_async_device_register(&fsl_qdma->dma_dev);
	if (ret) {
		dev_err(&pdev->dev, "Can't register Freescale qDMA engine.\n");
		return ret;
	}

	ret = fsl_qdma_reg_init(fsl_qdma);
	if (ret) {
		dev_err(&pdev->dev, "Can't Initialize the qDMA engine.\n");
		return ret;
	}

	return 0;
}

static int fsl_qdma_remove(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct fsl_qdma_engine *fsl_qdma = platform_get_drvdata(pdev);

	of_dma_controller_free(np);
	dma_async_device_unregister(&fsl_qdma->dma_dev);
	return 0;
}

static const struct of_device_id fsl_qdma_dt_ids[] = {
	{ .compatible = "fsl,ls-qdma", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fsl_qdma_dt_ids);

static struct platform_driver fsl_qdma_driver = {
	.driver		= {
		.name	= "fsl-qdma",
		.owner  = THIS_MODULE,
		.of_match_table = fsl_qdma_dt_ids,
	},
	.probe          = fsl_qdma_probe,
	.remove		= fsl_qdma_remove,
};

static int __init fsl_qdma_init(void)
{
	return platform_driver_register(&fsl_qdma_driver);
}
subsys_initcall(fsl_qdma_init);

static void __exit fsl_qdma_exit(void)
{
	platform_driver_unregister(&fsl_qdma_driver);
}
module_exit(fsl_qdma_exit);

MODULE_ALIAS("platform:fsl-qdma");
MODULE_DESCRIPTION("Freescale qDMA engine driver");
MODULE_LICENSE("GPL v2");
