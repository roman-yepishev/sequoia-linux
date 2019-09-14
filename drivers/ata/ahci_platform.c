/*
 * AHCI SATA platform driver
 *
 * Copyright 2004-2005  Red Hat, Inc.
 *   Jeff Garzik <jgarzik@pobox.com>
 * Copyright 2010  MontaVista Software, LLC.
 *   Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/libata.h>
#include <linux/ahci_platform.h>
#include <linux/clk.h>
#include <mach/reset.h>
#include "ahci.h"
#ifdef CONFIG_ARCH_M86XXX 
/* SATA Clocks */
static struct clk *sata_oob_clk; /* Core clock */
static struct clk *sata_pmu_clk; /* PMU alive clock */
static struct clk *sata_clk;	/* Sata AXI ref clock */
#endif 


static const struct ata_port_info ahci_port_info = {
	.flags		= AHCI_FLAG_COMMON,
	.pio_mask	= ATA_PIO4,
	.udma_mask	= ATA_UDMA6,
	.port_ops	= &ahci_platform_ops,
};
#ifdef CONFIG_PM
static int ahci_platform_suspend(struct platform_device *pdev, pm_message_t state)
{
        struct ata_host *host = platform_get_drvdata(pdev);
	int ret=0;
        if (host)
		ret = ata_host_suspend(host, state);

#ifdef CONFIG_ARCH_M86XXX
	if (!ret) /* sucessfully done the host suspend */
	{
		/* No do the clock disable PMU,OOB,AXI here */
		clk_disable(sata_clk);
		clk_disable(sata_oob_clk);
		clk_disable(sata_pmu_clk);
	}
#endif
	
        return ret;
}

static int ahci_platform_resume(struct platform_device *pdev)
{
        struct ata_host *host = platform_get_drvdata(pdev);

#ifdef CONFIG_ARCH_M86XXX
	/* Do the  clock enable here  PMU,OOB,AXI */
	clk_enable(sata_clk);
	clk_enable(sata_oob_clk);
	clk_enable(sata_pmu_clk);
#endif

        if (host) 
		ata_host_resume(host);

	return 0;
}
#else
#define ahci_platform_suspend NULL
#define ahci_platform_resume NULL
#endif

static int ahci_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ahci_host_priv *hpriv;
	int rc;

#ifdef CONFIG_ARCH_M86XXX
	/* Get the Reference and Enable  the SATA clocks here */

	sata_clk = clk_get(NULL,"sata");
	/* Error Handling , if no SATA(AXI) clock reference: return error */
	if (IS_ERR(sata_clk)) {
		pr_err("%s: Unable to obtain SATA(AXI) clock: %ld\n",__func__,PTR_ERR(sata_clk));
		return PTR_ERR(sata_clk);
 	}

	/*Enable the SATA(AXI) clock here */
        rc = clk_enable(sata_clk);
	if (rc){
		pr_err("%s: SATA(AXI) clock enable failed \n",__func__);
                return rc;
	}
	sata_oob_clk = clk_get(NULL,"sata_oob");
	/* Error Handling , if no SATA_OOB clock reference: return error */
	if (IS_ERR(sata_oob_clk)) {
		pr_err("%s: Unable to obtain SATA_OOB clock: %ld\n",__func__,PTR_ERR(sata_oob_clk));
		return PTR_ERR(sata_oob_clk);
 	}

	sata_pmu_clk = clk_get(NULL,"sata_pmu");
	/* Error Handling , if no SATA_PMU clock reference: return error */
	if (IS_ERR(sata_pmu_clk)) {
		pr_err("%s: Unable to obtain SATA_PMU clock: %ld\n",__func__,PTR_ERR(sata_pmu_clk));
		return PTR_ERR(sata_pmu_clk);
	}
	/*Enable the SATA(PMU and OOB) clocks here */
        rc = clk_enable(sata_oob_clk);
	if (rc){
		pr_err("%s: SATA_OOB clock enable failed \n",__func__);
                return rc;
	}

        rc = clk_enable(sata_pmu_clk);
	if (rc){
		pr_err("%s: SATA_PMU clock enable failed \n",__func__);
		return rc;
	}

	/* Set the SATA PMU clock to 30 MHZ and OOB clock to 125MHZ */
	clk_set_rate(sata_oob_clk,125000000);
	clk_set_rate(sata_pmu_clk,30000000);
	
#endif

	hpriv = ahci_platform_get_resources(pdev);
	if (IS_ERR(hpriv))
		return PTR_ERR(hpriv);

	rc = ahci_platform_enable_resources(hpriv);
	if (rc)
		return rc;

	if (of_device_is_compatible(dev->of_node, "hisilicon,hisi-ahci"))
		hpriv->flags |= AHCI_HFLAG_NO_FBS | AHCI_HFLAG_NO_NCQ;

	rc = ahci_platform_init_host(pdev, hpriv, &ahci_port_info);
	if (rc)
		goto disable_resources;

	return 0;
disable_resources:
	ahci_platform_disable_resources(hpriv);
	return rc;
}

static SIMPLE_DEV_PM_OPS(ahci_pm_ops, ahci_platform_suspend,
			 ahci_platform_resume);

static const struct of_device_id ahci_of_match[] = {
	{ .compatible = "generic-ahci", },
	/* Keep the following compatibles for device tree compatibility */
	{ .compatible = "fsl,ls1021a-ahci", },
	{ .compatible = "snps,spear-ahci", },
	{ .compatible = "snps,exynos5440-ahci", },
	{ .compatible = "ibm,476gtr-ahci", },
	{ .compatible = "snps,dwc-ahci", },
	{ .compatible = "hisilicon,hisi-ahci", },
	{},
};
MODULE_DEVICE_TABLE(of, ahci_of_match);

static struct platform_driver ahci_driver = {
	.probe = ahci_probe,
	.remove = ata_platform_remove_one,
	.driver = {
		.name = "ahci",
		.of_match_table = ahci_of_match,
		.pm = &ahci_pm_ops,
	},
};
module_platform_driver(ahci_driver);

MODULE_DESCRIPTION("AHCI SATA platform driver");
MODULE_AUTHOR("Anton Vorontsov <avorontsov@ru.mvista.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ahci");
