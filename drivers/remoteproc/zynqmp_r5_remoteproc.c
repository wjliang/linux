// SPDX-License-Identifier: GPL-2.0
/*
 * Zynq R5 Remote Processor driver
 *
 * Copyright (C) 2018 Xilinx, Inc.
 *
 */

#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/firmware/xlnx-zynqmp.h>
#include <linux/genalloc.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>
#include <linux/slab.h>

#include "remoteproc_internal.h"

/* IPI reg offsets */
#define TRIG_OFFSET		0x00000000
#define OBS_OFFSET		0x00000004
#define ISR_OFFSET		0x00000010
#define IMR_OFFSET		0x00000014
#define IER_OFFSET		0x00000018
#define IDR_OFFSET		0x0000001C
#define IPI_ALL_MASK		0x0F0F0301

/* RPU IPI mask */
#define RPU_IPI_INIT_MASK	0x00000100
#define RPU_IPI_MASK(n)		(RPU_IPI_INIT_MASK << (n))
#define RPU_0_IPI_MASK		RPU_IPI_MASK(0)
#define RPU_1_IPI_MASK		RPU_IPI_MASK(1)

/* Maximum TCM power nodes IDs */
#define MAX_TCM_PNODES 4

/* Register access macros */
#define reg_read(base, reg) \
	readl(((void __iomem *)(base)) + (reg))
#define reg_write(base, reg, val) \
	writel((val), ((void __iomem *)(base)) + (reg))

#define DEFAULT_FIRMWARE_NAME	"rproc-rpu-fw"

static bool autoboot __read_mostly;

struct zynqmp_r5_rproc_pdata;

/**
 * struct zynqmp_r5_rproc_pdata - zynqmp rpu remote processor instance state
 * @rproc: rproc handle
 * @eemi: eemi operations
 * @workqueue: workqueue for the RPU remoteproc
 * @ipi_base: virt ptr to IPI channel address registers for APU
 * @rpu_mode: RPU core configuration
 * @rpu_id: RPU CPU id
 * @rpu_pnode_id: RPU CPU power domain id
 * @mem_pools: list of gen_pool for firmware mmio_sram memory and their
 *             power domain IDs
 * @mems: list of rproc_mem_entries for firmware
 * @irq: IRQ number
 * @ipi_dest_mask: IPI destination mask for the IPI channel
 */
struct zynqmp_r5_rproc_pdata {
	struct rproc *rproc;
	const struct zynqmp_eemi_ops *eemi;
	struct work_struct workqueue;
	void __iomem *ipi_base;
	enum rpu_oper_mode rpu_mode;
	struct list_head mems;
	u32 ipi_dest_mask;
	u32 rpu_id;
	u32 rpu_pnode_id;
	int irq;
	u32 tcm_pnode_id[MAX_TCM_PNODES];
};

/**
 * r5_boot_addr_config - configure the boot address of R5
 * @pdata: platform data
 * @bootmem: boot from LOVEC or HIVEC
 *
 * This function will set the RPU boot address
 */
static void r5_boot_addr_config(struct zynqmp_r5_rproc_pdata *pdata,
				enum rpu_boot_mem bootmem)
{
	const struct zynqmp_eemi_ops *eemi = pdata->eemi;

	pr_debug("%s: R5 ID: %d, boot_dev %d\n",
		 __func__, pdata->rpu_id, bootmem);

	eemi->ioctl(pdata->rpu_pnode_id, IOCTL_RPU_BOOT_ADDR_CONFIG,
		    bootmem, 0, NULL);
}

/**
 * r5_mode_config - configure R5 operation mode
 * @pdata: platform data
 *
 * configure R5 to split mode or lockstep mode
 * based on the platform data.
 */
static void r5_mode_config(struct zynqmp_r5_rproc_pdata *pdata)
{
	const struct zynqmp_eemi_ops *eemi = pdata->eemi;

	pr_debug("%s: mode: %d\n", __func__, pdata->rpu_mode);

	eemi->ioctl(pdata->rpu_pnode_id, IOCTL_SET_RPU_OPER_MODE,
		    pdata->rpu_mode, 0, NULL);
}

/**
 * disable_ipi - disable IPI
 * @pdata: platform data
 *
 * Disable IPI interrupt
 */
static inline void disable_ipi(struct zynqmp_r5_rproc_pdata *pdata)
{
	/* Disable R5 IPI interrupt */
	if (pdata->ipi_base)
		reg_write(pdata->ipi_base, IDR_OFFSET, pdata->ipi_dest_mask);
}

/**
 * enable_ipi - enable IPI
 * @pdata: platform data
 *
 * Enable IPI interrupt
 */
static inline void enable_ipi(struct zynqmp_r5_rproc_pdata *pdata)
{
	/* Enable R5 IPI interrupt */
	if (pdata->ipi_base)
		reg_write(pdata->ipi_base, IER_OFFSET, pdata->ipi_dest_mask);
}

/**
 * event_notified_idr_cb - event notified idr callback
 * @id: idr id
 * @ptr: pointer to idr private data
 * @data: data passed to idr_for_each callback
 *
 * Pass notification to remoteproc virtio
 *
 * @return: 0. having return is to satisfy the idr_for_each() function
 *          pointer input argument requirement.
 */
static int event_notified_idr_cb(int id, void *ptr, void *data)
{
	struct rproc *rproc = data;

	(void)rproc_vq_interrupt(rproc, id);
	return 0;
}

static void handle_event_notified(struct work_struct *work)
{
	struct rproc *rproc;
	struct zynqmp_r5_rproc_pdata *local;

	local = container_of(work, struct zynqmp_r5_rproc_pdata, workqueue);
	rproc = local->rproc;
	idr_for_each(&rproc->notifyids, event_notified_idr_cb, rproc);
}

static int zynqmp_r5_rproc_start(struct rproc *rproc)
{
	struct device *dev = rproc->dev.parent;
	struct zynqmp_r5_rproc_pdata *local = rproc->priv;
	enum rpu_boot_mem bootmem;
	const struct zynqmp_eemi_ops *eemi = local->eemi;

	/* Set up R5 */
	if ((rproc->bootaddr & 0xF0000000) == 0xF0000000)
		bootmem = PM_RPU_BOOTMEM_HIVEC;
	else
		bootmem = PM_RPU_BOOTMEM_LOVEC;
	dev_info(dev, "RPU boot from %s.",
		 bootmem == PM_RPU_BOOTMEM_HIVEC ? "OCM" : "TCM");

	r5_mode_config(local);
	eemi->force_powerdown(local->rpu_pnode_id,
			      ZYNQMP_PM_REQUEST_ACK_BLOCKING);
	r5_boot_addr_config(local, bootmem);
	eemi->request_wakeup(local->rpu_pnode_id,
			     1, bootmem,
			     ZYNQMP_PM_REQUEST_ACK_NO);

	/* Make sure IPI is enabled */
	enable_ipi(local);

	return 0;
}

/* kick a firmware */
static void zynqmp_r5_rproc_kick(struct rproc *rproc, int vqid)
{
	struct device *dev = rproc->dev.parent;
	struct zynqmp_r5_rproc_pdata *local = rproc->priv;

	dev_dbg(dev, "KICK Firmware to start send messages vqid %d\n", vqid);

	/*
	 * send irq to R5 firmware
	 * Currently vqid is not used because we only got one.
	 */
	if (local->ipi_base)
		reg_write(local->ipi_base, TRIG_OFFSET, local->ipi_dest_mask);
}

/* power off the remote processor */
static int zynqmp_r5_rproc_stop(struct rproc *rproc)
{
	struct device *dev = rproc->dev.parent;
	struct zynqmp_r5_rproc_pdata *local = rproc->priv;
	const struct zynqmp_eemi_ops *eemi = local->eemi;

	dev_dbg(dev, "%s\n", __func__);

	disable_ipi(local);
	eemi->force_powerdown(local->rpu_pnode_id,
			      ZYNQMP_PM_REQUEST_ACK_BLOCKING);

	return 0;
}

static int zynqmp_r5_parse_fw(struct rproc *rproc, const struct firmware *fw)
{
	int ret;

	ret = rproc_elf_load_rsc_table(rproc, fw);
	if (ret == -EINVAL)
		/* No resource table */
		return 0;
	else
		return ret;
}

static struct rproc_ops zynqmp_r5_rproc_ops = {
	.start		= zynqmp_r5_rproc_start,
	.stop		= zynqmp_r5_rproc_stop,
	.kick		= zynqmp_r5_rproc_kick,
	.parse_fw	= zynqmp_r5_parse_fw,
};

static irqreturn_t r5_remoteproc_interrupt(int irq, void *dev_id)
{
	struct device *dev = dev_id;
	struct platform_device *pdev = to_platform_device(dev);
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct zynqmp_r5_rproc_pdata *local = rproc->priv;
	u32 ipi_reg;

	/* Check if there is a kick from R5 */
	ipi_reg = reg_read(local->ipi_base, ISR_OFFSET);
	if (!(ipi_reg & local->ipi_dest_mask))
		return IRQ_NONE;

	dev_dbg(dev, "KICK Linux because of pending message(irq%d)\n", irq);
	reg_write(local->ipi_base, ISR_OFFSET, local->ipi_dest_mask);
	schedule_work(&local->workqueue);

	return IRQ_HANDLED;
}

/* zynqmp_r5_tcm_alloc() - Allocate TCM memories
 *
 * @rproc: pointer to the remoteproc instance
 * @mem: pointer to the remoteproc memory entry
 *
 * Return 0 for success, negative value for failure
 */
static int zynqmp_r5_tcm_alloc(struct rproc *rproc,
			       struct rproc_mem_entry *mem)
{
	struct zynqmp_r5_rproc_pdata *local = rproc->priv;
	struct device *dev = rproc->dev.parent;
	const struct zynqmp_eemi_ops *eemi = local->eemi;
	int tcm_id;

	if (mem->da == 0x0)
		tcm_id = 0;
	else
		tcm_id = 1;

	while (tcm_id < MAX_TCM_PNODES) {
		int ret;

		ret = eemi->request_node(local->tcm_pnode_id[tcm_id],
					 ZYNQMP_PM_CAPABILITY_ACCESS, 0,
					 ZYNQMP_PM_REQUEST_ACK_BLOCKING
					);
		if (ret < 0) {
			dev_err(dev, "Failed to request TCM: %u\n",
				local->tcm_pnode_id[tcm_id]);
			return ret;
		}
		if (mem->len > 0x10000)
			tcm_id += 2;
		else
			break;
	}
	return 0;
}

/* zynqmp_r5_tcm_release() - Release TCM memories
 *
 * @rproc: pointer to the remoteproc instance
 * @mem: pointer to the remoteproc memory entry
 *
 * Return 0 for success, negative value for failure
 */
static int zynqmp_r5_tcm_release(struct rproc *rproc,
			         struct rproc_mem_entry *mem)
{
	struct zynqmp_r5_rproc_pdata *local = rproc->priv;
	const struct zynqmp_eemi_ops *eemi = local->eemi;
	int tcm_id;

	if (mem->da == 0x0)
		tcm_id = 0;
	else
		tcm_id = 1;

	while (tcm_id < MAX_TCM_PNODES) {
		eemi->release_node(local->tcm_pnode_id[tcm_id]);
		if (mem->len > 0x10000)
			tcm_id += 2;
		else
			break;
	}
	return 0;
}

/* zynqmp_r5_get_tcm_memories() - get tcm memories
 * @pdev: pointer to the platform device
 * @pdata: pointer to the remoteproc private data
 *
 * Function to create remoteproc memory entries for TCM memories.
 */
static int zynqmp_r5_get_tcms(struct platform_device *pdev,
			      struct zynqmp_r5_rproc_pdata *pdata)
{
	static const char * const mem_names[] = {"tcm_a", "tcm_b"};
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct rproc *rproc = pdata->rproc;
	int num_mems = 0;
	int i;
	struct property *prop;
	const __be32 *cur;
	u32 val;

	/* Get TCM power node ids */
	i = 0;
	of_property_for_each_u32(np, "tcm-pnode-id", prop, cur, val)
		pdata->tcm_pnode_id[i++] = val;

	/* Create remoteproc memories entries for TCM memories */
	num_mems = ARRAY_SIZE(mem_names);
	for (i = 0; i < num_mems; i++) {
		struct resource *res;
		struct rproc_mem_entry *mem;
		void *va;
		dma_addr_t dma;
		u32 da;
		int len;
		resource_size_t size;

		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   mem_names[i]);
		mem  = devm_kzalloc(dev, sizeof(struct rproc_mem_entry),
				    GFP_KERNEL);
		if (!mem)
			return -ENOMEM;
		/* Map it as normal memory */
		size = resource_size(res);
		len = (int)size;
		va = devm_ioremap_wc(dev, res->start, size);
		if (!va) {
			dev_err(dev, "Unable to map memory region: %pa+%x\n",
				&res->start, len);
			return -ENOMEM;
		}
		dma = (dma_addr_t)res->start;
		/* TCM memory:
		 *   TCM_0: da 0 <-> global addr 0xFFE00000
		 *   TCM_1: da 0 <-> global addr 0xFFE90000
		 */
		da = dma & 0x000FFFFF;
		if (da & 0x80000)
			da -= 0x90000;
		dev_dbg(dev, "%s: va = %p, da = 0x%x dma = 0x%llx\n",
			__func__, va, da, dma);
		mem = rproc_mem_entry_init(dev, va, dma, len, da,
					   zynqmp_r5_tcm_alloc,
					   zynqmp_r5_tcm_release,
					   mem_names[i]);
		if (!mem)
			return -ENOMEM;
		rproc_add_carveout(rproc, mem);
	}
	return 0;
}

/* zynqmp_r5_get_reserved_mems() - get reserved memories
 * @pdev: pointer to the platform device
 * @pdata: pointer to the remoteproc private data
 *
 * Function to create remoteproc memory entries from memory-region
 * property.
 */
static int zynqmp_r5_get_reserved_mems(struct platform_device *pdev,
				       struct zynqmp_r5_rproc_pdata *pdata)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct rproc *rproc = pdata->rproc;
	int num_mems;
	int i;

	num_mems = of_count_phandle_with_args(np, "memory-region", NULL);
	if (num_mems <= 0)
		return 0;
	for (i = 0; i < num_mems; i++) {
		struct device_node *node;
		struct resource res;
		resource_size_t size;
		struct rproc_mem_entry *mem;
		void *va;
		dma_addr_t dma;
		u32 da;
		const char *name;
		int len, ret;

		node = of_parse_phandle(np, "memory-region", i);
		ret = of_device_is_compatible(node, "rproc-prog-memory");
		if (!ret) {
			/* it is DMA memory. */
			dev_info(dev, "%s, dma memory %d\n", __func__, i);
			ret = of_reserved_mem_device_init_by_idx(dev, np, i);
			if (ret) {
				dev_err(dev, "unable to reserve DMA mem.\n");
				return ret;
			}
			continue;
		}
		ret = of_address_to_resource(node, 0, &res);
		if (ret) {
			dev_err(dev, "unable to resolve memory region.\n");
			return ret;
		}
		mem  = devm_kzalloc(dev, sizeof(struct rproc_mem_entry),
				    GFP_KERNEL);
		if (!mem)
			return -ENOMEM;
		/* Map it as normal memory */
		size = resource_size(&res);
		len = (int)size;
		va = devm_ioremap_wc(dev, res.start, size);
		if (!va) {
			dev_err(dev, "Unable to map memory region: %pa+%x\n",
				&res.start, len);
			return -ENOMEM;
		}
		dma = (dma_addr_t)res.start;
		da = (u32)res.start;
		name = of_node_full_name(node);
		dev_dbg(dev, "%s: mem %s, va = %p, da = 0x%x dma = 0x%llx\n",
			__func__, name, va, da, dma);
		mem = rproc_mem_entry_init(dev, va, dma, len, da,
					   NULL, NULL, name);
		if (!mem)
			return -ENOMEM;
		rproc_add_carveout(rproc, mem);
	}
	return 0;
}

/* zynqmp_r5_check_eemi_ops() - check if eemi operations defined
 * @pdata: pointer to the remoteproc private data
 *
 * Function to check if the required eemi operations have been defined.
 */
static int zynqmp_r5_check_eemi_ops(struct zynqmp_r5_rproc_pdata *pdata)
{
	const struct zynqmp_eemi_ops *eemi = zynqmp_pm_get_eemi_ops();

	pdata->eemi = NULL;
	if (!eemi) {
		pr_err("%s: missing required eemi operations\n", __func__);
		return -ENXIO;
	} else if (!eemi->request_node || !eemi->release_node) {
		pr_err("%s: missing eemi request/release node operation\n",
		       __func__);
		return -ENXIO;
	} else if (!eemi->request_wakeup || !eemi->force_powerdown) {
		pr_err("%s: missing eemi wakeup/shutdown operation\n",
		       __func__);
		return -ENXIO;
	} else if (!eemi->ioctl) {
		pr_err("%s: missing eemi ioctl operation\n",
		       __func__);
		return -ENXIO;
	} else {
		pdata->eemi = eemi;
		return 0;
	}
}

static int zynqmp_r5_remoteproc_probe(struct platform_device *pdev)
{
	const unsigned char *prop;
	struct resource *res;
	int ret = 0;
	struct zynqmp_r5_rproc_pdata *local;
	struct rproc *rproc;

	rproc = rproc_alloc(&pdev->dev, dev_name(&pdev->dev),
			    &zynqmp_r5_rproc_ops, NULL,
		sizeof(struct zynqmp_r5_rproc_pdata));
	if (!rproc) {
		dev_err(&pdev->dev, "rproc allocation failed\n");
		return -ENOMEM;
	}
	local = rproc->priv;
	local->rproc = rproc;

	platform_set_drvdata(pdev, rproc);

	/* Check platform management eemi operations */
	ret = zynqmp_r5_check_eemi_ops(local);
	if (ret) {
		dev_err(&pdev->dev, "eemi ops not defined.\n");
		goto rproc_fault;
	}

	/* Override parse_fw op to allow no resource table firmware */
	rproc->ops->parse_fw = zynqmp_r5_parse_fw;

	ret = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "dma_set_coherent_mask: %d\n", ret);
		goto rproc_fault;
	}

	/* Get the RPU power domain id */
	ret = of_property_read_u32(pdev->dev.of_node, "rpu-pnode-id",
				   &local->rpu_pnode_id);
	if (ret) {
		dev_err(&pdev->dev, "No RPU power node ID is specified.\n");
		ret = -EINVAL;
		goto rproc_fault;
	}
	dev_dbg(&pdev->dev, "RPU[%d] pnode_id = %d.\n",
		local->rpu_id, local->rpu_pnode_id);

	prop = of_get_property(pdev->dev.of_node, "core_conf", NULL);
	if (!prop) {
		dev_warn(&pdev->dev, "default core_conf used: lock-step\n");
		prop = "lock-step";
	}

	dev_info(&pdev->dev, "RPU core_conf: %s\n", prop);
	if (!strcmp(prop, "split0")) {
		local->rpu_mode = PM_RPU_MODE_SPLIT;
		local->rpu_id = 0;
		local->ipi_dest_mask = RPU_0_IPI_MASK;
	} else if (!strcmp(prop, "split1")) {
		local->rpu_mode = PM_RPU_MODE_SPLIT;
		local->rpu_id = 1;
		local->ipi_dest_mask = RPU_1_IPI_MASK;
	} else if (!strcmp(prop, "lock-step")) {
		local->rpu_mode = PM_RPU_MODE_LOCKSTEP;
		local->rpu_id = 0;
		local->ipi_dest_mask = RPU_0_IPI_MASK;
	} else {
		dev_err(&pdev->dev, "Invalid core_conf mode provided - %s , %d\n",
			prop, local->rpu_mode);
		ret = -EINVAL;
		goto rproc_fault;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ipi");
	if (res) {
		local->ipi_base = devm_ioremap(&pdev->dev, res->start,
					       resource_size(res));
		if (IS_ERR(local->ipi_base)) {
			pr_err("%s: Unable to map IPI\n", __func__);
			ret = PTR_ERR(local->ipi_base);
			goto rproc_fault;
		}
	} else {
		dev_info(&pdev->dev, "IPI resource is not specified.\n");
	}
	dev_dbg(&pdev->dev, "got ipi base address\n");

	/* Get TCM memories */
	ret = zynqmp_r5_get_tcms(pdev, local);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to get TCM memories.\n");
		goto rproc_fault;
	}
	dev_dbg(&pdev->dev, "got TCM memories\n");
	/* Get reserved memory regions for firmware */
	ret = zynqmp_r5_get_reserved_mems(pdev, local);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to get reserved memories.\n");
		goto rproc_fault;
	}
	dev_dbg(&pdev->dev, "got reserved memories.\n");

	/* Disable IPI before requesting IPI IRQ */
	disable_ipi(local);
	INIT_WORK(&local->workqueue, handle_event_notified);

	/* IPI IRQ */
	if (local->ipi_base) {
		ret = platform_get_irq(pdev, 0);
		if (ret < 0) {
			dev_err(&pdev->dev, "unable to find IPI IRQ\n");
			goto rproc_fault;
		}
		local->irq = ret;
		ret = devm_request_irq(&pdev->dev, local->irq,
				       r5_remoteproc_interrupt, IRQF_SHARED,
				       dev_name(&pdev->dev), &pdev->dev);
		if (ret) {
			dev_err(&pdev->dev, "IRQ %d already allocated\n",
				local->irq);
			goto rproc_fault;
		}
		dev_dbg(&pdev->dev, "notification irq: %d\n", local->irq);
	}

	rproc->auto_boot = autoboot;

#if 0
	ret = dma_coerce_mask_and_coherent(&rproc->dev, DMA_BIT_MASK(64));
	if (ret) {
		dev_err(&pdev->dev, "failed to set rproc dma mask\n");
		goto rproc_fault;
	}
#endif
	ret = rproc_add(local->rproc);
	if (ret) {
		dev_err(&pdev->dev, "rproc registration failed\n");
		goto rproc_fault;
	}
	return ret;

rproc_fault:
	rproc_free(local->rproc);

	return ret;
}

static int zynqmp_r5_remoteproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);

	dev_info(&pdev->dev, "%s\n", __func__);

	rproc_del(rproc);
	of_reserved_mem_device_release(&pdev->dev);
	rproc_free(rproc);

	return 0;
}

/* Match table for OF platform binding */
static const struct of_device_id zynqmp_r5_remoteproc_match[] = {
	{ .compatible = "xlnx,zynqmp-r5-remoteproc-1.0", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, zynqmp_r5_remoteproc_match);

static struct platform_driver zynqmp_r5_remoteproc_driver = {
	.probe = zynqmp_r5_remoteproc_probe,
	.remove = zynqmp_r5_remoteproc_remove,
	.driver = {
		.name = "zynqmp_r5_remoteproc",
		.of_match_table = zynqmp_r5_remoteproc_match,
	},
};
module_platform_driver(zynqmp_r5_remoteproc_driver);

module_param_named(autoboot,  autoboot, bool, 0444);
MODULE_PARM_DESC(autoboot,
		 "enable | disable autoboot. (default: true)");

MODULE_AUTHOR("Jason Wu <j.wu@xilinx.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ZynqMP R5 remote processor control driver");
