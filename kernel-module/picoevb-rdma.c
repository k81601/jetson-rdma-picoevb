/*
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/cdev.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/sizes.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#include "picoevb-rdma-ioctl.h"
#include "picoevb-rdma.h"

#define MODULENAME	"pdfc-pcie-test"

#define BAR_0	0

struct pevb_drvdata {
	u32 num_h2c_chans;
	u64 fpga_ram_size;
};

struct pevb {
	struct pci_dev			*pdev;
	struct device			*dev;
	const struct pevb_drvdata	*drvdata;
	struct device			*devnode;
	dev_t				devt;
	struct cdev			cdev;
	void __iomem * const		*iomap;
	struct semaphore		sem;
};

static struct class *pevb_class;

static u32 pevb_readl(struct pevb *pevb, int bar, u32 reg)
{
	u32 val;

	dev_dbg(&pevb->pdev->dev, "readl(0x%08x)\n", reg);
	val = readl(pevb->iomap[bar] + reg);
	dev_dbg(&pevb->pdev->dev, "readl(0x%08x) -> 0x%08x\n", reg, val);
	return val;
}

static void pevb_writel(struct pevb *pevb, int bar, u32 val, u32 reg)
{
	dev_dbg(&pevb->pdev->dev, "write(0x%08x, 0x%08x)\n", val, reg);
	writel(val, pevb->iomap[bar] + reg);
}

static int pevb_fops_open(struct inode *inode, struct file *filep)
{
	// struct pevb *pevb = container_of(inode->i_cdev, struct pevb, cdev);
	// struct pevb_file *pevb_file;

	printk("Open dummy file\n")

	// pevb_file = kzalloc(sizeof(*pevb_file), GFP_KERNEL);
	// if (!pevb_file)
	// 	return -ENOMEM;

	// pevb_file->pevb = pevb;

	// filep->private_data = pevb_file;

	return 0;
}

static int pevb_fops_release(struct inode *inode, struct file *filep)
{
	// struct pevb_file *pevb_file = filep->private_data;

	printk("close dummy file\n")
	// kfree(pevb_file);

	return 0;
}


static irqreturn_t pevb_irq_handler(int irq, void *data)
{
	// struct pevb *pevb = data;
	// u32 reg, status;
	irqreturn_t ret = IRQ_NONE;
	
	printk("Interrupt")

	return ret;
}

static int pevb_ioctl_led(struct pevb_file *pevb_file, unsigned long arg)
{
	// struct pevb *pevb = pevb_file->pevb;

	// pevb_writel(pevb, BAR_GPIO, arg, 0);
	return 0;
}

static long pevb_fops_unlocked_ioctl(struct file *filep, unsigned int cmd,
	unsigned long arg)
{
	struct pevb_file *pevb_file = filep->private_data;

	switch (cmd) {
	case PICOEVB_IOC_LED:
		printk("ioct PICOEVB_IOC_LED\n")
		return pevb_ioctl_led(pevb_file, arg);
	default:
		return -EINVAL;
	}
}

static const struct file_operations pevb_fops = {
	.owner		= THIS_MODULE,
	.open		= pevb_fops_open,
	.release	= pevb_fops_release,
	.unlocked_ioctl	= pevb_fops_unlocked_ioctl,
};

static int pevb_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct pevb *pevb;
	int ret;

	pevb = devm_kzalloc(&pdev->dev, sizeof(*pevb), GFP_KERNEL);
	if (!pevb)
		return -ENOMEM;
	pci_set_drvdata(pdev, pevb);
	pevb->pdev = pdev;
	pevb->drvdata = (const struct pevb_drvdata *)ent->driver_data;


	sema_init(&pevb->sem, 1);

	ret = alloc_chrdev_region(&pevb->devt, 0, 1, MODULENAME);
	if (ret < 0) {
		dev_err(&pdev->dev, "alloc_chrdev_region(): %d\n", ret);
		return ret;
	}

	cdev_init(&pevb->cdev, &pevb_fops);
	ret = cdev_add(&pevb->cdev, pevb->devt, 1);
	if (ret < 0) {
		dev_err(&pdev->dev, "cdev_add(): %d\n", ret);
		goto err_unregister_chrdev_region;
	}

	pevb->devnode = device_create(pevb_class, &pevb->pdev->dev, pevb->devt,
		NULL, "picoevb");
	if (!pevb->devnode) {
		ret = -ENOMEM;
		goto err_cdev_del;
	}

	ret = pcim_enable_device(pdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "pci_enable_device(): %d\n", ret);
		goto err_device_destroy;
	}

	ret = pcim_iomap_regions(pdev, BIT(BAR_0),
		MODULENAME);
	if (ret < 0) {
		dev_err(&pdev->dev, "pcim_iomap_regions(): %d\n", ret);
		goto err_device_destroy;
	}
	pevb->iomap = pcim_iomap_table(pdev);

	pci_set_master(pdev);
    pci_set_dma_mask(pdev, 0xffffffffffffffffU);

	ret = request_irq(pdev->irq, pevb_irq_handler, IRQF_SHARED,
		dev_name(&pdev->dev), pevb);
	if (ret) {
		dev_err(&pdev->dev, "request_irq(): %d\n", ret);
		goto err_clear_master;
	}

	return 0;

err_clear_master:
	pci_clear_master(pdev);
err_device_destroy:
	device_destroy(pevb_class, pevb->devt);
err_cdev_del:
	cdev_del(&pevb->cdev);
err_unregister_chrdev_region:
	unregister_chrdev_region(pevb->devt, 1);
	return ret;
}

static void pevb_remove(struct pci_dev *pdev)
{
	struct pevb *pevb = pci_get_drvdata(pdev);

	free_irq(pdev->irq, pevb);
	pci_clear_master(pdev);
	device_destroy(pevb_class, pevb->devt);
	cdev_del(&pevb->cdev);
	unregister_chrdev_region(pevb->devt, 1);
	// pdev->dev.dma_parms = NULL;
}

static void pevb_shutdown(struct pci_dev *pdev)
{
}

#define XILINX_VENDOR_ID		0x10EE
#define DEVICE_ID				0x7011
#define SUBDEVICE_ID			0x0007


static const struct pci_device_id pevb_pci_ids[] = {
	{
		.vendor = XILINX_VENDOR_ID,
		.device = DEVICE_ID,
		.subvendor = XILINX_VENDOR_ID,
		.subdevice = SUBDEVICE_ID,
	},
	{ },
};
MODULE_DEVICE_TABLE(pci, pevb_pci_ids);

static struct pci_driver pevb_driver = {
	.name		= MODULENAME,
	.id_table	= pevb_pci_ids,
	.probe		= pevb_probe,
	.remove		= pevb_remove,
	.shutdown	= pevb_shutdown,
};

static int __init pevb_init(void)
{
	int ret;

	pevb_class = class_create(THIS_MODULE, "picoevb");
	if (!pevb_class)
		return -ENOMEM;

	ret = pci_register_driver(&pevb_driver);
	if (ret)
		class_destroy(pevb_class);

	return ret;
}
module_init(pevb_init);

static void __exit pevb_exit(void)
{
	pci_unregister_driver(&pevb_driver);
	class_destroy(pevb_class);
}
module_exit(pevb_exit);

/*
 * For questions, comments, or support, visit:
 *     http://developer.nvidia.com/embedded-computing
 * To report specific verified bugs, visit:
 *     https://github.com/NVIDIA/jetson-rdma-picoevb/issues
 */
MODULE_AUTHOR("NVIDIA");
MODULE_LICENSE("GPL v2");
