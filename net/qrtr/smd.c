// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015, Sony Mobile Communications Inc.
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
 */

#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/rpmsg.h>
#include <linux/debugfs.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/io.h>

#include "qrtr.h"

/* CATMEM: read-only view of the MSS RAM (debug only) */
#define CATMEM_PHYS 0x86800000UL
#define CATMEM_SIZE 0x5500000UL

static void *catmem_virt;

static ssize_t catmem_read(struct file *filp, char __user *ubuf,
                           size_t count, loff_t *ppos)
{
        size_t n;

        if (!catmem_virt) {
                catmem_virt = memremap(CATMEM_PHYS, CATMEM_SIZE, MEMREMAP_WB);
                if (!catmem_virt)
                        return -ENOMEM;
                pr_info("CATMEM mapped %lx size %lx\n", CATMEM_PHYS, CATMEM_SIZE);
        }
        if (*ppos >= CATMEM_SIZE)
                return 0;
        n = min_t(size_t, count, CATMEM_SIZE - *ppos);
        if (copy_to_user(ubuf, catmem_virt + *ppos, n))
                return -EFAULT;
        *ppos += n;
        return n;
}

static const struct file_operations catmem_fops = {
        .read = catmem_read,
        .llseek = default_llseek,
};

/* CATDIAG: capture the SMD DIAG stream for offline inspection */
#define CATDIAG_SZ (1 << 20)
static u8 *catdiag_buf;
static size_t catdiag_len;
static DEFINE_SPINLOCK(catdiag_lock);

static int catdiag_cb(struct rpmsg_device *rpdev, void *data, int len,
                      void *priv, u32 addr)
{
        unsigned long flags;

        if (!catdiag_buf || len <= 0)
                return 0;
        spin_lock_irqsave(&catdiag_lock, flags);
        if (catdiag_len + len <= CATDIAG_SZ) {
                memcpy(catdiag_buf + catdiag_len, data, len);
                catdiag_len += len;
        }
        spin_unlock_irqrestore(&catdiag_lock, flags);
        return 0;
}

static int catdiag_probe(struct rpmsg_device *rpdev)
{
        if (!catdiag_buf)
                catdiag_buf = kzalloc(CATDIAG_SZ, GFP_KERNEL);
        pr_info("CATDIAG bound %s\n", rpdev->id.name);
        return 0;
}

static void catdiag_remove(struct rpmsg_device *rpdev) { }

static const struct rpmsg_device_id catdiag_id[] = {
        { "DIAG" },
        { "DIAG_CNTL" },
        { "DIAG_CMD" },
        { "DIAG_2" },
        {}
};
MODULE_DEVICE_TABLE(rpmsg, catdiag_id);

static struct rpmsg_driver catdiag_driver = {
        .probe = catdiag_probe,
        .remove = catdiag_remove,
        .callback = catdiag_cb,
        .id_table = catdiag_id,
        .drv = { .name = "catdiag" },
};

static ssize_t catdiag_read(struct file *f, char __user *ubuf,
                            size_t count, loff_t *ppos)
{
        size_t n, total;

        total = READ_ONCE(catdiag_len);
        if (!catdiag_buf || *ppos >= total)
                return 0;
        n = min_t(size_t, count, total - *ppos);
        if (copy_to_user(ubuf, catdiag_buf + *ppos, n))
                return -EFAULT;
        *ppos += n;
        return n;
}

static const struct proc_ops catdiag_pops = {
        .proc_read = catdiag_read,
        .proc_lseek = default_llseek,
};

struct qrtr_smd_dev {
	struct qrtr_endpoint ep;
	struct rpmsg_endpoint *channel;
	struct device *dev;
};

/* from smd to qrtr */
static int qcom_smd_qrtr_callback(struct rpmsg_device *rpdev,
				  void *data, int len, void *priv, u32 addr)
{
	struct qrtr_smd_dev *qdev = dev_get_drvdata(&rpdev->dev);
	int rc;

	if (!qdev)
		return -EAGAIN;

	pr_info("QRTRDBG %s/%s len=%d\n", dev_name(rpdev->dev.parent), rpdev->id.name, len);
	print_hex_dump(KERN_INFO, "QRTRDBG: ", DUMP_PREFIX_NONE, 16, 1,
		       data, min_t(int, len, 256), false);
	rc = qrtr_endpoint_post(&qdev->ep, data, len);
	if (rc == -EINVAL) {
		dev_err(qdev->dev, "invalid ipcrouter packet\n");
		/* return 0 to let smd drop the packet */
		rc = 0;
	}

	return rc;
}

/* from qrtr to smd */
static int qcom_smd_qrtr_send(struct qrtr_endpoint *ep, struct sk_buff *skb)
{
	struct qrtr_smd_dev *qdev = container_of(ep, struct qrtr_smd_dev, ep);
	int rc;

	rc = skb_linearize(skb);
	if (rc)
		goto out;

	rc = rpmsg_send(qdev->channel, skb->data, skb->len);

out:
	if (rc)
		kfree_skb(skb);
	else
		consume_skb(skb);
	return rc;
}

static int qcom_smd_qrtr_probe(struct rpmsg_device *rpdev)
{
	struct qrtr_smd_dev *qdev;
	int rc;

	qdev = devm_kzalloc(&rpdev->dev, sizeof(*qdev), GFP_KERNEL);
	if (!qdev)
		return -ENOMEM;

	qdev->channel = rpdev->ept;
	qdev->dev = &rpdev->dev;
	qdev->ep.xmit = qcom_smd_qrtr_send;

	rc = qrtr_endpoint_register(&qdev->ep, QRTR_EP_NID_AUTO);
	if (rc)
		return rc;

	dev_set_drvdata(&rpdev->dev, qdev);

	dev_dbg(&rpdev->dev, "Qualcomm SMD QRTR driver probed\n");

	return 0;
}

static void qcom_smd_qrtr_remove(struct rpmsg_device *rpdev)
{
	struct qrtr_smd_dev *qdev = dev_get_drvdata(&rpdev->dev);

	qrtr_endpoint_unregister(&qdev->ep);

	dev_set_drvdata(&rpdev->dev, NULL);
}

static const struct rpmsg_device_id qcom_smd_qrtr_smd_match[] = {
	{ "IPCRTR" },
	{}
};

static int __init catdiag_init(void)
{
        proc_create("catdiag", 0400, NULL, &catdiag_pops);
        return register_rpmsg_driver(&catdiag_driver);
}
late_initcall(catdiag_init);

static int __init catmem_init(void)
{
        debugfs_create_file("catmodemram", 0400, NULL, NULL, &catmem_fops);
        return 0;
}
late_initcall(catmem_init);

static struct rpmsg_driver qcom_smd_qrtr_driver = {
	.probe = qcom_smd_qrtr_probe,
	.remove = qcom_smd_qrtr_remove,
	.callback = qcom_smd_qrtr_callback,
	.id_table = qcom_smd_qrtr_smd_match,
	.drv = {
		.name = "qcom_smd_qrtr",
	},
};

module_rpmsg_driver(qcom_smd_qrtr_driver);

MODULE_ALIAS("rpmsg:IPCRTR");
MODULE_DESCRIPTION("Qualcomm IPC-Router SMD interface driver");
MODULE_LICENSE("GPL v2");
