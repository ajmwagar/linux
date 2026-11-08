// SPDX-License-Identifier: GPL-2.0
/*
 * camss-csiphy-3-4-2.c
 *
 * Qualcomm MSM Camera Subsystem - CSIPHY 3.4.2 D-PHY (2-phase) module
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/math64.h>

#include "camss-csiphy.h"
#include "camss.h"

#define CSIPHY_342_CMN_CTRL(n)		(0x800 + 0x4 * (n))
#define CSIPHY_342_STATUS(n)		(0x8b0 + 0x4 * (n))
#define CSIPHY_342_CLEAR(n)		(0x858 + 0x4 * (n))
#define CSIPHY_342_IRQ_CMD		0x828
#define CSIPHY_342_GLBL_RESET		0x140
#define CSIPHY_342_HW_VERSION		0x188
#define CSIPHY_342_LANE_OFFSET(n)	(0x200 * (n))
#define CSIPHY_342_CLOCK_OFFSET		0x700

#define CSIPHY_342_CFG1(n)		(0x000 + (n))
#define CSIPHY_342_CFG2(n)		(0x004 + (n))
#define CSIPHY_342_CFG3(n)		(0x008 + (n))
#define CSIPHY_342_CFG4(n)		(0x00c + (n))
#define CSIPHY_342_CFG5(n)		(0x010 + (n))
#define CSIPHY_342_CTRL5(n)		(0x014 + (n))
#define CSIPHY_342_TEST_IMP(n)		(0x01c + (n))
#define CSIPHY_342_MISC1(n)		(0x028 + (n))
#define CSIPHY_342_CFG6(n)		(0x02c + (n))
#define CSIPHY_342_CFG7(n)		(0x030 + (n))
#define CSIPHY_342_CFG8(n)		(0x034 + (n))
#define CSIPHY_342_CFG9(n)		(0x038 + (n))
#define CSIPHY_342_CTRL15(n)		(0x03c + (n))
#define CSIPHY_342_LNCK_CFG1		0x700
#define CSIPHY_342_LNCK_CFG9		0x738

static u8 csiphy_342_get_lane_mask(struct csiphy_lanes_cfg *lane_cfg)
{
	u8 mask = BIT(7);
	int i;

	for (i = 0; i < lane_cfg->num_data; i++)
		mask |= BIT(lane_cfg->data[i].pos);

	return mask;
}

static void csiphy_342_hw_version_read(struct csiphy_device *csiphy,
				       struct device *dev)
{
	u32 version = readl_relaxed(csiphy->base + CSIPHY_342_HW_VERSION);

	dev_dbg(dev, "CSIPHY 3.4.2 HW Version = 0x%08x\n", version);
}

static void csiphy_342_reset(struct csiphy_device *csiphy)
{
	writel_relaxed(0x1, csiphy->base + CSIPHY_342_GLBL_RESET);
	usleep_range(5000, 8000);
	writel_relaxed(0x0, csiphy->base + CSIPHY_342_GLBL_RESET);
}

static u8 csiphy_342_settle_cnt_calc(s64 link_freq, u32 timer_clk_rate)
{
	u32 ui, timer_period, t_hs_prepare_max, t_hs_prepare_zero_min;
	u32 t_hs_settle;

	if (link_freq <= 0 || !timer_clk_rate)
		return 0;

	ui = div_u64(1000000000000LL, link_freq);
	ui /= 2;
	t_hs_prepare_max = 85000 + 6 * ui;
	t_hs_prepare_zero_min = 145000 + 10 * ui;
	t_hs_settle = (t_hs_prepare_max + t_hs_prepare_zero_min) / 2;
	timer_period = div_u64(1000000000000LL, timer_clk_rate);

	return t_hs_settle / timer_period - 1;
}

static void csiphy_342_lanes_enable(struct csiphy_device *csiphy,
				     struct csiphy_config *cfg,
				     s64 link_freq, u8 lane_mask)
{
	struct csiphy_lanes_cfg *lane_cfg = &cfg->csi2->lane_cfg;
	u8 settle_cnt = csiphy_342_settle_cnt_calc(link_freq,
						   csiphy->timer_clk_rate);
	u32 lane_enable = readl_relaxed(csiphy->base + CSIPHY_342_CMN_CTRL(5));
	u32 offset;
	u8 i;

	/* Vendor 3.4.2 D-PHY lane enables use compacted data lane indexes. */
	lane_enable |= BIT(7);
	for (i = 0; i < lane_cfg->num_data; i++)
		lane_enable |= BIT(i * 2);

	writel_relaxed(lane_enable, csiphy->base + CSIPHY_342_CMN_CTRL(5));
	writel_relaxed(BIT(0), csiphy->base + CSIPHY_342_CMN_CTRL(6));
	writel_relaxed(0x6, csiphy->base + CSIPHY_342_CMN_CTRL(7));

	for (i = 0; i < lane_cfg->num_data; i++) {
		offset = CSIPHY_342_LANE_OFFSET(i);
		writel_relaxed(0x8e, csiphy->base + CSIPHY_342_CFG1(offset));
		writel_relaxed(0x08, csiphy->base + CSIPHY_342_CFG2(offset));
		writel_relaxed(settle_cnt, csiphy->base + CSIPHY_342_CFG3(offset));
		writel_relaxed(0x56, csiphy->base + CSIPHY_342_CFG5(offset));
		writel_relaxed(0x60, csiphy->base + CSIPHY_342_CTRL5(offset));
		writel_relaxed(0xe7, csiphy->base + CSIPHY_342_TEST_IMP(offset));
		writel_relaxed(0x00, csiphy->base + CSIPHY_342_MISC1(offset));
		writel_relaxed(0x01, csiphy->base + CSIPHY_342_CFG6(offset));
		writel_relaxed(0x00, csiphy->base + CSIPHY_342_CFG7(offset));
		writel_relaxed(0x03, csiphy->base + CSIPHY_342_CFG8(offset));
		writel_relaxed(0xfe, csiphy->base + CSIPHY_342_CFG9(offset));
		writel_relaxed(0xb8, csiphy->base + CSIPHY_342_CTRL15(offset));
	}

	offset = CSIPHY_342_CLOCK_OFFSET;
	writel_relaxed(0xff, csiphy->base + CSIPHY_342_CFG4(offset));
	writel_relaxed(0x56, csiphy->base + CSIPHY_342_CFG5(offset));
	writel_relaxed(0x60, csiphy->base + CSIPHY_342_CTRL5(offset));
	writel_relaxed(0xe7, csiphy->base + CSIPHY_342_TEST_IMP(offset));
	writel_relaxed(0x04, csiphy->base + CSIPHY_342_MISC1(offset));
	writel_relaxed(0x01, csiphy->base + CSIPHY_342_CFG6(offset));
	writel_relaxed(0x00, csiphy->base + CSIPHY_342_CFG7(offset));
	writel_relaxed(0x03, csiphy->base + CSIPHY_342_CFG8(offset));
	writel_relaxed(0x1f, csiphy->base + CSIPHY_342_CFG9(offset));
	writel_relaxed(0xb8, csiphy->base + CSIPHY_342_CTRL15(offset));
	writel_relaxed(0x80, csiphy->base + CSIPHY_342_LNCK_CFG1);
	writel_relaxed(0x1f, csiphy->base + CSIPHY_342_LNCK_CFG9);

	writel_relaxed(0x2, csiphy->base + CSIPHY_342_CMN_CTRL(0));

	/* Vendor interrupt masks. */
	writel_relaxed(0xff, csiphy->base + CSIPHY_342_CMN_CTRL(11));
	writel_relaxed(0xff, csiphy->base + CSIPHY_342_CMN_CTRL(12));
	writel_relaxed(0xfb, csiphy->base + CSIPHY_342_CMN_CTRL(13));
	writel_relaxed(0xff, csiphy->base + CSIPHY_342_CMN_CTRL(14));
	writel_relaxed(0x7f, csiphy->base + CSIPHY_342_CMN_CTRL(15));
	writel_relaxed(0xff, csiphy->base + CSIPHY_342_CMN_CTRL(16));
	writel_relaxed(0xff, csiphy->base + CSIPHY_342_CMN_CTRL(17));
	writel_relaxed(0xef, csiphy->base + CSIPHY_342_CMN_CTRL(18));
	writel_relaxed(0xff, csiphy->base + CSIPHY_342_CMN_CTRL(19));
	writel_relaxed(0xff, csiphy->base + CSIPHY_342_CMN_CTRL(20));
	writel_relaxed(0xff, csiphy->base + CSIPHY_342_CMN_CTRL(21));
}

static void csiphy_342_lanes_disable(struct csiphy_device *csiphy,
				      struct csiphy_config *cfg)
{
	struct csiphy_lanes_cfg *lane_cfg = &cfg->csi2->lane_cfg;
	u8 i;

	for (i = 0; i < lane_cfg->num_data; i++)
		writel_relaxed(0x00, csiphy->base +
			       CSIPHY_342_CFG2(CSIPHY_342_LANE_OFFSET(i)));

	writel_relaxed(0x00, csiphy->base + CSIPHY_342_CFG2(
			       CSIPHY_342_CLOCK_OFFSET));
	writel_relaxed(0x00, csiphy->base + CSIPHY_342_CMN_CTRL(5));
	writel_relaxed(0x00, csiphy->base + CSIPHY_342_CMN_CTRL(6));
}

static irqreturn_t csiphy_342_isr(int irq, void *dev)
{
	struct csiphy_device *csiphy = dev;
	u32 status;
	int i;

	for (i = 0; i < 11; i++) {
		status = readl_relaxed(csiphy->base + CSIPHY_342_STATUS(i));
		writel_relaxed(status, csiphy->base + CSIPHY_342_CLEAR(i));
	}

	writel_relaxed(0x1, csiphy->base + CSIPHY_342_IRQ_CMD);
	writel_relaxed(0x0, csiphy->base + CSIPHY_342_IRQ_CMD);

	return IRQ_HANDLED;
}

static int csiphy_342_init(struct csiphy_device *csiphy)
{
	return 0;
}

const struct csiphy_hw_ops csiphy_ops_3_4_2 = {
	.get_lane_mask = csiphy_342_get_lane_mask,
	.hw_version_read = csiphy_342_hw_version_read,
	.reset = csiphy_342_reset,
	.lanes_enable = csiphy_342_lanes_enable,
	.lanes_disable = csiphy_342_lanes_disable,
	.isr = csiphy_342_isr,
	.init = csiphy_342_init,
};
