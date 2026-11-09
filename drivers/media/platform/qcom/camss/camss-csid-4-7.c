// SPDX-License-Identifier: GPL-2.0
/*
 * camss-csid-4-7.c
 *
 * Qualcomm MSM Camera Subsystem - CSID (CSI Decoder) Module
 *
 * Copyright (C) 2020 Linaro Ltd.
 */
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>

#include "camss-csid.h"
#include "camss-csid-gen1.h"
#include "camss.h"

#define CAMSS_CSID_CORE_CTRL_0		0x004
#define CAMSS_CSID_CORE_CTRL_1		0x008
#define CAMSS_CSID_RST_CMD		0x010
#define CAMSS_CSID_CID_LUT_VC_n(n)	(0x014 + 0x4 * (n))
#define CAMSS_CSID_CID_n_CFG(n)		(0x024 + 0x4 * (n))
#define CAMSS_CSID_CID_n_CFG_ISPIF_EN	BIT(0)
#define CAMSS_CSID_CID_n_CFG_RDI_EN	BIT(1)
#define CAMSS_CSID_CID_n_CFG_DECODE_FORMAT_SHIFT	4
#define CAMSS_CSID_CID_n_CFG_PLAIN_FORMAT_8		(PLAIN_FORMAT_PLAIN8 << 8)
#define CAMSS_CSID_CID_n_CFG_PLAIN_FORMAT_16		(PLAIN_FORMAT_PLAIN16 << 8)
#define CAMSS_CSID_CID_n_CFG_PLAIN_ALIGNMENT_LSB	(0 << 9)
#define CAMSS_CSID_CID_n_CFG_PLAIN_ALIGNMENT_MSB	(1 << 9)
#define CAMSS_CSID_CID_n_CFG_RDI_MODE_RAW_DUMP		(0 << 10)
#define CAMSS_CSID_CID_n_CFG_RDI_MODE_PLAIN_PACKING	(1 << 10)
#define CAMSS_CSID_IRQ_CLEAR_CMD	0x064
#define CAMSS_CSID_IRQ_MASK		0x068
#define CAMSS_CSID_IRQ_STATUS		0x06c
#define CAT_CSID_IRQ_MASK_VAL		0x7f010800

/*
 * The QM215/8x53 CSID uses the older 3.4.3 layout.  Its general register
 * block is four bytes lower than the 4.7 layout.  Keep the 4.7 defaults
 * for the shared driver and translate only for 8x53.
 */
static inline bool csid_is_3_4_3(const struct csid_device *csid)
{
	return csid->camss->res->version == CAMSS_8x53;
}

static inline u32 csid_reg(const struct csid_device *csid, u32 offset)
{
	return csid_is_3_4_3(csid) ? offset - 4 : offset;
}
#define CAMSS_CSID_TG_CTRL		0x0a8
#define CAMSS_CSID_TG_CTRL_DISABLE	0xa06436
#define CAMSS_CSID_TG_CTRL_ENABLE	0xa06437
#define CAMSS_CSID_TG_VC_CFG		0x0ac
#define CAMSS_CSID_TG_VC_CFG_H_BLANKING		0x3ff
#define CAMSS_CSID_TG_VC_CFG_V_BLANKING		0x7f
#define CAMSS_CSID_TG_DT_n_CGG_0(n)	(0x0b4 + 0xc * (n))
#define CAMSS_CSID_TG_DT_n_CGG_1(n)	(0x0b8 + 0xc * (n))
#define CAMSS_CSID_TG_DT_n_CGG_2(n)	(0x0bc + 0xc * (n))

static void csid_configure_stream(struct csid_device *csid, u8 enable)
{
	struct csid_testgen_config *tg = &csid->testgen;
	u32 sink_code = csid->fmt[MSM_CSID_PAD_SINK].code;
	u32 src_code = csid->fmt[MSM_CSID_PAD_SRC].code;
	u32 val;

	if (enable) {
		struct v4l2_mbus_framefmt *input_format;
		const struct csid_format_info *format;
		u8 vc = 0; /* Virtual Channel 0 */
		u8 cid = vc * 4; /* id of Virtual Channel and Data Type set */
		u8 dt_shift;

		if (tg->enabled) {
			/* Config Test Generator */
			u32 num_bytes_per_line, num_lines;

			input_format = &csid->fmt[MSM_CSID_PAD_SRC];
			format = csid_get_fmt_entry(csid->res->formats->formats,
						    csid->res->formats->nformats,
						    input_format->code);
			num_bytes_per_line = input_format->width * format->bpp * format->spp / 8;
			num_lines = input_format->height;

			/* 31:24 V blank, 23:13 H blank, 3:2 num of active DT */
			/* 1:0 VC */
			val = ((CAMSS_CSID_TG_VC_CFG_V_BLANKING & 0xff) << 24) |
				  ((CAMSS_CSID_TG_VC_CFG_H_BLANKING & 0x7ff) << 13);
			writel_relaxed(val, csid->base + CAMSS_CSID_TG_VC_CFG);

			/* 28:16 bytes per lines, 12:0 num of lines */
			val = ((num_bytes_per_line & 0x1fff) << 16) |
				  (num_lines & 0x1fff);
			writel_relaxed(val, csid->base + CAMSS_CSID_TG_DT_n_CGG_0(0));

			/* 5:0 data type */
			val = format->data_type;
			writel_relaxed(val, csid->base + CAMSS_CSID_TG_DT_n_CGG_1(0));

			/* 2:0 output test pattern */
			val = tg->mode - 1;
			writel_relaxed(val, csid->base + CAMSS_CSID_TG_DT_n_CGG_2(0));
		} else {
			struct csid_phy_config *phy = &csid->phy;

			input_format = &csid->fmt[MSM_CSID_PAD_SINK];
			format = csid_get_fmt_entry(csid->res->formats->formats,
						    csid->res->formats->nformats,
						    input_format->code);

			val = phy->lane_cnt - 1;
			val |= phy->lane_assign << 4;

			writel_relaxed(val, csid->base + CAMSS_CSID_CORE_CTRL_0);

			val = phy->csiphy_id << 17;
			val |= 0x9;

			writel_relaxed(val, csid->base + CAMSS_CSID_CORE_CTRL_1);
		}

		/* Config LUT */

		dt_shift = (cid % 4) * 8;

		val = readl_relaxed(csid->base +
				   csid_reg(csid, CAMSS_CSID_CID_LUT_VC_n(vc)));
		val &= ~(0xff << dt_shift);
		val |= format->data_type << dt_shift;
		writel_relaxed(val, csid->base +
			       csid_reg(csid, CAMSS_CSID_CID_LUT_VC_n(vc)));

		val = CAMSS_CSID_CID_n_CFG_ISPIF_EN;
		val |= CAMSS_CSID_CID_n_CFG_RDI_EN;
		val |= format->decode_format << CAMSS_CSID_CID_n_CFG_DECODE_FORMAT_SHIFT;
		val |= CAMSS_CSID_CID_n_CFG_RDI_MODE_RAW_DUMP;

		if ((sink_code == MEDIA_BUS_FMT_SBGGR10_1X10 &&
		     src_code == MEDIA_BUS_FMT_SBGGR10_2X8_PADHI_LE) ||
		    (sink_code == MEDIA_BUS_FMT_Y10_1X10 &&
		     src_code == MEDIA_BUS_FMT_Y10_2X8_PADHI_LE)) {
			val |= CAMSS_CSID_CID_n_CFG_RDI_MODE_PLAIN_PACKING;
			val |= CAMSS_CSID_CID_n_CFG_PLAIN_FORMAT_16;
			val |= CAMSS_CSID_CID_n_CFG_PLAIN_ALIGNMENT_LSB;
		}

		writel_relaxed(val, csid->base +
			       csid_reg(csid, CAMSS_CSID_CID_n_CFG(cid)));

		if (tg->enabled) {
			val = CAMSS_CSID_TG_CTRL_ENABLE;
			writel_relaxed(val, csid->base + CAMSS_CSID_TG_CTRL);
		}
	} else {
		if (tg->enabled) {
			val = CAMSS_CSID_TG_CTRL_DISABLE;
			writel_relaxed(val, csid->base + CAMSS_CSID_TG_CTRL);
		}
	}
}

static int csid_configure_testgen_pattern(struct csid_device *csid, s32 val)
{
	if (val > 0 && val <= csid->testgen.nmodes)
		csid->testgen.mode = val;

	return 0;
}

/*
 * isr - CSID module interrupt service routine
 * @irq: Interrupt line
 * @dev: CSID device
 *
 * Return IRQ_HANDLED on success
 */
static irqreturn_t csid_isr(int irq, void *dev)
{
	struct csid_device *csid = dev;
	u32 value;

	value = readl_relaxed(csid->base +
			      csid_reg(csid, CAMSS_CSID_IRQ_STATUS));
	writel_relaxed(value, csid->base +
			   csid_reg(csid, CAMSS_CSID_IRQ_CLEAR_CMD));

	if ((value >> 11) & 0x1)
		complete(&csid->reset_complete);

	return IRQ_HANDLED;
}

/*
 * csid_reset - Trigger reset on CSID module and wait to complete
 * @csid: CSID device
 *
 * Return 0 on success or a negative error code otherwise
 */
static int csid_reset(struct csid_device *csid)
{
	int i;
	u32 status;
	unsigned long time;

	reinit_completion(&csid->reset_complete);

	if (csid_is_3_4_3(csid)) {
		/*
		 * CSID 3.4.3 has the reset-done IRQ masked by default.  The vendor
		 * mask value includes reset-done (bit 11) and the CSI2 data-lane
		 * error bits.
		 */
		writel_relaxed(CAT_CSID_IRQ_MASK_VAL,
			       csid->base + csid_reg(csid, CAMSS_CSID_IRQ_MASK));
		writel_relaxed(CAT_CSID_IRQ_MASK_VAL,
			       csid->base + csid_reg(csid, CAMSS_CSID_IRQ_CLEAR_CMD));

		writel_relaxed(0x7fff, csid->base +
			       csid_reg(csid, CAMSS_CSID_RST_CMD));

		/* Poll as well, because interrupt delivery is not reliable here. */
		for (i = 0; i < CSID_RESET_TIMEOUT_MS; i++) {
			status = readl_relaxed(csid->base +
					       csid_reg(csid, CAMSS_CSID_IRQ_STATUS));
			if (status & BIT(11)) {
				dev_info(csid->camss->dev,
					 "CSID reset completed, status=0x%08x\n",
					 status);
				return 0;
			}
			mdelay(1);
		}

		status = readl_relaxed(csid->base +
				       csid_reg(csid, CAMSS_CSID_IRQ_STATUS));
		dev_err(csid->camss->dev,
			"CSID reset timeout, status=0x%08x mask=0x%08x\n",
			status, readl_relaxed(csid->base +
				csid_reg(csid, CAMSS_CSID_IRQ_MASK)));
		return -EIO;
	}

	writel_relaxed(0x7fff, csid->base + CAMSS_CSID_RST_CMD);

	time = wait_for_completion_timeout(&csid->reset_complete,
					   msecs_to_jiffies(CSID_RESET_TIMEOUT_MS));
	if (!time) {
		dev_err(csid->camss->dev, "CSID reset timeout\n");
		return -EIO;
	}

	return 0;
}

static void csid_subdev_init(struct csid_device *csid)
{
	csid->testgen.modes = csid_testgen_modes;
	csid->testgen.nmodes = CSID_PAYLOAD_MODE_NUM_SUPPORTED_GEN1;
}

const struct csid_hw_ops csid_ops_4_7 = {
	.configure_stream = csid_configure_stream,
	.configure_testgen_pattern = csid_configure_testgen_pattern,
	.hw_version = csid_hw_version,
	.isr = csid_isr,
	.reset = csid_reset,
	.src_pad_code = csid_src_pad_code,
	.subdev_init = csid_subdev_init,
};
