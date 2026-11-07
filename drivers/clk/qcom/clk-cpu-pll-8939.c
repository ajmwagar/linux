// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm MSM8937/QM215 APCS A53SS PLL clocks.
 *
 * The 4.9 vendor GCC driver defines the APCS C0/C1 PLLs as SR2 PLLs with
 * L/M/N frequency tables.  The mainline QCOM clk-pll driver already provides
 * the matching SR2 ops, so this driver only exposes the two APCS PLL register
 * blocks and their frequency tables.
 */

#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "clk-pll.h"

#define APCS_PLL_MODE		0x00
#define APCS_PLL_L_VAL		0x04
#define APCS_PLL_M_VAL		0x08
#define APCS_PLL_N_VAL		0x0c
#define APCS_PLL_USER_CTL	0x10
#define APCS_PLL_STATUS		0x1c

#define APCS_PLL_LOCK_BIT	16

#define PLL_ENTRY(freq_, lval_) { .freq = (freq_), .l = (lval_), .m = 0, .n = 1 }

/*
 * 4.9 qcom,speed1-bin-v0-c0/c1 tables.  The APCS PLL parent is the 19.2 MHz
 * XO; m=0/n=1 means the mainline SR2 recalc computes parent_rate * l.
 */
static const struct pll_freq_tbl apcs_c0_freq[] = {
	PLL_ENTRY(249600000, 13),
	PLL_ENTRY(307200000, 16),
	PLL_ENTRY(345600000, 18),
	PLL_ENTRY(384000000, 20),
	PLL_ENTRY(460800000, 24),
	PLL_ENTRY(499200000, 26),
	PLL_ENTRY(518400000, 27),
	PLL_ENTRY(768000000, 40),
	PLL_ENTRY(806400000, 42),
	PLL_ENTRY(844800000, 44),
	PLL_ENTRY(883200000, 46),
	PLL_ENTRY(902400000, 47),
	PLL_ENTRY(921600000, 48),
	PLL_ENTRY(998400000, 52),
	PLL_ENTRY(1094400000, 57),
	PLL_ENTRY(1171200000, 61),
	PLL_ENTRY(1209600000, 63),
	PLL_ENTRY(1305600000, 68),
	PLL_ENTRY(1459200000, 76),
	{ }
};

static const struct pll_freq_tbl apcs_c1_freq[] = {
	PLL_ENTRY(345600000, 18),
	PLL_ENTRY(422400000, 22),
	PLL_ENTRY(499200000, 26),
	PLL_ENTRY(652800000, 34),
	PLL_ENTRY(729600000, 38),
	PLL_ENTRY(806400000, 42),
	PLL_ENTRY(844800000, 44),
	PLL_ENTRY(883200000, 46),
	PLL_ENTRY(960000000, 50),
	PLL_ENTRY(998400000, 52),
	PLL_ENTRY(1036800000, 54),
	PLL_ENTRY(1094400000, 57),
	PLL_ENTRY(1113600000, 58),
	PLL_ENTRY(1190400000, 62),
	PLL_ENTRY(1209600000, 63),
	PLL_ENTRY(1248000000, 65),
	PLL_ENTRY(1267200000, 66),
	PLL_ENTRY(1305600000, 68),
	PLL_ENTRY(1344000000, 70),
	PLL_ENTRY(1401000000, 73),
	PLL_ENTRY(1420800000, 74),
	PLL_ENTRY(1440000000, 75),
	PLL_ENTRY(1459200000, 76),
	PLL_ENTRY(1497600000, 78),
	PLL_ENTRY(1516800000, 79),
	PLL_ENTRY(1536000000, 80),
	PLL_ENTRY(1651200000, 86),
	PLL_ENTRY(1708800000, 89),
	PLL_ENTRY(1804800000, 94),
	PLL_ENTRY(1958400000, 102),
	PLL_ENTRY(2016000000, 105),
	{ }
};

struct apcs_pll_8939 {
	struct clk_pll pll;
	struct regmap *regmap;
	struct clk_init_data init;
	const char *parent_names[1];
};

static const struct regmap_config apcs_pll_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x20,
};

static int apcs_pll_8939_register(struct platform_device *pdev,
				  struct apcs_pll_8939 *p,
				  const char *name,
				  const struct pll_freq_tbl *freq_tbl,
				  const char *reg_name)
{
	struct device *dev = &pdev->dev;
	void __iomem *base;

	base = devm_platform_ioremap_resource_byname(pdev, reg_name);
	if (IS_ERR(base))
		return PTR_ERR(base);

	p->regmap = devm_regmap_init_mmio(dev, base, &apcs_pll_regmap_config);
	if (IS_ERR(p->regmap))
		return PTR_ERR(p->regmap);

	p->pll.mode_reg = APCS_PLL_MODE;
	p->pll.l_reg = APCS_PLL_L_VAL;
	p->pll.m_reg = APCS_PLL_M_VAL;
	p->pll.n_reg = APCS_PLL_N_VAL;
	p->pll.config_reg = APCS_PLL_USER_CTL;
	p->pll.status_reg = APCS_PLL_STATUS;
	p->pll.status_bit = APCS_PLL_LOCK_BIT;
	p->pll.freq_tbl = freq_tbl;
	p->parent_names[0] = "xo_board";
	p->init.name = name;
	p->init.ops = &clk_pll_sr2_ops;
	p->init.parent_names = p->parent_names;
	p->init.num_parents = 1;
	p->init.flags = CLK_GET_RATE_NOCACHE;
	p->pll.clkr.regmap = p->regmap;
	p->pll.clkr.hw.init = &p->init;

	return devm_clk_register_regmap(dev, &p->pll.clkr);
}

static int apcs_pll_8939_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct clk_hw_onecell_data *data;
	struct apcs_pll_8939 *c0, *c1;
	int ret;

	data = devm_kzalloc(dev, struct_size(data, hws, 2), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	data->num = 2;

	c0 = devm_kzalloc(dev, sizeof(*c0), GFP_KERNEL);
	c1 = devm_kzalloc(dev, sizeof(*c1), GFP_KERNEL);
	if (!c0 || !c1)
		return -ENOMEM;

	ret = apcs_pll_8939_register(pdev, c0, "a53ss_c0_pll",
				     apcs_c0_freq, "c0");
	if (ret)
		return ret;

	ret = apcs_pll_8939_register(pdev, c1, "a53ss_c1_pll",
				     apcs_c1_freq, "c1");
	if (ret)
		return ret;

	data->hws[0] = &c0->pll.clkr.hw;
	data->hws[1] = &c1->pll.clkr.hw;

	return devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, data);
}

static const struct of_device_id apcs_pll_8939_match[] = {
	{ .compatible = "qcom,cpu-pll-8939" },
	{ }
};
MODULE_DEVICE_TABLE(of, apcs_pll_8939_match);

static struct platform_driver apcs_pll_8939_driver = {
	.probe = apcs_pll_8939_probe,
	.driver = {
		.name = "qcom-cpu-pll-8939",
		.of_match_table = apcs_pll_8939_match,
	},
};
module_platform_driver(apcs_pll_8939_driver);

MODULE_DESCRIPTION("Qualcomm MSM8937/QM215 APCS A53SS PLL clocks");
MODULE_LICENSE("GPL");
