// SPDX-License-Identifier: GPL-2.0-only
/* CAT S22 identification-only diagnostic. Sequence follows stock
 * drivers/soc/qcom/wcnss/wcnss_vreg.c, with bounded polling/retries.
 * No PAS firmware start, XO frequency configuration or Wi-Fi transmission.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

static int cat_iris_id_probe(struct platform_device *pdev)
{
 struct device *dev = &pdev->dev, *pds[2] = {};
 const char *names[] = { "mx", "cx" };
 struct regulator_bulk_data rails[] = {
  { .supply = "vddxo" }, { .supply = "vddrfa" }, { .supply = "vddpa" },
 };
 const int uv[] = { 1800000, 1325000, 3300000 };
 /* L7 supplies XO, digital and PX; use the stock summed load. */
 const int ua[] = { 20000, 100000, 515000 };
 void __iomem *base;
 struct clk *clk;
 u32 val, cmd = BIT(5) | BIT(4), id = 0;
 int ret, i, attached = 0, active = 0, configured = 0, attempt;
 base = devm_platform_ioremap_resource_byname(pdev, "pmu");
 if (IS_ERR(base)) return PTR_ERR(base);
 clk = devm_clk_get(dev, "xo");
 if (IS_ERR(clk)) return PTR_ERR(clk);
 ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(rails), rails);
 if (ret) return ret;
 for (i = 0; i < 2; i++) {
  pds[i] = dev_pm_domain_attach_by_name(dev, names[i]);
  if (IS_ERR_OR_NULL(pds[i])) {
   ret = pds[i] ? PTR_ERR(pds[i]) : -ENODEV;
   goto detach;
  }
  attached++;
 }
 for (i = 0; i < 2; i++) {
  ret = dev_pm_genpd_set_performance_state(pds[i], INT_MAX);
  if (ret) goto power_off;
  ret = pm_runtime_resume_and_get(pds[i]);
  if (ret < 0) {
   dev_pm_genpd_set_performance_state(pds[i], 0);
   goto power_off;
  }
  active++;
 }
 for (i = 0; i < ARRAY_SIZE(rails); i++) {
  ret = regulator_set_voltage(rails[i].consumer, uv[i], uv[i]);
  if (ret) goto unvote;
  configured++;
  ret = regulator_set_load(rails[i].consumer, ua[i]);
  if (ret < 0) goto unvote;
 }
 ret = regulator_bulk_enable(ARRAY_SIZE(rails), rails);
 if (ret) goto unvote;
 ret = clk_prepare_enable(clk);
 if (ret) goto disable_rails;
 /* Resource is PMU 0xa21b000; offsets verified in stock Pronto-v3. */
 writel(0, base + 0x1004);
 writel(cmd, base + 0x1004);
 id = readl(base + 0x1134);
 for (attempt = 0; attempt < 5; attempt++) {
  writel(cmd | BIT(7), base + 0x1004);
  ret = readl_poll_timeout(base + 0x1004, val, !(val & BIT(8)), 10, 100000);
  if (ret) goto disable_clk;
  writel(cmd, base + 0x1004);
  writel((id & 0xffff) | 4, base + 0x1134);
  writel(cmd | BIT(9), base + 0x1004);
  ret = readl_poll_timeout(base + 0x1004, val, !(val & BIT(10)), 10, 100000);
  if (ret) goto disable_clk;
  id = readl(base + 0x1134);
  writel(cmd, base + 0x1004);
  dev_info(dev, "CAT IRIS attempt=%d raw=%08x chip=%04x xo-class=%u\n",
           attempt + 1, id, (id >> 16) & 0xffff, id >> 30);
  switch (id >> 16) {
  case 0x0200: case 0x0300: case 0x0400:
  case 0x5111: case 0x5112: case 0x9101: case 0x9110: case 0x8110:
   ret = 0;
   goto disable_clk;
  }
  ret = -ENODEV;
 }
disable_clk:
 writel(0, base + 0x1004);
 clk_disable_unprepare(clk);
disable_rails:
 regulator_bulk_disable(ARRAY_SIZE(rails), rails);
unvote:
 while (configured--) {
  regulator_set_load(rails[configured].consumer, 0);
  regulator_set_voltage(rails[configured].consumer, 0, INT_MAX);
 }
power_off:
 while (active--) {
  pm_runtime_put_sync(pds[active]);
  dev_pm_genpd_set_performance_state(pds[active], 0);
 }
detach:
 while (attached--) dev_pm_domain_detach(pds[attached], false);
 if (ret) dev_err(dev, "CAT IRIS identification failed: %d\n", ret);
 else dev_info(dev, "CAT IRIS identification complete; power votes released\n");
 return ret;
}
static const struct of_device_id cat_iris_match[] = {
 { .compatible = "cat,s22-iris-id" }, {}
};
MODULE_DEVICE_TABLE(of, cat_iris_match);
static struct platform_driver cat_iris_driver = {
 .probe = cat_iris_id_probe,
 .driver = { .name = "cat-iris-id", .of_match_table = cat_iris_match },
};
module_platform_driver(cat_iris_driver);
MODULE_LICENSE("GPL");
