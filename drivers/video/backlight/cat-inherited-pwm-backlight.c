// SPDX-License-Identifier: GPL-2.0-only
/* REVIEW CANDIDATE ONLY: not integrated into a kernel build or DT.
 * CAT S22 inherited illumination consumer. Existing PWM provider owns registers.
 * No GPIO, pinctrl, regulator, DSI, charger or DRM operations here.
 */
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>

struct cat_inherited_bl {
	struct device *dev;
	struct pwm_device *pwm;
	struct mutex lock;
	struct pwm_state inherited;
	struct pwm_state committed;
	bool faulted;
};

static bool cat_state_valid(const struct pwm_state *s)
{
	return s->period && s->duty_cycle <= s->period &&
		s->polarity == PWM_POLARITY_NORMAL;
}

/* Compare to a real inherited/previous hardware snapshot, never PWM cache.
 * Disabled LPG retains the previous duty registers; only output-disable matters.
 * Enabled period must match exactly. A single least-resolution PWM tick allows
 * get_state's ns ceiling -> apply's duty floor round trip (PM8916 >=6 bits).
 * This bound does NOT prove equal physical luminance or pin routing.
 */
static bool cat_state_matches(const struct pwm_state *got,
			      const struct pwm_state *wanted)
{
	u64 delta;

	if (!cat_state_valid(got) || got->enabled != wanted->enabled ||
	    got->polarity != wanted->polarity)
		return false;
	if (!wanted->enabled)
		return true;
	if (got->period != wanted->period)
		return false;
	delta = got->duty_cycle > wanted->duty_cycle ?
		got->duty_cycle - wanted->duty_cycle :
		wanted->duty_cycle - got->duty_cycle;
	return delta <= wanted->period / 63 + 1;
}

static int cat_apply_verify(struct cat_inherited_bl *bl,
			    const struct pwm_state *wanted,
			    struct pwm_state *actual)
{
	int ret;

	ret = pwm_apply_might_sleep(bl->pwm, wanted);
	if (ret)
		return ret;
	/* LPG's apply currently ignores individual regmap write failures. */
	ret = pwm_get_state_hw(bl->pwm, actual);
	if (ret)
		return ret;
	return cat_state_matches(actual, wanted) ? 0 : -EIO;
}

static int cat_bl_update(struct backlight_device *bd)
{
	struct cat_inherited_bl *bl = bl_get_data(bd);
	struct pwm_state wanted, actual = { 0 }, restored = { 0 };
	bool restore = backlight_get_brightness(bd) != 0;
	int ret, rollback;

	mutex_lock(&bl->lock);
	if (bl->faulted) {
		ret = -EIO;
		goto out;
	}
	wanted = bl->inherited;
	if (!restore) {
		wanted.duty_cycle = 0;
		wanted.enabled = false;
	} else if (!wanted.enabled || !wanted.duty_cycle) {
		/* No invented brightness if boot handed us an unlit state. */
		ret = -ENODATA;
		goto out;
	}
	ret = cat_apply_verify(bl, &wanted, &actual);
	if (!ret) {
		bl->committed = actual;
		dev_info(bl->dev, "illumination PWM %s verified (not panel power/routing proof)\n",
			 restore ? "restore" : "off");
		goto out;
	}
	/* No policy success on failure. Restore prior verified physical state,
	 * then latch fault even if rollback works. No silent automatic retry.
	 */
	rollback = cat_apply_verify(bl, &bl->committed, &restored);
	bl->faulted = true;
	dev_err(bl->dev, "PWM transition failed: %d; rollback verify=%d; fault latched\n",
		ret, rollback);
out:
	mutex_unlock(&bl->lock);
	return ret;
}

static int cat_bl_actual(struct backlight_device *bd)
{
	struct cat_inherited_bl *bl = bl_get_data(bd);
	struct pwm_state actual = { 0 };
	int ret;

	mutex_lock(&bl->lock);
	ret = pwm_get_state_hw(bl->pwm, &actual);
	if (!ret)
		ret = cat_state_valid(&actual) ?
			(actual.enabled && actual.duty_cycle ? 1 : 0) : -EIO;
	mutex_unlock(&bl->lock);
	return ret;
}

static const struct backlight_ops cat_bl_ops = {
	.update_status = cat_bl_update,
	.get_brightness = cat_bl_actual,
	/* No BL_CORE_SUSPENDRESUME: no implicit hardware policy transitions. */
};

static int cat_bl_probe(struct platform_device *pdev)
{
	struct backlight_properties props = { 0 };
	struct cat_inherited_bl *bl;
	struct backlight_device *bd;
	int ret;

	bl = devm_kzalloc(&pdev->dev, sizeof(*bl), GFP_KERNEL);
	if (!bl)
		return -ENOMEM;
	bl->dev = &pdev->dev;
	mutex_init(&bl->lock);
	bl->pwm = devm_pwm_get(&pdev->dev, NULL);
	if (IS_ERR(bl->pwm))
		return dev_err_probe(&pdev->dev, PTR_ERR(bl->pwm), "PWM request failed\n");
	ret = pwm_get_state_hw(bl->pwm, &bl->inherited);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "inherited hardware snapshot failed\n");
	if (!cat_state_valid(&bl->inherited))
		return dev_err_probe(&pdev->dev, -EINVAL, "invalid inherited PWM snapshot\n");
	bl->committed = bl->inherited;
	props.type = BACKLIGHT_RAW;
	props.max_brightness = 1;
	props.brightness = bl->inherited.enabled && bl->inherited.duty_cycle ? 1 : 0;
	props.power = BACKLIGHT_POWER_ON;
	bd = devm_backlight_device_register(&pdev->dev, "cat-inner-inherited",
					   &pdev->dev, bl, &cat_bl_ops, &props);
	if (IS_ERR(bd))
		return dev_err_probe(&pdev->dev, PTR_ERR(bd), "backlight registration failed\n");
	/* Intentionally NO pwm_init_state/apply/backlight_update_status at probe.
	 * Remove/shutdown have no hardware callbacks. devm PWM release must be
	 * verified not to disable on exact target core/provider before integration.
	 */
	dev_info(&pdev->dev, "inherited snapshot preserved: period=%llu duty=%llu enabled=%u; binary off/exact-baseline restore\n",
		 bl->inherited.period, bl->inherited.duty_cycle, bl->inherited.enabled);
	return 0;
}

static const struct of_device_id cat_bl_match[] = {
	{ .compatible = "cat,s22-inherited-pwm-backlight" },
	{ }
};
MODULE_DEVICE_TABLE(of, cat_bl_match);
static struct platform_driver cat_bl_driver = {
	.probe = cat_bl_probe,
	.driver = {
		.name = "cat-s22-inherited-backlight",
		.of_match_table = cat_bl_match,
	},
};
module_platform_driver(cat_bl_driver);
MODULE_DESCRIPTION("CAT S22 preserving inherited PWM illumination consumer");
MODULE_LICENSE("GPL");
