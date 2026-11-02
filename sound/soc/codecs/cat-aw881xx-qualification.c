// SPDX-License-Identifier: GPL-2.0-only
/* CAT S22 AW881xx ID1806 codec bring-up. Default playback blocked.
 * Explicit ALSA opt-in: sleepable prepare loads/verifies while muted, then
 * unmutes at the fixed -24dB hardware ceiling. No sysfs playback protocol.
 */
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/firmware.h>
#include <linux/mutex.h>
#include <crypto/sha2.h>
#include <sound/soc.h>
#include "cat-aw881xx-tone-core.h"

struct cat_amp {
	struct gpio_desc *reset;
	struct regmap *map;
	struct device *dev;
	struct mutex lock;
	struct cat_amp_io io;
	bool opened, prepared;
	bool alsa_playback;
};
static int cat_io_read(void *ctx, u8 reg, unsigned int *v)
{
	struct cat_amp *amp = ctx;
	int ret = regmap_read(amp->map, reg, v);

	if (ret)
		dev_err(amp->dev, "qualification read %02x failed: %d\n", reg, ret);
	else if (reg == 0x41)
		dev_dbg(amp->dev, "DSP read=%04x\n", *v);
	return ret;
}
static int cat_io_write(void *ctx, u8 reg, u16 v)
{
	return regmap_write(((struct cat_amp *)ctx)->map, reg, v);
}
static void cat_io_reset(void *ctx, int asserted)
{
	gpiod_set_value_cansleep(((struct cat_amp *)ctx)->reset, asserted);
}
static void cat_io_wait(void *ctx, unsigned int lo, unsigned int hi)
{
	usleep_range(lo, hi);
}
/* No persistent calibration access: caller explicitly supplies a RAM firmware
 * record. Exact CAT ACF SHA256 prevents structurally valid replacement assets.
 */
static int cat_amp_load_muted(struct cat_amp *amp)
{
	static const u8 expected[32] = {
		0x82,0x51,0xeb,0xfd,0xd7,0x9a,0xfb,0x56,0x6a,0x8f,0x27,0xfd,0x89,0x69,0xb8,0x6c,
		0x5c,0x22,0x9d,0xa2,0xfb,0xd9,0xf3,0x06,0xd3,0xe0,0x9e,0x0e,0x06,0x7c,0xba,0x9e
	};
	const struct firmware *acf, *cal;
	struct cat_amp_profile p[2];
	u8 digest[32];
	unsigned int i, value = 0;
	int ret;

	ret = request_firmware(&acf, "cat-s22/aw881xx_acf.bin", amp->dev);
	if (ret) return ret;
	sha256(acf->data, acf->size, digest);
	if (memcmp(digest, expected, 32)) { ret = -EINVAL; goto acf_out; }
	ret = cat_amp_parse(acf->data, acf->size, p);
	if (ret) goto acf_out;
	ret = request_firmware(&cal, "cat-s22/aw_cali.bin", amp->dev);
	if (ret) goto acf_out;
	if (cal->size != 10) { ret = -EINVAL; goto cal_out; }
	for (i = 0; i < 10 && cal->data[i] == ' '; i++) {}
	if (i == 10) { ret = -EINVAL; goto cal_out; }
	for (; i < 10; i++) {
		if (cal->data[i] < '0' || cal->data[i] > '9' || value > 40000) {
			ret = -EINVAL; goto cal_out;
		}
		value = value * 10 + cal->data[i] - '0';
	}
	if (value < 1000 || value > 40000 || value * 4096 / 1000 > 65535) {
		ret = -ERANGE; goto cal_out;
	}
	ret = cat_amp_prepare(&amp->io, p, value * 4096 / 1000);
	if (ret)
		dev_err(amp->dev, "muted preparation failed: %d PID=%04x\n", ret, amp->io.observed_pid);
cal_out:
	release_firmware(cal);
acf_out:
	release_firmware(acf);
	if (ret) cat_io_reset(amp, 1);
	return ret;
}
static void cat_amp_reset(void *p)
{
	gpiod_set_value_cansleep(p, 1); /* active-low reset asserted */
}

static int cat_amp_startup(struct snd_pcm_substream *s, struct snd_soc_dai *dai)
{
	struct cat_amp *amp = snd_soc_component_get_drvdata(dai->component);
	struct gpio_desc *reset = amp->reset;

	if (amp->alsa_playback ||
	    of_property_read_bool(dai->dev->of_node, "cat,reset-held-transport-test")) {
		/* Opening alone never powers the amplifier. */
		if (!reset)
			return -ENODEV;
		mutex_lock(&amp->lock);
		if (amp->opened) {
			mutex_unlock(&amp->lock);
			return -EBUSY;
		}
		cat_amp_reset(reset);
		amp->opened = true;
		mutex_unlock(&amp->lock);
		dev_info(dai->dev, "transport-only open: amplifier reset asserted\n");
		return 0;
	}
	dev_err(dai->dev, "playback blocked: CAT profile/calibration not qualified\n");
	return -EOPNOTSUPP;
}

static void cat_amp_shutdown(struct snd_pcm_substream *s, struct snd_soc_dai *dai)
{
	struct cat_amp *amp = snd_soc_component_get_drvdata(dai->component);

	mutex_lock(&amp->lock);
	/* Reset even if shutdown follows a failed preparation. */
	cat_io_reset(amp, 1);
	amp->opened = amp->prepared = false;
	mutex_unlock(&amp->lock);
}
static int cat_amp_prepare_pcm(struct snd_pcm_substream *s, struct snd_soc_dai *dai)
{
	struct cat_amp *amp = snd_soc_component_get_drvdata(dai->component);
	int ret = 0;

	if (!amp->alsa_playback)
		return 0; /* Explicit reset-held transport test remains silent. */
	/* ASoC runs CPU DAI prepare first: Q6AFE supplies I2S clocks here.
	 * This callback is sleepable; never put firmware/I2C work in trigger.
	 * Return errors through PCM prepare rather than ignoring DAPM errors.
	 */
	mutex_lock(&amp->lock);
	if (!amp->opened) { ret = -EPERM; goto out; }
	if (!amp->prepared) {
		ret = cat_amp_load_muted(amp);
		if (ret) goto out;
	}
	ret = cat_amp_unmute(&amp->io);
	amp->prepared = !ret;
	if (!ret) dev_info(amp->dev, "ALSA prepared: verified profile, fixed -24dB\n");
out:
	mutex_unlock(&amp->lock);
	return ret;
}
static int cat_amp_hw_free(struct snd_pcm_substream *s, struct snd_soc_dai *dai)
{
	struct cat_amp *amp = snd_soc_component_get_drvdata(dai->component);

	mutex_lock(&amp->lock);
	/* Physical reset does not depend on a working I2C bus or DSP clocks. */
	cat_io_reset(amp, 1);
	amp->prepared = false;
	mutex_unlock(&amp->lock);
	return 0;
}
static int cat_amp_mute(struct snd_soc_dai *dai, int mute, int stream)
{
	struct cat_amp *amp = snd_soc_component_get_drvdata(dai->component);
	int ret = 0;

	mutex_lock(&amp->lock);
	if (amp->alsa_playback && amp->prepared) {
		ret = mute ? cat_amp_update(&amp->io, 8, 1, 1) : cat_amp_unmute(&amp->io);
		if (ret) {
			cat_io_reset(amp, 1);
			amp->prepared = false;
		}
	}
	mutex_unlock(&amp->lock);
	return ret;
}
static const struct snd_soc_dai_ops cat_amp_ops = {
	.startup = cat_amp_startup,
	.prepare = cat_amp_prepare_pcm,
	.hw_free = cat_amp_hw_free,
	.mute_stream = cat_amp_mute,
	.no_capture_mute = 1,
	.shutdown = cat_amp_shutdown,
};
static struct snd_soc_dai_driver cat_amp_dai = {
	.name = "cat-aw881xx-aif",
	.playback = {
		.stream_name = "Amplifier Playback",
		.channels_min = 1, .channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &cat_amp_ops,
};
static const struct snd_soc_dapm_widget cat_amp_widgets[] = {
	SND_SOC_DAPM_AIF_IN("AIF RX", "Amplifier Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUTPUT("Speaker"),
};
static const struct snd_soc_dapm_route cat_amp_routes[] = {
	{ "Speaker", NULL, "AIF RX" },
};
static const struct snd_soc_component_driver cat_amp_component = {
	.name = "cat-aw881xx-qualification",
	.dapm_widgets = cat_amp_widgets,
	.num_dapm_widgets = ARRAY_SIZE(cat_amp_widgets),
	.dapm_routes = cat_amp_routes,
	.num_dapm_routes = ARRAY_SIZE(cat_amp_routes),
};
static const struct regmap_config cat_amp_regmap = {
	.reg_bits = 8, .val_bits = 16,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.max_register = 0x79,
	.cache_type = REGCACHE_NONE,
};
static int cat_amp_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct cat_amp *amp;
	struct gpio_desc *reset;
	struct regmap *map;
	unsigned int id, sys, pwm, pid;
	int ret;

	amp = devm_kzalloc(dev, sizeof(*amp), GFP_KERNEL);
	if (!amp)
		return -ENOMEM;
	amp->dev = dev;
	mutex_init(&amp->lock);
	amp->alsa_playback = of_property_read_bool(dev->of_node, "cat,alsa-playback-test");
	amp->io = (struct cat_amp_io){.ctx = amp, .read = cat_io_read, .write = cat_io_write,
		.reset = cat_io_reset, .wait = cat_io_wait};
	reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset), "reset ownership\n");
	ret = devm_add_action_or_reset(dev, cat_amp_reset, reset);
	if (ret)
		return ret;
	map = devm_regmap_init_i2c(client, &cat_amp_regmap);
	if (IS_ERR(map))
		return PTR_ERR(map);
	usleep_range(1000, 1010);
	gpiod_set_value_cansleep(reset, 0);
	usleep_range(1000, 1010);
	ret = regmap_read(map, 0, &id);
	if (ret)
		return ret;
	if (id != 0x1806)
		return dev_err_probe(dev, -ENODEV, "unexpected ID %04x\n", id);
	ret = regmap_read(map, 4, &sys);
	if (ret)
		return ret;
	ret = regmap_read(map, 8, &pwm);
	if (ret)
		return ret;
	if (!(sys & 1) || !(pwm & 1))
		return dev_err_probe(dev, -EPERM, "unsafe reset defaults %04x/%04x\n", sys, pwm);
	ret = regmap_write(map, 0x40, 0x1f80); /* address selector, not DSP data */
	if (ret)
		return ret;
	ret = regmap_read(map, 0x41, &pid);
	if (ret)
		return ret;
	ret = regmap_read(map, 0, &id); /* stock DSP access restore */
	if (ret)
		return ret;
	if (id != 0x1806 || pid != 0)
		return dev_err_probe(dev, -ENODEV, "unqualified ID/PID %04x/%04x\n", id, pid);
	/* No clocks or profile: leave device physically in reset even when bound. */
	cat_amp_reset(reset);
	amp->reset = reset;

	amp->map = map;
	i2c_set_clientdata(client, amp);

	dev_info(dev, "ID1806 PID1 mute/powerdown verified; reset held; playback blocked\n");
	return devm_snd_soc_register_component(dev, &cat_amp_component, &cat_amp_dai, 1);
}
static const struct of_device_id cat_amp_of_match[] = {
	{ .compatible = "cat,s22-aw881xx-qualification" }, {}
};
MODULE_DEVICE_TABLE(of, cat_amp_of_match);
static struct i2c_driver cat_amp_driver = {
	.driver = { .name = "cat-aw881xx-qualification", .of_match_table = cat_amp_of_match },
	.probe = cat_amp_probe,
};
module_i2c_driver(cat_amp_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CAT S22 ID1806 default-blocked bounded audio qualification");
