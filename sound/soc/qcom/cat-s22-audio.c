// SPDX-License-Identifier: GPL-2.0-only
/* CAT S22 DPCM topology qualification. Codec startup denies playback. */
#include <linux/module.h>
#include <linux/io.h>
#include <linux/mux/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <dt-bindings/sound/qcom,q6afe.h>
#include "qdsp6/q6afe.h"
#include "common.h"

struct cat_transport {
	struct mux_control *mux;
	struct pinctrl *pins;
	struct pinctrl_state *active, *sleep;
	bool running;
	void __iomem *mic_mux, *speaker_mux;
	u32 mic_saved, speaker_saved;
	bool capture_running;
};

static int cat_transport_startup(struct snd_pcm_substream *s)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(s);
	struct cat_transport *t = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu = snd_soc_rtd_to_cpu(rtd, 0);
	int ret, cleanup;

	/* One fixed playback backend; do not silently share ownership. */
	if (s->stream != SNDRV_PCM_STREAM_PLAYBACK || cpu->id != QUINARY_MI2S_RX)
		return -EINVAL;
	if (t->running)
		return -EBUSY;
	ret = mux_control_try_select(t->mux, 1);
	if (ret)
		return ret;
	ret = snd_soc_dai_set_fmt(cpu, SND_SOC_DAIFMT_BP_FP | SND_SOC_DAIFMT_I2S);
	if (ret)
		goto mux;
	/* CAT stock derives BCLK = 48000 * 16 * 2 for this port. */
	ret = snd_soc_dai_set_sysclk(cpu, Q6AFE_LPASS_CLK_ID_QUI_MI2S_IBIT,
				    48000 * 16 * 2, SNDRV_PCM_STREAM_PLAYBACK);
	if (ret)
		goto clock;
	ret = pinctrl_select_state(t->pins, t->active);
	if (ret)
		goto clock;
	t->running = true;
	dev_info(rtd->card->dev, "QUINARY transport test: clocks active; amplifier reset held\n");
	return 0;
clock:
	cleanup = snd_soc_dai_set_sysclk(cpu, Q6AFE_LPASS_CLK_ID_QUI_MI2S_IBIT,
				       0, SNDRV_PCM_STREAM_PLAYBACK);
	if (cleanup)
		dev_err(rtd->card->dev, "clock rollback failed: %d\n", cleanup);
	cleanup = pinctrl_select_state(t->pins, t->sleep);
	if (cleanup)
		dev_err(rtd->card->dev, "pin rollback failed: %d\n", cleanup);
mux:
	cleanup = mux_control_deselect(t->mux);
	if (cleanup)
		dev_err(rtd->card->dev, "mux release failed: %d\n", cleanup);
	return ret;
}

static void cat_transport_shutdown(struct snd_pcm_substream *s)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(s);
	struct cat_transport *t = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu = snd_soc_rtd_to_cpu(rtd, 0);
	int ret;

	if (!t->running)
		return;
	ret = snd_soc_dai_set_sysclk(cpu, Q6AFE_LPASS_CLK_ID_QUI_MI2S_IBIT,
				    0, SNDRV_PCM_STREAM_PLAYBACK);
	if (ret)
		dev_err(rtd->card->dev, "clock shutdown failed: %d\n", ret);
	ret = pinctrl_select_state(t->pins, t->sleep);
	if (ret)
		dev_err(rtd->card->dev, "pin sleep failed: %d\n", ret);
	ret = mux_control_deselect(t->mux);
	if (ret)
		dev_err(rtd->card->dev, "mux release failed: %d\n", ret);
	t->running = false;
	dev_info(rtd->card->dev, "QUINARY transport test stopped; amplifier reset held\n");
}

static int cat_transport_fixup(struct snd_soc_pcm_runtime *rtd,
			       struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_mask *format = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);

	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;
	snd_mask_none(format);
	snd_mask_set_format(format, SNDRV_PCM_FORMAT_S16_LE);
	return 0;
}

static const struct snd_soc_ops cat_transport_ops = {
	.startup = cat_transport_startup,
	.shutdown = cat_transport_shutdown,
};

/* Fitted stock TERT startup selects MIC bit21 and speaker bit16.
 * Restore only owned bits, preserving unrelated mux fields on every exit.
 * Resources are exclusively requested at probe, never selected at probe.
 */
static void cat_capture_restore_mux(struct cat_transport *t)
{
	u32 val;

	val = readl(t->mic_mux);
	writel((val & ~BIT(21)) | t->mic_saved, t->mic_mux);
	val = readl(t->speaker_mux);
	writel((val & ~BIT(16)) | t->speaker_saved, t->speaker_mux);
}

static int cat_capture_startup(struct snd_pcm_substream *s)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(s);
	struct cat_transport *t = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu = snd_soc_rtd_to_cpu(rtd, 0);
	u32 val;
	int ret, cleanup;

	if (s->stream != SNDRV_PCM_STREAM_CAPTURE || cpu->id != TERTIARY_MI2S_TX)
		return -EINVAL;
	if (t->capture_running)
		return -EBUSY;
	/* The late probe disables the mic pin until userspace selects it.
	 * DAPM cannot report the pin On until this backend powers the path,
	 * so checking pin status here is circular and blocks every capture.
	 */
	val = readl(t->speaker_mux);
	t->speaker_saved = val & BIT(16);
	writel(val | BIT(16), t->speaker_mux);
	val = readl(t->mic_mux);
	t->mic_saved = val & BIT(21);
	writel(val | BIT(21), t->mic_mux);
	ret = snd_soc_dai_set_fmt(cpu, SND_SOC_DAIFMT_BP_FP | SND_SOC_DAIFMT_I2S);
	if (ret)
		goto restore;
	/* Exact stock clock request: TERT port0x1005, ID0x104, 1536000Hz. */
	ret = snd_soc_dai_set_sysclk(cpu, Q6AFE_LPASS_CLK_ID_TER_MI2S_IBIT,
				    1536000, SNDRV_PCM_STREAM_CAPTURE);
	if (ret) {
		cleanup = snd_soc_dai_set_sysclk(cpu, Q6AFE_LPASS_CLK_ID_TER_MI2S_IBIT,
					       0, SNDRV_PCM_STREAM_CAPTURE);
		if (cleanup)
			dev_err(rtd->card->dev, "TERT clock rollback failed: %d\n", cleanup);
		goto restore;
	}
	t->capture_running = true;
	return 0;
restore:
	cat_capture_restore_mux(t);
	return ret;
}

static void cat_capture_shutdown(struct snd_pcm_substream *s)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(s);
	struct cat_transport *t = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu = snd_soc_rtd_to_cpu(rtd, 0);
	int ret;

	if (!t->capture_running)
		return;
	ret = snd_soc_dai_set_sysclk(cpu, Q6AFE_LPASS_CLK_ID_TER_MI2S_IBIT,
				    0, SNDRV_PCM_STREAM_CAPTURE);
	if (ret)
		dev_err(rtd->card->dev, "TERT clock shutdown failed: %d\n", ret);
	cat_capture_restore_mux(t);
	t->capture_running = false;
}

static int cat_capture_fixup(struct snd_soc_pcm_runtime *rtd,
			     struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_mask *format = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);

	rate->min = rate->max = 48000;
	channels->min = channels->max = 1;
	snd_mask_none(format);
	snd_mask_set_format(format, SNDRV_PCM_FORMAT_S16_LE);
	return 0;
}

static const struct snd_soc_ops cat_capture_ops = {
	.startup = cat_capture_startup,
	.shutdown = cat_capture_shutdown,
};

static const struct snd_soc_dapm_route cat_pdm_routes[] = {
	{ "LPASS_PDM_TX", NULL, "PDM_TX" },
};

static const struct snd_pcm_hardware cat_pdm_hw = {
	.info			= SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	.rates			= SNDRV_PCM_RATE_48000,
	.rate_min		= 48000,
	.rate_max		= 48000,
	.channels_min		= 1,
	.channels_max		= 1,
	.buffer_bytes_max	= 128 * 1024,
	.period_bytes_min	= 4096,
	.period_bytes_max	= 8192,
	.periods_min		= 2,
	.periods_max		= 128,
};

static int cat_pdm_startup(struct snd_pcm_substream *s)
{
	snd_soc_set_runtime_hwparams(s, &cat_pdm_hw);
	return 0;
}

static const struct snd_soc_ops cat_pdm_ops = {
	.startup = cat_pdm_startup,
};

static int cat_add_pdm_capture_link(struct snd_soc_card *card)
{
	struct snd_soc_dai_link *links, *link;
	struct snd_soc_dai_link_component *codec;
	struct device_node *np;
	int ret;

	np = of_parse_phandle(card->dev->of_node, "aux-devs", 0);
	if (!np)
		return -ENODEV;

	links = devm_kcalloc(card->dev, card->num_links + 1,
			     sizeof(*links), GFP_KERNEL);
	codec = devm_kzalloc(card->dev, sizeof(*codec), GFP_KERNEL);
	if (!links || !codec) {
		of_node_put(np);
		return -ENOMEM;
	}
	memcpy(links, card->dai_link, card->num_links * sizeof(*links));

	link = &links[card->num_links];
	codec->of_node = np;
	codec->dai_name = "pm8916_wcd_analog_pdm_tx";
	link->name = "PDM Capture";
	link->stream_name = "PDM Capture";
	link->cpus = &snd_soc_dummy_dlc;
	link->num_cpus = 1;
	link->codecs = codec;
	link->num_codecs = 1;
	link->platforms = &snd_soc_dummy_dlc;
	link->num_platforms = 1;
	link->nonatomic = 1;
	link->ignore_suspend = 1;
	link->ops = &cat_pdm_ops;

	card->dai_link = links;
	card->num_links++;
	ret = 0;
	return ret;
}

static int cat_capture_late_probe(struct snd_soc_card *card)
{
	static const char * const disabled[] = {
		"AMIC2", "AMIC3", "EAR", "HPH_L", "HPH_R", "SPK_OUT",
	};
	struct snd_soc_dapm_context *dapm = snd_soc_card_to_dapm(card);
	int i, ret;

	/* A connected ALSA input pin must not silently select another capsule.
	 * Explicit userspace Handset Mic switch is required in addition to PCM.
	 */
	for (i = 0; i < ARRAY_SIZE(disabled); i++) {
		ret = snd_soc_dapm_disable_pin(dapm, disabled[i]);
		if (ret)
			return ret;
	}
	ret = snd_soc_dapm_add_routes(dapm, cat_pdm_routes,
				      ARRAY_SIZE(cat_pdm_routes));
	if (ret)
		return ret;
	ret = snd_soc_dapm_force_enable_pin(dapm, "PDM Capture");
	if (ret)
		return ret;
	ret = snd_soc_dapm_force_enable_pin(dapm, "PDM_TX");
	if (ret)
		return ret;
	ret = snd_soc_dapm_force_enable_pin(dapm, "ADC1");
	if (ret)
		return ret;
	ret = snd_soc_dapm_force_enable_pin(dapm, "Handset Mic");
	if (ret)
		return ret;
	ret = snd_soc_dapm_force_enable_pin(dapm, "MIC BIAS External1");
	if (ret)
		return ret;
	ret = snd_soc_dapm_force_enable_pin(dapm, "AMIC1");
	if (ret)
		return ret;
	{
		struct snd_soc_component *c;
		unsigned int v50 = 0xffff, v160 = 0xffff, v141 = 0xffff, v163 = 0xffff;

		for_each_card_components(card, c) {
			if (!c->name || !strstr(c->name, "audio-codec@f000"))
				continue;
			snd_soc_component_update_bits(c, 0xf048, 0x0c, 0x0c);
			snd_soc_component_update_bits(c, 0xf049, 0x20, 0x20);
			snd_soc_component_update_bits(c, 0xf04a, 0x10, 0x10);
			snd_soc_component_update_bits(c, 0xf140, 0x80, 0x80);
			snd_soc_component_write(c, 0xf143, 0x49);
			snd_soc_component_update_bits(c, 0xf050, 0x3, 0x0);
			snd_soc_component_update_bits(c, 0xf160, 0x80, 0x80);
			v50 = snd_soc_component_read(c, 0xf050);
			v160 = snd_soc_component_read(c, 0xf160);
			v141 = snd_soc_component_read(c, 0xf141);
			v163 = snd_soc_component_read(c, 0xf163);
			dev_info(card->dev, "ANALOG f048=%02x f049=%02x f04a=%02x\n",
				 snd_soc_component_read(c, 0xf048),
				 snd_soc_component_read(c, 0xf049),
				 snd_soc_component_read(c, 0xf04a));
		}
		dev_info(card->dev, "ANALOG f050=%02x f160=%02x f141=%02x f163=%02x\n",
			 v50, v160, v141, v163);
	}
	return snd_soc_dapm_sync(dapm);
}

static int cat_card_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card;
	struct cat_transport *t;
	int ret, i, j;

	card = devm_kzalloc(&pdev->dev, sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;
	card->driver_name = "cat-s22";
	card->dev = &pdev->dev;
	card->owner = THIS_MODULE;
	ret = qcom_snd_parse_of(card);
	if (ret)
		return ret;
	/* Overlay merge can reverse child order. Routing owns cross-component
	 * routes to ASM widgets: frontend components must be probed first.
	 * Keep frontend links before backend routing links independent of DT order.
	 */
	for (i = 1; i < card->num_links; i++) {
		struct snd_soc_dai_link link = card->dai_link[i];

		for (j = i; j > 0 && link.dynamic && !card->dai_link[j - 1].dynamic; j--)
			card->dai_link[j] = card->dai_link[j - 1];
		card->dai_link[j] = link;
	}
	ret = cat_add_pdm_capture_link(card);
	if (ret)
		return ret;
	t = devm_kzalloc(&pdev->dev, sizeof(*t), GFP_KERNEL);
	if (!t)
		return -ENOMEM;
	snd_soc_card_set_drvdata(card, t);
	if (of_property_read_bool(pdev->dev.of_node, "cat,handset-capture-test")) {
		card->late_probe = cat_capture_late_probe;
		t->mic_mux = devm_platform_ioremap_resource_byname(pdev, "mic-mux");
		if (IS_ERR(t->mic_mux))
			return PTR_ERR(t->mic_mux);
		t->speaker_mux = devm_platform_ioremap_resource_byname(pdev, "speaker-mux");
		if (IS_ERR(t->speaker_mux))
			return PTR_ERR(t->speaker_mux);
		for (i = 0; i < card->num_links; i++) {
			if (!card->dai_link[i].name ||
			    strcmp(card->dai_link[i].name, "TERT_MI2S_TX"))
				continue;
			card->dai_link[i].ops = &cat_capture_ops;
			card->dai_link[i].be_hw_params_fixup = cat_capture_fixup;
		}
	}
	if (of_property_read_bool(pdev->dev.of_node, "cat,reset-held-transport-test")) {
		t->mux = devm_mux_control_get(&pdev->dev, "quinary");
		if (IS_ERR(t->mux))
			return dev_err_probe(&pdev->dev, PTR_ERR(t->mux), "QUINARY mux\n");
		t->pins = devm_pinctrl_get(&pdev->dev);
		if (IS_ERR(t->pins))
			return dev_err_probe(&pdev->dev, PTR_ERR(t->pins), "QUINARY pins\n");
		t->active = pinctrl_lookup_state(t->pins, "quinary-active");
		if (IS_ERR(t->active))
			return PTR_ERR(t->active);
		t->sleep = pinctrl_lookup_state(t->pins, "quinary-sleep");
		if (IS_ERR(t->sleep))
			return PTR_ERR(t->sleep);
		/* No pin or mux selection at probe. The codec still holds reset. */
		snd_soc_card_set_drvdata(card, t);
		for (i = 0; i < card->num_links; i++) {
			/* Speaker-only ops must never configure a capture backend. */
			if (!card->dai_link[i].no_pcm ||
			    !card->dai_link[i].name ||
			    strcmp(card->dai_link[i].name, "QUINARY_MI2S_RX"))
				continue;
			card->dai_link[i].ops = &cat_transport_ops;
			card->dai_link[i].be_hw_params_fixup = cat_transport_fixup;
		}
	}
	return devm_snd_soc_register_card(&pdev->dev, card);
}
static const struct of_device_id cat_card_of_match[] = {
	{ .compatible = "cat,s22-audio" }, {}
};
MODULE_DEVICE_TABLE(of, cat_card_of_match);
static struct platform_driver cat_card_driver = {
	.probe = cat_card_probe,
	.driver = { .name = "cat-s22-audio", .of_match_table = cat_card_of_match },
};
module_platform_driver(cat_card_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CAT S22 audio topology qualification");
