// SPDX-License-Identifier: GPL-2.0
/*
 * DRM driver for MIPI DBI compatible display panels
 *
 * Copyright 2022 Noralf Trønnes
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_mipi_dbi.h>
#include <drm/drm_modes.h>
#include <drm/drm_modeset_helper.h>
#include <drm/drm_print.h>

#include <video/mipi_display.h>

struct panel_mipi_dbi_format {
	const char *name;
	u32 fourcc;
	unsigned int bpp;
};

static const struct panel_mipi_dbi_format panel_mipi_dbi_formats[] = {
	{ "r5g6b5", DRM_FORMAT_RGB565, 16 },
	{ "b6x2g6x2r6x2", DRM_FORMAT_RGB888, 24 },
};

static int panel_mipi_dbi_get_format(struct device *dev, u32 *formats, unsigned int *bpp)
{
	const char *format_name;
	unsigned int i;
	int ret;

	formats[1] = DRM_FORMAT_XRGB8888;

	ret = device_property_read_string(dev, "format", &format_name);
	if (ret) {
		/* Old Device Trees don't have this property */
		formats[0] = DRM_FORMAT_RGB565;
		*bpp = 16;
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(panel_mipi_dbi_formats); i++) {
		const struct panel_mipi_dbi_format *format = &panel_mipi_dbi_formats[i];

		if (strcmp(format_name, format->name))
			continue;

		formats[0] = format->fourcc;
		*bpp = format->bpp;
		return 0;
	}

	dev_err(dev, "Pixel format is not supported: '%s'\n", format_name);

	return -EINVAL;
}

static const u8 panel_mipi_dbi_magic[15] = { 'M', 'I', 'P', 'I', ' ', 'D', 'B', 'I',
					     0, 0, 0, 0, 0, 0, 0 };

/*
 * The display controller configuration is stored in a firmware file.
 * The Device Tree 'compatible' property value with a '.bin' suffix is passed
 * to request_firmware() to fetch this file.
 */
struct panel_mipi_dbi_config {
	/* Magic string: panel_mipi_dbi_magic */
	u8 magic[15];

	/* Config file format version */
	u8 file_format_version;

	/*
	 * MIPI commands to execute when the display pipeline is enabled.
	 * This is used to configure the display controller.
	 *
	 * The commands are stored in a byte array with the format:
	 *     command, num_parameters, [ parameter, ...], command, ...
	 *
	 * Some commands require a pause before the next command can be received.
	 * Inserting a delay in the command sequence is done by using the NOP command with one
	 * parameter: delay in miliseconds (the No Operation command is part of the MIPI Display
	 * Command Set where it has no parameters).
	 *
	 * Example:
	 *     command 0x11
	 *     sleep 120ms
	 *     command 0xb1 parameters 0x01, 0x2c, 0x2d
	 *     command 0x29
	 *
	 * Byte sequence:
	 *     0x11 0x00
	 *     0x00 0x01 0x78
	 *     0xb1 0x03 0x01 0x2c 0x2d
	 *     0x29 0x00
	 */
	u8 commands[];
};

struct panel_mipi_dbi_commands {
	const u8 *buf;
	size_t len;
};

static struct panel_mipi_dbi_commands *
panel_mipi_dbi_check_commands(struct device *dev, const struct firmware *fw)
{
	const struct panel_mipi_dbi_config *config = (struct panel_mipi_dbi_config *)fw->data;
	struct panel_mipi_dbi_commands *commands;
	size_t size = fw->size, commands_len;
	unsigned int i = 0;

	if (size < sizeof(*config) + 2) { /* At least 1 command */
		dev_err(dev, "config: file size=%zu is too small\n", size);
		return ERR_PTR(-EINVAL);
	}

	if (memcmp(config->magic, panel_mipi_dbi_magic, sizeof(config->magic))) {
		dev_err(dev, "config: Bad magic: %15ph\n", config->magic);
		return ERR_PTR(-EINVAL);
	}

	if (config->file_format_version != 1) {
		dev_err(dev, "config: version=%u is not supported\n", config->file_format_version);
		return ERR_PTR(-EINVAL);
	}

	drm_dev_dbg(dev, DRM_UT_DRIVER, "size=%zu version=%u\n", size, config->file_format_version);

	commands_len = size - sizeof(*config);

	while ((i + 1) < commands_len) {
		u8 command = config->commands[i++];
		u8 num_parameters = config->commands[i++];
		const u8 *parameters = &config->commands[i];

		i += num_parameters;
		if (i > commands_len) {
			dev_err(dev, "config: command=0x%02x num_parameters=%u overflows\n",
				command, num_parameters);
			return ERR_PTR(-EINVAL);
		}

		if (command == 0x00 && num_parameters == 1)
			drm_dev_dbg(dev, DRM_UT_DRIVER, "sleep %ums\n", parameters[0]);
		else
			drm_dev_dbg(dev, DRM_UT_DRIVER, "command %02x %*ph\n",
				    command, num_parameters, parameters);
	}

	if (i != commands_len) {
		dev_err(dev, "config: malformed command array\n");
		return ERR_PTR(-EINVAL);
	}

	commands = devm_kzalloc(dev, sizeof(*commands), GFP_KERNEL);
	if (!commands)
		return ERR_PTR(-ENOMEM);

	commands->len = commands_len;
	commands->buf = devm_kmemdup(dev, config->commands, commands->len, GFP_KERNEL);
	if (!commands->buf)
		return ERR_PTR(-ENOMEM);

	return commands;
}

static struct panel_mipi_dbi_commands *panel_mipi_dbi_commands_from_fw(struct device *dev)
{
	struct panel_mipi_dbi_commands *commands;
	const struct firmware *fw;
	const char *compatible;
	char fw_name[40];
	int ret;

	ret = of_property_read_string_index(dev->of_node, "compatible", 0, &compatible);
	if (ret)
		return ERR_PTR(ret);

	snprintf(fw_name, sizeof(fw_name), "%s.bin", compatible);
	ret = request_firmware(&fw, fw_name, dev);
	if (ret) {
		dev_err(dev, "No config file found for compatible '%s' (error=%d)\n",
			compatible, ret);

		return ERR_PTR(ret);
	}

	commands = panel_mipi_dbi_check_commands(dev, fw);
	release_firmware(fw);

	return commands;
}

static void panel_mipi_dbi_commands_execute(struct mipi_dbi *dbi,
					    struct panel_mipi_dbi_commands *commands)
{
	unsigned int i = 0;

	if (!commands)
		return;

	while (i < commands->len) {
		u8 command = commands->buf[i++];
		u8 num_parameters = commands->buf[i++];
		const u8 *parameters = &commands->buf[i];

		if (command == 0x00 && num_parameters == 1)
			msleep(parameters[0]);
		else if (num_parameters)
			mipi_dbi_command_stackbuf(dbi, command, parameters, num_parameters);
		else
			mipi_dbi_command(dbi, command);

		i += num_parameters;
	}
}

struct panel_mipi_dbi_device {
	struct mipi_dbi_dev dbidev;

	struct drm_plane plane;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_connector connector;
	/* Only CAT's lifecycle bypasses the generic reset/power helpers. */
	bool cat_s22;
	bool cat_powered;
};

static struct panel_mipi_dbi_device *to_panel_mipi_dbi_device(struct drm_device *dev)
{
	return container_of(drm_to_mipi_dbi_dev(dev), struct panel_mipi_dbi_device, dbidev);
}

static const u64 panel_mipi_dbi_plane_format_modifiers[] = {
	DRM_MIPI_DBI_PLANE_FORMAT_MODIFIERS,
};

static const struct drm_plane_helper_funcs panel_mipi_dbi_plane_helper_funcs = {
	DRM_MIPI_DBI_PLANE_HELPER_FUNCS,
};

static const struct drm_plane_funcs panel_mipi_dbi_plane_funcs = {
	DRM_MIPI_DBI_PLANE_FUNCS,
	.destroy = drm_plane_cleanup,
};

static void panel_mipi_dbi_cat_poweroff(struct panel_mipi_dbi_device *panel)
{
	struct mipi_dbi_dev *dbidev = &panel->dbidev;
	int ret;

	if (!panel->cat_powered)
		return;
	ret = regulator_disable(dbidev->io_regulator);
	if (ret)
		drm_err(&dbidev->drm, "CAT I/O power-off failed: %d\n", ret);
	ret = regulator_disable(dbidev->regulator);
	if (ret)
		drm_err(&dbidev->drm, "CAT power-off failed: %d\n", ret);
	panel->cat_powered = false;
}

static int panel_mipi_dbi_cat_poweron(struct panel_mipi_dbi_device *panel)
{
	struct mipi_dbi_dev *dbidev = &panel->dbidev;
	int ret;

	ret = regulator_enable(dbidev->regulator);
	if (ret)
		return ret;
	ret = regulator_enable(dbidev->io_regulator);
	if (ret) {
		int err = regulator_disable(dbidev->regulator);

		if (err)
			drm_err(&dbidev->drm, "CAT power rollback failed: %d\n", err);
		return ret;
	}
	panel->cat_powered = true;
	/* Active-low descriptor: logical 0 = physical high. Stock H10/L20/H120. */
	gpiod_set_value_cansleep(dbidev->dbi.reset, 0);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(dbidev->dbi.reset, 1);
	msleep(20);
	gpiod_set_value_cansleep(dbidev->dbi.reset, 0);
	msleep(120);
	/* Firmware owns the sole software reset and its 50ms wait. */
	return 0;
}

static int panel_mipi_dbi_cat_commands(struct mipi_dbi_dev *dbidev)
{
	struct panel_mipi_dbi_commands *commands = dbidev->driver_private;
	struct mipi_dbi *dbi = &dbidev->dbi;
	unsigned int i = 0;
	int ret;

	if (!commands)
		return -EINVAL;

	while (i < commands->len) {
		u8 command = commands->buf[i++];
		u8 count = commands->buf[i++];
		const u8 *parameters = &commands->buf[i];

		if (!command && count == 1) {
			msleep(parameters[0]);
		} else {
			ret = mipi_dbi_command_stackbuf(dbi, command, parameters, count);
			if (ret)
				return ret;
		}
		i += count;
	}
	return 0;
}

static void panel_mipi_dbi_cat_off(struct panel_mipi_dbi_device *panel)
{
	struct mipi_dbi_dev *dbidev = &panel->dbidev;
	int ret;

	ret = backlight_disable(dbidev->backlight);
	if (ret)
		drm_err(&dbidev->drm, "CAT backlight disable failed: %d\n", ret);
	if (!panel->cat_powered)
		return;
	ret = mipi_dbi_command(&dbidev->dbi, MIPI_DCS_SET_DISPLAY_OFF);
	if (ret)
		drm_err(&dbidev->drm, "CAT display-off failed: %d\n", ret);
	msleep(32);
	ret = mipi_dbi_command(&dbidev->dbi, MIPI_DCS_ENTER_SLEEP_MODE);
	if (ret)
		drm_err(&dbidev->drm, "CAT sleep-in failed: %d\n", ret);
	msleep(32);
	panel_mipi_dbi_cat_poweroff(panel);
	drm_info(&dbidev->drm, "CAT outer off sequence completed (errors logged above)\n");
}

static void panel_mipi_dbi_crtc_helper_atomic_disable(struct drm_crtc *crtc,
						   struct drm_atomic_commit *state)
{
	struct panel_mipi_dbi_device *panel = to_panel_mipi_dbi_device(crtc->dev);
	int idx;

	if (!panel->cat_s22) {
		drm_mipi_dbi_crtc_helper_atomic_disable(crtc, state);
		return;
	}
	if (!drm_dev_enter(crtc->dev, &idx))
		return;
	panel_mipi_dbi_cat_off(panel);
	drm_dev_exit(idx);
}

static void panel_mipi_dbi_crtc_helper_atomic_enable(struct drm_crtc *crtc,
						     struct drm_atomic_commit *state)
{
	struct drm_device *drm = crtc->dev;
	struct panel_mipi_dbi_device *panel_mipi_dbi = to_panel_mipi_dbi_device(drm);
	struct mipi_dbi_dev *dbidev = &panel_mipi_dbi->dbidev;
	struct mipi_dbi *dbi = &dbidev->dbi;
	int ret, idx;

	if (!drm_dev_enter(drm, &idx))
		return;

	drm_dbg(drm, "\n");

	if (panel_mipi_dbi->cat_s22) {
		ret = panel_mipi_dbi_cat_poweron(panel_mipi_dbi);
		if (ret)
			goto cat_error;
		ret = panel_mipi_dbi_cat_commands(dbidev);
		if (ret)
			goto cat_error;
		ret = backlight_enable(dbidev->backlight);
		if (ret)
			goto cat_error;
		drm_info(drm, "CAT outer init and binary backlight enable completed\n");
		goto out_exit;
	}

	ret = mipi_dbi_poweron_conditional_reset(dbidev);
	if (ret < 0)
		goto out_exit;
	if (!ret)
		panel_mipi_dbi_commands_execute(dbi, dbidev->driver_private);

	backlight_enable(dbidev->backlight);
	goto out_exit;
cat_error:
	drm_err(drm, "CAT enable failed: %d\n", ret);
	panel_mipi_dbi_cat_off(panel_mipi_dbi);
out_exit:
	drm_dev_exit(idx);
}

static const struct drm_crtc_helper_funcs panel_mipi_dbi_crtc_helper_funcs = {
	.mode_valid = drm_mipi_dbi_crtc_helper_mode_valid,
	.atomic_check = drm_mipi_dbi_crtc_helper_atomic_check,
	.atomic_disable = panel_mipi_dbi_crtc_helper_atomic_disable,
	.atomic_enable = panel_mipi_dbi_crtc_helper_atomic_enable,
};

static const struct drm_crtc_funcs panel_mipi_dbi_crtc_funcs = {
	DRM_MIPI_DBI_CRTC_FUNCS,
	.destroy = drm_crtc_cleanup,
};

static const struct drm_encoder_funcs panel_mipi_dbi_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static const struct drm_connector_helper_funcs panel_mipi_dbi_connector_helper_funcs = {
	DRM_MIPI_DBI_CONNECTOR_HELPER_FUNCS,
};

static const struct drm_connector_funcs panel_mipi_dbi_connector_funcs = {
	DRM_MIPI_DBI_CONNECTOR_FUNCS,
	.destroy = drm_connector_cleanup,
};

static const struct drm_mode_config_helper_funcs panel_mipi_dbi_mode_config_helper_funcs = {
	DRM_MIPI_DBI_MODE_CONFIG_HELPER_FUNCS,
};

static const struct drm_mode_config_funcs panel_mipi_dbi_mode_config_funcs = {
	DRM_MIPI_DBI_MODE_CONFIG_FUNCS,
};

DEFINE_DRM_GEM_DMA_FOPS(panel_mipi_dbi_fops);

static const struct drm_driver panel_mipi_dbi_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops			= &panel_mipi_dbi_fops,
	DRM_GEM_DMA_DRIVER_OPS_VMAP,
	DRM_FBDEV_DMA_DRIVER_OPS,
	.debugfs_init		= mipi_dbi_debugfs_init,
	.name			= "panel-mipi-dbi",
	.desc			= "MIPI DBI compatible display panel",
	.major			= 1,
	.minor			= 0,
};

static int panel_mipi_dbi_get_mode(struct mipi_dbi_dev *dbidev, struct drm_display_mode *mode)
{
	struct device *dev = dbidev->drm.dev;
	u16 hback_porch, vback_porch;
	int ret;

	ret = of_get_drm_panel_display_mode(dev->of_node, mode, NULL);
	if (ret) {
		dev_err(dev, "%pOF: failed to get panel-timing (error=%d)\n", dev->of_node, ret);
		return ret;
	}

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;

	hback_porch = mode->htotal - mode->hsync_end;
	vback_porch = mode->vtotal - mode->vsync_end;

	/*
	 * Make sure width and height are set and that only back porch and
	 * pixelclock are set in the other timing values. Also check that
	 * width and height don't exceed the 16-bit value specified by MIPI DCS.
	 */
	if (!mode->hdisplay || !mode->vdisplay || mode->flags ||
	    mode->hsync_end > mode->hdisplay || (hback_porch + mode->hdisplay) > 0xffff ||
	    mode->vsync_end > mode->vdisplay || (vback_porch + mode->vdisplay) > 0xffff) {
		dev_err(dev, "%pOF: panel-timing out of bounds\n", dev->of_node);
		return -EINVAL;
	}

	/* The driver doesn't use the pixel clock but it is mandatory so fake one if not set */
	if (!mode->clock)
		mode->clock = mode->htotal * mode->vtotal * 60 / 1000;

	dbidev->top_offset = vback_porch;
	dbidev->left_offset = hback_porch;

	return 0;
}

static int panel_mipi_dbi_spi_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct panel_mipi_dbi_device *panel_mipi_dbi;
	struct mipi_dbi_dev *dbidev;
	struct drm_device *drm;
	struct drm_display_mode mode;
	struct mipi_dbi *dbi;
	struct gpio_desc *dc;
	unsigned int bpp;
	size_t buf_size;
	u32 formats[2];
	struct drm_plane *plane;
	struct drm_crtc *crtc;
	struct drm_encoder *encoder;
	struct drm_connector *connector;
	int ret;

	panel_mipi_dbi = devm_drm_dev_alloc(dev, &panel_mipi_dbi_driver,
					    struct panel_mipi_dbi_device,
					    dbidev.drm);
	if (IS_ERR(panel_mipi_dbi))
		return PTR_ERR(panel_mipi_dbi);
	dbidev = &panel_mipi_dbi->dbidev;
	dbi = &dbidev->dbi;
	drm = &dbidev->drm;
	panel_mipi_dbi->cat_s22 = of_device_is_compatible(dev->of_node, "cat,s22flip-outer");

	ret = panel_mipi_dbi_get_mode(dbidev, &mode);
	if (ret)
		return ret;

	dbidev->regulator = devm_regulator_get(dev, "power");
	if (IS_ERR(dbidev->regulator))
		return dev_err_probe(dev, PTR_ERR(dbidev->regulator),
				     "Failed to get regulator 'power'\n");

	dbidev->io_regulator = devm_regulator_get(dev, "io");
	if (IS_ERR(dbidev->io_regulator))
		return dev_err_probe(dev, PTR_ERR(dbidev->io_regulator),
				     "Failed to get regulator 'io'\n");

	if (panel_mipi_dbi->cat_s22 &&
	    (!dbidev->regulator || !dbidev->io_regulator))
		return dev_err_probe(dev, -EINVAL, "CAT requires power and I/O regulators\n");

	dbidev->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(dbidev->backlight))
		return dev_err_probe(dev, PTR_ERR(dbidev->backlight), "Failed to get backlight\n");

	dbi->reset = devm_gpiod_get_optional(dev, "reset",
					   panel_mipi_dbi->cat_s22 ? GPIOD_OUT_LOW : GPIOD_OUT_HIGH);
	if (IS_ERR(dbi->reset))
		return dev_err_probe(dev, PTR_ERR(dbi->reset), "Failed to get GPIO 'reset'\n");

	if (panel_mipi_dbi->cat_s22 && (!dbi->reset || !dbidev->backlight))
		return dev_err_probe(dev, -EINVAL, "CAT requires reset and backlight\n");

	/* Multiple panels can share the "dc" GPIO, but only if they are on the same SPI bus! */
	dc = devm_gpiod_get_optional(dev, "dc", GPIOD_OUT_LOW | GPIOD_FLAGS_BIT_NONEXCLUSIVE);
	if (IS_ERR(dc))
		return dev_err_probe(dev, PTR_ERR(dc), "Failed to get GPIO 'dc'\n");

	ret = mipi_dbi_spi_init(spi, dbi, dc);
	if (ret)
		return ret;

	if (device_property_present(dev, "write-only"))
		dbi->read_commands = NULL;

	dbidev->driver_private = panel_mipi_dbi_commands_from_fw(dev);
	if (IS_ERR(dbidev->driver_private))
		return PTR_ERR(dbidev->driver_private);

	ret = panel_mipi_dbi_get_format(dev, formats, &bpp);
	if (ret)
		return ret;

	/*
	 * The fitted CAT panel uses RGB565 over Type C option 3. Use
	 * byte-wide transfers with the DBI core's big-endian conversion,
	 * avoiding native 16-bit word ordering in the QUP PIO path.
	 * Set both fields: conversion and transfer width must agree.
	 */
	if (panel_mipi_dbi->cat_s22 && formats[0] == DRM_FORMAT_RGB565 && dc) {
		dbi->write_memory_bpw = 8;
		dbi->swap_bytes = true;
	}

	buf_size = DIV_ROUND_UP(mode.hdisplay * mode.vdisplay * bpp, 8);
	ret = drm_mipi_dbi_dev_init(dbidev, &mode, formats[0], 0, buf_size);
	if (ret)
		return ret;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;

	drm->mode_config.min_width = dbidev->mode.hdisplay;
	drm->mode_config.max_width = dbidev->mode.hdisplay;
	drm->mode_config.min_height = dbidev->mode.vdisplay;
	drm->mode_config.max_height = dbidev->mode.vdisplay;
	drm->mode_config.funcs = &panel_mipi_dbi_mode_config_funcs;
	drm->mode_config.preferred_depth = bpp;
	drm->mode_config.helper_private = &panel_mipi_dbi_mode_config_helper_funcs;

	plane = &panel_mipi_dbi->plane;
	ret = drm_universal_plane_init(drm, plane, 0, &panel_mipi_dbi_plane_funcs,
				       formats, ARRAY_SIZE(formats),
				       panel_mipi_dbi_plane_format_modifiers,
				       DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ret;
	drm_plane_helper_add(plane, &panel_mipi_dbi_plane_helper_funcs);
	drm_plane_enable_fb_damage_clips(plane);

	crtc = &panel_mipi_dbi->crtc;
	ret = drm_crtc_init_with_planes(drm, crtc, plane, NULL, &panel_mipi_dbi_crtc_funcs,
					NULL);
	if (ret)
		return ret;
	drm_crtc_helper_add(crtc, &panel_mipi_dbi_crtc_helper_funcs);

	encoder = &panel_mipi_dbi->encoder;
	ret = drm_encoder_init(drm, encoder, &panel_mipi_dbi_encoder_funcs,
			       DRM_MODE_ENCODER_NONE, NULL);
	if (ret)
		return ret;
	encoder->possible_crtcs = drm_crtc_mask(crtc);

	connector = &panel_mipi_dbi->connector;
	ret = drm_connector_init(drm, connector, &panel_mipi_dbi_connector_funcs,
				 DRM_MODE_CONNECTOR_SPI);
	if (ret)
		return ret;
	drm_connector_helper_add(connector, &panel_mipi_dbi_connector_helper_funcs);

	ret = drm_connector_attach_encoder(connector, encoder);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	spi_set_drvdata(spi, drm);

	if (bpp == 16)
		drm_client_setup_with_fourcc(drm, DRM_FORMAT_RGB565);
	else
		drm_client_setup_with_fourcc(drm, DRM_FORMAT_RGB888);

	return 0;
}

static void panel_mipi_dbi_spi_remove(struct spi_device *spi)
{
	struct drm_device *drm = spi_get_drvdata(spi);

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);
}

static void panel_mipi_dbi_spi_shutdown(struct spi_device *spi)
{
	drm_atomic_helper_shutdown(spi_get_drvdata(spi));
}

static int __maybe_unused panel_mipi_dbi_pm_suspend(struct device *dev)
{
	return drm_mode_config_helper_suspend(dev_get_drvdata(dev));
}

static int __maybe_unused panel_mipi_dbi_pm_resume(struct device *dev)
{
	drm_mode_config_helper_resume(dev_get_drvdata(dev));

	return 0;
}

static const struct dev_pm_ops panel_mipi_dbi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(panel_mipi_dbi_pm_suspend, panel_mipi_dbi_pm_resume)
};

static const struct of_device_id panel_mipi_dbi_spi_of_match[] = {
	{ .compatible = "panel-mipi-dbi-spi" },
	{},
};
MODULE_DEVICE_TABLE(of, panel_mipi_dbi_spi_of_match);

static const struct spi_device_id panel_mipi_dbi_spi_id[] = {
	{ "panel-mipi-dbi-spi", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, panel_mipi_dbi_spi_id);

static struct spi_driver panel_mipi_dbi_spi_driver = {
	.driver = {
		.name = "panel-mipi-dbi-spi",
		.of_match_table = panel_mipi_dbi_spi_of_match,
		.pm = &panel_mipi_dbi_pm_ops,
	},
	.id_table = panel_mipi_dbi_spi_id,
	.probe = panel_mipi_dbi_spi_probe,
	.remove = panel_mipi_dbi_spi_remove,
	.shutdown = panel_mipi_dbi_spi_shutdown,
};
module_spi_driver(panel_mipi_dbi_spi_driver);

MODULE_DESCRIPTION("MIPI DBI compatible display panel driver");
MODULE_AUTHOR("Noralf Trønnes");
MODULE_LICENSE("GPL");
