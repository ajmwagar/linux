/* CAT S22 diagnostic console: early boot and resident handoff. */
#include <linux/font.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>

/* Parsed after cat_console_start(), before the resident early_initcall. */
static bool cat_display_simpledrm;

static int __init cat_display_setup(char *arg)
{
	if (!arg || strcmp(arg, "simpledrm"))
		return -EINVAL;
	cat_display_simpledrm = true;
	return 0;
}
early_param("cat_display", cat_display_setup);

static unsigned int cat_col, cat_line;

static void __init cat_console_clear_line(unsigned int line)
{
	unsigned int y;

	for (y = 0; y < 16; y++) {
		void __iomem *p = early_ioremap(0x90001000ULL +
			(line * 16 + y) * 1440, 1440);
		if (!p)
			return;
		memset_io(p, 0, 1440);
		wmb();
		early_iounmap(p, 1440);
	}
}

static void __init cat_console_newline(void)
{
	cat_col = 0;
	cat_line = (cat_line + 1) % 40;
	cat_console_clear_line(cat_line);
}

static void __init cat_console_write(struct console *con, const char *s,
				    unsigned int count)
{
	while (count--) {
		unsigned char c = *s++;
		unsigned int y, x;

		if (c == '\n') {
			cat_console_newline();
			continue;
		}
		if (c == '\r') {
			cat_col = 0;
			continue;
		}
		if (c < 32)
			c = ' ';
		if (cat_col == 60)
			cat_console_newline();
		for (y = 0; y < 16; y++) {
			u8 bits = font_vga_8x16.data[c * 16 + y];
			void __iomem *p = early_ioremap(0x90001000ULL +
				(cat_line * 16 + y) * 1440 + cat_col * 24, 24);
			if (!p)
				return;
			for (x = 0; x < 8; x++)
				memset_io(p + x * 3, bits & (0x80 >> x) ? 255 : 0, 3);
			wmb();
			early_iounmap(p, 24);
		}
		cat_col++;
	}
}

static struct console cat_boot_console __initdata = {
	.name = "catfb",
	.write = cat_console_write,
	.flags = CON_BOOT | CON_PRINTBUFFER | CON_ENABLED,
	.index = -1,
};

static void __init cat_console_start(void)
{
	unsigned int line;

	for (line = 0; line < 40; line++)
		cat_console_clear_line(line);
	console_loglevel = 8;
	register_console(&cat_boot_console);
	pr_info("CAT framebuffer boot console: 60x40, circular screen\n");
}

static void __iomem *cat_fb;

static void cat_resident_newline(void)
{
	cat_col = 0;
	cat_line = (cat_line + 1) % 40;
	memset_io(cat_fb + cat_line * 16 * 1440, 0, 16 * 1440);
}

static void cat_resident_write(struct console *con, const char *s,
			       unsigned int count)
{
	while (count--) {
		unsigned char c = *s++;
		unsigned int y, x;

		if (c == '\n') {
			cat_resident_newline();
			continue;
		}
		if (c == '\r') {
			cat_col = 0;
			continue;
		}
		if (c < 32)
			c = ' ';
		if (cat_col == 60)
			cat_resident_newline();
		for (y = 0; y < 16; y++) {
			u8 bits = font_vga_8x16.data[c * 16 + y];
			void __iomem *p = cat_fb +
				(cat_line * 16 + y) * 1440 + cat_col * 24;

			for (x = 0; x < 8; x++)
				memset_io(p + x * 3, bits & (0x80 >> x) ? 255 : 0, 3);
		}
		cat_col++;
	}
	wmb();
}

static struct console cat_resident_console = {
	.name = "catlog",
	.write = cat_resident_write,
	.flags = CON_PRINTBUFFER | CON_ENABLED,
	.index = -1,
};

static int __init cat_console_handoff(void)
{
	/* Do not keep a Device mapping alongside simpledrm's WC mapping.
	 * Early drawing ends here; serial and pstore consoles are untouched.
	 */
	if (cat_display_simpledrm) {
		unregister_console(&cat_boot_console);
		pr_info("CAT framebuffer released for simpledrm presentation\n");
		return 0;
	}
	/* Range is reserved and removed from the linear RAM map before paging. */
	cat_fb = ioremap(0x90001000ULL, 640 * 1440);
	if (!cat_fb)
		panic("CAT resident framebuffer mapping failed");
	unregister_console(&cat_boot_console);
	register_console(&cat_resident_console);
	pr_info("CAT resident framebuffer console active through initcalls\n");
	return 0;
}
early_initcall(cat_console_handoff);

/* Diagnostic read-only export; fixed reserved range, never arbitrary memory.
 * Live scanout may change during reads: this is not an atomic snapshot.
 */
static ssize_t cat_framebuffer_read(struct file *file, struct kobject *kobj,
		const struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	if (!cat_fb)
		return -ENODEV;
	if (off < 0)
		return -EINVAL;
	if (off >= 640 * 1440)
		return 0;
	count = min_t(size_t, count, 640 * 1440 - off);
	memcpy_fromio(buf, cat_fb + off, count);
	return count;
}
static const struct bin_attribute cat_framebuffer_attr = {
	.attr = { .name = "cat_framebuffer", .mode = 0400 },
	.size = 640 * 1440,
	.read = cat_framebuffer_read,
};
static int __init cat_framebuffer_export_init(void)
{
	int ret;

	/* No stale diagnostic mapping or export while DRM owns scanout. */
	if (cat_display_simpledrm)
		return 0;
	ret = sysfs_create_bin_file(kernel_kobj, &cat_framebuffer_attr);
	if (ret)
		pr_err("CAT framebuffer export failed: %d\n", ret);
	return ret;
}
late_initcall(cat_framebuffer_export_init);
