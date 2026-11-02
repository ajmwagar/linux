/* SPDX-License-Identifier: GPL-2.0-only */
/* CAT binary-derived bounded PID1 qualification sequence, not a general codec.
 * Shared with the host fault-injection test; callbacks return 0 or negative errno.
 * Caller serializes ALL operations and supplies verified exact CAT ACF bytes.
 */
#ifndef CAT_AW881XX_TONE_CORE_H
#define CAT_AW881XX_TONE_CORE_H
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/errno.h>
#else
#include <stdint.h>
#include <stddef.h>
#include <errno.h>
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
#endif
struct cat_amp_io {
	void *ctx;
	int (*read)(void *, u8, unsigned int *);
	int (*write)(void *, u8, u16);
	void (*reset)(void *, int); /* logical 1 = physical reset asserted */
	void (*wait)(void *, unsigned int, unsigned int);
	unsigned int observed_pid; /* diagnostics; not an override */
};
struct cat_amp_section { const u8 *data; unsigned int size; };
struct cat_amp_profile { struct cat_amp_section regs, cfg, fw; };
static u16 cat_le16(const u8 *p) { return p[0] | (u16)p[1] << 8; }
static u32 cat_le32(const u8 *p) { return cat_le16(p) | (u32)cat_le16(p + 2) << 16; }
static unsigned int cat_crc8(const u8 *p, unsigned int n)
{
	unsigned int c = 0, i;
	while (n--) {
		c ^= *p++;
		for (i = 0; i < 8; i++) c = (c >> 1) ^ ((c & 1) ? 0x8c : 0);
	}
	return c;
}
/* Exact pinned ACF only. Hash is checked by caller before this parser.
 * Parse both triplets, enforce channel0/chip1806, select fitted PID1 by name.
 */
static int cat_amp_parse(const u8 *d, unsigned int n, struct cat_amp_profile profiles[2])
{
	unsigned int i, j, seen[2] = {0, 0}, starts[6], ends[6];
	profiles[0] = profiles[1] = (struct cat_amp_profile){0};
	if (n != 6352 || cat_le32(d) != 0x0a15f908 ||
	    cat_le32(d + 52) != 576 || cat_le32(d + 56) != 6 ||
	    cat_le32(d + 60) != 80 || cat_le32(d + 64) != 0x01000000)
		return -EINVAL;
	for (i = 0; i < 6; i++) {
		const u8 *e = d + 80 + i * 96;
		unsigned int pass, kind = cat_le32(e + 28);
		unsigned int size = cat_le32(e + 32), off = cat_le32(e + 36), bit;
		struct cat_amp_section s;
		const char *name = "aw88194";
		for (j = 0; j < 7; j++) if (e[4 + j] != name[j]) return -EINVAL;
		if (e[11] == 0) pass = 0;
		else if (e[11] == 'A' && e[12] == 0) pass = 1;
		else return -EINVAL;
		if (cat_le32(e) != 2 || cat_le16(e + 20) || cat_le32(e + 76) != 0x1806)
			return -EINVAL;
		if (kind == 0) { bit = 1; if (size != 140) return -EINVAL; }
		else if (kind == 2) { bit = 2; if (size != (pass ? 924 : 412)) return -EINVAL; }
		else if (kind == 3) { bit = 4; if (size != 2040) return -EINVAL; }
		else return -EINVAL;
		if ((seen[pass] & bit) || off < 656 || off > n || size > n - off)
			return -EINVAL;
		starts[i] = off; ends[i] = off + size;
		for (j = 0; j < i; j++) if (off < ends[j] && starts[j] < ends[i]) return -EINVAL;
		if (cat_le32(e + 40) != cat_crc8(d + off, size)) return -EINVAL;
		seen[pass] |= bit;
		s = (struct cat_amp_section){d + off, size};
		if (kind == 0) profiles[pass].regs = s;
		else if (kind == 2) profiles[pass].cfg = s;
		else profiles[pass].fw = s;
	}
	return seen[0] == 7 && seen[1] == 7 ? 0 : -EINVAL;
}
static int cat_amp_check(struct cat_amp_io *io, u8 reg, u16 mask, u16 value)
{
	unsigned int v;
	int ret = io->read(io->ctx, reg, &v);
	return ret ? ret : ((v & mask) == value ? 0 : -EIO);
}
static int cat_amp_update(struct cat_amp_io *io, u8 reg, u16 mask, u16 value)
{
	unsigned int v;
	int ret = io->read(io->ctx, reg, &v);
	if (ret) return ret;
	ret = io->write(io->ctx, reg, (v & ~mask) | value);
	return ret ? ret : cat_amp_check(io, reg, mask, value);
}
static int cat_amp_dsp_read(struct cat_amp_io *io, u16 addr, unsigned int *v)
{
	int ret = io->write(io->ctx, 0x40, addr);
	if (!ret) ret = io->read(io->ctx, 0x41, v);
	if (!ret) ret = cat_amp_check(io, 0, 0xffff, 0x1806);
	return ret;
}
static int cat_amp_dsp_write(struct cat_amp_io *io, u16 addr, u16 v)
{
	unsigned int got;
	int ret = io->write(io->ctx, 0x40, addr);
	if (!ret) ret = io->write(io->ctx, 0x41, v);
	if (!ret) ret = cat_amp_dsp_read(io, addr, &got);
	return ret ? ret : (got == v ? 0 : -EIO);
}
/* ACF LE word -> BE I2C word is the stock pair swap, NOT legacy loading.
 * The DSP data port auto-increments as used by stock uploads. Verify the
 * complete section in a second sequential pass; no cached/unchecked upload.
 */
static int cat_amp_upload(struct cat_amp_io *io, u16 addr, struct cat_amp_section s)
{
	unsigned int i, got;
	int ret = io->write(io->ctx, 0x40, addr);

	if (ret) return ret;
	for (i = 0; i < s.size; i += 2) {
		ret = io->write(io->ctx, 0x41, cat_le16(s.data + i));
		if (ret) return ret;
	}
	ret = io->write(io->ctx, 0x40, addr);
	if (ret) return ret;
	for (i = 0; i < s.size; i += 2) {
		ret = io->read(io->ctx, 0x41, &got);
		if (ret) return ret;
		if (got != cat_le16(s.data + i)) return -EIO;
	}
	return cat_amp_check(io, 0, 0xffff, 0x1806);
}
static int cat_amp_poll(struct cat_amp_io *io, u16 mask, u16 value)
{
	unsigned int i, v;
	int ret;
	for (i = 0; i < 10; i++) {
		ret = io->read(io->ctx, 1, &v);
		if (ret) return ret;
		if ((v & mask) == value) return 0;
		io->wait(io->ctx, 2000, 2020);
	}
	return -ETIMEDOUT;
}
/* Muted preparation ONLY. No unmute, fades, monitoring or retries/reloads.
 * Efuses are read live, not replaced with the earlier observed samples.
 */
static int cat_amp_prepare(struct cat_amp_io *io, const struct cat_amp_profile profiles[2], u16 saved)
{
	const struct cat_amp_profile *p = &profiles[0];
	unsigned int i, pid, a, b, base, off, pass = 0;
	int ret, ical, vcal;
	long result;
#define CAT_STEP(expr) do { ret = (expr); if (ret) goto fail; } while (0)
	io->reset(io->ctx, 1);
	io->wait(io->ctx, 1000, 1010);
	io->reset(io->ctx, 0);
	io->wait(io->ctx, 1000, 1010);
	CAT_STEP(cat_amp_check(io, 0, 0xffff, 0x1806));
	CAT_STEP(cat_amp_check(io, 4, 1, 1));
	CAT_STEP(cat_amp_check(io, 8, 1, 1));
	CAT_STEP(cat_amp_dsp_read(io, 0x1f80, &pid));
	io->observed_pid = pid;
	if (pid) { ret = -ENODEV; goto fail; }
load_profile:
	for (i = 0; i < p->regs.size; i += 4) {
		u16 reg = cat_le16(p->regs.data + i), val = cat_le16(p->regs.data + i + 2);
		if (reg > 0x79) { ret = -EINVAL; goto fail; }
		if (reg == 3) val = 0xffff;
		if (reg == 4) val |= 7; /* powerdown, amplifier down, DSP bypass */
		if (reg == 8) val |= 1; /* never remove hardware mute while loading */
		if (reg == 0x0f) val = (val & 0xff) | 0x4000; /* fixed -24 dB */
		CAT_STEP(io->write(io->ctx, reg, val));
	}
	CAT_STEP(cat_amp_check(io, 8, 1, 1));
	CAT_STEP(cat_amp_check(io, 0x0f, 0xff00, 0x4000));
	CAT_STEP(cat_amp_update(io, 4, 1, 0)); /* power PLL, amplifier still down */
	io->wait(io->ctx, 2000, 2100);
	CAT_STEP(cat_amp_poll(io, 1, 1)); /* CAT c0c67690 PLLS */
	CAT_STEP(cat_amp_update(io, 0x11, 0x1000, 0)); /* OSC for memory upload */
	CAT_STEP(cat_amp_upload(io, 0x8c00, p->fw));
	CAT_STEP(cat_amp_upload(io, 0x8600, p->cfg));
	CAT_STEP(cat_amp_dsp_read(io, 0x866d, &base));
	CAT_STEP(io->read(io->ctx, 0x78, &a));
	CAT_STEP(io->read(io->ctx, 0x79, &b));
	ical = 1000 + 2 * (int)(signed char)(a & 0xff);
	vcal = 1000 + 5 * ((int)(short)b >> 10);
	if (base / 2 <= 204 || ical <= 0 || vcal <= 0) { ret = -ERANGE; goto fail; }
	result = ((0x3a37L * vcal / ical) * base) >> 12;
	if (result < 0 || result > 65535) { ret = -ERANGE; goto fail; }
	CAT_STEP(cat_amp_dsp_write(io, 0x866d, result));
	CAT_STEP(cat_amp_dsp_read(io, 0x8678, &off));
	if (!saved || saved > 65535 - off) { ret = -ERANGE; goto fail; }
	CAT_STEP(cat_amp_dsp_write(io, 0x8677, saved + off));
	CAT_STEP(cat_amp_update(io, 0x11, 0x1000, 0x1000)); /* PLL memory */
	CAT_STEP(cat_amp_dsp_read(io, 0x1f80, &pid));
	io->observed_pid = pid;
	/* CAT c0c6bb70: post-load PID3 selects pass1. Exactly one switch;
	 * never loop/retry an unstable or unknown PID. Still hardware-muted.
	 */
	if (pid == 0x6e90 && !pass) {
		pass = 1;
		p = &profiles[1];
		goto load_profile;
	}
	if (pid != (pass ? 0x6e90 : 0)) { ret = -ENODEV; goto fail; }
	CAT_STEP(cat_amp_update(io, 4, 2, 0));
	io->wait(io->ctx, 1000, 1050);
	CAT_STEP(cat_amp_poll(io, 0x4b3b, 0x0311)); /* CAT c0c6d218 */
	CAT_STEP(cat_amp_update(io, 4, 4, 0)); /* DSP run, remains muted */
	io->wait(io->ctx, 2000, 2010);
	CAT_STEP(io->read(io->ctx, 0x42, &a));
	if (!a) { ret = -EIO; goto fail; }
	CAT_STEP(cat_amp_update(io, 6, 1, 1));
	CAT_STEP(cat_amp_check(io, 8, 1, 1));
	CAT_STEP(cat_amp_check(io, 0x0f, 0xff00, 0x4000));
	return 0;
fail:
	io->reset(io->ctx, 1);
	return ret;
#undef CAT_STEP
}
static int cat_amp_health(struct cat_amp_io *io)
{
	unsigned int watchdog;
	int ret = cat_amp_check(io, 0x0f, 0xff00, 0x4000);

	if (!ret) ret = cat_amp_poll(io, 0x4b3b, 0x0311);
	if (!ret) ret = io->read(io->ctx, 0x42, &watchdog);
	if (!ret && !watchdog) ret = -EIO;
	if (ret) io->reset(io->ctx, 1);
	return ret;
}
static int cat_amp_unmute(struct cat_amp_io *io)
{
	int ret = cat_amp_health(io);
	if (!ret) ret = cat_amp_update(io, 8, 1, 0);
	if (ret) io->reset(io->ctx, 1);
	return ret;
}
/* Reset is unconditional, even when the bus no longer answers. No fade. */
static void cat_amp_stop(struct cat_amp_io *io)
{
	cat_amp_update(io, 8, 1, 1);
	io->wait(io->ctx, 4000, 4100);
	cat_amp_update(io, 6, 1, 0);
	io->wait(io->ctx, 1000, 1100);
	io->write(io->ctx, 3, 0xffff);
	cat_amp_update(io, 4, 7, 7);
	io->reset(io->ctx, 1);
}
#endif
