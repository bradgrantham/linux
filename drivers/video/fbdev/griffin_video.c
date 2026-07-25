// SPDX-License-Identifier: GPL-2.0
/*
 * Griffin framebuffer (fbdev) + VIDEO vsync IRQ handling.
 *
 * Display pipeline: the ENGINE CPLD (0xD00000) bus-masters the framebuffer
 * from RAM into a FIFO; the VIDEO CPLD (0xE00000) drains it as VGA 640x480@60
 * 1bpp.  ENGINE's SOURCE_PAGE register is only A[23:16], so the framebuffer
 * must be 64 KiB-aligned -- it comes from a reserved-memory carveout named in
 * this node's "memory-region" (no runtime aligned-allocator needed on nommu).
 *
 * Memory layout ("palette-and-pixels", griffin.yml): each scanline is an
 * 84-byte record: 4-byte header (byte 0 = fg, byte 1 = bg, both R3G3B2,
 * latched in-band by VIDEO during hblank; bytes 2-3 reserved) followed by 80
 * pixel bytes (640 px, 1bpp, MSB leftmost).  fbdev/fbcon can't describe the
 * in-line headers, so screen_base points 4 bytes past the carveout: line N's
 * pixels then sit exactly at screen_base + N*84 with line_length 84, and the
 * 4 "slack" bytes at the end of each fbdev row are really line N+1's header.
 * cfb_* drawing ops are row-bounded to xres (80 bytes), so they never touch
 * the headers, which are pre-filled once at probe (white on black).
 *
 * Enable order (griffin.yml): quiesce first (defensive -- u-boot already
 * did), fill memory, SOURCE_PAGE, ENGINE.DMA_EN, then VIDEO.ENABLE.
 *
 * vsync IRQ: the latch has no mask of its own -- CTRL.IRQENB gates only the
 * pin (added to the CPLD for exactly this driver's benefit) and the latch
 * sets every frame regardless.  IRQENB is raised only after the ack handler
 * is installed, and any stale latch is acked first; without that ordering a
 * pending vsync would livelock the CPU the moment IRQENB went up.
 */

#include <linux/console.h>
#include <linux/fb.h>
#include <linux/font.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

/* VIDEO registers (byte-wide, odd addresses; reg index 0). */
#define VIDEO_CLRINT	0x03	/* w: write any value to ack the vsync latch */
#define VIDEO_CTRL	0x05	/* w: control */
#define VIDEO_CTRL_ENABLE	BIT(0)
#define VIDEO_CTRL_IRQENB	BIT(1)

/* ENGINE registers (byte-wide, odd addresses; reg index 1). */
#define ENGINE_SOURCE_PAGE	0x03	/* w: framebuffer A[23:16] */
#define ENGINE_CTRL		0x05	/* w: bit0 DMA_EN */
#define ENGINE_CTRL_DMA_EN	BIT(0)

#define GRIFFIN_XRES		640
#define GRIFFIN_YRES		480
#define GRIFFIN_LINE_HDR	4
#define GRIFFIN_LINE_PIXBYTES	80
#define GRIFFIN_LINE_STRIDE	(GRIFFIN_LINE_HDR + GRIFFIN_LINE_PIXBYTES)
#define GRIFFIN_FB_SIZE		(GRIFFIN_LINE_STRIDE * GRIFFIN_YRES)

#define GRIFFIN_FG_R3G3B2	0xFF	/* white */
#define GRIFFIN_BG_R3G3B2	0x00	/* black */

struct griffin_fb {
	void __iomem *video;
	void __iomem *engine;
	void __iomem *fb;	/* carveout base (headers included) */
	struct fb_info *info;
};

/*
 * Early framebuffer boot console: renders printk into the framebuffer from
 * early kernel init, so the display stays live between u-boot (which leaves
 * the ENGINE/VIDEO pipeline running on the same carveout) and fbcon.
 *
 * Registered directly as a CON_BOOT console from a console_initcall rather
 * than through the earlycon framework: earlycon supports exactly one early
 * console and the DUART already claims it (setup_earlycon -> -EALREADY for
 * a second).  CON_PRINTBUFFER replays everything printed before
 * registration, so nothing is lost, and like any boot console it is
 * auto-unregistered when the first real console (tty0/fbcon) comes up.
 *
 * It runs long before the DT platform device exists, and on this machine
 * every address is fixed by construction (the carveout must be the
 * 64 KiB-aligned reserved-memory block, SOURCE_PAGE is A[23:16] only), so
 * the addresses are compiled in.  Rendering is a micro-textport: glyph
 * blits from the built-in VGA8x16 font, one contiguous memmove per scroll
 * -- the same layout tricks as the fbdev driver below.  The probe of the
 * real driver silences it (it would otherwise scribble over fbcon's screen
 * until the boot console is unregistered).
 */

#define GRIFFIN_EARLY_FB	((u8 *)0x007f0000)
#define GRIFFIN_EARLY_ENGINE	((u8 *)0x00d00000)
#define GRIFFIN_EARLY_VIDEO	((u8 *)0x00e00000)
#define GRIFFIN_EARLY_COLS	(GRIFFIN_XRES / 8)
#define GRIFFIN_EARLY_ROWS	(GRIFFIN_YRES / 16)

static struct {
	const struct font_desc *font;
	int x, y;
	bool stop;		/* set by griffin_video_probe: fbcon owns the fb now */
} griffin_earlyfb;

static void griffin_earlyfb_scroll(void)
{
	u8 *fb = GRIFFIN_EARLY_FB;
	int line;

	/* One contiguous block move, headers over identical headers. */
	memmove(fb, fb + 16 * GRIFFIN_LINE_STRIDE,
		(GRIFFIN_EARLY_ROWS - 1) * 16 * GRIFFIN_LINE_STRIDE);
	for (line = (GRIFFIN_EARLY_ROWS - 1) * 16; line < GRIFFIN_YRES; line++)
		memset(fb + line * GRIFFIN_LINE_STRIDE + GRIFFIN_LINE_HDR, 0,
		       GRIFFIN_LINE_PIXBYTES);
}

static void griffin_earlyfb_putc(char c)
{
	const u8 *glyph;
	u8 *dst;
	int row;

	if (c == '\n') {
		griffin_earlyfb.x = 0;
		griffin_earlyfb.y++;
	} else if (c == '\r') {
		griffin_earlyfb.x = 0;
		return;
	} else if (c == '\t') {
		griffin_earlyfb.x = (griffin_earlyfb.x + 8) & ~7;
	} else if ((unsigned char)c >= 0x20) {
		if (griffin_earlyfb.x >= GRIFFIN_EARLY_COLS) {
			griffin_earlyfb.x = 0;
			griffin_earlyfb.y++;
		}
		if (griffin_earlyfb.y >= GRIFFIN_EARLY_ROWS) {
			griffin_earlyfb_scroll();
			griffin_earlyfb.y = GRIFFIN_EARLY_ROWS - 1;
		}
		glyph = (const u8 *)griffin_earlyfb.font->data +
			(unsigned char)c * 16;
		dst = GRIFFIN_EARLY_FB +
			griffin_earlyfb.y * 16 * GRIFFIN_LINE_STRIDE +
			GRIFFIN_LINE_HDR + griffin_earlyfb.x;
		for (row = 0; row < 16; row++) {
			*dst = glyph[row];
			dst += GRIFFIN_LINE_STRIDE;
		}
		griffin_earlyfb.x++;
		return;
	} else {
		return;
	}

	if (griffin_earlyfb.y >= GRIFFIN_EARLY_ROWS) {
		griffin_earlyfb_scroll();
		griffin_earlyfb.y = GRIFFIN_EARLY_ROWS - 1;
	}
}

static void griffin_earlyfb_write(struct console *con, const char *s,
				  unsigned int count)
{
	if (griffin_earlyfb.stop)
		return;
	while (count--)
		griffin_earlyfb_putc(*s++);
}

static struct console griffin_earlyfb_console = {
	.name	= "griffin_fb",
	.write	= griffin_earlyfb_write,
	/* CON_ENABLED preset (the netconsole pattern): the cmdline carries
	 * console=tty0/ttyS0, and an extra console that matches neither is
	 * only accepted pre-enabled. */
	.flags	= CON_PRINTBUFFER | CON_BOOT | CON_ENABLED,
	.index	= 0,
};

static int __init griffin_earlyfb_init(void)
{
	u8 *fb = GRIFFIN_EARLY_FB;
	int line;

	/* FB_GRIFFIN can be built under COMPILE_TEST; only ever touch the
	 * hardware on the real machine. */
	if (!of_machine_is_compatible("griffin,griffin"))
		return 0;

	griffin_earlyfb.font = find_font("VGA8x16");
	if (!griffin_earlyfb.font)
		return -ENODEV;

	/* Fresh screen: black pixels, white-on-black headers.  Then make
	 * sure the pipeline is lit (u-boot normally left it running on this
	 * very carveout; these writes are idempotent) -- SOURCE_PAGE,
	 * ENGINE, then VIDEO, never IRQENB (no ack handler yet). */
	for (line = 0; line < GRIFFIN_YRES; line++) {
		u8 *hdr = fb + line * GRIFFIN_LINE_STRIDE;

		hdr[0] = GRIFFIN_FG_R3G3B2;
		hdr[1] = GRIFFIN_BG_R3G3B2;
		hdr[2] = 0;
		hdr[3] = 0;
		memset(hdr + GRIFFIN_LINE_HDR, 0, GRIFFIN_LINE_PIXBYTES);
	}
	GRIFFIN_EARLY_ENGINE[ENGINE_SOURCE_PAGE] =
		(unsigned long)GRIFFIN_EARLY_FB >> 16;
	GRIFFIN_EARLY_ENGINE[ENGINE_CTRL] = ENGINE_CTRL_DMA_EN;
	GRIFFIN_EARLY_VIDEO[VIDEO_CTRL] = VIDEO_CTRL_ENABLE;

	register_console(&griffin_earlyfb_console);
	return 0;
}
console_initcall(griffin_earlyfb_init);

static irqreturn_t griffin_vsync_irq(int irq, void *dev_id)
{
	struct griffin_fb *gf = dev_id;

	/* vsync is VIDEO's only IRQ source; unconditional ack. */
	writeb(0x00, gf->video + VIDEO_CLRINT);
	return IRQ_HANDLED;
}

static const struct fb_fix_screeninfo griffin_fb_fix = {
	.id		= "griffin",
	.type		= FB_TYPE_PACKED_PIXELS,
	.visual		= FB_VISUAL_MONO10,	/* 1 = white (via line palette) */
	.line_length	= GRIFFIN_LINE_STRIDE,
	.accel		= FB_ACCEL_NONE,
};

static const struct fb_var_screeninfo griffin_fb_var = {
	.xres		= GRIFFIN_XRES,
	.yres		= GRIFFIN_YRES,
	.xres_virtual	= GRIFFIN_XRES,
	.yres_virtual	= GRIFFIN_YRES,
	.bits_per_pixel	= 1,
	.red		= { .length = 1 },
	.green		= { .length = 1 },
	.blue		= { .length = 1 },
	.activate	= FB_ACTIVATE_NOW,
	.height		= -1,
	.width		= -1,
	.vmode		= FB_VMODE_NONINTERLACED,
};

/*
 * Draw ops: the generic cfb_* engines are portability code (any bpp, any bit
 * alignment, foreign endian) and on this -m68000 build their inner loops
 * compile to an out-of-line call per 32-bit word plus __mulsi3 software
 * multiplies in setup -- ~6x slower than the ROM firmware's textport, which
 * scrolls the same buffer with unrolled move.l copies.  fbcon only ever draws
 * on 8-pixel character-cell boundaries, so everything it generates is
 * byte-aligned in this 1bpp layout; fast-path those cases with the kernel's
 * m68k memcpy/memmove/memset (hand-written unroll-by-8 move.l, the same loop
 * the firmware uses) and punt anything else to the generic ops.
 *
 * The in-band 4-byte line headers constrain the fast paths differently:
 * copyarea's full-width block move may span them (it copies headers over
 * identical headers, set once at probe), but fillrect must stay within each
 * line's 80 pixel bytes or it would overwrite the palette.
 */

static void griffin_copyarea(struct fb_info *info,
			     const struct fb_copyarea *area)
{
	u8 *base = (u8 __force *)info->screen_base;
	unsigned int stride = info->fix.line_length;
	unsigned int wbytes = area->width >> 3;
	u8 *src, *dst;
	int line;

	/* fbcon moves whole character cells: sx == dx, all multiples of 8. */
	if (((area->sx | area->dx | area->width) & 7) || area->sx != area->dx) {
		cfb_copyarea(info, area);
		return;
	}

	src = base + area->sy * stride + (area->sx >> 3);
	dst = base + area->dy * stride + (area->dx >> 3);

	if (area->sx == 0 && wbytes == GRIFFIN_LINE_PIXBYTES) {
		/* Full-width (the scroll bmove): one contiguous block,
		 * interior headers copied over identical headers. */
		memmove(dst, src, (area->height - 1) * stride + wbytes);
		return;
	}

	/* Partial-width run: per scanline, ordered so overlapping rows are
	 * read before they are overwritten. */
	if (area->dy <= area->sy) {
		for (line = 0; line < area->height; line++)
			memmove(dst + line * stride, src + line * stride,
				wbytes);
	} else {
		for (line = area->height - 1; line >= 0; line--)
			memmove(dst + line * stride, src + line * stride,
				wbytes);
	}
}

static void griffin_fillrect(struct fb_info *info,
			     const struct fb_fillrect *rect)
{
	u8 *base = (u8 __force *)info->screen_base;
	unsigned int stride = info->fix.line_length;
	u8 val = (rect->color & 1) ? 0xFF : 0x00;
	u8 *dst;
	unsigned int line;

	if (((rect->dx | rect->width) & 7) || rect->rop != ROP_COPY) {
		cfb_fillrect(info, rect);
		return;
	}

	/* Per scanline: never cross the in-band palette headers. */
	dst = base + rect->dy * stride + (rect->dx >> 3);
	for (line = 0; line < rect->height; line++)
		memset(dst + line * stride, val, rect->width >> 3);
}

static void griffin_imageblit(struct fb_info *info,
			      const struct fb_image *image)
{
	u8 *base = (u8 __force *)info->screen_base;
	unsigned int stride = info->fix.line_length;
	unsigned int wbytes = image->width >> 3;
	const u8 *src = image->data;
	u8 fg = (image->fg_color & 1) ? 0xFF : 0x00;
	u8 bg = (image->bg_color & 1) ? 0xFF : 0x00;
	u8 *dst;
	unsigned int line, i;

	if (image->depth != 1 || ((image->dx | image->width) & 7)) {
		cfb_imageblit(info, image);
		return;
	}

	/* 1bpp glyph rows onto 1bpp lines, both MSB-leftmost: each source
	 * byte maps straight to a framebuffer byte through the fg/bg pair
	 * (white-on-black is the identity). */
	dst = base + image->dy * stride + (image->dx >> 3);
	for (line = 0; line < image->height; line++) {
		for (i = 0; i < wbytes; i++) {
			u8 s = src[i];

			dst[i] = (s & fg) | (~s & bg);
		}
		src += wbytes;
		dst += stride;
	}
}

static const struct fb_ops griffin_fb_ops = {
	.owner = THIS_MODULE,
	__FB_DEFAULT_IOMEM_OPS_RDWR,
	.fb_fillrect	= griffin_fillrect,
	.fb_copyarea	= griffin_copyarea,
	.fb_imageblit	= griffin_imageblit,
	__FB_DEFAULT_IOMEM_OPS_MMAP,
};

static int griffin_video_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct griffin_fb *gf;
	struct device_node *mem_np;
	struct resource res;
	struct fb_info *info;
	int irq, ret, line;

	gf = devm_kzalloc(dev, sizeof(*gf), GFP_KERNEL);
	if (!gf)
		return -ENOMEM;

	gf->video = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(gf->video))
		return PTR_ERR(gf->video);
	gf->engine = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(gf->engine))
		return PTR_ERR(gf->engine);

	/* Framebuffer carveout (64 KiB-aligned; SOURCE_PAGE is A[23:16] only). */
	mem_np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!mem_np) {
		dev_err(dev, "no memory-region\n");
		return -EINVAL;
	}
	ret = of_address_to_resource(mem_np, 0, &res);
	of_node_put(mem_np);
	if (ret)
		return ret;
	if (res.start & 0xFFFF || resource_size(&res) < GRIFFIN_FB_SIZE) {
		dev_err(dev, "carveout %pR unusable (need 64K-aligned, >= %u bytes)\n",
			&res, GRIFFIN_FB_SIZE);
		return -EINVAL;
	}
	gf->fb = devm_ioremap(dev, res.start, resource_size(&res));
	if (!gf->fb)
		return -ENOMEM;

	/* From here the framebuffer belongs to fbcon; the early boot console
	 * must stop scribbling into it (printk keeps calling boot consoles
	 * until the first real console registers). */
	griffin_earlyfb.stop = true;

	/* Quiesce (defensively) and ack any stale vsync. */
	writeb(0x00, gf->engine + ENGINE_CTRL);
	writeb(0x00, gf->video + VIDEO_CTRL);
	writeb(0x00, gf->video + VIDEO_CLRINT);

	/* Pre-fill: every line header = white-on-black palette, pixels clear. */
	memset_io(gf->fb, 0, GRIFFIN_FB_SIZE);
	for (line = 0; line < GRIFFIN_YRES; line++) {
		writeb(GRIFFIN_FG_R3G3B2, gf->fb + line * GRIFFIN_LINE_STRIDE);
		writeb(GRIFFIN_BG_R3G3B2, gf->fb + line * GRIFFIN_LINE_STRIDE + 1);
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	ret = devm_request_irq(dev, irq, griffin_vsync_irq, 0,
			       "griffin-video-vsync", gf);
	if (ret)
		return ret;

	info = framebuffer_alloc(0, dev);
	if (!info)
		return -ENOMEM;
	gf->info = info;

	info->fbops = &griffin_fb_ops;
	info->var = griffin_fb_var;
	info->fix = griffin_fb_fix;
	/*
	 * The framebuffer is plain RAM (the carveout), so reads are as cheap as
	 * writes: advertise READS_FAST + COPYAREA so fbcon picks SCROLL_MOVE and
	 * relocates glyph runs with fb_copyarea (cfb_copyarea) instead of
	 * SCROLL_REDRAW re-imageblitting every character each scroll.  Requires
	 * CONFIG_FRAMEBUFFER_CONSOLE_LEGACY_ACCELERATION; without it fbcon forces
	 * SCROLL_REDRAW regardless (see fb_scrollmode()).
	 */
	info->flags = FBINFO_READS_FAST | FBINFO_HWACCEL_COPYAREA;
	/* +4: skip line 0's header so pixels land at screen_base + N*stride. */
	info->screen_base = (char __iomem *)gf->fb + GRIFFIN_LINE_HDR;
	info->screen_size = GRIFFIN_FB_SIZE - GRIFFIN_LINE_HDR;
	info->fix.smem_start = res.start + GRIFFIN_LINE_HDR;
	info->fix.smem_len = GRIFFIN_FB_SIZE - GRIFFIN_LINE_HDR;

	ret = register_framebuffer(info);
	if (ret) {
		framebuffer_release(info);
		return ret;
	}

	/* Light it up: SOURCE_PAGE, ENGINE first, then VIDEO (+IRQENB, now
	 * that the ack handler is live). */
	writeb(res.start >> 16, gf->engine + ENGINE_SOURCE_PAGE);
	writeb(ENGINE_CTRL_DMA_EN, gf->engine + ENGINE_CTRL);
	writeb(VIDEO_CTRL_ENABLE | VIDEO_CTRL_IRQENB, gf->video + VIDEO_CTRL);

	platform_set_drvdata(pdev, gf);
	dev_info(dev, "640x480x1 fb at %pa (84-byte stride, in-band palette)\n",
		 &res.start);
	return 0;
}

static void griffin_video_remove(struct platform_device *pdev)
{
	struct griffin_fb *gf = platform_get_drvdata(pdev);

	writeb(0x00, gf->engine + ENGINE_CTRL);
	writeb(0x00, gf->video + VIDEO_CTRL);
	unregister_framebuffer(gf->info);
	framebuffer_release(gf->info);
}

static const struct of_device_id griffin_video_ids[] = {
	{ .compatible = "griffin,video" },
	{ }
};
MODULE_DEVICE_TABLE(of, griffin_video_ids);

static struct platform_driver griffin_video_driver = {
	.probe	= griffin_video_probe,
	.remove	= griffin_video_remove,
	.driver	= {
		.name		= "griffin-video",
		.of_match_table	= griffin_video_ids,
	},
};
module_platform_driver(griffin_video_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Griffin 640x480x1 framebuffer (ENGINE/VIDEO CPLD pipeline)");
