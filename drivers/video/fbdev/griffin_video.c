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

#include <linux/fb.h>
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

static const struct fb_ops griffin_fb_ops = {
	.owner = THIS_MODULE,
	FB_DEFAULT_IOMEM_OPS,
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

	/* Quiesce (defensively; u-boot already did) and ack any stale vsync. */
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
