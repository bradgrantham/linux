// SPDX-License-Identifier: GPL-2.0
/*
 * Griffin VIDEO vsync IRQ handling.
 *
 * VIDEO has no interrupt-enable/mask register -- only CLRINT to acknowledge --
 * and its vsync timing generator free-runs regardless of CTRL.ENABLE (verified
 * against the Griffin emulator's model; matches "Enable after ENGINE" in
 * griffin.yml, which only makes sense if the sync generator is already running
 * before ENGINE/VIDEO are enabled for display).  So once interrupts are
 * unmasked, autovector level 6 fires every frame (~60 Hz) unconditionally, and
 * without a handler the CPU livelocks: RTE restores an SR mask below 6, the
 * still-latched vsync IRQ immediately retriggers, forever.  This is exactly
 * why the Griffin ROM firmware always installs a video ISR at boot (crt0.s)
 * regardless of whether video is in use.
 *
 * This is currently ack-only: it keeps the machine alive once IRQs are enabled
 * (required for M6's timer) with no display support yet.  M9 extends this file
 * into the real fbdev driver (framebuffer memory-region mapping, ENGINE
 * SOURCE_PAGE programming, fb_info registration) using this same probe/IRQ
 * scaffolding.
 */

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define VIDEO_CLRINT	0x03	/* w: write any value to ack the latched vsync IRQ */
#define VIDEO_CTRL	0x05	/* w: control register */

#define VIDEO_CTRL_ENABLE	BIT(0)	/* pixel output / FIFO reads (leave 0: M9) */
#define VIDEO_CTRL_IRQENB	BIT(1)	/* gate ~VIDEO_IRQ from the vsync latch */

struct griffin_video_priv {
	void __iomem *base;
};

static irqreturn_t griffin_video_irq(int irq, void *dev_id)
{
	struct griffin_video_priv *priv = dev_id;

	/* vsync is VIDEO's only IRQ source (no status bits to check, unlike the
	 * DUART's shared ISR register) -- unconditional ack. */
	writeb(0x00, priv->base + VIDEO_CLRINT);
	return IRQ_HANDLED;
}

static int griffin_video_probe(struct platform_device *pdev)
{
	struct griffin_video_priv *priv;
	int irq, ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	writeb(0x00, priv->base + VIDEO_CLRINT);	/* ack any stale latch first */

	ret = devm_request_irq(&pdev->dev, irq, griffin_video_irq, 0,
			       "griffin-video-vsync", priv);
	if (ret)
		return ret;

	/*
	 * Only now -- handler installed and any pre-existing latch acked --
	 * turn on IRQENB.  Before this, the vsync latch has no path to the
	 * CPU (IRQENB resets to 0), which is what makes a platform_driver
	 * (probed well after the kernel globally enables interrupts) safe
	 * for a source that would otherwise need a handler from boot.
	 */
	writeb(VIDEO_CTRL_IRQENB, priv->base + VIDEO_CTRL);

	platform_set_drvdata(pdev, priv);
	dev_info(&pdev->dev, "vsync IRQ acked at %pR (fbdev pending: M9)\n",
		 &pdev->resource[0]);
	return 0;
}

static const struct of_device_id griffin_video_ids[] = {
	{ .compatible = "griffin,video" },
	{ }
};
MODULE_DEVICE_TABLE(of, griffin_video_ids);

static struct platform_driver griffin_video_driver = {
	.probe	= griffin_video_probe,
	.driver	= {
		.name		= "griffin-video",
		.of_match_table	= griffin_video_ids,
	},
};
module_platform_driver(griffin_video_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Griffin VIDEO vsync IRQ handling (fbdev pending)");
