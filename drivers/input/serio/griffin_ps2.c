// SPDX-License-Identifier: GPL-2.0-only
/*
 * Griffin GLUE PS/2 keyboard frame engine — serio port.
 *
 * The GLUE CPLD assembles whole PS/2 frames in hardware and raises one
 * level-4 autovector IRQ per received byte (DT hwirq 3); STATUS.RX_READY
 * latches with the byte in RX_DATA and a write-1-to-clear to CLEAR acks it.
 * Like VIDEO's vsync latch there is no self-mask: the interrupt line stays
 * asserted until both RX_READY and TX_DONE are cleared, so the ISR must W1C
 * before returning and probe must ack stale latches before request_irq
 * (same discipline as griffin_video.c).
 *
 * This is a thin serio port: atkbd sits on top and does all scan-set-2
 * decoding.  serio->write is real (LEDs, typematic, and the reset/GETID
 * probe): host->device TX pulls CLK low via CTRL for >= 100 us (request-to-
 * send inhibit), writes the byte to TX_DATA with the odd-parity bit carried
 * in address bit 1, then releases; TX_DONE is acked by the ISR.  A keyboard
 * that re-sends BAT (0xAA) because no host ever configured it is handled by
 * atkbd's reconnect path — with a live write, it fully re-initializes the
 * device.
 *
 * Registers per griffin.yml (byte-wide, odd offsets within the GLUE
 * window); the firmware's ps2.cpp is the reference implementation.
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/serio.h>
#include <linux/slab.h>

#define DRV_NAME "griffin_ps2"

#define PS2_TX_DATA		0x09	/* w: byte to send; write starts frame */
#define PS2_TX_DATA_PARITY	0x02	/* address bit 1 = odd parity bit */
#define PS2_STATUS		0x11	/* r */
#define PS2_STATUS_RX_READY	BIT(0)
#define PS2_STATUS_TX_DONE	BIT(1)
#define PS2_STATUS_TX_ACK	BIT(2)	/* 0 = device acknowledged */
#define PS2_STATUS_RX_PARITY	BIT(3)	/* received parity bit */
#define PS2_STATUS_RX_FRAME_ERR	BIT(4)
#define PS2_CLEAR		0x11	/* w: W1C bits 0/1 — the IRQ ack */
#define PS2_CLEAR_RX_READY	BIT(0)
#define PS2_CLEAR_TX_DONE	BIT(1)
#define PS2_CTRL		0x13	/* w: bit0 drive CLK low (inhibit) */
#define PS2_CTRL_CLK		BIT(0)
#define PS2_RX_DATA		0x15	/* r: valid while RX_READY */

struct griffin_ps2 {
	struct serio *io;
	void __iomem *base;
};

/* PS/2 odd parity: the parity bit makes the total number of 1s odd. */
static bool griffin_ps2_odd_parity(u8 x)
{
	return !(hweight8(x) & 1);
}

static irqreturn_t griffin_ps2_isr(int irq, void *dev_id)
{
	struct griffin_ps2 *ps2 = dev_id;
	irqreturn_t handled = IRQ_NONE;
	u8 status;

	while ((status = readb(ps2->base + PS2_STATUS)) &
	       (PS2_STATUS_RX_READY | PS2_STATUS_TX_DONE)) {
		u8 ack = 0;

		if (status & PS2_STATUS_RX_READY) {
			u8 byte = readb(ps2->base + PS2_RX_DATA);
			unsigned int flags = 0;

			if (status & PS2_STATUS_RX_FRAME_ERR)
				flags |= SERIO_FRAME;
			if (!!(status & PS2_STATUS_RX_PARITY) !=
			    griffin_ps2_odd_parity(byte))
				flags |= SERIO_PARITY;
			serio_interrupt(ps2->io, byte, flags);
			ack |= PS2_CLEAR_RX_READY;
		}
		if (status & PS2_STATUS_TX_DONE)
			ack |= PS2_CLEAR_TX_DONE;

		/* W1C before leaving: the level-4 line stays asserted until
		 * both latches are clear (no self-mask). */
		writeb(ack, ps2->base + PS2_CLEAR);
		handled = IRQ_HANDLED;
	}

	return handled;
}

static int griffin_ps2_write(struct serio *io, unsigned char val)
{
	struct griffin_ps2 *ps2 = io->port_data;

	/* Request-to-send: inhibit via CLK low >= 100 us, then trigger the
	 * frame; the odd-parity bit rides address bit 1 (griffin.yml).
	 * Completion (TX_DONE) is acked in the ISR. */
	writeb(PS2_CTRL_CLK, ps2->base + PS2_CTRL);
	udelay(150);
	writeb(val, ps2->base + PS2_TX_DATA +
	       (griffin_ps2_odd_parity(val) ? PS2_TX_DATA_PARITY : 0));
	writeb(0, ps2->base + PS2_CTRL);

	return 0;
}

static void griffin_ps2_ack_stale(struct griffin_ps2 *ps2)
{
	if (readb(ps2->base + PS2_STATUS) & PS2_STATUS_RX_READY)
		(void)readb(ps2->base + PS2_RX_DATA);
	writeb(PS2_CLEAR_RX_READY | PS2_CLEAR_TX_DONE, ps2->base + PS2_CLEAR);
}

static int griffin_ps2_open(struct serio *io)
{
	struct griffin_ps2 *ps2 = io->port_data;

	griffin_ps2_ack_stale(ps2);
	return 0;
}

static int griffin_ps2_probe(struct platform_device *pdev)
{
	struct griffin_ps2 *ps2;
	struct serio *serio;
	int error, irq;

	ps2 = devm_kzalloc(&pdev->dev, sizeof(*ps2), GFP_KERNEL);
	if (!ps2)
		return -ENOMEM;

	ps2->base = devm_platform_get_and_ioremap_resource(pdev, 0, NULL);
	if (IS_ERR(ps2->base))
		return PTR_ERR(ps2->base);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return -ENXIO;

	/* Ack anything latched by the boot chain before the IRQ can fire. */
	griffin_ps2_ack_stale(ps2);

	error = devm_request_irq(&pdev->dev, irq, griffin_ps2_isr, 0,
				 DRV_NAME, ps2);
	if (error) {
		dev_err(&pdev->dev, "could not request IRQ %d\n", irq);
		return error;
	}

	serio = kzalloc_obj(*serio);
	if (!serio)
		return -ENOMEM;

	serio->id.type		= SERIO_8042;
	serio->write		= griffin_ps2_write;
	serio->open		= griffin_ps2_open;
	strscpy(serio->name, dev_name(&pdev->dev), sizeof(serio->name));
	strscpy(serio->phys, dev_name(&pdev->dev), sizeof(serio->phys));
	serio->port_data	= ps2;
	serio->dev.parent	= &pdev->dev;
	ps2->io			= serio;

	serio_register_port(ps2->io);
	platform_set_drvdata(pdev, ps2);
	dev_info(&pdev->dev, "GLUE PS/2 frame engine (irq %d)\n", irq);

	return 0;
}

static void griffin_ps2_remove(struct platform_device *pdev)
{
	struct griffin_ps2 *ps2 = platform_get_drvdata(pdev);

	serio_unregister_port(ps2->io);
}

static const struct of_device_id griffin_ps2_match[] = {
	{ .compatible = "griffin,glue" },
	{ }
};
MODULE_DEVICE_TABLE(of, griffin_ps2_match);

static struct platform_driver griffin_ps2_driver = {
	.probe		= griffin_ps2_probe,
	.remove		= griffin_ps2_remove,
	.driver	= {
		.name	= DRV_NAME,
		.of_match_table = griffin_ps2_match,
	},
};
module_platform_driver(griffin_ps2_driver);

MODULE_DESCRIPTION("Griffin GLUE PS/2 frame engine serio driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
