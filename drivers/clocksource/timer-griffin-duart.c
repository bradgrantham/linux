// SPDX-License-Identifier: GPL-2.0
/*
 * Griffin XR68C681 DUART counter/timer clockevent.
 *
 * Uses the DUART's 16-bit counter/timer in Timer mode (ACR[6]=1, X1/CLK) as a
 * periodic tick source.  In this mode the counter is a free-running square-wave
 * generator: once started (STARTCC) it reloads from the preload and keeps
 * running forever -- STOPCC only clears the latched IRQ status, it does not
 * halt the counter (verified against the Griffin emulator's DUART model).
 * That makes the hardware inherently periodic, not one-shot-reprogrammable, so
 * this driver only implements CLOCK_EVT_FEAT_PERIODIC: set_state_periodic and
 * set_state_shutdown mask/unmask the interrupt at IMR rather than touching the
 * counter itself.
 *
 * IRQ: RXRDYA and CTR_READY share DUART autovector level 5 (hwirq 4 in the
 * generic m68k intc domain -- see griffin.dts for the level-to-hwirq mapping).
 * No RX-driven tty exists yet (that's M5), so IMR only needs CTR_READY here,
 * but the IRQ is requested IRQF_SHARED and the handler checks ISR before
 * treating the interrupt as its own, so M5 can add its own handler on the same
 * line without touching this file.
 *
 * NB: IMR is write-only (no readback register), so any driver that later
 * shares this line must keep its own shadow and OR/AND bits into it rather
 * than assume it can read-modify-write IMR -- same caveat as the ACR sharing
 * risk noted in u-boot's serial_xr68c681.c for real hardware.
 */

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of.h>

#include "timer-of.h"

/* DUART register offsets (byte-wide, odd addresses). */
#define DUART_ACR	0x09	/* w: auxiliary control register */
#define DUART_ISR	0x0B	/* r: interrupt status register */
#define DUART_IMR	0x0B	/* w: interrupt mask register */
#define DUART_CTUR	0x0D	/* w: counter/timer upper preload */
#define DUART_CTLR	0x0F	/* w: counter/timer lower preload */
#define DUART_STARTCC	0x1D	/* r: start counter/timer command */
#define DUART_STOPCC	0x1F	/* r: stop counter/timer command (ack in Timer mode) */

#define DUART_ISR_CTR_READY	BIT(3)
#define DUART_IMR_CTR_READY	BIT(3)

#define DUART_ACR_CT_MODE_TIMER_X1_CLK	(6 << 4)	/* Timer, X1/CLK, BRG_SET=0 */

#define DUART_CLOCK_DEFAULT	3686400

static int griffin_timer_set_state_periodic(struct clock_event_device *evt)
{
	struct timer_of *to = to_timer_of(evt);

	writeb(DUART_IMR_CTR_READY, timer_of_base(to) + DUART_IMR);
	return 0;
}

static int griffin_timer_set_state_shutdown(struct clock_event_device *evt)
{
	struct timer_of *to = to_timer_of(evt);

	writeb(0x00, timer_of_base(to) + DUART_IMR);
	return 0;
}

static irqreturn_t griffin_timer_irq(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;
	struct timer_of *to = to_timer_of(evt);
	void __iomem *base = timer_of_base(to);

	if (!(readb(base + DUART_ISR) & DUART_ISR_CTR_READY))
		return IRQ_NONE;	/* not ours -- shared with the RX line */

	readb(base + DUART_STOPCC);	/* ack: clears IRQ status, keeps running */
	evt->event_handler(evt);
	return IRQ_HANDLED;
}

static struct clock_event_device griffin_clockevent_device = {
	.name			= "griffin_duart_timer",
	.features		= CLOCK_EVT_FEAT_PERIODIC,
	.set_state_periodic	= griffin_timer_set_state_periodic,
	.set_state_shutdown	= griffin_timer_set_state_shutdown,
	.rating			= 200,
};

static int __init griffin_timer_init_of(struct device_node *np)
{
	u32 duart_clock = DUART_CLOCK_DEFAULT;
	struct timer_of *to;
	void __iomem *base;
	u32 preload;
	int ret;

	to = kzalloc(sizeof(*to), GFP_KERNEL);
	if (!to)
		return -ENOMEM;

	to->flags = TIMER_OF_BASE | TIMER_OF_IRQ;
	to->of_irq.handler = griffin_timer_irq;
	to->of_irq.flags = IRQF_TIMER | IRQF_SHARED;
	ret = timer_of_init(np, to);
	if (ret)
		return ret;

	of_property_read_u32(np, "clock-frequency", &duart_clock);
	base = timer_of_base(to);

	/* Ensure masked before programming. */
	writeb(0x00, base + DUART_IMR);

	/*
	 * Timer mode fires once per full square-wave period = 2*preload input
	 * cycles (matches the 68681 family and the Griffin emulator's model),
	 * so preload = duart_clock / (2*HZ) gives an exact HZ-rate tick with no
	 * rounding error for Griffin's 3.6864 MHz / 100 Hz combination
	 * (preload = 18432 = 0x4800).
	 */
	preload = duart_clock / (2 * HZ);
	if (preload == 0 || preload > 0x10000)
	{
		pr_err("%pOF: HZ=%u not representable with a 16-bit preload at %u Hz\n",
		       np, HZ, duart_clock);
		return -EINVAL;
	}
	writeb((preload >> 8) & 0xFF, base + DUART_CTUR);
	writeb(preload & 0xFF, base + DUART_CTLR);

	/* ACR[7] (BRG_SET) left 0 to match the earlycon/serial driver's baud
	 * setup; only bits 6:4 (C/T mode) are ours to set. */
	writeb(DUART_ACR_CT_MODE_TIMER_X1_CLK, base + DUART_ACR);
	readb(base + DUART_STARTCC);	/* start the free-running counter */

	griffin_clockevent_device.cpumask = cpu_possible_mask;
	griffin_clockevent_device.irq = to->of_irq.irq;
	to->clkevt = griffin_clockevent_device;

	clockevents_config_and_register(&to->clkevt, HZ, 1, 1);

	return 0;
}

TIMER_OF_DECLARE(griffin_duart, "griffin,duart-timer", griffin_timer_init_of);
