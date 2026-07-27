// SPDX-License-Identifier: GPL-2.0
/*
 * Griffin XR68C681 DUART: earlycon, full interrupt-driven tty console, and
 * the periodic clockevent, all in one driver.
 *
 * These three pieces share ONE physical chip and, critically, one write-only
 * IMR (interrupt mask register) with no readback -- a real 68681 datasheet
 * limitation, not a Griffin simplification. Two independently-probed drivers
 * each blindly writing IMR would silently clobber each other's enabled bit
 * (this used to be two files: this tty driver and a separate
 * drivers/clocksource/timer-griffin-duart.c; merged after hitting exactly
 * that hazard implementing RX/TX). Real 68681-family drivers upstream
 * (sccnxp.c) solve the same problem the same way: one driver instance owns
 * the whole chip's interrupt-relevant register space via a single in-memory
 * IMR shadow.
 *
 * Two init paths, run at different times, on purpose:
 *   - griffin_duart_timer_init() (TIMER_OF_DECLARE) runs early, from
 *     time_init()/timer_probe(), well before tty_init() -- required, since
 *     jiffies/scheduling need a working clockevent that early. It owns the
 *     ioremap, the IRQ (requested once, for the chip's whole lifetime), and
 *     the IMR shadow.
 *   - griffin_uart_probe() (a normal platform_driver, bound through the
 *     generic DT machine's of_platform_populate()) runs much later, safely
 *     after tty_init(), and is what registering a uart_port/tty device
 *     actually requires. It reuses the already-live IRQ and IMR shadow from
 *     the early half -- no second request_irq(), no separate lock.
 * The one shared ISR (installed once, early) checks CTR_READY unconditionally
 * and RXRDY/TXRDY only once the late half has set port_active, so before that
 * point the tty bits are simply never true and nothing checks for them.
 */

#include <linux/clockchips.h>
#include <linux/console.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kfifo.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/spinlock.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>

/* Register offsets (byte-wide, odd addresses within the DUART window).
 * Channel B mirrors channel A at +0x10; SRB/CRB share SRA/CRA's bit
 * layouts. */
#define XR_MR1A		0x01	/* rw: MR1A/MR2A share offset, ptr auto-advances */
#define XR_SRA		0x03	/* r:  channel A status register */
#define XR_CSRA		0x03	/* w:  channel A clock select */
#define XR_CRA		0x05	/* w:  channel A command register */
#define XR_RBA		0x07	/* r:  channel A receive buffer */
#define XR_TBA		0x07	/* w:  channel A transmit buffer */
#define XR_ACR		0x09	/* w:  auxiliary control register */
#define XR_ISR		0x0B	/* r:  interrupt status register */
#define XR_IMR		0x0B	/* w:  interrupt mask register (write-only!) */
#define XR_CTUR		0x0D	/* w:  counter/timer upper preload */
#define XR_CTLR		0x0F	/* w:  counter/timer lower preload */
#define XR_MR1B		0x11	/* rw: MR1B/MR2B share offset, ptr auto-advances */
#define XR_SRB		0x13	/* r:  channel B status register */
#define XR_CSRB		0x13	/* w:  channel B clock select */
#define XR_CRB		0x15	/* w:  channel B command register */
#define XR_RBB		0x17	/* r:  channel B receive buffer */
#define XR_TBB		0x17	/* w:  channel B transmit buffer */
#define XR_STARTCC	0x1D	/* r:  start counter/timer command */
#define XR_STOPCC	0x1F	/* r:  stop counter/timer command (Timer mode: ack only) */

#define XR_SRA_RXRDY	BIT(0)
#define XR_SRA_TXRDY	BIT(2)
#define XR_SRA_TXEMT	BIT(3)
#define XR_SRA_OE	BIT(4)
#define XR_SRA_PE	BIT(5)
#define XR_SRA_FE	BIT(6)

#define XR_ISR_TXRDYA		BIT(0)
#define XR_ISR_RXRDYA		BIT(1)
#define XR_ISR_CTR_READY	BIT(3)
#define XR_ISR_TXRDYB		BIT(4)
#define XR_ISR_RXRDYB		BIT(5)

#define XR_IMR_TXRDYA		BIT(0)
#define XR_IMR_RXRDYA		BIT(1)
#define XR_IMR_CTR_READY	BIT(3)
#define XR_IMR_TXRDYB		BIT(4)
#define XR_IMR_RXRDYB		BIT(5)

#define XR_ACR_CT_MODE_TIMER_X1_CLK	(6 << 4)	/* Timer, X1/CLK, BRG_SET=0 */

#define DUART_CLOCK_DEFAULT	3686400

#define PORT_GRIFFIN		1

/* ------------------------------------------------------------------------
 * earlycon: independent of everything below (its own temporary mapping via
 * the generic earlycon mechanism), needed for output before start_kernel().
 * ------------------------------------------------------------------------ */

static void griffin_duart_early_putc(struct uart_port *port, unsigned char c)
{
	while (!(readb(port->membase + XR_SRA) & XR_SRA_TXRDY))
		cpu_relax();
	writeb(c, port->membase + XR_TBA);
}

static void griffin_duart_early_write(struct console *con, const char *s,
				      unsigned int n)
{
	struct earlycon_device *dev = con->data;

	uart_console_write(&dev->port, s, n, griffin_duart_early_putc);
}

static int __init griffin_duart_earlycon_setup(struct earlycon_device *device,
					       const char *options)
{
	if (!device->port.membase)
		return -ENODEV;

	device->con->write = griffin_duart_early_write;
	return 0;
}

OF_EARLYCON_DECLARE(griffin_duart, "griffin,duart-xr68c681",
		    griffin_duart_earlycon_setup);

/* ------------------------------------------------------------------------
 * Shared state: one instance (Griffin has exactly one DUART).
 * ------------------------------------------------------------------------ */

/* One tty channel of the DUART: the uart_port plus everything that differs
 * between channels A and B (register offsets, IMR bits).  The chip-shared
 * pieces (base, IMR shadow, clockevent, the one IRQ) stay in griffin_duart;
 * since Griffin has exactly one DUART, the ops reach it via the &gd
 * singleton and get the channel via container_of on the embedded port. */
struct griffin_uart_chan {
	struct uart_port port;
	bool active;		/* set once startup() has run for this channel */
	u8 sr, cr, rb, tb;	/* register offsets */
	u8 imr_rx, imr_tx;	/* this channel's IMR/ISR bits */
};

struct griffin_duart {
	void __iomem *base;
	spinlock_t imr_lock;
	u8 imr_shadow;

	struct clock_event_device clkevt;

	struct griffin_uart_chan chan[2];	/* 0 = A (console), 1 = B */
};

static struct griffin_duart gd;

static inline struct griffin_uart_chan *to_chan(struct uart_port *port)
{
	return container_of(port, struct griffin_uart_chan, port);
}

static void griffin_imr_update(struct griffin_duart *d, u8 set_mask, u8 clear_mask)
{
	unsigned long flags;

	spin_lock_irqsave(&d->imr_lock, flags);
	d->imr_shadow = (d->imr_shadow & ~clear_mask) | set_mask;
	writeb(d->imr_shadow, d->base + XR_IMR);
	spin_unlock_irqrestore(&d->imr_lock, flags);
}

/* ------------------------------------------------------------------------
 * tty RX/TX (called from the shared ISR once port_active).
 * ------------------------------------------------------------------------ */

static void griffin_uart_rx_chars(struct griffin_duart *d,
				  struct griffin_uart_chan *c)
{
	struct uart_port *port = &c->port;
	u8 sr, ch;
	char flag;

	while ((sr = readb(d->base + c->sr)) & XR_SRA_RXRDY) {
		ch = readb(d->base + c->rb);
		flag = TTY_NORMAL;
		port->icount.rx++;

		if (sr & (XR_SRA_OE | XR_SRA_PE | XR_SRA_FE)) {
			if (sr & XR_SRA_FE) {
				port->icount.frame++;
				flag = TTY_FRAME;
			} else if (sr & XR_SRA_PE) {
				port->icount.parity++;
				flag = TTY_PARITY;
			}
			if (sr & XR_SRA_OE)
				port->icount.overrun++;
			writeb(0x40, d->base + c->cr);	/* MC=4: reset error status */
		}

		if (uart_handle_sysrq_char(port, ch))
			continue;

		uart_insert_char(port, sr, XR_SRA_OE, ch, flag);
	}

	tty_flip_buffer_push(&port->state->port);
}

#define GRIFFIN_WAKEUP_CHARS 256

static void griffin_uart_stop_tx(struct uart_port *port);

static void griffin_uart_tx_chars(struct griffin_duart *d,
				  struct griffin_uart_chan *c)
{
	struct uart_port *port = &c->port;
	struct tty_port *tport = &port->state->port;
	unsigned char ch;

	if (port->x_char) {
		writeb(port->x_char, d->base + c->tb);
		port->icount.tx++;
		port->x_char = 0;
		return;
	}

	if (kfifo_is_empty(&tport->xmit_fifo) || uart_tx_stopped(port)) {
		griffin_uart_stop_tx(port);
		return;
	}

	while (readb(d->base + c->sr) & XR_SRA_TXRDY) {
		if (!uart_fifo_get(port, &ch))
			break;
		writeb(ch, d->base + c->tb);
		port->icount.tx++;
	}

	if (kfifo_len(&tport->xmit_fifo) < GRIFFIN_WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (kfifo_is_empty(&tport->xmit_fifo))
		griffin_uart_stop_tx(port);
}

/* ------------------------------------------------------------------------
 * Shared ISR: CTR_READY always meaningful; RXRDY/TXRDY only once a tty port
 * exists.  Only ever one request_irq() for this line (see file header), so
 * no IRQF_SHARED/IRQ_NONE dance is needed between "sub-drivers" anymore.
 * ------------------------------------------------------------------------ */

static irqreturn_t griffin_duart_isr(int irq, void *dev_id)
{
	struct griffin_duart *d = dev_id;
	u8 isr = readb(d->base + XR_ISR);
	irqreturn_t ret = IRQ_NONE;

	if (isr & XR_ISR_CTR_READY) {
		readb(d->base + XR_STOPCC);	/* ack: Timer mode only clears IRQ status */
		d->clkevt.event_handler(&d->clkevt);
		ret = IRQ_HANDLED;
	}

	if (d->chan[0].active) {
		if (isr & XR_ISR_RXRDYA) {
			griffin_uart_rx_chars(d, &d->chan[0]);
			ret = IRQ_HANDLED;
		}
		if (isr & XR_ISR_TXRDYA) {
			griffin_uart_tx_chars(d, &d->chan[0]);
			ret = IRQ_HANDLED;
		}
	}

	if (d->chan[1].active) {
		if (isr & XR_ISR_RXRDYB) {
			griffin_uart_rx_chars(d, &d->chan[1]);
			ret = IRQ_HANDLED;
		}
		if (isr & XR_ISR_TXRDYB) {
			griffin_uart_tx_chars(d, &d->chan[1]);
			ret = IRQ_HANDLED;
		}
	}

	return ret;
}

/* ------------------------------------------------------------------------
 * Clockevent (early half).
 * ------------------------------------------------------------------------ */

static int griffin_clkevt_set_state_periodic(struct clock_event_device *evt)
{
	struct griffin_duart *d = container_of(evt, struct griffin_duart, clkevt);

	griffin_imr_update(d, XR_IMR_CTR_READY, 0);
	return 0;
}

static int griffin_clkevt_set_state_shutdown(struct clock_event_device *evt)
{
	struct griffin_duart *d = container_of(evt, struct griffin_duart, clkevt);

	griffin_imr_update(d, 0, XR_IMR_CTR_READY);
	return 0;
}

static int __init griffin_duart_timer_init(struct device_node *np)
{
	struct griffin_duart *d = &gd;
	u32 duart_clock = DUART_CLOCK_DEFAULT;
	u32 preload;
	int irq, ret;

	d->base = of_iomap(np, 0);
	if (!d->base)
		return -ENOMEM;

	spin_lock_init(&d->imr_lock);
	of_property_read_u32(np, "clock-frequency", &duart_clock);

	irq = irq_of_parse_and_map(np, 0);
	if (!irq)
		return -EINVAL;

	writeb(0x00, d->base + XR_IMR);	/* start fully masked */

	/*
	 * Timer mode fires once per full square-wave period = 2*preload input
	 * cycles (matches the 68681 family and the Griffin emulator's model),
	 * so preload = duart_clock / (2*HZ) gives an exact HZ-rate tick with no
	 * rounding error for Griffin's 3.6864 MHz / 100 Hz combination
	 * (preload = 18432 = 0x4800).
	 */
	preload = duart_clock / (2 * HZ);
	if (preload == 0 || preload > 0x10000) {
		pr_err("%pOF: HZ=%u not representable with a 16-bit preload at %u Hz\n",
		       np, HZ, duart_clock);
		return -EINVAL;
	}
	writeb((preload >> 8) & 0xFF, d->base + XR_CTUR);
	writeb(preload & 0xFF, d->base + XR_CTLR);

	/* ACR[7] (BRG_SET) left 0 to match the earlycon/serial baud setup done
	 * earlier in boot by u-boot; only bits 6:4 (C/T mode) are ours to set. */
	writeb(XR_ACR_CT_MODE_TIMER_X1_CLK, d->base + XR_ACR);
	readb(d->base + XR_STARTCC);	/* start the free-running counter */

	d->clkevt.name = "griffin_duart_timer";
	d->clkevt.features = CLOCK_EVT_FEAT_PERIODIC;
	d->clkevt.set_state_periodic = griffin_clkevt_set_state_periodic;
	d->clkevt.set_state_shutdown = griffin_clkevt_set_state_shutdown;
	d->clkevt.rating = 200;
	d->clkevt.cpumask = cpu_possible_mask;
	d->clkevt.irq = irq;

	ret = request_irq(irq, griffin_duart_isr, IRQF_TIMER, "griffin-duart", d);
	if (ret)
		return ret;

	clockevents_config_and_register(&d->clkevt, HZ, 1, 1);

	return 0;
}
TIMER_OF_DECLARE(griffin_duart, "griffin,duart-xr68c681", griffin_duart_timer_init);

/* ------------------------------------------------------------------------
 * uart_ops (late half).
 * ------------------------------------------------------------------------ */

static unsigned int griffin_uart_tx_empty(struct uart_port *port)
{
	struct griffin_uart_chan *c = to_chan(port);

	return (readb(gd.base + c->sr) & XR_SRA_TXEMT) ? TIOCSER_TEMT : 0;
}

static void griffin_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/* No modem control lines wired on Griffin's DUART channels. */
}

static unsigned int griffin_uart_get_mctrl(struct uart_port *port)
{
	return TIOCM_CAR | TIOCM_DSR | TIOCM_CTS;
}

static void griffin_uart_start_tx(struct uart_port *port)
{
	griffin_imr_update(&gd, to_chan(port)->imr_tx, 0);
}

static void griffin_uart_stop_tx(struct uart_port *port)
{
	griffin_imr_update(&gd, 0, to_chan(port)->imr_tx);
}

static void griffin_uart_stop_rx(struct uart_port *port)
{
	griffin_imr_update(&gd, 0, to_chan(port)->imr_rx);
}

static void griffin_uart_break_ctl(struct uart_port *port, int ctl)
{
	struct griffin_uart_chan *c = to_chan(port);

	writeb(ctl ? 0x60 : 0x70, gd.base + c->cr);	/* MC=6 start / MC=7 stop break */
}

static int griffin_uart_startup(struct uart_port *port)
{
	struct griffin_uart_chan *c = to_chan(port);
	unsigned long flags;

	/*
	 * The IRQ is already live (requested during the early clockevent init,
	 * for the chip's whole lifetime) -- no request_irq() here.
	 */
	spin_lock_irqsave(&port->lock, flags);
	writeb(0x05, gd.base + c->cr);	/* EC=1,TC=1: enable RX+TX */
	c->active = true;
	spin_unlock_irqrestore(&port->lock, flags);

	griffin_imr_update(&gd, c->imr_rx, 0);
	return 0;
}

static void griffin_uart_shutdown(struct uart_port *port)
{
	struct griffin_uart_chan *c = to_chan(port);
	unsigned long flags;

	griffin_imr_update(&gd, 0, c->imr_rx | c->imr_tx);

	spin_lock_irqsave(&port->lock, flags);
	c->active = false;
	writeb(0x0A, gd.base + c->cr);	/* EC=2,TC=2: disable RX+TX */
	spin_unlock_irqrestore(&port->lock, flags);
}

static void griffin_uart_set_termios(struct uart_port *port, struct ktermios *termios,
				     const struct ktermios *old)
{
	unsigned long flags;

	/*
	 * Hardware is fixed at 115200 8N1, configured earlier in boot (u-boot's
	 * serial_xr68c681 driver) and never reprogrammed here (MR1A/CSRA/ACR
	 * are left alone) -- there is no dynamic baud/format reconfiguration.
	 * Just keep termios bookkeeping consistent so upper layers (timeouts,
	 * TIOCGSERIAL) behave.
	 */
	tty_termios_encode_baud_rate(termios, 115200, 115200);

	spin_lock_irqsave(&port->lock, flags);
	uart_update_timeout(port, termios->c_cflag, 115200);
	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *griffin_uart_type(struct uart_port *port)
{
	return "griffin_duart";
}

static void griffin_uart_release_port(struct uart_port *port)
{
}

static int griffin_uart_request_port(struct uart_port *port)
{
	return 0;
}

static void griffin_uart_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_GRIFFIN;
}

static int griffin_uart_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_GRIFFIN)
		return -EINVAL;
	return 0;
}

static const struct uart_ops griffin_uart_ops = {
	.tx_empty	= griffin_uart_tx_empty,
	.set_mctrl	= griffin_uart_set_mctrl,
	.get_mctrl	= griffin_uart_get_mctrl,
	.stop_tx	= griffin_uart_stop_tx,
	.start_tx	= griffin_uart_start_tx,
	.stop_rx	= griffin_uart_stop_rx,
	.break_ctl	= griffin_uart_break_ctl,
	.startup	= griffin_uart_startup,
	.shutdown	= griffin_uart_shutdown,
	.set_termios	= griffin_uart_set_termios,
	.type		= griffin_uart_type,
	.release_port	= griffin_uart_release_port,
	.request_port	= griffin_uart_request_port,
	.config_port	= griffin_uart_config_port,
	.verify_port	= griffin_uart_verify_port,
};

/* ------------------------------------------------------------------------
 * Console (real, post-tty_init(); hands off from earlycon automatically).
 * ------------------------------------------------------------------------ */

static void griffin_console_putchar(struct uart_port *port, unsigned char ch)
{
	/* The console is strictly channel A. */
	while (!(readb(gd.base + XR_SRA) & XR_SRA_TXRDY))
		cpu_relax();
	writeb(ch, gd.base + XR_TBA);
}

static void griffin_console_write(struct console *co, const char *s, unsigned int count)
{
	uart_console_write(&gd.chan[0].port, s, count, griffin_console_putchar);
}

static int griffin_console_setup(struct console *co, char *options)
{
	if (!gd.base)
		return -ENODEV;
	return 0;
}

static struct uart_driver griffin_uart_driver;

static struct console griffin_console = {
	.name	= "ttyS",
	.write	= griffin_console_write,
	.device	= uart_console_device,
	.setup	= griffin_console_setup,
	.flags	= CON_PRINTBUFFER,
	.index	= -1,
	.data	= &griffin_uart_driver,
};

static struct uart_driver griffin_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= "griffin_duart",
	.dev_name	= "ttyS",
	.major		= TTY_MAJOR,
	.minor		= 64,
	.nr		= 2,	/* ttyS0 = channel A (console), ttyS1 = channel B */
	.cons		= &griffin_console,
};

/* ------------------------------------------------------------------------
 * platform_driver (late half): registers the tty port itself.
 * ------------------------------------------------------------------------ */

/* One-time channel B hardware init.  Neither the ROM nor u-boot ever
 * programs channel B (their serial code is A-only), so set up 115200 8N1
 * here, mirroring u-boot's channel A recipe.  The BRG-extend bits are
 * per-channel commands via CRB; ACR is shared with the timer and must not
 * be touched.  RX/TX enable happens in startup() when the port opens. */
static void griffin_uart_init_chan_b(struct griffin_duart *d)
{
	writeb(0x30, d->base + XR_CRB);	/* reset transmitter */
	writeb(0x20, d->base + XR_CRB);	/* reset receiver */
	writeb(0x10, d->base + XR_CRB);	/* reset MR pointer */
	writeb(0x13, d->base + XR_MR1B);	/* MR1B: 8 bits, no parity */
	writeb(0x07, d->base + XR_MR1B);	/* MR2B: 1 stop bit */
	writeb(0x80, d->base + XR_CRB);	/* set RX BRG extend */
	writeb(0xA0, d->base + XR_CRB);	/* set TX BRG extend */
	writeb(0x88, d->base + XR_CSRB);	/* 115200 with extend */
}

static int griffin_uart_probe(struct platform_device *pdev)
{
	struct griffin_duart *d = &gd;
	struct resource *res;
	int ret, i;

	if (!d->base) {
		dev_err(&pdev->dev, "early clockevent init did not run\n");
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	d->chan[0].sr = XR_SRA;
	d->chan[0].cr = XR_CRA;
	d->chan[0].rb = XR_RBA;
	d->chan[0].tb = XR_TBA;
	d->chan[0].imr_rx = XR_IMR_RXRDYA;
	d->chan[0].imr_tx = XR_IMR_TXRDYA;

	d->chan[1].sr = XR_SRB;
	d->chan[1].cr = XR_CRB;
	d->chan[1].rb = XR_RBB;
	d->chan[1].tb = XR_TBB;
	d->chan[1].imr_rx = XR_IMR_RXRDYB;
	d->chan[1].imr_tx = XR_IMR_TXRDYB;

	griffin_uart_init_chan_b(d);

	ret = uart_register_driver(&griffin_uart_driver);
	if (ret)
		return ret;

	for (i = 0; i < 2; i++) {
		struct uart_port *port = &d->chan[i].port;

		port->membase = d->base;
		port->mapbase = res->start;
		port->iotype = UPIO_MEM;
		port->irq = d->clkevt.irq;
		port->uartclk = 115200 * 16;	/* informational only -- baud is fixed */
		port->fifosize = 1;
		port->ops = &griffin_uart_ops;
		port->flags = UPF_BOOT_AUTOCONF;
		port->line = i;
		port->type = PORT_GRIFFIN;
		port->dev = &pdev->dev;

		ret = uart_add_one_port(&griffin_uart_driver, port);
		if (ret) {
			while (--i >= 0)
				uart_remove_one_port(&griffin_uart_driver,
						     &d->chan[i].port);
			uart_unregister_driver(&griffin_uart_driver);
			return ret;
		}
	}

	platform_set_drvdata(pdev, d);
	return 0;
}

static void griffin_uart_remove(struct platform_device *pdev)
{
	struct griffin_duart *d = platform_get_drvdata(pdev);

	uart_remove_one_port(&griffin_uart_driver, &d->chan[1].port);
	uart_remove_one_port(&griffin_uart_driver, &d->chan[0].port);
	uart_unregister_driver(&griffin_uart_driver);
}

static const struct of_device_id griffin_uart_of_match[] = {
	{ .compatible = "griffin,duart-xr68c681" },
	{ }
};
MODULE_DEVICE_TABLE(of, griffin_uart_of_match);

static struct platform_driver griffin_uart_platform_driver = {
	.probe	= griffin_uart_probe,
	.remove	= griffin_uart_remove,
	.driver	= {
		.name		= "griffin-duart",
		.of_match_table	= griffin_uart_of_match,
	},
};
module_platform_driver(griffin_uart_platform_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Griffin XR68C681 DUART: earlycon, tty console, clockevent");
