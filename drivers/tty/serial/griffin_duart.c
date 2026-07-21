// SPDX-License-Identifier: GPL-2.0
/*
 * Griffin XR68C681 DUART serial support.
 *
 * Earlycon for the Griffin homebrew 68010 machine: channel A of the XR68C681
 * at 0xF80000, byte registers on odd addresses.  u-boot's serial driver has
 * already configured 115200 8N1, so earlycon only needs polled TX: wait for
 * SRA.TXRDY (bit 2), write TBA.  The full uart_driver/tty (interrupt RX on
 * autovector level 5) follows in the bring-up plan's M5.
 */

#include <linux/console.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/serial_core.h>

#define XR_SRA		0x03	/* r: channel A status register */
#define XR_TBA		0x07	/* w: channel A transmit buffer */

#define XR_SRA_TXRDY	0x04

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
