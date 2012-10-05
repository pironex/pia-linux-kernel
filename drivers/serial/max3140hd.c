/*
 *
 *  Copyright (C) 2008 Christian Pellegrin <chripell@evolware.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 *
 * Notes: the MAX3100 doesn't provide an interrupt on CTS so we have
 * to use polling for flow control. TX empty IRQ is unusable, since
 * writing conf clears FIFO buffer and we cannot have this interrupt
 * always asking us for attention.
 *
 * Example platform data:

 static struct plat_max3100 max3100_plat_data = {
 .loopback = 0,
 .crystal = 0,
 .poll_time = 100,
 };

 static struct spi_board_info spi_board_info[] = {
 {
 .modalias	= "max3100",
 .platform_data	= &max3100_plat_data,
 .irq		= IRQ_EINT12,
 .max_speed_hz	= 5*1000*1000,
 .chip_select	= 0,
 },
 };

 * The initial minor number is 209 in the low-density serial port:
 * mknod /dev/ttyMAX0 c 204 209
 */

#define MAX3100_MAJOR 204
#define MAX3100_MINOR 209
/* 4 MAX3100s should be enough for everyone */
#define MAX_MAX3100 4

#define THREADED
#define DEBUG
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/spi/spi.h>
#include <linux/freezer.h>
#include <linux/string.h>

#include <linux/kthread.h>
#include <linux/serial_max3140hd.h>

#define MAX3100_C    (1<<14)
#define MAX3100_D    (0<<14)
#define MAX3100_W    (1<<15)
#define MAX3100_RX   (0<<15)

#define MAX3100_WC   (MAX3100_W  | MAX3100_C)
#define MAX3100_RC   (MAX3100_RX | MAX3100_C)
#define MAX3100_WD   (MAX3100_W  | MAX3100_D)
#define MAX3100_RD   (MAX3100_RX | MAX3100_D)
#define MAX3100_CMD  (3 << 14)

#define MAX3100_T    (1<<14)
#define MAX3100_R    (1<<15)

#define MAX3100_FEN  (1<<13)
#define MAX3100_SHDN (1<<12)
#define MAX3100_TM   (1<<11)
#define MAX3100_RM   (1<<10)
#define MAX3100_PM   (1<<9)
#define MAX3100_RAM  (1<<8)
#define MAX3100_IR   (1<<7)
#define MAX3100_ST   (1<<6)
#define MAX3100_PE   (1<<5)
#define MAX3100_L    (1<<4)
#define MAX3100_BAUD (0xf)

#define MAX3100_TE   (1<<10)
#define MAX3100_RAFE (1<<10)
#define MAX3100_RTS  (1<<9)
#define MAX3100_CTS  (1<<9)
#define MAX3100_PT   (1<<8)
#define MAX3100_DATA (0xff)

#define MAX3100_RT   (MAX3100_R | MAX3100_T)
#define MAX3100_RTC  (MAX3100_RT | MAX3100_CTS | MAX3100_RAFE)

/* the following simulate a status reg for ignore_status_mask */
#define MAX3100_STATUS_PE 1
#define MAX3100_STATUS_FE 2
#define MAX3100_STATUS_OE 4

#define MAX3100_RX_FIFOLEN 8
#define MAX3100_TX_FIFOLEN 2
#define MAX3140_WORDSIZE   2 /* size in bytes per spi word */


struct max3100_port {
	struct uart_port port;
	struct spi_device *spi;
	const char *name;
	spinlock_t spi_lock;
	u16 spi_rxbuf[MAX3100_RX_FIFOLEN];
	u16 spi_txbuf[MAX3100_RX_FIFOLEN];
	struct spi_message *spi_msg;
	struct spi_transfer *spi_tran;
	struct spi_message *spi_rts_msg;
	//struct spi_message *spi_rts_tran;

	int cts;	        /* last CTS received for flow ctrl */
	int tx_empty;		/* last TX empty bit */

	spinlock_t conf_lock;	/* shared data */
	int conf_commit;	/* need to make changes */
	int conf;		/* configuration for the MAX31000
				 * (bits 0-7, bits 8-11 are irqs) */
	int rts_commit;	        /* need to change rts */
	int rts;		/* rts status */
	int baud;		/* current baud rate */

	int parity;		/* keeps track if we should send parity */
#define MAX3100_PARITY_ON 1
#define MAX3100_PARITY_ODD 2
#define MAX3100_7BIT 4
	int rx_enabled;	        /* if we should rx chars */
	int rx_count;           /* track number of chars in rx flip buffer */

	int irq;		/* irq assigned to the max3100 */

	int minor;		/* minor number */
	int crystal;		/* 1 if 3.6864Mhz crystal 0 for 1.8432 */
	int loopback;		/* 1 if we are in loopback mode */
	int invert_rts;		/* 1 if RTS output logic is inverted */
	s64 rts_sleep;		/* wait time for rts release */
	struct timespec prev_ts;
	struct timespec now_ts;

	/* for handling irqs: need workqueue since we do spi_sync */
	struct workqueue_struct *workqueue;
	struct work_struct work;
	/* set to 1 to make the workhandler exit as soon as possible */
	int  force_end_work;
	/* need to know we are suspending to avoid deadlock on workqueue */
	int suspending;

	/* hook for suspending MAX3100 via dedicated pin */
	void (*max3100_hw_suspend) (int suspend);

	/* poll time (in ms) for ctrl lines */
	int poll_time;
	/* and its timer */
	struct timer_list	timer;
	struct hrtimer rts_timer;
	struct timespec ts1;
	struct timespec ts2;
	struct timespec ts3;
	int icnt;

#ifdef THREADED
#define BIT_DRIVER_DISABLE	1
#define BIT_IRQ_PENDING		2
#define BIT_RX_PENDING		3
#define BIT_TX_STARTED		4
#define BIT_CONF_COMMIT		5
	wait_queue_head_t wq;
	struct task_struct *main_thread;
	struct task_struct *read_thread;
	struct mutex thread_mutex;
	unsigned long flags;
#endif
};

static struct max3100_port *max3100s[MAX_MAX3100]; /* the chips */
static DEFINE_MUTEX(max3100s_lock);		   /* race on probe */

static inline int max3100_do_parity(struct max3100_port *s, u16 c)
{
	int parity;

	if (s->parity & MAX3100_PARITY_ODD)
		parity = 1;
	else
		parity = 0;

	if (s->parity & MAX3100_7BIT)
		c &= 0x7f;
	else
		c &= 0xff;

	parity = parity ^ (hweight8(c) & 1);
	return parity;
}

static inline int max3100_check_parity(struct max3100_port *s, u16 c)
{
	return max3100_do_parity(s, c) == ((c >> 8) & 1);
}

static inline void max3100_calc_parity(struct max3100_port *s, u16 *c)
{
	if (s->parity & MAX3100_7BIT)
		*c &= 0x7f;
	else
		*c &= 0xff;

	if (s->parity & MAX3100_PARITY_ON)
		*c |= max3100_do_parity(s, *c) << 8;
}

static void max3100_timeout(unsigned long data)
{
	struct max3100_port *s = (struct max3100_port *)data;
	int timer_state = -1;
	if (s->port.state) {
		if (!test_and_set_bit(BIT_IRQ_PENDING, &s->flags));
			wake_up_process(s->main_thread);
		timer_state = mod_timer(&s->timer, jiffies + s->poll_time);
		dev_dbg(&s->spi->dev, "poll timeout %lu\n", jiffies + s->poll_time);
	}
}

static int max3140_sr(struct max3100_port *s, const void *txbuf, void *rxbuf,
                      unsigned len)
{
	struct spi_device *spi = s->spi;
	struct spi_message	message;
	struct spi_transfer	x[len];
	int ret, i;
	u16 *rx;

	spi_message_init(&message);
	memset(&x, 0, (sizeof(x[0]) * len));
	for (i = 0; i < len; ++i) {
		x[i].len = MAX3140_WORDSIZE;
		x[i].tx_buf = txbuf;
		x[i].rx_buf = rxbuf;
		x[i].speed_hz = spi->max_speed_hz;
		x[i].cs_change = 1;
		spi_message_add_tail(&x[i], &message);
	}

	ret = spi_sync(spi, &message);
	rx = x[i].rx_buf;
	s->tx_empty = (*rx & MAX3100_T);

	return ret;
}

static int max3140_sr1(struct max3100_port *s, const void *txbuf, void *rxbuf)
{
	struct spi_device *spi = s->spi;
	struct spi_message	message;
	struct spi_transfer	x;
	u16 *rx = rxbuf;
	int ret;

	spi_message_init(&message);
	memset(&x, 0, sizeof x);
	x.len = MAX3140_WORDSIZE;
	x.tx_buf = txbuf;
	x.rx_buf = rxbuf;
	x.speed_hz = spi->max_speed_hz;
	//x.cs_change = 1;
	spi_message_add_tail(&x, &message);

	ret = spi_sync(spi, &message);

	s->tx_empty = ((*rx & MAX3100_T) > 0);
	if (*rx & MAX3100_R)
		set_bit(BIT_RX_PENDING, &s->flags);
	else
		clear_bit(BIT_RX_PENDING, &s->flags);

	return ret;
}

static void max3140_receive_chars(struct max3100_port *s, unsigned char *str, int len)
{
	struct uart_port *port = &s->port;
	struct tty_struct *tty;
	int usable;

	/* If uart is not opened, just return */
	if (!port->state)
		return;

	tty = port->state->port.tty;
	if (!tty)
		return;

	while (len) {
		usable = tty_buffer_request_room(tty, len);
		if (usable) {
			tty_insert_flip_string(tty, str, usable);
			str += usable;
			port->icount.rx += usable;
		}
		len -= usable;
	}
	tty_flip_buffer_push(tty);
}

/* send single command to max3140 */
static int max3140_cmd(struct max3100_port *s, u16 tx)
{
	void *buf;
	u16 *obuf, *ibuf;
	u16 rx;
	u8 ch;
	int ret;

	buf = kzalloc(8, GFP_KERNEL | GFP_DMA);
	if (!buf)
		return -ENOMEM;

	obuf = buf;
	ibuf = buf + 4;
	//*obuf = cpu_to_be16(tx);
	*obuf = tx;
	ret = max3140_sr1(s, obuf, ibuf);
	dev_dbg(&s->spi->dev, "%s: t0x%04x o0x%04x i0x%04x\n", __func__, tx, *obuf, *ibuf);
	if (ret) {
		dev_dbg(&s->spi->dev, "%s: error %d while sending 0x%x\n",
				__func__, ret, *obuf);
		goto exit;
	}

	//rx = be16_to_cpu(*ibuf);
	rx = *ibuf;

	/* If some valid data is read back */
	if (test_bit(BIT_RX_PENDING, &s->flags)) {
		ch = rx & 0xff;
		max3140_receive_chars(s, &ch, 1);
	}

//	if (rx & MAX3100_R) {
//		ch = rx & 0xff;
//		max3140_receive_chars(s, &ch, 1);
//	}

exit:
	kfree(buf);
	return ret;
}

#define MAX3100_SETRTS(r) \
	(r ? (s->invert_rts ? 0 : MAX3100_RTS) : (s->invert_rts ? MAX3100_RTS : 0))
static enum hrtimer_restart rts_timer_handler(struct hrtimer *handle)
{
	struct max3100_port *s =
			container_of(handle, struct max3100_port, rts_timer);

	if (!test_and_set_bit(BIT_DRIVER_DISABLE, &s->flags))
		wake_up_process(s->main_thread);

	dev_dbg(&s->spi->dev, "RTS timeout done\n");

	return HRTIMER_NORESTART;
}

static int max3140_read_fifo(struct max3100_port *s)
{
	int i, j = 0;
	//u16 *obuf;
	u16 *ibuf;
	u16 cur;
	u8 str[MAX3100_RX_FIFOLEN];
	int len = MAX3100_RX_FIFOLEN * sizeof (u16);

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	memset(s->spi_txbuf, 0, len);
	memset(s->spi_rxbuf, 0, len);

	ibuf = s->spi_rxbuf;
	for (i = 0; i < MAX3100_RX_FIFOLEN; ++i) {
		if (max3140_sr1(s, &s->spi_txbuf[i], &s->spi_rxbuf[i]/*, 2 len*/)) {
			return 0;
		}
		//cur = be16_to_cpu(*ibuf);
		cur = *ibuf;
		if (test_and_clear_bit(BIT_RX_PENDING, &s->flags)) {
			str[j++] = cur & 0xff;
		} else {
			break;
		}

		ibuf++;
	}

	if (j) {
		max3140_receive_chars(s, str, j);
		/* keep RX_PENDING flag, when FIFO was full,
		 * otherwise IRQ line is never deasserted */
		if (j == MAX3100_RX_FIFOLEN)
			set_bit(BIT_RX_PENDING, &s->flags);
	}
	dev_dbg(&s->spi->dev, "%s cnt: %d, tx_empty %d\n", __func__, j, s->tx_empty);

	return j;
}
static int max3140_send_and_receive(struct max3100_port *s)
{
	struct timespec now, start, end;
	int rxchars, rxlen;
	u16 tx, rx;
	u8 crx;
	struct circ_buf *xmit = &s->port.state->xmit;
	unsigned long diff_ns;

	dev_dbg(&s->spi->dev, "%s\n", __func__);
	rxchars = 0;
	rxlen = 0;
	getrawmonotonic(&start);

	do {
		while (test_bit(BIT_TX_STARTED, &s->flags) && s->tx_empty) {
			tx = 0xffff;
			if (s->port.x_char) {
				tx = s->port.x_char;
				s->port.icount.tx++;
				s->port.x_char = 0;
			} else if (!uart_circ_empty(xmit) && !uart_tx_stopped(&s->port)) {
				tx = xmit->buf[xmit->tail];
				xmit->tail = (xmit->tail + 1) &
						(UART_XMIT_SIZE - 1);
				s->port.icount.tx++;
			} else {
				clear_bit(BIT_TX_STARTED, &s->flags);
			}
			if (tx != 0xffff) {
				max3100_calc_parity(s, &tx);
				// force driver on while sending
				tx |= MAX3100_WD | MAX3100_SETRTS(0);
				///tx = cpu_to_be16(tx);
				max3140_sr1(s, &tx, &rx);
				///rx = be16_to_cpu(rx);

				s->prev_ts.tv_nsec = s->now_ts.tv_nsec;
				s->prev_ts.tv_sec = s->now_ts.tv_sec;
				getrawmonotonic(&now);

				// prev_ts not finished
				if (timespec_compare(&s->prev_ts, &now) > 0) {
					// prev byte not yet complete
					// now becomes prev_ts as base of last byte delay
					s->now_ts.tv_nsec = s->prev_ts.tv_nsec;
					s->now_ts.tv_sec = s->prev_ts.tv_sec;
				} else {
					s->now_ts.tv_nsec = now.tv_nsec;
					s->now_ts.tv_sec = now.tv_sec;
				}
				// 1 byte delay + rest of previous byte
				timespec_add_ns(&s->now_ts, s->rts_sleep);

				/* unlikely to receive something here */
				if (test_and_clear_bit(BIT_RX_PENDING, &s->flags)) {
					crx = rx & 0xff;
					//rxchars++;
					max3140_receive_chars(s, &crx, 1);
				}
				// wait until all data sent
				if (uart_circ_empty(xmit) || uart_tx_stopped(&s->port)) {
					diff_ns = timespec_to_ns(&s->now_ts) -
							timespec_to_ns(&now);
					//ndelay(diff_ns);
					hrtimer_start(&s->rts_timer, ktime_set(0, diff_ns),
					              HRTIMER_MODE_REL);
					dev_dbg(&s->spi->dev, "ns_old:%lu\n", diff_ns);
				}
			}
		}
		rxchars += max3140_read_fifo(s);


		if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
			uart_write_wakeup(&s->port);

		if (rxchars > 16 && s->port.state->port.tty != NULL) {
			tty_flip_buffer_push(s->port.state->port.tty);
			rxchars = 0;
		}

	} while (!s->force_end_work &&
		 !freezing(current) &&
		 ((test_and_clear_bit(BIT_RX_PENDING, &s->flags)) ||
		  (s->tx_empty && (s->flags & (1 << BIT_TX_STARTED)) &&
		   !uart_tx_stopped(&s->port))));

	if (rxchars > 0 && s->port.state->port.tty != NULL) {
		tty_flip_buffer_push(s->port.state->port.tty);
	}
	getrawmonotonic(&end);
	dev_dbg(&s->spi->dev, "%s %ld.%ld - %ld.%ld\n", __func__,
	        start.tv_sec, start.tv_nsec,
	        end.tv_sec, end.tv_nsec);

	return 0;
}
static void max3140_driver_disable(struct max3100_port *s, u8 dis)
{
	u16 cmd = MAX3100_WD | MAX3100_TE | MAX3100_SETRTS(dis);
	max3140_cmd(s, cmd);
	dev_dbg(&s->spi->dev, "%s\n", __func__);
}
static int max3140_main_thread(void *_max)
{
	struct max3100_port *s = _max;
	wait_queue_head_t *wq = &s->wq;
	int ret = 0;
	//struct circ_buf *xmit = &s->con_xmit;

	init_waitqueue_head(wq);
	dev_dbg(&s->spi->dev, "%s\n", __func__);

	do {
		wait_event_interruptible(*wq, s->flags || kthread_should_stop());

		mutex_lock(&s->thread_mutex);

		if (test_and_clear_bit(BIT_CONF_COMMIT, &s->flags)) {
			max3140_cmd(s, MAX3100_WC | s->conf);
		}
		//if (test_and_clear_bit(BIT_TX_STARTED, &s->flags)) {
		if ((s->flags & (1 << BIT_TX_STARTED)) > 0) {
			dev_dbg(&s->spi->dev, "tx\n");
			max3140_send_and_receive(s);
		}
		if (test_and_clear_bit(BIT_IRQ_PENDING, &s->flags)) {
			dev_dbg(&s->spi->dev, "irq\n");
			max3140_send_and_receive(s);
		}
		if (test_and_clear_bit(BIT_DRIVER_DISABLE, &s->flags)) {
			dev_dbg(&s->spi->dev, "dd\n");
			max3140_driver_disable(s, 1);
		}

		mutex_unlock(&s->thread_mutex);
		dev_dbg(&s->spi->dev, "%s loop done\n", __func__);

	} while (!kthread_should_stop());

	return ret;
}

static irqreturn_t max3140_irq(int irq, void *dev_id)
{
	struct max3100_port *s = dev_id;

	/* max3140's irq is a falling edge, not level triggered,
	 * so no need to disable the irq */
	if (!test_and_set_bit(BIT_IRQ_PENDING, &s->flags))
		wake_up_process(s->main_thread);

	return IRQ_HANDLED;
}

static void max3100_enable_ms(struct uart_port *port)
{
	struct max3100_port *s = container_of(port,
					      struct max3100_port,
					      port);

	if (s->poll_time > 0)
		mod_timer(&s->timer, jiffies);
	dev_dbg(&s->spi->dev, "%s %d\n", __func__, s->poll_time);
}

static void max3100_start_tx(struct uart_port *port)
{
	struct max3100_port *s = container_of(port,
					      struct max3100_port,
					      port);

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	if (!test_and_set_bit(BIT_TX_STARTED, &s->flags))
		wake_up_process(s->main_thread);
}

static void max3100_stop_rx(struct uart_port *port)
{
	struct max3100_port *s = container_of(port,
					      struct max3100_port,
					      port);

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	s->rx_enabled = 0;
	s->conf &= ~MAX3100_RM;
	if (!test_and_set_bit(BIT_CONF_COMMIT, &s->flags))
		wake_up_process(s->main_thread);
}

static unsigned int max3100_tx_empty(struct uart_port *port)
{
	struct max3100_port *s = container_of(port,
					      struct max3100_port,
					      port);

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	/* may not be truly up-to-date */
	//FIXME remove max3100_dowork(s);
	return s->tx_empty;
}

static unsigned int max3100_get_mctrl(struct uart_port *port)
{
	struct max3100_port *s = container_of(port,
					      struct max3100_port,
					      port);

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	/* may not be truly up-to-date */
	// FIXME remove max3100_dowork(s);
	/* always assert DCD and DSR since these lines are not wired */
	// FIXME check if i need to evaluate cts
	return (s->cts ? TIOCM_CTS : 0) | TIOCM_DSR | TIOCM_CAR;
}

static void max3100_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct max3100_port *s = container_of(port,
					      struct max3100_port,
					      port);
	int rts;

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	rts = (mctrl & TIOCM_RTS) > 0;

	if (s->rts != rts) {
		dev_dbg(&s->spi->dev, "%s crts %u\n", __func__, mctrl);
		s->rts = rts;
		s->rts_commit = 1;
		//FIXME mctrl rts_commit max3100_dowork(s);
	}
}

static void
max3100_set_termios(struct uart_port *port, struct ktermios *termios,
		    struct ktermios *old)
{
	struct max3100_port *s = container_of(port,
					      struct max3100_port,
					      port);
	int baud = 0;
	unsigned cflag;
	int bits_per_byte = 10;
	u32 param_new, param_mask, parity = 0;

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	cflag = termios->c_cflag;
	param_new = 0;
	param_mask = 0;

	baud = tty_termios_baud_rate(termios);
	param_new = s->conf & MAX3100_BAUD;
	switch (baud) {
	case 300:
		if (s->crystal)
			baud = s->baud;
		else
			param_new = 15;
		break;
	case 600:
		param_new = 14 + s->crystal;
		break;
	case 1200:
		param_new = 13 + s->crystal;
		break;
	case 2400:
		param_new = 12 + s->crystal;
		break;
	case 4800:
		param_new = 11 + s->crystal;
		break;
	case 9600:
		param_new = 10 + s->crystal;
		break;
	case 19200:
		param_new = 9 + s->crystal;
		break;
	case 38400:
		param_new = 8 + s->crystal;
		break;
	case 57600:
		param_new = 1 + s->crystal;
		break;
	case 115200:
		param_new = 0 + s->crystal;
		break;
	case 230400:
		if (s->crystal)
			param_new = 0;
		else
			baud = s->baud;
		break;
	default:
		baud = s->baud;
	}
	tty_termios_encode_baud_rate(termios, baud, baud);
	s->baud = baud;
	param_mask |= MAX3100_BAUD;

	if ((cflag & CSIZE) == CS8) {
		param_new &= ~MAX3100_L;
		parity &= ~MAX3100_7BIT;
	} else {
		param_new |= MAX3100_L;
		parity |= MAX3100_7BIT;
		cflag = (cflag & ~CSIZE) | CS7;
		bits_per_byte--;
	}
	param_mask |= MAX3100_L;

	if (cflag & CSTOPB) {
		param_new |= MAX3100_ST;
		bits_per_byte++;
	} else {
		param_new &= ~MAX3100_ST;
	}
	param_mask |= MAX3100_ST;

	if (cflag & PARENB) {
		param_new |= MAX3100_PE;
		parity |= MAX3100_PARITY_ON;
		bits_per_byte++;
	} else {
		param_new &= ~MAX3100_PE;
		parity &= ~MAX3100_PARITY_ON;
	}
	param_mask |= MAX3100_PE;

	if (cflag & PARODD)
		parity |= MAX3100_PARITY_ODD;
	else
		parity &= ~MAX3100_PARITY_ODD;

	// pause after last tx byte depending on bits per byte
	s->rts_sleep = 1000000*bits_per_byte/baud*1000;
	dev_notice(&s->spi->dev, "Sleep setup for %d bits/byte\n", bits_per_byte);

	/* mask termios capabilities we don't support */
	cflag &= ~CMSPAR;
	termios->c_cflag = cflag;

	s->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		s->port.ignore_status_mask |=
			MAX3100_STATUS_PE | MAX3100_STATUS_FE |
			MAX3100_STATUS_OE;

	/* we are sending char from a workqueue so enable */
	s->port.state->port.tty->low_latency = 1;

	if (s->poll_time > 0)
		del_timer_sync(&s->timer);

	uart_update_timeout(port, termios->c_cflag, baud);

	spin_lock(&s->conf_lock);
	s->conf = (s->conf & ~param_mask) | (param_new & param_mask);
	s->conf_commit = 1;
	s->parity = parity;
	spin_unlock(&s->conf_lock);

	set_bit(BIT_CONF_COMMIT, &s->flags);
	set_bit(BIT_DRIVER_DISABLE, &s->flags);
	wake_up_process(s->main_thread);

	if (UART_ENABLE_MS(&s->port, termios->c_cflag))
		max3100_enable_ms(&s->port);
}

static void max3100_shutdown(struct uart_port *port)
{
	struct max3100_port *s = container_of(port,
					      struct max3100_port,
					      port);
	u16 tx;

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	if (s->suspending)
		return;

	s->force_end_work = 1;

	if (s->poll_time > 0)
		del_timer_sync(&s->timer);
	hrtimer_cancel(&s->rts_timer);

	// TODO remove
	if (s->workqueue) {
		flush_workqueue(s->workqueue);
		destroy_workqueue(s->workqueue);
		s->workqueue = NULL;
	}

	if (s->irq)
		free_irq(s->irq, s);

	/* set shutdown mode to save power */
	tx = (MAX3100_WC | s->conf | MAX3100_SHDN);
	max3140_cmd(s, tx);
	//set_bit(BIT_CONF_COMMIT, &s->flags);
	//wake_up_process(s->main_thread);
}

static int max3100_startup(struct uart_port *port)
{
	struct max3100_port *s = container_of(port,
					      struct max3100_port,
					      port);
	char b[12];

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	s->conf = MAX3100_RM | MAX3100_TM;
	s->baud = s->crystal ? 230400 : 115200;
	s->rx_enabled = 1;

	if (s->suspending)
		return 0;

	s->force_end_work = 0;
	s->parity = 0;
	s->rts = 0;

	sprintf(b, "max3100-%d", s->minor);
	if (request_irq(s->irq, max3140_irq,
			IRQF_TRIGGER_FALLING, dev_name(&s->spi->dev)/*"max3100"*/, s) < 0) {
		dev_warn(&s->spi->dev, "cannot allocate irq %d\n", s->irq);
		s->irq = 0;
		return -EBUSY;
	}

	if (s->loopback) {
		u16 tx = 0x4001;
		max3140_cmd(s, tx);
	}

	if (s->max3100_hw_suspend)
		s->max3100_hw_suspend(0);

	set_bit(BIT_CONF_COMMIT, &s->flags);
	wake_up_process(s->main_thread);
	//max3140_cmd(s, s->conf | MAX3100_WC);
	/* wait for clock to settle */
	msleep(50);

	max3100_enable_ms(&s->port);

	return 0;
}

static const char *max3100_type(struct uart_port *port)
{
	struct max3100_port *s = container_of(port,
					      struct max3100_port,
					      port);

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	return s->port.type == PORT_MAX3100 ? "MAX3100" : NULL;
}

static void max3100_release_port(struct uart_port *port)
{
	struct max3100_port *s = container_of(port,
					      struct max3100_port,
					      port);

	dev_dbg(&s->spi->dev, "%s\n", __func__);
}

static void max3100_config_port(struct uart_port *port, int flags)
{
	struct max3100_port *s = container_of(port,
					      struct max3100_port,
					      port);

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	if (flags & UART_CONFIG_TYPE)
		s->port.type = PORT_MAX3100;
}

static int max3100_verify_port(struct uart_port *port,
			       struct serial_struct *ser)
{
	struct max3100_port *s = container_of(port,
					      struct max3100_port,
					      port);
	int ret = -EINVAL;

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	if (ser->type == PORT_UNKNOWN || ser->type == PORT_MAX3100)
		ret = 0;
	return ret;
}

static void max3100_stop_tx(struct uart_port *port)
{
	struct max3100_port *s = container_of(port,
					      struct max3100_port,
					      port);

	dev_dbg(&s->spi->dev, "%s\n", __func__);
}

static int max3100_request_port(struct uart_port *port)
{
	struct max3100_port *s = container_of(port,
					      struct max3100_port,
					      port);

	dev_dbg(&s->spi->dev, "%s\n", __func__);
	return 0;
}

static void max3100_break_ctl(struct uart_port *port, int break_state)
{
	struct max3100_port *s = container_of(port,
					      struct max3100_port,
					      port);

	dev_dbg(&s->spi->dev, "%s\n", __func__);
}

static struct uart_ops max3100_ops = {
	.tx_empty	= max3100_tx_empty,
	.set_mctrl	= max3100_set_mctrl,
	.get_mctrl	= max3100_get_mctrl,
	.stop_tx        = max3100_stop_tx,
	.start_tx	= max3100_start_tx,
	.stop_rx	= max3100_stop_rx,
	.enable_ms      = max3100_enable_ms,
	.break_ctl      = max3100_break_ctl,
	.startup	= max3100_startup,
	.shutdown	= max3100_shutdown,
	.set_termios	= max3100_set_termios,
	.type		= max3100_type,
	.release_port   = max3100_release_port,
	.request_port   = max3100_request_port,
	.config_port	= max3100_config_port,
	.verify_port	= max3100_verify_port,
};

static struct uart_driver max3100_uart_driver = {
	.owner          = THIS_MODULE,
	.driver_name    = "ttyMAX",
	.dev_name       = "ttyMAX",
	.major          = MAX3100_MAJOR,
	.minor          = MAX3100_MINOR,
	.nr             = MAX_MAX3100,
};
static int uart_driver_registered;

static int __devinit max3100_probe(struct spi_device *spi)
{
	int i, retval;
	struct plat_max3140hd *pdata;
	char name[16];
	u16 tx;
	struct sched_param scheduler_param = { .sched_priority = 60 };

	mutex_lock(&max3100s_lock);

	if (!uart_driver_registered) {
		uart_driver_registered = 1;
		retval = uart_register_driver(&max3100_uart_driver);
		if (retval) {
			printk(KERN_ERR "Couldn't register max3100 uart driver\n");
			mutex_unlock(&max3100s_lock);
			return retval;
		}
	}

	for (i = 0; i < MAX_MAX3100; i++)
		if (!max3100s[i])
			break;
	if (i == MAX_MAX3100) {
		dev_warn(&spi->dev, "too many MAX3100 chips\n");
		mutex_unlock(&max3100s_lock);
		return -ENOMEM;
	}

	max3100s[i] = kzalloc(sizeof(struct max3100_port), GFP_KERNEL);
	if (!max3100s[i]) {
		dev_warn(&spi->dev,
			 "kmalloc for max3100 structure %d failed!\n", i);
		mutex_unlock(&max3100s_lock);
		return -ENOMEM;
	}
	snprintf(name, 16, "max3140-%d", i);
	max3100s[i]->name = kstrndup(name, 16, GFP_KERNEL);

	// setup SPI basics
	spi->bits_per_word = 16;
	// pre-allocate everything because transfers are always
	// single and 2 bytes long
	max3100s[i]->spi = spi;


	max3100s[i]->spi_msg = spi_message_alloc(1, GFP_ATOMIC);
	if (!max3100s[i]->spi_msg) {
		kfree(max3100s[i]);
		return -ENOMEM;
	}
	max3100s[i]->spi_tran = list_first_entry(&max3100s[i]->spi_msg->transfers,
	                           			struct spi_transfer, transfer_list);
	max3100s[i]->spi_tran->tx_buf = &max3100s[i]->spi_txbuf;
	max3100s[i]->spi_tran->rx_buf = &max3100s[i]->spi_rxbuf;
	max3100s[i]->spi_tran->len = 2;
	max3100s[i]->spi_msg->context = max3100s[i];

	mutex_init(&max3100s[i]->thread_mutex);
	max3100s[i]->flags = 0;
	max3100s[i]->main_thread = kthread_run(max3140_main_thread,
					max3100s[i], "max3140_main");
	if (IS_ERR(max3100s[i]->main_thread)) {
		int ret = PTR_ERR(max3100s[i]->main_thread);
		spi_message_free(max3100s[i]->spi_msg);
		return ret;
	}
	if (sched_setscheduler(max3100s[i]->main_thread, SCHED_FIFO,
			&scheduler_param))
		dev_warn(&spi->dev, "Error setting scheduler, using default.\n");

	max3100s[i]->irq = spi->irq;
	spin_lock_init(&max3100s[i]->spi_lock);
	spin_lock_init(&max3100s[i]->conf_lock);
	dev_set_drvdata(&spi->dev, max3100s[i]);
	pdata = spi->dev.platform_data;

	/* driver options */
	max3100s[i]->crystal = pdata->crystal;
	max3100s[i]->loopback = pdata->loopback;
	max3100s[i]->invert_rts = pdata->invert_rts;
	max3100s[i]->minor = i;
	//max3100s[i]->max3100_hw_suspend = pdata->max3100_hw_suspend;

	/* poll timer */
	max3100s[i]->poll_time = pdata->poll_time * HZ / 1000;
	if (pdata->poll_time > 0 && max3100s[i]->poll_time == 0)
		max3100s[i]->poll_time = 1;

	init_timer(&max3100s[i]->timer);
	max3100s[i]->timer.function = max3100_timeout;
	max3100s[i]->timer.data = (unsigned long) max3100s[i];
	hrtimer_init(&max3100s[i]->rts_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	max3100s[i]->rts_timer.function = rts_timer_handler;

	dev_dbg(&spi->dev, "%s: adding port %d\n", __func__, i);

	/* port setup */
	max3100s[i]->port.irq = max3100s[i]->irq;
	max3100s[i]->port.uartclk = max3100s[i]->crystal ? 3686400 : 1843200;
	max3100s[i]->port.fifosize = 2; /* TX "FIFO" is only 2 bytes */
	max3100s[i]->port.ops = &max3100_ops;
	max3100s[i]->port.flags = UPF_SKIP_TEST | UPF_BOOT_AUTOCONF;
	max3100s[i]->port.line = i;
	max3100s[i]->port.type = PORT_MAX3100;
	max3100s[i]->port.dev = &spi->dev;

	// CHECK
	//Give membase a psudo value to pass serial_core's check
	//max->port.membase = (void *)0xff110000;
	retval = uart_add_one_port(&max3100_uart_driver, &max3100s[i]->port);
	if (retval < 0)
		dev_warn(&spi->dev,
			 "uart_add_one_port failed for line %d with error %d\n",
			 i, retval);

	/* set shutdown mode to save power. Will be woken-up on open */
	tx = MAX3100_WC | MAX3100_SHDN;
	max3140_cmd(max3100s[i], tx);
	mutex_unlock(&max3100s_lock);
	return 0;
}

static int __devexit max3100_remove(struct spi_device *spi)
{
	struct max3100_port *s = dev_get_drvdata(&spi->dev);
	int i;

	mutex_lock(&max3100s_lock);

	/* find out the index for the chip we are removing */
	for (i = 0; i < MAX_MAX3100; i++)
		if (max3100s[i] == s)
			break;

	dev_dbg(&spi->dev, "%s: removing port %d\n", __func__, i);
	uart_remove_one_port(&max3100_uart_driver, &max3100s[i]->port);
	kfree(max3100s[i]->name);
	kfree(max3100s[i]);
	max3100s[i] = NULL;

	/* check if this is the last chip we have */
	for (i = 0; i < MAX_MAX3100; i++)
		if (max3100s[i]) {
			mutex_unlock(&max3100s_lock);
			return 0;
		}
	pr_debug("removing max3100 driver\n");
	uart_unregister_driver(&max3100_uart_driver);

	mutex_unlock(&max3100s_lock);
	return 0;
}

#ifdef CONFIG_PM

static int max3140_suspend(struct spi_device *spi, pm_message_t state)
{
	struct max3100_port *s = dev_get_drvdata(&spi->dev);
	u16 tx;

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	disable_irq(s->irq);

	s->suspending = 1;
	uart_suspend_port(&max3100_uart_driver, &s->port);

	/* no HW suspend, so do SW one */
	tx = (s->conf | MAX3100_WC | MAX3100_SHDN);
	max3140_cmd(s, tx);

	return 0;
}

static int max3140_resume(struct spi_device *spi)
{
	struct max3100_port *s = dev_get_drvdata(&spi->dev);

	dev_err(&s->spi->dev, "%s\n", __func__);

	if (s->max3100_hw_suspend)
		s->max3100_hw_suspend(0);
	uart_resume_port(&max3100_uart_driver, &s->port);
	s->suspending = 0;

	enable_irq(s->irq);

	s->conf_commit = 1;
	set_bit(BIT_CONF_COMMIT, &s->flags);
	wake_up_process(s->main_thread);

	return 0;
}

#else
#define max3140_suspend NULL
#define max3140_resume  NULL
#endif

static struct spi_driver max3140_driver = {
	.driver = {
		.name		= "max3140hd",
		.bus		= &spi_bus_type,
		.owner		= THIS_MODULE,
	},

	.probe		= max3100_probe,
	.remove		= __devexit_p(max3100_remove),
	.suspend	= max3140_suspend,
	.resume		= max3140_resume,
};

static int __init max3100_init(void)
{
	return spi_register_driver(&max3140_driver);
}
module_init(max3100_init);

static void __exit max3100_exit(void)
{
	spi_unregister_driver(&max3140_driver);
}
module_exit(max3100_exit);

MODULE_DESCRIPTION("MAX3140 driver");
MODULE_AUTHOR("Bjoern Krombholz <b.krombholz@pironex.de>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:max3140hd");
