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

#ifdef THREADED
#include <linux/kthread.h>
#endif
#include <linux/serial_max3100.h>

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
	spinlock_t spi_lock;
#ifdef THREADED
	u16 spi_rxbuf[MAX3100_RX_FIFOLEN];
	u16 spi_txbuf[MAX3100_RX_FIFOLEN];
#else
	u16 spi_rxbuf;
	u16 spi_txbuf;
#endif
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

#ifdef ORIGINAL
static void max3100_work(struct work_struct *w);
static void max3100_dowork(struct max3100_port *s)
{
	if (!s->force_end_work && !work_pending(&s->work) &&
	    !freezing(current) && !s->suspending) {
		queue_work(s->workqueue, &s->work);
		dev_dbg(&s->spi->dev, "%s - %d\n", __func__, s->icnt);
//		getrawmonotonic(&s->ts1);
//		if (s->icnt > 8) {
//			dev_dbg(&s->spi->dev, "icnt: %d\n", s->icnt);
//		}
		s->icnt = 0;
	} else {
		//dev_dbg(&s->spi->dev, "%s: not queued\n", __func__);
		//getrawmonotonic(&ts2);
		s->icnt++;
	}
}
#else
static inline void max3100_dowork(struct max3100_port *s)
{

}
#endif

static void max3100_timeout(unsigned long data)
{
	struct max3100_port *s = (struct max3100_port *)data;
	int timer_state = -1;
	if (s->port.state) {
		max3100_dowork(s);
		timer_state = mod_timer(&s->timer, jiffies + s->poll_time);
		//dev_dbg(&s->spi->dev, "poll timeout %d\n", jiffies + s->poll_time);
	}
	dev_dbg(&s->spi->dev, "%s - %d\n", __func__, timer_state);
}
#ifndef THREADED
static int max3100_sr(struct max3100_port *s, u16 tx, u16 *rx)
{
	struct spi_message message;
	u16 etx, erx;
	int status;
	struct spi_transfer tran = {
		.tx_buf = &etx,
		.rx_buf = &erx,
		.len = 2,
	};

	etx = cpu_to_be16(tx);
	spi_message_init(&message);
	spi_message_add_tail(&tran, &message);

	status = spi_sync(s->spi, &message);
	dev_dbg(&s->spi->dev, "sr: %04x %04x\n",
		        etx, erx);
	if (status) {
		dev_warn(&s->spi->dev, "error while calling spi_sync\n");
		return -EIO;
	}
	*rx = be16_to_cpu(erx);
	s->tx_empty = (*rx & MAX3100_T) > 0;
	//dev_dbg(&s->spi->dev, "%04x - %04x\n", tx, *rx);
	return 0;
}
#endif

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
	if ((s->flags & (1 << BIT_RX_PENDING)) > 0) {
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

#ifndef THREADED
static int max3100_handlerx(struct max3100_port *s, u16 rx)
{
	unsigned int ch, flg, status = 0;
	int ret = 0, cts;

	if (rx & MAX3100_R && s->rx_enabled) {
		//dev_dbg(&s->spi->dev, "%s\n", __func__);
		ch = rx & (s->parity & MAX3100_7BIT ? 0x7f : 0xff);
		if (rx & MAX3100_RAFE) {
			s->port.icount.frame++;
			flg = TTY_FRAME;
			status |= MAX3100_STATUS_FE;
		} else {
			if (s->parity & MAX3100_PARITY_ON) {
				if (max3100_check_parity(s, rx)) {
					s->port.icount.rx++;
					flg = TTY_NORMAL;
				} else {
					s->port.icount.parity++;
					flg = TTY_PARITY;
					status |= MAX3100_STATUS_PE;
				}
			} else {
				s->port.icount.rx++;
				flg = TTY_NORMAL;
			}
		}
		uart_insert_char(&s->port, status, MAX3100_STATUS_OE, ch, flg);
		ret = 1;
	}

	cts = (rx & MAX3100_CTS) > 0;
	if (s->cts != cts) {
		s->cts = cts;
		uart_handle_cts_change(&s->port, cts ? TIOCM_CTS : 0);
	}
	s->tx_empty = (rx & MAX3100_T) > 0;

	return ret;
}
#endif /* THREADED */

#ifdef ASYNC
static void cb_rts_complete(void *context)
{
	struct max3100_port *s = context;
	u16 *erx;
	const u16 *etx;
	u16 rx;
	struct spi_message *msg = s->spi_rts_msg;
	struct spi_transfer *tran = list_first_entry(&s->spi_rts_msg->transfers,
				struct spi_transfer, transfer_list);
	erx = tran->rx_buf;
	etx = tran->tx_buf;
	dev_dbg(&s->spi->dev, "RTS off: %d, %04x %04x\n", msg->status,
	        *etx, *erx);
	//spi_transfer_del(&s->spi_tran);
	//spi_message_free(s->spi_msg);
	rx = be16_to_cpu(*erx);
	kfree(tran->rx_buf);
	kfree(tran->tx_buf);
	spi_message_free(msg);

	// do nothing here
}
#endif /* ASYNC */

#ifdef ASYNC2
static int max3100_sr_async2(struct max3100_port *s, u16 tx, u16 *rx);
#endif

#define MAX3100_SETRTS(r) \
	(r ? (s->invert_rts ? 0 : MAX3100_RTS) : (s->invert_rts ? MAX3100_RTS : 0))
static enum hrtimer_restart rts_timer_handler(struct hrtimer *handle)
{
	struct max3100_port *s =
			container_of(handle, struct max3100_port, rts_timer);
#ifdef ASYNC
	u16 rx;
	unsigned long flags;
	u16 tx;
	u16 *etx;

	struct spi_message *msg = spi_message_alloc(1, GFP_ATOMIC);
	//kmalloc(sizeof(struct spi_message), GFP_ATOMIC);
	int status;
	//struct spi_transfer *tran = list_first_entry(&msg->transfers,
	//		struct spi_transfer, transfer_list);
	s->spi_rts_msg = msg;
	//s->spi_tran = tran;
	//&s->spi_tran;
	// struct spi_message *msg = s->spi_msg;
	//OLD struct spi_transfer *tran = s->spi_tran;
	///tran->tx_buf = &s->spi_txbuf;
	///tran->rx_buf = &s->spi_rxbuf;
	dev_dbg(&s->spi->dev, "RTS timeout\n");

	tran->tx_buf = kmalloc(2, GFP_ATOMIC);
	tran->rx_buf = kmalloc(2, GFP_ATOMIC);
	tran->len = 2;
	etx = (u16 *)tran->tx_buf;
	tx = MAX3100_WD | MAX3100_TE | MAX3100_SETRTS(1);
	*etx = cpu_to_be16(tx);

	///dev_dbg(&s->spi->dev, "tx: %04x\n", s->spi_txbuf);
	//s->spi_txbuf[0] = ((u8*)&etx)[0];
	//s->spi_txbuf[1] = ((u8*)&etx)[1];
	//spi_message_init(&message);

	msg->complete = cb_rts_complete;
	msg->context = s;

	//spi_message_add_tail(tran, message);
	//spin_lock_irqsave(&s->spi_lock, flags);
	status = spi_async(s->spi, msg);
	//spin_unlock_irqrestore(&s->spi_lock, flags);
	if (status) {
		dev_warn(&s->spi->dev, "error while calling sr_async\n");

		return HRTIMER_RESTART;
	}
#elif defined ASYNC2
	status = max3100_sr_async2(s,
			MAX3100_WD | MAX3100_TE | MAX3100_SETRTS(1), &rx);
	if (status <= 0) {
		dev_warn(&s->spi->dev, "error while calling sr_async2\n");

		return HRTIMER_RESTART;
	}
#elif defined(THREADED)
	if (!test_and_set_bit(BIT_DRIVER_DISABLE, &s->flags))
		wake_up_process(s->main_thread);
#endif

	dev_dbg(&s->spi->dev, "RTS timeout done\n");

	//	*rx = be16_to_cpu(erx);
//	s->tx_empty = (*rx & MAX3100_T) > 0;
//	//dev_dbg(&s->spi->dev, "%04x - %04x\n", tx, *rx);
//
//	max3100_sr(s, MAX3100_WD | MAX3100_TE |
//							MAX3100_SETRTS(1), &rx);
//	// double check but we almost never receive a byte here, so safe the call
//	if ((rx & MAX3100_R) &&
//			(max3100_handlerx(s, rx) > 0) &&
//			(s->port.state->port.tty != NULL)) {
//		tty_flip_buffer_push(s->port.state->port.tty);
//	}
//	spin_lock(&s->conf_lock);
//	s->rts = 1;
//	s->rts_commit = 1;
//	spin_unlock(&s->conf_lock);
//	max3100_dowork(s);

	return HRTIMER_NORESTART;
}

#ifdef ORIGINAL
static void max3100_work(struct work_struct *w)
{
	struct timespec now;
	struct max3100_port *s = container_of(w, struct max3100_port, work);
	int rxchars;
	u16 tx, rx;
	int conf, cconf, rts, crts;
	struct circ_buf *xmit = &s->port.state->xmit;
	unsigned long diff_ns;
	//long long diff_irq;

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	getrawmonotonic(&s->ts3);
//	diff_irq = timespec_to_ns(&ts3)-timespec_to_ns(&ts1);
//	if (diff_irq > 1000000)
//		dev_dbg(&s->spi->dev, "%lld\n", diff_irq);
	rxchars = 0;
	do {
		spin_lock(&s->conf_lock);
		conf = s->conf;
		cconf = s->conf_commit;
		s->conf_commit = 0;
		rts = s->rts;
		crts = s->rts_commit;
		s->rts_commit = 0;
		spin_unlock(&s->conf_lock);
		if (cconf) {
#ifdef ASYNC2
			max3100_sr_async2(s, MAX3100_WC | conf, &rx);
#else
			max3100_sr(s, MAX3100_WC | conf, &rx);
#endif
		}
		if (crts) {
#ifdef ASYNC2
			max3100_sr_async2(s,
				MAX3100_WD | MAX3100_TE |
				MAX3100_SETRTS(s->rts), &rx);
#else
				max3100_sr(s, MAX3100_WD | MAX3100_TE | MAX3100_SETRTS(s->rts),
				           &rx);
#endif
			rxchars += max3100_handlerx(s, rx);
		}
#if 1
#ifdef ASYNC2
		max3100_sr_async2(s, MAX3100_RD, &rx);
#else
				max3100_sr(s, MAX3100_RD, &rx);
#endif
		rxchars += max3100_handlerx(s, rx);

		if (rx & MAX3100_T) {
			tx = 0xffff;
			if (s->port.x_char) {
				tx = s->port.x_char;
				s->port.icount.tx++;
				s->port.x_char = 0;
			} else if (!uart_circ_empty(xmit) &&
				   !uart_tx_stopped(&s->port)) {
				tx = xmit->buf[xmit->tail];
				xmit->tail = (xmit->tail + 1) &
					(UART_XMIT_SIZE - 1);
				s->port.icount.tx++;
			}
			if (tx != 0xffff) {
				max3100_calc_parity(s, &tx);
				// force rts off while sending
				tx |= MAX3100_WD | MAX3100_SETRTS(0);
#ifdef ASYNC2
				max3100_sr_async2(s, tx, &rx);
#else
				max3100_sr(s, tx, &rx);
#endif

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

//				if (!s->rts)
//					s->rts = 1;
				rxchars += max3100_handlerx(s, rx);
				// HACK for half duplex mode
				// wait until all data sent
				if (uart_circ_empty(xmit) || uart_tx_stopped(&s->port)) {
					diff_ns = timespec_to_ns(&s->now_ts) -
							timespec_to_ns(&now);
					//ndelay(diff_ns);
					hrtimer_start(&s->rts_timer, ktime_set(0, diff_ns),
					              HRTIMER_MODE_REL);
					dev_dbg(&s->spi->dev, "ns_old:%lu\n", diff_ns);
//					tx = MAX3100_WD | MAX3100_TE |
//							MAX3100_SETRTS(1);
//					max3100_sr(s, tx, &rx);
//					rxchars += max3100_handlerx(s, rx);
					//dev_dbg(&s->spi->dev, "Test: %04x %04x\n", tx, rx);

				}

				// disable rts after send
//				max3100_sr(s, MAX3100_WD | MAX3100_TE |
//						MAX3100_SETRTS(1), &rx);
//				rxchars += max3100_handlerx(s, rx);

			} else {
#if 0
				if (s->rts) {
					max3100_sr(s, MAX3100_WD | MAX3100_TE |
							MAX3100_SETRTS(0), &rx);
					rxchars += max3100_handlerx(s, rx);
					s->rts = 0;
//					s->rts_commit = 1;
				}
				// always disable RTS if no transmission
//				dev_err(&s->spi->dev, "r0\n");
#endif
			}
		}

		if (rxchars > 16 && s->port.state->port.tty != NULL) {
			tty_flip_buffer_push(s->port.state->port.tty);
			rxchars = 0;
		}
#endif
		if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
			uart_write_wakeup(&s->port);
	} while (!s->force_end_work &&
		 !freezing(current) &&
		 ((rx & MAX3100_R) ||
		  (!uart_circ_empty(xmit) &&
		   !uart_tx_stopped(&s->port))));
	if (rxchars > 0 && s->port.state->port.tty != NULL) {
		tty_flip_buffer_push(s->port.state->port.tty);
		//if (rxchars > 5) dev_dbg(&s->spi->dev, "rx:%d\n", rxchars);
	}
}
#endif

#ifdef ASYNC
static void cb_tx_complete(void *context)
{
	struct max3100_port *s = context;
	struct timespec now;
	unsigned long diff_ns;
	int rxchars = 0;
	struct circ_buf *xmit = &s->port.state->xmit;
	u16 rx = be16_to_cpu(s->spi_rxbuf);

	dev_dbg(&s->spi->dev, "%s: %d, %04x %04x\n", __func__,
	        s->spi_msg->status,
	        s->spi_txbuf, s->spi_rxbuf);
	getrawmonotonic(&now);

	s->prev_ts.tv_nsec = s->now_ts.tv_nsec;
	s->prev_ts.tv_sec = s->now_ts.tv_sec;

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

	rxchars += max3100_handlerx(s, rx);
	// HACK for half duplex mode
	// wait until all data sent
	if (uart_circ_empty(xmit) || uart_tx_stopped(&s->port)) {
		diff_ns = timespec_to_ns(&s->now_ts) -
				timespec_to_ns(&now);
		//ndelay(diff_ns);
		hrtimer_start(&s->rts_timer, ktime_set(0, diff_ns),
		              HRTIMER_MODE_REL);
		dev_dbg(&s->spi->dev, "ns:%lu\n", diff_ns);
//					tx = MAX3100_WD | MAX3100_TE |
//							MAX3100_SETRTS(1);
//					max3100_sr(s, tx, &rx);
//					rxchars += max3100_handlerx(s, rx);
		//dev_dbg(&s->spi->dev, "Test: %04x %04x\n", tx, rx);

	}

	// disable rts after send
//				max3100_sr(s, MAX3100_WD | MAX3100_TE |
//						MAX3100_SETRTS(1), &rx);
//				rxchars += max3100_handlerx(s, rx);
}

static int max3100_sr_async(struct max3100_port *s, u16 tx,
                            void (*complete)(void *context));

/* callback for READ_DATA reply
 * process according to R/T flags
 */
static void max3100_cb_check(void *context)
{
	struct max3100_port *s = context;
	struct circ_buf *xmit = &s->port.state->xmit;
	struct timespec now;
	unsigned long diff_ns;


	u16 *erx = &s->spi_rxbuf;
	u16 rx, tx;
	int rxchars = 0;
	if (s->spi_msg->status) {
		dev_warn(&s->spi->dev, "rs_complete status != 0\n");

		return;
	}
	dev_dbg(&s->spi->dev, "%s: %d, %04x %04x %d irq:%lu\n", __func__,
	        s->spi_msg->status,
	        s->spi_txbuf, s->spi_rxbuf, s->rx_count, s->port.irqflags);

	rx = be16_to_cpu(*erx);
	if (rx & MAX3100_R) {
		rxchars = max3100_handlerx(s, rx);

		if (rxchars > 0) {
			s->rx_count++;
			// check for more bytes in buffer
			max3100_sr_async(s, MAX3100_RD, max3100_cb_check);
		}
		if (s->rx_count > 16 && s->port.state->port.tty != NULL) {
			tty_flip_buffer_push(s->port.state->port.tty);
			s->rx_count = 0;
		}

	} else {
		if (s->rx_count > 0 && s->port.state->port.tty != NULL) {
			tty_flip_buffer_push(s->port.state->port.tty);
			dev_dbg(&s->spi->dev, "rx:%d\n", s->rx_count);
			s->rx_count = 0;
		}
		if (rx & MAX3100_T) {
				tx = 0xffff;
				if (s->port.x_char) {
					tx = s->port.x_char;
					s->port.icount.tx++;
					s->port.x_char = 0;
				} else if (!uart_circ_empty(xmit) && !uart_tx_stopped(&s->port)) {
					tx = xmit->buf[xmit->tail];
					xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
					s->port.icount.tx++;
				}
				if (tx != 0xffff) {
					max3100_calc_parity(s, &tx);
					// force rts off while sending
					tx |= MAX3100_WD | MAX3100_SETRTS(0);
					max3100_sr_async(s, tx, cb_tx_complete);
					// => cb_tx_complete

					getrawmonotonic(&now);
					s->prev_ts.tv_nsec = s->now_ts.tv_nsec;
					s->prev_ts.tv_sec = s->now_ts.tv_sec;

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

					rxchars += max3100_handlerx(s, rx);
					// HACK for half duplex mode
					// wait until all data sent
					if (uart_circ_empty(xmit) || uart_tx_stopped(&s->port)) {
						diff_ns = timespec_to_ns(&s->now_ts) -
								timespec_to_ns(&now);
						//ndelay(diff_ns);
						hrtimer_start(&s->rts_timer, ktime_set(0, diff_ns),
						              HRTIMER_MODE_REL);
						dev_dbg(&s->spi->dev, "ns:%lu\n", diff_ns);
				//					tx = MAX3100_WD | MAX3100_TE |
				//							MAX3100_SETRTS(1);
				//					max3100_sr(s, tx, &rx);
				//					rxchars += max3100_handlerx(s, rx);
						//dev_dbg(&s->spi->dev, "Test: %04x %04x\n", tx, rx);

					}
				}
		}
	}


#ifdef CHECK
	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&s->port);
#endif

	dev_dbg(&s->spi->dev, "%s: done\n", __func__);

//    while (!s->force_end_work &&
//	 !freezing(current) &&
//	 ((rx & MAX3100_R) ||
//	  (!uart_circ_empty(xmit) &&
//	   !uart_tx_stopped(&s->port))));
}

static int max3100_sr_async(struct max3100_port *s, u16 tx,
                            void (*complete)(void *context))
{
	//DECLARE_COMPLETION_ONSTACK(done);
	int status;
	unsigned long flags;
	struct spi_message *msg = s->spi_msg;
	//struct spi_transfer *tran = s->spi_tran;
	spi_message_init(msg);
	spi_message_add_tail(s->spi_tran, msg);

	s->spi_txbuf = cpu_to_be16(tx); // linked to tran->txbuf
	s->spi_rxbuf = 0;
	msg->context = s;
	msg->complete = complete;

	dev_dbg(&s->spi->dev, "spi_async: %04x\n", s->spi_txbuf);
	spin_lock_irqsave(&s->spi_lock, flags);
	status = spi_async(s->spi, msg);
	spin_unlock_irqrestore(&s->spi_lock, flags);
	if (status) {
		dev_warn(&s->spi->dev, "error while calling spi_async\n");
		return -1;
	}
	//wait_for_completion(&done);
	//if (status)

	return 0;
}

static irqreturn_t max3100_irq_new(int irqno, void *dev_id)
{
	//DECLARE_COMPLETION_ONSTACK(done);
	struct max3100_port *s = dev_id;
	u16 *tx = (u16 *)s->spi_tran->tx_buf;
	//int status = -1;

	// check our status first, reading data if there is some in buffer
	struct spi_message *msg = s->spi_msg;


	dev_dbg(&s->spi->dev, "%d == %d i:%lu\n", s->irq, irqno, s->port.irqflags);
	if (s->rts_commit || s->conf_commit) {
		max3100_dowork(s);
		dev_dbg(&s->spi->dev, "change rts or conf per work queue\n");
		return IRQ_HANDLED;
	}

	max3100_sr_async(s, MAX3100_RD, max3100_cb_check);
	// cb => max3100_cb_check(...)

	return IRQ_HANDLED;
}
#endif

#ifdef ASYNC2
/* ISR for incomming MAX3100 interrupts
 * using async SPI communication for data transfers or workqueue for
 * configuration changes
 */
static void spidev_complete(void *arg)
{
	complete(arg);
}

static int max3100_sr_async2(struct max3100_port *s, u16 tx, u16 *rx)
{
	DECLARE_COMPLETION_ONSTACK(done);
	struct spi_message *msg = s->spi_msg;
	//struct spi_transfer *tran = s->spi_tran;
	int status = -1;

	spi_message_init(msg);
	spi_message_add_tail(s->spi_tran, msg);
	msg->context = &done;
	msg->complete = spidev_complete;
	s->spi_txbuf = cpu_to_be16(tx); // linked to tran->txbuf
	s->spi_rxbuf = 0;
	//dev_dbg(&s->spi->dev, "%s t: %04x\n", __func__, tx);

	//spin_lock_irq(&s->spi_lock);
	status = spi_async(s->spi, msg);
	//spin_unlock_irq(&s->spi_lock);

	if (status == 0) {
		wait_for_completion(&done);
		status = msg->status;
		if (status == 0) {
			status = msg->actual_length;
			*rx = be16_to_cpu(s->spi_rxbuf);
		}
		dev_dbg(&s->spi->dev, "%s: %04x %04x\n", __func__,
		        tx, *rx);
	} else {
		dev_warn(&s->spi->dev, "%s: couldn't send %d\n", __func__,
				status);
	}
	return status;
}
static irqreturn_t max3100_irq_new2(int irqno, void *dev_id)
{
	struct max3100_port *s = dev_id;
	u16 rx, tx;
	int rxchars = 0;
	struct circ_buf *xmit = &s->port.state->xmit;

	// check our status first, reading data if there is some in buffer

	dev_dbg(&s->spi->dev, "%d == %d i:%lu\n", s->irq, irqno, s->port.irqflags);
	if (s->rts_commit || s->conf_commit) {
		max3100_dowork(s);
		dev_dbg(&s->spi->dev, "change rts or conf per work queue\n");
		return IRQ_HANDLED;
	}

	do {
		max3100_sr_async2(s, MAX3100_RD, &rx);
		if (rx & MAX3100_R) {
			rxchars += max3100_handlerx(s, rx);
		} else {
			if (rxchars > 0 && s->port.state->port.tty != NULL) {
				tty_flip_buffer_push(s->port.state->port.tty);
				dev_dbg(&s->spi->dev, "rx:%d\n", rxchars);
				s->rx_count = 0;
			}
			if (rx & MAX3100_T) {
				tx = 0xffff;
				if (s->port.x_char) {
					tx = s->port.x_char;
					s->port.icount.tx++;
					s->port.x_char = 0;
				} else if (!uart_circ_empty(xmit) && !uart_tx_stopped(&s->port)) {
					tx = xmit->buf[xmit->tail];
					xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
					s->port.icount.tx++;
				}
				if (tx != 0xffff) {
					max3100_calc_parity(s, &tx);
					// force rts off while sending
					tx |= MAX3100_WD | MAX3100_SETRTS(0);
					max3100_sr_async2(s, tx, &rx);
					// => cb_tx_complete
				}
			}
		}
		if (rxchars > 16 && s->port.state->port.tty != NULL) {
			tty_flip_buffer_push(s->port.state->port.tty);
			rxchars = 0;
		}
		if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
			uart_write_wakeup(&s->port);

	} while (!s->force_end_work &&
			!freezing(current) &&
			((rx & MAX3100_R) ||
				(!uart_circ_empty(xmit) && (rx & MAX3100_T) &&
						!uart_tx_stopped(&s->port))));

	//max3100_sr_async(s, MAX3100_RD, max3100_cb_check);
	// cb => max3100_cb_check(...)

	return IRQ_HANDLED;
}
#endif

#ifdef ORIGINAL
static irqreturn_t max3100_irq(int irqno, void *dev_id)
{
	struct max3100_port *s = dev_id;

	dev_dbg(&s->spi->dev, "%d\n", s->irq);

	max3100_dowork(s);
	return IRQ_HANDLED;
}
#endif

#ifdef THREADED
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
		//if (cur & MAX3100_R) {
		if (test_and_clear_bit(BIT_RX_PENDING, &s->flags)) {
			str[j++] = cur & 0xff;
		} else {
			break;
		}

		ibuf++;
	}

//	ibuf = s->spi_rxbuf;
//	for (i = 0; i < MAX3100_RX_FIFOLEN; ++i) {
//		cur = be16_to_cpu(ibuf[i]);
//		dev_dbg(&s->spi->dev, "%s %04x\n", __func__, cur);
//		if (cur & MAX3100_R) {
//			str[j++] = cur & 0xff;
//		}
//	}
	/* check if last transfer was not empty */
//	if (i == MAX3100_RX_FIFOLEN)
//		set_bit(BIT_RX_PENDING, &s->flags);
//	s->tx_empty = (cur & MAX3100_T);

	if (j) {
		max3140_receive_chars(s, str, j);
		/* keep RX_PENDING flag, when FIFO was full,
		 * otherwise IRQ line is never deasserted */
		if (j == MAX3100_RX_FIFOLEN)
			set_bit(BIT_RX_PENDING, &s->flags);
	}
	dev_dbg(&s->spi->dev, "%s cnt: %d, tx_empty %d\n", __func__, j, s->tx_empty);
	//max3140_sr(s, MAX3100_RD, &rx, MAX3100_RX_FIFOLEN);

	return j;
}
static int max3140_send_and_receive(struct max3100_port *s)
{
	struct timespec now, start, end;
	int rxchars, rxlen;
	u16 tx, rx;
	u8 crx;
	int conf, cconf, rts, crts;
	struct circ_buf *xmit = &s->port.state->xmit;
	unsigned long diff_ns;

	dev_dbg(&s->spi->dev, "%s\n", __func__);
	rxchars = 0;
	rxlen = 0;
	getrawmonotonic(&start);

	do {
		while ((s->flags & (1 << BIT_TX_STARTED)) && s->tx_empty) {
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
				//if (rx & MAX3100_R) {
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
					//tx = MAX3100_WD | MAX3100_TE |
					//MAX3100_SETRTS(1);
					//max3100_sr(s, tx, &rx);
					//rxchars += max3100_handlerx(s, rx);
					//dev_dbg(&s->spi->dev, "Test: %04x %04x\n", tx, rx);

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
		//if (rxchars > 5) dev_dbg(&s->spi->dev, "rx:%d\n", rxchars);
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

//		/* first handle console output */
//		if (test_and_clear_bit(CON_TX_NEEDED, &s->uart_flags))
//			send_circ_buf(s, xmit);
//
//		/* handle uart output */
//		if (test_and_clear_bit(UART_TX_NEEDED, &s->uart_flags))
//			transmit_char(s);

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

#endif

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

#ifndef THREADED
	max3100_dowork(s);
#else
	//max3140_cmd(s, MAX3100_RC);
	if (!test_and_set_bit(BIT_TX_STARTED, &s->flags))
		wake_up_process(s->main_thread);
#endif
}

static void max3100_stop_rx(struct uart_port *port)
{
	struct max3100_port *s = container_of(port,
					      struct max3100_port,
					      port);

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	s->rx_enabled = 0;
#ifndef THREADED
	spin_lock(&s->conf_lock);
#endif
	s->conf &= ~MAX3100_RM;
#ifndef THREADED
	s->conf_commit = 1;
	spin_unlock(&s->conf_lock);
	max3100_dowork(s);
#else
	if (!test_and_set_bit(BIT_CONF_COMMIT, &s->flags))
		wake_up_process(s->main_thread);
#endif
}

static unsigned int max3100_tx_empty(struct uart_port *port)
{
	struct max3100_port *s = container_of(port,
					      struct max3100_port,
					      port);

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	/* may not be truly up-to-date */
	max3100_dowork(s);
	return s->tx_empty;
}

static unsigned int max3100_get_mctrl(struct uart_port *port)
{
	struct max3100_port *s = container_of(port,
					      struct max3100_port,
					      port);

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	/* may not be truly up-to-date */
	max3100_dowork(s);
	/* always assert DCD and DSR since these lines are not wired */
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

	spin_lock(&s->conf_lock);
	if (s->rts != rts) {
		dev_dbg(&s->spi->dev, "%s crts %u\n", __func__, mctrl);
		s->rts = rts;
		s->rts_commit = 1;
		max3100_dowork(s);
	}
	spin_unlock(&s->conf_lock);
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
	u16 rx;

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
#ifndef THREADED
	max3100_dowork(s);
#else
	//max3140_cmd(s, s->conf | MAX3100_WC);
	//max3140_cmd(s, MAX3100_RC);

	set_bit(BIT_CONF_COMMIT, &s->flags);
	set_bit(BIT_DRIVER_DISABLE, &s->flags);
	wake_up_process(s->main_thread);
#endif

	if (UART_ENABLE_MS(&s->port, termios->c_cflag))
		max3100_enable_ms(&s->port);
}

static void max3100_shutdown(struct uart_port *port)
{
	struct max3100_port *s = container_of(port,
					      struct max3100_port,
					      port);

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	if (s->suspending)
		return;

	s->force_end_work = 1;

	if (s->poll_time > 0)
		del_timer_sync(&s->timer);
	hrtimer_cancel(&s->rts_timer);

	if (s->workqueue) {
		flush_workqueue(s->workqueue);
		destroy_workqueue(s->workqueue);
		s->workqueue = NULL;
	}
	if (s->irq)
		free_irq(s->irq, s);

	/* set shutdown mode to save power */
	if (s->max3100_hw_suspend)
		s->max3100_hw_suspend(1);
	else  {
		u16 tx, rx;

		tx = MAX3100_WC | MAX3100_SHDN;
#ifndef THREADED
		max3100_sr(s, tx, &rx);
#else
		max3140_cmd(s, tx);
#endif
	}
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
#ifdef ORIGINAL
	//s->workqueue = create_freezeable_workqueue(b);
	s->workqueue = alloc_workqueue(b, WQ_HIGHPRI | WQ_FREEZEABLE | WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!s->workqueue) {
		dev_warn(&s->spi->dev, "cannot create workqueue\n");
		return -EBUSY;
	}
	INIT_WORK(&s->work, max3100_work);

	if (request_irq(s->irq, max3100_irq,
			IRQF_TRIGGER_FALLING, "max3100", s) < 0) {
		dev_warn(&s->spi->dev, "cannot allocate irq %d\n", s->irq);
		s->irq = 0;
		destroy_workqueue(s->workqueue);
		s->workqueue = NULL;
		return -EBUSY;
	}
#elif defined(ASYNC2)
	if (request_threaded_irq(s->irq, NULL, max3100_irq_new2,
			IRQF_TRIGGER_FALLING, "max3100", s) < 0) {
//		if (request_irq(s->irq, max3100_irq_new2,
//				IRQF_TRIGGER_FALLING, "max3100", s) < 0) {
		dev_warn(&s->spi->dev, "cannot allocate irq %d\n", s->irq);
		s->irq = 0;
		destroy_workqueue(s->workqueue);
		s->workqueue = NULL;
		return -EBUSY;
	}
#elif defined(THREADED)
	if (request_irq(s->irq, max3140_irq,
			IRQF_TRIGGER_FALLING, "max3100", s) < 0) {
		dev_warn(&s->spi->dev, "cannot allocate irq %d\n", s->irq);
		s->irq = 0;
		return -EBUSY;
	}
#else

#endif

	if (s->loopback) {
		u16 tx, rx;
		tx = 0x4001;
#ifndef THREADED
		max3100_sr(s, tx, &rx);
#else
		max3140_cmd(s, tx);
#endif
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
	struct plat_max3100 *pdata;
	u16 tx, rx;
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

#ifdef THREADED
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
#endif

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
	max3100s[i]->max3100_hw_suspend = pdata->max3100_hw_suspend;

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
	if (max3100s[i]->max3100_hw_suspend)
		max3100s[i]->max3100_hw_suspend(1);
	else {
		tx = MAX3100_WC | MAX3100_SHDN;
#ifdef THREADED
		max3140_cmd(max3100s[i], tx);
#else
		max3100_sr(max3100s[i], tx, &rx);
#endif
	}
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

static int max3100_suspend(struct spi_device *spi, pm_message_t state)
{
	struct max3100_port *s = dev_get_drvdata(&spi->dev);

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	disable_irq(s->irq);

	s->suspending = 1;
	uart_suspend_port(&max3100_uart_driver, &s->port);

	if (s->max3100_hw_suspend)
		s->max3100_hw_suspend(1);
	else {
		/* no HW suspend, so do SW one */
		u16 tx, rx;

		tx = MAX3100_WC | MAX3100_SHDN;
#ifndef THREADED
		max3100_sr(s, tx, &rx);
#else
		max3140_cmd(s, tx);
#endif
	}
	return 0;
}

static int max3100_resume(struct spi_device *spi)
{
	struct max3100_port *s = dev_get_drvdata(&spi->dev);

	dev_err(&s->spi->dev, "%s\n", __func__);

	if (s->max3100_hw_suspend)
		s->max3100_hw_suspend(0);
	uart_resume_port(&max3100_uart_driver, &s->port);
	s->suspending = 0;

	enable_irq(s->irq);

	s->conf_commit = 1;
	if (s->workqueue)
		max3100_dowork(s);

	return 0;
}

#else
#define max3100_suspend NULL
#define max3100_resume  NULL
#endif

static struct spi_driver max3140_driver = {
	.driver = {
		.name		= "max3140hd",
		.bus		= &spi_bus_type,
		.owner		= THIS_MODULE,
	},

	.probe		= max3100_probe,
	.remove		= __devexit_p(max3100_remove),
	.suspend	= max3100_suspend,
	.resume		= max3100_resume,
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
