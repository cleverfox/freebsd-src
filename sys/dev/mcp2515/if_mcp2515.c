/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Vladimir Goncharov <devel@viruzzz.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Microchip MCP2515 stand-alone CAN controller (SPI), classic CAN 2.0B.
 *
 * The chip is reached over spibus(4); its INT pin is an FDT GPIO interrupt.
 * The driver presents a "canN" network interface to the netcan(4) PF_CAN stack:
 * it hands received frames to can_input() and transmits frames delivered to
 * if_transmit().  BPF tapping and local loopback are done centrally by netcan
 * (can_input()/can_output()), so this driver only moves frames to/from silicon.
 *
 * Concurrency: SPI transfers sleep, but if_transmit() runs in the network
 * epoch and the INT handler in primary interrupt context, neither of which may
 * sleep.  Both transmit and interrupt work are therefore deferred to a fast
 * taskqueue whose worker thread performs the SPI, serialized by an sx(9) lock.
 * The INT pin is edge-triggered: the filter only schedules the task, and the
 * task drains the chip's interrupt flags, so it neither sleeps in the wrong
 * context nor leaves a level interrupt asserted.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/mbuf.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/sx.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_private.h>
#include <net/if_types.h>
#include <net/bpf.h>
#include <net/vnet.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/spibus/spi.h>
#include "spibus_if.h"

#include <netcan/can.h>
#include <netcan/can_var.h>

/* SPI instruction set (datasheet table 12-1). */
#define	MCP_RESET	0xC0
#define	MCP_READ	0x03
#define	MCP_WRITE	0x02
#define	MCP_RTS		0x80	/* | (1 << n) for TXBn */
#define	MCP_READ_STATUS	0xA0
#define	MCP_BIT_MODIFY	0x05

/* Registers. */
#define	CANSTAT		0x0E
#define	CANCTRL		0x0F
#define	CNF3		0x28
#define	CNF2		0x29
#define	CNF1		0x2A
#define	CANINTE		0x2B
#define	CANINTF		0x2C
#define	EFLG		0x2D
#define	TXB0CTRL	0x30
#define	TXB0SIDH	0x31	/* SIDH,SIDL,EID8,EID0,DLC,D0.. follow */
#define	RXB0CTRL	0x60
#define	RXB0SIDH	0x61
#define	RXB1CTRL	0x70
#define	RXB1SIDH	0x71

/* CANCTRL/CANSTAT mode bits (REQOP / OPMOD). */
#define	MODE_NORMAL	0x00
#define	MODE_LOOPBACK	0x40
#define	MODE_LISTEN	0x60
#define	MODE_CONFIG	0x80
#define	MODE_MASK	0xE0

/* CANINTF / CANINTE bits. */
#define	RX0IF		0x01
#define	RX1IF		0x02
#define	TX0IF		0x04
#define	TX1IF		0x08
#define	TX2IF		0x10
#define	ERRIF		0x20
#define	INTE_RX		(RX0IF | RX1IF)
#define	INTE_ERR	ERRIF

/* TXBnCTRL / RXBnSIDL / RXBnDLC bits. */
#define	TXREQ		0x08
#define	TXERR		0x10
#define	SIDL_IDE	0x08	/* extended identifier present */
#define	SIDL_EXIDE_TX	0x08	/* set in TX SIDL to send extended */
#define	DLC_RTR		0x40
#define	DLC_LEN_MASK	0x0F

#define	MCP_TX_TIMEOUT	1000	/* TXREQ poll iterations (~10 ms) */

struct mcp2515_softc {
	device_t		 dev;
	device_t		 parent;	/* spibus */
	struct ifnet		*ifp;
	struct sx		 sx;		/* serializes chip access */
	struct mtx		 txlock;	/* protects txq */
	struct mbufq		 txq;
	struct taskqueue	*tq;
	struct task		 tx_task;
	struct task		 int_task;
	struct resource		*irq_res;
	int			 irq_rid;
	void			*irq_cookie;
	uint32_t		 fosc;		/* crystal Hz */
	int			 bitrate;	/* kbit/s */
	bool			 running;
	bool			 published;	/* if_attach() done */
};

/*
 * Bit-timing computation, implemented from the MCP2515 data sheet (DS21801,
 * "Bit Time Configuration") and the standard CAN nominal bit-time model.  It
 * is original code, not derived from Linux's GPL-licensed bit-timing routines.
 *
 * A nominal bit is SyncSeg(1) + PropSeg + PhaseSeg1 + PhaseSeg2 time quanta
 * (TQ), with TQ = 2 * (BRP + 1) / Fosc -- i.e. the TQ clock is Fosc / 2.  The
 * data sheet limits: PropSeg, PhaseSeg1 = 1..8 TQ; PhaseSeg2 = 2..8 TQ;
 * SJW = 1..4 TQ; BRP + 1 = 1..64.  The sample point is at the end of PhaseSeg1.
 */
#define	MCP_NTQ_MIN	5	/* 1 + 1 + 1 + 2 */
#define	MCP_NTQ_MAX	25	/* 1 + 8 + 8 + 8 */
#define	MCP_DIV_MAX	64	/* prescaler, = BRP + 1 */
#define	MCP_SEG_MAX	8
#define	MCP_PS2_MIN	2

/* CiA-recommended sample point (per mille) for a given nominal bitrate. */
static u_int
mcp_sample_point(u_int bitrate)
{
	if (bitrate > 800000)
		return (750);
	if (bitrate > 500000)
		return (800);
	return (875);
}

/*
 * Solve for CNF1/2/3 from the oscillator frequency and a target bitrate.
 * Scan the prescaler (BRP + 1 = 1..64) for the divisor giving a whole number
 * of time quanta per bit (NTQ) in range with the least bitrate error, then lay
 * out the segments so the sample point lands near the CiA target.  Returns 0,
 * or EINVAL if the bitrate cannot be met within ~5%.
 */
static int
mcp_compute_cnf(u_int fosc, u_int bitrate, uint8_t *cnf1, uint8_t *cnf2,
    uint8_t *cnf3)
{
	u_int tqclk, div, best_div = 0, best_ntq = 0, best_err = ~0U;
	u_int ntq, tqf, rate, err, sp, ps2, ps1, prop, rem, sjw;

	if (bitrate == 0)
		return (EINVAL);
	tqclk = fosc / 2;			/* TQ clock = Fosc / 2 */

	for (div = 1; div <= MCP_DIV_MAX; div++) {
		tqf = tqclk / div;		/* time quanta per second */
		if (tqf < bitrate * MCP_NTQ_MIN)
			break;			/* NTQ too small now */
		ntq = (tqf + bitrate / 2) / bitrate;	/* rounded */
		if (ntq > MCP_NTQ_MAX)
			ntq = MCP_NTQ_MAX;
		rate = tqf / ntq;
		err = (rate > bitrate) ? rate - bitrate : bitrate - rate;
		if (err < best_err) {
			best_err = err;
			best_div = div;
			best_ntq = ntq;
			if (err == 0)
				break;
		}
	}
	if (best_div == 0 || best_err * 20 > bitrate)	/* worse than ~5% */
		return (EINVAL);

	/*
	 * Lay out NTQ quanta: PhaseSeg2 spans the part after the sample point,
	 * SyncSeg is 1, the rest is PropSeg + PhaseSeg1 (each <= 8 TQ).
	 */
	ntq = best_ntq;
	sp = mcp_sample_point(bitrate);
	ps2 = ntq - (ntq * sp) / 1000;
	if (ps2 < MCP_PS2_MIN)
		ps2 = MCP_PS2_MIN;
	else if (ps2 > MCP_SEG_MAX)
		ps2 = MCP_SEG_MAX;
	if (ntq - 1 - ps2 > 2 * MCP_SEG_MAX)
		ps2 = ntq - 1 - 2 * MCP_SEG_MAX;	/* cap remainder */
	rem = ntq - 1 - ps2;			/* PropSeg + PhaseSeg1 */
	ps1 = (rem + 1) / 2;
	if (ps1 > MCP_SEG_MAX)
		ps1 = MCP_SEG_MAX;
	prop = rem - ps1;

	/* SJW = 1 TQ: the conservative default (the data sheet allows 1..4). */
	sjw = 1;

	*cnf1 = ((sjw - 1) << 6) | (best_div - 1);	/* SJW | BRP */
	*cnf2 = 0x80 | ((ps1 - 1) << 3) | (prop - 1);	/* BTLMODE */
	*cnf3 = ps2 - 1;				/* PHSEG2 */
	return (0);
}

static struct ofw_compat_data compat_data[] = {
	{ "microchip,mcp2515",	1 },
	{ NULL,			0 },
};

/*
 * Low-level SPI helpers.  All assume the caller holds sc->sx (or that the chip
 * is otherwise quiescent, e.g. during attach before the interface is up).
 */
static int
mcp_xfer(struct mcp2515_softc *sc, uint8_t *tx, uint8_t *rx, size_t n)
{
	struct spi_command cmd = SPI_COMMAND_INITIALIZER;

	cmd.tx_cmd = tx;
	cmd.rx_cmd = rx;
	cmd.tx_cmd_sz = cmd.rx_cmd_sz = n;
	return (SPIBUS_TRANSFER(sc->parent, sc->dev, &cmd));
}

static void
mcp_reset(struct mcp2515_softc *sc)
{
	uint8_t tx = MCP_RESET, rx;

	(void)mcp_xfer(sc, &tx, &rx, 1);
	DELAY(10000);		/* oscillator start-up + internal reset */
}

static uint8_t
mcp_read(struct mcp2515_softc *sc, uint8_t addr)
{
	uint8_t tx[3] = { MCP_READ, addr, 0 }, rx[3];

	(void)mcp_xfer(sc, tx, rx, sizeof(tx));
	return (rx[2]);
}

static void
mcp_write(struct mcp2515_softc *sc, uint8_t addr, uint8_t val)
{
	uint8_t tx[3] = { MCP_WRITE, addr, val }, rx[3];

	(void)mcp_xfer(sc, tx, rx, sizeof(tx));
}

static void
mcp_modify(struct mcp2515_softc *sc, uint8_t addr, uint8_t mask, uint8_t val)
{
	uint8_t tx[4] = { MCP_BIT_MODIFY, addr, mask, val }, rx[4];

	(void)mcp_xfer(sc, tx, rx, sizeof(tx));
}

static int
mcp_set_mode(struct mcp2515_softc *sc, uint8_t mode)
{
	int i;

	mcp_modify(sc, CANCTRL, MODE_MASK, mode);
	for (i = 0; i < 100; i++) {
		if ((mcp_read(sc, CANSTAT) & MODE_MASK) == mode)
			return (0);
		DELAY(1000);
	}
	device_printf(sc->dev, "mode 0x%02x not reached (CANSTAT=0x%02x)\n",
	    mode, mcp_read(sc, CANSTAT));
	return (ETIMEDOUT);
}

/* Program CNF1..3 for the configured bitrate.  Chip must be in config mode. */
static int
mcp_set_timing(struct mcp2515_softc *sc)
{
	uint8_t cnf1, cnf2, cnf3;

	if (mcp_compute_cnf(sc->fosc, (u_int)sc->bitrate * 1000,
	    &cnf1, &cnf2, &cnf3) != 0) {
		device_printf(sc->dev,
		    "cannot derive bit timing for %d kbit/s at %u Hz\n",
		    sc->bitrate, sc->fosc);
		return (EINVAL);
	}
	mcp_write(sc, CNF1, cnf1);
	mcp_write(sc, CNF2, cnf2);
	mcp_write(sc, CNF3, cnf3);
	return (0);
}

/* Bring the controller into config mode with filters open (accept-all). */
static int
mcp_configure(struct mcp2515_softc *sc)
{
	int error;

	mcp_reset(sc);
	if ((mcp_read(sc, CANSTAT) & MODE_MASK) != MODE_CONFIG) {
		device_printf(sc->dev, "not in config mode after reset\n");
		return (ENXIO);
	}
	if ((error = mcp_set_timing(sc)) != 0)
		return (error);
	/* Receive any frame; roll RXB0 overflow into RXB1. */
	mcp_modify(sc, RXB0CTRL, 0x64, 0x60 | 0x04);
	mcp_modify(sc, RXB1CTRL, 0x60, 0x60);
	mcp_write(sc, CANINTE, INTE_RX | INTE_ERR);
	mcp_write(sc, CANINTF, 0);
	return (0);
}

/* Load one classic CAN frame into TXB0 and request transmission. */
static void
mcp_hw_send(struct mcp2515_softc *sc, struct mbuf *m)
{
	struct can_frame *cf;
	uint8_t buf[2 + 5 + CAN_MAX_DLEN], rx[sizeof(buf)];
	canid_t id;
	int eff, rtr, len, i;

	if (m->m_len < (int)sizeof(struct can_frame)) {
		m = m_pullup(m, sizeof(struct can_frame));
		if (m == NULL)
			return;
	}
	cf = mtod(m, struct can_frame *);
	eff = (cf->can_id & CAN_EFF_FLAG) != 0;
	rtr = (cf->can_id & CAN_RTR_FLAG) != 0;
	id = cf->can_id & (eff ? CAN_EFF_MASK : CAN_SFF_MASK);
	len = cf->len & DLC_LEN_MASK;
	if (len > CAN_MAX_DLEN)
		len = CAN_MAX_DLEN;

	/* Wait for the buffer to be free from any previous transmission. */
	for (i = 0; i < MCP_TX_TIMEOUT; i++) {
		if ((mcp_read(sc, TXB0CTRL) & TXREQ) == 0)
			break;
		DELAY(10);
	}

	buf[0] = MCP_WRITE;
	buf[1] = TXB0SIDH;
	if (eff) {
		buf[2] = id >> 21;
		buf[3] = ((id >> 18) & 0x07) << 5 | SIDL_EXIDE_TX |
		    ((id >> 16) & 0x03);
		buf[4] = (id >> 8) & 0xFF;
		buf[5] = id & 0xFF;
	} else {
		buf[2] = (id >> 3) & 0xFF;
		buf[3] = (id & 0x07) << 5;
		buf[4] = 0;
		buf[5] = 0;
	}
	buf[6] = (rtr ? DLC_RTR : 0) | len;
	for (i = 0; i < len; i++)
		buf[7 + i] = cf->data[i];
	(void)mcp_xfer(sc, buf, rx, 7 + len);

	/* RTS for TXB0. */
	buf[0] = MCP_RTS | 0x01;
	(void)mcp_xfer(sc, buf, rx, 1);
}

/* Read one classic CAN frame from RX buffer n into a fresh mbuf. */
static struct mbuf *
mcp_hw_recv(struct mcp2515_softc *sc, int n)
{
	uint8_t base = (n == 0) ? RXB0SIDH : RXB1SIDH;
	uint8_t flag = (n == 0) ? RX0IF : RX1IF;
	uint8_t tx[2 + 5 + CAN_MAX_DLEN], rx[sizeof(tx)];
	struct can_frame *cf;
	struct mbuf *m;
	canid_t id;
	int eff, len, i;

	memset(tx, 0, sizeof(tx));
	tx[0] = MCP_READ;
	tx[1] = base;
	(void)mcp_xfer(sc, tx, rx, sizeof(tx));	/* SIDH..data */

	eff = (rx[3] & SIDL_IDE) != 0;
	if (eff) {
		id = ((canid_t)rx[2] << 21) |
		    ((canid_t)(rx[3] >> 5) << 18) |
		    ((canid_t)(rx[3] & 0x03) << 16) |
		    ((canid_t)rx[4] << 8) | rx[5];
		id &= CAN_EFF_MASK;
		id |= CAN_EFF_FLAG;
	} else {
		id = ((canid_t)rx[2] << 3) | (rx[3] >> 5);
		id &= CAN_SFF_MASK;
	}
	len = rx[6] & DLC_LEN_MASK;
	if (len > CAN_MAX_DLEN)
		len = CAN_MAX_DLEN;
	if (rx[6] & DLC_RTR)
		id |= CAN_RTR_FLAG;

	m = m_gethdr(M_NOWAIT, MT_DATA);
	if (m == NULL) {
		mcp_modify(sc, CANINTF, flag, 0);
		if_inc_counter(sc->ifp, IFCOUNTER_IQDROPS, 1);
		return (NULL);
	}
	cf = mtod(m, struct can_frame *);
	memset(cf, 0, sizeof(*cf));
	cf->can_id = id;
	cf->len = len;
	if ((id & CAN_RTR_FLAG) == 0)
		for (i = 0; i < len; i++)
			cf->data[i] = rx[7 + i];
	m->m_len = m->m_pkthdr.len = sizeof(struct can_frame);

	mcp_modify(sc, CANINTF, flag, 0);	/* clear RXnIF */
	return (m);
}

/*
 * Deferred transmit: runs in the taskqueue thread (may sleep), drains the
 * software queue into the chip.
 */
static void
mcp2515_tx_task(void *arg, int pending __unused)
{
	struct mcp2515_softc *sc = arg;
	struct mbuf *m;

	for (;;) {
		mtx_lock(&sc->txlock);
		m = mbufq_dequeue(&sc->txq);
		mtx_unlock(&sc->txlock);
		if (m == NULL)
			break;
		sx_xlock(&sc->sx);
		if (sc->running) {
			mcp_hw_send(sc, m);
			if_inc_counter(sc->ifp, IFCOUNTER_OPACKETS, 1);
			if_inc_counter(sc->ifp, IFCOUNTER_OBYTES,
			    m->m_pkthdr.len);
		} else {
			if_inc_counter(sc->ifp, IFCOUNTER_OERRORS, 1);
		}
		sx_xunlock(&sc->sx);
		m_freem(m);
	}
}

static int
mcp2515_transmit(struct ifnet *ifp, struct mbuf *m)
{
	struct mcp2515_softc *sc = if_getsoftc(ifp);
	int error;

	mtx_lock(&sc->txlock);
	error = mbufq_enqueue(&sc->txq, m);
	mtx_unlock(&sc->txlock);
	if (error != 0) {
		m_freem(m);
		if_inc_counter(ifp, IFCOUNTER_OQDROPS, 1);
		return (error);
	}
	taskqueue_enqueue(sc->tq, &sc->tx_task);
	return (0);
}

static void
mcp2515_qflush(struct ifnet *ifp)
{
	struct mcp2515_softc *sc = if_getsoftc(ifp);

	mtx_lock(&sc->txlock);
	mbufq_drain(&sc->txq);
	mtx_unlock(&sc->txlock);
}

/*
 * Hard interrupt filter.  This runs in primary interrupt context, where
 * sleeping is prohibited and SPI transfers (which sleep) are not allowed, so
 * it only schedules the deferred task.  The INT pin is configured
 * edge-triggered (falling) in the device tree, so it does not re-fire while
 * the task drains the chip.
 */
static int
mcp2515_intr(void *arg)
{
	struct mcp2515_softc *sc = arg;

	taskqueue_enqueue(sc->tq, &sc->int_task);
	return (FILTER_HANDLED);
}

/*
 * Deferred interrupt work, in taskqueue (kernel-thread) context where the sx
 * lock and SPI may sleep.  Drain CANINTF until clear so that frames arriving
 * while we run are not lost.
 */
static void
mcp2515_int_task(void *arg, int pending __unused)
{
	struct mcp2515_softc *sc = arg;
	struct mbuf *m0, *m1;
	uint8_t flags, eflg;

	for (;;) {
		m0 = m1 = NULL;
		sx_xlock(&sc->sx);
		if (!sc->running) {
			sx_xunlock(&sc->sx);
			break;
		}
		flags = mcp_read(sc, CANINTF);
		if (flags == 0) {
			sx_xunlock(&sc->sx);
			break;
		}
		if (flags & RX0IF)
			m0 = mcp_hw_recv(sc, 0);
		if (flags & RX1IF)
			m1 = mcp_hw_recv(sc, 1);
		if (flags & (TX0IF | TX1IF | TX2IF))
			mcp_modify(sc, CANINTF,
			    flags & (TX0IF | TX1IF | TX2IF), 0);
		if (flags & ERRIF) {
			eflg = mcp_read(sc, EFLG);
			if (eflg & 0xC0)	/* RX0OVR | RX1OVR */
				if_inc_counter(sc->ifp, IFCOUNTER_IERRORS, 1);
			mcp_write(sc, EFLG, 0);
			mcp_modify(sc, CANINTF, ERRIF, 0);
		}
		sx_xunlock(&sc->sx);

		/*
		 * can_input() touches the VNET-virtualized CAN pcb list, but
		 * this runs in a taskqueue thread with no vnet context set, so
		 * establish it (as if_cantap does on its RX path) to avoid a
		 * NULL curvnet dereference.
		 */
		if (m0 != NULL) {
			if_inc_counter(sc->ifp, IFCOUNTER_IPACKETS, 1);
			CURVNET_SET_QUIET(if_getvnet(sc->ifp));
			can_input(sc->ifp, m0);
			CURVNET_RESTORE();
		}
		if (m1 != NULL) {
			if_inc_counter(sc->ifp, IFCOUNTER_IPACKETS, 1);
			CURVNET_SET_QUIET(if_getvnet(sc->ifp));
			can_input(sc->ifp, m1);
			CURVNET_RESTORE();
		}
	}
}

static int
mcp2515_up(struct mcp2515_softc *sc)
{
	int error;

	sx_xlock(&sc->sx);
	error = mcp_configure(sc);
	if (error == 0)
		error = mcp_set_mode(sc, MODE_NORMAL);
	if (error == 0) {
		sc->running = true;
		if_setdrvflagbits(sc->ifp, IFF_DRV_RUNNING, 0);
	}
	sx_xunlock(&sc->sx);
	return (error);
}

static void
mcp2515_down(struct mcp2515_softc *sc)
{

	sx_xlock(&sc->sx);
	sc->running = false;
	(void)mcp_set_mode(sc, MODE_CONFIG);
	if_setdrvflagbits(sc->ifp, 0, IFF_DRV_RUNNING);
	sx_xunlock(&sc->sx);
	mcp2515_qflush(sc->ifp);
}

static int
mcp2515_ioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
{
	struct mcp2515_softc *sc = if_getsoftc(ifp);
	struct ifreq *ifr = (struct ifreq *)data;

	switch (cmd) {
	case SIOCSIFFLAGS:
		if (if_getflags(ifp) & IFF_UP) {
			if (!sc->running)
				return (mcp2515_up(sc));
		} else {
			if (sc->running)
				mcp2515_down(sc);
		}
		return (0);
	case SIOCSIFMTU:
		/* Classic CAN only. */
		if (ifr->ifr_mtu != (int)sizeof(struct can_frame))
			return (EINVAL);
		if_setmtu(ifp, ifr->ifr_mtu);
		return (0);
	case SIOCADDMULTI:
	case SIOCDELMULTI:
		return (0);	/* CAN has no multicast addressing */
	default:
		return (EINVAL);
	}
}

static int
mcp2515_sysctl_bitrate(SYSCTL_HANDLER_ARGS)
{
	struct mcp2515_softc *sc = arg1;
	uint8_t cnf1, cnf2, cnf3;
	int error, val = sc->bitrate;

	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	/* Accept any rate the timing solver can reach at this oscillator. */
	if (val < 5 || val > 1000 ||
	    mcp_compute_cnf(sc->fosc, (u_int)val * 1000,
	    &cnf1, &cnf2, &cnf3) != 0)
		return (EINVAL);
	sc->bitrate = val;
	if (sc->running) {
		/* Re-program timing: cycle config -> normal. */
		mcp2515_down(sc);
		return (mcp2515_up(sc));
	}
	return (0);
}

/*
 * Second attach stage, run from a config_intrhook: probing the chip takes
 * SPI transfers, which sleep with a timeout in the controller driver, and
 * during a cold boot device_attach() runs before timers work, where a timed
 * sleep panics.  In the hook, sleeping is allowed (the same pattern as the
 * at45d(4)/mx25l(4) SPI flash drivers); when the module is loaded after
 * boot, the hook simply runs right away.
 */
static void
mcp2515_delayed_attach(void *arg)
{
	struct mcp2515_softc *sc = arg;
	int error;

	/* Probe the chip: reset and confirm it lands in config mode. */
	sx_xlock(&sc->sx);
	error = mcp_configure(sc);
	if (error == 0)
		(void)mcp_set_mode(sc, MODE_CONFIG);
	sx_xunlock(&sc->sx);
	if (error != 0) {
		device_printf(sc->dev, "MCP2515 not responding\n");
		return;
	}

	/* Publish the interface; nothing below this can fail. */
	if_attach(sc->ifp);
	bpfattach(sc->ifp, DLT_CAN_SOCKETCAN, 0);
	sc->published = true;

	device_printf(sc->dev, "%s: MCP2515 @ %u MHz xtal, default %d kbit/s\n",
	    if_name(sc->ifp), sc->fosc / 1000000, sc->bitrate);
}

static int
mcp2515_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "Microchip MCP2515 CAN controller");
	return (BUS_PROBE_DEFAULT);
}

static int
mcp2515_attach(device_t dev)
{
	struct mcp2515_softc *sc = device_get_softc(dev);
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *tree;
	struct ifnet *ifp;
	phandle_t node;
	uint32_t freq;
	int error;

	sc->dev = dev;
	sc->parent = device_get_parent(dev);
	sc->bitrate = 500;
	sc->fosc = 8000000;
	node = ofw_bus_get_node(dev);
	if (OF_getencprop(node, "clock-frequency", &freq, sizeof(freq)) > 0)
		sc->fosc = freq;
	if (sc->fosc < 1000000 || sc->fosc > 25000000) {
		device_printf(dev, "oscillator %u Hz out of range (1-25 MHz)\n",
		    sc->fosc);
		return (ENXIO);
	}

	sx_init(&sc->sx, "mcp2515");
	mtx_init(&sc->txlock, "mcp2515 txq", NULL, MTX_DEF);
	mbufq_init(&sc->txq, 64);
	TASK_INIT(&sc->tx_task, 0, mcp2515_tx_task, sc);
	TASK_INIT(&sc->int_task, 0, mcp2515_int_task, sc);
	/*
	 * A "fast" taskqueue: its enqueue lock is a spin mutex, so the
	 * interrupt filter may enqueue work from primary interrupt context.
	 * The worker thread still runs tasks in a sleepable context (SPI).
	 */
	sc->tq = taskqueue_create_fast("mcp2515 taskq", M_WAITOK,
	    taskqueue_thread_enqueue, &sc->tq);
	taskqueue_start_threads(&sc->tq, 1, PI_NET, "%s taskq",
	    device_get_nameunit(dev));

	/*
	 * Allocate and configure the ifnet, but defer if_attach()/bpfattach()
	 * to the delayed attach stage: no error path then needs if_detach()
	 * (which crashes on a CAN ifnet via rib_walk_del during teardown).
	 * The chip is not touched until that stage either; should its INT
	 * line fire before then, the interrupt task sees !sc->running and
	 * returns without any SPI access.
	 */
	ifp = if_alloc(IFT_OTHER);
	sc->ifp = ifp;
	if_setsoftc(ifp, sc);
	if_initname(ifp, "can", device_get_unit(dev));
	if_setmtu(ifp, sizeof(struct can_frame));	/* classic CAN */
	if_setflags(ifp, IFF_SIMPLEX);			/* not IP-capable */
	if_settransmitfn(ifp, mcp2515_transmit);
	if_setqflushfn(ifp, mcp2515_qflush);
	if_setioctlfn(ifp, mcp2515_ioctl);
	if_setoutputfn(ifp, can_if_output);

	/* INT pin: FDT GPIO interrupt; ithread (SPI may sleep). */
	sc->irq_rid = 0;
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &sc->irq_rid,
	    RF_ACTIVE);
	if (sc->irq_res == NULL) {
		device_printf(dev, "cannot allocate IRQ\n");
		error = ENXIO;
		goto fail_ifp;
	}
	error = bus_setup_intr(dev, sc->irq_res, INTR_TYPE_NET | INTR_MPSAFE,
	    mcp2515_intr, NULL, sc, &sc->irq_cookie);
	if (error != 0) {
		device_printf(dev, "cannot set up interrupt\n");
		goto fail_irq;
	}

	ctx = device_get_sysctl_ctx(dev);
	tree = device_get_sysctl_tree(dev);
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "bitrate",
	    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_NEEDGIANT, sc, 0,
	    mcp2515_sysctl_bitrate, "I", "CAN bitrate (kbit/s)");

	/*
	 * Probe the chip and publish the interface from a config_intrhook:
	 * SPI transfers sleep, which panics in device_attach() during a cold
	 * boot (before timers work) when the driver is compiled in or
	 * preloaded.  See mcp2515_delayed_attach().
	 */
	config_intrhook_oneshot(mcp2515_delayed_attach, sc);
	return (0);

fail_irq:
	bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rid, sc->irq_res);
fail_ifp:
	if_free(ifp);
	taskqueue_free(sc->tq);
	mbufq_drain(&sc->txq);
	mtx_destroy(&sc->txlock);
	sx_destroy(&sc->sx);
	return (error);
}

static int
mcp2515_detach(device_t dev)
{
	struct mcp2515_softc *sc = device_get_softc(dev);

	if (sc->running)
		mcp2515_down(sc);
	if (sc->irq_cookie != NULL)
		bus_teardown_intr(dev, sc->irq_res, sc->irq_cookie);
	if (sc->irq_res != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rid,
		    sc->irq_res);
	if (sc->ifp != NULL) {
		if (sc->published) {
			bpfdetach(sc->ifp);
			if_detach(sc->ifp);
		}
		if_free(sc->ifp);
	}
	if (sc->tq != NULL) {
		taskqueue_drain(sc->tq, &sc->tx_task);
		taskqueue_drain(sc->tq, &sc->int_task);
		taskqueue_free(sc->tq);
	}
	mbufq_drain(&sc->txq);
	mtx_destroy(&sc->txlock);
	sx_destroy(&sc->sx);
	return (0);
}

static device_method_t mcp2515_methods[] = {
	DEVMETHOD(device_probe,		mcp2515_probe),
	DEVMETHOD(device_attach,	mcp2515_attach),
	DEVMETHOD(device_detach,	mcp2515_detach),
	DEVMETHOD_END
};

static driver_t mcp2515_driver = {
	"mcp2515",
	mcp2515_methods,
	sizeof(struct mcp2515_softc),
};

DRIVER_MODULE(mcp2515, spibus, mcp2515_driver, NULL, NULL);
MODULE_DEPEND(mcp2515, spibus, 1, 1, 1);
MODULE_DEPEND(mcp2515, can, 1, 1, 1);
SPIBUS_FDT_PNP_INFO(compat_data);
