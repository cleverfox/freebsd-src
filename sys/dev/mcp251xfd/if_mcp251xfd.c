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
 * Microchip MCP2517FD / MCP2518FD / MCP251863 stand-alone CAN FD controller
 * (SPI).  Implemented from the Microchip "MCP25XXFD Family Reference Manual"
 * (DS20005678B), the MCP2518FD data sheet (DS20006027B) and the MCP2518FD
 * errata sheet (DS80000789E); it shares no code with the GPL Linux driver.
 *
 * The chip is reached over spibus(4); its INT pin is an FDT GPIO interrupt.
 * The driver presents a "canN" network interface to the netcan(4) PF_CAN
 * stack, transmitting both classic CAN 2.0B and CAN FD frames.  The frame
 * format follows the interface MTU, as elsewhere in netcan: MTU 16 (the
 * default) runs the controller in Normal CAN 2.0 mode, MTU 72 in Normal
 * CAN FD mode with bit-rate switching between the nominal (sysctl bitrate)
 * and data phase (sysctl dbitrate) bit rates.
 *
 * Unlike the MCP2515 with its fixed buffers, this family exposes 2 KB of
 * message RAM partitioned into FIFOs.  The driver uses a static layout:
 * FIFO 1 as an 8-deep transmit FIFO and FIFO 2 as a 16-deep receive FIFO,
 * both with 64-byte payload objects, plus one accept-all filter routing every
 * frame into FIFO 2.  The transmit event FIFO and TXQ are disabled.
 *
 * Concurrency follows mcp2515(4): SPI transfers sleep, but if_transmit()
 * runs in the network epoch and the INT handler in primary interrupt context,
 * neither of which may sleep, so both are deferred to a fast taskqueue whose
 * worker thread performs the SPI, serialized by an sx(9) lock.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/endian.h>
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
#include <netcan/can/error.h>
#include <netcan/can_var.h>

/*
 * SPI instruction format (DS20005678B section 4): a 16-bit command word of
 * 4-bit opcode and 12-bit register/RAM address, followed by data bytes.
 * Registers are 32-bit little-endian and byte-addressable; RAM (0x400-0xBFF)
 * must be accessed 4-byte aligned in multiples of 4 bytes.
 */
#define	MCPFD_OP_RESET		0x0
#define	MCPFD_OP_READ		0x3
#define	MCPFD_OP_WRITE		0x2

#define	MCPFD_RAM_BASE		0x400
#define	MCPFD_RAM_SIZE		2048

/* CAN FD controller module registers (DS20005678B chapter 4). */
#define	C1CON			0x000
#define	C1NBTCFG		0x004
#define	C1DBTCFG		0x008
#define	C1TDC			0x00C
#define	C1INT			0x01C
#define	C1TREC			0x034
#define	C1BDIAG0		0x038
#define	C1BDIAG1		0x03C
#define	C1FIFOCON(n)		(0x050 + 12 * (n))	/* n = 1..31 */
#define	C1FIFOSTA(n)		(0x054 + 12 * (n))
#define	C1FIFOUA(n)		(0x058 + 12 * (n))
#define	C1FLTCON(n)		(0x1D0 + 4 * (n))	/* 4 filters/reg */
#define	C1FLTOBJ(n)		(0x1F0 + 8 * (n))
#define	C1MASK(n)		(0x1F4 + 8 * (n))

/* MCP2517/18FD-specific registers at 0xE00 (DS20006027B chapter 5). */
#define	MCPFD_OSC		0xE00
#define	MCPFD_IOCON		0xE04
#define	MCPFD_ECCCON		0xE0C

/* C1CON bits. */
#define	C1CON_ISOCRCEN		(1U << 5)	/* ISO 11898-1:2015 FD CRC */
#define	C1CON_PXEDIS		(1U << 6)	/* protocol exception disable */
#define	C1CON_WAKFIL		(1U << 8)
#define	C1CON_WFT_T11		(3U << 9)	/* wake-up filter time */
#define	C1CON_RTXAT		(1U << 16)	/* restrict TX attempts */
#define	C1CON_OPMOD_SHIFT	21
#define	C1CON_OPMOD_MASK	(7U << 21)
#define	C1CON_REQOP_SHIFT	24

/* C1TREC bits (error counters and error state). */
#define	TREC_REC(x)		((x) & 0xFF)
#define	TREC_TEC(x)		(((x) >> 8) & 0xFF)
#define	TREC_RXWARN		(1U << 17)
#define	TREC_TXWARN		(1U << 18)
#define	TREC_RXBP		(1U << 19)	/* RX error passive */
#define	TREC_TXBP		(1U << 20)	/* TX error passive */
#define	TREC_TXBO		(1U << 21)	/* bus off */

/* C1BDIAG1 sticky bus-error flags (N = nominal, D = data bit rate). */
#define	BDIAG1_NBIT0ERR		(1U << 16)
#define	BDIAG1_NBIT1ERR		(1U << 17)
#define	BDIAG1_NACKERR		(1U << 18)
#define	BDIAG1_NFORMERR		(1U << 19)
#define	BDIAG1_NSTUFERR		(1U << 20)
#define	BDIAG1_NCRCERR		(1U << 21)
#define	BDIAG1_DBIT0ERR		(1U << 24)
#define	BDIAG1_DBIT1ERR		(1U << 25)
#define	BDIAG1_DFORMERR		(1U << 27)
#define	BDIAG1_DSTUFERR		(1U << 28)
#define	BDIAG1_DCRCERR		(1U << 29)

/* OPMOD/REQOP mode codes. */
#define	MCPFD_MODE_NORMAL_FD	0
#define	MCPFD_MODE_SLEEP	1
#define	MCPFD_MODE_INT_LOOP	2
#define	MCPFD_MODE_LISTENONLY	3
#define	MCPFD_MODE_CONFIG	4
#define	MCPFD_MODE_EXT_LOOP	5
#define	MCPFD_MODE_NORMAL_CAN20	6
#define	MCPFD_MODE_RESTRICTED	7

/* C1INT: flags in bits 15:0, matching enables in bits 31:16. */
#define	C1INT_RXIF		(1U << 1)
#define	C1INT_MODIF		(1U << 3)
#define	C1INT_TXATIF		(1U << 10)
#define	C1INT_RXOVIF		(1U << 11)
#define	C1INT_SERRIF		(1U << 12)
#define	C1INT_CERRIF		(1U << 13)
#define	C1INT_IVMIF		(1U << 15)
#define	C1INT_RXIE		(1U << 17)
#define	C1INT_TXATIE		(1U << 26)
#define	C1INT_RXOVIE		(1U << 27)
#define	C1INT_CERRIE		(1U << 29)
#define	C1INT_IVMIE		(1U << 31)
/* Flags cleared by writing 0 (the rest mirror FIFO/module status). */
#define	C1INT_CLEARABLE		0xF00C
#define	C1INT_HANDLED		(C1INT_RXIF | C1INT_TXATIF | C1INT_RXOVIF | \
				 C1INT_MODIF | C1INT_SERRIF | C1INT_CERRIF | \
				 C1INT_IVMIF)

/* C1FIFOCONn bits. */
#define	FIFOCON_TFNRFNIE	(1U << 0)	/* TX not full / RX not empty */
#define	FIFOCON_RXOVIE		(1U << 3)
#define	FIFOCON_TXATIE		(1U << 4)	/* TX attempts exhausted */
#define	FIFOCON_TXEN		(1U << 7)
#define	FIFOCON_TXAT_SHIFT	21		/* 0 one-shot, 1 = 3, 3 = inf */
#define	FIFOCON_FSIZE_SHIFT	24		/* objects - 1 */
#define	FIFOCON_PLSIZE_SHIFT	29		/* 7 = 64-byte payload */
#define	FIFOCON_FRESET		(1U << 10)	/* self-clearing FIFO reset */
/* Byte 1 of C1FIFOCONn, written alone to trigger the action bits. */
#define	FIFOCON_B1_UINC		0x01		/* bit 8 */
#define	FIFOCON_B1_TXREQ	0x02		/* bit 9 */
#define	FIFOCON_B1_FRESET	0x04		/* bit 10 */

/* C1FIFOSTAn bits. */
#define	FIFOSTA_TFNRFNIF	(1U << 0)
#define	FIFOSTA_TFERFFIF	(1U << 2)	/* TX empty / RX full */
#define	FIFOSTA_RXOVIF		(1U << 3)
#define	FIFOSTA_FIFOCI(x)	(((x) >> 8) & 0x1F)	/* tail index */

/* C1FLTCONn: one control byte per filter. */
#define	FLTCON_FLTEN		0x80

/* TX/RX message object (DS20005678B tables 3-2..3-6), two 32-bit LE words. */
#define	MCPFD_OBJ_HDRLEN	8
#define	MCPFD_OBJ_SID_MASK	0x7FF
#define	MCPFD_OBJ_EID_SHIFT	11		/* EID[17:0] in word 0 */
#define	MCPFD_OBJ_DLC_MASK	0x0F		/* word 1 */
#define	MCPFD_OBJ_IDE		(1U << 4)
#define	MCPFD_OBJ_RTR		(1U << 5)
#define	MCPFD_OBJ_BRS		(1U << 6)
#define	MCPFD_OBJ_FDF		(1U << 7)
#define	MCPFD_OBJ_ESI		(1U << 8)

/* OSC register bits. */
#define	OSC_PLLEN		(1U << 0)
#define	OSC_CLKODIV10		(3U << 5)	/* POR default CLKO divider */
#define	OSC_PLLRDY		(1U << 8)
#define	OSC_OSCRDY		(1U << 10)

/* ECCCON byte 0. */
#define	ECCCON_ECCEN		0x01

/* IOCON byte 3: INT0/INT1 pins as GPIO, push-pull INT/TXCAN. */
#define	IOCON_B3_PM0_PM1	0x03

/*
 * Static message RAM layout: FIFO 1 = TX, FIFO 2 = RX, both with 64-byte
 * payload objects (72 bytes per object), 8 * 72 + 16 * 72 = 1728 <= 2048.
 */
#define	MCPFD_TX_FIFO		1
#define	MCPFD_TX_DEPTH		8
#define	MCPFD_TX_OBJSZ		(MCPFD_OBJ_HDRLEN + CANFD_MAX_DLEN)
#define	MCPFD_RX_FIFO		2
#define	MCPFD_RX_DEPTH		16

#define	MCPFD_TX_TIMEOUT	1000	/* FIFO-full poll iterations (~10 ms) */
#define	MCPFD_MODE_TIMEOUT	100	/* mode-change polls (~100 ms) */

struct mcp251xfd_softc {
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
	uint32_t		 sysclk;	/* SYSCLK Hz (fosc or PLL) */
	int			 bitrate;	/* nominal, kbit/s */
	int			 dbitrate;	/* data phase, kbit/s */
	int			 txattempts;	/* -1 unlim/0 one-shot/3 */
	int			 txat_dbg;	/* TX-abort diagnostics left */
	bool			 berr;		/* per-bus-error reporting */
	bool			 fd;		/* CAN FD mode (MTU 72) */
	bool			 running;
	bool			 published;	/* if_attach() done */
};

/* CAN FD DLC coding (ISO 11898-1): payload length for each DLC value. */
static const uint8_t mcpfd_dlc2len[16] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64
};

static uint8_t
mcpfd_len2dlc(u_int len)
{
	u_int dlc;

	if (len <= 8)
		return (len);
	for (dlc = 9; dlc < 15; dlc++)
		if (len <= mcpfd_dlc2len[dlc])
			break;
	return (dlc);
}

/*
 * Bit-timing computation, shared between the nominal and data phases.  The
 * approach follows mcp2515(4)'s solver, generalized to the wider MCP25XXFD
 * field ranges (DS20005678B registers C1NBTCFG/C1DBTCFG): the time quantum
 * clock is SYSCLK / BRP (no fixed /2 as on the MCP2515), a bit is
 * SyncSeg(1) + TSEG1 + TSEG2 quanta, and the sample point ends TSEG1.
 */
struct mcpfd_bt_limits {
	u_int	tseg1_max;	/* TSEG1 = PropSeg + PhaseSeg1 */
	u_int	tseg2_max;
	u_int	sjw_max;
	u_int	brp_max;
	u_int	ntq_min;
	u_int	ntq_max;	/* 1 + tseg1_max + tseg2_max */
};

static const struct mcpfd_bt_limits mcpfd_nominal_limits = {
	.tseg1_max = 256, .tseg2_max = 128, .sjw_max = 128,
	.brp_max = 256, .ntq_min = 8, .ntq_max = 385,
};

static const struct mcpfd_bt_limits mcpfd_data_limits = {
	.tseg1_max = 32, .tseg2_max = 16, .sjw_max = 16,
	.brp_max = 256, .ntq_min = 5, .ntq_max = 49,
};

#define	MCPFD_TSEG2_MIN	2

/* CiA-recommended sample point (per mille) for a given bitrate. */
static u_int
mcpfd_sample_point(u_int bitrate)
{
	if (bitrate > 800000)
		return (750);
	if (bitrate > 500000)
		return (800);
	return (875);
}

/*
 * Solve for BRP/TSEG1/TSEG2/SJW (all as programmed + 1, i.e. in natural
 * units) from the SYSCLK frequency and a target bitrate.  Returns 0, or
 * EINVAL if the bitrate cannot be met within ~1%.
 */
static int
mcpfd_compute_bt(u_int fclk, u_int bitrate, const struct mcpfd_bt_limits *lim,
    u_int *brpp, u_int *tseg1p, u_int *tseg2p, u_int *sjwp)
{
	u_int brp, best_brp = 0, best_ntq = 0, best_err = ~0U;
	u_int ntq, tqf, rate, err, sp, tseg1, tseg2, sjw;

	if (bitrate == 0)
		return (EINVAL);
	for (brp = 1; brp <= lim->brp_max; brp++) {
		tqf = fclk / brp;		/* time quanta per second */
		if (tqf < bitrate * lim->ntq_min)
			break;			/* NTQ too small from here on */
		ntq = (tqf + bitrate / 2) / bitrate;	/* rounded */
		if (ntq > lim->ntq_max)
			ntq = lim->ntq_max;
		rate = tqf / ntq;
		err = (rate > bitrate) ? rate - bitrate : bitrate - rate;
		if (err < best_err) {
			best_err = err;
			best_brp = brp;
			best_ntq = ntq;
			if (err == 0)
				break;
		}
	}
	if (best_brp == 0 || best_err * 100 > bitrate)	/* worse than ~1% */
		return (EINVAL);

	/*
	 * Lay out the quanta: TSEG2 spans the part after the sample point,
	 * SyncSeg is 1, TSEG1 the rest.
	 */
	ntq = best_ntq;
	sp = mcpfd_sample_point(bitrate);
	tseg2 = ntq - (ntq * sp) / 1000;
	if (tseg2 < MCPFD_TSEG2_MIN)
		tseg2 = MCPFD_TSEG2_MIN;
	else if (tseg2 > lim->tseg2_max)
		tseg2 = lim->tseg2_max;
	if (ntq - 1 - tseg2 > lim->tseg1_max)
		tseg2 = ntq - 1 - lim->tseg1_max;	/* cap remainder */
	tseg1 = ntq - 1 - tseg2;

	/* Large SJW eases resynchronization; cap at PhaseSeg2. */
	sjw = (tseg2 < lim->sjw_max) ? tseg2 : lim->sjw_max;

	*brpp = best_brp;
	*tseg1p = tseg1;
	*tseg2p = tseg2;
	*sjwp = sjw;
	return (0);
}

static struct ofw_compat_data compat_data[] = {
	{ "microchip,mcp2517fd",	1 },
	{ "microchip,mcp2518fd",	1 },
	{ "microchip,mcp251863",	1 },
	{ NULL,				0 },
};

/*
 * Low-level SPI helpers.  All assume the caller holds sc->sx (or that the
 * chip is otherwise quiescent, e.g. during attach before the interface is
 * up).  The two command bytes carry the opcode and register/RAM address.
 */
static int
mcpfd_xfer(struct mcp251xfd_softc *sc, uint8_t *tx, uint8_t *rx, size_t n)
{
	struct spi_command cmd = SPI_COMMAND_INITIALIZER;

	cmd.tx_cmd = tx;
	cmd.rx_cmd = rx;
	cmd.tx_cmd_sz = cmd.rx_cmd_sz = n;
	return (SPIBUS_TRANSFER(sc->parent, sc->dev, &cmd));
}

static void
mcpfd_setcmd(uint8_t *buf, u_int op, uint16_t addr)
{
	buf[0] = (op << 4) | ((addr >> 8) & 0x0F);
	buf[1] = addr & 0xFF;
}

static void
mcpfd_reset(struct mcp251xfd_softc *sc)
{
	uint8_t tx[2] = { 0, 0 }, rx[2];

	(void)mcpfd_xfer(sc, tx, rx, sizeof(tx));
	DELAY(1000);
}

static void
mcpfd_read_buf(struct mcp251xfd_softc *sc, uint16_t addr, uint8_t *buf,
    size_t n)
{
	uint8_t tx[2 + MCPFD_OBJ_HDRLEN + CANFD_MAX_DLEN];
	uint8_t rx[sizeof(tx)];

	KASSERT(n <= sizeof(tx) - 2, ("mcp251xfd: read too long"));
	memset(tx, 0, 2 + n);
	mcpfd_setcmd(tx, MCPFD_OP_READ, addr);
	(void)mcpfd_xfer(sc, tx, rx, 2 + n);
	memcpy(buf, rx + 2, n);
}

static void
mcpfd_write_buf(struct mcp251xfd_softc *sc, uint16_t addr, const uint8_t *buf,
    size_t n)
{
	uint8_t tx[2 + MCPFD_OBJ_HDRLEN + CANFD_MAX_DLEN];
	uint8_t rx[sizeof(tx)];

	KASSERT(n <= sizeof(tx) - 2, ("mcp251xfd: write too long"));
	mcpfd_setcmd(tx, MCPFD_OP_WRITE, addr);
	memcpy(tx + 2, buf, n);
	(void)mcpfd_xfer(sc, tx, rx, 2 + n);
}

static uint32_t
mcpfd_read32(struct mcp251xfd_softc *sc, uint16_t addr)
{
	uint8_t buf[4];

	mcpfd_read_buf(sc, addr, buf, sizeof(buf));
	return (le32dec(buf));
}

static void
mcpfd_write32(struct mcp251xfd_softc *sc, uint16_t addr, uint32_t val)
{
	uint8_t buf[4];

	le32enc(buf, val);
	mcpfd_write_buf(sc, addr, buf, sizeof(buf));
}

/*
 * Single-byte register write.  Also the required access width for IOCON:
 * per MCP2518FD errata DS80000789E item 5, a 32-bit write to IOCON can
 * corrupt the LAT0/LAT1 bits, so IOCON must be written bytewise.
 */
static void
mcpfd_write8(struct mcp251xfd_softc *sc, uint16_t addr, uint8_t val)
{

	mcpfd_write_buf(sc, addr, &val, 1);
}

static void
mcpfd_write16(struct mcp251xfd_softc *sc, uint16_t addr, uint16_t val)
{
	uint8_t buf[2];

	buf[0] = val & 0xFF;
	buf[1] = val >> 8;
	mcpfd_write_buf(sc, addr, buf, sizeof(buf));
}

static int
mcpfd_set_mode(struct mcp251xfd_softc *sc, u_int mode)
{
	u_int cur;
	int i;

	/* REQOP lives in bits 26:24: byte 3 of C1CON (ABAT/TXBWS zeroed). */
	mcpfd_write8(sc, C1CON + 3, mode);
	for (i = 0; i < MCPFD_MODE_TIMEOUT; i++) {
		cur = (mcpfd_read32(sc, C1CON) & C1CON_OPMOD_MASK) >>
		    C1CON_OPMOD_SHIFT;
		if (cur == mode)
			return (0);
		DELAY(1000);
	}
	device_printf(sc->dev, "mode %u not reached (OPMOD=%u)\n", mode, cur);
	return (ETIMEDOUT);
}

/* Program C1NBTCFG/C1DBTCFG/C1TDC.  Chip must be in configuration mode. */
static int
mcpfd_set_timing(struct mcp251xfd_softc *sc)
{
	u_int brp, tseg1, tseg2, sjw, tdco;
	int error;

	error = mcpfd_compute_bt(sc->sysclk, (u_int)sc->bitrate * 1000,
	    &mcpfd_nominal_limits, &brp, &tseg1, &tseg2, &sjw);
	if (error != 0) {
		device_printf(sc->dev,
		    "cannot derive nominal bit timing for %d kbit/s at %u Hz\n",
		    sc->bitrate, sc->sysclk);
		return (error);
	}
	mcpfd_write32(sc, C1NBTCFG, ((brp - 1) << 24) | ((tseg1 - 1) << 16) |
	    ((tseg2 - 1) << 8) | (sjw - 1));

	error = mcpfd_compute_bt(sc->sysclk, (u_int)sc->dbitrate * 1000,
	    &mcpfd_data_limits, &brp, &tseg1, &tseg2, &sjw);
	if (error != 0) {
		/* Data timing only matters when FD (MTU 72) is selected. */
		if (!sc->fd)
			return (0);
		device_printf(sc->dev,
		    "cannot derive data bit timing for %d kbit/s at %u Hz\n",
		    sc->dbitrate, sc->sysclk);
		return (error);
	}
	mcpfd_write32(sc, C1DBTCFG, ((brp - 1) << 24) | ((tseg1 - 1) << 16) |
	    ((tseg2 - 1) << 8) | (sjw - 1));

	/*
	 * Transmitter delay compensation for the data phase, in automatic
	 * mode: the secondary sample point is the measured loop delay plus
	 * TDCO, set to the data-phase sample point in SYSCLK cycles
	 * (DS20005678B, C1TDC).  TDCO is a 7-bit signed field.
	 */
	tdco = brp * (1 + tseg1);
	if (tdco > 63)
		tdco = 63;
	mcpfd_write32(sc, C1TDC, (2U << 16) | (tdco << 8));
	return (0);
}

/*
 * Full chip (re)configuration: soft reset, oscillator, ECC + RAM init, base
 * configuration, bit timing, FIFOs, filter and interrupt enables.  Leaves
 * the chip in configuration mode.
 */
static int
mcpfd_configure(struct mcp251xfd_softc *sc)
{
	uint8_t zeros[64];
	uint32_t val;
	uint16_t addr;
	int error, i;

	/*
	 * The RESET instruction is only specified for configuration mode, so
	 * request it first (best effort: the chip may be unresponsive or
	 * already there), then reset and verify the C1CON reset signature.
	 */
	mcpfd_write8(sc, C1CON + 3, MCPFD_MODE_CONFIG);
	DELAY(2000);
	mcpfd_reset(sc);

	val = mcpfd_read32(sc, C1CON);
	if ((val & (C1CON_OPMOD_MASK | C1CON_PXEDIS | C1CON_ISOCRCEN)) !=
	    ((MCPFD_MODE_CONFIG << C1CON_OPMOD_SHIFT) | C1CON_PXEDIS |
	    C1CON_ISOCRCEN)) {
		device_printf(sc->dev,
		    "bad C1CON after reset: 0x%08x\n", val);
		return (ENXIO);
	}

	/* Oscillator: PLL (x10) only for a 4 MHz crystal, no SYSCLK divide. */
	mcpfd_write8(sc, MCPFD_OSC,
	    OSC_CLKODIV10 | (sc->fosc == 4000000 ? OSC_PLLEN : 0));
	for (i = 0; i < 100; i++) {
		val = mcpfd_read32(sc, MCPFD_OSC);
		if ((val & OSC_OSCRDY) != 0 &&
		    (sc->fosc != 4000000 || (val & OSC_PLLRDY) != 0))
			break;
		DELAY(100);
	}
	if ((val & OSC_OSCRDY) == 0) {
		device_printf(sc->dev, "oscillator not ready (OSC=0x%08x)\n",
		    val);
		return (ENXIO);
	}

	/* INT0/INT1 pins as GPIO inputs; bytewise per errata DS80000789E 5. */
	mcpfd_write8(sc, MCPFD_IOCON + 3, IOCON_B3_PM0_PM1);

	/*
	 * Enable ECC, then initialize the whole message RAM so every word
	 * has valid parity before it is ever read.  Doubles as an SPI sanity
	 * check: verify one written word reads back.
	 */
	mcpfd_write8(sc, MCPFD_ECCCON, ECCCON_ECCEN);
	mcpfd_write32(sc, MCPFD_RAM_BASE, 0xA55A5AA5);
	if (mcpfd_read32(sc, MCPFD_RAM_BASE) != 0xA55A5AA5) {
		device_printf(sc->dev, "message RAM readback failed\n");
		return (ENXIO);
	}
	memset(zeros, 0, sizeof(zeros));
	for (addr = MCPFD_RAM_BASE; addr < MCPFD_RAM_BASE + MCPFD_RAM_SIZE;
	    addr += sizeof(zeros))
		mcpfd_write_buf(sc, addr, zeros, sizeof(zeros));

	/* Base configuration: ISO FD CRC, TXQ and TEF off, stay in config. */
	val = ((uint32_t)MCPFD_MODE_CONFIG << C1CON_REQOP_SHIFT) |
	    C1CON_WFT_T11 | C1CON_WAKFIL | C1CON_PXEDIS | C1CON_ISOCRCEN;
	if (sc->txattempts >= 0)
		val |= C1CON_RTXAT;	/* honor the per-FIFO TXAT limit */
	mcpfd_write32(sc, C1CON, val);

	if ((error = mcpfd_set_timing(sc)) != 0)
		return (error);

	/*
	 * TX FIFO: 8 objects x 64-byte payload.  TXAT encodes the
	 * retransmission limit: 0 = none (one-shot), 1 = three attempts,
	 * 3 = unlimited; it takes effect only with C1CON.RTXAT set.
	 */
	val = (7U << FIFOCON_PLSIZE_SHIFT) |
	    ((MCPFD_TX_DEPTH - 1U) << FIFOCON_FSIZE_SHIFT) | FIFOCON_TXEN;
	if (sc->txattempts < 0)
		val |= 3U << FIFOCON_TXAT_SHIFT;
	else
		val |= ((sc->txattempts == 3) ? 1U : 0U) <<
		    FIFOCON_TXAT_SHIFT | FIFOCON_TXATIE;
	mcpfd_write32(sc, C1FIFOCON(MCPFD_TX_FIFO), val);

	/* RX FIFO: 16 objects x 64-byte payload, not-empty + overflow IRQ. */
	mcpfd_write32(sc, C1FIFOCON(MCPFD_RX_FIFO),
	    (7U << FIFOCON_PLSIZE_SHIFT) |
	    ((MCPFD_RX_DEPTH - 1U) << FIFOCON_FSIZE_SHIFT) |
	    FIFOCON_RXOVIE | FIFOCON_TFNRFNIE);

	/* Filter 0: match everything (mask 0), deliver into the RX FIFO. */
	mcpfd_write32(sc, C1FLTOBJ(0), 0);
	mcpfd_write32(sc, C1MASK(0), 0);
	mcpfd_write8(sc, C1FLTCON(0), FLTCON_FLTEN | MCPFD_RX_FIFO);

	/* Interrupt enables; also clears any stale clearable flags. */
	val = C1INT_RXIE | C1INT_RXOVIE | C1INT_CERRIE;
	if (sc->txattempts >= 0)
		val |= C1INT_TXATIE;
	if (sc->berr)
		val |= C1INT_IVMIE;	/* per-bus-error interrupt: floods */
	mcpfd_write32(sc, C1INT, val);
	return (0);
}

/* Load one frame into the TX FIFO and request transmission. */
static void
mcpfd_hw_send(struct mcp251xfd_softc *sc, struct mbuf *m)
{
	uint8_t obj[MCPFD_OBJ_HDRLEN + CANFD_MAX_DLEN];
	union {
		struct can_frame cf;
		struct canfd_frame cfd;
	} u;
	const uint8_t *payload;
	uint32_t w0, w1;
	canid_t id;
	uint16_t addr;
	u_int dlc, len, wire;
	bool fdf, eff, rtr;
	int i;

	/* Copy the frame out; the caller keeps ownership of the mbuf. */
	fdf = (m->m_pkthdr.len == CANFD_MTU);
	m_copydata(m, 0, m->m_pkthdr.len, (caddr_t)&u);

	if (fdf) {
		id = u.cfd.can_id;
		rtr = false;			/* no remote frames in FD */
		len = (u.cfd.len > CANFD_MAX_DLEN) ? CANFD_MAX_DLEN : u.cfd.len;
		dlc = mcpfd_len2dlc(len);
		wire = mcpfd_dlc2len[dlc];	/* len rounded to a valid DLC */
		w1 = dlc | MCPFD_OBJ_FDF |
		    ((u.cfd.flags & CANFD_BRS) ? MCPFD_OBJ_BRS : 0);
		payload = u.cfd.data;
	} else {
		id = u.cf.can_id;
		rtr = (id & CAN_RTR_FLAG) != 0;
		len = (u.cf.len > CAN_MAX_DLEN) ? CAN_MAX_DLEN : u.cf.len;
		/* Pass through a raw DLC of 9..15 (8-byte payload). */
		dlc = (len == CAN_MAX_DLEN && u.cf.len8_dlc > CAN_MAX_DLC &&
		    u.cf.len8_dlc <= CAN_MAX_RAW_DLC) ? u.cf.len8_dlc : len;
		wire = len;
		w1 = dlc | (rtr ? MCPFD_OBJ_RTR : 0);
	}
	eff = (id & CAN_EFF_FLAG) != 0;
	if (eff) {
		id &= CAN_EFF_MASK;
		/* SID = 11 MSBs, EID = 18 LSBs of the 29-bit identifier. */
		w0 = ((id >> 18) & MCPFD_OBJ_SID_MASK) |
		    ((id & 0x3FFFF) << MCPFD_OBJ_EID_SHIFT);
		w1 |= MCPFD_OBJ_IDE;
	} else {
		w0 = id & MCPFD_OBJ_SID_MASK;
	}

	/* Wait for a free slot in the TX FIFO. */
	for (i = 0; i < MCPFD_TX_TIMEOUT; i++) {
		if ((mcpfd_read32(sc, C1FIFOSTA(MCPFD_TX_FIFO)) &
		    FIFOSTA_TFNRFNIF) != 0)
			break;
		DELAY(10);
	}
	if (i == MCPFD_TX_TIMEOUT) {
		if_inc_counter(sc->ifp, IFCOUNTER_OERRORS, 1);
		return;
	}

	addr = MCPFD_RAM_BASE + mcpfd_read32(sc, C1FIFOUA(MCPFD_TX_FIFO));
	le32enc(obj, w0);
	le32enc(obj + 4, w1);
	memset(obj + MCPFD_OBJ_HDRLEN, 0,
	    roundup2(wire, 4));			/* pad partial words */
	if (!rtr)
		memcpy(obj + MCPFD_OBJ_HDRLEN, payload, len);
	mcpfd_write_buf(sc, addr, obj, MCPFD_OBJ_HDRLEN + roundup2(wire, 4));

	/* Increment the FIFO and request transmission in one action write. */
	mcpfd_write8(sc, C1FIFOCON(MCPFD_TX_FIFO) + 1,
	    FIFOCON_B1_UINC | FIFOCON_B1_TXREQ);
}

/*
 * Read one frame from the RX FIFO into a fresh mbuf; NULL when the FIFO is
 * empty (or on allocation failure, in which case the frame is consumed and
 * counted so the drain loop makes progress either way).
 */
static struct mbuf *
mcpfd_hw_recv(struct mcp251xfd_softc *sc)
{
	uint8_t hdr[MCPFD_OBJ_HDRLEN], data[CANFD_MAX_DLEN];
	struct canfd_frame *cfd;
	struct can_frame *cf;
	struct mbuf *m;
	uint32_t w0, w1;
	uint16_t addr;
	canid_t id;
	u_int dlc, len;
	bool fdf, rtr;

	if ((mcpfd_read32(sc, C1FIFOSTA(MCPFD_RX_FIFO)) &
	    FIFOSTA_TFNRFNIF) == 0)
		return (NULL);

	addr = MCPFD_RAM_BASE + mcpfd_read32(sc, C1FIFOUA(MCPFD_RX_FIFO));
	mcpfd_read_buf(sc, addr, hdr, sizeof(hdr));
	w0 = le32dec(hdr);
	w1 = le32dec(hdr + 4);

	dlc = w1 & MCPFD_OBJ_DLC_MASK;
	fdf = (w1 & MCPFD_OBJ_FDF) != 0;
	rtr = !fdf && (w1 & MCPFD_OBJ_RTR) != 0;
	len = fdf ? mcpfd_dlc2len[dlc] :
	    (dlc > CAN_MAX_DLEN ? CAN_MAX_DLEN : dlc);
	if (len > 0 && !rtr)
		mcpfd_read_buf(sc, addr + MCPFD_OBJ_HDRLEN, data,
		    roundup2(len, 4));

	/* Free the FIFO slot before anything that can fail. */
	mcpfd_write8(sc, C1FIFOCON(MCPFD_RX_FIFO) + 1, FIFOCON_B1_UINC);

	if ((w1 & MCPFD_OBJ_IDE) != 0) {
		id = ((w0 & MCPFD_OBJ_SID_MASK) << 18) |
		    ((w0 >> MCPFD_OBJ_EID_SHIFT) & 0x3FFFF);
		id |= CAN_EFF_FLAG;
	} else {
		id = w0 & MCPFD_OBJ_SID_MASK;
	}
	if (rtr)
		id |= CAN_RTR_FLAG;

	m = m_gethdr(M_NOWAIT, MT_DATA);
	if (m == NULL) {
		if_inc_counter(sc->ifp, IFCOUNTER_IQDROPS, 1);
		return (NULL);
	}
	if (fdf) {
		cfd = mtod(m, struct canfd_frame *);
		memset(cfd, 0, sizeof(*cfd));
		cfd->can_id = id;
		cfd->len = len;
		cfd->flags = ((w1 & MCPFD_OBJ_BRS) ? CANFD_BRS : 0) |
		    ((w1 & MCPFD_OBJ_ESI) ? CANFD_ESI : 0);
		memcpy(cfd->data, data, len);
		m->m_len = m->m_pkthdr.len = sizeof(*cfd);
	} else {
		cf = mtod(m, struct can_frame *);
		memset(cf, 0, sizeof(*cf));
		cf->can_id = id;
		cf->len = len;
		if (dlc > CAN_MAX_DLC)
			cf->len8_dlc = dlc;
		if (!rtr)
			memcpy(cf->data, data, len);
		m->m_len = m->m_pkthdr.len = sizeof(*cf);
	}
	return (m);
}

/*
 * Deferred transmit: runs in the taskqueue thread (may sleep), drains the
 * software queue into the chip.
 */
static void
mcp251xfd_tx_task(void *arg, int pending __unused)
{
	struct mcp251xfd_softc *sc = arg;
	struct mbuf *m;

	for (;;) {
		mtx_lock(&sc->txlock);
		m = mbufq_dequeue(&sc->txq);
		mtx_unlock(&sc->txlock);
		if (m == NULL)
			break;
		sx_xlock(&sc->sx);
		if (sc->running) {
			mcpfd_hw_send(sc, m);
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
mcp251xfd_transmit(struct ifnet *ifp, struct mbuf *m)
{
	struct mcp251xfd_softc *sc = if_getsoftc(ifp);
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
mcp251xfd_qflush(struct ifnet *ifp)
{
	struct mcp251xfd_softc *sc = if_getsoftc(ifp);

	mtx_lock(&sc->txlock);
	mbufq_drain(&sc->txq);
	mtx_unlock(&sc->txlock);
}

/*
 * Hard interrupt filter.  Runs in primary interrupt context where SPI
 * transfers (which sleep) are not allowed, so it only schedules the task.
 */
static int
mcp251xfd_intr(void *arg)
{
	struct mcp251xfd_softc *sc = arg;

	taskqueue_enqueue(sc->tq, &sc->int_task);
	return (FILTER_HANDLED);
}

/*
 * Queue a CAN error message frame (netcan/can/error.h): a classic frame
 * with CAN_ERR_FLAG plus error classes in the id and 8 bytes of detail.
 * Delivered through can_input() like bus traffic; netcan hands it only to
 * sockets that opted in via CAN_RAW_ERR_FILTER.
 */
static void
mcpfd_err_enqueue(struct mcp251xfd_softc *sc, struct mbufq *q, canid_t id,
    const uint8_t *data)
{
	struct can_frame *cf;
	struct mbuf *m;

	m = m_gethdr(M_NOWAIT, MT_DATA);
	if (m == NULL)
		return;
	cf = mtod(m, struct can_frame *);
	memset(cf, 0, sizeof(*cf));
	cf->can_id = CAN_ERR_FLAG | id;
	cf->len = CAN_ERR_DLC;
	memcpy(cf->data, data, CAN_ERR_DLC);
	m->m_len = m->m_pkthdr.len = sizeof(*cf);
	if (mbufq_enqueue(q, m) != 0)
		m_freem(m);
}

/*
 * Deferred interrupt work, in taskqueue (kernel-thread) context where the sx
 * lock and SPI may sleep.  Drain C1INT until no handled flag remains set so
 * that events arriving while we run are not lost.
 */
static void
mcp251xfd_int_task(void *arg, int pending __unused)
{
	struct mcp251xfd_softc *sc = arg;
	struct mbufq rxq;
	struct mbuf *m;
	uint32_t ints, sta, ua, trec, bdiag;
	canid_t id;
	u_int head, queued;
	uint8_t d[CAN_ERR_DLC];
	int i;

	mbufq_init(&rxq, 4 * MCPFD_RX_DEPTH);
	for (;;) {
		sx_xlock(&sc->sx);
		if (!sc->running) {
			sx_xunlock(&sc->sx);
			break;
		}
		ints = mcpfd_read32(sc, C1INT);
		if ((ints & C1INT_HANDLED) == 0) {
			sx_xunlock(&sc->sx);
			break;
		}
		if (ints & C1INT_RXIF) {
			while ((m = mcpfd_hw_recv(sc)) != NULL) {
				if (mbufq_enqueue(&rxq, m) != 0) {
					m_freem(m);
					if_inc_counter(sc->ifp,
					    IFCOUNTER_IQDROPS, 1);
				} else {
					if_inc_counter(sc->ifp,
					    IFCOUNTER_IPACKETS, 1);
				}
			}
		}
		if (ints & C1INT_TXATIF) {
			/*
			 * TX attempts exhausted: the chip aborted the frame
			 * at the FIFO tail and stopped the FIFO (TXREQ
			 * cleared), but the aborted frame remains stored -
			 * FIFOCI stays put across aborts, and a later TXREQ
			 * would resend it with a fresh attempt budget.  For
			 * real limited-attempts semantics, flush the FIFO
			 * with FRESET; frames queued behind the aborted one
			 * were bound for the same dead bus and are dropped
			 * and counted too.  (The tail pointer is hardware-
			 * managed; single frames cannot be skipped.)
			 */
			sta = mcpfd_read32(sc, C1FIFOSTA(MCPFD_TX_FIFO));
			ua = mcpfd_read32(sc, C1FIFOUA(MCPFD_TX_FIFO));
			head = (ua / MCPFD_TX_OBJSZ) % MCPFD_TX_DEPTH;
			queued = (head + MCPFD_TX_DEPTH -
			    FIFOSTA_FIFOCI(sta)) % MCPFD_TX_DEPTH;
			if (queued == 0)	/* wrapped: full, not empty */
				queued = ((sta & FIFOSTA_TFERFFIF) == 0) ?
				    MCPFD_TX_DEPTH : 1;
			mcpfd_write8(sc, C1FIFOCON(MCPFD_TX_FIFO) + 1,
			    FIFOCON_B1_FRESET);
			for (i = 0; i < 100; i++) {
				if ((mcpfd_read32(sc,
				    C1FIFOCON(MCPFD_TX_FIFO)) &
				    FIFOCON_FRESET) == 0)
					break;
				DELAY(10);
			}
			mcpfd_write8(sc, C1FIFOSTA(MCPFD_TX_FIFO), 0);
			if (sc->txat_dbg < 3) {
				sc->txat_dbg++;
				device_printf(sc->dev,
				    "TX attempts exhausted, %u frame(s) "
				    "dropped (FIFOSTA=0x%08x)\n", queued, sta);
			}
			if_inc_counter(sc->ifp, IFCOUNTER_OERRORS, queued);
			memset(d, 0, sizeof(d));
			mcpfd_err_enqueue(sc, &rxq, CAN_ERR_ACK, d);
		}
		if (ints & C1INT_RXOVIF) {
			/* Clear RXOVIF in the FIFO status (write-0 bits). */
			mcpfd_write8(sc, C1FIFOSTA(MCPFD_RX_FIFO), 0);
			if_inc_counter(sc->ifp, IFCOUNTER_IQDROPS, 1);
			memset(d, 0, sizeof(d));
			d[1] = CAN_ERR_CRTL_RX_OVERFLOW;
			mcpfd_err_enqueue(sc, &rxq, CAN_ERR_CRTL, d);
		}
		if (ints & C1INT_CERRIF) {
			/* Error-state transition; report the new state. */
			trec = mcpfd_read32(sc, C1TREC);
			if_inc_counter(sc->ifp, IFCOUNTER_IERRORS, 1);
			memset(d, 0, sizeof(d));
			id = CAN_ERR_CRTL | CAN_ERR_CNT;
			if (trec & TREC_TXBO)
				id |= CAN_ERR_BUSOFF;
			if (trec & TREC_TXBP)
				d[1] |= CAN_ERR_CRTL_TX_PASSIVE;
			else if (trec & TREC_TXWARN)
				d[1] |= CAN_ERR_CRTL_TX_WARNING;
			if (trec & TREC_RXBP)
				d[1] |= CAN_ERR_CRTL_RX_PASSIVE;
			else if (trec & TREC_RXWARN)
				d[1] |= CAN_ERR_CRTL_RX_WARNING;
			if (d[1] == 0)
				d[1] = CAN_ERR_CRTL_ACTIVE;
			d[6] = TREC_TEC(trec);
			d[7] = TREC_REC(trec);
			mcpfd_err_enqueue(sc, &rxq, id, d);
		}
		if (ints & C1INT_IVMIF) {
			/* Per-bus-error detail; reported only with berr. */
			bdiag = mcpfd_read32(sc, C1BDIAG1);
			mcpfd_write32(sc, C1BDIAG1, 0);
			if_inc_counter(sc->ifp, IFCOUNTER_IERRORS, 1);
			if (sc->berr) {
				memset(d, 0, sizeof(d));
				id = CAN_ERR_BUSERROR | CAN_ERR_PROT;
				if (bdiag &
				    (BDIAG1_NBIT0ERR | BDIAG1_DBIT0ERR))
					d[2] |= CAN_ERR_PROT_BIT0;
				if (bdiag &
				    (BDIAG1_NBIT1ERR | BDIAG1_DBIT1ERR))
					d[2] |= CAN_ERR_PROT_BIT1;
				if (bdiag &
				    (BDIAG1_NFORMERR | BDIAG1_DFORMERR))
					d[2] |= CAN_ERR_PROT_FORM;
				if (bdiag &
				    (BDIAG1_NSTUFERR | BDIAG1_DSTUFERR))
					d[2] |= CAN_ERR_PROT_STUFF;
				if (bdiag &
				    (BDIAG1_NCRCERR | BDIAG1_DCRCERR))
					d[3] = CAN_ERR_PROT_LOC_CRC_SEQ;
				if (bdiag & BDIAG1_NACKERR) {
					id |= CAN_ERR_ACK;
					d[3] = CAN_ERR_PROT_LOC_ACK;
				}
				mcpfd_err_enqueue(sc, &rxq, id, d);
			}
		}
		if (ints & C1INT_CLEARABLE)
			/* Write-0-to-clear; leave unobserved flags alone. */
			mcpfd_write16(sc, C1INT,
			    (uint16_t)~(ints & C1INT_CLEARABLE));
		sx_xunlock(&sc->sx);

		/*
		 * can_input() touches the VNET-virtualized CAN pcb list, but
		 * this runs in a taskqueue thread with no vnet context set,
		 * so establish it to avoid a NULL curvnet dereference.
		 */
		while ((m = mbufq_dequeue(&rxq)) != NULL) {
			CURVNET_SET_QUIET(if_getvnet(sc->ifp));
			can_input(sc->ifp, m);
			CURVNET_RESTORE();
		}
	}
}

static int
mcp251xfd_up(struct mcp251xfd_softc *sc)
{
	int error;

	sx_xlock(&sc->sx);
	sc->txat_dbg = 0;
	error = mcpfd_configure(sc);
	if (error == 0 && sc->txattempts >= 0)
		device_printf(sc->dev,
		    "TX attempt limit %d: C1CON=0x%08x FIFOCON=0x%08x\n",
		    sc->txattempts, mcpfd_read32(sc, C1CON),
		    mcpfd_read32(sc, C1FIFOCON(MCPFD_TX_FIFO)));
	if (error == 0)
		error = mcpfd_set_mode(sc, sc->fd ?
		    MCPFD_MODE_NORMAL_FD : MCPFD_MODE_NORMAL_CAN20);
	if (error == 0) {
		sc->running = true;
		if_setdrvflagbits(sc->ifp, IFF_DRV_RUNNING, 0);
	}
	sx_xunlock(&sc->sx);
	return (error);
}

static void
mcp251xfd_down(struct mcp251xfd_softc *sc)
{

	sx_xlock(&sc->sx);
	sc->running = false;
	(void)mcpfd_set_mode(sc, MCPFD_MODE_CONFIG);
	if_setdrvflagbits(sc->ifp, 0, IFF_DRV_RUNNING);
	sx_xunlock(&sc->sx);
	mcp251xfd_qflush(sc->ifp);
}

static int
mcp251xfd_ioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
{
	struct mcp251xfd_softc *sc = if_getsoftc(ifp);
	struct ifreq *ifr = (struct ifreq *)data;
	bool fd;
	int error;

	switch (cmd) {
	case SIOCSIFFLAGS:
		if (if_getflags(ifp) & IFF_UP) {
			if (!sc->running)
				return (mcp251xfd_up(sc));
		} else {
			if (sc->running)
				mcp251xfd_down(sc);
		}
		return (0);
	case SIOCSIFMTU:
		/* MTU selects the frame format: 16 classic, 72 CAN FD. */
		if (ifr->ifr_mtu != (int)CAN_MTU &&
		    ifr->ifr_mtu != (int)CANFD_MTU)
			return (EINVAL);
		fd = (ifr->ifr_mtu == (int)CANFD_MTU);
		if (fd != sc->fd) {
			sc->fd = fd;
			if (sc->running) {
				mcp251xfd_down(sc);
				if ((error = mcp251xfd_up(sc)) != 0) {
					sc->fd = !fd;
					return (error);
				}
			}
		}
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
mcp251xfd_sysctl_bitrate(SYSCTL_HANDLER_ARGS)
{
	struct mcp251xfd_softc *sc = arg1;
	u_int brp, tseg1, tseg2, sjw;
	int error, val = sc->bitrate;

	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (val < 5 || val > 1000 ||
	    mcpfd_compute_bt(sc->sysclk, (u_int)val * 1000,
	    &mcpfd_nominal_limits, &brp, &tseg1, &tseg2, &sjw) != 0)
		return (EINVAL);
	sc->bitrate = val;
	if (sc->running) {
		mcp251xfd_down(sc);
		return (mcp251xfd_up(sc));
	}
	return (0);
}

static int
mcp251xfd_sysctl_dbitrate(SYSCTL_HANDLER_ARGS)
{
	struct mcp251xfd_softc *sc = arg1;
	u_int brp, tseg1, tseg2, sjw;
	int error, val = sc->dbitrate;

	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (val < sc->bitrate || val > 8000 ||
	    mcpfd_compute_bt(sc->sysclk, (u_int)val * 1000,
	    &mcpfd_data_limits, &brp, &tseg1, &tseg2, &sjw) != 0)
		return (EINVAL);
	sc->dbitrate = val;
	if (sc->running) {
		mcp251xfd_down(sc);
		return (mcp251xfd_up(sc));
	}
	return (0);
}

/*
 * TX retransmission limit: -1 = unlimited (the ISO 11898 default), 0 =
 * one-shot (no retransmission), 3 = three attempts; these are the only
 * modes the silicon encodes.  Exhausted attempts drop the frame, count an
 * output error and emit a CAN_ERR_ACK error frame.
 */
static int
mcp251xfd_sysctl_txattempts(SYSCTL_HANDLER_ARGS)
{
	struct mcp251xfd_softc *sc = arg1;
	int error, val = sc->txattempts;

	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (val != -1 && val != 0 && val != 3)
		return (EINVAL);
	sc->txattempts = val;
	if (sc->running) {
		mcp251xfd_down(sc);
		return (mcp251xfd_up(sc));
	}
	return (0);
}

/* Bus-error reporting: emit a CAN_ERR_BUSERROR frame per bus error. */
static int
mcp251xfd_sysctl_berr(SYSCTL_HANDLER_ARGS)
{
	struct mcp251xfd_softc *sc = arg1;
	int error, val = sc->berr ? 1 : 0;

	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	sc->berr = (val != 0);
	if (sc->running) {
		mcp251xfd_down(sc);
		return (mcp251xfd_up(sc));
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
mcp251xfd_delayed_attach(void *arg)
{
	struct mcp251xfd_softc *sc = arg;
	int error;

	/* Probe the chip: full configuration exercises reset, OSC and RAM. */
	sx_xlock(&sc->sx);
	error = mcpfd_configure(sc);
	sx_xunlock(&sc->sx);
	if (error != 0) {
		device_printf(sc->dev, "MCP251xFD not responding\n");
		return;
	}

	/* Publish the interface; nothing below this can fail. */
	if_attach(sc->ifp);
	bpfattach(sc->ifp, DLT_CAN_SOCKETCAN, 0);
	sc->published = true;

	device_printf(sc->dev,
	    "%s: MCP251xFD @ %u MHz xtal, default %d/%d kbit/s\n",
	    if_name(sc->ifp), sc->fosc / 1000000, sc->bitrate, sc->dbitrate);
}

static int
mcp251xfd_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "Microchip MCP251xFD CAN FD controller");
	return (BUS_PROBE_DEFAULT);
}

static int
mcp251xfd_attach(device_t dev)
{
	struct mcp251xfd_softc *sc = device_get_softc(dev);
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *tree;
	struct ifnet *ifp;
	phandle_t node;
	uint32_t freq;
	int error;

	sc->dev = dev;
	sc->parent = device_get_parent(dev);
	sc->bitrate = 500;
	sc->dbitrate = 2000;
	sc->txattempts = -1;	/* unlimited: standard CAN semantics */
	sc->fosc = 40000000;
	node = ofw_bus_get_node(dev);
	if (OF_getencprop(node, "clock-frequency", &freq, sizeof(freq)) > 0)
		sc->fosc = freq;
	if (sc->fosc < 4000000 || sc->fosc > 40000000) {
		device_printf(dev, "oscillator %u Hz out of range (4-40 MHz)\n",
		    sc->fosc);
		return (ENXIO);
	}
	/* The x10 PLL is specified for a 4 MHz oscillator input only. */
	sc->sysclk = (sc->fosc == 4000000) ? 40000000 : sc->fosc;

	sx_init(&sc->sx, "mcp251xfd");
	mtx_init(&sc->txlock, "mcp251xfd txq", NULL, MTX_DEF);
	mbufq_init(&sc->txq, 64);
	TASK_INIT(&sc->tx_task, 0, mcp251xfd_tx_task, sc);
	TASK_INIT(&sc->int_task, 0, mcp251xfd_int_task, sc);
	/*
	 * A "fast" taskqueue: its enqueue lock is a spin mutex, so the
	 * interrupt filter may enqueue work from primary interrupt context.
	 * The worker thread still runs tasks in a sleepable context (SPI).
	 */
	sc->tq = taskqueue_create_fast("mcp251xfd taskq", M_WAITOK,
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
	if_setmtu(ifp, CAN_MTU);		/* classic until mtu 72 */
	if_setflags(ifp, IFF_SIMPLEX);		/* not IP-capable */
	if_settransmitfn(ifp, mcp251xfd_transmit);
	if_setqflushfn(ifp, mcp251xfd_qflush);
	if_setioctlfn(ifp, mcp251xfd_ioctl);
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
	    mcp251xfd_intr, NULL, sc, &sc->irq_cookie);
	if (error != 0) {
		device_printf(dev, "cannot set up interrupt\n");
		goto fail_irq;
	}

	ctx = device_get_sysctl_ctx(dev);
	tree = device_get_sysctl_tree(dev);
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "bitrate",
	    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_NEEDGIANT, sc, 0,
	    mcp251xfd_sysctl_bitrate, "I", "CAN nominal bitrate (kbit/s)");
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "dbitrate",
	    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_NEEDGIANT, sc, 0,
	    mcp251xfd_sysctl_dbitrate, "I", "CAN FD data bitrate (kbit/s)");
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "txattempts",
	    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_NEEDGIANT, sc, 0,
	    mcp251xfd_sysctl_txattempts, "I",
	    "TX attempts (-1 unlimited, 0 one-shot, 3 attempts)");
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "berr_reporting",
	    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_NEEDGIANT, sc, 0,
	    mcp251xfd_sysctl_berr, "I",
	    "emit an error frame per bus error (may flood)");

	/*
	 * Probe the chip and publish the interface from a config_intrhook:
	 * SPI transfers sleep, which panics in device_attach() during a cold
	 * boot (before timers work) when the driver is compiled in or
	 * preloaded.  See mcp251xfd_delayed_attach().
	 */
	config_intrhook_oneshot(mcp251xfd_delayed_attach, sc);
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
mcp251xfd_detach(device_t dev)
{
	struct mcp251xfd_softc *sc = device_get_softc(dev);

	if (sc->running)
		mcp251xfd_down(sc);
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

static device_method_t mcp251xfd_methods[] = {
	DEVMETHOD(device_probe,		mcp251xfd_probe),
	DEVMETHOD(device_attach,	mcp251xfd_attach),
	DEVMETHOD(device_detach,	mcp251xfd_detach),
	DEVMETHOD_END
};

static driver_t mcp251xfd_driver = {
	"mcp251xfd",
	mcp251xfd_methods,
	sizeof(struct mcp251xfd_softc),
};

DRIVER_MODULE(mcp251xfd, spibus, mcp251xfd_driver, NULL, NULL);
MODULE_DEPEND(mcp251xfd, spibus, 1, 1, 1);
MODULE_DEPEND(mcp251xfd, can, 1, 1, 1);
SPIBUS_FDT_PNP_INFO(compat_data);
