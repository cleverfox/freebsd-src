/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2002-2007 Volkswagen Group Electronic Research
 * All rights reserved.
 *
 * Copyright (c) 2026 Vladimir Goncharov <devel@viruzzz.org>
 *
 * Authors: Oliver Hartkopp <oliver.hartkopp@volkswagen.de>
 *          Urs Thuermann   <urs.thuermann@volkswagen.de>
 *
 * Derived from the Linux SocketCAN UAPI headers, used under the BSD-3-Clause
 * option of their dual (GPL-2.0-only OR BSD-3-Clause) license.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of Volkswagen nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Definitions of the CAN netlink interface, kept compatible with Linux
 * (include/uapi/linux/can/netlink.h).  These describe the rtnetlink link-type
 * ("can") attributes used to configure a CAN controller's bit timing, control
 * modes and to query its state -- the interface used by iproute2's `ip link set
 * canX type can ...` and by libsocketcan.
 *
 * Virtual interfaces (vcan) carry none of these attributes; they are reported
 * with link kind "vcan" and managed with the generic link operations.  A real
 * CAN controller driver fills/parses these attributes from its netlink
 * dump/modify handlers.
 */

#ifndef _NETCAN_CAN_NETLINK_H_
#define _NETCAN_CAN_NETLINK_H_

#include <sys/types.h>

/*
 * CAN bit-timing parameters
 *
 * For further information, please read chapter "8 BIT TIMING REQUIREMENTS" of
 * the "Bosch CAN Specification version 2.0".
 */
struct can_bittiming {
	uint32_t	bitrate;	/* bit-rate in bits/second */
	uint32_t	sample_point;	/* sample point in 0.1% units */
	uint32_t	tq;		/* time quanta (TQ) in nanoseconds */
	uint32_t	prop_seg;	/* propagation segment in TQs */
	uint32_t	phase_seg1;	/* phase buffer segment 1 in TQs */
	uint32_t	phase_seg2;	/* phase buffer segment 2 in TQs */
	uint32_t	sjw;		/* synchronisation jump width, TQs */
	uint32_t	brp;		/* bit-rate prescaler */
};

/*
 * CAN hardware-dependent bit-timing constant
 *
 * Used for calculating and checking bit-timing parameters
 */
struct can_bittiming_const {
	char		name[16];	/* name of the CAN controller */
	uint32_t	tseg1_min;	/* TSEG1 = prop_seg + phase_seg1 */
	uint32_t	tseg1_max;
	uint32_t	tseg2_min;	/* TSEG2 = phase_seg2 */
	uint32_t	tseg2_max;
	uint32_t	sjw_max;	/* synchronisation jump width */
	uint32_t	brp_min;	/* bit-rate prescaler */
	uint32_t	brp_max;
	uint32_t	brp_inc;
};

/* CAN clock parameters */
struct can_clock {
	uint32_t	freq;		/* CAN system clock frequency, Hz */
};

/* CAN operational and error states */
enum can_state {
	CAN_STATE_ERROR_ACTIVE = 0,	/* RX/TX error count < 96 */
	CAN_STATE_ERROR_WARNING,	/* RX/TX error count < 128 */
	CAN_STATE_ERROR_PASSIVE,	/* RX/TX error count < 256 */
	CAN_STATE_BUS_OFF,		/* RX/TX error count >= 256 */
	CAN_STATE_STOPPED,		/* Device is stopped */
	CAN_STATE_SLEEPING,		/* Device is sleeping */
	CAN_STATE_MAX
};

/* CAN bus error counters */
struct can_berr_counter {
	uint16_t	txerr;
	uint16_t	rxerr;
};

/* CAN controller mode */
struct can_ctrlmode {
	uint32_t	mask;
	uint32_t	flags;
};

#define CAN_CTRLMODE_LOOPBACK		0x001	/* Loopback mode */
#define CAN_CTRLMODE_LISTENONLY		0x002	/* Listen-only mode */
#define CAN_CTRLMODE_3_SAMPLES		0x004	/* Triple sampling mode */
#define CAN_CTRLMODE_ONE_SHOT		0x008	/* One-Shot mode */
#define CAN_CTRLMODE_BERR_REPORTING	0x010	/* Bus-error reporting */
#define CAN_CTRLMODE_FD			0x020	/* CAN FD mode */
#define CAN_CTRLMODE_PRESUME_ACK	0x040	/* Ignore missing CAN ACKs */
#define CAN_CTRLMODE_FD_NON_ISO		0x080	/* CAN FD in non-ISO mode */
#define CAN_CTRLMODE_CC_LEN8_DLC	0x100	/* Classic CAN DLC option */
#define CAN_CTRLMODE_TDC_AUTO		0x200	/* CAN transiver auto TDC */
#define CAN_CTRLMODE_TDC_MANUAL		0x400	/* CAN transiver man. TDC */

/* CAN device statistics */
struct can_device_stats {
	uint32_t	bus_error;	/* Bus errors */
	uint32_t	error_warning;	/* Changes to error warning state */
	uint32_t	error_passive;	/* Changes to error passive state */
	uint32_t	bus_off;	/* Changes to bus off state */
	uint32_t	arbitration_lost;	/* Arbitration lost errors */
	uint32_t	restarts;	/* CAN controller re-starts */
};

/* CAN netlink interface (IFLA_INFO_DATA for link kind "can") */
enum {
	IFLA_CAN_UNSPEC,
	IFLA_CAN_BITTIMING,
	IFLA_CAN_BITTIMING_CONST,
	IFLA_CAN_CLOCK,
	IFLA_CAN_STATE,
	IFLA_CAN_CTRLMODE,
	IFLA_CAN_RESTART,
	IFLA_CAN_RESTART_MS,
	IFLA_CAN_BERR_COUNTER,
	IFLA_CAN_DATA_BITTIMING,
	IFLA_CAN_DATA_BITTIMING_CONST,
	IFLA_CAN_TERMINATION,
	IFLA_CAN_TERMINATION_CONST,
	IFLA_CAN_BITRATE_CONST,
	IFLA_CAN_DATA_BITRATE_CONST,
	IFLA_CAN_BITRATE_MAX,
	IFLA_CAN_TDC,
	IFLA_CAN_CTRLMODE_EXT,
	__IFLA_CAN_MAX
};
#define IFLA_CAN_MAX	(__IFLA_CAN_MAX - 1)

/* u16 termination range and a terminating value (matches Linux) */
#define CAN_TERMINATION_DISABLED	0

#endif /* !_NETCAN_CAN_NETLINK_H_ */
