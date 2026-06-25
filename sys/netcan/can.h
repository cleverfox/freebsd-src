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
 * Definitions for the CAN network layer (socket address, CAN frame, CAN
 * filter).  Kept binary- and source-compatible with Linux SocketCAN
 * (include/uapi/linux/can.h) as much as the BSD sockaddr convention allows.
 *
 * The classical CAN frame, the CAN id encoding, the filter structure and all
 * flag/mask macros are identical to Linux.  The only intentional divergence is
 * the leading sa_len byte in struct sockaddr_can, which is mandated by the BSD
 * socket framework and absent on Linux.  Portable applications must memset the
 * address and assign the named fields, in which case the layout is compatible
 * (can_ifindex lands at offset 4 on both systems).
 */

#ifndef _NETCAN_CAN_H_
#define _NETCAN_CAN_H_

#include <sys/types.h>
#include <sys/socket.h>

/* special address description flags for the CAN_ID */
#define CAN_EFF_FLAG	0x80000000U	/* EFF/SFF is set in the MSB */
#define CAN_RTR_FLAG	0x40000000U	/* remote transmission request */
#define CAN_ERR_FLAG	0x20000000U	/* error message frame */

/* valid bits in CAN ID for frame formats */
#define CAN_SFF_MASK	0x000007FFU	/* standard frame format (SFF) */
#define CAN_EFF_MASK	0x1FFFFFFFU	/* extended frame format (EFF) */
#define CAN_ERR_MASK	0x1FFFFFFFU	/* omit EFF, RTR, ERR flags */

/*
 * Controller Area Network Identifier structure
 *
 * bit 0-28	: CAN identifier (11/29 bit)
 * bit 29	: error message frame flag (0 = data frame, 1 = error message)
 * bit 30	: remote transmission request flag (1 = rtr frame)
 * bit 31	: frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
 */
typedef uint32_t canid_t;

#define CAN_SFF_ID_BITS	11
#define CAN_EFF_ID_BITS	29

/* error class mask, see <netcan/can/error.h> */
typedef uint32_t can_err_mask_t;

/* CAN payload length and DLC definitions according to ISO 11898-1 */
#define CAN_MAX_DLC	8
#define CAN_MAX_RAW_DLC	15
#define CAN_MAX_DLEN	8

/* CAN FD payload length and DLC definitions according to ISO 11898-7 */
#define CANFD_MAX_DLC	15
#define CANFD_MAX_DLEN	64

/**
 * struct can_frame - Classical CAN frame structure (aka CAN 2.0B)
 * @can_id:   CAN ID of the frame and CAN_*_FLAG flags, see canid_t definition
 * @len:      CAN frame payload length in byte (0 .. 8)
 * @can_dlc:  deprecated name for CAN frame payload length in byte (0 .. 8)
 * @len8_dlc: optional DLC value (9 .. 15) at 8 byte payload length
 * @data:     CAN frame payload (up to 8 byte)
 */
struct can_frame {
	canid_t	can_id;		/* 32 bit CAN_ID + EFF/RTR/ERR flags */
	union {
		uint8_t	len;	/* frame payload length in byte */
		uint8_t	can_dlc; /* deprecated */
	} __packed;
	uint8_t	__pad;		/* padding */
	uint8_t	__res0;		/* reserved / padding */
	uint8_t	len8_dlc;	/* optional DLC for 8 byte payload length */
	uint8_t	data[CAN_MAX_DLEN] __aligned(8);
};

/*
 * defined bits for canfd_frame.flags
 */
#define CANFD_BRS	0x01	/* bit rate switch (2nd bitrate for payload) */
#define CANFD_ESI	0x02	/* error state indicator of the transmitter */
#define CANFD_FDF	0x04	/* mark CAN FD for dual use of canfd_frame */

/**
 * struct canfd_frame - CAN flexible data rate frame structure
 *
 * Provided for source compatibility with Linux.  CAN FD is not yet handled by
 * the FreeBSD CAN stack; sockets reject FD-sized frames until support lands.
 */
struct canfd_frame {
	canid_t	can_id;		/* 32 bit CAN_ID + EFF/RTR/ERR flags */
	uint8_t	len;		/* frame payload length in byte */
	uint8_t	flags;		/* additional flags for CAN FD */
	uint8_t	__res0;		/* reserved / padding */
	uint8_t	__res1;		/* reserved / padding */
	uint8_t	data[CANFD_MAX_DLEN] __aligned(8);
};

#define CAN_MTU		(sizeof(struct can_frame))
#define CANFD_MTU	(sizeof(struct canfd_frame))

/* particular protocols of the protocol family PF_CAN */
#define CAN_RAW		1	/* RAW sockets */
#define CAN_BCM		2	/* Broadcast Manager */
#define CAN_TP16	3	/* VAG Transport Protocol v1.6 */
#define CAN_TP20	4	/* VAG Transport Protocol v2.0 */
#define CAN_MCNET	5	/* Bosch MCNet */
#define CAN_ISOTP	6	/* ISO 15765-2 Transport Protocol */
#define CAN_J1939	7	/* SAE J1939 */
#define CAN_NPROTO	8

#define SOL_CAN_BASE	100

/**
 * struct sockaddr_can - the sockaddr structure for CAN sockets
 * @can_len:     BSD sockaddr length (not present on Linux)
 * @can_family:  address family number AF_CAN
 * @can_ifindex: CAN network interface index (0 = bind to all interfaces)
 * @can_addr:    protocol specific address information
 */
struct sockaddr_can {
	uint8_t		can_len;
	sa_family_t	can_family;
	int		can_ifindex;
	union {
		/* transport protocol class address information (e.g. ISOTP) */
		struct { canid_t rx_id, tx_id; } tp;

		/* J1939 address information */
		struct {
			uint64_t	name;
			uint32_t	pgn;
			uint8_t		addr;
		} j1939;

		/* reserved for future CAN protocols address information */
	} can_addr;
};

/**
 * struct can_filter - CAN ID based filter in can_register().
 * @can_id:   relevant bits of CAN ID which are not masked out.
 * @can_mask: CAN mask (see description)
 *
 * A filter matches, when
 *
 *          <received_can_id> & can_mask == can_id & can_mask
 *
 * The filter can be inverted (CAN_INV_FILTER bit set in can_id) or it can
 * filter for error message frames (CAN_ERR_FLAG bit set in can_mask).
 */
struct can_filter {
	canid_t	can_id;
	canid_t	can_mask;
};

#define CAN_INV_FILTER	0x20000000U	/* to be set in can_filter.can_id */

#ifdef _KERNEL
#define	satoscan(sa)	((struct sockaddr_can *)(sa))
#define	scantosa(scan)	((struct sockaddr *)(scan))
#endif /* _KERNEL */

#endif /* !_NETCAN_CAN_H_ */
