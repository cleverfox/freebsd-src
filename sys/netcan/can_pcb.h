/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2003, 2017 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * Copyright (c) 2026 Vladimir Goncharov <devel@viruzzz.org>
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Robert Swindells and Manuel Bouyer.
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
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * CAN protocol control block.  Derived from NetBSD's netcan/can_pcb.h but
 * adapted to the FreeBSD socket framework: a single VNET-virtualized list of
 * control blocks protected by one global mutex (the rtsock.c model), no
 * private per-pcb hash tables, and lifetime tied to the socket.
 */

#ifndef _NETCAN_CAN_PCB_H_
#define _NETCAN_CAN_PCB_H_

#include <sys/queue.h>

#ifdef _KERNEL
#include <sys/lock.h>
#include <sys/mutex.h>
#include <net/vnet.h>

struct canpcb {
	LIST_ENTRY(canpcb) canp_list;	/* list of all CAN pcbs */
	struct socket	*canp_socket;	/* back pointer to socket */
	int		canp_ifindex;	/* bound interface index (0 = all) */
	int		canp_flags;	/* see below */
	struct can_filter *canp_filters; /* RX acceptance filter array */
	int		canp_nfilters;	/* number of entries in canp_filters */
	can_err_mask_t	canp_err_mask;	/* error frame filter (0 = none) */
};

LIST_HEAD(canpcbhead, canpcb);

/* flags in canp_flags */
#define CANP_NO_LOOPBACK	0x0001	/* local loopback disabled */
#define CANP_RECEIVE_OWN	0x0002	/* receive own messages */
#define CANP_JOIN_FILTERS	0x0004	/* AND all filters instead of OR */
#define CANP_FD_FRAMES		0x0008	/* allow CAN FD frames (tx and rx) */

#define	sotocanpcb(so)		((struct canpcb *)(so)->so_pcb)

/*
 * The global table of CAN control blocks is virtualized per network stack.
 * A single global mutex (not virtualized) serializes list mutation and the
 * receive-side fan-out walk, mirroring rtsock.c's rtsock_mtx + V_route_cb.
 */
VNET_DECLARE(struct canpcbhead, canpcb_list);
#define	V_canpcb_list		VNET(canpcb_list)

extern struct mtx can_pcb_mtx;
extern int can_pcb_count;	/* live pcbs; gates module unload */
#define	CANP_LIST_LOCK()	mtx_lock(&can_pcb_mtx)
#define	CANP_LIST_UNLOCK()	mtx_unlock(&can_pcb_mtx)
#define	CANP_LIST_LOCK_ASSERT()	mtx_assert(&can_pcb_mtx, MA_OWNED)

int	can_pcballoc(struct socket *);
void	can_pcbdetach(struct canpcb *);
int	can_pcbbind(struct canpcb *, struct sockaddr_can *);
void	can_pcbdisconnect(struct canpcb *);
void	can_setsockaddr(struct canpcb *, struct sockaddr_can *);
int	can_pcbsetfilter(struct canpcb *, struct can_filter *, int);
bool	can_pcbfilter(struct canpcb *, struct mbuf *);

#endif /* _KERNEL */

#endif /* !_NETCAN_CAN_PCB_H_ */
