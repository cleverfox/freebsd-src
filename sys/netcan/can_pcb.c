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
 * CAN protocol control blocks.  Derived from NetBSD netcan/can_pcb.c, adapted
 * to the FreeBSD socket framework.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>
#include <sys/mutex.h>
#include <sys/protosw.h>
#include <sys/socket.h>
#include <sys/socketvar.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/vnet.h>

#include <netcan/can.h>
#include <netcan/can/raw.h>
#include <netcan/can_var.h>
#include <netcan/can_pcb.h>

MALLOC_DEFINE(M_CANPCB, "canpcb", "CAN protocol control block");

VNET_DEFINE(struct canpcbhead, canpcb_list);

struct mtx can_pcb_mtx;
MTX_SYSINIT(can_pcb, &can_pcb_mtx, "CAN pcb list", MTX_DEF);

/* Global count of live control blocks, used to gate module unload. */
int can_pcb_count;

void
can_pcb_init(void)
{

	LIST_INIT(&V_canpcb_list);
}

void
can_pcb_destroy(void)
{

	/* List must be empty; the domain refuses unload while pcbs exist. */
	KASSERT(LIST_EMPTY(&V_canpcb_list),
	    ("%s: canpcb list not empty", __func__));
}

int
can_pcballoc(struct socket *so)
{
	struct canpcb *canp;
	struct can_filter *filt;

	canp = malloc(sizeof(*canp), M_CANPCB, M_NOWAIT | M_ZERO);
	if (canp == NULL)
		return (ENOBUFS);

	/* Default acceptance filter: a single match-all entry {0, 0}. */
	filt = malloc(sizeof(*filt), M_CANPCB, M_NOWAIT | M_ZERO);
	if (filt == NULL) {
		free(canp, M_CANPCB);
		return (ENOBUFS);
	}

	canp->canp_socket = so;
	canp->canp_filters = filt;
	canp->canp_nfilters = 1;
	/* Linux defaults: loopback ON, receive-own OFF. */

	so->so_pcb = canp;

	CANP_LIST_LOCK();
	LIST_INSERT_HEAD(&V_canpcb_list, canp, canp_list);
	can_pcb_count++;
	CANP_LIST_UNLOCK();

	return (0);
}

void
can_pcbdetach(struct canpcb *canp)
{
	struct socket *so = canp->canp_socket;

	CANP_LIST_LOCK();
	LIST_REMOVE(canp, canp_list);
	can_pcb_count--;
	CANP_LIST_UNLOCK();

	so->so_pcb = NULL;
	if (canp->canp_filters != NULL)
		free(canp->canp_filters, M_CANPCB);
	free(canp, M_CANPCB);
}

int
can_pcbbind(struct canpcb *canp, struct sockaddr_can *scan)
{
	struct ifnet *ifp;

	if (scan->can_family != AF_CAN)
		return (EAFNOSUPPORT);
	if (scan->can_len != sizeof(*scan))
		return (EINVAL);

	if (scan->can_ifindex != 0) {
		struct epoch_tracker et;

		NET_EPOCH_ENTER(et);
		ifp = ifnet_byindex(scan->can_ifindex);
		NET_EPOCH_EXIT(et);
		if (ifp == NULL)
			return (ENODEV);
		canp->canp_ifindex = scan->can_ifindex;
		/*
		 * Binding to a specific interface gives the socket a default
		 * destination, so mark it connected; this lets send(2)/write(2)
		 * be used without supplying an address (Linux semantics).
		 */
		soisconnected(canp->canp_socket);
	} else {
		/* ifindex 0: receive from / send default to all interfaces. */
		canp->canp_ifindex = 0;
		canp->canp_socket->so_state &= ~SS_ISCONNECTED;
	}
	return (0);
}

void
can_pcbdisconnect(struct canpcb *canp)
{

	canp->canp_ifindex = 0;
}

void
can_setsockaddr(struct canpcb *canp, struct sockaddr_can *scan)
{

	memset(scan, 0, sizeof(*scan));
	scan->can_family = AF_CAN;
	scan->can_len = sizeof(*scan);
	scan->can_ifindex = canp->canp_ifindex;
}

/*
 * Replace the socket's acceptance filter set, install-before-remove so a
 * failed allocation leaves the existing filter intact (Linux semantics).
 * nfilters == 0 is legal and means "receive nothing".
 */
int
can_pcbsetfilter(struct canpcb *canp, struct can_filter *fp, int nfilters)
{
	struct can_filter *newf, *oldf;

	if (nfilters < 0 || nfilters > CAN_RAW_FILTER_MAX)
		return (EINVAL);

	if (nfilters > 0) {
		newf = malloc(sizeof(*newf) * nfilters, M_CANPCB, M_NOWAIT);
		if (newf == NULL)
			return (ENOBUFS);
		memcpy(newf, fp, sizeof(*newf) * nfilters);
	} else {
		newf = NULL;
	}

	/* Swap atomically with respect to the receive walk. */
	CANP_LIST_LOCK();
	oldf = canp->canp_filters;
	canp->canp_filters = newf;
	canp->canp_nfilters = nfilters;
	CANP_LIST_UNLOCK();

	if (oldf != NULL)
		free(oldf, M_CANPCB);
	return (0);
}

/*
 * Match a single filter against an incoming CAN id.  The CAN_INV_FILTER bit
 * (bit 29, shared with CAN_ERR_FLAG) is a filter directive, not an id bit, so
 * it is stripped from the stored can_id before the masked comparison and then
 * used to optionally invert the result.
 */
static __inline bool
can_filter_match(const struct can_filter *f, canid_t id)
{
	canid_t fid = f->can_id & ~CAN_INV_FILTER;
	bool match = ((id & f->can_mask) == (fid & f->can_mask));

	if (f->can_id & CAN_INV_FILTER)
		match = !match;
	return (match);
}

/*
 * Test an incoming frame against the socket's acceptance filters.
 * Returns true if the frame should be delivered.
 *
 * Match formula (per filter): (id & mask) == (can_id & mask), optionally
 * inverted by CAN_INV_FILTER in the filter's can_id.  Error message frames
 * (CAN_ERR_FLAG set) are matched only against canp_err_mask.  Multiple data
 * filters are OR-combined, unless CANP_JOIN_FILTERS requests AND.
 *
 * Caller must hold CANP_LIST_LOCK (filters are swapped under it).
 */
bool
can_pcbfilter(struct canpcb *canp, struct mbuf *m)
{
	struct can_frame *cf;
	struct can_filter *f;
	canid_t id;
	int i;

	CANP_LIST_LOCK_ASSERT();
	KASSERT((m->m_flags & M_PKTHDR) != 0,
	    ("%s: no pkthdr", __func__));

	cf = mtod(m, struct can_frame *);
	id = cf->can_id;

	/* Error message frames go solely through the error filter. */
	if (id & CAN_ERR_FLAG)
		return ((id & canp->canp_err_mask) != 0);

	if (canp->canp_nfilters == 0)
		return (false);

	if (canp->canp_flags & CANP_JOIN_FILTERS) {
		/* AND: every filter must match. */
		for (i = 0; i < canp->canp_nfilters; i++) {
			f = &canp->canp_filters[i];
			if (!can_filter_match(f, id))
				return (false);
		}
		return (true);
	}

	/* OR: any filter matching accepts. */
	for (i = 0; i < canp->canp_nfilters; i++) {
		f = &canp->canp_filters[i];
		if (can_filter_match(f, id))
			return (true);
	}
	return (false);
}
