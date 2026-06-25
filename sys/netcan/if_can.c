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
 * CAN protocol input path: receive-side fan-out to all matching sockets.
 * Derived from NetBSD netcan/can.c (canintr), rewritten to the FreeBSD
 * control-block list / sockbuf model (the rtsock.c rts_input idiom).
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/lock.h>
#include <sys/mbuf.h>
#include <sys/mutex.h>
#include <sys/socket.h>
#include <sys/socketvar.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/bpf.h>
#include <net/route.h>
#include <net/vnet.h>

#include <netcan/can.h>
#include <netcan/can_var.h>
#include <netcan/can_pcb.h>

/*
 * Tap a frame to BPF listeners.  SocketCAN BPF (DLT_CAN_SOCKETCAN) expects the
 * CAN id in network byte order, so byte-swap around the tap and restore.
 * Caller guarantees the can_frame header is contiguous in the first mbuf.
 */
void
can_bpf_mtap(struct ifnet *ifp, struct mbuf *m)
{
	struct can_frame *cf;
	canid_t oid;

	if (!bpf_peers_present_if(ifp))
		return;
	cf = mtod(m, struct can_frame *);
	oid = cf->can_id;
	cf->can_id = htonl(oid);
	bpf_mtap_if(ifp, m);
	cf->can_id = oid;
}

/*
 * if_output for CAN interfaces.  A CAN interface is not an IP-capable link: it
 * carries only CAN frames, submitted via if_transmit by the protocol's send
 * path.  Reject any layer-3 output attempt (e.g. an IPv6 ND solicitation from
 * a stray auto-configured link-local address) rather than leaving if_output
 * NULL, which the IP stack would call and panic on.
 */
int
can_if_output(struct ifnet *ifp, struct mbuf *m, const struct sockaddr *dst,
    struct route *ro)
{

	m_freem(m);
	return (EAFNOSUPPORT);
}

static void
can_append(struct canpcb *canp, struct sockaddr_can *from, struct mbuf *m)
{
	struct socket *so = canp->canp_socket;

	if (sbappendaddr(&so->so_rcv, (struct sockaddr *)from, m, NULL) == 0) {
		soroverflow(so);
		m_freem(m);
	} else
		sorwakeup(so);
}

/*
 * Deliver a CAN frame to every socket whose binding and acceptance filter
 * accept it.  Consumes m.  Used both for frames received from the wire
 * (sender == NULL) and for the local loopback echo of a transmitted frame
 * (sender == the originating pcb, which is skipped unless it set
 * CAN_RAW_RECV_OWN_MSGS).  This does NOT tap BPF -- callers tap once on the
 * TX (can_output) or RX (can_input) side so each frame is captured once.
 *
 * The caller must run in the interface's network stack (CURVNET) so
 * V_canpcb_list refers to the right instance.
 */
void
can_deliver(struct ifnet *ifp, struct mbuf *m, struct canpcb *sender)
{
	struct sockaddr_can from;
	struct canpcb *canp, *last;
	int rcv_ifindex;
	bool is_fd;

	/* Need the frame header contiguous for filtering. */
	if (m->m_len < (int)sizeof(struct can_frame)) {
		m = m_pullup(m, sizeof(struct can_frame));
		if (m == NULL)
			return;
	}

	/*
	 * Frame type is determined by length: anything larger than a classic
	 * frame is CAN FD.  Mark FD frames with CANFD_FDF as Linux does, and
	 * deliver them only to sockets that requested CAN_RAW_FD_FRAMES.
	 */
	is_fd = (m->m_pkthdr.len > (int)CAN_MTU);
	if (is_fd)
		mtod(m, struct canfd_frame *)->flags |= CANFD_FDF;

	rcv_ifindex = if_getindex(ifp);

	memset(&from, 0, sizeof(from));
	from.can_len = sizeof(from);
	from.can_family = AF_CAN;
	from.can_ifindex = rcv_ifindex;

	last = NULL;
	CANP_LIST_LOCK();
	LIST_FOREACH(canp, &V_canpcb_list, canp_list) {
		struct mbuf *mc;

		/* Bound to a specific interface? */
		if (canp->canp_ifindex != 0 &&
		    canp->canp_ifindex != rcv_ifindex)
			continue;
		/* FD frames go only to sockets that opted in. */
		if (is_fd && (canp->canp_flags & CANP_FD_FRAMES) == 0)
			continue;
		/* Don't echo to the sender unless it asked for it. */
		if (canp == sender &&
		    (canp->canp_flags & CANP_RECEIVE_OWN) == 0)
			continue;
		/* Acceptance filter. */
		if (!can_pcbfilter(canp, m))
			continue;

		if (last != NULL) {
			mc = m_copym(m, 0, M_COPYALL, M_NOWAIT);
			if (mc != NULL)
				can_append(last, &from, mc);
		}
		last = canp;
	}
	if (last != NULL)
		can_append(last, &from, m);
	else
		m_freem(m);
	CANP_LIST_UNLOCK();
}

/*
 * Receive path: a driver (real RX, or the slcan cdev) hands us a frame coming
 * off the wire.  Tap it to BPF once, then fan it out to all matching sockets.
 */
void
can_input(struct ifnet *ifp, struct mbuf *m)
{

	if (m->m_len < (int)sizeof(struct can_frame)) {
		m = m_pullup(m, sizeof(struct can_frame));
		if (m == NULL)
			return;
	}
	can_bpf_mtap(ifp, m);
	can_deliver(ifp, m, NULL);
}
