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
 * CAN_RAW protocol logic.  Derived from NetBSD netcan/can.c, adapted to the
 * FreeBSD protosw method model and re-aligned to the Linux CAN_RAW socket API
 * (option level SOL_CAN_RAW and Linux option numbers).
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
#include <sys/sockopt.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/vnet.h>

#include <netcan/can.h>
#include <netcan/can/raw.h>
#include <netcan/can_var.h>
#include <netcan/can_pcb.h>

int	can_sendspace = 4096;		/* really max datagram size */
int	can_recvspace = 40 * (1024 + sizeof(struct sockaddr_can));

/*
 * Validate an outgoing frame and return the on-wire length the interface
 * should see, or 0 if the frame is not acceptable.  A classic frame is always
 * allowed; a CAN FD frame requires CAN_RAW_FD_FRAMES on the socket.  The frame
 * type is determined by the written length (mirrors Linux raw_check_txframe()).
 */
static u_int
can_check_txframe(struct canpcb *canp, struct mbuf *m)
{

	if (m->m_pkthdr.len == CAN_MTU)
		return (CAN_MTU);
	if (m->m_pkthdr.len == CANFD_MTU &&
	    (canp->canp_flags & CANP_FD_FRAMES) != 0)
		return (CANFD_MTU);
	return (0);
}

/*
 * Hand a fully formed frame to the bound/selected interface.
 *
 * This is the central send path (the analogue of Linux can_send()).  Unless the
 * socket disabled local loopback, a copy of the frame is delivered to the other
 * CAN sockets on the interface (honouring per-socket receive-own-messages), and
 * the original is handed to the driver to put on the wire.  Drivers therefore
 * only deal with the wire; local echo is done here for every interface type.
 */
static int
can_output(struct canpcb *canp, u_int ifindex, struct mbuf *m)
{
	struct epoch_tracker et;
	struct ifnet *ifp;
	u_int len;
	int error;

	len = can_check_txframe(canp, m);
	if (len == 0) {
		m_freem(m);
		return (EINVAL);
	}

	NET_EPOCH_ENTER(et);
	ifp = ifnet_byindex(ifindex);
	if (ifp == NULL) {
		NET_EPOCH_EXIT(et);
		m_freem(m);
		return (ENXIO);
	}
	if ((if_getflags(ifp) & IFF_UP) == 0) {
		NET_EPOCH_EXIT(et);
		m_freem(m);
		return (ENETDOWN);
	}
	if (len > if_getmtu(ifp)) {
		NET_EPOCH_EXIT(et);
		m_freem(m);
		return (EMSGSIZE);
	}

	/* Tap the transmitted frame to BPF (tcpdump) once, on TX. */
	can_bpf_mtap(ifp, m);

	/* Local loopback to other CAN sockets unless disabled. */
	if ((canp->canp_flags & CANP_NO_LOOPBACK) == 0) {
		struct mbuf *mc = m_copym(m, 0, M_COPYALL, M_NOWAIT);

		if (mc != NULL)
			can_deliver(ifp, mc, canp);
	}

	/* Hand to the driver (cdev for slcan, dropped for vcan). */
	error = if_transmit(ifp, m);
	NET_EPOCH_EXIT(et);
	return (error);
}

/*
 * pr_usrreqs-equivalent methods (flat protosw entry points).
 */
static int
can_attach(struct socket *so, int proto, struct thread *td)
{
	int error;

	KASSERT(sotocanpcb(so) == NULL, ("%s: so_pcb != NULL", __func__));

	error = soreserve(so, can_sendspace, can_recvspace);
	if (error != 0)
		return (error);
	return (can_pcballoc(so));
}

static void
can_detach(struct socket *so)
{
	struct canpcb *canp = sotocanpcb(so);

	KASSERT(canp != NULL, ("%s: canp == NULL", __func__));
	can_pcbdetach(canp);
}

static void
can_close(struct socket *so)
{

	soisdisconnected(so);
}

static int
can_bind(struct socket *so, struct sockaddr *nam, struct thread *td)
{
	struct canpcb *canp = sotocanpcb(so);

	if (nam->sa_len != sizeof(struct sockaddr_can))
		return (EINVAL);
	return (can_pcbbind(canp, satoscan(nam)));
}

static int
can_disconnect(struct socket *so)
{
	struct canpcb *canp = sotocanpcb(so);

	can_pcbdisconnect(canp);
	soisdisconnected(so);
	return (0);
}

static int
can_shutdown(struct socket *so, enum shutdown_how how)
{

	switch (how) {
	case SHUT_RD:
		socantrcvmore(so);
		break;
	case SHUT_RDWR:
		socantrcvmore(so);
		/* FALLTHROUGH */
	case SHUT_WR:
		socantsendmore(so);
	}
	return (0);
}

static int
can_sockaddr(struct socket *so, struct sockaddr *nam)
{

	can_setsockaddr(sotocanpcb(so), satoscan(nam));
	return (0);
}

static int
can_peeraddr(struct socket *so, struct sockaddr *nam)
{

	return (EOPNOTSUPP);
}

static int
can_send(struct socket *so, int flags, struct mbuf *m, struct sockaddr *nam,
    struct mbuf *control, struct thread *td)
{
	struct canpcb *canp = sotocanpcb(so);
	u_int ifindex;
	int error;

	if (control != NULL && control->m_len != 0) {
		m_freem(control);
		m_freem(m);
		return (EINVAL);
	}
	if (control != NULL)
		m_freem(control);
	if (flags & PRUS_OOB) {
		m_freem(m);
		return (EOPNOTSUPP);
	}
	if (canp == NULL) {
		m_freem(m);
		return (ECONNRESET);
	}

	if (nam != NULL) {
		struct sockaddr_can *scan = satoscan(nam);

		if (nam->sa_len != sizeof(*scan) ||
		    scan->can_family != AF_CAN) {
			m_freem(m);
			return (EINVAL);
		}
		ifindex = scan->can_ifindex;
	} else {
		ifindex = canp->canp_ifindex;
	}
	if (ifindex == 0) {
		m_freem(m);
		return (ENXIO);
	}

	KASSERT((m->m_flags & M_PKTHDR) != 0, ("%s: no pkthdr", __func__));

	/* Ensure the frame header is contiguous for filtering / bpf. */
	if (m->m_len < (int)CAN_MTU) {
		m = m_pullup(m, CAN_MTU);
		if (m == NULL)
			return (ENOBUFS);
	}

	error = can_output(canp, ifindex, m);
	return (error);
}

/*
 * setsockopt / getsockopt (SOL_CAN_RAW only).
 */
static int
can_getop(struct canpcb *canp, struct sockopt *sopt)
{
	struct can_filter *tmp;
	int optval, n;
	size_t sz;
	int error;

	switch (sopt->sopt_name) {
	case CAN_RAW_FILTER:
		CANP_LIST_LOCK();
		n = canp->canp_nfilters;
		sz = (size_t)n * sizeof(struct can_filter);
		tmp = (n > 0) ? malloc(sz, M_TEMP, M_NOWAIT) : NULL;
		if (n > 0 && tmp == NULL) {
			CANP_LIST_UNLOCK();
			return (ENOMEM);
		}
		if (n > 0)
			memcpy(tmp, canp->canp_filters, sz);
		CANP_LIST_UNLOCK();
		error = sooptcopyout(sopt, tmp, sz);
		if (tmp != NULL)
			free(tmp, M_TEMP);
		break;
	case CAN_RAW_ERR_FILTER:
		optval = canp->canp_err_mask;
		error = sooptcopyout(sopt, &optval, sizeof(optval));
		break;
	case CAN_RAW_LOOPBACK:
		optval = (canp->canp_flags & CANP_NO_LOOPBACK) ? 0 : 1;
		error = sooptcopyout(sopt, &optval, sizeof(optval));
		break;
	case CAN_RAW_RECV_OWN_MSGS:
		optval = (canp->canp_flags & CANP_RECEIVE_OWN) ? 1 : 0;
		error = sooptcopyout(sopt, &optval, sizeof(optval));
		break;
	case CAN_RAW_JOIN_FILTERS:
		optval = (canp->canp_flags & CANP_JOIN_FILTERS) ? 1 : 0;
		error = sooptcopyout(sopt, &optval, sizeof(optval));
		break;
	case CAN_RAW_FD_FRAMES:
		optval = (canp->canp_flags & CANP_FD_FRAMES) ? 1 : 0;
		error = sooptcopyout(sopt, &optval, sizeof(optval));
		break;
	default:
		error = ENOPROTOOPT;
		break;
	}
	return (error);
}

static int
can_setop(struct canpcb *canp, struct sockopt *sopt)
{
	struct can_filter *buf;
	can_err_mask_t mask;
	size_t valsize;
	int optval, nf;
	int error;

	switch (sopt->sopt_name) {
	case CAN_RAW_FILTER:
		valsize = sopt->sopt_valsize;
		if (valsize % sizeof(struct can_filter) != 0)
			return (EINVAL);
		nf = valsize / sizeof(struct can_filter);
		if (nf > CAN_RAW_FILTER_MAX)
			return (EINVAL);
		if (nf == 0)
			return (can_pcbsetfilter(canp, NULL, 0));
		buf = malloc(valsize, M_TEMP, M_WAITOK);
		error = sooptcopyin(sopt, buf, valsize, valsize);
		if (error == 0)
			error = can_pcbsetfilter(canp, buf, nf);
		free(buf, M_TEMP);
		break;
	case CAN_RAW_ERR_FILTER:
		error = sooptcopyin(sopt, &mask, sizeof(mask), sizeof(mask));
		if (error == 0)
			canp->canp_err_mask = mask & CAN_ERR_MASK;
		break;
	case CAN_RAW_LOOPBACK:
		error = sooptcopyin(sopt, &optval, sizeof(optval),
		    sizeof(optval));
		if (error == 0) {
			if (optval)
				canp->canp_flags &= ~CANP_NO_LOOPBACK;
			else
				canp->canp_flags |= CANP_NO_LOOPBACK;
		}
		break;
	case CAN_RAW_RECV_OWN_MSGS:
		error = sooptcopyin(sopt, &optval, sizeof(optval),
		    sizeof(optval));
		if (error == 0) {
			if (optval)
				canp->canp_flags |= CANP_RECEIVE_OWN;
			else
				canp->canp_flags &= ~CANP_RECEIVE_OWN;
		}
		break;
	case CAN_RAW_JOIN_FILTERS:
		error = sooptcopyin(sopt, &optval, sizeof(optval),
		    sizeof(optval));
		if (error == 0) {
			if (optval)
				canp->canp_flags |= CANP_JOIN_FILTERS;
			else
				canp->canp_flags &= ~CANP_JOIN_FILTERS;
		}
		break;
	case CAN_RAW_FD_FRAMES:
		error = sooptcopyin(sopt, &optval, sizeof(optval),
		    sizeof(optval));
		if (error == 0) {
			if (optval)
				canp->canp_flags |= CANP_FD_FRAMES;
			else
				canp->canp_flags &= ~CANP_FD_FRAMES;
		}
		break;
	default:
		error = ENOPROTOOPT;
		break;
	}
	return (error);
}

int
can_ctloutput(struct socket *so, struct sockopt *sopt)
{
	struct canpcb *canp = sotocanpcb(so);

	if (canp == NULL)
		return (ECONNRESET);
	if (sopt->sopt_level != SOL_CAN_RAW)
		return (EINVAL);

	if (sopt->sopt_dir == SOPT_SET)
		return (can_setop(canp, sopt));
	else
		return (can_getop(canp, sopt));
}

struct protosw cansw[] = {
{
	.pr_type =	SOCK_RAW,
	.pr_flags =	PR_ATOMIC | PR_ADDR,
	.pr_attach =	can_attach,
	.pr_detach =	can_detach,
	.pr_bind =	can_bind,
	.pr_disconnect = can_disconnect,
	.pr_shutdown =	can_shutdown,
	.pr_close =	can_close,
	.pr_abort =	can_close,
	.pr_send =	can_send,
	.pr_sockaddr =	can_sockaddr,
	.pr_peeraddr =	can_peeraddr,
	.pr_ctloutput =	can_ctloutput,
},
};
