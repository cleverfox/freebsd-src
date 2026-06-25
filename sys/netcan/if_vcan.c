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
 * Virtual CAN interface ("vcan"), the FreeBSD equivalent of Linux's vcan and
 * NetBSD's canloop: a pure software interface that loops every transmitted
 * frame straight back into the CAN input path.  Useful for development and for
 * applications that exercise CAN without hardware.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/mbuf.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/socket.h>
#include <sys/sockio.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_private.h>
#include <net/if_clone.h>
#include <net/if_types.h>
#include <net/bpf.h>
#include <net/vnet.h>

#include <netcan/can.h>
#include <netcan/can_var.h>

static const char vcanname[] = "vcan";

VNET_DEFINE_STATIC(struct if_clone *, vcan_cloner);
#define	V_vcan_cloner	VNET(vcan_cloner)

static int
vcan_transmit(struct ifnet *ifp, struct mbuf *m)
{

	M_ASSERTPKTHDR(m);

	/*
	 * vcan has no wire: the local loopback to CAN sockets is performed
	 * centrally in can_output() (like every other CAN interface), so here
	 * we only account for the transmit and free the wire copy.
	 */
	if_inc_counter(ifp, IFCOUNTER_OPACKETS, 1);
	if_inc_counter(ifp, IFCOUNTER_OBYTES, m->m_pkthdr.len);
	m_freem(m);
	return (0);
}

static void
vcan_qflush(struct ifnet *ifp __unused)
{
}

static int
vcan_ioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
{
	struct ifreq *ifr = (struct ifreq *)data;
	int error = 0;

	switch (cmd) {
	case SIOCSIFADDR:
		ifp->if_flags |= IFF_UP;
		ifp->if_drv_flags |= IFF_DRV_RUNNING;
		break;
	case SIOCSIFFLAGS:
		break;
	case SIOCSIFMTU:
		/* Allow only a classic or an FD frame MTU. */
		if (ifr->ifr_mtu != (int)sizeof(struct can_frame) &&
		    ifr->ifr_mtu != (int)sizeof(struct canfd_frame))
			error = EINVAL;
		else
			ifp->if_mtu = ifr->ifr_mtu;
		break;
	case SIOCADDMULTI:
	case SIOCDELMULTI:
		/* CAN has no multicast addressing; accept silently. */
		break;
	default:
		error = EINVAL;
		break;
	}
	return (error);
}

static int
vcan_clone_create(struct if_clone *ifc, char *name, size_t len,
    struct ifc_data *ifd, struct ifnet **ifpp)
{
	struct ifnet *ifp;

	ifp = if_alloc(IFT_OTHER);
	if_initname(ifp, vcanname, ifd->unit);
	/* FD-capable by default; classic frames still fit. */
	ifp->if_mtu = sizeof(struct canfd_frame);
	/* Not IP-capable: no IFF_MULTICAST, so IPv6 ND never runs. */
	ifp->if_flags = IFF_SIMPLEX;
	ifp->if_drv_flags = IFF_DRV_RUNNING;
	ifp->if_hdrlen = 0;
	ifp->if_addrlen = 0;
	ifp->if_transmit = vcan_transmit;
	ifp->if_qflush = vcan_qflush;
	ifp->if_ioctl = vcan_ioctl;
	ifp->if_output = can_if_output;
	if_attach(ifp);
	bpfattach(ifp, DLT_CAN_SOCKETCAN, 0);
	*ifpp = ifp;

	return (0);
}

static int
vcan_clone_destroy(struct if_clone *ifc, struct ifnet *ifp, uint32_t flags)
{

	bpfdetach(ifp);
	if_detach(ifp);
	if_free(ifp);
	return (0);
}

static void
vnet_vcan_init(const void *unused __unused)
{
	struct if_clone_addreq req = {
		.create_f = vcan_clone_create,
		.destroy_f = vcan_clone_destroy,
		.flags = IFC_F_AUTOUNIT,
	};

	V_vcan_cloner = ifc_attach_cloner(vcanname, &req);
}
VNET_SYSINIT(vnet_vcan_init, SI_SUB_PROTO_IF, SI_ORDER_ANY,
    vnet_vcan_init, NULL);

static void
vnet_vcan_uninit(const void *unused __unused)
{

	ifc_detach_cloner(V_vcan_cloner);
	V_vcan_cloner = NULL;
}
VNET_SYSUNINIT(vnet_vcan_uninit, SI_SUB_INIT_IF, SI_ORDER_SECOND,
    vnet_vcan_uninit, NULL);
