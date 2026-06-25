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
 * Kernel-internal definitions for the CAN protocol family.
 */

#ifndef _NETCAN_CAN_VAR_H_
#define _NETCAN_CAN_VAR_H_

#ifdef _KERNEL
#include <sys/malloc.h>
#include <sys/protosw.h>

struct ifnet;
struct mbuf;
struct socket;
struct sockopt;
struct canpcb;
struct sockaddr;
struct route;

MALLOC_DECLARE(M_CANPCB);

/*
 * Common structure for CAN interface drivers / pseudo interfaces.  A CAN
 * driver's softc should embed this at the start so the protocol layer can
 * reach the link parameters.  For pure virtual interfaces it may be unused.
 */
struct canif_softc {
	void		*csc_dev;	/* device_t, driver private */
	uint32_t	csc_linkmodes;	/* CAN_LINKMODE_* (future) */
};

/* CAN link-level modes (subset; matches Linux/NetBSD naming) */
#define CAN_LINKMODE_LOOPBACK		0x01
#define CAN_LINKMODE_LISTENONLY		0x02
#define CAN_LINKMODE_3SAMPLES		0x04
#define CAN_LINKMODE_PRESUME_ACK	0x08

/* Tunables / defaults (can.c) */
extern int can_sendspace;
extern int can_recvspace;

/* CAN_RAW protocol switch table (can.c), referenced by the domain. */
extern struct protosw cansw[];

/* protocol entry points */
int	can_ctloutput(struct socket *, struct sockopt *);

/* control block layer (can_pcb.c) */
void	can_pcb_init(void);
void	can_pcb_destroy(void);

/* input / interface glue (if_can.c) */
void	can_input(struct ifnet *, struct mbuf *);
void	can_deliver(struct ifnet *, struct mbuf *, struct canpcb *);
void	can_bpf_mtap(struct ifnet *, struct mbuf *);
int	can_if_output(struct ifnet *, struct mbuf *, const struct sockaddr *,
	    struct route *);

#endif /* _KERNEL */

#endif /* !_NETCAN_CAN_VAR_H_ */
