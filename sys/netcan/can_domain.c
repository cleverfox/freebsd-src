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
 * CAN protocol family domain registration and loadable-module glue.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/domain.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/protosw.h>
#include <sys/socket.h>
#include <sys/sysctl.h>

#include <net/vnet.h>

#include <netcan/can.h>
#include <netcan/can_var.h>
#include <netcan/can_pcb.h>

FEATURE(can, "Controller Area Network (PF_CAN) sockets");

static struct domain candomain = {
	.dom_family =	PF_CAN,
	.dom_name =	"can",
	.dom_flags =	DOMF_UNLOADABLE,
	.dom_nprotosw =	1,
	.dom_protosw =	{ &cansw[0] },
};
DOMAIN_SET(can);

/*
 * Per-vnet initialization of the control-block list.  VNET memory is zeroed,
 * so the list head is already empty; LIST_INIT makes that explicit.
 */
static void
vnet_can_init(const void *unused __unused)
{

	can_pcb_init();
}
VNET_SYSINIT(vnet_can_init, SI_SUB_PROTO_DOMAIN, SI_ORDER_THIRD,
    vnet_can_init, NULL);

static void
vnet_can_uninit(const void *unused __unused)
{

	can_pcb_destroy();
}
VNET_SYSUNINIT(vnet_can_uninit, SI_SUB_PROTO_DOMAIN, SI_ORDER_THIRD,
    vnet_can_uninit, NULL);

static int
can_modevent(module_t mod __unused, int what, void *priv __unused)
{
	int error = 0;

	switch (what) {
	case MOD_LOAD:
		break;
	case MOD_UNLOAD:
		/*
		 * Refuse to unload while any CAN socket is still open; the
		 * domain (and its protosw methods) would vanish underneath it.
		 */
		CANP_LIST_LOCK();
		if (can_pcb_count != 0)
			error = EBUSY;
		CANP_LIST_UNLOCK();
		break;
	default:
		error = EOPNOTSUPP;
		break;
	}
	return (error);
}

static moduledata_t can_mod = {
	"can",
	can_modevent,
	NULL
};

DECLARE_MODULE(can, can_mod, SI_SUB_PSEUDO, SI_ORDER_ANY);
MODULE_VERSION(can, 1);
