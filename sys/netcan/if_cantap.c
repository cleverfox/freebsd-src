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
 * Userspace-backed CAN interface ("slcan" / CAN tap).
 *
 * FreeBSD has no Linux-style N_SLCAN tty line discipline, so serial-line CAN
 * adapters are driven by a userspace daemon (slcand) instead.  This driver is
 * the kernel half: a cloned CAN network interface whose back end is a character
 * device, analogous to tun(4)/tap(4):
 *
 *   - frames transmitted by CAN sockets on the interface are queued and become
 *     readable on /dev/slcanN (the daemon forwards them to the serial adapter);
 *   - frames written to /dev/slcanN by the daemon (received from the adapter)
 *     are injected into the CAN input path and delivered to sockets.
 *
 * The on-cdev wire format is a raw struct can_frame (16 bytes) or canfd_frame
 * (72 bytes), identical to the socket payload.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/conf.h>
#include <sys/fcntl.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/poll.h>
#include <sys/proc.h>
#include <sys/selinfo.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/uio.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_private.h>
#include <net/if_clone.h>
#include <net/if_types.h>
#include <net/bpf.h>
#include <net/vnet.h>

#include <netcan/can.h>
#include <netcan/can_var.h>

#define	CANTAP_NAME	"slcan"
#define	CANTAP_QMAX	256	/* max queued app-TX frames */

struct cantap_softc {
	struct ifnet	*sc_ifp;
	struct cdev	*sc_cdev;
	struct mtx	 sc_mtx;
	struct mbufq	 sc_txq;	/* app TX awaiting cdev read */
	struct selinfo	 sc_rsel;	/* readers blocked on the cdev */
	int		 sc_open;	/* cdev currently open */
};

static d_open_t		cantap_dev_open;
static d_close_t	cantap_dev_close;
static d_read_t		cantap_dev_read;
static d_write_t	cantap_dev_write;
static d_poll_t		cantap_dev_poll;

static struct cdevsw cantap_cdevsw = {
	.d_version =	D_VERSION,
	.d_open =	cantap_dev_open,
	.d_close =	cantap_dev_close,
	.d_read =	cantap_dev_read,
	.d_write =	cantap_dev_write,
	.d_poll =	cantap_dev_poll,
	.d_name =	CANTAP_NAME,
};

VNET_DEFINE_STATIC(struct if_clone *, cantap_cloner);
#define	V_cantap_cloner	VNET(cantap_cloner)

/*
 * Interface transmit: an application sent a frame on the interface.  Queue it
 * for the userspace daemon to read and forward to the serial adapter.
 */
static int
cantap_transmit(struct ifnet *ifp, struct mbuf *m)
{
	struct cantap_softc *sc = ifp->if_softc;
	int error, plen;

	M_ASSERTPKTHDR(m);

	if (m->m_len < (int)sizeof(struct can_frame)) {
		m = m_pullup(m, sizeof(struct can_frame));
		if (m == NULL) {
			if_inc_counter(ifp, IFCOUNTER_OERRORS, 1);
			return (ENOBUFS);
		}
	}
	/* BPF tap is done in can_output(); here we only forward to the cdev. */
	plen = m->m_pkthdr.len;

	mtx_lock(&sc->sc_mtx);
	if (!sc->sc_open) {
		/* No daemon attached: nothing to carry the frame. */
		mtx_unlock(&sc->sc_mtx);
		if_inc_counter(ifp, IFCOUNTER_OERRORS, 1);
		m_freem(m);
		return (0);
	}
	error = mbufq_enqueue(&sc->sc_txq, m);
	if (error != 0) {
		mtx_unlock(&sc->sc_mtx);
		if_inc_counter(ifp, IFCOUNTER_OQDROPS, 1);
		m_freem(m);
		return (error);
	}
	selwakeup(&sc->sc_rsel);
	wakeup(sc);
	mtx_unlock(&sc->sc_mtx);

	if_inc_counter(ifp, IFCOUNTER_OPACKETS, 1);
	if_inc_counter(ifp, IFCOUNTER_OBYTES, plen);
	return (0);
}

static void
cantap_qflush(struct ifnet *ifp)
{
	struct cantap_softc *sc = ifp->if_softc;

	mtx_lock(&sc->sc_mtx);
	mbufq_drain(&sc->sc_txq);
	mtx_unlock(&sc->sc_mtx);
}

static int
cantap_ioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
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
		if (ifr->ifr_mtu != (int)sizeof(struct can_frame) &&
		    ifr->ifr_mtu != (int)sizeof(struct canfd_frame))
			error = EINVAL;
		else
			ifp->if_mtu = ifr->ifr_mtu;
		break;
	case SIOCADDMULTI:
	case SIOCDELMULTI:
		break;
	default:
		error = EINVAL;
		break;
	}
	return (error);
}

/*
 * Character device side (the daemon).
 */
static int
cantap_dev_open(struct cdev *dev, int oflags, int devtype, struct thread *td)
{
	struct cantap_softc *sc = dev->si_drv1;
	int error = 0;

	mtx_lock(&sc->sc_mtx);
	if (sc->sc_open)
		error = EBUSY;
	else
		sc->sc_open = 1;
	mtx_unlock(&sc->sc_mtx);
	return (error);
}

static int
cantap_dev_close(struct cdev *dev, int fflag, int devtype, struct thread *td)
{
	struct cantap_softc *sc = dev->si_drv1;

	mtx_lock(&sc->sc_mtx);
	sc->sc_open = 0;
	mbufq_drain(&sc->sc_txq);
	mtx_unlock(&sc->sc_mtx);
	return (0);
}

/* Daemon reads an app-transmitted frame to forward to the adapter. */
static int
cantap_dev_read(struct cdev *dev, struct uio *uio, int ioflag)
{
	struct cantap_softc *sc = dev->si_drv1;
	struct mbuf *m;
	int error = 0;

	mtx_lock(&sc->sc_mtx);
	while ((m = mbufq_dequeue(&sc->sc_txq)) == NULL) {
		if (ioflag & O_NONBLOCK) {
			mtx_unlock(&sc->sc_mtx);
			return (EWOULDBLOCK);
		}
		error = msleep(sc, &sc->sc_mtx, PCATCH, CANTAP_NAME, 0);
		if (error != 0) {
			mtx_unlock(&sc->sc_mtx);
			return (error);
		}
	}
	mtx_unlock(&sc->sc_mtx);

	if (m->m_pkthdr.len > uio->uio_resid)
		error = EMSGSIZE;
	for (; m != NULL && error == 0; m = m->m_next)
		error = uiomove(mtod(m, void *), m->m_len, uio);
	m_freem(m);
	return (error);
}

/* Daemon writes a frame received from the adapter; inject it as RX. */
static int
cantap_dev_write(struct cdev *dev, struct uio *uio, int ioflag)
{
	struct cantap_softc *sc = dev->si_drv1;
	struct ifnet *ifp = sc->sc_ifp;
	struct mbuf *m;
	int len = uio->uio_resid;
	int error;

	if (len != (int)sizeof(struct can_frame) &&
	    len != (int)sizeof(struct canfd_frame))
		return (EINVAL);

	m = m_get2(len, M_WAITOK, MT_DATA, M_PKTHDR);
	error = uiomove(mtod(m, void *), len, uio);
	if (error != 0) {
		m_freem(m);
		return (error);
	}
	m->m_len = m->m_pkthdr.len = len;
	m->m_pkthdr.rcvif = ifp;

	if_inc_counter(ifp, IFCOUNTER_IPACKETS, 1);
	if_inc_counter(ifp, IFCOUNTER_IBYTES, len);

	CURVNET_SET_QUIET(ifp->if_vnet);
	can_input(ifp, m);
	CURVNET_RESTORE();
	return (0);
}

static int
cantap_dev_poll(struct cdev *dev, int events, struct thread *td)
{
	struct cantap_softc *sc = dev->si_drv1;
	int revents = 0;

	if (events & (POLLIN | POLLRDNORM)) {
		mtx_lock(&sc->sc_mtx);
		if (mbufq_len(&sc->sc_txq) > 0)
			revents |= events & (POLLIN | POLLRDNORM);
		else
			selrecord(td, &sc->sc_rsel);
		mtx_unlock(&sc->sc_mtx);
	}
	/* The cdev is always writable (injection is synchronous). */
	revents |= events & (POLLOUT | POLLWRNORM);
	return (revents);
}

static int
cantap_clone_create(struct if_clone *ifc, char *name, size_t len,
    struct ifc_data *ifd, struct ifnet **ifpp)
{
	struct cantap_softc *sc;
	struct make_dev_args mda;
	struct ifnet *ifp;
	int error;

	sc = malloc(sizeof(*sc), M_DEVBUF, M_WAITOK | M_ZERO);
	mtx_init(&sc->sc_mtx, "cantap", NULL, MTX_DEF);
	mbufq_init(&sc->sc_txq, CANTAP_QMAX);

	make_dev_args_init(&mda);
	mda.mda_devsw = &cantap_cdevsw;
	mda.mda_uid = UID_ROOT;
	mda.mda_gid = GID_WHEEL;
	mda.mda_mode = 0600;
	mda.mda_si_drv1 = sc;
	error = make_dev_s(&mda, &sc->sc_cdev, "%s%d", CANTAP_NAME, ifd->unit);
	if (error != 0) {
		mbufq_drain(&sc->sc_txq);
		mtx_destroy(&sc->sc_mtx);
		free(sc, M_DEVBUF);
		return (error);
	}

	ifp = sc->sc_ifp = if_alloc(IFT_OTHER);
	ifp->if_softc = sc;
	if_initname(ifp, CANTAP_NAME, ifd->unit);
	ifp->if_mtu = sizeof(struct canfd_frame);
	/* Not IP-capable: no IFF_MULTICAST, so IPv6 ND never runs. */
	ifp->if_flags = IFF_SIMPLEX;
	ifp->if_drv_flags = IFF_DRV_RUNNING;
	ifp->if_hdrlen = 0;
	ifp->if_addrlen = 0;
	ifp->if_transmit = cantap_transmit;
	ifp->if_qflush = cantap_qflush;
	ifp->if_ioctl = cantap_ioctl;
	ifp->if_output = can_if_output;
	if_attach(ifp);
	bpfattach(ifp, DLT_CAN_SOCKETCAN, 0);

	*ifpp = ifp;
	return (0);
}

static int
cantap_clone_destroy(struct if_clone *ifc, struct ifnet *ifp, uint32_t flags)
{
	struct cantap_softc *sc = ifp->if_softc;

	destroy_dev(sc->sc_cdev);	/* drains cdev threads */
	bpfdetach(ifp);
	if_detach(ifp);
	if_free(ifp);

	seldrain(&sc->sc_rsel);
	mbufq_drain(&sc->sc_txq);
	mtx_destroy(&sc->sc_mtx);
	free(sc, M_DEVBUF);
	return (0);
}

static void
vnet_cantap_init(const void *unused __unused)
{
	struct if_clone_addreq req = {
		.create_f = cantap_clone_create,
		.destroy_f = cantap_clone_destroy,
		.flags = IFC_F_AUTOUNIT,
	};

	V_cantap_cloner = ifc_attach_cloner(CANTAP_NAME, &req);
}
VNET_SYSINIT(vnet_cantap_init, SI_SUB_PROTO_IF, SI_ORDER_ANY,
    vnet_cantap_init, NULL);

static void
vnet_cantap_uninit(const void *unused __unused)
{

	ifc_detach_cloner(V_cantap_cloner);
	V_cantap_cloner = NULL;
}
VNET_SYSUNINIT(vnet_cantap_uninit, SI_SUB_INIT_IF, SI_ORDER_SECOND,
    vnet_cantap_uninit, NULL);
