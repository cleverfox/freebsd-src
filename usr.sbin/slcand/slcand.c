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
 * slcand - FreeBSD userspace daemon for serial-line CAN (slcan / Lawicel)
 * adapters, with an optional TCP transport for tunnelling CAN between machines.
 *
 * FreeBSD has no N_SLCAN tty line discipline, so this daemon bridges, in
 * userspace, between a "link" (a serial adapter or a TCP connection) and a
 * cdev-backed CAN interface (if_cantap, "slcanN" + /dev/slcanN):
 *
 *     link (serial slcan | TCP) <-> slcand <-> /dev/slcanN <-> sockets
 *
 * Usage:
 *   serial:        slcand [opts] <serialdev> [ifname]
 *   TCP server:    slcand [opts] -p <port>   [ifname]
 *   TCP client:    slcand [opts] -t <host:port> [ifname]
 *
 *   -o          send "open channel" (O) to the adapter (serial or TCP)
 *   -c          send "close channel" (C) on exit (serial or TCP)
 *   -s <0-8>    nominal CAN bitrate code (S0=10k .. S6=500k .. S8=1M)
 *   -y <1-8>    CAN FD data-phase bitrate code (Y1=1M..Y8=8M)
 *   -b <baud>   serial UART baud (default 115200)
 *   -v          print bridged frames to stdout (candump-like)
 *   -p <port>   TCP server: listen and bridge accepted connections
 *   -t <h:p>    TCP client: connect to host:port and bridge (auto-reconnects)
 *   [ifname]    CAN interface to create/use (default slcan0)
 *
 * Two slcand instances over TCP form a shared virtual CAN bus: a frame sent on
 * one machine's slcanN appears on the other's.  The slcan wire format (see
 * sl_proto.md) is identical on serial and TCP; only the transport differs.
 */

#include <sys/param.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/sockio.h>

#include <net/if.h>
#include <netcan/can.h>
#include <netinet/in.h>
#include <netinet/tcp.h>

#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#define	RECONNECT_DELAY	2	/* seconds between TCP client reconnect tries */

static volatile sig_atomic_t stop;
static int verbose;

static void
onsig(int s __unused)
{
	stop = 1;
}

/* Sleep up to secs seconds, returning early if a termination signal arrives. */
static void
wait_stop(int secs)
{
	int i;

	for (i = 0; i < secs && !stop; i++)
		(void)sleep(1);
}

static void
setup_signals(void)
{
	struct sigaction sa;

	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = onsig;
	(void)sigaction(SIGINT, &sa, NULL);
	(void)sigaction(SIGTERM, &sa, NULL);
	sa.sa_handler = SIG_IGN;
	(void)sigaction(SIGPIPE, &sa, NULL);
}

/* CAN FD DLC code (0..15) <-> data length. */
static const uint8_t fd_dlc2len[16] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64
};

static int
fd_len2dlc(int len)
{
	int i;

	for (i = 0; i < 16; i++)
		if (fd_dlc2len[i] >= len)
			return (i);
	return (15);
}

static int
hexval(int c)
{
	if (c >= '0' && c <= '9')
		return (c - '0');
	if (c >= 'a' && c <= 'f')
		return (c - 'a' + 10);
	if (c >= 'A' && c <= 'F')
		return (c - 'A' + 10);
	return (-1);
}

static const struct {
	long		baud;
	speed_t		speed;
} baud_table[] = {
	{ 9600, B9600 },	{ 19200, B19200 },
	{ 38400, B38400 },	{ 57600, B57600 },
	{ 115200, B115200 },	{ 230400, B230400 },
	{ 460800, B460800 },	{ 921600, B921600 },
};

static speed_t
baud_to_speed(long b)
{
	unsigned int i;

	for (i = 0; i < nitems(baud_table); i++)
		if (baud_table[i].baud == b)
			return (baud_table[i].speed);
	return (0);
}

/*
 * Parse one slcan line (without trailing CR) into a raw CAN frame in out[].
 * Returns CAN_MTU/CANFD_MTU (bytes to write to /dev/slcanN), or -1.
 */
static int
slcan_parse(const char *l, size_t n, uint8_t *out)
{
	int eff, rtr, fd, brs, idlen, dlc, len, i, hi, lo;
	canid_t id = 0;

	if (n < 2)
		return (-1);
	eff = rtr = fd = brs = 0;
	switch (l[0]) {
	case 't':
		break;
	case 'T':
		eff = 1;
		break;
	case 'r':
		rtr = 1;
		break;
	case 'R':
		eff = 1;
		rtr = 1;
		break;
	case 'd':
		fd = 1;
		break;
	case 'D':
		fd = 1;
		eff = 1;
		break;
	case 'b':
		fd = 1;
		brs = 1;
		break;
	case 'B':
		fd = 1;
		eff = 1;
		brs = 1;
		break;
	default:
		return (-1);
	}
	idlen = eff ? 8 : 3;
	if ((int)n < 1 + idlen + 1)
		return (-1);
	for (i = 0; i < idlen; i++) {
		int v = hexval(l[1 + i]);
		if (v < 0)
			return (-1);
		id = (id << 4) | v;
	}
	dlc = hexval(l[1 + idlen]);
	if (dlc < 0)
		return (-1);

	if (fd) {
		struct canfd_frame *cf = (struct canfd_frame *)(void *)out;
		const char *d = l + 1 + idlen + 1;

		if (dlc > 15)
			return (-1);
		len = fd_dlc2len[dlc];
		if ((int)n < (d - l) + 2 * len)
			return (-1);
		memset(cf, 0, sizeof(*cf));
		cf->can_id = id & (eff ? CAN_EFF_MASK : CAN_SFF_MASK);
		if (eff)
			cf->can_id |= CAN_EFF_FLAG;
		cf->len = len;
		cf->flags = CANFD_FDF | (brs ? CANFD_BRS : 0);
		for (i = 0; i < len; i++) {
			hi = hexval(d[2 * i]);
			lo = hexval(d[2 * i + 1]);
			if (hi < 0 || lo < 0)
				return (-1);
			cf->data[i] = (hi << 4) | lo;
		}
		return (CANFD_MTU);
	} else {
		struct can_frame *cf = (struct can_frame *)(void *)out;
		const char *d = l + 1 + idlen + 1;

		if (dlc > CAN_MAX_DLEN)
			return (-1);
		memset(cf, 0, sizeof(*cf));
		cf->can_id = id & (eff ? CAN_EFF_MASK : CAN_SFF_MASK);
		if (eff)
			cf->can_id |= CAN_EFF_FLAG;
		if (rtr)
			cf->can_id |= CAN_RTR_FLAG;
		cf->len = dlc;
		if (!rtr) {
			if ((int)n < (d - l) + 2 * dlc)
				return (-1);
			for (i = 0; i < dlc; i++) {
				hi = hexval(d[2 * i]);
				lo = hexval(d[2 * i + 1]);
				if (hi < 0 || lo < 0)
					return (-1);
				cf->data[i] = (hi << 4) | lo;
			}
		}
		return (CAN_MTU);
	}
}

/* Format a raw CAN frame (CAN_MTU/CANFD_MTU) into an slcan line incl. CR. */
static int
slcan_format(const uint8_t *buf, int buflen, char *out, size_t osz)
{
	int eff, i, len, l;

	if (buflen == (int)CANFD_MTU) {
		const struct canfd_frame *cf =
		    (const struct canfd_frame *)(const void *)buf;
		int brs = !!(cf->flags & CANFD_BRS);
		canid_t id;
		int dlc, plen;
		char c;

		eff = !!(cf->can_id & CAN_EFF_FLAG);
		id = cf->can_id & (eff ? CAN_EFF_MASK : CAN_SFF_MASK);
		c = brs ? (eff ? 'B' : 'b') : (eff ? 'D' : 'd');
		dlc = fd_len2dlc(cf->len);
		plen = fd_dlc2len[dlc];
		if (eff)
			l = snprintf(out, osz, "%c%08X%X", c, id, dlc);
		else
			l = snprintf(out, osz, "%c%03X%X", c, id, dlc);
		for (i = 0; i < plen; i++)
			l += snprintf(out + l, osz - l, "%02X",
			    i < cf->len ? cf->data[i] : 0);
		l += snprintf(out + l, osz - l, "\r");
		return (l);
	} else {
		const struct can_frame *cf =
		    (const struct can_frame *)(const void *)buf;
		int rtr = !!(cf->can_id & CAN_RTR_FLAG);
		canid_t id;
		char c;

		eff = !!(cf->can_id & CAN_EFF_FLAG);
		id = cf->can_id & (eff ? CAN_EFF_MASK : CAN_SFF_MASK);
		c = rtr ? (eff ? 'R' : 'r') : (eff ? 'T' : 't');
		len = cf->len & 0xf;
		if (eff)
			l = snprintf(out, osz, "%c%08X%u", c, id, len);
		else
			l = snprintf(out, osz, "%c%03X%u", c, id, len);
		if (!rtr) {
			for (i = 0; i < len && i < CAN_MAX_DLEN; i++)
				l += snprintf(out + l, osz - l, "%02X",
				    cf->data[i]);
		}
		l += snprintf(out + l, osz - l, "\r");
		return (l);
	}
}

/*
 * Print a raw CAN frame (CAN_MTU/CANFD_MTU) to stdout in a candump-like form,
 * tagged with the interface and direction ("RX" = from the link/bus into the
 * interface, "TX" = from the interface out to the link/bus).  Used by -v.
 */
static void
dump_frame(const char *ifname, const char *dir, const uint8_t *buf, int buflen)
{
	const uint8_t *data;
	canid_t id;
	int eff, i, len, maxdata;
	int fd = 0, rtr = 0, brs = 0;

	if (buflen == (int)CANFD_MTU) {
		const struct canfd_frame *cf =
		    (const struct canfd_frame *)(const void *)buf;

		fd = 1;
		id = cf->can_id;
		len = cf->len;
		data = cf->data;
		brs = !!(cf->flags & CANFD_BRS);
		maxdata = CANFD_MAX_DLEN;
	} else {
		const struct can_frame *cf =
		    (const struct can_frame *)(const void *)buf;

		id = cf->can_id;
		len = cf->len & 0xf;
		data = cf->data;
		rtr = !!(id & CAN_RTR_FLAG);
		maxdata = CAN_MAX_DLEN;
	}
	eff = !!(id & CAN_EFF_FLAG);
	id &= eff ? CAN_EFF_MASK : CAN_SFF_MASK;

	printf("%-7s %s %0*X [%d]", ifname, dir, eff ? 8 : 3, id, len);
	if (rtr) {
		printf("  remote request");
	} else {
		for (i = 0; i < len && i < maxdata; i++)
			printf(" %02X", data[i]);
	}
	if (fd)
		printf("  (FD%s)", brs ? ",BRS" : "");
	printf("\n");
	fflush(stdout);
}

static void
iface_up(const char *ifname)
{
	struct ifreq ifr;
	int s;

	if ((s = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
		err(1, "socket");
	memset(&ifr, 0, sizeof(ifr));
	strlcpy(ifr.ifr_name, ifname, sizeof(ifr.ifr_name));
	if (ioctl(s, SIOCIFCREATE, &ifr) < 0 && errno != EEXIST)
		err(1, "SIOCIFCREATE %s", ifname);
	if (ioctl(s, SIOCGIFFLAGS, &ifr) < 0)
		err(1, "SIOCGIFFLAGS");
	ifr.ifr_flags |= IFF_UP;
	if (ioctl(s, SIOCSIFFLAGS, &ifr) < 0)
		err(1, "SIOCSIFFLAGS up");
	close(s);
}

/*
 * Send the slcan channel-setup sequence on an already-open link (serial or
 * TCP): reset (C), optional nominal bitrate (S), optional CAN FD data bitrate
 * (Y), and optionally open the channel (O).  A peer slcand ignores these
 * non-frame lines, so this is harmless when tunnelling between two daemons; a
 * real adapter reached over TCP needs them (notably "O") to go on-bus.
 */
static void
slcan_setup(int fd, int bitrate, int databitrate, int open_chan)
{
	char cmd[8];

	(void)write(fd, "C\r", 2);
	usleep(10000);
	if (bitrate >= 0 && bitrate <= 8) {
		snprintf(cmd, sizeof(cmd), "S%d\r", bitrate);
		(void)write(fd, cmd, strlen(cmd));
		usleep(10000);
	}
	if (databitrate >= 0 && databitrate <= 8) {
		snprintf(cmd, sizeof(cmd), "Y%d\r", databitrate);
		(void)write(fd, cmd, strlen(cmd));
		usleep(10000);
	}
	if (open_chan) {
		(void)write(fd, "O\r", 2);
		usleep(10000);
	}
}

/*
 * Open and raw-configure a serial adapter; send slcan setup commands.
 * If wait_secs > 0, retry while the device node is absent (it may not have been
 * created yet by USB enumeration at boot time).
 */
static int
serial_open(const char *dev, long baud, int bitrate, int databitrate,
    int open_chan, int wait_secs)
{
	struct termios tio;
	speed_t spd;
	int fd, tries = 0, maxtries = wait_secs * 2;	/* 2 tries/second */

	if ((spd = baud_to_speed(baud)) == 0)
		errx(1, "unsupported baud %ld", baud);
	while ((fd = open(dev, O_RDWR | O_NOCTTY)) < 0) {
		if ((errno != ENOENT && errno != ENXIO) || tries >= maxtries)
			err(1, "open %s", dev);
		usleep(500000);
		tries++;
	}
	if (tcgetattr(fd, &tio) < 0)
		err(1, "tcgetattr");
	cfmakeraw(&tio);
	cfsetspeed(&tio, spd);
	tio.c_cflag |= CLOCAL | CREAD;
	if (tcsetattr(fd, TCSANOW, &tio) < 0)
		err(1, "tcsetattr");

	slcan_setup(fd, bitrate, databitrate, open_chan);
	return (fd);
}

static int
tcp_listen(const char *port)
{
	struct addrinfo hints, *res, *ai;
	int lfd = -1, on = 1, off = 0;

	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_flags = AI_PASSIVE;
	if (getaddrinfo(NULL, port, &hints, &res) != 0)
		errx(1, "getaddrinfo(*:%s)", port);
	for (ai = res; ai != NULL; ai = ai->ai_next) {
		lfd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
		if (lfd < 0)
			continue;
		setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
		/* Accept IPv4 too on an IPv6 wildcard socket. */
		if (ai->ai_family == AF_INET6)
			setsockopt(lfd, IPPROTO_IPV6, IPV6_V6ONLY, &off,
			    sizeof(off));
		if (bind(lfd, ai->ai_addr, ai->ai_addrlen) == 0)
			break;
		close(lfd);
		lfd = -1;
	}
	freeaddrinfo(res);
	if (lfd < 0)
		err(1, "bind :%s", port);
	if (listen(lfd, 1) < 0)
		err(1, "listen");
	return (lfd);
}

static int
tcp_connect(const char *hostport)
{
	struct addrinfo hints, *res, *ai;
	char buf[256], *host, *port;
	int fd = -1, on = 1;

	/* Work on a copy: the reconnect loop calls this repeatedly. */
	strlcpy(buf, hostport, sizeof(buf));
	port = strrchr(buf, ':');
	if (port == NULL)
		errx(1, "expected host:port, got %s", hostport);
	*port++ = '\0';
	host = buf;
	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	if (getaddrinfo(host, port, &hints, &res) != 0) {
		warnx("getaddrinfo(%s:%s) failed", host, port);
		return (-1);
	}
	for (ai = res; ai != NULL; ai = ai->ai_next) {
		fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
		if (fd < 0)
			continue;
		if (connect(fd, ai->ai_addr, ai->ai_addrlen) == 0)
			break;
		close(fd);
		fd = -1;
	}
	freeaddrinfo(res);
	if (fd < 0) {
		warn("connect %s", hostport);
		return (-1);
	}
	setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));
	return (fd);
}

/*
 * A complete slcan line was received from the link: parse it and inject the
 * resulting raw CAN frame into the cdev (and echo it under -v).
 */
static void
link_line_to_can(const char *line, size_t linelen, int dfd, const char *ifname)
{
	_Alignas(struct canfd_frame) uint8_t fr[CANFD_MTU];
	int mtu;

	if (linelen == 0)
		return;
	if ((mtu = slcan_parse(line, linelen, fr)) <= 0)
		return;
	(void)write(dfd, fr, mtu);
	if (verbose)
		dump_frame(ifname, "RX", fr, mtu);
}

/*
 * Bridge a link fd (serial or TCP) and the CAN cdev until EOF on the link or a
 * termination signal.  Returns 0 on link EOF, -1 on signal/stop.
 */
static int
bridge(int linkfd, int dfd, const char *ifname)
{
	struct pollfd pfd[2] = {
		{ .fd = linkfd, .events = POLLIN },
		{ .fd = dfd, .events = POLLIN },
	};
	char line[256];
	size_t linelen = 0;

	while (!stop) {
		if (poll(pfd, 2, 1000) < 0) {
			if (errno == EINTR)
				continue;
			err(1, "poll");
		}

		/* link -> CAN: accumulate to CR, parse, inject. */
		if (pfd[0].revents & (POLLIN | POLLHUP)) {
			char buf[512];
			ssize_t i, n;

			n = read(linkfd, buf, sizeof(buf));
			if (n <= 0)
				return (0);	/* link closed */
			for (i = 0; i < n; i++) {
				char c = buf[i];
				if (c == '\r' || c == '\a') {
					link_line_to_can(line, linelen, dfd,
					    ifname);
					linelen = 0;
				} else if (linelen < sizeof(line) - 1) {
					line[linelen++] = c;
				}
			}
		}

		/* CAN -> link: read a frame, emit slcan ASCII. */
		if (pfd[1].revents & POLLIN) {
			_Alignas(struct canfd_frame) uint8_t fr[CANFD_MTU];
			ssize_t n = read(dfd, fr, sizeof(fr));
			if (n == (ssize_t)CAN_MTU || n == (ssize_t)CANFD_MTU) {
				char out[160];
				int l = slcan_format(fr, n, out, sizeof(out));
				if (write(linkfd, out, l) < 0 && errno == EPIPE)
					return (0);
				if (verbose)
					dump_frame(ifname, "TX", fr, n);
			}
		}
	}
	return (-1);
}

int
main(int argc, char **argv)
{
	const char *ifname = "slcan0";
	const char *serialdev = NULL, *tcpport = NULL;
	char *tcphost = NULL;
	char devpath[64];
	long baud = 115200;
	int bitrate = -1, databitrate = -1, open_chan = 0, close_chan = 0;
	int wait_secs = 0;
	int sfd = -1, lfd = -1, dfd, ch;

	while ((ch = getopt(argc, argv, "b:cop:s:t:vw:y:")) != -1) {
		switch (ch) {
		case 'b':
			baud = atol(optarg);
			break;
		case 'c':
			close_chan = 1;
			break;
		case 'o':
			open_chan = 1;
			break;
		case 'p':
			tcpport = optarg;
			break;
		case 's':
			bitrate = atoi(optarg);
			break;
		case 't':
			tcphost = optarg;
			break;
		case 'v':
			verbose = 1;
			break;
		case 'w':
			wait_secs = atoi(optarg);
			break;
		case 'y':
			databitrate = atoi(optarg);
			break;
		default:
			goto usage;
		}
	}
	argc -= optind;
	argv += optind;

	if (tcpport != NULL || tcphost != NULL) {
		/* TCP mode: no serial device; ifname is the only arg. */
		if (argc >= 1)
			ifname = argv[0];
	} else {
		if (argc < 1) {
usage:
			fprintf(stderr,
			    "usage: slcand [-cov] [-s 0-8] [-y 1-8] "
			    "[-b baud] [-w secs] <serialdev> [ifname]\n"
			    "       slcand [-cov] [-s 0-8] [-y 1-8] -p port "
			    "[ifname]   (TCP server)\n"
			    "       slcand [-cov] [-s 0-8] [-y 1-8] "
			    "-t host:port [ifname]   (TCP client)\n"
			    "   -o sends the open (O) command, needed to put a "
			    "real adapter on-bus (serial or TCP)\n"
			    "   -v prints bridged frames to stdout\n"
			    "   -w secs: wait up to secs for <serialdev> to "
			    "appear (boot/USB)\n");
			return (2);
		}
		serialdev = argv[0];
		if (argc >= 2)
			ifname = argv[1];
	}

	setup_signals();

	/*
	 * Establish the link first for serial mode: a not-yet-present serial
	 * device (e.g. a USB adapter still enumerating at boot, with -w to
	 * wait) should not leave an orphaned CAN interface behind.  A TCP
	 * client instead raises the interface and (re)connects in its loop.
	 */
	if (serialdev != NULL)
		sfd = serial_open(serialdev, baud, bitrate, databitrate,
		    open_chan, wait_secs);
	else if (tcpport != NULL)
		lfd = tcp_listen(tcpport);
	/* A TCP client (-t) connects inside its reconnect loop below. */

	/* Now create/raise the CAN interface and open its backing cdev. */
	iface_up(ifname);
	snprintf(devpath, sizeof(devpath), "/dev/%s", ifname);
	if ((dfd = open(devpath, O_RDWR)) < 0)
		err(1, "open %s", devpath);

	if (serialdev != NULL) {
		fprintf(stderr, "slcand: serial %s <-> %s\n", serialdev,
		    ifname);
		bridge(sfd, dfd, ifname);
		if (close_chan)
			(void)write(sfd, "C\r", 2);
		close(sfd);
	} else if (tcphost != NULL) {
		/*
		 * TCP client: connect to the server and bridge.  On link loss
		 * (or if the server is not up yet), wait and reconnect, keeping
		 * the CAN interface up across the gap.
		 */
		while (!stop) {
			sfd = tcp_connect(tcphost);
			if (sfd < 0) {
				wait_stop(RECONNECT_DELAY);
				continue;
			}
			fprintf(stderr, "slcand: tcp %s <-> %s connected\n",
			    tcphost, ifname);
			slcan_setup(sfd, bitrate, databitrate, open_chan);
			if (bridge(sfd, dfd, ifname) < 0) {
				/* Terminating signal: close the channel. */
				if (close_chan)
					(void)write(sfd, "C\r", 2);
				close(sfd);
				break;
			}
			/* Link EOF: the server went away; reconnect. */
			fprintf(stderr, "slcand: tcp %s disconnected\n",
			    tcphost);
			close(sfd);
			wait_stop(RECONNECT_DELAY);
		}
	} else {
		fprintf(stderr, "slcand: tcp listen :%s <-> %s\n", tcpport,
		    ifname);
		while (!stop) {
			int cfd, on;

			cfd = accept(lfd, NULL, NULL);
			if (cfd < 0) {
				if (errno == EINTR)
					continue;
				err(1, "accept");
			}
			on = 1;
			setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &on,
			    sizeof(on));
			fprintf(stderr, "slcand: client connected\n");
			slcan_setup(cfd, bitrate, databitrate, open_chan);
			bridge(cfd, dfd, ifname);
			if (close_chan)
				(void)write(cfd, "C\r", 2);
			close(cfd);
			fprintf(stderr, "slcand: client disconnected\n");
		}
		close(lfd);
	}

	close(dfd);
	return (0);
}
