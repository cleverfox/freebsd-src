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
 * ATF tests for the FreeBSD CAN_RAW socket stack.  Each test runs in its own
 * vnet jail (see Makefile), creates a fresh vcan interface, and exercises the
 * Linux-compatible socket behaviour.
 */

#include <sys/param.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <net/if.h>
#include <netcan/can.h>
#include <netcan/can/raw.h>

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

#include <atf-c.h>

#define	VCAN	"vcan0"

/*
 * Create the vcan interface (optionally bringing it up) and return its
 * ifindex.  Pre-destroys any leftover so the suite is also runnable directly
 * (outside a per-test vnet jail) during development.
 */
static unsigned int
vcan_create(int bring_up)
{
	struct ifreq ifr;
	unsigned int idx;
	int s;

	ATF_REQUIRE((s = socket(AF_INET, SOCK_DGRAM, 0)) >= 0);

	memset(&ifr, 0, sizeof(ifr));
	strlcpy(ifr.ifr_name, VCAN, sizeof(ifr.ifr_name));
	(void)ioctl(s, SIOCIFDESTROY, &ifr);	/* ignore ENXIO */

	memset(&ifr, 0, sizeof(ifr));
	strlcpy(ifr.ifr_name, VCAN, sizeof(ifr.ifr_name));
	ATF_REQUIRE_MSG(ioctl(s, SIOCIFCREATE, &ifr) == 0,
	    "SIOCIFCREATE %s: %s", VCAN, strerror(errno));

	if (bring_up) {
		memset(&ifr, 0, sizeof(ifr));
		strlcpy(ifr.ifr_name, VCAN, sizeof(ifr.ifr_name));
		ifr.ifr_flags = IFF_UP;
		ATF_REQUIRE_MSG(ioctl(s, SIOCSIFFLAGS, &ifr) == 0,
		    "SIOCSIFFLAGS up: %s", strerror(errno));
	}

	close(s);
	ATF_REQUIRE((idx = if_nametoindex(VCAN)) != 0);
	return (idx);
}

static unsigned int
vcan_up(void)
{
	return (vcan_create(1));
}

static int
can_bound(unsigned int idx, int nonblock)
{
	struct sockaddr_can addr;
	int s;

	ATF_REQUIRE((s = socket(AF_CAN, SOCK_RAW, CAN_RAW)) >= 0);
	memset(&addr, 0, sizeof(addr));
	addr.can_len = sizeof(addr);
	addr.can_family = AF_CAN;
	addr.can_ifindex = idx;
	ATF_REQUIRE_MSG(bind(s, (struct sockaddr *)&addr, sizeof(addr)) == 0,
	    "bind: %s", strerror(errno));
	if (nonblock)
		ATF_REQUIRE(fcntl(s, F_SETFL, O_NONBLOCK) == 0);
	return (s);
}

static void
send_classic(int s, canid_t id, uint8_t b0)
{
	struct can_frame f;

	memset(&f, 0, sizeof(f));
	f.can_id = id;
	f.len = 1;
	f.data[0] = b0;
	ATF_REQUIRE_MSG(write(s, &f, sizeof(f)) == (ssize_t)sizeof(f),
	    "write: %s", strerror(errno));
}

/* 1 if a frame was read (consumed), 0 if none available. */
static int
got_frame(int s, canid_t *idp)
{
	struct can_frame f;
	ssize_t n = read(s, &f, sizeof(f));

	if (n == (ssize_t)sizeof(f)) {
		if (idp != NULL)
			*idp = f.can_id;
		return (1);
	}
	return (0);
}

ATF_TC_WITHOUT_HEAD(socket_open);
ATF_TC_BODY(socket_open, tc)
{
	int s = socket(AF_CAN, SOCK_RAW, CAN_RAW);
	ATF_REQUIRE_MSG(s >= 0, "socket(AF_CAN): %s", strerror(errno));
	close(s);
}

ATF_TC_WITHOUT_HEAD(bind_bad_ifindex);
ATF_TC_BODY(bind_bad_ifindex, tc)
{
	struct sockaddr_can addr;
	int s;

	ATF_REQUIRE((s = socket(AF_CAN, SOCK_RAW, CAN_RAW)) >= 0);
	memset(&addr, 0, sizeof(addr));
	addr.can_len = sizeof(addr);
	addr.can_family = AF_CAN;
	addr.can_ifindex = 9999;	/* nonexistent */
	ATF_REQUIRE_ERRNO(ENODEV,
	    bind(s, (struct sockaddr *)&addr, sizeof(addr)) != 0);
	close(s);
}

ATF_TC_WITHOUT_HEAD(loopback);
ATF_TC_BODY(loopback, tc)
{
	unsigned int idx = vcan_up();
	int a = can_bound(idx, 1);
	int b = can_bound(idx, 1);
	canid_t id;

	send_classic(a, 0x111, 0xaa);
	ATF_CHECK_MSG(got_frame(b, &id) == 1 && id == 0x111,
	    "peer did not receive frame");
	ATF_CHECK_MSG(got_frame(a, NULL) == 0,
	    "sender received its own frame with recv_own off");
	close(a);
	close(b);
}

ATF_TC_WITHOUT_HEAD(recv_own_msgs);
ATF_TC_BODY(recv_own_msgs, tc)
{
	unsigned int idx = vcan_up();
	int a = can_bound(idx, 1);
	int on = 1;

	ATF_REQUIRE(setsockopt(a, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS,
	    &on, sizeof(on)) == 0);
	send_classic(a, 0x222, 0xbb);
	ATF_CHECK_MSG(got_frame(a, NULL) == 1,
	    "sender did not receive own frame with recv_own on");
	close(a);
}

ATF_TC_WITHOUT_HEAD(loopback_disabled);
ATF_TC_BODY(loopback_disabled, tc)
{
	unsigned int idx = vcan_up();
	int a = can_bound(idx, 1);
	int b = can_bound(idx, 1);
	int off = 0;

	ATF_REQUIRE(setsockopt(a, SOL_CAN_RAW, CAN_RAW_LOOPBACK,
	    &off, sizeof(off)) == 0);
	send_classic(a, 0x333, 0xcc);
	ATF_CHECK_MSG(got_frame(b, NULL) == 0,
	    "loopback-off frame was still delivered");
	close(a);
	close(b);
}

ATF_TC_WITHOUT_HEAD(filter);
ATF_TC_BODY(filter, tc)
{
	unsigned int idx = vcan_up();
	int a = can_bound(idx, 1);
	int b = can_bound(idx, 1);
	struct can_filter filt = { .can_id = 0x200, .can_mask = CAN_SFF_MASK };

	ATF_REQUIRE(setsockopt(b, SOL_CAN_RAW, CAN_RAW_FILTER,
	    &filt, sizeof(filt)) == 0);
	send_classic(a, 0x123, 0x01);
	ATF_CHECK_MSG(got_frame(b, NULL) == 0, "non-matching id passed filter");
	send_classic(a, 0x200, 0x02);
	ATF_CHECK_MSG(got_frame(b, NULL) == 1, "matching id was filtered out");
	close(a);
	close(b);
}

ATF_TC_WITHOUT_HEAD(filter_inverted);
ATF_TC_BODY(filter_inverted, tc)
{
	unsigned int idx = vcan_up();
	int a = can_bound(idx, 1);
	int b = can_bound(idx, 1);
	struct can_filter filt = {
		.can_id = 0x200 | CAN_INV_FILTER,
		.can_mask = CAN_SFF_MASK,
	};

	ATF_REQUIRE(setsockopt(b, SOL_CAN_RAW, CAN_RAW_FILTER,
	    &filt, sizeof(filt)) == 0);
	/* inverted: everything EXCEPT 0x200 matches */
	send_classic(a, 0x200, 0x01);
	ATF_CHECK_MSG(got_frame(b, NULL) == 0, "inv filter let 0x200 through");
	send_classic(a, 0x201, 0x02);
	ATF_CHECK_MSG(got_frame(b, NULL) == 1, "inv filter dropped 0x201");
	close(a);
	close(b);
}

ATF_TC_WITHOUT_HEAD(sockopt_roundtrip);
ATF_TC_BODY(sockopt_roundtrip, tc)
{
	int s = socket(AF_CAN, SOCK_RAW, CAN_RAW);
	int on = 1, v = -1;
	socklen_t vlen = sizeof(v);

	ATF_REQUIRE(s >= 0);
	ATF_REQUIRE(setsockopt(s, SOL_CAN_RAW, CAN_RAW_JOIN_FILTERS,
	    &on, sizeof(on)) == 0);
	ATF_REQUIRE(getsockopt(s, SOL_CAN_RAW, CAN_RAW_JOIN_FILTERS,
	    &v, &vlen) == 0);
	ATF_CHECK_EQ(1, v);
	ATF_CHECK_EQ(sizeof(int), vlen);
	/* unknown option -> ENOPROTOOPT */
	ATF_CHECK_ERRNO(ENOPROTOOPT,
	    setsockopt(s, SOL_CAN_RAW, 999, &on, sizeof(on)) != 0);
	close(s);
}

ATF_TC_WITHOUT_HEAD(fd_frames);
ATF_TC_BODY(fd_frames, tc)
{
	unsigned int idx = vcan_up();
	int a = can_bound(idx, 1);
	int b = can_bound(idx, 1);
	struct canfd_frame fd, rx;
	int on = 1;
	ssize_t n;

	/* FD tx without the option is rejected. */
	memset(&fd, 0, sizeof(fd));
	fd.can_id = 0x444;
	fd.len = 16;
	ATF_CHECK_ERRNO(EINVAL, write(a, &fd, sizeof(fd)) < 0);

	ATF_REQUIRE(setsockopt(a, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
	    &on, sizeof(on)) == 0);
	ATF_REQUIRE(setsockopt(b, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
	    &on, sizeof(on)) == 0);

	memset(&fd, 0, sizeof(fd));
	fd.can_id = 0x555;
	fd.len = 16;
	fd.flags = CANFD_BRS;
	memset(fd.data, 0x5a, fd.len);
	ATF_REQUIRE(write(a, &fd, sizeof(fd)) == (ssize_t)sizeof(fd));

	memset(&rx, 0, sizeof(rx));
	n = read(b, &rx, sizeof(rx));
	ATF_CHECK_EQ_MSG((ssize_t)sizeof(rx), n, "FD frame not delivered");
	ATF_CHECK_EQ(0x555, rx.can_id);
	ATF_CHECK_EQ(16, rx.len);
	ATF_CHECK(rx.data[0] == 0x5a && rx.data[15] == 0x5a);
	ATF_CHECK_MSG((rx.flags & CANFD_FDF) != 0, "CANFD_FDF not set");
	close(a);
	close(b);
}

ATF_TC_WITHOUT_HEAD(fd_gating);
ATF_TC_BODY(fd_gating, tc)
{
	unsigned int idx = vcan_up();
	int a = can_bound(idx, 1);	/* FD sender */
	int b = can_bound(idx, 1);	/* classic-only receiver */
	struct canfd_frame fd;
	int on = 1;

	ATF_REQUIRE(setsockopt(a, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
	    &on, sizeof(on)) == 0);
	memset(&fd, 0, sizeof(fd));
	fd.can_id = 0x666;
	fd.len = 32;
	ATF_REQUIRE(write(a, &fd, sizeof(fd)) == (ssize_t)sizeof(fd));
	ATF_CHECK_MSG(got_frame(b, NULL) == 0,
	    "classic-only socket received an FD frame");
	close(a);
	close(b);
}

ATF_TC_WITHOUT_HEAD(send_down_interface);
ATF_TC_BODY(send_down_interface, tc)
{
	struct sockaddr_can addr;
	struct can_frame f;
	unsigned int idx;
	int cs;

	/* Create vcan0 but leave it DOWN. */
	idx = vcan_create(0);

	ATF_REQUIRE((cs = socket(AF_CAN, SOCK_RAW, CAN_RAW)) >= 0);
	memset(&addr, 0, sizeof(addr));
	addr.can_len = sizeof(addr);
	addr.can_family = AF_CAN;
	addr.can_ifindex = idx;
	ATF_REQUIRE(bind(cs, (struct sockaddr *)&addr, sizeof(addr)) == 0);

	memset(&f, 0, sizeof(f));
	f.can_id = 0x1;
	f.len = 1;
	ATF_CHECK_ERRNO(ENETDOWN, write(cs, &f, sizeof(f)) < 0);
	close(cs);
}

ATF_TP_ADD_TCS(tp)
{
	ATF_TP_ADD_TC(tp, socket_open);
	ATF_TP_ADD_TC(tp, bind_bad_ifindex);
	ATF_TP_ADD_TC(tp, loopback);
	ATF_TP_ADD_TC(tp, recv_own_msgs);
	ATF_TP_ADD_TC(tp, loopback_disabled);
	ATF_TP_ADD_TC(tp, filter);
	ATF_TP_ADD_TC(tp, filter_inverted);
	ATF_TP_ADD_TC(tp, sockopt_roundtrip);
	ATF_TP_ADD_TC(tp, fd_frames);
	ATF_TP_ADD_TC(tp, fd_gating);
	ATF_TP_ADD_TC(tp, send_down_interface);

	return (atf_no_error());
}
