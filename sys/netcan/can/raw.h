/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2002-2007 Volkswagen Group Electronic Research
 * All rights reserved.
 *
 * Copyright (c) 2026 Vladimir Goncharov <devel@viruzzz.org>
 *
 * Authors: Oliver Hartkopp <oliver.hartkopp@volkswagen.de>
 *          Urs Thuermann   <urs.thuermann@volkswagen.de>
 *
 * Derived from the Linux SocketCAN UAPI headers, used under the BSD-3-Clause
 * option of their dual (GPL-2.0-only OR BSD-3-Clause) license.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of Volkswagen nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Definitions for raw CAN sockets.  Option level and option numbers match
 * Linux (include/uapi/linux/can/raw.h) so that setsockopt(2)/getsockopt(2)
 * source code is portable between Linux and FreeBSD.
 */

#ifndef _NETCAN_CAN_RAW_H_
#define _NETCAN_CAN_RAW_H_

#include <netcan/can.h>

#define SOL_CAN_RAW		(SOL_CAN_BASE + CAN_RAW)	/* == 101 */
#define CAN_RAW_FILTER_MAX	512	/* max filters via setsockopt */

/* for socket options affecting the socket (not the global system) */
enum {
	CAN_RAW_FILTER = 1,	/* set 0 .. n can_filter(s)          */
	CAN_RAW_ERR_FILTER,	/* set filter for error frames       */
	CAN_RAW_LOOPBACK,	/* local loopback (default:on)       */
	CAN_RAW_RECV_OWN_MSGS,	/* receive my own msgs (default:off) */
	CAN_RAW_FD_FRAMES,	/* allow CAN FD frames (default:off) */
	CAN_RAW_JOIN_FILTERS,	/* all filters must match to trigger */
	CAN_RAW_XL_FRAMES,	/* allow CAN XL frames (default:off) */
	CAN_RAW_XL_VCID_OPTS,	/* CAN XL VCID configuration options */
};

#endif /* !_NETCAN_CAN_RAW_H_ */
