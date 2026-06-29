.PATH:	${SRCTOP}/sys/dev/mcp2515

KMOD=	mcp2515
SRCS=	if_mcp2515.c

# Generated headers.
SRCS+=	\
	bus_if.h \
	device_if.h \
	ofw_bus_if.h \
	opt_platform.h \
	spibus_if.h \

.include <bsd.kmod.mk>
