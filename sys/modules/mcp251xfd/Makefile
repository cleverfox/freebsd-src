.PATH:	${SRCTOP}/sys/dev/mcp251xfd

KMOD=	mcp251xfd
SRCS=	if_mcp251xfd.c

# Generated headers.
SRCS+=	\
	bus_if.h \
	device_if.h \
	ofw_bus_if.h \
	opt_platform.h \
	spibus_if.h \

.include <bsd.kmod.mk>
