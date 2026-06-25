PACKAGE=	tests

TESTSDIR=	${TESTSBASE}/sys/netcan

ATF_TESTS_C+=	can_raw_test

# Each test runs in its own vnet jail so it can freely create a vcan(4)
# interface and open CAN raw sockets in isolation.
TEST_METADATA+=	required_user="root"
TEST_METADATA+=	required_kmods="can"
TEST_METADATA+=	execenv="jail"
TEST_METADATA+=	execenv_jail_params="vnet allow.raw_sockets"

.include <bsd.test.mk>
