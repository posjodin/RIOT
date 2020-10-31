MODULE = sim7020
ifneq (,$(filter sim7020_sock_udp,$(USEMODULE)))
  DIRS += sock/udp
endif
ifneq (,$(filter sim7020_sock_dns,$(USEMODULE)))
  DIRS += sock/dns
endif

include $(RIOTBASE)/Makefile.base
