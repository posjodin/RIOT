include ../Makefile.tests_common

FEATURES_REQUIRED += periph_uart
FEATURES_OPTIONAL += periph_lpuart  # STM32 L0 and L4 provides lpuart support

USEMODULE += shell
USEMODULE += xtimer
FEATURES_OPTIONAL += periph_rtt

# We use floating vars. Add library.
PRINTF_LIB_FLT  = -Wl,-u,vfprintf -lprintf_flt -lm
PRINTF_LIB_MIN  = -Wl,-u,vfprintf -lprintf_min
PRINTF_LIB      = $(PRINTF_LIB_FLT)
BASELIBS           = $(PRINTF_LIB) 

include $(RIOTBASE)/Makefile.include
