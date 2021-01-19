include ../Makefile.tests_common

FEATURES_REQUIRED += periph_uart
FEATURES_OPTIONAL += periph_lpuart  # STM32 L0 and L4 provides lpuart support

USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += ps
USEMODULE += i2c_scan
USEMODULE += xtimer

FEATURES_REQUIRED = periph_i2c
FEATURES_OPTIONAL += periph_rtt

# We use floating vars. Add library.
PRINTF_LIB_FLT  = -Wl,-u,vfprintf -lprintf_flt -lm
PRINTF_LIB_MIN  = -Wl,-u,vfprintf -lprintf_min
PRINTF_LIB      = $(PRINTF_LIB_FLT)
BASELIBS           = $(PRINTF_LIB) 

CFLAGS += -DI2C_NUMOF=1U -DI2C_BUS_SPEED=I2C_SPEED_NORMAL

include $(RIOTBASE)/Makefile.include
