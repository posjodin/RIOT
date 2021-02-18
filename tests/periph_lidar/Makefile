include ../Makefile.tests_common

FEATURES_REQUIRED = periph_i2c
FEATURES_OPTIONAL += periph_rtt
FEATURES_REQUIRED += periph_uart
FEATURES_OPTIONAL += periph_lpuart  # STM32 L0 and L4 provides lpuart support
USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += ps
USEMODULE += i2c_scan
USEMODULE += xtimer

CFLAGS += -DI2C_NUMOF=1U -DI2C_BUS_SPEED=I2C_SPEED_NORMAL

include $(RIOTBASE)/Makefile.include
