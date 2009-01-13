#
# Makefile for the kernel FIM device drivers
#

obj-$(CONFIG_FIM_SERIAL)	+= fim_serial.o


# -- If a PIC number is defined than pass it to the drivers
ifneq ($(CONFIG_FIM_SERIAL_NUMBER),)
EXTRA_CFLAGS 			+= -DFIM_SERIAL_NUMBER=$(CONFIG_FIM_SERIAL_NUMBER)
endif

ifneq ($(CONFIG_FIM_SERIAL_NUMBER0),)
EXTRA_CFLAGS 			+= -DFIM_SERIAL_NUMBER=0
endif

ifneq ($(CONFIG_FIM_SERIAL_NUMBER1),)
EXTRA_CFLAGS 			+= -DFIM_SERIAL_NUMBER=1
endif

