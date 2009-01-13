#
# Makefile for the kernel FIM device drivers
#

obj-$(CONFIG_FIM_SDIO)		+= fim_sdio.o


# -- If a PIC number is defined than pass it to the drivers
ifneq ($(CONFIG_FIM_SDIO_NUMBER),)
EXTRA_CFLAGS 			+= -DFIM_SDIO_NUMBER=$(CONFIG_FIM_SDIO_NUMBER)
endif

