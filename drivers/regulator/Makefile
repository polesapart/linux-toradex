#
# Makefile for regulator drivers.
#

obj-$(CONFIG_REGULATOR) += reg-core.o 

ifeq ($(CONFIG_REGULATOR_DEBUG),y)
	EXTRA_CFLAGS		+= -DDEBUG
endif


