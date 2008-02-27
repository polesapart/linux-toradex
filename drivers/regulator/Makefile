#
# Makefile for regulator drivers.
#

obj-$(CONFIG_REGULATOR) += reg-core.o 

ifeq ($(CONFIG_REGULATOR_DEBUG),y)
	EXTRA_CFLAGS		+= -DDEBUG
endif

obj-$(CONFIG_REGULATOR_MC13783)  += mc13783/
obj-$(CONFIG_REGULATOR_WM8350)  += wm8350/