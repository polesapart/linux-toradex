#
# Makefile for PMIC drivers .
#

ifeq ($(CONFIG_PMIC_DEBUG),y)
 EXTRA_CFLAGS  += -DDEBUG
endif

obj-$(CONFIG_PMIC) += pmic.o

obj-$(CONFIG_PMIC_WM8350) += \
 pmic-wm8350.o pmic-wm8350-bus.o pmic-wm8350-i2c.o


