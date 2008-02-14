#
# Makefile for the linux kernel.
#

# Object file lists.

obj-y			:= system.o iomux.o cpu.o mm.o clock.o devices.o serial.o irq.o dma.o

obj-$(CONFIG_MACH_MX33ADS)	+= mx33ads.o mx33ads_gpio.o

# power management
obj-$(CONFIG_MXC_DVFS)  += dvfs.o
