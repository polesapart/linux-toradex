# arch/arm/mach-s3c2443/Makefile
#
# Copyright 2007 Simtec Electronics
#
# Licensed under GPLv2

obj-y				:=
obj-m				:=
obj-n				:=
obj-				:=

obj-$(CONFIG_CPU_S3C2443)	+= s3c2443.o
obj-$(CONFIG_CPU_S3C2443)	+= irq.o
obj-$(CONFIG_CPU_S3C2443)	+= clock.o

obj-$(CONFIG_S3C2443_DMA)	+= dma.o

# Machine support

obj-$(CONFIG_MACH_SMDK2443)	+= mach-smdk2443.o
obj-$(CONFIG_MACH_CC9M2443JS)   += mach-cc9m2443js.o

obj-$(CONFIG_MACH_CC9M2443JS)   += gpio.o

# Support for the USB overload protection
obj-$(CONFIG_MACH_CC9M2443JS_USB) += cc9m2443js-usb.o

# Power Management support
obj-$(CONFIG_MACH_CC9M2443JS_PM) += cc9m2443js-pm.o cc9m2443js-sleep.o
