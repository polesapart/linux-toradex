#
# Makefile for the kernel FIM device drivers
#

obj-$(CONFIG_FIM_ONE_CAN)		+= fim_can.o
obj-$(CONFIG_FIM_TWO_CAN)		+= fim_can.o

#
# SUPERCEDED stuff !! Remove in the future
#
# -- If a PIC number is defined than pass it to the drivers
## ifneq ($(CONFIG_FIM_CAN_NUMBER),)
## EXTRA_CFLAGS 			+= -DFIM_CAN_NUMBER=$(CONFIG_FIM_CAN_NUMBER)
## endif
## 
## ifneq ($(CONFIG_FIM_CAN_NUMBER0),)
## EXTRA_CFLAGS                    += -DFIM_CAN_NUMBER=0
## endif
## 
## ifneq ($(CONFIG_FIM_CAN_NUMBER1),)
## EXTRA_CFLAGS                    += -DFIM_CAN_NUMBER=1
## endif
## 
## 
## # -- Initial bit rate when compiled as built-in
## ifneq ($(CONFIG_FIM_CAN_BITRATE),)
## EXTRA_CFLAGS 			+= -DFIM_CAN_BITRATE=$(CONFIG_FIM_CAN_BITRATE)
## endif
## 
