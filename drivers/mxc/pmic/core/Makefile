#
# Makefile for the PMIC core drivers.
#
obj-$(CONFIG_MXC_SPI_PMIC_CORE) += pmic_core_spi_mod.o

obj-$(CONFIG_MXC_I2C_MCU_PMIC_CORE) += pmic_core_i2c_mod.o

pmic_core_i2c_mod-objs := pmic_external.o pmic_event.o mcu_pmic_core.o

obj-$(CONFIG_MXC_PMIC_CHARDEV)	+= pmic-dev.o

pmic_core_spi_mod-objs := pmic_external.o pmic_event.o pmic_core_spi.o

ifeq ($(CONFIG_MXC_PMIC_MC13783),y)
pmic_core_spi_mod-objs += mc13783.o
endif

ifeq ($(CONFIG_MXC_PMIC_SC55112),y)
pmic_core_spi_mod-objs += sc55112.o
endif

ifeq ($(CONFIG_MXC_PMIC_MC9SDZ60), y)
pmic_core_i2c_mod-objs += mc9sdz60.o
pmic_core_i2c_mod-objs += max8660.o
endif
