/*
 * Copyright 2008 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file mc9sdz60/mcu_pmic_power.c
 * @brief This is the main file of mc9sdz60 Power Control driver.
 * @This driver is deprecated, replaced by Wolfson regulator architecture
 * @ingroup PMIC_POWER
 */

/*
 * Includes
 */
#include <linux/platform_device.h>
#include <asm/ioctl.h>
#include <asm/arch/pmic_status.h>
#include <asm/arch/pmic_external.h>
#include <asm/arch/pmic_power.h>
#include "../core/pmic_config.h"

/*
 * PMIC Power Control API
 */


/*!
 * This is the suspend of power management for Power Control
 * API module.
 *
 * @param   pdev  the device structure used to give information on which power
 *                device (0 through 3 channels) to suspend
 * @param   state the power state the device is entering
 *
 * @return  The function always returns 0.
 */
static int pmic_power_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
};

/*!
 * This is the resume of power management API.
 * It suports RESTORE state.
 *
 * @param   pdev  the device structure used to give information on which power
 *                device (0 through 3 channels) to suspend
 *
 * @return  The function always returns 0.
 */
static int pmic_power_resume(struct platform_device *pdev)
{
	return 0;
};

/*!
 * This function sets user power off in power control register and thus powers
 * off the phone.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_off(void)
{
#define POWER_OFF_DELAY_MS 200
	u8 mask = 0;
	CHECK_ERROR(pmic_write_reg
		    (REG_MCU_DELAY_CONFIG, POWER_OFF_DELAY_MS, 0xff));
	SET_BIT_IN_BYTE(mask, 4);
	CHECK_ERROR(pmic_write_reg(REG_MCU_POWER_CTL, mask, mask));

	return PMIC_SUCCESS;
}
EXPORT_SYMBOL(pmic_power_off);

/*!
 * This function turns on a regulator.
 *
 * @param        regulator    The regulator to be truned on.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_regulator_on(t_pmic_regulator regulator)
{
	u8 reg_mask = 0;
	int reg_num;

	if (regulator > MCU_LDO7 || regulator < MCU_SW1)
		return PMIC_NOT_SUPPORTED;

	switch (regulator) {
	case MCU_SW1:
		SET_BIT_IN_BYTE(reg_mask, 0);
		reg_num = REG_MCU_POWER_CTL;
		break;
	case MCU_SW2:
		SET_BIT_IN_BYTE(reg_mask, 1);
		reg_num = REG_MCU_POWER_CTL;
		break;
	case MCU_SW3:
		SET_BIT_IN_BYTE(reg_mask, 0);
		reg_num = REG_MAX8660_OUTPUT_ENABLE_1;
		break;
	case MCU_SW4:
		SET_BIT_IN_BYTE(reg_mask, 2);
		reg_num = REG_MAX8660_OUTPUT_ENABLE_1;
		break;
	case MCU_LDO5:
		SET_BIT_IN_BYTE(reg_mask, 3);
		reg_num = REG_MCU_POWER_CTL;
		break;
	case MCU_LDO6:
		SET_BIT_IN_BYTE(reg_mask, 1);
		reg_num = REG_MAX8660_OUTPUT_ENABLE_2;
		break;
	case MCU_LDO7:
		SET_BIT_IN_BYTE(reg_mask, 2);
		reg_num = REG_MAX8660_OUTPUT_ENABLE_2;
		break;

	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(reg_num, reg_mask, reg_mask));

	return PMIC_SUCCESS;
}
EXPORT_SYMBOL(pmic_power_regulator_on);

/*!
 * This function turns off a regulator.
 *
 * @param        regulator    The regulator to be truned off.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_regulator_off(t_pmic_regulator regulator)
{
	u8 reg_mask = 0;
	unsigned int reg_read = 0;
	int reg_num;

	if (regulator > MCU_LDO7 || regulator < MCU_SW1)
		return PMIC_NOT_SUPPORTED;

	switch (regulator) {
	case MCU_SW1:
		SET_BIT_IN_BYTE(reg_mask, 0);
		reg_num = REG_MCU_POWER_CTL;
		break;
	case MCU_SW2:
		SET_BIT_IN_BYTE(reg_mask, 1);
		reg_num = REG_MCU_POWER_CTL;
		break;
	case MCU_SW3:
		SET_BIT_IN_BYTE(reg_mask, 0);
		reg_num = REG_MAX8660_OUTPUT_ENABLE_1;
		break;
	case MCU_SW4:
		SET_BIT_IN_BYTE(reg_mask, 2);
		reg_num = REG_MAX8660_OUTPUT_ENABLE_1;
		break;
	case MCU_LDO5:
		SET_BIT_IN_BYTE(reg_mask, 3);
		reg_num = REG_MCU_POWER_CTL;
		break;
	case MCU_LDO6:
		SET_BIT_IN_BYTE(reg_mask, 1);
		reg_num = REG_MAX8660_OUTPUT_ENABLE_2;
		break;
	case MCU_LDO7:
		SET_BIT_IN_BYTE(reg_mask, 2);
		reg_num = REG_MAX8660_OUTPUT_ENABLE_2;
		break;

	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(reg_num, 0, reg_mask));

	/* handle sw3,4 */
	switch (regulator) {
	case MCU_SW3:
		reg_mask = 0;
		SET_BIT_IN_BYTE(reg_mask, 2);
		CHECK_ERROR(pmic_read_reg
			    (REG_MCU_POWER_CTL, &reg_read, reg_mask));

		/* check if hw pin enable sw34 */
		if (0 != reg_read) {

			/* keep sw4 on */
			reg_mask = 0;
			SET_BIT_IN_BYTE(reg_mask, 2);
			CHECK_ERROR(pmic_write_reg
				    (REG_MAX8660_OUTPUT_ENABLE_1, reg_mask,
				     reg_mask));

			/* disable hw pin to actually turn off sw3 */
			reg_mask = 0;
			SET_BIT_IN_BYTE(reg_mask, 2);
			CHECK_ERROR(pmic_write_reg
				    (REG_MCU_POWER_CTL, 0, reg_mask));
		}
		break;
	case MCU_SW4:
		reg_mask = 0;
		SET_BIT_IN_BYTE(reg_mask, 2);
		CHECK_ERROR(pmic_read_reg
			    (REG_MCU_POWER_CTL, &reg_read, reg_mask));

		/* check if hw pin enable sw34 */
		if (0 != reg_read) {

			/* keep sw3 on */
			reg_mask = 0;
			SET_BIT_IN_BYTE(reg_mask, 0);
			CHECK_ERROR(pmic_write_reg
				    (REG_MAX8660_OUTPUT_ENABLE_1, reg_mask,
				     reg_mask));

			/* disable hw pin to actually turn off sw4 */
			reg_mask = 0;
			SET_BIT_IN_BYTE(reg_mask, 2);
			CHECK_ERROR(pmic_write_reg
				    (REG_MCU_POWER_CTL, 0, reg_mask));
		}
		break;
	default:
		break;
	}
	return PMIC_SUCCESS;
}
EXPORT_SYMBOL(pmic_power_regulator_off);

/*!
 * This function sets the regulator output voltage.
 *
 * @param        regulator    The regulator to be truned off.
 * @param        voltage      The regulator output voltage.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_regulator_set_voltage(t_pmic_regulator regulator,
					     t_regulator_voltage voltage)
{
	u8 reg_mask = 0;

	if (regulator > MCU_LDO7 || regulator < MCU_SW3)
		return PMIC_NOT_SUPPORTED;

	switch (regulator) {
	case MCU_SW3:
		if ((voltage.mcu_sw34 < MCU_SW34_0_725V)
		    || voltage.mcu_sw34 > MCU_SW34_1_8V)
			return PMIC_PARAMETER_ERROR;

		/* hold on */
		SET_BIT_IN_BYTE(reg_mask, 0);
		CHECK_ERROR(pmic_write_reg
			    (REG_MAX8660_VOLT_CHANGE_CONTROL_1, 0, reg_mask));

		/* set volt */
		CHECK_ERROR(pmic_write_reg
			    (REG_MAX8660_V3_TARGET_VOLT_1, voltage.mcu_sw34,
			     0xff));

		/* start ramp */
		SET_BIT_IN_BYTE(reg_mask, 0);
		CHECK_ERROR(pmic_write_reg
			    (REG_MAX8660_VOLT_CHANGE_CONTROL_1, reg_mask,
			     reg_mask));
		break;
	case MCU_SW4:
		if ((voltage.mcu_sw34 < MCU_SW34_0_725V)
		    || voltage.mcu_sw34 > MCU_SW34_1_8V)
			return PMIC_PARAMETER_ERROR;

		/* hold on */
		SET_BIT_IN_BYTE(reg_mask, 4);
		CHECK_ERROR(pmic_write_reg
			    (REG_MAX8660_VOLT_CHANGE_CONTROL_1, 0, reg_mask));

		/* set volt */
		CHECK_ERROR(pmic_write_reg
			    (REG_MAX8660_V4_TARGET_VOLT_1, voltage.mcu_sw34,
			     0xff));

		/* start ramp */
		SET_BIT_IN_BYTE(reg_mask, 4);
		CHECK_ERROR(pmic_write_reg
			    (REG_MAX8660_VOLT_CHANGE_CONTROL_1, reg_mask,
			     reg_mask));
		break;
	case MCU_LDO5:
		if (voltage.mcu_ldo5 < MCU_LDO5_1_725V
		    || voltage.mcu_ldo5 > MCU_LDO5_2_0V)
			return PMIC_PARAMETER_ERROR;

		/* hold on */
		SET_BIT_IN_BYTE(reg_mask, 6);
		CHECK_ERROR(pmic_write_reg
			    (REG_MAX8660_VOLT_CHANGE_CONTROL_1, 0, reg_mask));

		/* set volt */
		CHECK_ERROR(pmic_write_reg
			    (REG_MAX8660_V5_TARGET_VOLT_1, voltage.mcu_ldo5,
			     0xff));

		/* start ramp */
		SET_BIT_IN_BYTE(reg_mask, 6);
		CHECK_ERROR(pmic_write_reg
			    (REG_MAX8660_VOLT_CHANGE_CONTROL_1, reg_mask,
			     reg_mask));
		break;

		break;
	case MCU_LDO6:
		if (voltage.mcu_ldo67 < MCU_LDO67_1_8V
		    || voltage.mcu_ldo67 > MCU_LDO67_3_3V)
			return PMIC_PARAMETER_ERROR;

		/* set volt */
		CHECK_ERROR(pmic_write_reg
			    (REG_MAX8660_V6V7_TARGET_VOLT, voltage.mcu_ldo67,
			     0x0f));

		break;
	case MCU_LDO7:
		if (voltage.mcu_ldo67 < MCU_LDO67_1_8V
		    || voltage.mcu_ldo67 > MCU_LDO67_3_3V)
			return PMIC_PARAMETER_ERROR;
		break;

		/* set volt */
		CHECK_ERROR(pmic_write_reg
			    (REG_MAX8660_V6V7_TARGET_VOLT,
			     (voltage.mcu_ldo67 << 4), 0xf0));

	default:
		return PMIC_PARAMETER_ERROR;
	}

	return PMIC_SUCCESS;
}
EXPORT_SYMBOL(pmic_power_regulator_set_voltage);

/*!
 * This function retrives the regulator output voltage.
 *
 * @param        regulator    The regulator to be truned off.
 * @param        voltage      Pointer to regulator output voltage.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_regulator_get_voltage(t_pmic_regulator regulator,
					     t_regulator_voltage *voltage)
{
	unsigned int reg_val = 0;

	if (regulator > MCU_LDO7 || regulator < MCU_SW3)
		return PMIC_NOT_SUPPORTED;

	switch (regulator) {
	case MCU_SW3:
		CHECK_ERROR(pmic_read_reg
			    (REG_MAX8660_V3_TARGET_VOLT_1, &reg_val, 0xff));
		voltage->mcu_sw34 = reg_val;
		break;
	case MCU_SW4:
		CHECK_ERROR(pmic_read_reg
			    (REG_MAX8660_V4_TARGET_VOLT_1, &reg_val, 0xff));
		voltage->mcu_sw34 = reg_val;
		break;
	case MCU_LDO5:
		CHECK_ERROR(pmic_read_reg
			    (REG_MAX8660_V5_TARGET_VOLT_1, &reg_val, 0xff));
		voltage->mcu_ldo5 = reg_val;
		break;
	case MCU_LDO6:
		CHECK_ERROR(pmic_read_reg
			    (REG_MAX8660_V6V7_TARGET_VOLT, &reg_val, 0x0f));
		voltage->mcu_ldo67 = reg_val;
		break;
	case MCU_LDO7:
		CHECK_ERROR(pmic_read_reg
			    (REG_MAX8660_V6V7_TARGET_VOLT, &reg_val, 0xf0));
		voltage->mcu_ldo67 = (reg_val >> 4) & 0xf;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}
	return PMIC_SUCCESS;
}
EXPORT_SYMBOL(pmic_power_regulator_get_voltage);

/*
 * Initialization and Exit
 */

PMIC_STATUS pmic_power_regulator_init()
{
	t_regulator_voltage volt;
	volt.mcu_sw34 = MCU_SW34_1_2V;
	CHECK_ERROR(pmic_power_regulator_set_voltage(MCU_SW3, volt));
	volt.mcu_sw34 = MCU_SW34_1_5V;
	CHECK_ERROR(pmic_power_regulator_set_voltage(MCU_SW4, volt));
	volt.mcu_ldo5 = MCU_LDO5_1_8V;
	CHECK_ERROR(pmic_power_regulator_set_voltage(MCU_LDO5, volt));
	volt.mcu_ldo67 = MCU_LDO67_2_5V;
	CHECK_ERROR(pmic_power_regulator_set_voltage(MCU_LDO6, volt));
	volt.mcu_ldo67 = MCU_LDO67_2_8V;
	CHECK_ERROR(pmic_power_regulator_set_voltage(MCU_LDO7, volt));

	return PMIC_SUCCESS;
}

static int pmic_power_probe(struct platform_device *pdev)
{
	printk(KERN_INFO "PMIC Power successfully probed\n");
	pmic_power_regulator_init();
	return 0;
}

static struct platform_driver pmic_power_driver_ldm = {
	.driver = {
		   .name = "pmic_power",
		   },
	.suspend = pmic_power_suspend,
	.resume = pmic_power_resume,
	.probe = pmic_power_probe,
	.remove = NULL,
};

static int __init pmic_power_init(void)
{
	pr_debug("PMIC Power driver loading..\n");
	return platform_driver_register(&pmic_power_driver_ldm);
}
static void __exit pmic_power_exit(void)
{
	platform_driver_unregister(&pmic_power_driver_ldm);
	pr_debug("PMIC Power driver successfully unloaded\n");
}

/*
 * Module entry points
 */

subsys_initcall(pmic_power_init);
module_exit(pmic_power_exit);

MODULE_DESCRIPTION("PMIC Power Control device driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
