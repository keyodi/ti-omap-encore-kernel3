/*
 * Copyright (C) 2009 Texas Instruments Inc.
 *
 * Modified from mach-omap2/board-zoom.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/wl12xx.h>
#include <linux/mmc/host.h>
#include <linux/switch.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/common.h>
#include <plat/usb.h>
#include <plat/mmc.h>

#include <mach/board-encore.h>

#include <linux/cyttsp.h>
#include <linux/input/ft5x06.h>
#include <linux/input/kxtf9.h>
#include <linux/power/max17042.h>

#include "mux.h"
#include "hsmmc.h"
#include "common-board-devices.h"
#include "twl4030.h"

#define KXTF9_DEVICE_ID                 "kxtf9"
#define KXTF9_I2C_SLAVE_ADDRESS         0x0F
#define KXTF9_GPIO_FOR_PWR              34

#define CYTTSP_I2C_SLAVEADDRESS  	34
#define OMAP_CYTTSP_GPIO         	99
#define OMAP_CYTTSP_RESET_GPIO   	46

#define FT5x06_I2C_SLAVEADDRESS  	(0x70 >> 1)
#define OMAP_FT5x06_GPIO         	99
#define OMAP_FT5x06_RESET_GPIO   	46

#define MAX17042_GPIO_FOR_IRQ 		100
#define KXTF9_GPIO_FOR_IRQ  		113

#define OMAP_ENCORE_WLAN_PMENA_GPIO	22

#ifdef CONFIG_BATTERY_MAX17042
static void max17042_dev_init(void)
{
	printk("board-3621_evt1a.c: max17042_dev_init ...\n");

	if (gpio_request(MAX17042_GPIO_FOR_IRQ, "max17042_irq") < 0) {
		printk(KERN_ERR "Can't get GPIO for max17042 IRQ\n");
		return;
	}

	printk("board-3621_evt1a.c: max17042_dev_init > Init max17042 irq pin %d !\n", MAX17042_GPIO_FOR_IRQ);
	gpio_direction_input(MAX17042_GPIO_FOR_IRQ);
	printk("max17042 GPIO pin read %d\n", gpio_get_value(MAX17042_GPIO_FOR_IRQ));
}
#endif

static void kxtf9_dev_init(void)
{
	printk("board-3621_evt1a.c: kxtf9_dev_init ...\n");

	if (gpio_request(KXTF9_GPIO_FOR_IRQ, "kxtf9_irq") < 0)
	{
		printk("Can't get GPIO for kxtf9 IRQ\n");
		return;
	}

	printk("board-3621_evt1a.c: kxtf9_dev_init > Init kxtf9 irq pin %d !\n",
			KXTF9_GPIO_FOR_IRQ);
	gpio_direction_input(KXTF9_GPIO_FOR_IRQ);
}


struct kxtf9_platform_data kxtf9_platform_data_here = {
	.min_interval   = 1,
	.poll_interval  = 1000,

	.g_range        = KXTF9_G_8G,
	.shift_adj      = SHIFT_ADJ_2G,

	// Map the axes from the sensor to the device.

	//. SETTINGS FOR THE EVT1A:
	.axis_map_x     = 1,
	.axis_map_y     = 0,
	.axis_map_z     = 2,
	.negate_x       = 1,
	.negate_y       = 0,
	.negate_z       = 0,
	.data_odr_init          = ODR12_5F,
	.ctrl_reg1_init         = KXTF9_G_8G | RES_12BIT | TDTE | WUFE | TPE,
	.int_ctrl_init          = KXTF9_IEN | KXTF9_IEA | KXTF9_IEL,
	.int_ctrl_init          = KXTF9_IEN,
	.tilt_timer_init        = 0x03,
	.engine_odr_init        = OTP12_5 | OWUF50 | OTDT400,
	.wuf_timer_init         = 0x16,
	.wuf_thresh_init        = 0x28,
	.tdt_timer_init         = 0x78,
	.tdt_h_thresh_init      = 0xFF,
	.tdt_l_thresh_init      = 0x14,
	.tdt_tap_timer_init     = 0x53,
	.tdt_total_timer_init   = 0x24,
	.tdt_latency_timer_init = 0x10,
	.tdt_window_timer_init  = 0xA0,

	.gpio = KXTF9_GPIO_FOR_IRQ,
};

int ft5x06_dev_init(int resource)
{
	if (resource) {
		if (gpio_request(OMAP_FT5x06_RESET_GPIO, "ft5x06_reset") < 0) {
			printk(KERN_ERR "can't get ft5x06 xreset GPIO\n");
			return -1;
		}

		if (gpio_request(OMAP_FT5x06_GPIO, "ft5x06_touch") < 0) {
			printk(KERN_ERR "can't get ft5x06 interrupt GPIO\n");
			return -1;
		}

		gpio_direction_input(OMAP_FT5x06_GPIO);
	} else {
		gpio_free(OMAP_FT5x06_GPIO);
		gpio_free(OMAP_FT5x06_RESET_GPIO);
	}

	return 0;
}

static struct ft5x06_platform_data ft5x06_platform_data = {
	.maxx = 600,
	.maxy = 1024,
	.flags = 0,
	.reset_gpio = OMAP_FT5x06_RESET_GPIO,
	.use_st = FT_USE_ST,
	.use_mt = FT_USE_MT,
	.use_trk_id = 1, //FT_USE_TRACKING_ID,
	.use_sleep = FT_USE_SLEEP,
	.use_gestures = 0,
};

int cyttsp_dev_init(int resource)
{
	if (resource) {
		if (gpio_request(OMAP_CYTTSP_RESET_GPIO, "tma340_reset") < 0) {
			printk(KERN_ERR "can't get tma340 xreset GPIO\n");
			return -1;
		}

		if (gpio_request(OMAP_CYTTSP_GPIO, "cyttsp_touch") < 0) {
			printk(KERN_ERR "can't get cyttsp interrupt GPIO\n");
			return -1;
		}

		gpio_direction_input(OMAP_CYTTSP_GPIO);
		/* omap_set_gpio_debounce(OMAP_CYTTSP_GPIO, 0); */

	} else {
		gpio_free(OMAP_CYTTSP_GPIO);
		gpio_free(OMAP_CYTTSP_RESET_GPIO);
	}

	return 0;
}

static struct cyttsp_platform_data cyttsp_platform_data = {
        .maxx = 480,
        .maxy = 800,
        .flags = 0,
        .gen = CY_GEN3,
        .use_st = CY_USE_ST,
        .use_mt = CY_USE_MT,
        .use_hndshk = CY_SEND_HNDSHK,
        .use_trk_id = 1, //CY_USE_TRACKING_ID,
        .use_sleep = CY_USE_SLEEP,
        .use_gestures = 0,
        /* activate up to 4 groups
         * and set active distance
         */
        .gest_set = 0,
        /* change act_intrvl to customize the Active power state 
         * scanning/processing refresh interval for Operating mode
         */
        .act_intrvl = CY_ACT_INTRVL_DFLT,
        /* change tch_tmout to customize the touch timeout for the
         * Active power state for Operating mode
         */
        .tch_tmout = CY_TCH_TMOUT_DFLT,
        /* change lp_intrvl to customize the Low Power power state 
         * scanning/processing refresh interval for Operating mode
         */
        .lp_intrvl = CY_LP_INTRVL_DFLT,
};

/* Encore keymap*/
static uint32_t board_keymap[] = {
	KEY(0, 0, KEY_HOME),
	KEY(1, 0, KEY_VOLUMEUP),
	KEY(2, 0, KEY_VOLUMEDOWN),
};

static struct matrix_keymap_data board_map_data = {
	.keymap			= board_keymap,
	.keymap_size		= ARRAY_SIZE(board_keymap),
};

static struct twl4030_keypad_data encore_kp_twl4030_data = {
	.keymap_data	= &board_map_data,
	.rows		= 8,
	.cols		= 8,
	.rep		= 1,
};

static struct gpio_keys_button encore_gpio_buttons[] = {
	{
		.code			= KEY_POWER,
		.gpio			= 14,
		.desc			= "POWER",
		.active_low		= 0,
		.wakeup			= 1,
	},
	{
		.code			= KEY_HOME,
		.gpio			= 48,
		.desc			= "HOME",
		.active_low		= 1,
		.wakeup			= 1,
	},
};

static struct gpio_keys_platform_data encore_gpio_key_info = {
	.buttons	= encore_gpio_buttons,
	.nbuttons	= ARRAY_SIZE(encore_gpio_buttons),
};

static struct platform_device encore_keys_gpio = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
	.platform_data	= &encore_gpio_key_info,
	},
};

static struct __initdata twl4030_power_data encore_t2scripts_data = {
	.use_poweroff	= true,
};

static struct regulator_consumer_supply encore_vmmc1_supply = {
	.supply		= "vmmc",
};

static struct regulator_consumer_supply encore_vsim_supply = {
	.supply		= "vmmc_aux",
};

static struct regulator_consumer_supply encore_vmmc2_supply = {
	.supply		= "vmmc",
};

static struct regulator_consumer_supply encore_vmmc3_supply = {
	.supply		= "vmmc",
	.dev_name	= "omap_hsmmc.2",
};

/* VMMC1 for OMAP VDD_MMC1 (i/o) and MMC1 card */
static struct regulator_init_data encore_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &encore_vmmc1_supply,
};

/* VMMC2 for MMC2 card */
static struct regulator_init_data encore_vmmc2 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 1850000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &encore_vmmc2_supply,
};

/* VSIM for OMAP VDD_MMC1A (i/o for DAT4..DAT7) */
static struct regulator_init_data encore_vsim = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &encore_vsim_supply,
};

static struct regulator_init_data encore_vmmc3 = {
	.constraints = {
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies = &encore_vmmc3_supply,
};

static struct fixed_voltage_config encore_vwlan = {
	.supply_name		= "vwl1271",
	.microvolts		= 1800000, /* 1.8V */
	.gpio			= OMAP_ENCORE_WLAN_PMENA_GPIO,
	.startup_delay		= 70000, /* 70msec */
	.enable_high		= 1,
	.enabled_at_boot	= 0,
	.init_data		= &encore_vmmc3,
};

static struct regulator_consumer_supply encore_lcd_tp_supply[] = {
	{ .supply = "vtp" },
	{ .supply = "vlcd" },
};

static struct regulator_init_data encore_lcd_tp_vinit = {
	.constraints = {
		.min_uV = 3300000,
		.max_uV = 3300000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = 2,
	.consumer_supplies = encore_lcd_tp_supply,
};

static struct fixed_voltage_config encore_lcd_touch_reg_data = {
	.supply_name = "vdd_lcdtp",
	.microvolts = 3300000,
	.gpio = 36,
	.enable_high = 1,
	.enabled_at_boot = 1,
	.init_data = &encore_lcd_tp_vinit,
};

static struct platform_device encore_lcd_touch_regulator_device = {
	.name   = "reg-fixed-voltage",
	.id     = -1,
	.dev    = {
		.platform_data = &encore_lcd_touch_reg_data,
	},
};

#ifdef CONFIG_CHARGER_MAX8903
static struct platform_device max8903_charger_device = {
	.name           = "max8903_charger",
	.id             = -1,
};
#endif

static struct platform_device *encore_board_devices[] __initdata = {
	&encore_keys_gpio,
	&encore_lcd_touch_regulator_device,
#ifdef CONFIG_CHARGER_MAX8903
	&max8903_charger_device,
#endif
};

static struct platform_device omap_vwlan_device = {
	.name		= "reg-fixed-voltage",
	.id		= 1,
	.dev = {
		.platform_data	= &encore_vwlan,
	},
};

#ifdef CONFIG_BATTERY_MAX17042
struct max17042_platform_data max17042_platform_data_here = {

	.gpio = MAX17042_GPIO_FOR_IRQ,

};
#endif

/* The order is reverted in this table so that internal eMMC is presented
 * as first mmc card for compatibility with existing android installations */
static struct omap2_hsmmc_info mmc[] = {
	{
		.name		= "internal",
		.mmc		= 2,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.nonremovable	= true,
		.power_saving	= true,
	},
	{
		.name		= "external",
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_wp	= -EINVAL,
		.power_saving	= true,
	},
	{
		.name		= "wl1271",
		.mmc		= 3,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD | MMC_PM_KEEP_POWER,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.nonremovable	= true,
	},

	{}      /* Terminator */
};

static int encore_hsmmc_card_detect(struct device *dev, int slot)
{
	struct omap_mmc_platform_data *mmc = dev->platform_data;

	/* Encore board EVT2 and later has pin high when card is present) */
	return gpio_get_value_cansleep(mmc->slots[0].switch_pin);
}

static int encore_twl4030_hsmmc_late_init(struct device *dev)
{
        int ret = 0;
        struct platform_device *pdev = container_of(dev,
                                struct platform_device, dev);
        struct omap_mmc_platform_data *pdata = dev->platform_data;

	if(is_encore_board_evt2()) {
		/* Setting MMC1 (external) Card detect */
		if (pdev->id == 0) {
			pdata->slots[0].card_detect = encore_hsmmc_card_detect;
		}
	}
        return ret;
}

static __init void encore_hsmmc_set_late_init(struct device *dev)
{
	struct omap_mmc_platform_data *pdata;

	/* dev can be null if CONFIG_MMC_OMAP_HS is not set */
	if (!dev)
		return;

	pdata = dev->platform_data;
	pdata->init = encore_twl4030_hsmmc_late_init;
}

static int encore_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
struct omap2_hsmmc_info *c;
	/* gpio + 0 is "mmc0_cd" (input/IRQ),
	 * gpio + 1 is "mmc1_cd" (input/IRQ)
	 */
printk("******IN boxer_twl_gpio_setup********\n");
	mmc[1].gpio_cd = gpio + 0;
	mmc[0].gpio_cd = gpio + 1;
	omap2_hsmmc_init(mmc);
	for (c = mmc; c->mmc; c++)
                encore_hsmmc_set_late_init(c->dev);

	/* link regulators to MMC adapters ... we "know" the
	 * regulators will be set up only *after* we return.
	*/
	encore_vmmc1_supply.dev = mmc[1].dev;
	encore_vsim_supply.dev = mmc[1].dev;
	encore_vmmc2_supply.dev = mmc[0].dev;

	return 0;
}

static struct regulator_consumer_supply encore_vpll2_supplies[] = {
	REGULATOR_SUPPLY("vdds_dsi", "omapdss"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi1"),
};

static struct regulator_consumer_supply encore_vdda_dac_supply =
	REGULATOR_SUPPLY("vdda_dac", "omapdss_venc");

static struct regulator_init_data encore_vpll2 = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies		= ARRAY_SIZE(encore_vpll2_supplies),
	.consumer_supplies		= encore_vpll2_supplies,
};

static struct regulator_init_data encore_vdac = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies		= 1,
	.consumer_supplies		= &encore_vdda_dac_supply,
};

static struct twl4030_usb_data encore_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static struct twl4030_gpio_platform_data encore_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.setup		= encore_twl_gpio_setup,
};

static struct twl4030_madc_platform_data encore_madc_data = {
	.irq_line	= 1,
};

static struct twl4030_codec_audio_data encore_audio_data;

static struct twl4030_codec_data encore_codec_data = {
	.audio_mclk = 26000000,
	.audio = &encore_audio_data,
};

static struct twl4030_platform_data encore_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.madc		= &encore_madc_data,
	.usb		= &encore_usb_data,
	.gpio		= &encore_gpio_data,
	.keypad		= &encore_kp_twl4030_data,
	.power		= &encore_t2scripts_data,
	//.codec		= &encore_codec_data,
	.vmmc1          = &encore_vmmc1,
	.vmmc2          = &encore_vmmc2,
	.vsim           = &encore_vsim,
	.vpll2		= &encore_vpll2,
	.vdac		= &encore_vdac,
};

static struct i2c_board_info __initdata encore_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO(KXTF9_DEVICE_ID, KXTF9_I2C_SLAVE_ADDRESS),
		.platform_data = &kxtf9_platform_data_here,
		.irq = OMAP_GPIO_IRQ(KXTF9_GPIO_FOR_IRQ),
	},
	{
		I2C_BOARD_INFO(MAX17042_DEVICE_ID, MAX17042_I2C_SLAVE_ADDRESS),
		.platform_data = &max17042_platform_data_here,
		.irq = OMAP_GPIO_IRQ(MAX17042_GPIO_FOR_IRQ),
	},

};

static struct i2c_board_info __initdata encore_i2c_bus2_info[] = {
	{
		I2C_BOARD_INFO(CY_I2C_NAME, CYTTSP_I2C_SLAVEADDRESS),
		.platform_data = &cyttsp_platform_data,
		.irq = OMAP_GPIO_IRQ(OMAP_CYTTSP_GPIO),
	},
	{
 		I2C_BOARD_INFO(FT_I2C_NAME, FT5x06_I2C_SLAVEADDRESS),
 		.platform_data = &ft5x06_platform_data,
		.irq = OMAP_GPIO_IRQ(OMAP_FT5x06_GPIO),
	},
	{                
		I2C_BOARD_INFO("tlv320aic31xx-codec", 0x18),
	},	

};

static int __init omap_i2c_init(void)
{
	int err;	

	omap_pmic_init(1, 100, "tps65921", INT_34XX_SYS_NIRQ, &encore_twldata);

	err=i2c_register_board_info(1, encore_i2c_boardinfo, ARRAY_SIZE(encore_i2c_boardinfo));
	if (err)
	  return err;

	omap_register_i2c_bus(2, 400, encore_i2c_bus2_info,
			ARRAY_SIZE(encore_i2c_bus2_info));

	return 0;
}

void __init encore_peripherals_init(void)
{
	platform_add_devices(encore_board_devices,
		ARRAY_SIZE(encore_board_devices));
	twl4030_get_scripts(&encore_t2scripts_data);
	omap_i2c_init();
	platform_device_register(&omap_vwlan_device);
	usb_musb_init(NULL);
	omap_serial_init();

	kxtf9_dev_init();
#ifdef CONFIG_BATTERY_MAX17042
        max17042_dev_init();
#endif
}
