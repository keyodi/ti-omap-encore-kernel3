/*
 * Copyright (C) 2009 Texas Instruments Inc.
 *
 * Modified from mach-omap2/board-encore.c
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
#include <linux/synaptics_i2c_rmi.h>
#include <linux/leds-omap4430sdp-display.h>

#include <media/v4l2-int-device.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/common.h>
#include <plat/usb.h>
#include <linux/switch.h>
#include <mach/board-zoom.h>

#include <linux/input/cyttsp.h>
#include <linux/input/ft5x06.h>

#include "mux.h"
#include "hsmmc.h"
#include "common-board-devices.h"
#include "twl4030.h"

#define CYTTSP_I2C_SLAVEADDRESS  34
#define OMAP_CYTTSP_GPIO         99
#define OMAP_CYTTSP_RESET_GPIO   46

#define FT5x06_I2C_SLAVEADDRESS  (0x70 >> 1)
#define OMAP_FT5x06_GPIO         99
#define OMAP_FT5x06_RESET_GPIO   46

#define OMAP_encore_WLAN_PMENA_GPIO	(101)
#define OMAP_encore_WLAN_IRQ_GPIO		(162)
#define OMAP_SYNAPTICS_GPIO			(163)

/* PWM output/clock enable for LCD backlight*/
#define REG_INTBR_GPBR1				(0xc)
#define REG_INTBR_GPBR1_PWM1_OUT_EN		(0x1 << 3)
#define REG_INTBR_GPBR1_PWM1_OUT_EN_MASK	(0x1 << 3)
#define REG_INTBR_GPBR1_PWM1_CLK_EN		(0x1 << 1)
#define REG_INTBR_GPBR1_PWM1_CLK_EN_MASK	(0x1 << 1)

/* pin mux for LCD backlight*/
#define REG_INTBR_PMBR1				(0xd)
#define REG_INTBR_PMBR1_PWM1_PIN_EN		(0x3 << 4)
#define REG_INTBR_PMBR1_PWM1_PIN_MASK		(0x3 << 4)

#define MAX_CYCLES				(0x7f)
#define MIN_CYCLES				(75)
#define LCD_PANEL_ENABLE_GPIO			(7 + OMAP_MAX_GPIO_LINES)

int  ft5x06_dev_init(int resource)
{
    if (resource)
    {
        if (gpio_request(OMAP_FT5x06_RESET_GPIO, "ft5x06_reset") < 0)
        {
            printk(KERN_ERR "can't get ft5x06 xreset GPIO\n");
            return -1;
        }

        if (gpio_request(OMAP_FT5x06_GPIO, "ft5x06_touch") < 0)
        {
            printk(KERN_ERR "can't get ft5x06 interrupt GPIO\n");
            return -1;
        }

        gpio_direction_input(OMAP_FT5x06_GPIO);
    }
    else
    {
        gpio_free(OMAP_FT5x06_GPIO);
        gpio_free(OMAP_FT5x06_RESET_GPIO);
    }

    return 0;
}

static struct ft5x06_platform_data ft5x06_platform_data = {
    .maxx = 1024,
    .maxy = 600,
    .flags = FLIP_DATA_FLAG | REVERSE_Y_FLAG | REVERSE_X_FLAG,
    .reset_gpio = OMAP_FT5x06_RESET_GPIO,
    .use_st = FT_USE_ST,
    .use_mt = FT_USE_MT,
    .use_trk_id = 1, //FT_USE_TRACKING_ID,
    .use_sleep = FT_USE_SLEEP,
    .use_gestures = 0,
};

int  cyttsp_dev_init(int resource)
{
        if (resource)
        {
                if (gpio_request(OMAP_CYTTSP_RESET_GPIO, "tma340_reset") < 0) {
                        printk(KERN_ERR "can't get tma340 xreset GPIO\n");
                        return -1;
                }

                if (gpio_request(OMAP_CYTTSP_GPIO, "cyttsp_touch") < 0) {
                        printk(KERN_ERR "can't get cyttsp interrupt GPIO\n");
                        return -1;
                }

                gpio_direction_input(OMAP_CYTTSP_GPIO);
        }
        else
        {
                gpio_free(OMAP_CYTTSP_GPIO);
                gpio_free(OMAP_CYTTSP_RESET_GPIO);
        }
    return 0;
}

static struct cyttsp_platform_data cyttsp_platform_data = {
        .maxx = 480,
        .maxy = 800,
        .flags = FLIP_DATA_FLAG | REVERSE_Y_FLAG,
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

static struct __initdata twl4030_power_data encore_t2scripts_data;

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

static struct gpio_switch_platform_data headset_switch_data = {
	.name		= "h2w",
	.gpio		= OMAP_MAX_GPIO_LINES + 2, /* TWL4030 GPIO_2 */
};

static struct platform_device headset_switch_device = {
	.name		= "switch-gpio",
	.id		= -1,
	.dev		= {
		.platform_data = &headset_switch_data,
	}
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
	.gpio			= OMAP_encore_WLAN_PMENA_GPIO,
	.startup_delay		= 70000, /* 70msec */
	.enable_high		= 1,
	.enabled_at_boot	= 0,
	.init_data		= &encore_vmmc3,
};

static struct platform_device *encore_board_devices[] __initdata = {
	&encore_keys_gpio,	
	&headset_switch_device,
};

static struct platform_device omap_vwlan_device = {
	.name		= "reg-fixed-voltage",
	.id		= 1,
	.dev = {
		.platform_data	= &encore_vwlan,
	},
};

static struct wl12xx_platform_data omap_zoom_wlan_data __initdata = {
	.irq = OMAP_GPIO_IRQ(OMAP_encore_WLAN_IRQ_GPIO),
	/* encore ref clock is 26 MHz */
	.board_ref_clock = 1,
};

static void encore_pwm_config(u8 brightness)
{
	u8 pwm_off = 0;

	pwm_off = (MIN_CYCLES * (LED_FULL - brightness) +
		   MAX_CYCLES * (brightness - LED_OFF)) /
		(LED_FULL - LED_OFF);

	pwm_off = clamp(pwm_off, (u8)MIN_CYCLES, (u8)MAX_CYCLES);

	/* start at 0 */
	twl_i2c_write_u8(TWL4030_MODULE_PWM1, 0, 0);
	twl_i2c_write_u8(TWL4030_MODULE_PWM1, pwm_off, 1);
}

static void encore_pwm_enable(int enable)
{
	u8 gpbr1;

	twl_i2c_read_u8(TWL4030_MODULE_INTBR, &gpbr1, REG_INTBR_GPBR1);
	gpbr1 &= ~REG_INTBR_GPBR1_PWM1_OUT_EN_MASK;
	gpbr1 |= (enable ? REG_INTBR_GPBR1_PWM1_OUT_EN : 0);
	twl_i2c_write_u8(TWL4030_MODULE_INTBR, gpbr1, REG_INTBR_GPBR1);

	twl_i2c_read_u8(TWL4030_MODULE_INTBR, &gpbr1, REG_INTBR_GPBR1);
	gpbr1 &= ~REG_INTBR_GPBR1_PWM1_CLK_EN_MASK;
	gpbr1 |= (enable ? REG_INTBR_GPBR1_PWM1_CLK_EN : 0);
	twl_i2c_write_u8(TWL4030_MODULE_INTBR, gpbr1, REG_INTBR_GPBR1);
}

static void encore_set_primary_brightness(u8 brightness)
{
	u8 pmbr1;
	static int encore_pwm1_config;
	static int encore_pwm1_output_enabled;

	if (encore_pwm1_config == 0) {
		twl_i2c_read_u8(TWL4030_MODULE_INTBR, &pmbr1, REG_INTBR_PMBR1);

		pmbr1 &= ~REG_INTBR_PMBR1_PWM1_PIN_MASK;
		pmbr1 |=  REG_INTBR_PMBR1_PWM1_PIN_EN;
		twl_i2c_write_u8(TWL4030_MODULE_INTBR, pmbr1, REG_INTBR_PMBR1);

		encore_pwm1_config = 1;
	}

	if (!brightness) {
		encore_pwm_enable(0);
		encore_pwm1_output_enabled = 0;
		return;
	}

	encore_pwm_config(brightness);
	if (encore_pwm1_output_enabled == 0) {
		encore_pwm_enable(1);
		encore_pwm1_output_enabled = 1;
	}
}

static struct omap4430_sdp_disp_led_platform_data encore_disp_led_data = {
	.flags = LEDS_CTRL_AS_ONE_DISPLAY,
	.primary_display_set = encore_set_primary_brightness,
	.secondary_display_set = NULL,
};


static struct platform_device encore_disp_led = {
	.name   =       "display_led",
	.id     =       -1,
	.dev    = {
		.platform_data = &encore_disp_led_data,
	},
};

static struct omap2_hsmmc_info mmc[] = {
	{
		.name		= "internal",
		.mmc		= 2,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_wp	= -EINVAL,
		.power_saving	= true,
	},
	{
		.name		= "external",
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.nonremovable	= true,
		.power_saving	= true,
	},
	{
		.name		= "wl1271",
		.mmc		= 3,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
		.nonremovable	= true,
	},
	{}      /* Terminator */
};

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

static int encore_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	int ret;

	/* gpio + 0 is "mmc0_cd" (input/IRQ) */
	mmc[0].gpio_cd = gpio + 0;
	omap2_hsmmc_init(mmc);

	/* link regulators to MMC adapters ... we "know" the
	 * regulators will be set up only *after* we return.
	*/
	encore_vmmc1_supply.dev = mmc[0].dev;
	encore_vsim_supply.dev = mmc[0].dev;
	encore_vmmc2_supply.dev = mmc[1].dev;

	ret = gpio_request_one(LCD_PANEL_ENABLE_GPIO, GPIOF_OUT_INIT_LOW,
			       "lcd enable");
	if (ret)
		pr_err("Failed to get LCD_PANEL_ENABLE_GPIO (gpio%d).\n",
				LCD_PANEL_ENABLE_GPIO);

	return ret;
}

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
	.codec		= &encore_codec_data,
	.vmmc1          = &encore_vmmc1,
	.vmmc2          = &encore_vmmc2,
	.vsim           = &encore_vsim,
	.vpll2		= &encore_vpll2,
	.vdac		= &encore_vdac,
};

static void synaptics_dev_init(void)
{
	/* Set the ts_gpio pin mux */
	omap_mux_init_signal("gpio_163", OMAP_PIN_INPUT_PULLUP);

	if (gpio_request(OMAP_SYNAPTICS_GPIO, "touch") < 0) {
		printk(KERN_ERR "can't get synaptics pen down GPIO\n");
		return;
	}
	gpio_direction_input(OMAP_SYNAPTICS_GPIO);
	gpio_set_debounce(OMAP_SYNAPTICS_GPIO, 310);
}

static int synaptics_power(int power_state)
{
	/* TODO: synaptics is powered by vbatt */
	return 0;
}

static struct synaptics_i2c_rmi_platform_data synaptics_platform_data[] = {
	{
		.version        = 0x0,
		.power          = &synaptics_power,
		.flags          = SYNAPTICS_SWAP_XY,
		.irqflags       = IRQF_TRIGGER_LOW,
	}
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

#if 0

#if (defined(CONFIG_VIDEO_IMX046) || defined(CONFIG_VIDEO_IMX046_MODULE)) && \
	defined(CONFIG_VIDEO_OMAP3)
	{
		I2C_BOARD_INFO(IMX046_NAME, IMX046_I2C_ADDR),
		.platform_data = &encore_imx046_platform_data,
	},
#endif
#if (defined(CONFIG_VIDEO_LV8093) || defined(CONFIG_VIDEO_LV8093_MODULE)) && \
	defined(CONFIG_VIDEO_OMAP3)
	{
		I2C_BOARD_INFO(LV8093_NAME,  LV8093_AF_I2C_ADDR),
		.platform_data = &encore_lv8093_platform_data,
	},
#endif

#endif
};

static int __init omap_i2c_init(void)
{
	omap_pmic_init(1, 2400, "twl5030", INT_34XX_SYS_NIRQ, &encore_twldata);
	omap_register_i2c_bus(2, 100, encore_i2c_bus2_info,
			ARRAY_SIZE(encore_i2c_bus2_info));
	//omap_register_i2c_bus(3, 400, encore_i2c_bus3_info,
			//ARRAY_SIZE(encore_i2c_bus3_info));
	return 0;
}

static void enable_board_wakeup_source(void)
{
	/* T2 interrupt line (keypad) */
	omap_mux_init_signal("sys_nirq",
		OMAP_WAKEUP_EN | OMAP_PIN_INPUT_PULLUP);
}

void __init zoom_peripherals_init(void)
{
	platform_add_devices(encore_board_devices,
		ARRAY_SIZE(encore_board_devices));
	twl4030_get_scripts(&encore_t2scripts_data);
	omap_i2c_init();
	synaptics_dev_init();
	platform_device_register(&omap_vwlan_device);
	platform_device_register(&encore_disp_led);
	usb_musb_init(NULL);
	enable_board_wakeup_source();
	omap_serial_init();
}
