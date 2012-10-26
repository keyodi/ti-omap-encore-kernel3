/*
 * Copyright (C) 2011 Texas Instruments Inc.
 *
 * Modified from mach-omap2/board-zoom2.c
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
#include <linux/i2c/twl.h>
#include <linux/regulator/machine.h>
#include <linux/leds.h>
#include <linux/spi/spi.h>

#include <video/omapdss.h>
#include <plat/mcspi.h>
#include <plat/omap-pm.h>
#include <plat/board.h>

#include "mux.h"

#define LCD_CABC0_GPIO                  44
#define LCD_CABC1_GPIO                  45
#define LCD_BACKLIGHT_EN_EVT2           47

#define DEFAULT_BACKLIGHT_BRIGHTNESS    105

static void boxer_backlight_set_power(struct omap_pwm_led_platform_data *self, int on_off)
{
	gpio_direction_output(LCD_BACKLIGHT_EN_EVT2, !on_off);
	gpio_set_value(LCD_BACKLIGHT_EN_EVT2, !on_off);
}

static struct omap_pwm_led_platform_data boxer_backlight_data = {
	.name		 = "lcd-backlight",
	.default_trigger = "backlight",
	.intensity_timer = 8,
	.bkl_max	 = 254,
	.bkl_min	 = 5,
	.bkl_freq	 = 128,
	.invert		 = 0,
	.def_brightness	 = DEFAULT_BACKLIGHT_BRIGHTNESS,
	.set_power	 = boxer_backlight_set_power,
};

static struct platform_device boxer_backlight_led_device = {
	.name		= "omap_pwm_led",
	.id		= 0,
	.dev		= {
	.platform_data = &boxer_backlight_data,
	},
};

static void __init boxer_backlight_init(void)
{
        printk("Enabling backlight PWM for LCD\n");
        //boxer_backlight_data.def_on = 1; // change the PWM polarity

        gpio_request(LCD_BACKLIGHT_EN_EVT2, "lcd backlight evt2");

        gpio_request(LCD_CABC0_GPIO, "lcd CABC0");
        gpio_direction_output(LCD_CABC0_GPIO,0);
        gpio_set_value(LCD_CABC0_GPIO,0);

        gpio_request(LCD_CABC1_GPIO, "lcd CABC1");
        gpio_direction_output(LCD_CABC1_GPIO,0);
        gpio_set_value(LCD_CABC1_GPIO,0);
}

static int encore_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	boxer_backlight_set_power(NULL, 1);

	return 0;
}

static void encore_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	boxer_backlight_set_power(NULL, 0);
}

static struct omap_dss_device evt_lcd_device = {
	.phy		= {
		.dpi	= {
			.data_lines	= 24,
		},
	},
	.clocks		= {
		.dispc	= {
			.channel	= {
				.lck_div        = 1,
				.pck_div        = 4,
				.lcd_clk_src    = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC,
			},
			.dispc_fclk_src = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC,
		},
	},
        .panel          = {
		.config		= OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |
				  OMAP_DSS_LCD_IHS | OMAP_DSS_LCD_IPC,
		.timings	= {
			.x_res          = 1024,
			.y_res          = 600,
			.pixel_clock    = 65000, /* in kHz */
			.hfp            = 70,	 /* HFP fix 160 */
			.hsw            = 40,    /* HSW = 1~140 */
			.hbp            = 200,   /* HSW + HBP = 160 */
			.vfp            = 10,    /* VFP fix 12 */
			.vsw            = 10,    /* VSW = 1~20 */
			.vbp            = 11,    /* VSW + VBP = 23 */
		},
		.width_in_um = 158000,
		.height_in_um = 92000,
        },
	.name = "lcd",
	.driver_name = "boxer_panel",
	.type = OMAP_DISPLAY_TYPE_DPI,
	.channel = OMAP_DSS_CHANNEL_LCD,
	.platform_enable  = encore_panel_enable_lcd,
	.platform_disable  = encore_panel_disable_lcd,
};

static struct omap_dss_device *evt_dss_devices[] = {
	&evt_lcd_device,
};

static struct omap_dss_board_info evt_dss_data = {
	.num_devices = ARRAY_SIZE(evt_dss_devices),
	.devices = evt_dss_devices,
	.default_device = &evt_lcd_device,
};

static struct omap2_mcspi_device_config evt_lcd_mcspi_config = {
	.turbo_mode             = 0,
	.single_channel         = 1,  /* 0: slave, 1: master */

};

struct spi_board_info evt_spi_board_info[] __initdata = {
	{
		.modalias               = "boxer_disp_spi",
		.bus_num                = 4,
		.chip_select            = 0,
		.max_speed_hz           = 375000,
		.controller_data        = &evt_lcd_mcspi_config,
	},
};

static struct platform_device *evt_panel_devices[] __initdata = {
	&boxer_backlight_led_device,
};

void __init encore_display_init(void)
{
        boxer_backlight_init();
	spi_register_board_info(evt_spi_board_info,
				ARRAY_SIZE(evt_spi_board_info));

	platform_add_devices(evt_panel_devices, ARRAY_SIZE(evt_panel_devices));
        omap_display_init(&evt_dss_data);

}

