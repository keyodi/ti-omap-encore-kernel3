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

#define LCD_PANEL_ENABLE_GPIO		36
#define LCD_CABC0_GPIO                  44
#define LCD_CABC1_GPIO                  45
#define LCD_BACKLIGHT_EN_EVT2           47

#define DEFAULT_BACKLIGHT_BRIGHTNESS    105

/*---backlight--------------------------------------------------------------------*/
static void boxer_backlight_set_power(struct omap_pwm_led_platform_data *self, int on_off)
{
	gpio_direction_output(LCD_BACKLIGHT_EN_EVT2, !on_off);
	gpio_set_value(LCD_BACKLIGHT_EN_EVT2, !on_off);
}

static struct omap_pwm_led_platform_data boxer_backlight_data = {
	.name            = "lcd-backlight",
	.intensity_timer = 8,
	.def_on          = 0,   // PWM high == backlight OFF, PWM low == backlight ON
	.def_brightness  = DEFAULT_BACKLIGHT_BRIGHTNESS,
	.set_power       = boxer_backlight_set_power,
};

static struct platform_device boxer_backlight_led_device = {
	.name		= "omap_pwm_led",
	.id		= -1,
	.dev		= {
	.platform_data = &boxer_backlight_data,
	},
};

static void __init boxer_backlight_init(void)
{
        printk("Enabling backlight PWM for LCD\n");
        boxer_backlight_data.def_on = 1; // change the PWM polarity

        gpio_request(LCD_BACKLIGHT_EN_EVT2, "lcd backlight evt2");

        gpio_request(LCD_CABC0_GPIO, "lcd CABC0");
        gpio_direction_output(LCD_CABC0_GPIO,0);
        gpio_set_value(LCD_CABC0_GPIO,0);

        gpio_request(LCD_CABC1_GPIO, "lcd CABC1");
        gpio_direction_output(LCD_CABC1_GPIO,0);
        gpio_set_value(LCD_CABC1_GPIO,0);
}

/*--------------------------------------------------------------------------*/
struct boxer_panel_data {
	struct regulator *vlcd;
};

static struct boxer_panel_data boxer_panel;

static inline struct boxer_panel_data * get_panel_data(struct omap_dss_device *dssdev)
{
	return dssdev->data;
}

static struct omap_dss_device evt_lcd_device = {
	.name = "lcd",
	.driver_name = "boxer_panel",
	.type = OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines = 24,
	.channel = OMAP_DSS_CHANNEL_LCD,
	.data = &boxer_panel,
};

static struct omap_dss_device *evt_dss_devices[] = {
	&evt_lcd_device,
};

static struct omap_dss_board_info evt_dss_data = {
	.num_devices = ARRAY_SIZE(evt_dss_devices),
	.devices = evt_dss_devices,
	.default_device = &evt_lcd_device,
};

/*--------------------------------------------------------------------------*/
static struct omap2_mcspi_device_config evt_lcd_mcspi_config = {
	.turbo_mode             = 0,
	.single_channel         = 1,  /* 0: slave, 1: master */
};

struct spi_board_info evt_spi_board_info[] __initdata = {
	[0] = {
		.modalias               = "boxer_disp_spi",
		.bus_num                = 4,
		.chip_select            = 0,
		.max_speed_hz           = 375000,
		.controller_data        = &evt_lcd_mcspi_config,
	},
};

/*--------------------------------------------------------------------------*/
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

