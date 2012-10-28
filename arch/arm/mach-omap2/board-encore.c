/*
 * Copyright (C) 2009-2010 Texas Instruments Inc.
 * Mikkel Christensen <mlc@ti.com>
 * Felipe Balbi <balbi@ti.com>
 *
 * Modified from mach-omap2/board-ldp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/i2c/twl.h>
#include <linux/mtd/nand.h>
#include <linux/memblock.h>
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#include <linux/wl12xx.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/mm.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <plat/common.h>
#include <plat/board.h>
#include <plat/usb.h>

#include <mach/board-encore.h>

#include "mux.h"
#include "sdram-hynix-h8mbx00u0mer-0em.h"
#include "omap_ion.h"
#include "timer-gp.h"

#define ZOOM3_EHCI_RESET_GPIO		64
#define ZOOM3_McBSP3_BT_GPIO            164
#define ENCORE_BT_RESET_GPIO            60
#define ENCORE_WIFI_IRQ_GPIO		15
#define ENCORE_WIFI_ENPOW_GPIO		16

#ifdef CONFIG_WLAN_POWER_EVT1
#define ENABLE_VAUX2_DEDICATED		0x05
#define ENABLE_VAUX2_DEV_GRP		0x20
#endif

#define WILINK_UART_DEV_NAME            "/dev/ttyO1"

/* These are defined in arch/arm/kernel/setup.c */
extern unsigned int system_serial_low;
extern unsigned int system_serial_high;

static void __init omap_encore_init_early(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(h8mbx00u0mer0em_sdrc_params,
				h8mbx00u0mer0em_sdrc_params);
#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(1);
#endif
}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	/* WLAN IRQ - GPIO 162 */
	OMAP3_MUX(MCBSP1_CLKX, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),
	/* WLAN POWER ENABLE - GPIO 101 */
	OMAP3_MUX(CAM_D2, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	/* WLAN SDIO: MMC3 CMD */
	OMAP3_MUX(MCSPI1_CS1, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP),
	/* WLAN SDIO: MMC3 CLK */
	OMAP3_MUX(ETK_CLK, OMAP_MUX_MODE2 | OMAP_PIN_INPUT_PULLUP),
	/* WLAN SDIO: MMC3 DAT[0-3] */
	OMAP3_MUX(ETK_D3, OMAP_MUX_MODE2 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(ETK_D4, OMAP_MUX_MODE2 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(ETK_D5, OMAP_MUX_MODE2 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(ETK_D6, OMAP_MUX_MODE2 | OMAP_PIN_INPUT_PULLUP),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux       NULL
#endif

static const struct usbhs_omap_board_data usbhs_bdata __initconst = {
	.port_mode[0]		= OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[1]		= OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[2]		= OMAP_USBHS_PORT_MODE_UNUSED,
	.phy_reset		= true,
	.reset_gpio_port[0]	= -EINVAL,
	.reset_gpio_port[1]	= ZOOM3_EHCI_RESET_GPIO,
	.reset_gpio_port[2]	= -EINVAL,
};

static int plat_kim_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* TODO: wait for HCI-LL sleep */
	return 0;
}
static int plat_kim_resume(struct platform_device *pdev)
{
	return 0;
}

/* wl127x BT, FM, GPS connectivity chip */
struct ti_st_plat_data wilink_pdata = {
	.nshutdown_gpio = 60,
	.dev_name = WILINK_UART_DEV_NAME,
	.flow_cntrl = 1,
	.baud_rate = 3000000,
	.suspend = plat_kim_suspend,
	.resume = plat_kim_resume,
};
static struct platform_device wl127x_device = {
	.name           = "kim",
	.id             = -1,
	.dev.platform_data = &wilink_pdata,
};
static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};

static struct platform_device *encore_devices[] __initdata = {
	&wl127x_device,
	&btwilink_device,
};

/* Fix to prevent VIO leakage on wl127x */
static int wl127x_vio_leakage_fix(void)
{
	int ret = 0;

	pr_info(" wl127x_vio_leakage_fix\n");

	ret = gpio_request(ENCORE_BT_RESET_GPIO, "wl127x_bten");
	if (ret < 0) {
		pr_err("wl127x_bten gpio_%d request fail",
			ENCORE_BT_RESET_GPIO);
		goto fail;
	}

	gpio_direction_output(ENCORE_BT_RESET_GPIO, 1);
	mdelay(10);
	gpio_direction_output(ENCORE_BT_RESET_GPIO, 0);
	udelay(64);

	gpio_free(ENCORE_BT_RESET_GPIO);
fail:
	return ret;
}

static struct wl12xx_platform_data encore_wlan_data __initdata = {
	.irq = OMAP_GPIO_IRQ(ENCORE_WIFI_IRQ_GPIO),
	.board_ref_clock = WL12XX_REFCLOCK_38,
};

static int __init encore_wifi_v18io_power_enable(void) {
	int retval = 0;

#ifdef CONFIG_WLAN_POWER_EVT1
	printk("Enabling VAUX for wifi\n");
	retval = twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VAUX2_DEDICATED,
			TWL4030_VAUX2_DEDICATED);
	retval |= twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VAUX2_DEV_GRP,
			TWL4030_VAUX2_DEV_GRP);
	if (retval != 0)
		printk("Enabling VAUX for wifi failed with error %d\n", retval);
#endif

	return retval;
}

static int __init encore_wifi_init(void)
{
	int ret;

	gpio_request(ENCORE_WIFI_ENPOW_GPIO, "wifi_en_pow");
	encore_wifi_v18io_power_enable();
	gpio_direction_output(ENCORE_WIFI_ENPOW_GPIO, 1);

	ret = wl12xx_set_platform_data(&encore_wlan_data);
	if (ret)
		pr_err("Error setting wl12xx data\n");

	return ret;
}
device_initcall(encore_wifi_init);

static void __init omap_encore_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBP);

	encore_peripherals_init();
	encore_display_init();
	omap_register_ion();
	/* Added to register encore devices */
	platform_add_devices(encore_devices, ARRAY_SIZE(encore_devices));
	wl127x_vio_leakage_fix();
}

static void __init encore_reserve(void)
{
	/* do the static reservations first */
	memblock_remove(OMAP3_PHYS_ADDR_SMC_MEM, PHYS_ADDR_SMC_SIZE);

#ifdef CONFIG_ION_OMAP
	omap_ion_init();
#endif
	omap_reserve();
}

#ifdef CONFIG_OMAP_MUX
  #error "EVT1A port relies on the bootloader for MUX configuration."
#endif

/*
 * Android expects the bootloader to pass the device serial number as a
 * parameter on the kernel command line, but encore's bootloader doesn't
 * do this.  Since the kernel already knows the serial number, let's just
 * add the appropriate parameter to the command line we present to userspace
 * (in /proc/cmdline).
 */
#define SERIALNO_PARAM "androidboot.serialno"
static int __init encore_cmdline_set_serialno_hack(void)
{
	/*
	 * The final cmdline will have 16 digits, a space, an =, and a trailing
	 * \0 as well as the contents of saved_command_line and SERIALNO_PARAM,
	 * hence the 19
	 */
	size_t len = strlen(saved_command_line) + strlen(SERIALNO_PARAM) + 19;
	char *buf = kmalloc(len, GFP_ATOMIC);
	if (buf) {
		snprintf(buf, len, "%s %s=%08x%08x",
				saved_command_line,
				SERIALNO_PARAM,
				system_serial_high, system_serial_low);
		/* XXX This leaks strlen(saved_command_line) bytes of memory
		 * Do we care? */
		saved_command_line = buf;
	}

	return 0;
}
device_initcall(encore_cmdline_set_serialno_hack);

MACHINE_START(ENCORE, "encore")
	.boot_params	= 0x80000100,
	.reserve	= encore_reserve,
	.map_io		= omap3_map_io,
	.init_early	= omap_encore_init_early,
	.init_irq	= omap_init_irq,
	.init_machine	= omap_encore_init,
	.timer		= &omap_timer,
MACHINE_END
