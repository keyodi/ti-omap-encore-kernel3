/*
 * linux/sound/soc/codecs/tlv320aic3100.c
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 *
 * Based on sound/soc/codecs/wm8753.c by Liam Girdwood
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Revision History
 *
 * 0.1	Nov-2010	Reference driver implentation for
 *			OMAP3 Platforms
 * 0.2	Jun-2011	Ported the code-base to 2.6.35 Kernel
 * 0.3	Jul-2011	Added the ABE based Playback and Recording feature
 * 0.4	Jul-2011	Fixed/Updated the Recording feature
 *			at Android level
 * 0.5	Aug-2011	Addressing the ABE LP changes and gain
 *			for playback path
 * 0.6	Aug-2011	Fixed the Recording CM Settings and also the
 *			MIC Coarse Gain, Fine Gain and PGA Settings for
 *			better Recording Quality at Android. Added support
 *			for High-Pass and Low-Pass Filters on the Recording
 *			Path.
 * 0.7	Sep-2011	Updated the Speaker Gain to 12db from 6db
 * 0.8	Sep-2011	Increased the MIC PGA Gain to 30db from 24db as per
 *			the recommendation from the customer
 * 0.9	Sep-2011	Updated the DRC related Registers since the
 *			customer requested the feature. DRC is enabled on
 *			Speaker Path. As part of DRC feature, also updated
 *			the DAC PRB mode to 2.
 * 1.0	Sep-2011	As per the discussion with TI R&D Team, switching off
 *			Codec Power supplies during suspend() and resume()
 *			has been disabled due to limitation on current HW.
 * 1.1	Oct-2011	Added aic3100_hp_power_up() and aic3100_hp_power_down()
 *			functions to handle Headset Driver Power-up and
 *			Power-down sequences. Invoked the headset Power up
 *			function once during initialization.
 * 1.2	Oct-2011	Added function aic3100_get_record_status() to
 *			get Recording status.
 * 1.3	Dec-2011	Ported the code base to 3.0 kernel.
 * 1.4	Jan-2012	Implemented the DAPM support for power management.
 */

/******************************************************************************
 * INCLUDE HEADER FILES
 *****************************************************************************/
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/clk.h>
#include <sound/jack.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <asm/div64.h>

#include "tlv320aic3100.h"
#include <mach/gpio.h>

#include <plat/clock.h>

/* Flags for using different support functionalities of the codec */

#define CODEC_POWER_OFF
#define AIC3100_CODEC_SUPPORT
#define ACCLAIM_SUPPORT
/*#define AIC3110_CODEC_SUPPORT */

/******************************************************************************
 * GLOBAL VARIABLES AND FUNCTION DECLARATIONS
 *****************************************************************************/

static int snd_soc_info_volsw_2r_aic31xx(struct snd_kcontrol *,
					 struct snd_ctl_elem_info *);
static int snd_soc_get_volsw_2r_aic31xx(struct snd_kcontrol *,
					struct snd_ctl_elem_value *);
static int snd_soc_put_volsw_2r_aic31xx(struct snd_kcontrol *,
					struct snd_ctl_elem_value *);
static int __new_control_info(struct snd_kcontrol *,
				struct snd_ctl_elem_info *);
static int __new_control_get(struct snd_kcontrol *,
				struct snd_ctl_elem_value *);
static int __new_control_put(struct snd_kcontrol *,
				struct snd_ctl_elem_value *);
static int aic31xx_dac_mute(struct snd_soc_codec *codec, int mute);
static int dac3100_power_down(struct snd_soc_codec *codec);
static int dac3100_power_up(struct snd_soc_codec *codec);

static int right_dac_gain = 0xFC;
static int left_dac_gain = 0xFC;
static int hp_r_analog_gain = 0x9E;
static int hp_l_analog_gain = 0x9E;
static int spkr_analog_gain = 0x80;


#define NUM_INIT_REGS (sizeof(aic31xx_reg_init) /       \
				sizeof(struct aic31xx_configs))

#define SOC_SINGLE_AIC31xx(xname)                                       \
	{                                                               \
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,     \
			.info = __new_control_info, .get = __new_control_get, \
			.put = __new_control_put,                       \
			.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,      \
			}

#define SOC_DOUBLE_R_AIC31xx(xname, reg_left, reg_right, shift, mask, invert) \
	{								\
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),   \
			.info = snd_soc_info_volsw_2r_aic31xx,          \
			.get = snd_soc_get_volsw_2r_aic31xx,            \
			.put = snd_soc_put_volsw_2r_aic31xx,		\
			.private_value = (reg_left) | ((shift) << 8)  | \
			((mask) << 12) | ((invert) << 20) | ((reg_right) << 24)}

#define	AIC31xx_CACHEREGNUM 256

static u8 aic31xx_reg_ctl;

/*
 * Whenever tinyplay or tinycap is used, aic31xx_hw_params() function gets
 * called. This function reprograms the clock dividers. This flag can be used to
 * disable this when the clock dividers are programmed by pps config file
 */
static int soc_static_freq_config = 1;

/* Codec Private Struct variable */
struct aic31xx_priv *aic31xx;

/*
 * Global Variables introduced to reduce Headphone Analog Volume Control
 * Registers at run-time
 */
struct i2c_msg i2c_right_transaction[120];
struct i2c_msg i2c_left_transaction[120];

/*
 * the structure contains the pll settings for different sampling rates.
 *
 * mclk, rate, p_val, pll_j, pll_d, dosr, ndac, mdac, aosr, nadc, madc, blck_N
 */
static const struct aic31xx_rate_divs aic31xx_divs[] = {
	/*
 * mclk, rate, p_val, pll_j, pll_d, dosr, ndac, mdac, aosr, nadc, madc, blck_N,
 * codec_speficic_initializations
 */
	/* 8k rate */
	// DDenchev (MMS)
	{12000000, 8000, 1, 7, 1680, 128, 2, 42, 4,
	 {{DAC_PRB_SEL_REG, 7}, {61, 1}}},
	{13000000, 8000, 1, 6, 3803, 128, 3, 27, 4,
	 {{DAC_PRB_SEL_REG, 7}, {61, 4}}},
	{24000000, 8000, 2, 7, 6800, 768, 15, 1, 24,
	 {{DAC_PRB_SEL_REG, 1}, {61, 1}}},

	/* 11.025k rate */
	// DDenchev (MMS)
	{12000000, 11025, 1, 7, 560, 128, 5, 12, 4,
	 {{DAC_PRB_SEL_REG, 1}, {61, 1}}},
	{13000000, 11025, 1, 6, 1876, 128, 3, 19, 4,
	 {{DAC_PRB_SEL_REG, 1}, {61, 4}}},
	{24000000, 11025, 2, 7, 5264, 512, 16, 1, 16,
	 {{DAC_PRB_SEL_REG, 1}, {61, 1}}},

	/* 12k rate */
	// DDenchev (MMS)
	{12000000, 12000, 1, 7, 1680, 128, 2, 28, 4,
	 {{DAC_PRB_SEL_REG, 1}, {61, 1}}},
	{13000000, 12000, 1, 6, 3803, 128, 3, 18, 4,
	 {{DAC_PRB_SEL_REG, 1}, {61, 4}}},

	/* 16k rate */
	// DDenchev (MMS)
	{12000000, 16000, 1, 7, 1680, 128, 2, 21, 4,
	 {{DAC_PRB_SEL_REG, 1}, {61, 1}}},
	{13000000, 16000, 1, 6, 6166, 128, 3, 14, 4,
	 {{DAC_PRB_SEL_REG, 1}, {61, 4}}},
	{24000000, 16000, 2, 7, 6800, 384, 15, 1, 12,
	 {{DAC_PRB_SEL_REG, 1}, {61, 1}}},

	/* 22.05k rate */
	// DDenchev (MMS)
	{12000000, 22050, 1, 7, 560, 128, 5, 6, 4,
	 {{DAC_PRB_SEL_REG, 1}, {61, 1}}},
	{13000000, 22050, 1, 6, 5132, 128, 3, 10, 4,
	 {{DAC_PRB_SEL_REG, 1}, {61, 4}}},
	{24000000, 22050, 2, 7, 5264, 256, 16, 1, 8,
	 {{DAC_PRB_SEL_REG, 1}, {61, 1}}},

	/* 24k rate */
	// DDenchev (MMS)
	{12000000, 24000, 1, 7, 1680, 128, 2, 14, 4,
	 {{DAC_PRB_SEL_REG, 1}, {61, 1}}},
	{13000000, 24000, 1, 6, 3803, 128, 3, 9, 4,
	 {{DAC_PRB_SEL_REG, 1}, {61, 4}}},

	/* 32k rate */
	// DDenchev (MMS)
	{12000000, 32000, 1, 6, 1440, 128, 2, 9, 4,
	 {{DAC_PRB_SEL_REG, 1}, {61, 1}}},
	{13000000, 32000, 1, 6, 6166, 128, 3, 7, 4,
	 {{DAC_PRB_SEL_REG, 1}, {61, 4}}},
	{24000000, 32000, 2, 7, 1680, 192, 7, 2, 6,
	 {{DAC_PRB_SEL_REG, 1}, {61, 1}}},

	/* 44.1k rate */
	// DDenchev (MMS)
	{12000000, 44100, 1, 7, 560, 128, 5, 3, 4,
	 {{DAC_PRB_SEL_REG, 7}, {61, 1}}},
	{13000000, 44100, 1, 6, 5132, 128, 3, 5, 4,
	 {{DAC_PRB_SEL_REG, 7}, {61, 4}}},
	{24000000, 44100, 2, 7, 5264, 128, 8, 2, 4,
	 {{DAC_PRB_SEL_REG, 1}, {61, 1}}},

	/* 48k rate */
	// DDenchev (MMS)
	{12000000, 48000, 1, 7, 1680, 128, 2, 7, 4,
	 {{DAC_PRB_SEL_REG, 1}, {61, 1}}},
	{13000000, 48000, 1, 6, 6166, 128, 7, 2, 4,
	 {{DAC_PRB_SEL_REG, 1}, {61, 4}}},
	{24000000, 48000, 2, 8, 1920, 128, 8, 2, 4,
	 {{DAC_PRB_SEL_REG, 1}, {61, 1}}},

	/*96k rate : GT 21/12/2009: NOT MODIFIED */
	{12000000, 96000, 1, 8, 1920, 64, 2, 8, 2,
	 {{DAC_PRB_SEL_REG, 7}, {61, 7}}},
	{13000000, 96000, 1, 6, 6166, 64, 7, 2, 2,
	 {{DAC_PRB_SEL_REG, 7}, {61, 10}}},
	{24000000, 96000, 2, 8, 1920, 64, 4, 4, 2,
	 {{DAC_PRB_SEL_REG, 7}, {61, 7}}},

	/*192k : GT 21/12/2009: NOT MODIFIED */
	{12000000, 192000, 1, 8, 1920, 32, 2, 8, 1,
	 {{DAC_PRB_SEL_REG, 17}, {61, 13}}},
	{13000000, 192000, 1, 6, 6166, 32, 7, 2, 1,
	 {{DAC_PRB_SEL_REG, 17}, {61, 13}}},
	{24000000, 192000, 2, 8, 1920, 32, 4, 4, 1,
	 {{DAC_PRB_SEL_REG, 17}, {61, 13}}},
};

/* Caching the register values */
static const u8 aic31xx_reg[AIC31xx_CACHEREGNUM] = {
	0x00, 0x00, 0x00, 0x02,	/* 0 */
	0x00, 0x11, 0x04, 0x00,	/* 4 */
	0x00, 0x00, 0x00, 0x01,	/* 8 */
	0x01, 0x00, 0x80, 0x80,	/* 12 */
	0x08, 0x00, 0x01, 0x01,	/* 16 */
	0x80, 0x80, 0x04, 0x00,	/* 20 */
	0x00, 0x00, 0x01, 0x00,	/* 24 */
	0x00, 0x00, 0x01, 0x00,	/* 28 */
	0x00, 0x00, 0x00, 0x00,	/* 32 */
	0x00, 0x00, 0x00, 0x00,	/* 36 */
	0x00, 0x00, 0x00, 0x00,	/* 40 */
	0x00, 0x00, 0x00, 0x00,	/* 44 */
	0x00, 0x00, 0x00, 0x00,	/* 48 */
//	0x00, 0x02, 0x02, 0x00,	/* 52 */
	0x00, 0x00, 0x02, 0x00,	/* 52 */
	0x00, 0x00, 0x00, 0x00,	/* 56 */
	0x01, 0x04, 0x00, 0x14,	/* 60 */
	0x0C, 0x00, 0x00, 0x00,	/* 64 */
	0x0F, 0x38, 0x00, 0x00,	/* 68 */
	0x00, 0x00, 0x00, 0xEE,	/* 72 */
	0x10, 0xD8, 0x7E, 0xE3,	/* 76 */
	0x00, 0x00, 0x80, 0x00,	/* 80 */
	0x00, 0x00, 0x00, 0x00,	/* 84 */
//	0x7F, 0x00, 0x00, 0x00,	/* 88 */
	0x00, 0x00, 0x00, 0x00,	/* 88 */
	0x00, 0x00, 0x00, 0x00,	/* 92 */
	0x00, 0x00, 0x00, 0x00,	/* 96 */
	0x00, 0x00, 0x00, 0x00,	/* 100 */
	0x00, 0x00, 0x00, 0x00,	/* 104 */
	0x00, 0x00, 0x00, 0x00,	/* 108 */
	0x00, 0x00, 0x00, 0x00,	/* 112 */
	0x00, 0x00, 0x00, 0x00,	/* 116 */
	0x00, 0x00, 0x00, 0x00,	/* 120 */
	0x00, 0x00, 0x00, 0x00,	/* 124 - PAGE0 Registers(127) ends here */
	0x00, 0x00, 0x00, 0x00,	/* 128, PAGE1-0 */
	0x00, 0x00, 0x00, 0x00,	/* 132, PAGE1-4 */
	0x00, 0x00, 0x00, 0x00,	/* 136, PAGE1-8 */
	0x00, 0x00, 0x00, 0x00,	/* 140, PAGE1-12 */
	0x00, 0x00, 0x00, 0x00,	/* 144, PAGE1-16 */
	0x00, 0x00, 0x00, 0x00,	/* 148, PAGE1-20 */
	0x00, 0x00, 0x00, 0x00,	/* 152, PAGE1-24 */
	0x00, 0x00, 0x00, 0x04,	/* 156, PAGE1-28 */
	0x06, 0x3E, 0x00, 0x00,	/* 160, PAGE1-32 */
	0x7F, 0x7F, 0x7F, 0x7F,	/* 164, PAGE1-36 */
	0x02, 0x02, 0x00, 0x00,	/* 168, PAGE1-40 */
	0x00, 0x00, 0x00, 0x80,	/* 172, PAGE1-44 */
	0x00, 0x00, 0x00,	/* 176, PAGE1-48 */
};

/*
 * aic31xx initialization data
 *
 * This structure contains the initialization required for aic31xx.  These
 * registers values (reg_val) are written into the respective aic31xx
 * register offset (reg_offset) to initialize aic31xx.  These values are used
 * in aic31xx_init() function only.
 */
static const struct aic31xx_configs aic31xx_reg_init[] = {
/* Carry out the software reset */
//	{RESET, 0x01},
//	{RESET, 0x00},
	/* Connect MIC1_L and MIC1_R to CM */
        //	{MICPGA_CM, 0xC0},
	/* PLL is CODEC_CLKIN */
	{CLK_REG_1, PLLCLK_2_CODEC_CLKIN},
        /* Switch off PLL */
        {CLK_REG_2,  0x00},
        {CLK_REG_3,  0x00},
        {CLK_REG_4,  0x00},
        {CLK_REG_5,  0x00},
        /* Switch off NDAC and MDAC and BCLK_N Dividers */
        {NDAC_CLK_REG, 0x00},
        {MDAC_CLK_REG,  0x00},
        {DAC_OSR_MSB,     0x00},
        {DAC_OSR_LSB,     0x00},
        /* DAC_MOD_CLK is BCLK source */
	{INTERFACE_SET_REG_2, DAC_MOD_CLK_2_BDIV_CLKIN},

	/* Switch off BCLK_N Divider */
	{BCLK_N_VAL, 0x00},

	/* Setting up DAC Channel */
	{DAC_CHN_REG, LDAC_2_LCHN | RDAC_2_RCHN /*| SOFT_STEP_2WCLK8*/},
	{DAC_MUTE_CTRL_REG, 0x0C},   	/* Mistral: Updated this value from 0x00 to 0x0C to MUTE DAC Left and Right channels */

        /* Mistral Added following configuration for EQ Setup */
        /* Mistral: Updated the DACR volume also to -6dB */
        {LDAC_VOL, 0xFC}, /* Update DACL volume control from -6db to -1.5db to get speaker and Headphone volume loud enough on Encore device */
        {RDAC_VOL, 0xFC}, /* Update DACR volume control from -6db to -1.5db to get speaker and Headphone volume loud enough on Encore device */
        {VOL_MICDECT_ADC, 0x00}, /* Configure the DAC Volume to default settings */

        /* reg[1][42] should be configured at 0x04. This is already done above */
        /* reg[1][35] should be configured correctly. This is already done above */
        /* reg[1][32] should be powered up, Will be done at run-time */
        /* reg[1][38] should be routed to Class-D Output Driver. This is already done above */


        /* Headphone drivers */
	{HEADPHONE_DRIVER, 0x04}, /* 0xCC Mistral: Updated this value from 0xC4. We do not need to Power up HPL and HPR at startup */
	{CLASSD_SPEAKER_AMP, 0x06},            /* Mistral: Updated this value from 0xc6 to 0x06 */
	/* Headphone powerup */
	{HP_POP_CTRL, (BIT7 | HP_POWER_UP_15_3_MSEC 
                      | HP_RAMP_UP_STIME_3_9MS 
                      | HP_BANDGAP_COMMON_MODE)},  /* Mistral modified value to power down DAC after HP and SPK Amplifiers */
        {PGA_RAMP_CTRL, 0x70},        /* Speaker Ramp up time scaled to 30.5ms */
	/* DAC_L and DAC_R Output Mixer Routing */
	{DAC_MIX_CTRL, 0x44},  //DAC_X is routed to the channel mixer amplifier
	/* HPL gain 0db initial */
	{L_ANLOG_VOL_2_HPL, 0x9E},
	/* HPR gain 0db initial */
	{R_ANLOG_VOL_2_HPR, 0x9E},

	{L_ANLOG_VOL_2_SPL, 0x80},        /* Mistral: Updated Values en plus 0x80 - 0x92 */

        /* HPL mute and gain 0db */
	{HPL_DRIVER, 0x00}, /* Mistral: Updated this value from 0x2. Put HPL to common-mode and MUTED state at startup */
	/* HPR mute and gain 0db */
	{HPR_DRIVER, 0x00}, /* Mistral: Updated this value from 0x2. Put HPR to common-mode and MUTED state at startup */
        {SPL_DRIVER, 0x00},		/* Keep the Speaker driver in MUTE State by Default */

	{HP_DRIVER_CTRL, 0x00}, /* Configured the default value to 0x00 */
};

/*
 * soc_enum array Structure Initialization
 */

static char const *dac_mute[] = {"Unmute", "Mute"};
static char const *dacvoltage_control[] = {"1.35 V", "1.5 V ", "1.65 V", "1.8 V"};
static char const *drc_enable[] = {"Disabled", "Enabled"};

static const struct soc_enum aic31xx_enum[] = {
	SOC_ENUM_SINGLE(DAC_MUTE_CTRL_REG, 3, 2, dac_mute),
	SOC_ENUM_SINGLE(DAC_MUTE_CTRL_REG, 2, 2, dac_mute),
	SOC_ENUM_SINGLE(HEADPHONE_DRIVER, 3, 4, dacvoltage_control),
	SOC_ENUM_DOUBLE(DRC_CTRL_1, 6, 5, 2, drc_enable),
};

/*
 * controls that need to be exported to the user space
 */
static const struct snd_kcontrol_new aic31xx_snd_controls[] = {
	/* Output */
	/* sound new kcontrol for PCM Playback volume control */
	SOC_DOUBLE_R_AIC31xx("DAC Playback Volume", LDAC_VOL, RDAC_VOL, 0, 0xAf,
			     0),
	/* sound new kcontrol for HP driver gain */
	SOC_DOUBLE_R_AIC31xx("HP Driver Gain", HPL_DRIVER, HPR_DRIVER, 0, 0x23, 0),
	/* sound new kcontrol for LO driver gain */
	SOC_DOUBLE_R_AIC31xx("LO Driver Gain", SPL_DRIVER, SPR_DRIVER, 0, 0x23, 0),
	/* sound new kcontrol for HP mute */
	SOC_DOUBLE_R("HP DAC Playback Switch", HPL_DRIVER, HPR_DRIVER, 2,
		     0x01, 1),
	/* sound new kcontrol for LO mute */
	SOC_DOUBLE_R("LO DAC Playback Switch", SPL_DRIVER, SPR_DRIVER, 2,
		     0x01, 1),

	/* sound new kcontrol for Analog Volume Control for headphone and Speaker Outputs
         * Please refer to Table 5-24 of the Codec DataSheet
         */
        SOC_DOUBLE_R_AIC31xx("HP Analog Volume",   L_ANLOG_VOL_2_HPL, R_ANLOG_VOL_2_HPR, 0, 0x7F, 1),
        SOC_DOUBLE_R_AIC31xx("SPKR Analog Volume", L_ANLOG_VOL_2_SPL, R_ANLOG_VOL_2_SPR, 0, 0x7F, 1),

	/* sound new kcontrol for Programming the registers from user space */
	SOC_SINGLE_AIC31xx("Program Registers"),

        /* Enumerations SOCs Controls */
        SOC_ENUM ("LEFT  DAC MUTE", aic31xx_enum[LMUTE_ENUM]),
        SOC_ENUM ("RIGHT DAC MUTE", aic31xx_enum[RMUTE_ENUM]),
        SOC_ENUM ("HP Driver Voltage level", aic31xx_enum[DACEXTRA_ENUM]),
        SOC_ENUM ("DRC Status",aic31xx_enum[DACCONTROL_ENUM]),

        /* Dynamic Range Compression Control */
        SOC_SINGLE ("DRC Hysteresis Value (0=0db 3=db)", DRC_CTRL_1, 0, 0x03, 0),
        SOC_SINGLE ("DRC Threshold Value (0=-3db,7=-24db)",  DRC_CTRL_1, 2, 0x07, 0),
        SOC_SINGLE ("DRC Hold Time",   DRC_CTRL_2, 3, 0x0F, 0),
        SOC_SINGLE ("DRC Attack Time", DRC_CTRL_3, 4, 0x0F, 0),
        SOC_SINGLE ("DRC Delay Rate",  DRC_CTRL_3, 0, 0x0F, 0),

};

/*
 * DAPM Mixer Controls
 */
/* Left DAC_L Mixer */
static const struct snd_kcontrol_new hpl_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("L_DAC switch", DAC_MIX_CTRL, 6, 2, 1),
	SOC_DAPM_SINGLE("MIC1_L switch", DAC_MIX_CTRL, 5, 1, 0),
//	SOC_DAPM_SINGLE("Left_Bypass switch", HPL_ROUTE_CTL, 1, 1, 0),
};

/* Right DAC_R Mixer */
static const struct snd_kcontrol_new hpr_output_mixer_controls[] = {

	SOC_DAPM_SINGLE("R_DAC switch", DAC_MIX_CTRL, 2, 2, 1),
	SOC_DAPM_SINGLE("MIC1_R switch", DAC_MIX_CTRL, 1, 1, 0),
//	SOC_DAPM_SINGLE("Right_Bypass switch", HPR_ROUTE_CTL, 1, 1, 0),
};

static const struct snd_kcontrol_new lol_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("L_DAC switch", DAC_MIX_CTRL, 6, 2, 1),    //SOC_DAPM_SINGLE("L_DAC switch", LOL_ROUTE_CTL, 3, 1, 0),
	SOC_DAPM_SINGLE("MIC1_L switch", DAC_MIX_CTRL, 5, 1, 0),
//	SOC_DAPM_SINGLE("Left_Bypass switch", HPL_ROUTE_CTL, 1, 1, 0),
};

static const struct snd_kcontrol_new lor_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("R_DAC switch", DAC_MIX_CTRL, 2, 2, 1),    //SOC_DAPM_SINGLE("R_DAC switch", LOR_ROUTE_CTL, 3, 1, 0),
	SOC_DAPM_SINGLE("MIC1_R switch", DAC_MIX_CTRL, 1, 1, 0),
//	SOC_DAPM_SINGLE("Right_Bypass switch", LOR_ROUTE_CTL, 1, 1, 0),
};

/* Right DAC_R Mixer */
static const struct snd_kcontrol_new left_input_mixer_controls[] = {
	SOC_DAPM_SINGLE("MIC1_L switch", MIC_GAIN, 6, 1, 0),
	SOC_DAPM_SINGLE("MIC1_M switch", MIC_GAIN, 2, 1, 0),
};

static const struct snd_kcontrol_new right_input_mixer_controls[] = {
	SOC_DAPM_SINGLE("MIC1_R switch", MIC_GAIN, 4, 1, 0),
	SOC_DAPM_SINGLE("MIC1_M switch", MIC_GAIN, 2, 1, 0),
};

/* Left Output Mixer */
static const struct snd_kcontrol_new
left_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("From DAC_L", DAC_MIX_CTRL, 6, 1, 0),
	SOC_DAPM_SINGLE("From MIC1LP", DAC_MIX_CTRL, 5, 1, 0),
	SOC_DAPM_SINGLE("From MIC1RP", DAC_MIX_CTRL, 4, 1, 0),
};

static int pll_power_on_event(struct snd_soc_dapm_widget *w, \
			struct snd_kcontrol *kcontrol, int event)
{
	if (event == (SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD))
		mdelay(10);
	return 0;
}

static int aic31xx_dac_power_up_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	u8 counter, value;
	struct snd_soc_codec *codec = w->codec;

	if (SND_SOC_DAPM_EVENT_ON(event)) {

		/* Check for the DAC FLAG register to know if the DAC is really
		 * powered up
		 */
		counter = 0;
		do {
			mdelay(1);
			value = codec->read(codec, DAC_FLAG_1);
			counter++;
			DBG("##DACEn Poll\r\n");
		} while ((counter < 20) && ((value & 0x88) == 0));

	} else if (SND_SOC_DAPM_EVENT_OFF(event)) {

		/* Check for the DAC FLAG register to know if the DAC is
		 * powered down
		 */
		counter = 0;
		do {
			mdelay(1);
			value = codec->read(codec, DAC_FLAG_1);
			counter++;
			DBG("##DAC switched off\r\n");
		} while ((counter < 20) && ((value | 0x00) == 0));

	}
	return 0;
}

static int aic31xx_adc_power_up_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	u8 counter, value;
	struct snd_soc_codec *codec = w->codec;

	if (SND_SOC_DAPM_EVENT_ON(event)) {

		/* Check for the ADC FLAG register to know if the ADC is
		 * really powered up
		 */
		counter = 0;
		do {
			mdelay(10);
			value = codec->read(codec, ADC_FLAG);
			counter++;
			DBG("##ADCEn Poll\r\n");
		} while ((counter < 40) && ((value & 0x40) == 0));

	} else if (SND_SOC_DAPM_EVENT_OFF(event)) {

		/* Check for the ADC FLAG register to know if the ADC is
		 * powered down
		 */
		counter = 0;
		do {
			mdelay(1);
			value = codec->read(codec, ADC_FLAG);
			counter++;
			DBG("##ADC switched off\r\n");
		} while ((counter < 20) && ((value | 0x00) == 0));

	}
	return 0;
}

static int aic31xx_hp_power_up_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	u8 counter, value;
	struct snd_soc_codec *codec = w->codec;

	if (event & SND_SOC_DAPM_PRE_PMU) {
		DBG(KERN_INFO "pre_pmu: switching to HP\n");
		codec->write(codec, HP_DRIVER_CTRL, 0x00);

	} else if (event & SND_SOC_DAPM_POST_PMU) {

		/* Check for the DAC FLAG register to know if the HPL & HPR are
		 * really powered up
		 */
		counter = 0;
		do {
			mdelay(5);
			value = codec->read(codec, DAC_FLAG_1);
			counter++;
		} while ((value & 0x22) == 0);
		DBG(KERN_INFO "##HPL Power up Iterations %d\r\n", counter);

/*		aic31xx_config_hp_volume(codec, 0); */

	} else if (event & SND_SOC_DAPM_PRE_PMD) {
		DBG(KERN_INFO "pre_pmd: switching to LO\n");
		codec->write(codec, HP_DRIVER_CTRL, 0x06);

	} else if (event & SND_SOC_DAPM_POST_PMD) {

		/* Check for the DAC FLAG register to know if the HPL & HPR are
		 * powered down
		 */
		counter = 0;
		do {
			mdelay(5);
			value = codec->read(codec, DAC_FLAG_1);
			counter++;
		} while ((counter < 10) && ((value & 0x22) != 0));
		DBG(KERN_INFO "##HPL Power down Iterations %d\r\n", counter);

	}
	return 0;
}

static int aic31xx_sp_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	u8 counter, value;
	struct snd_soc_codec *codec = w->codec;

	if (SND_SOC_DAPM_EVENT_ON(event)) {

#ifdef AIC3100_CODEC_SUPPORT
		/* Change the DAC channel path for mono speakers on AIC3100 */
		snd_soc_update_bits(codec, DAC_CHN_REG, 0x3C,
			      (CLEAR | LDAC_LCHN_RCHN_2));
#endif
		/* Check for the DAC FLAG register to know if the SPL & SPR are
		 * really powered up
		 */
		counter = 0;
		do {
			mdelay(5);
			value = codec->read(codec, DAC_FLAG_1);
			counter++;
		} while ((value & 0x11) == 0);
		DBG(KERN_INFO "##SP Power up Iterations %d\r\n", counter);

	} else if (SND_SOC_DAPM_EVENT_OFF(event)) {

#ifdef AIC3100_CODEC_SUPPORT
		/* Change the DAC channel path for mono speakers on AIC3100 */
		snd_soc_update_bits(codec, DAC_CHN_REG, 0x3C,
			 (CLEAR | LDAC_2_LCHN | RDAC_2_RCHN));
#endif
		/* Check for the DAC FLAG register to know if the SPL & SPR are
		 * powered down
		 */
		counter = 0;
		do {
			mdelay(5);
			value = codec->read(codec, DAC_FLAG_1);
			counter++;
		} while ((counter < 10) && ((value & 0x11) != 0));
		DBG(KERN_INFO "##SPL Power down Iterations %d\r\n", counter);

	}
	return 0;
}

/*
 * DAPM Widget Controls
 */
static const struct snd_soc_dapm_widget aic31xx_dapm_widgets[] = {
	/* Left DAC to Left Outputs */
	/* dapm widget (stream domain) for left DAC */
	SND_SOC_DAPM_DAC("Left DAC", "Left Playback", DAC_CHN_REG, 7, 1),

	/* dapm widget (path domain) for left DAC_L Mixer */

	SND_SOC_DAPM_MIXER("HPL Output Mixer", SND_SOC_NOPM, 0, 0,
			   &hpl_output_mixer_controls[0],
			   ARRAY_SIZE(hpl_output_mixer_controls)),
	SND_SOC_DAPM_PGA("HPL Power", HEADPHONE_DRIVER, 7, 0, NULL, 0),

	SND_SOC_DAPM_MIXER("LOL Output Mixer", SND_SOC_NOPM, 0, 0,
			   &lol_output_mixer_controls[0],
			   ARRAY_SIZE(lol_output_mixer_controls)),
	SND_SOC_DAPM_PGA("LOL Power", CLASSD_SPEAKER_AMP, 7, 1, NULL, 0),

	/* Right DAC to Right Outputs */

	/* dapm widget (stream domain) for right DAC */
	SND_SOC_DAPM_DAC("Right DAC", "Right Playback", DAC_CHN_REG, 6, 1),
	/* dapm widget (path domain) for right DAC_R mixer */
	SND_SOC_DAPM_MIXER("HPR Output Mixer", SND_SOC_NOPM, 0, 0,
			   &hpr_output_mixer_controls[0],
			   ARRAY_SIZE(hpr_output_mixer_controls)),
	SND_SOC_DAPM_PGA("HPR Power", HEADPHONE_DRIVER, 6, 0, NULL, 0),

	SND_SOC_DAPM_MIXER("LOR Output Mixer", SND_SOC_NOPM, 0, 0,
			   &lor_output_mixer_controls[0],
			   ARRAY_SIZE(lor_output_mixer_controls)),
	SND_SOC_DAPM_PGA("LOR Power", CLASSD_SPEAKER_AMP, 6, 1, NULL, 0),

	SND_SOC_DAPM_MIXER("Left Input Mixer", SND_SOC_NOPM, 0, 0,
			   &left_input_mixer_controls[0],
			   ARRAY_SIZE(left_input_mixer_controls)),

	SND_SOC_DAPM_MIXER("Right Input Mixer", SND_SOC_NOPM, 0, 0,
			   &right_input_mixer_controls[0],
			   ARRAY_SIZE(right_input_mixer_controls)),

	/* No widgets are required for ADC since the DAC3100 Audio
         * Codec Chipset does not contain a ADC
         */

	/* dapm widget (platform domain) name for HPLOUT */
	SND_SOC_DAPM_OUTPUT("HPL"),
	/* dapm widget (platform domain) name for HPROUT */
	SND_SOC_DAPM_OUTPUT("HPR"),
	/* dapm widget (platform domain) name for LOLOUT */
	SND_SOC_DAPM_OUTPUT("LOL"),
	/* dapm widget (platform domain) name for LOROUT */
	SND_SOC_DAPM_OUTPUT("LOR"),

	/* dapm widget (platform domain) name for MIC1LP */
	SND_SOC_DAPM_INPUT("MIC1LP"),
	/* dapm widget (platform domain) name for MIC1RP*/
	SND_SOC_DAPM_INPUT("MIC1RP"),
	/* dapm widget (platform domain) name for MIC1LM */
	SND_SOC_DAPM_INPUT("MIC1LM"),
};


/*
 * DAPM audio route definition - Defines an audio route originating at source
 * via control and finishing at sink.
 */
static const struct snd_soc_dapm_route aic31xx_dapm_routes[] = {
/* ******** Right Output ******** */
	{"HPR Output Mixer", "R_DAC switch", "Right DAC"},
	{"HPR Output Mixer",  "MIC1_R switch", "MIC1RP"},
	//{"HPR Output Mixer", "Right_Bypass switch", "Right_Bypass"},

	{"HPR Power", NULL, "HPR Output Mixer"},
	{"HPR", NULL, "HPR Power"},


	{"LOR Output Mixer", "R_DAC switch", "Right DAC"},
	{"LOR Output Mixer",  "MIC1_R switch", "MIC1RP"},
//	{"LOR Output Mixer", "Right_Bypass switch", "Right_Bypass"},

	{"LOR Power", NULL, "LOR Output Mixer"},
	{"LOR", NULL, "LOR Power"},

	/* ******** Left Output ******** */
	{"HPL Output Mixer", "L_DAC switch", "Left DAC"},
	{"HPL Output Mixer", "MIC1_L switch", "MIC1LP"},
	//{"HPL Output Mixer", "Left_Bypass switch", "Left_Bypass"},

	{"HPL Power", NULL, "HPL Output Mixer"},
	{"HPL", NULL, "HPL Power"},


	{"LOL Output Mixer", "L_DAC switch", "Left DAC"},
	{"LOL Output Mixer", "MIC1_L switch", "MIC1LP"},
//	{"LOL Output Mixer", "Left_Bypass switch", "Left_Bypass"},

	{"LOL Power", NULL, "LOL Output Mixer"},
	{"LOL", NULL, "LOL Power"},

	/* ******** terminator ******** */
	//{NULL, NULL, NULL},
};

#define AIC31xx_DAPM_ROUTE_NUM (sizeof(aic31xx_dapm_routes)/		\
				sizeof(struct snd_soc_dapm_route))

#ifdef AIC31xx_GPIO_INTR_SUPPORT
/* GPIO Interrupt Worker Thread related Global Vars */
static struct work_struct codec_int_work;
#endif

/*
 * aic31xx_change_page- switch between page 0 and page 1.
 */
int aic31xx_change_page(struct snd_soc_codec *codec, u8 new_page)
{
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	u8 data[2];

	data[0] = 0;
	data[1] = new_page;
	aic31xx->page_no = new_page;
/*	DBG("##aic31xx_change_page=> %d\nw 30 %02x %02x\n", new_page, data[0],
	    data[1]);

	DBG("w 30 %02x %02x\n", data[0], data[1]);*/

	if (codec->hw_write(codec->control_data, data, 2) != 2) {
		printk(KERN_ERR "Error in changing page to %d\n", new_page);
		return -1;
	}
	return 0;
}

#if defined(EN_REG_CACHE)
/*
 * aic31xx_write_reg_cache - write aic31xx register cache
 */
static inline void aic31xx_write_reg_cache(struct snd_soc_codec *codec,
							u16 reg, u8 value)
{
	u8 *cache = codec->reg_cache;

	if (reg >= AIC31xx_CACHEREGNUM)
		return 0;

	cache[reg] = value;
}

/*
 * aic31xx_read_reg_cache -read the aic31xx registers through the Register
 * Cache Array instead of I2C Transfers
 */
static unsigned char
aic31xx_read_reg_cache(struct snd_soc_codec *codec, unsigned int reg)
{
	u8 *cache = codec->reg_cache;

	/* Confirm the Register Offset is within the Array bounds */
	if (reg >= AIC31xx_CACHEREGNUM)
		return 0;

	return cache[reg];
}
#endif

/*
 * aic31xx_write - write to the aic3100 register space.
 */
int aic31xx_write(struct snd_soc_codec *codec, unsigned int reg,
						unsigned int value)
{
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	u8 data[2];
	u8 page;

	page = reg / 128;
	data[AIC31xx_REG_OFFSET_INDEX] = reg % 128;

	if (aic31xx->page_no != page)
		aic31xx_change_page(codec, page);

	/* data is
	 *   D15..D8 aic31xx register offset
	 *   D7...D0 register data
	 */
	data[AIC31xx_REG_DATA_INDEX] = value & AIC31xx_8BITS_MASK;

#if defined(EN_REG_CACHE)
	if ((page == 0) || (page == 1))
		codec->write_reg_cache(codec, reg, value);
#endif

	if (!data[AIC31xx_REG_OFFSET_INDEX])
		/* if the write is to reg0 update aic31xx->page_no */
		aic31xx->page_no = value;

	if (codec->hw_write(codec->control_data, data, 2) != 2) {
		printk(KERN_ERR "Error in i2c write\n");
		return -EIO;
	}
	return 0;
}

/*
 * aic31xx_read - read the aic3100 register space.
 */
unsigned int aic31xx_read(struct snd_soc_codec *codec, unsigned int reg)
{
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	u8 value;
	u8 page = reg / 128;

	/* Can be used to optimize the Reads from page 0 and 1 */
#if defined(EN_REG_CACHE)
	u8 cache_value;
	if ((page == 0) || (page == 1)) {
		cache_value = codec->read_reg_cache(codec, reg);
		DBG("Reg%x-Cache %02x\n", reg, value);
	}
#endif
	reg = reg % 128;

	if (aic31xx->page_no != page)
		aic31xx_change_page(codec, page);

	codec->hw_write(codec->control_data, (char *)&reg, 1);
	i2c_master_recv(codec->control_data, &value, 1);

	return value;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3100_power_up
 * Purpose  : This function powers up the codec.
 *
 *----------------------------------------------------------------------------
 */
static int dac3100_power_up (struct snd_soc_codec *codec)
{
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	u8 value;
	u8 counter;

	DBG("##++ dac3100_power_up dac3100->master=%d, dac3100->power_status=%d, "
            "dac3100->headset_connected=%d\n",
            aic31xx->master, aic31xx->power_status, aic31xx->headset_connected);

	DBG("##dac3100 power up .................\n");
	//mdelay(2000);
	/* all power is driven by DAPM system */
	if (aic31xx->master && (aic31xx->power_status != 1) ) {

		/* WA, enable the gpt11_fclk to avoid audio quality problems
                   when screen OFF */
		//clk_enable(gpt11_fclk);

        mdelay(5);
		/* Switch on PLL */
		value = aic31xx_read(codec, CLK_REG_2);
		aic31xx_write(codec, CLK_REG_2, (value | ENABLE_PLL));

		/* Switch on NDAC Divider */
		value = aic31xx_read(codec, NDAC_CLK_REG);
		aic31xx_write(codec, NDAC_CLK_REG, value | ENABLE_NDAC);

		/* Switch on MDAC Divider */
		value = aic31xx_read(codec, MDAC_CLK_REG);
		aic31xx_write(codec, MDAC_CLK_REG,
			      value | ENABLE_MDAC);

		/* Switch on BCLK_N Divider */
		value = aic31xx_read(codec, BCLK_N_VAL);
		aic31xx_write(codec, BCLK_N_VAL, value | ENABLE_BCLK);

                /* Switch ON Left and Right DACs */
		value = aic31xx_read (codec, DAC_CHN_REG);
		aic31xx_write (codec, DAC_CHN_REG, (value | ENABLE_DAC));

        /* Check for the DAC FLAG register to know if the DAC is
           really powered up */
        counter = 0;
        do {
                mdelay(10);
                value = aic31xx_read (codec, DAC_FLAG_1);
                counter++;
                DBG("##DACEn Poll\r\n");
        } while ((counter < 20) && ((value & 0x88) == 0));

        DBG("##-- dac3100_power_up\n");

		/* Check whether the Headset or Speaker Driver needs Power Up */
		if (aic31xx->headset_connected) {
                        /* It is observed that turning ON Speaker, helps reduce
                           the pop up noise */
                        /* Switch ON the Class_D Speaker Amplifier */
                        value = aic31xx_read (codec, CLASSD_SPEAKER_AMP);
                        aic31xx_write (codec, CLASSD_SPEAKER_AMP, (value | 0x80));
#if 0
                        /* Enable routing between left analog volume to HPL
                         * and right analog volume to HPR
                         */
                        value = aic31xx_read(codec, LEFT_ANALOG_HPL);
                        aic31xx_write(codec, LEFT_ANALOG_HPL, (0x14 | 0x80));
                        value = aic31xx_read(codec, RIGHT_ANALOG_HPR);
                        aic31xx_write(codec, RIGHT_ANALOG_HPR, (0x14 | 0x80));
                        mdelay(20);

                        dac3100_config_hp_volume(codec, 0);
#endif

                        /* Switch ON Left and Right Headphone Drivers */
                        value = aic31xx_read (codec, HEADPHONE_DRIVER);
                        aic31xx_write (codec, HEADPHONE_DRIVER, (value | 0xC0)); /* 0xCC */

                        /* Check for the DAC FLAG Register to know if the Left
                           Output Driver is powered up */
                        counter = 0;
                        do {
                                mdelay (10);
                                value = aic31xx_read (codec, DAC_FLAG_1);
                                counter++;
                                DBG("##HPL Poll..\n");
                        } while ((value & 0x22) == 0);
                        DBG("##HPL Power up Iterations %d\r\n", counter);

#if 0
                        value = aic31xx_read(codec, HPHONE_DRIVERS);
                        aic31xx_write(codec, HPHONE_DRIVERS, (value | 0x40));

                        /* Check for the DAC FLAG Register to know if the HPHONE
                           Driver is powered up */
                        counter = 0;
                        do {
                                mdelay (5);
                                value = aic31xx_read(codec, DAC_FLAG_1);
                                counter++;
                                DBG("##HPR Poll..\n");
                        } while ((value & 0x02) == 0);
#endif
                        DBG("##HPR Power Up Iterations %d\n", counter);
                        //mdelay (60);


		} else {
                        /* Left Analog Speaker Volume update */
                        aic31xx_write (codec, L_ANLOG_VOL_2_SPL, spkr_analog_gain);

                        /* Switch ON the Class_D Speaker Amplifier */
                        value = aic31xx_read (codec, CLASSD_SPEAKER_AMP);
                        aic31xx_write (codec, CLASSD_SPEAKER_AMP, (value | 0x80));
                        mdelay(100);
                }
                aic31xx->power_status = 1;
        }

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3100_power_down
 * Purpose  : This function powers down the codec.
 *
 *----------------------------------------------------------------------------
 */
static int dac3100_power_down (struct snd_soc_codec *codec)
{
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	volatile u8 value;
        volatile u32 counter;

	DBG("##++ dac3100_power_down dac3100->master=%d\n", aic31xx->master);

	if (aic31xx->master && (aic31xx->power_status != 0)) {

		/* Check whether the Headset or Speaker Driver needs Power Down */
		if(aic31xx->headset_connected) {

                        /* Switch off the Head phone Drivers */
                        value = aic31xx_read (codec, HEADPHONE_DRIVER);
                        aic31xx_write (codec, HEADPHONE_DRIVER, (value & ~0xC0)); /* 0xCC */

                        /* Now first check if the HPR is fully powered down */
                        counter = 0;
                        do {
                                mdelay(5);
                                value = aic31xx_read (codec, DAC_FLAG_1);
                                counter++;
                        }while ((counter < 100) && ((value & 0x22) != 0));
                        DBG ("##HPHONE RIGHT DRIVER Power Down. counter %d\r\n",
                             counter);
                        //mdelay(50);
#if 0
                        value = aic31xx_read (codec, HPHONE_DRIVERS);
                        aic31xx_write (codec, HPHONE_DRIVERS, (value & ~0x80));

                        /* Now poll the Page 0 Reg 37 DAC FLAG Register for HPL
                         * fully powered down */
                        counter = 0;
                        do {
                                mdelay(5);
                                value = aic31xx_read (codec, DAC_FLAG_1);
                                counter++;
                        } while ((counter < 20) && ((value & 0x20) != 0));
#endif
                        DBG("##HPHONE LEFT DRIVER Power Down counter %d \r\n",
                            counter);

                        value = aic31xx_read (codec, CLASSD_SPEAKER_AMP);
                        aic31xx_write (codec, CLASSD_SPEAKER_AMP, (value & ~0x80));
#if 0
                        /* Disable routing between left analog volume to HPL
                         * and right analog volume to HPR
                         */
                        value = aic31xx_read(codec, LEFT_ANALOG_HPL);
                        aic31xx_write(codec, LEFT_ANALOG_HPL, (value & ~0x80));
                        value = aic31xx_read(codec, RIGHT_ANALOG_HPR);
                        aic31xx_write(codec, RIGHT_ANALOG_HPR, (value & ~0x80));

                        dac3100_config_hp_volume(codec, 1);
#endif
		} else {
			/* Switch OFF the Class_D Speaker Amplifier */
			value = aic31xx_read (codec, CLASSD_SPEAKER_AMP);
			aic31xx_write (codec, CLASSD_SPEAKER_AMP, (value & ~0x80));

                        /* MUTE THE Class-D Speaker Driver */
                        value = aic31xx_read (codec, SPL_DRIVER);
                        value &= ~0x04;
                        aic31xx_write (codec, SPL_DRIVER, value);
                        DBG ("##SPL_DRIVER MUTE Completed..\r\n");
	        }
                /* Switch OFF Left and Right DACs */

		value = aic31xx_read (codec, DAC_CHN_REG);
		aic31xx_write (codec, DAC_CHN_REG, (value & ~ENABLE_DAC));

                /* Check for the DAC FLAG register to know if the DAC is really
                   powered down */
                counter = 0;
                do {
                        mdelay(10);
                        value = aic31xx_read (codec, DAC_FLAG_1);
                        counter++;
                } while ((counter < 100) && ((value & 0x88) != 0));
                DBG ("##Left and Right DAC off Counter %d\r\n", counter);

                /* Switch off BCLK_N Divider */
		aic31xx_write(codec, BCLK_N_VAL, value & ~ENABLE_BCLK);

                /* Switch off MDAC Divider */
		value = aic31xx_read(codec, MDAC_CLK_REG);
		aic31xx_write(codec, MDAC_CLK_REG,
                              value & ~ENABLE_MDAC);

		/* Switch off NDAC Divider */
		value = aic31xx_read(codec, NDAC_CLK_REG);
		aic31xx_write(codec, NDAC_CLK_REG,
                              value & ~ENABLE_NDAC);

		/* Switch off PLL */
		value = aic31xx_read(codec, CLK_REG_2);
		aic31xx_write(codec, CLK_REG_2, (value & ~ENABLE_PLL));

		//if (gpt11_fclk->usecount > 0)
		//	clk_disable(gpt11_fclk);

		aic31xx->power_status = 0;
	}

	DBG("##-- dac3100_power_down\n");

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : debug_print_registers
 * Purpose  : Debug routine to dump all the Registers of Page 0
 *
 *----------------------------------------------------------------------------
 */
void debug_print_registers(struct snd_soc_codec *codec)
{
	int i;
	u8 data;

	DBG("### Page 0 Regs from 0 to 95\n");

	for (i = 0; i < 95; i++) {
		data = (u8) codec->read(codec, i);
		printk(KERN_INFO "reg = %d val = %x\n", i, data);
	}
	DBG("-------------------------------------------\n");
	DBG("### Page 1 Regs from 30 to 52\n");

	for (i = 158; i < 180; i++) {
		data = (u8) codec->read(codec, i);
		printk(KERN_INFO "reg = %d val = %x\n", (i%128), data);
	}

	DBG("####SPL_DRIVER_GAIN %d\n\n",
			codec->read(codec, SPL_DRIVER));

	DBG("##### L_ANALOG_VOL_2_SPL %d R_ANLOG_VOL_2_SPR %d\n\n",
			codec->read(codec, L_ANLOG_VOL_2_SPL),
			codec->read(codec, R_ANLOG_VOL_2_SPR));
	DBG("#### LDAC_VOL %d RDAC_VOL %d\n\n",
			codec->read(codec, LDAC_VOL),
			codec->read(codec, RDAC_VOL));
	DBG("###OVER Temperature STATUS ( Page 0 Reg 3) %x\n\n",
			codec->read(codec, OT_FLAG));
	DBG("###SHORT CIRCUIT STATUS (Page 0 Reg 44) %x\n\n",
			codec->read(codec, INTR_FLAG_1));
	DBG("###INTR_FLAG: SHORT_CKT(Page 0 Reg 46) %x\n\n",
			codec->read(codec, INTR_FLAG_2));
	DBG("###Speaker_Driver_Short_Circuit ( Page 1 Reg 32)%x\n\n",
			codec->read(codec, CLASSD_SPEAKER_AMP));
	DBG("@@@  MIC_PGA (P1 R47) = 0x%x\n\n",
			codec->read(codec, MIC_PGA));
	DBG("@@@  ADC_FGA (P0 R82) = 0x%x\n\n",
			codec->read(codec, ADC_FGA));
	DBG("@@@  ADC_CGA (P0 R83) = 0x%x\n\n",
			codec->read(codec, ADC_CGA));
}

/*
 * __new_control_get - read data of new control for program the aic31xx
 * registers.
 */
static int __new_control_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	u32 val;

	val = codec->read(codec, aic31xx_reg_ctl);
	ucontrol->value.integer.value[0] = val;

	return 0;
}

/*
 * __new_control_put - this is called to pass data from user/application to
 * the driver.
 */
static int __new_control_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);

	u32 data_from_user = ucontrol->value.integer.value[0];
	u8 data[2];

	aic31xx_reg_ctl = data[0] = (u8) ((data_from_user & 0xFF00) >> 8);
	data[1] = (u8) ((data_from_user & 0x00FF));

	if (!data[0])
		aic31xx->page_no = data[1];

	if (codec->hw_write(codec->control_data, data, 2) != 2) {
		printk(KERN_ALERT "Error in i2c write\n");
		return -EIO;
	}

	return 0;
}

/*
 * snd_soc_info_volsw_2r_aic31xx - Info routine for the ASoC Widget related
 * to the Volume Control
 */
static int snd_soc_info_volsw_2r_aic31xx(struct snd_kcontrol *kcontrol,
						struct snd_ctl_elem_info *uinfo)
{
	int mask = (kcontrol->private_value >> 12) & 0xff;

	uinfo->type = mask == 1 ? SNDRV_CTL_ELEM_TYPE_BOOLEAN :
		SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = mask;
	return 0;
}

/*
 * snd_soc_get_volsw_2r_aic31xx - Callback to get the value of a double mixer
 * control that spans two registers.
 */
static int snd_soc_get_volsw_2r_aic31xx(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int reg = kcontrol->private_value & AIC31xx_8BITS_MASK;
	int reg2 = (kcontrol->private_value >> 24) & AIC31xx_8BITS_MASK;
	int mask;
	int shift;
	unsigned short val, val2;

	/* Check the id name of the kcontrol and configure the mask and shift */
	if (!strcmp(kcontrol->id.name, "DAC Playback Volume")) {
		mask = AIC31xx_8BITS_MASK;
		shift = 0;
	} else if (!strcmp(kcontrol->id.name, "HP Driver Gain")) {
		mask = 0xF;
		shift = 3;
	} else if (!strcmp(kcontrol->id.name, "LO Driver Gain")) {
		mask = 0x3;
		shift = 3;
	} else if (!strcmp(kcontrol->id.name, "PGA Capture Volume")) {
		mask = 0x7F;
		shift = 0;
	} else if (!strcmp(kcontrol->id.name, "HP Analog Volume")) {
                mask = 0x7F;
                shift = 0;
        } else if (!strcmp(kcontrol->id.name, "SPKR Analog Volume")) {
                mask = 0x7F;
                shift = 0;
        } else {
		printk(KERN_ALERT "Invalid kcontrol name\n");
		return -1;
	}

	val = (snd_soc_read(codec, reg) >> shift) & mask;
	val2 = (snd_soc_read(codec, reg2) >> shift) & mask;

	if (!strcmp(kcontrol->id.name, "DAC Playback Volume")) {
		ucontrol->value.integer.value[0] =
                        (val <= 48) ? (val + 127) : (val - 129);
		ucontrol->value.integer.value[1] =
                        (val2 <= 48) ? (val2 + 127) : (val2 - 129);
	} else if (!strcmp(kcontrol->id.name, "HP Driver Gain")) {
		ucontrol->value.integer.value[0] =
                        (val <= 9) ? (val + 0) : (val - 15);
		ucontrol->value.integer.value[1] =
                        (val2 <= 9) ? (val2 + 0) : (val2 - 15);
	} else if (!strcmp(kcontrol->id.name, "LO Driver Gain")) {
		ucontrol->value.integer.value[0] =
                        ((val/6) <= 4) ? ((val/6 -1)*6) : ((val/6 - 0)*6);
		ucontrol->value.integer.value[1] =
                        ((val2/6) <= 4) ? ((val2/6-1)*6) : ((val2/6 - 0)*6);
	} else if (!strcmp(kcontrol->id.name, "PGA Capture Volume")) {
		ucontrol->value.integer.value[0] =
                        ((val*2) <= 40) ? ((val*2 + 24)/2) : ((val*2 - 254)/2);
		ucontrol->value.integer.value[1] =
                        ((val2*2) <= 40) ? ((val2*2 + 24)/2) : ((val2*2 - 254)/2);
        } else if (!strcmp(kcontrol->id.name, "HP Analog Volume")) {
                ucontrol->value.integer.value[0] =
                        ((val*2) <= 40) ? ((val*2 +24)/2) :((val2*2 - 254)/2);
                ucontrol->value.integer.value[1] =
                        ((val*2) <=40) ? ((val*2 + 24)/2) : ((val2*2 - 254)/2);
	} else if (!strcmp(kcontrol->id.name, "SPKR Analog Volume")) {
                ucontrol->value.integer.value[0] =
                        ((val*2) <= 40) ? ((val*2 +24)/2) :((val2*2 - 254)/2);
                ucontrol->value.integer.value[1] =
                        ((val*2) <=40) ? ((val*2 + 24)/2) : ((val2*2 - 254)/2);
        }

	return 0;
}

/*
 * snd_soc_put_volsw_2r_aic31xx - Callback to set the value of a double mixer
 * control that spans two registers.
 */
static int snd_soc_put_volsw_2r_aic31xx(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int reg = kcontrol->private_value & AIC31xx_8BITS_MASK;
	int reg2 = (kcontrol->private_value >> 24) & AIC31xx_8BITS_MASK;
	int err;
	unsigned short val, val2, val_mask;

	val = ucontrol->value.integer.value[0];
	val2 = ucontrol->value.integer.value[1];

 /* Mistral: This block needs to be revisted */
	if (!strcmp(kcontrol->id.name, "DAC Playback Volume")) {
		val = (val >= 127) ? (val - 127) : (val + 129);
//		 (val <= 48) ? (val + 127) : (val - 129);
		val2 = (val2 >= 127) ? (val2 - 127) : (val2 + 129);
		val_mask = AIC31xx_8BITS_MASK;	/* 8 bits */
	} else if (!strcmp(kcontrol->id.name, "HP Driver Gain")) {
		val = (val >= 0) ? (val - 0) : (val + 15);
		val2 = (val2 >= 0) ? (val2 - 0) : (val2 + 15);
		val_mask = 0xF;	/* 4 bits */
	} else if (!strcmp(kcontrol->id.name, "LO Driver Gain")) {
		val = (val/6 >= 1) ? ((val/6 +1)*6) : ((val/6 + 0)*6);
		val2 = (val2/6 >= 1) ? ((val2/6 +1)*6) : ((val2/6 + 0)*6);
		val_mask = 0x3;	/* 2 bits */
	} else if (!strcmp(kcontrol->id.name, "PGA Capture Volume")) {
		val = (val*2 >= 24) ? ((val*2 - 24)/2) : ((val*2 + 254)/2);
		val2 = (val2*2 >= 24) ? ((val2*2 - 24)/2) : ((val2*2 + 254)/2);
		val_mask = 0x7F;	/* 7 bits */
	} else if (!strcmp(kcontrol->id.name, "HP Analog Volume")) {
                val_mask = 0x7F; /* 7 Bits */
        } else if (!strcmp(kcontrol->id.name, "SPKR Analog Volume")) {
                val_mask = 0x7F; /* 7 Bits */
	} else {
		printk(KERN_ALERT "Invalid control name\n");
		return -1;
	}
	err = snd_soc_update_bits(codec, reg, val_mask, val);
	if (err < 0) {
		printk(KERN_ALERT "Error while updating bits\n");
		return err;
	}

	err = snd_soc_update_bits(codec, reg2, val_mask, val2);
	return err;
}

/*
 * __new_control_info - This function is to initialize data for new control
 * required to program the aic31xx registers.
 */
static int __new_control_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 65535;

	return 0;
}

/*
 * aic31xx_get_divs - get required divisor from the "aic31xx_divs" table.
 */
static inline int aic31xx_get_divs(int mclk, int rate)
{
	int i;

	DBG("###+ aic31xx_get_divs mclk(%d) rate(%d)\n", mclk, rate);

	for (i = 0; i < ARRAY_SIZE(aic31xx_divs); i++) {
		if ((aic31xx_divs[i].rate == rate)
		    && (aic31xx_divs[i].mclk == mclk)) {
			return i;
		}
	}
	printk(KERN_ALERT "Master clock and sample rate is not supported\n");
	return -EINVAL;
}

int dac3100_change_page(struct snd_soc_codec *codec, u8 new_page)
{
	struct dac3100_priv *dac3100 = snd_soc_codec_get_drvdata(codec);;
	u8 data[2];

	data[0] = 0;
	data[1] = new_page;
	aic31xx->page_no = new_page;
	DBG("##dac3100_change_page => %d\nw 30 %02x %02x\n", new_page, data[0], data[1]);

//	DBG("w 30 %02x %02x\n", data[0], data[1]);

	if (codec->hw_write(codec->control_data, data, 2) != 2) {
		printk(KERN_ALERT "Error in changing page to %d\n", new_page);
		return -1;
	}
	return 0;
}

/*
 * aic31xx_add_widgets
 *
 * The following are the main widgets supported: Left DAC to Left Outputs,
 * Right DAC to Right Outputs, Left Inputs to Left ADC, Right Inputs to Right
 * ADC
 */
static int aic31xx_add_widgets(struct snd_soc_codec *codec)
{
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret = 0;
	DBG("##aic31xx_add_widgets\n");

	ret = snd_soc_dapm_new_controls(dapm, aic31xx_dapm_widgets,
					ARRAY_SIZE(aic31xx_dapm_widgets));
	if (!ret)
		printk(KERN_INFO "Completed adding dapm widgets size = %d\n",
					ARRAY_SIZE(aic31xx_dapm_widgets));

	ret = snd_soc_dapm_add_routes(dapm, aic31xx_dapm_routes,
					ARRAY_SIZE(aic31xx_dapm_routes));
	if (!ret)
		printk(KERN_INFO "Completed adding DAPM routes = %d\n",
				ARRAY_SIZE(aic31xx_dapm_routes));

	ret = snd_soc_dapm_new_widgets(dapm);

	if (!ret)
		printk(KERN_INFO "widgets updated\n");

	return 0;
}

/*
 * aic31xx_hw_params
 *
 * This function is to set the hardware parameters for aic31xx.  The
 * functions set the sample rate and audio serial data word length.
 */
static int aic31xx_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *tmp)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	int i;
	u8 data;

	DBG("##+ SET aic31xx_hw_params\n");

	mutex_lock(&aic31xx->mutex);

	dac3100_power_down(codec);
	codec->dapm.bias_level = 2;

	i = aic31xx_get_divs(aic31xx->sysclk, params_rate(params));
	DBG("##- Sampling rate: %d, %d\n", params_rate(params), i);


	if (i < 0) {
		printk(KERN_ALERT "sampling rate not supported\n");
		return i;
	}

	if (soc_static_freq_config) {

		/* We will fix R value to 1 and will make P & J=K.D as varialble */

		/* Setting P & R values */
		aic31xx_write(codec, CLK_REG_2,
			      ((aic31xx_divs[i].p_val << 4) | 0x01));

		/* J value */
		aic31xx_write(codec, CLK_REG_3, aic31xx_divs[i].pll_j);

		/* MSB & LSB for D value */
		aic31xx_write(codec, CLK_REG_4, (aic31xx_divs[i].pll_d >> 8));
		aic31xx_write(codec, CLK_REG_5,
			      (aic31xx_divs[i].pll_d & AIC31xx_8BITS_MASK));

		/* NDAC divider value */
		aic31xx_write(codec, NDAC_CLK_REG, aic31xx_divs[i].ndac);

		/* MDAC divider value */
		aic31xx_write(codec, MDAC_CLK_REG, aic31xx_divs[i].mdac);

		/* DOSR MSB & LSB values */
		aic31xx_write(codec, DAC_OSR_MSB, aic31xx_divs[i].dosr >> 8);
		aic31xx_write(codec, DAC_OSR_LSB,
			      aic31xx_divs[i].dosr & AIC31xx_8BITS_MASK);
	}
	/* BCLK N divider */
	aic31xx_write(codec, BCLK_N_VAL, aic31xx_divs[i].blck_N);

	data = aic31xx_read(codec, INTERFACE_SET_REG_1);

	data = data & ~(3 << 4);

	printk(KERN_ALERT "##- Data length: %d\n", params_format(params));

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		data |= (AIC31xx_WORD_LEN_20BITS << DAC_OSR_MSB_SHIFT);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		data |= (AIC31xx_WORD_LEN_24BITS << DAC_OSR_MSB_SHIFT);
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		data |= (AIC31xx_WORD_LEN_32BITS << DAC_OSR_MSB_SHIFT);
		break;
	}

 	/* Write to Page 0 Reg 27 for the Codec Interface control 1 Register */
	aic31xx_write(codec, INTERFACE_SET_REG_1, data);

  	/* Switch on the Codec into ON State after all the above configuration */
	dac3100_power_up(codec);

        /* Add the Processing blocks section as per the discussion with Design team */
        aic31xx_write (codec, DAC_PRB_SEL_REG, 0x03);

	DBG("##- SET dac3100_hw_params\n");

	mutex_unlock(&aic31xx->mutex);

	return 0;
}

/*
 * aic31xx_config_hp_volume
 *
 * Configure the I2C Transaction global variables. One of them is for ramping
 * down the HP.
 * Analog Volume and the other one is for ramping up the HP Analog Volume
 */
void aic31xx_config_hp_volume(struct snd_soc_codec *codec, int mute)
{
	struct i2c_client *client = codec->control_data;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	unsigned int count;
	struct aic31xx_configs  *pReg;
	signed char regval;
	unsigned char low_value;
	unsigned int  reg_update_count;

	/* User has requested to mute or bring down the Headphone Analog Volume
	 * Move from 0 db to -35.2 db
	 */
	if (mute > 0) {
		aic31xx_write(codec, L_ANLOG_VOL_2_HPL, 255);
        	aic31xx_write(codec, R_ANLOG_VOL_2_HPR, 255);
	} else {
		/* User has requested to unmute or bring up the Headphone Analog
		 * Volume Move from -35.2 db to 0 db
		 */
		aic31xx_write(codec, L_ANLOG_VOL_2_HPL, hp_l_analog_gain);
        	aic31xx_write(codec, R_ANLOG_VOL_2_HPR, hp_r_analog_gain);
	}

	/* Change to Page 1 */
	aic31xx_change_page(codec, 1);

	if (aic31xx->i2c_regs_status == 0) {
		for (count = 0; count < reg_update_count; count++) {
			i2c_right_transaction[count].addr = client->addr;
			i2c_right_transaction[count].flags =
				client->flags & I2C_M_TEN;
			i2c_right_transaction[count].len = 2;
			i2c_right_transaction[count].buf = (char *)
				&aic31xx->hp_analog_right_vol[count];
		}

		for (count = 0; count < reg_update_count; count++) {
			i2c_left_transaction[count].addr = client->addr;
			i2c_left_transaction[count].flags =
				client->flags & I2C_M_TEN;
			i2c_left_transaction[count].len = 2;
			i2c_left_transaction[count].buf = (char *)
				&aic31xx->hp_analog_left_vol[count];
		}
		aic31xx->i2c_regs_status = 1;
	}
	/* Perform bulk I2C transactions */
	if (i2c_transfer(client->adapter, i2c_right_transaction,
			reg_update_count) != reg_update_count) {
		printk(KERN_ERR "Error while Write brust i2c data error on "
				"R_ANLOG_VOL_2_HPR!\n");
	}
	if (i2c_transfer(client->adapter, i2c_left_transaction,
			reg_update_count) != reg_update_count) {
		printk(KERN_ERR "Error while Write brust i2c data error on "
				"L_ANLOG_VOL_2_HPL!\n");
	}
}

/*
 * aic31xx_adc_mute
 *
 * Mutes or Unmutes the ADC part of the Codec depending on the
 * value of the Mute flag.
 */
static int aic31xx_adc_mute(struct snd_soc_codec *codec, int mute)
{
	int retval = 0;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);

	if ((mute) && (aic31xx->mute != 1)) {
		retval = snd_soc_update_bits(codec, ADC_FGA, 0x80,
							(CLEAR | BIT7));
		aic31xx->mute = 1;
	} else if (!mute ) {
		retval = snd_soc_update_bits(codec, ADC_FGA, 0x80,
							(CLEAR & ~BIT7));
		aic31xx->mute = 0;
	}
#ifdef AIC31xx_DEBUG
	debug_print_registers(codec);
#endif
	return retval;
}

/*
 * aic31xx_dac_mute - mute or unmute the left and right DAC
 */
static int aic31xx_dac_mute(struct snd_soc_codec *codec, int mute)
{
	u8       dac_reg;
	volatile u8 value;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	volatile u16 time_out_counter;

	DBG("##+ new dac3100_mute %d (current state is %d, headset_connected=%d) \n",
               mute, aic31xx->mute, aic31xx->headset_connected);

	dac_reg = aic31xx_read(codec, DAC_MUTE_CTRL_REG);

	/* Also update the global Playback Status Flag. This is required for
           biquad update. */
	if ((mute) && (aic31xx->mute != 1)) {
		aic31xx->playback_status = 0;
                if (aic31xx->headset_connected) {

                        /* Switch ON the Class_D Speaker Amplifier */
			value = aic31xx_read (codec, CLASSD_SPEAKER_AMP);
			aic31xx_write (codec, CLASSD_SPEAKER_AMP, (value | 0x80));

                        /* Page 47 of the datasheets requires unmuting HP and
                           Speaker drivers first */
                        /* MUTE the Headphone Left and Right */
                        value = aic31xx_read (codec, HPL_DRIVER);
                        aic31xx_write (codec, HPL_DRIVER, (value & ~0x04));

                        value = aic31xx_read (codec, HPR_DRIVER);
                        aic31xx_write (codec, HPR_DRIVER, (value & ~0x04));
                        DBG("##MUTED the HPL and HPR DRIVER REGS\n");
#if 1
                    aic31xx_config_hp_volume (codec, mute);
//                        mdelay(10);
#else
                    /* Bring the HP Analog Volume Control Registers back to default value */
                    value = aic31xx_read (codec, RIGHT_ANALOG_HPR);
                    while (value < hp_l_analog_gain) {
                            value++;
                            aic31xx_write (codec, R_ANLOG_VOL_2_HPR, value);
                            mdelay(2);
                            aic31xx_write (codec, L_ANLOG_VOL_2_HPL, value);
                            mdelay(2);
                    }

                    DBG("Moved RIGHT_ANALOG_HPR to %d\r\n", (value & 0x7F));
#endif

                } else {
                        /* MUTE THE Class-D Speaker Driver */
                        value = aic31xx_read (codec, SPL_DRIVER);
                        aic31xx_write (codec, SPL_DRIVER, (value & ~0x04));

                        DBG("##SPL MUTE REGS\n");
                }

		DBG("##muting DAC .................\n");

	        aic31xx_write (codec, DAC_MUTE_CTRL_REG, dac_reg | MUTE_ON);

                DBG("##DAC MUTE Completed..\r\n");

                /* Change the DACL and DACR volumes values to lowest value */
                aic31xx_write (codec, LDAC_VOL, 0x81);
                aic31xx_write (codec, RDAC_VOL, 0x81);

                do {
                        mdelay(5);
                        /* Poll the DAC_FLAG register Page 0 38 for the DAC MUTE
                           Operation Completion Status */
                        value = aic31xx_read (codec, DAC_FLAG_2);
                        time_out_counter++;
                } while ((time_out_counter < 20) && ((value & 0x11) == 0));
                DBG("##DAC Vol Poll Completed counter  %d regval %x\r\n",
                    time_out_counter, value);

		aic31xx->mute = 1;
	} else if ((!mute) && (aic31xx->mute != 0)) {
               	aic31xx->playback_status = 1;
                /* We will enable the DAC UNMUTE first and finally the
                   Headphone UNMUTE to avoid pops */


                if (aic31xx->headset_connected) {
                        /*Read the contents of the Page 0 Reg 63 DAC Data-Path
                         Setup Register. Just retain the upper two bits and
                         lower two bits
                        */
                        value = (aic31xx_read(codec, DAC_CHN_REG) & 0xC3);
                        aic31xx_write(codec, DAC_CHN_REG, (value | LDAC_2_LCHN | RDAC_2_RCHN));
                	/* Restore the values of the DACL and DACR */
                	aic31xx_write (codec, LDAC_VOL, left_dac_gain);
                	aic31xx_write (codec, RDAC_VOL, right_dac_gain);

			time_out_counter = 0;
                	do {
                	        mdelay (5);
                	        value = aic31xx_read (codec, DAC_FLAG_2);
                	        time_out_counter ++;
                	} while ((time_out_counter < 100) && ((value & 0x11) == 0));

			DBG("##Changed DAC Volume back counter %d.\n", time_out_counter);

	                aic31xx_write (codec, DAC_MUTE_CTRL_REG, (dac_reg & ~MUTE_ON));

	                DBG ("##DAC UNMUTED ...\n");
#if 1
                    aic31xx_config_hp_volume (codec, mute);
    //                    mdelay(10);
#else
                    /* Bring the HP Analog Volume Control Registers back to default value */
                    value = aic31xx_read (codec, RIGHT_ANALOG_HPR);
                    while (value > hp_l_analog_gain) {
                            value--;
                            aic31xx_write (codec, RIGHT_ANALOG_HPR, value);
                            mdelay(2);
                            aic31xx_write (codec, LEFT_ANALOG_HPL, value);
                            mdelay(2);
                    }

                    DBG("Moved RIGHT_ANALOG_HPR to %d\r\n", (value & 0x7F));

#endif
                    /* Page 47 of the datasheets requires unmuting HP and
                       Speaker drivers first */
                    /* UNMUTE the Headphone Left and Right */
                    value = aic31xx_read (codec, HPL_DRIVER);
                    aic31xx_write (codec, HPL_DRIVER, (value | 0x04));

                    value = aic31xx_read (codec, HPR_DRIVER);
                    aic31xx_write (codec, HPR_DRIVER, (value | 0x04));
                    DBG("##UNMUTED the HPL and HPR DRIVER REGS\n");


                    /* Switch OFF the Class_D Speaker Amplifier */
                    value = aic31xx_read (codec, CLASSD_SPEAKER_AMP);
                    aic31xx_write (codec, CLASSD_SPEAKER_AMP, (value & ~0x80));


                } else {
                        /*Read the contents of the Page 0 Reg 63 DAC Data-Path
                         Setup Register. Just retain the upper two bits and
                         lower two bits
                        */
                        value = (aic31xx_read(codec, DAC_CHN_REG) & 0xC3);
                        aic31xx_write(codec, DAC_CHN_REG,
                                      (value | LDAC_LCHN_RCHN_2));

                	/* Restore the values of the DACL and DACR */
                	aic31xx_write (codec, LDAC_VOL, left_dac_gain);
                	aic31xx_write (codec, RDAC_VOL, right_dac_gain);

			time_out_counter = 0;
                	do {
                	        mdelay (5);
                	        value = aic31xx_read (codec, DAC_FLAG_2);
                	        time_out_counter ++;
                	} while ((time_out_counter < 100) && ((value & 0x11) == 0));

			DBG("##Changed DAC Volume back counter %d.\n", time_out_counter);

	                aic31xx_write (codec, DAC_MUTE_CTRL_REG, (dac_reg & ~MUTE_ON));

	                DBG ("##DAC UNMUTED ...\n");
                        /* UNMUTE THE Class-D Speaker Driver */
                        value = aic31xx_read (codec, SPL_DRIVER);
                        aic31xx_write (codec, SPL_DRIVER, (value | 0x04));

                        DBG("##SPL UNMUTE REGS\n");
                }

        	aic31xx->power_status = 1;
        	aic31xx->mute = 0;
        }

	DBG("##-dac3100_mute %d\n", mute);

	return 0;
}

/*
 * aic31xx_mute- mute or unmute the left and right DAC
 */
static int aic31xx_mute(struct snd_soc_dai *dai, int mute)
{
	int result = 0;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(dai->codec);

        DBG ("##dac3100_mute handler mute=%d.\r\n", mute);

	mutex_lock(&aic31xx->mutex);

	result = aic31xx_dac_mute(dai->codec, mute);

	mutex_unlock(&aic31xx->mutex);

	return result;
}

/* aic31xx_set_dai_sysclk -
 * DAI Callback function to set the Codec Master Clock Input
 */
static int aic31xx_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				   int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);

	DBG("##aic31xx_set_dai_sysclk clk_id(%d) (%d)\n", clk_id, freq);

	switch (freq) {
	case dac3100_FREQ_12000000:
	case dac3100_FREQ_24000000:
	case dac3100_FREQ_13000000:
		aic31xx->sysclk = freq;
		return 0;
	}
	printk(KERN_ALERT "Invalid frequency to set DAI system clock\n");
	return -EINVAL;
}

/* aic31xx_set_dai_fmt -
 * DAI call-back function for setting the Codec Operating Format
*/
static int aic31xx_set_dai_fmt(struct snd_soc_dai *codec_dai,
				unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	u8 iface_reg;

	iface_reg = codec->read(codec, INTERFACE_SET_REG_1);
	iface_reg = iface_reg & ~(3 << 6 | 3 << 2);

	DBG("##+ aic31xx_set_dai_fmt (%x)\n", fmt);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		aic31xx->master = 1;
		iface_reg |= BIT_CLK_MASTER | WORD_CLK_MASTER;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		aic31xx->master = 0;
		break;
	default:
		printk(KERN_ALERT "Invalid DAI master/slave interface\n");
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface_reg |= (AIC31xx_DSP_MODE << CLK_REG_3_SHIFT);
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		iface_reg |= (AIC31xx_RIGHT_JUSTIFIED_MODE << CLK_REG_3_SHIFT);
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface_reg |= (AIC31xx_LEFT_JUSTIFIED_MODE << CLK_REG_3_SHIFT);
		break;
	default:
		printk(KERN_ALERT "Invalid DAI interface format\n");
		return -EINVAL;
	}

	DBG("##- aic31xx_set_dai_fmt (%x)\n", iface_reg);
	codec->write(codec, INTERFACE_SET_REG_1, iface_reg);
	return 0;
}

/*
 * aic31xx_set_bias_level - This function is to get triggered when dapm
 * events occurs.
 */
static int aic31xx_set_bias_level(struct snd_soc_codec *codec,
					enum snd_soc_bias_level level)
{
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	int result = 0;
	mutex_lock(&aic31xx->mutex);

	DBG("##++ aic31xx_set_bias_level %d\n", level);

	if (level == codec->dapm.bias_level) {
		DBG("##%s :Current & previous levels same\n", __func__);
		goto unlock;
	}

	DBG("###aic31xx_set_bias_level New Level %d\n", level);

	switch (level) {
	/* full On */
	case SND_SOC_BIAS_ON:
		DBG("###aic31xx_set_bias_level BIAS_ON\n");
		dac3100_power_up(codec);
		break;

	/* partial On */
	case SND_SOC_BIAS_PREPARE:
		DBG("###aic31xx_set_bias_level BIAS_PREPARE\n");
		break;

	/* Off, with power */
	case SND_SOC_BIAS_STANDBY:
		DBG("###aic31xx_set_bias_level STANDBY\n");
		dac3100_power_down(codec);
		break;

	/* Off, without power */
	case SND_SOC_BIAS_OFF:
		DBG("###aic31xx_set_bias_level OFF\n");
		aic31xx_dac_mute(codec, 1);
		dac3100_power_down(codec);
		break;
	}
	codec->dapm.bias_level = level;


	DBG("##-- aic31xx_set_bias_level\n");
unlock:
	mutex_unlock(&aic31xx->mutex);

	return result;
}

/* aic31xx_suspend - ALSA callback function called during
 * system level suspend operation
*/
static int aic31xx_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
	DBG(KERN_INFO "%s: entered\n");

	if (aic31xx->playback_status == 0) {
		aic31xx_set_bias_level(codec, SND_SOC_BIAS_OFF);

		/* Bit 7 of Page 1/ Reg 46 gives the soft powerdown control.
		 * Setting this bit will further reduces the amount of power
		 * consumption
		 */
		snd_soc_update_bits(codec, MICBIAS_CTRL, 0x80, BIT7);

		/*Disable Audio clock from FREF_CLK1_OUT*/
		omap_writew(omap_readw(0x4a30a314) & 0xFEFF, 0x4a30a314);

#ifdef CODEC_POWER_OFF
		gpio_set_value(AUDIO_CODEC_PWR_ON_GPIO, 0);
#endif
	}
	DBG("%s: Exiting\n", __func__);
	return 0;
}

/* aic31xx_resume - ALSA calback function called during
 * system level resume operation
 */
static int aic31xx_resume(struct snd_soc_codec *codec)
{
	DBG(KERN_INFO "%s: entered\n", __func__);

#ifdef CODEC_POWER_OFF
	gpio_set_value(AUDIO_CODEC_PWR_ON_GPIO, 1);

	/* sleep for 10 ms to allow the voltage to stabilize */
	msleep(20);
#endif

	/*Enable Audio clock from FREF_CLK1_OUT*/
	omap_writew(omap_readw(0x4a30a314) | ~0xFEFF, 0x4a30a314);

	/* Perform the Device Soft Power UP */
	snd_soc_update_bits(codec, MICBIAS_CTRL, 0x80, (CLEAR & ~BIT7));

	DBG("aic31xx_resume: Suspend_bias_level %d\r\n",
		codec->dapm.suspend_bias_level);
	DBG("aic31xx_resume: codec_bias_level %d\r\n", codec->dapm.bias_level);

	DBG("- aic31xx_resume\n");

	return 0;
}

#ifdef AIC31xx_GPIO_INTR_SUPPORT

static irqreturn_t aic31xx_codec_irq_handler(int irq, void *data)
{
	DBG(KERN_ALERT "interrupt of codec found\n");
	schedule_work(&codec_int_work);
	return IRQ_HANDLED;
}

#endif /* #ifdef AIC31xx_GPIO_INTR_SUPPORT */

/*
 * tlv320aic31xx_init - initialise the aic31xx driver register the mixer and
 * codec interfaces with the kernel.
 */
static int tlv320aic31xx_init(struct snd_soc_codec *codec)
{
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	int ret = 0;
	int i = 0;
	u32 val;

	DBG(KERN_ALERT "##+tlv320aic31xx_init\n");

	/* Initialize private data for the codec */
	aic31xx->mute              =  1;
	aic31xx->headset_connected =  0;
	aic31xx->power_status      =  0;
	aic31xx->playback_status   =  0;
	aic31xx->capture_stream    =  0;
	aic31xx->i2c_regs_status   =  0;

	mutex_init(&aic31xx->mutex);

	aic31xx->page_no = 0;
	aic31xx->page_no = 0;

	aic31xx_change_page(codec, 0x00);
	codec->write(codec, RESET, 0x01);
	mdelay(10);

	for (i = 0; i < NUM_INIT_REGS; i++) {
		codec->write(codec, aic31xx_reg_init[i].reg_offset,
			      aic31xx_reg_init[i].reg_val);
		mdelay(10); /* Added delay across register writes */
	}

	/* Power-off the DAC and Headphone Drivers initially */
	aic31xx->power_status = 1;
	aic31xx->headset_connected = 1;
	aic31xx->mute = 0;
	aic31xx_dac_mute(codec, 1);
	aic31xx->headset_connected = 0;

#ifdef AIC31xx_GPIO_INTR_SUPPORT
	ret = request_irq(codec_interrupt, aic31xx_codec_irq_handler,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
			  IRQF_DISABLED | IRQF_SHARED , "aic31xx_codec_int",
			  codec);
	if (ret != 0) {
		printk(KERN_ERR "%s: irq registration failed\n", __func__);
		free_irq(codec_interrupt, codec);
	}
#endif
	DBG(KERN_ALERT "##-tlv320aic31xx_init\n");
	return ret;

}

/*
 * DAI ops
 */
static struct snd_soc_dai_ops aic31xx_dai_ops = {
	.hw_params      = aic31xx_hw_params,
	.digital_mute   = aic31xx_mute,
	.set_sysclk     = aic31xx_set_dai_sysclk,
	.set_fmt        = aic31xx_set_dai_fmt,
};

/*
 * aic31xx_get_record_status
 *
 * Helper function which checks if the Record Path is
 * active within the Audio Codec Driver. This function
 * returns the following:
 * 0	- No Recording Session in Progress or Recording stopped
 * 1	- Recording session is active and in progress
 */
int aic31xx_get_record_status(void)
{
	/* Check the global Codec Priv Struct */
	if (unlikely(aic31xx == 0))
		return 0;

	/* Check the Priv Struct power_status and
	 * capture_status variable along with mute
	 * All these three will let us know if recording is
	 * active and in progress.
	 */
	if (aic31xx->power_status && aic31xx->capture_stream &&
			aic31xx->mute == 0)
		return 1;

	return 0;
}
EXPORT_SYMBOL_GPL(aic31xx_get_record_status);

#ifdef AIC31xx_GPIO_INTR_SUPPORT
/*
 * i2c_aic31xx_headset_access_work - Worker Thread Function
 */
static void aic31xx_codec_access_work(struct work_struct *work)
{
	/* place holder for any code related to GPIO 103 interrrupt
	 * its a generic configurable interrupt from the codec.
	 * Enable AIC31xx_GPIO_INTR_SUPPORT to enable the interrupt routines
	 * and this work queue function.
	 */
}

/*
 * codec_int_gpio_bh - general codec interrupt via GPIO
 */
static void codec_int_gpio_bh(struct work_struct *work)
{
	printk(KERN_DEBUG "Work Done\n");
}

#endif

typedef unsigned int (*hw_read_t)(struct snd_soc_codec *, unsigned int);

/*
 * aic31xx_probe - This is first driver function called by the SoC core
 * driver.
 */
static int aic31xx_probe(struct snd_soc_codec *codec)
{
	int ret = 0;
#ifdef ACCLAIM_SUPPORT
//	int gpio = AUDIO_CODEC_PWR_ON_GPIO;
#endif
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	struct aic31xx_jack_data *jack;
	DBG("+aic31xx_probe: function entered\r\n");

	if (!codec) {
		printk(KERN_ERR "aic31xx_probe: Codec not yet Registered..\n");
		return -ENODEV;
	}
#ifdef ACCLAIM_SUPPORT
// conflict w/ power on in tlv320aic31xx_init?????????????????????????  keyodi
//
//	ret = gpio_request(gpio, AUDIO_CODEC_PWR_ON_GPIO_NAME);
//	gpio_direction_output(gpio, 0);
//	gpio_set_value(gpio, 1);
#endif
	codec->control_data = aic31xx->control_data;
	codec->hw_write = (hw_write_t) i2c_master_send;
	codec->hw_read = (hw_read_t) i2c_master_recv;
	codec->read = aic31xx_read;
	codec->write = aic31xx_write;
	/* use switch-class based headset reporting if platform requires it */
	jack = &aic31xx->hs_jack;
	jack->sdev.name = "h2w";
	ret = switch_dev_register(&jack->sdev);
	if (ret) {
		printk(KERN_INFO"error registering switch device %d\n", ret);
		goto reg_err;
	}

	/* Codec Initialization Routine*/
	ret = tlv320aic31xx_init(codec);
	if (ret < 0) {
		printk(KERN_ERR "%s: Failed to initialise device\n", __func__);
		return ret;
	}
	snd_soc_add_controls(codec, aic31xx_snd_controls,
				ARRAY_SIZE(aic31xx_snd_controls));

	aic31xx_add_widgets(codec);
//#ifdef CONFIG_MINIDSP
//aic3111_minidsp_program(codec);
//aic3111_add_minidsp_controls(codec);
//#endif

#ifdef CONFIG_ADAPTIVE_FILTER
	aic31xx_add_biquads_controls(codec);

	aic31xx_add_EQ_mixer_controls(codec);

	aic31xx_parse_biquad_array(codec);
#endif

	return ret;

reg_err:
	kfree(aic31xx);
	return ret;
}

/*
 * Remove aic31xx soc device
 */
static int aic31xx_remove(struct snd_soc_codec *codec)
{
	/* power down chip */
	if (codec->control_data)
		aic31xx_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

/*
 * It is SoC Codec DAI structure which has DAI capabilities viz., playback
 * and capture, DAI runtime information viz. state of DAI and pop wait state,
 * and DAI private data.  The aic31xx rates ranges from 8k to 192k The PCM
 * bit format supported are 16, 20, 24 and 32 bits
 */
struct snd_soc_dai_driver aic31xx_dai = {
	.name = "tlv320aic31xx-dai",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = AIC31xx_RATES,
		.formats = AIC31xx_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = AIC31xx_RATES,
		.formats = AIC31xx_FORMATS,
	},
	.ops = &aic31xx_dai_ops,
};

static struct snd_soc_codec_driver aic31xx_codec = {
	.read = aic31xx_read,
	.write = aic31xx_write,
	.set_bias_level = aic31xx_set_bias_level,
	.reg_cache_size = AIC31xx_CACHEREGNUM,
	.reg_word_size = sizeof(u8),
	.reg_cache_default = aic31xx_reg,
	.probe = aic31xx_probe,
	.remove = aic31xx_remove,
	.suspend = NULL, //aic31xx_suspend,
	.resume = NULL, //aic31xx_resume,
};

/*
 * This function attaches the i2c client and initializes aic31xx codec.
 */
static int tlv320aic31xx_i2c_probe(struct i2c_client *i2c,
					const struct i2c_device_id *id)
{
	int ret;
#ifdef ACCLAIM_SUPPORT
	int codec_interrupt_gpio = AUDIO_CODEC_INTERRUPT;
	int codec_interrupt = 0;
#endif
	DBG(KERN_INFO "%s: started\n", __func__);
	aic31xx = kzalloc(sizeof(struct aic31xx_priv), GFP_KERNEL);
	if (aic31xx == NULL) {
		printk(KERN_ERR "tlv320aic31xx_i2c_probe: Failed to create "
				"Codec Private Data\n");
		return -ENOMEM;
	}

	aic31xx->control_data = i2c;
	i2c_set_clientdata(i2c, aic31xx);

#ifdef AIC31xx_GPIO_INTR_SUPPORT
	INIT_WORK(&codec_init_works, aic31xx_codec_access_work);
#endif

#ifdef ACCLAIM_SUPPORT
	ret = gpio_request(codec_interrupt_gpio, "Codec Interrupt");
	if (ret < 0) {
		printk(KERN_INFO "%s: error in gpio request. codec interrupt"
				"failed\n", __func__);
		return ret;
	}
	gpio_direction_input(codec_interrupt_gpio);
	codec_interrupt = OMAP_GPIO_IRQ(codec_interrupt_gpio);


	ret = gpio_request(AUDIO_CODEC_PWR_ON_GPIO,
				 AUDIO_CODEC_PWR_ON_GPIO_NAME);
	if (ret < 0) {
		printk(KERN_ERR "%s: Unable get gpio for CODEC POWER %d\n",
				__func__, AUDIO_CODEC_PWR_ON_GPIO);
	}
	gpio_direction_output(AUDIO_CODEC_PWR_ON_GPIO, 1);
	gpio_set_value(AUDIO_CODEC_PWR_ON_GPIO, 1);
	mdelay(10);

	ret = gpio_request(AUDIO_CODEC_RESET_GPIO, AUDIO_CODEC_RESET_GPIO_NAME);
	if (ret < 0) {
		printk(KERN_ERR "%s: Unable get gpio for Reset %d\n",
				__func__, AUDIO_CODEC_RESET_GPIO);
	}
	gpio_direction_output(AUDIO_CODEC_RESET_GPIO, 1);
	gpio_set_value(AUDIO_CODEC_RESET_GPIO, 1);
	mdelay(10);
#endif
	ret = snd_soc_register_codec(&i2c->dev, &aic31xx_codec,
				     &aic31xx_dai, 1);
printk("DEVICE :::::::::::::::::::::::::::: %s\n", i2c->name);

	if (ret < 0)
		printk(KERN_ERR "tlv320aic31xx_i2c_probe: failed to attach"
				" codec at addr\n");
	else
		DBG("aic31xx_register sucess...\r\n");
	DBG("tlv320aic31xx_i2c_probe exited...\r\n");
	return ret;
}

/*
 * This function removes the i2c client and uninitializes AIC31xx codec.
 */
static int __exit tlv320aic31xx_i2c_remove(struct i2c_client *i2c)
{
	snd_soc_unregister_codec(&i2c->dev);
	kfree(i2c_get_clientdata(i2c));

	return 0;
}

/* i2c Device ID Struct used during Driver Initialization and Shutdown */
static const struct i2c_device_id tlv320aic31xx_id[] = {
	{"tlv320aic31xx-codec", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, tlv320aic31xx_id);

/* Definition of the struct i2c_driver structure */
static struct i2c_driver tlv320aic31xx_i2c_driver = {
	.driver = {
		.name = "tlv320aic31xx-codec",
		.owner = THIS_MODULE,
	},
	.probe = tlv320aic31xx_i2c_probe,
	.remove = __exit_p(tlv320aic31xx_i2c_remove),
	.id_table = tlv320aic31xx_id,
};

/* I2C Init Routine */
static int __init aic31xx_i2c_init(void)
{
	int ret;
	DBG(KERN_INFO "%s: mod init\n", __func__);
	ret = i2c_add_driver(&tlv320aic31xx_i2c_driver);
	if (ret) {
		printk(KERN_ERR "%s: error regsitering i2c driver, %d\n",
					       __func__, ret);
	}
	return ret;
}

/* I2C Exit Routine */
static void __exit aic31xx_i2c_exit(void)
{
	i2c_del_driver(&tlv320aic31xx_i2c_driver);
}

module_init(aic31xx_i2c_init);
module_exit(aic31xx_i2c_exit);

MODULE_DESCRIPTION("ASoC TLV320AIC31xx codec driver");
MODULE_AUTHOR("ravindra@mistralsolutions.com");
MODULE_AUTHOR("santosh.s@mistralsolutions.com");
MODULE_AUTHOR("preetam@mistralsolutions.com");
MODULE_LICENSE("GPL");
