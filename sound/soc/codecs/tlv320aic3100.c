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

//#define CODEC_POWER_OFF
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
static struct snd_soc_codec *codec_work_var_glob;

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


#define	AIC31xx_CACHEREGNUM (ARRAY_SIZE(aic31xx_reg))

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
            /* 8k rate */
            {13000000, 8000, 1, 6, 3803, 128, 3, 27, 128, 3, 27, 4},
            /* 11.025k rate */
            {13000000, 11025, 1, 6, 1876, 128, 3, 19, 128, 3, 19, 4},
            /* 12K rate */
            {13000000, 12000, 1, 6, 3803, 128, 3, 18, 128, 3, 18, 4},
            /* 16k rate */
            {13000000, 16000, 1, 6, 6166, 128, 3, 14, 128, 3, 14, 4},
            /* 22.05k rate */
            {13000000, 22050, 1, 6, 5132, 128, 3, 10, 128, 3, 10, 4},
            /* 24k rate */
            {13000000, 24000, 1, 6, 3803, 128, 3, 9, 128, 3, 9, 4},
            /* 32k rate */
            {13000000, 32000, 1, 6, 6166, 128, 3, 7, 128, 3, 7, 4},
            /* 44.1k rate */
            {13000000, 44100, 1, 6, 5132, 128, 3, 5, 128, 3, 5, 4},
            /* 48k rate */
            {13000000, 48000, 1, 6, 6166, 128, 7, 2, 128, 7, 2, 4},
            /*96k rate */
            {13000000, 96000, 1, 6, 6166, 64, 7, 2, 64, 7, 2, 2},
            /*192k */
            {13000000, 192000, 1, 6, 6166, 32, 7, 2, 32, 7, 2, 1},

};

/* Caching the register values */
static const u8 aic31xx_reg[] = {
	/* Page 0 Register */
	0x00, 0x00, 0x02, 0x00, 0x00, 0x11, 0x04, 0x00,
	0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x80, 0x80,
	0x08, 0x00, 0x01, 0x01, 0x80, 0x80, 0x04, 0x00,
	0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x02, 0x00,
	0x02, 0x00, 0x00, 0x00, 0x01, 0x04, 0x00, 0x14,
	0x0C, 0x00, 0x00, 0x00, 0x0F, 0x38, 0x00, 0x00,
	0x00, 0x00, 0x00, 0xEE, 0x10, 0xD8, 0x7E, 0x73,
	0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	/* Page 1 Registers */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04,
	0x05, 0x3E, 0x00, 0x00, 0x7F, 0x7F, 0x7F, 0x7F,
	0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
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

	/* Clock settings */
	{CLK_REG_1, CODEC_MUX_VALUE},

	/* Switch off PLL while Initiazling Codec */
	{INTERFACE_SET_REG_1, BCLK_DIR_CTRL},

        /* DAC_MOD_CLK is BCLK source */
	{INTERFACE_SET_REG_2, DAC_MOD_CLK_2_BDIV_CLKIN},

	/* Configuring HP in Line out Mode */
	{HP_DRIVER_CTRL, 0x06},

	/* HP pop removal settings */
	{HP_POP_CTRL, (BIT7 | HP_POWER_UP_15_3_MSEC | HP_RAMP_UP_STIME_3_9MS |
			    HP_BANDGAP_COMMON_MODE)},

	/* Speaker Ramp up time scaled to 30.5ms */
        {PGA_RAMP_CTRL, 0x70},

	/* Headset Detect setting */
	{INTL_CRTL_REG_1, 0xC0},

	/* short circuit protection of HP and Speaker power bits */
	{HP_SPK_ERR_CTL, 3},

	/* Headset detection enabled by default and Debounce programmed to 32 ms
	 * for Headset Detection and 32ms for Headset button-press Detection
	 */
	{HEADSET_DETECT, (HP_DEBOUNCE_32_MS | HS_DETECT_EN |
					HS_BUTTON_PRESS_32_MS)},

	/* DAC PRB configured to PRB_2 */
	{DAC_PRB_SEL_REG, 0x02},

	/*DRC settings */
	{DRC_CTRL_3, 0xB6},
	{DRC_CTRL_2, 0x00},
	{DRC_CTRL_1, 0x1A}, /* Increased Threshold to -21db */

	/* Switch off BCLK_N Divider */
	{BCLK_N_VAL, 0x00},

	/* Setting up DAC Channel */
	{DAC_CHN_REG, LDAC_2_LCHN | RDAC_2_RCHN},

	/* Mistral: Updated this value from 0x00 to 0x0C to MUTE DAC Left and Right channels */
	{DAC_MUTE_CTRL_REG, 0x0C},

	/* Configure the DAC Volume to default settings */
        {VOL_MICDECT_ADC, 0x00},

        /* Headphone drivers */
	/* 0xCC Mistral: Updated this value from 0xC4. We do not need to Power
	   up HPL and HPR at startup */
	{HEADPHONE_DRIVER, 0x04},

	/* Mistral: Updated this value from 0xc6 to 0x06 */
	{CLASSD_SPEAKER_AMP, 0x06},

	/* DAC_L and DAC_R Output Mixer Routing */
	/* DAC_X is routed to the channel mixer amplifier */
	{DAC_MIX_CTRL, 0x44},
	/* HPL gain 0db initial */
	{L_ANLOG_VOL_2_HPL, 0x9E},
	/* HPR gain 0db initial */
	{R_ANLOG_VOL_2_HPR, 0x9E},
	/* Mistral: Updated Values en plus 0x80 - 0x92 */
	{L_ANLOG_VOL_2_SPL, 0x80},
        /* HPL mute and gain 0db */
	/* Mistral: Updated this value from 0x2. Put HPL to common-mode and MUTED state at startup */
	{HPL_DRIVER, 0x00},
	/* HPR mute and gain 0db */
	/* Mistral: Updated this value from 0x2. Put HPR to common-mode and MUTED state at startup */
	{HPR_DRIVER, 0x00},
	/* Keep the Speaker driver in MUTE State by Default */
        {SPL_DRIVER, 0x00},
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

static const DECLARE_TLV_DB_SCALE(dac_vol_tlv, -6350, 50, 0);
static const DECLARE_TLV_DB_SCALE(class_D_drv_tlv, 600, 600, 0);
static const DECLARE_TLV_DB_SCALE(hp_vol_tlv, -7830, 60, 0);
static const DECLARE_TLV_DB_SCALE(sp_vol_tlv, -7830, 60, 0);

/*
 * controls that need to be exported to the user space
 */
static const struct snd_kcontrol_new aic31xx_snd_controls[] = {
	/* DAC Volume Control */
	 SOC_DOUBLE_R_SX_TLV("DAC Playback Volume", LDAC_VOL, RDAC_VOL, 8,
						0xffffff81, 0x30, dac_vol_tlv),
	/* HP driver mute control */
	SOC_DOUBLE_R("HP driver mute", HPL_DRIVER, HPR_DRIVER, 2, 2, 0),

	/* SPK driver mute control */
	SOC_SINGLE("SP driver mute", SPL_DRIVER, 2, 2, 0),

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

	/* Added for Debugging */
	SOC_SINGLE("LoopBack_Control", INTERFACE_SET_REG_2, 4, 4, 0),

	/* SP Class-D driver output stage gain Control */
	SOC_SINGLE_TLV("Class-D driver Vol", SPL_DRIVER, 3, 0x04, 0,
							class_D_drv_tlv),

	/* HP Analog Gain Volume Control */
	SOC_DOUBLE_R_TLV("HP Analog Gain", L_ANLOG_VOL_2_HPL,
			R_ANLOG_VOL_2_HPR, 0, 0x7F, 1, hp_vol_tlv),

	/* SP Analog Gain Volume Control */
	SOC_SINGLE_TLV("SP Analog Gain(0 = 0 dB, 127 = -78.3 dB)",
			L_ANLOG_VOL_2_SPL, 0, 0x7F, 1, sp_vol_tlv),

	/* Program Registers */
	SOC_SINGLE_AIC31xx("Program Registers"),
};

/*
 * DAPM Mixer Controls
 */
/* Left Output Mixer */
static const struct snd_kcontrol_new
left_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("From DAC_L", DAC_MIX_CTRL, 6, 1, 0),
	SOC_DAPM_SINGLE("From MIC1LP", DAC_MIX_CTRL, 5, 1, 0),
	//SOC_DAPM_SINGLE("From MIC1RP", DAC_MIX_CTRL, 4, 1, 0),
};

/* Right Output Mixer - Valid only for AIC31xx,3110,3100 */
static const struct
snd_kcontrol_new right_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("From DAC_R", DAC_MIX_CTRL, 2, 1, 0),
	SOC_DAPM_SINGLE("From MIC1RP", DAC_MIX_CTRL, 1, 1, 0),
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
	/* DACs */
	SND_SOC_DAPM_DAC_E("Left DAC", "Left Playback", DAC_CHN_REG, 7, 0,
			aic31xx_dac_power_up_event, SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_DAC_E("Right DAC", "Right Playback", DAC_CHN_REG, 6, 0,
			aic31xx_dac_power_up_event, SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_POST_PMD),

	/* Output drivers */
	SND_SOC_DAPM_PGA_E("HPL Driver", HEADPHONE_DRIVER, 7, 0,
			NULL, 0, aic31xx_hp_power_up_event,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("HPR Driver", HEADPHONE_DRIVER, 6, 0,
			NULL, 0, aic31xx_hp_power_up_event,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	/* For AIC3100 as is mono only left
	 * channel class-D can be powered up/down
	 */
	SND_SOC_DAPM_PGA_E("SPL Class - D", CLASSD_SPEAKER_AMP, 7, 0, NULL, 0,
			aic31xx_sp_event, SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MIXER("Left Input Mixer", SND_SOC_NOPM, 0, 0,
			   &left_input_mixer_controls[0],
			   ARRAY_SIZE(left_input_mixer_controls)),

	SND_SOC_DAPM_MIXER("Right Input Mixer", SND_SOC_NOPM, 0, 0,
			   &right_input_mixer_controls[0],
			   ARRAY_SIZE(right_input_mixer_controls)),

	/* Output Mixers */
	SND_SOC_DAPM_MIXER("Left Output Mixer", SND_SOC_NOPM, 0, 0,
			left_output_mixer_controls,
			ARRAY_SIZE(left_output_mixer_controls)),
	SND_SOC_DAPM_MIXER("Right Output Mixer", SND_SOC_NOPM, 0, 0,
			right_output_mixer_controls,
			ARRAY_SIZE(right_output_mixer_controls)),

	/* PLL CLK */
	SND_SOC_DAPM_SUPPLY("PLLCLK", CLK_REG_2, 7, 0, pll_power_on_event,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("CODEC_CLK_IN", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("NDAC_DIV", NDAC_CLK_REG, 7, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("MDAC_DIV", MDAC_CLK_REG, 7, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("NADC_DIV", NADC_CLK_REG, 7, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("MADC_DIV", MADC_CLK_REG, 7, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("BCLK_N_DIV", BCLK_N_VAL, 7, 0, NULL, 0),

	/* No widgets are required for ADC since the DAC3100 Audio
         * Codec Chipset does not contain a ADC
         */

	/* Outputs */
	SND_SOC_DAPM_OUTPUT("HPL"),
	SND_SOC_DAPM_OUTPUT("HPR"),
	SND_SOC_DAPM_OUTPUT("SPL"),

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
	/* PLL Clock related settings*/
	{"CODEC_CLK_IN", NULL, "PLLCLK"},
	{"NDAC_DIV", NULL, "CODEC_CLK_IN"},
	{"NADC_DIV", NULL, "CODEC_CLK_IN"},
	{"MDAC_DIV", NULL, "NDAC_DIV"},
	{"MADC_DIV", NULL, "NADC_DIV"},
	{"BCLK_N_DIV", NULL, "MADC_DIV"},
	{"BCLK_N_DIV", NULL, "MDAC_DIV"},

	/* Clocks for DAC */
	{"Left DAC", NULL, "MDAC_DIV" },
	{"Right DAC", NULL, "MDAC_DIV"},
	{"Left DAC", NULL, "BCLK_N_DIV" },
	{"Right DAC", NULL, "BCLK_N_DIV"},

	/* Left Output */
	{"Left Output Mixer", "From DAC_L", "Left DAC"},
	{"Left Output Mixer", "From MIC1LP", "MIC1LP"},

	/* Right Output */
	{"Right Output Mixer", "From DAC_R", "Right DAC"},
	{"Right Output Mixer", "From MIC1RP", "MIC1RP"},

	/* HPL path */
	{"HPL Driver", NULL, "Left Output Mixer"},
	{"HPL", NULL, "HPL Driver"},

	/* HPR path */
	{"HPR Driver", NULL, "Right Output Mixer"},
	{"HPR", NULL, "HPR Driver"},

	/* SPK L path */
	{"SPL Class - D", NULL, "Left Output Mixer"},
	{"SPL", NULL, "SPL Class - D"},
};

#define AIC31xx_DAPM_ROUTE_NUM (sizeof(aic31xx_dapm_routes)/		\
				sizeof(struct snd_soc_dapm_route))

/* GPIO Interrupt Worker Thread related Global Vars */
static struct work_struct codec_int_work;

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
		printk(KERN_ERR "Error in i2c write (aic31xx_write)\n");
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
		printk(KERN_ALERT "Error in i2c write (new_control_put)\n");
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

	/* Check the id name of the kcontrol and configure the mask and
	 * shift */
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

	/* TODO: This block needs to be revisted */
	if (!strcmp(kcontrol->id.name, "DAC Playback Volume")) {
		val = (val >= 127) ? (val - 127) : (val + 129);
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

	/* Setting the playback status.
	 * Update the capture_stream Member of the Codec's Private structure
	 * to denote that we will be performing Audio capture from now on.
	 */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		aic31xx->playback_status = 1;
		aic31xx->playback_stream = 1;
	} else if ((substream->stream != SNDRV_PCM_STREAM_PLAYBACK) &&
							(codec->active < 2))
		aic31xx->playback_stream = 0;
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		aic31xx->playback_status = 1;
		aic31xx->capture_stream = 1;
	} else if ((substream->stream != SNDRV_PCM_STREAM_CAPTURE) &&
							(codec->active < 2))
		aic31xx->capture_stream = 0;

	codec->dapm.bias_level = 2;

	i = aic31xx_get_divs(aic31xx->sysclk, params_rate(params));

	if (i < 0) {
		printk(KERN_ALERT "sampling rate not supported\n");
		mutex_unlock(&aic31xx->mutex);
		return i;
	}

	if (soc_static_freq_config) {
		/*
		 * We will fix R value to 1 and will make P & J=K.D as
		 * varialble
		 */

		/* Setting P & R values */
		snd_soc_update_bits(codec, CLK_REG_2, 0x7F,
			      ((aic31xx_divs[i].p_val << 4) | 0x01));

		/* J value */
		snd_soc_update_bits(codec, CLK_REG_3, 0x3F,
					aic31xx_divs[i].pll_j);

		/* MSB & LSB for D value */
		snd_soc_update_bits(codec, CLK_REG_4, 0x3F,
					(aic31xx_divs[i].pll_d >> 8));
		snd_soc_update_bits(codec, CLK_REG_5, 0xFF,
			      (aic31xx_divs[i].pll_d & AIC31xx_8BITS_MASK));

		/* NDAC divider value */
		snd_soc_update_bits(codec, NDAC_CLK_REG, 0x7F,
						aic31xx_divs[i].ndac);

		/* MDAC divider value */
		snd_soc_update_bits(codec, MDAC_CLK_REG, 0x7F,
						aic31xx_divs[i].mdac);

		/* NADC and MADC divider values */
		snd_soc_update_bits(codec, NADC_CLK_REG, 0x7F,
						aic31xx_divs[i].nadc);
		snd_soc_update_bits(codec, MADC_CLK_REG, 0x7F,
						aic31xx_divs[i].madc);

		/* DOSR MSB & LSB values */
		snd_soc_update_bits(codec, DAC_OSR_MSB, 0x03,
						aic31xx_divs[i].dosr >> 8);
		snd_soc_update_bits(codec, DAC_OSR_LSB, 0xFF,
			      aic31xx_divs[i].dosr & AIC31xx_8BITS_MASK);
	}
	/* BCLK N divider */
	snd_soc_update_bits(codec, BCLK_N_VAL, 0x7F,
						aic31xx_divs[i].blck_N);

	data = codec->read(codec, INTERFACE_SET_REG_1);
	data = data & ~(3 << 4);

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

	/* Write to Page 0 Reg 27 for the Codec Interface control 1
	 * Register */
	codec->write(codec, INTERFACE_SET_REG_1, data);

	DBG("##- SET dac3100_hw_params\n");

	/*setting pmdown_time of pcm rtd structure to 0*/
	rtd->pmdown_time = 0;

	mutex_unlock(&aic31xx->mutex);
	return 0;
}

/*
 * aic31xx_dac_mute - mute or unmute the left and right DAC
 */
static int aic31xx_dac_mute(struct snd_soc_codec *codec, int mute)
{
	u8 dac_reg;
	volatile u8 value;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	volatile u16 time_out_counter;

	DBG("%s: mute = %d\t priv->mute = %d\t headset_detect = %d\n",
		__func__, mute, aic31xx->mute, aic31xx->headset_connected);

	dac_reg = codec->read(codec, DAC_MUTE_CTRL_REG);

	/* Also update the global Playback Status Flag. This is required for
           biquad update. */
	if ((mute) && (aic31xx->mute != 1)) {
		aic31xx->playback_status = 0;
                if (aic31xx->headset_connected) {

                        /* Switch ON the Class_D Speaker Amplifier */
			value = codec->read (codec, CLASSD_SPEAKER_AMP);
			codec->write(codec, CLASSD_SPEAKER_AMP, (value | 0x80));

                        /* Page 47 of the datasheets requires unmuting HP and
                           Speaker drivers first */
                        /* MUTE the Headphone Left and Right */
                        value = codec->read (codec, HPL_DRIVER);
                        codec->write(codec, HPL_DRIVER, (value & ~0x04));

                        value = codec->read (codec, HPR_DRIVER);
                        codec->write(codec, HPR_DRIVER, (value & ~0x04));
                        DBG("##MUTED the HPL and HPR DRIVER REGS\n");
                } else {
                        /* MUTE THE Class-D Speaker Driver */
                        value = codec->read (codec, SPL_DRIVER);
                        codec->write(codec, SPL_DRIVER, (value & ~0x04));

                        DBG("##SPL MUTE REGS\n");
                }

		DBG("##muting DAC .................\n");

	        codec->write(codec, DAC_MUTE_CTRL_REG, dac_reg | MUTE_ON);

                DBG("##DAC MUTE Completed..\r\n");

                do {
                        mdelay(5);
                        /* Poll the DAC_FLAG register Page 0 38 for the DAC MUTE
                           Operation Completion Status */
			value = codec->read(codec, DAC_FLAG_2);
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
                        value = (codec->read(codec, DAC_CHN_REG) & 0xC3);
                        codec->write(codec, DAC_CHN_REG, (value | LDAC_2_LCHN | RDAC_2_RCHN));
                	/* Restore the values of the DACL and DACR */
			codec->write(codec, LDAC_VOL, 0xFC);
			codec->write(codec, RDAC_VOL, 0xFC);

			time_out_counter = 0;
                	do {
                	        mdelay (5);
				value = codec->read(codec, DAC_FLAG_2);
                	        time_out_counter ++;
                	} while ((time_out_counter < 100) && ((value & 0x11) == 0));

			DBG("##Changed DAC Volume back counter %d.\n", time_out_counter);

	                codec->write(codec, DAC_MUTE_CTRL_REG, (dac_reg & ~MUTE_ON));

	                DBG ("##DAC UNMUTED ...\n");

                    /* Page 47 of the datasheets requires unmuting HP and
                       Speaker drivers first */
                    /* UNMUTE the Headphone Left and Right */
                    value = codec->read (codec, HPL_DRIVER);
                    codec->write(codec, HPL_DRIVER, (value | 0x04));

                    value = codec->read (codec, HPR_DRIVER);
                    codec->write(codec, HPR_DRIVER, (value | 0x04));
                    DBG("##UNMUTED the HPL and HPR DRIVER REGS\n");

                    /* Switch OFF the Class_D Speaker Amplifier */
                    value = codec->read (codec, CLASSD_SPEAKER_AMP);
                    codec->write (codec, CLASSD_SPEAKER_AMP, (value & ~0x80));

                } else {
                        /*Read the contents of the Page 0 Reg 63 DAC Data-Path
                         Setup Register. Just retain the upper two bits and
                         lower two bits
                        */
                        value = (codec->read(codec, DAC_CHN_REG) & 0xC3);
                        codec->write(codec, DAC_CHN_REG,
                                      (value | LDAC_LCHN_RCHN_2));

                	/* Restore the values of the DACL and DACR */
			codec->write(codec, LDAC_VOL, 0xFC);
			codec->write(codec, RDAC_VOL, 0xFC);

			time_out_counter = 0;
                	do {
                	        mdelay (5);
				value = codec->read(codec, DAC_FLAG_2);
                	        time_out_counter ++;
                	} while ((time_out_counter < 100) && ((value & 0x11) == 0));

			DBG("##Changed DAC Volume back counter %d.\n", time_out_counter);

	                codec->write (codec, DAC_MUTE_CTRL_REG, (dac_reg & ~MUTE_ON));

	                DBG ("##DAC UNMUTED ...\n");
                        /* UNMUTE THE Class-D Speaker Driver */
                        value = codec->read(codec, SPL_DRIVER);
                        codec->write(codec, SPL_DRIVER, (value | 0x04));

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
	struct snd_soc_codec *codec = dai->codec;
	DBG(KERN_INFO "%s: mute = %d\t priv_mute = %d\n", __func__, mute, aic31xx->mute);

	mutex_lock(&aic31xx->mutex);

	/* Check for playback and record status and accordingly
	 * mute or unmute the ADC or the DAC
	 */
	if ((mute == 1) && (codec->active != 0)) {
		if (aic31xx->playback_stream == 1) {
			printk(KERN_INFO "Session still active\n");
			mutex_unlock(&aic31xx->mutex);
		return 0;
		}
	}

	if (aic31xx->playback_stream)
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
		break;

	/* partial On */
	case SND_SOC_BIAS_PREPARE:
		DBG("###aic31xx_set_bias_level BIAS_PREPARE\n");
		break;

	/* Off, with power */
	case SND_SOC_BIAS_STANDBY:
		DBG("###aic31xx_set_bias_level STANDBY\n");
		break;

	/* Off, without power */
	case SND_SOC_BIAS_OFF:
		DBG("###aic31xx_set_bias_level OFF\n");
		break;
	}
	codec->dapm.bias_level = level;


	DBG("##-- aic31xx_set_bias_level\n");
unlock:
	mutex_unlock(&aic31xx->mutex);

	return result;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3100_headset_speaker_path
 * Purpose  : This function is to check for the presence of Headset and
 *            configure the Headphone of the Class D Speaker driver
 *            Registers appropriately.
 *
 *----------------------------------------------------------------------------
 */
static int dac3100_headset_speaker_path(struct snd_soc_codec *codec)
{
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	int headset_detect_gpio = AUDIO_CODEC_HPH_DETECT_GPIO;
	int headset_present = 0;

	mutex_lock(&aic31xx->mutex);

	headset_present = !(gpio_get_value(headset_detect_gpio));

	aic31xx->headset_connected = headset_present;

        DBG("%s: %d\n", __func__, headset_present);

        if (aic31xx->playback_status == 1) {

		/* If codec was not powered up, power up the same. */
		if(headset_present) {
			DBG("headset present and headset path Activated\n");
                        aic31xx_write(codec, HEADPHONE_DRIVER, 0xC4); // 0xCC ON
                        aic31xx_write(codec, CLASSD_SPEAKER_AMP, 0x06); // OFF
		} else {
			DBG( "headset removed and headset path "
                               "Deactivated\n");

                        aic31xx_write(codec, HEADPHONE_DRIVER ,0x04); // OFF
                        aic31xx_write(codec, CLASSD_SPEAKER_AMP ,0xC6 ); //ON
                }

                /* We will force the dac3100->mute to 1 to ensure that the
                 * following function executes completely.
                 */
                aic31xx->mute = 1;
                /* Now unmute the appropriate Codec sections with Volume Ramping */
                aic31xx_dac_mute(codec, 0);

#ifdef CONFIG_ADAPTIVE_FILTER
		/* Update the Biquad Array */
		aic31xx_update_biquad_array(codec, headset_present,
					    aic31xx->playback_status);
#endif
	}
	mutex_unlock(&aic31xx->mutex);
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : i2c_dac3100_headset_access_work
 * Purpose  : Worker Thread Function.
 *
 *----------------------------------------------------------------------------
 */
static void i2c_dac3100_headset_access_work (struct work_struct *work)
{
	dac3100_headset_speaker_path(codec_work_var_glob);
}

/* aic31xx_suspend - ALSA callback function called during
 * system level suspend operation
*/
static int aic31xx_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
	DBG(KERN_INFO "%s: entered\n", __func__);

	if (aic31xx->playback_status == 0) {
		aic31xx_set_bias_level(codec, SND_SOC_BIAS_OFF);

		/* Bit 7 of Page 1/ Reg 46 gives the soft powerdown control.
		 * Setting this bit will further reduces the amount of power
		 * consumption
		 */
		snd_soc_update_bits(codec, MICBIAS_CTRL, 0x80, BIT7);

		/*Disable Audio clock from FREF_CLK1_OUT*/
		//omap_writew(omap_readw(0x4a30a314) & 0xFEFF, 0x4a30a314);

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
	int i;

	DBG(KERN_INFO "%s: entered\n", __func__);

#ifdef CODEC_POWER_OFF
	gpio_set_value(AUDIO_CODEC_PWR_ON_GPIO, 1);
	/* sleep for 10 ms to allow the voltage to stabilize */
	msleep(50);
#endif

	for (i = 0; i < sizeof(aic31xx_reg_init) / sizeof(struct aic31xx_configs); i++) {
		aic31xx_write(codec, aic31xx_reg_init[i].reg_offset, aic31xx_reg_init[i].reg_val);
	}
	aic31xx_write(codec, CLK_REG_1, PLLCLK_2_CODEC_CLKIN);
	aic31xx_write(codec, L_ANLOG_VOL_2_HPL, 0x9E);
	aic31xx_write(codec, R_ANLOG_VOL_2_HPR, 0x9E);
	aic31xx_write(codec, L_ANLOG_VOL_2_SPL, 0x80);
	aic31xx_write(codec, R_ANLOG_VOL_2_SPR, 0x80);

	/* Perform the Device Soft Power UP */
	snd_soc_update_bits(codec, MICBIAS_CTRL, 0x80, (CLEAR & ~BIT7));

	/* Check to see if headphone is connected */
	dac3100_headset_speaker_path(codec);

	/* Added delay as per the suggestion from TI Audio team */
        mdelay (50);
	DBG("aic31xx_resume: Suspend_bias_level %d\r\n",
		codec->dapm.suspend_bias_level);
	DBG("aic31xx_resume: codec_bias_level %d\r\n", codec->dapm.bias_level);

	DBG("- aic31xx_resume\n");

	return 0;
}

static irqreturn_t aic31xx_codec_irq_handler(int irq, void *data)
{
	struct snd_soc_codec *codec = data;
	DBG(KERN_ALERT "interrupt of codec found\n");
	codec_work_var_glob = codec;
	schedule_work(&codec_int_work);
	return IRQ_HANDLED;
}

/*
 * tlv320aic31xx_init - initialise the aic31xx driver register the mixer and
 * codec interfaces with the kernel.
 */
static int tlv320aic31xx_init(struct snd_soc_codec *codec)
{
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	int ret = 0;
	int i = 0;
	int hph_detect_gpio = AUDIO_CODEC_HPH_DETECT_GPIO;
	int hph_detect_irq = 0;

	DBG(KERN_ALERT "##+tlv320aic31xx_init\n");

	/* Initialize private data for the codec */
	aic31xx->mute              =  1;
	aic31xx->headset_connected =  0;
	aic31xx->power_status      =  0;
	aic31xx->playback_status   =  0;
	aic31xx->i2c_regs_status   =  0;

	mutex_init(&aic31xx->mutex);

	ret = gpio_request(hph_detect_gpio, "dac3100-headset");

	if (ret < 0) {
		goto err1;
	}
	gpio_direction_input(hph_detect_gpio);
	hph_detect_irq = OMAP_GPIO_IRQ(hph_detect_gpio);

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

	ret = request_irq(hph_detect_irq, aic31xx_codec_irq_handler,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
			  IRQF_DISABLED | IRQF_SHARED , "aic31xx_codec_int",
			  codec);
	if (ret != 0) {
		printk(KERN_ERR "%s: irq registration failed\n", __func__);
		free_irq(hph_detect_irq, codec);
	}

	DBG(KERN_ALERT "##-tlv320aic31xx_init\n");
	return ret;

err1:
	free_irq(hph_detect_irq, codec);
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

	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	struct aic31xx_jack_data *jack;
	DBG("+aic31xx_probe: function entered\r\n");

	if (!codec) {
		printk(KERN_ERR "aic31xx_probe: Codec not yet Registered..\n");
		return -ENODEV;
	}

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
		printk(KERN_INFO "error registering switch device %d\n", ret);
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

#ifdef CONFIG_ADAPTIVE_FILTER
	aic31xx_add_biquads_controls(codec);

	aic31xx_add_EQ_mixer_controls(codec);

	aic31xx_parse_biquad_array(codec);
#endif

	dac3100_headset_speaker_path(codec);

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
		.channels_min = 1,
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
	.suspend = aic31xx_suspend,
	.resume = aic31xx_resume,
};

/*
 * This function attaches the i2c client and initializes aic31xx codec.
 */
static int tlv320aic31xx_i2c_probe(struct i2c_client *i2c,
					const struct i2c_device_id *id)
{
	int ret;
	int codec_interrupt_gpio = AUDIO_CODEC_INTERRUPT;
	int codec_interrupt = 0;

	DBG(KERN_INFO "%s: started\n", __func__);
	aic31xx = kzalloc(sizeof(struct aic31xx_priv), GFP_KERNEL);
	if (aic31xx == NULL) {
		printk(KERN_ERR "tlv320aic31xx_i2c_probe: Failed to create "
				"Codec Private Data\n");
		return -ENOMEM;
	}

	aic31xx->control_data = i2c;
	i2c_set_clientdata(i2c, aic31xx);

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

	INIT_WORK(&codec_int_work, i2c_dac3100_headset_access_work);

	ret = snd_soc_register_codec(&i2c->dev, &aic31xx_codec,
				     &aic31xx_dai, 1);

	if (ret < 0)
		printk(KERN_ERR "tlv320aic31xx_i2c_probe: failed to attach"
				" codec at addr\n");
	else
		DBG("aic31xx_register success...\r\n");

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
