/*
 * SDRC register values for the Hynix H8MBX00U0MER-0EM
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ARCH_ARM_MACH_OMAP2_SDRAM_HYNIX_H8MBX00U0MER0EM
#define __ARCH_ARM_MACH_OMAP2_SDRAM_HYNIX_H8MBX00U0MER0EM

#include <plat/sdrc.h>

/* Hynix H8MBX00U0MER-0EM */
static struct omap_sdrc_params h8mbx00u0mer0em_sdrc_params[] = {
	[0] = {
		.rate        = 166000000,
		.actim_ctrla = 0x629db4c6, //V_ACTIMA_166
		.actim_ctrlb = 0x00012214, //V_ACTIMB_166
		.rfr_ctrl    = 0x0004dc01,
		.mr          = 0x00000032,
	},
	[1] = {
		.rate        = 165941176,
		.actim_ctrla = 0x629db4c6, //V_ACTIMA_166
		.actim_ctrlb = 0x00012214, //V_ACTIMB_166
		.rfr_ctrl    = 0x0004dc01,
		.mr          = 0x00000032,
	},
	[2] = {
		.rate        = 83000000,
		.actim_ctrla = 0x31512283,  // V_ACTIMA_83
		.actim_ctrlb = 0x0001220a,  //V_ACTIMB_83
		.rfr_ctrl    = 0x00025501,
		.mr          = 0x00000022,
	},
	[3] = {
		.rate        = 82970588,
		.actim_ctrla = 0x31512283,  // V_ACTIMA_83
		.actim_ctrlb = 0x0001220a,  // V_ACTIMB_83
		.rfr_ctrl    = 0x00025501,
		.mr          = 0x00000022,
	},
	[4] = {
		.rate        = 0
	},
};

#endif
