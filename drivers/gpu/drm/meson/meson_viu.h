/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2016 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 */

/* Video Input Unit */

#ifndef __MESON_VIU_H
#define __MESON_VIU_H

#include "meson_drv.h"
#include "meson_registers.h"

/* Translate legacy word addresses; VPP blending and scaling do not move. */
static inline unsigned int meson_viu_mif_reg(struct meson_drm *priv,
					  unsigned int reg)
{
	if (!priv->sc2) {
		if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A)) {
			if (reg == VD1_IF0_GEN_REG3)
				return 0x3216;
			if (reg == VD2_IF0_GEN_REG3)
				return 0x3236;
			return reg + 0x17b0;
		}
		return reg;
	}

	/* GEN through FIFO retain their order in both SC2 MIF banks. */
	if (reg >= VD1_IF0_GEN_REG && reg <= VD1_IF0_LUMA_FIFO_SIZE)
		return SC2_VD1_IF0_GEN_REG + reg - VD1_IF0_GEN_REG;
	if (reg >= VD2_IF0_GEN_REG && reg <= VD2_IF0_LUMA_FIFO_SIZE)
		return SC2_VD2_IF0_GEN_REG + reg - VD2_IF0_GEN_REG;

	switch (reg) {
	case VD1_IF0_RANGE_MAP_Y:
		return SC2_VD1_IF0_RANGE_MAP_Y;
	case VD1_IF0_RANGE_MAP_CB:
		return SC2_VD1_IF0_RANGE_MAP_CB;
	case VD1_IF0_RANGE_MAP_CR:
		return SC2_VD1_IF0_RANGE_MAP_CR;
	case VD1_IF0_GEN_REG2:
		return SC2_VD1_IF0_GEN_REG2;
	case VD1_IF0_GEN_REG3:
		return SC2_VD1_IF0_GEN_REG3;
	case VIU_VD1_FMT_CTRL:
		return SC2_VIU_VD1_FMT_CTRL;
	case VIU_VD1_FMT_W:
		return SC2_VIU_VD1_FMT_W;
	case VD2_IF0_RANGE_MAP_Y:
		return SC2_VD2_IF0_RANGE_MAP_Y;
	case VD2_IF0_RANGE_MAP_CB:
		return SC2_VD2_IF0_RANGE_MAP_CB;
	case VD2_IF0_RANGE_MAP_CR:
		return SC2_VD2_IF0_RANGE_MAP_CR;
	case VD2_IF0_GEN_REG2:
		return SC2_VD2_IF0_GEN_REG2;
	case VD2_IF0_GEN_REG3:
		return SC2_VD2_IF0_GEN_REG3;
	case VIU_VD2_FMT_CTRL:
		return SC2_VIU_VD2_FMT_CTRL;
	case VIU_VD2_FMT_W:
		return SC2_VIU_VD2_FMT_W;
	default:
		return reg;
	}
}

static inline unsigned int meson_viu_afbc_reg(struct meson_drm *priv,
					   unsigned int index,
					   unsigned int reg)
{
	/* Both SC2 decoders have the same field order, not a shared bank. */
	if (priv->sc2)
		return (index ? SC2_VD2_AFBC_ENABLE : SC2_AFBC_ENABLE) +
			reg - AFBC_ENABLE;
	return reg;
}

void meson_viu_video_afbc_select(struct meson_drm *priv,
				unsigned int index, bool afbc);

/* OSDx_BLKx_CFG */
#define OSD_MALI_SRC_EN		BIT(30)

#define OSD_CANVAS_SEL		16

#define OSD_ENDIANNESS_LE	BIT(15)
#define OSD_ENDIANNESS_BE	(0)

#define OSD_BLK_MODE_422	(0x03 << 8)
#define OSD_BLK_MODE_16		(0x04 << 8)
#define OSD_BLK_MODE_32		(0x05 << 8)
#define OSD_BLK_MODE_24		(0x07 << 8)

#define OSD_OUTPUT_COLOR_RGB	BIT(7)
#define OSD_OUTPUT_COLOR_YUV	(0)

#define OSD_COLOR_MATRIX_32_RGBA	(0x00 << 2)
#define OSD_COLOR_MATRIX_32_ARGB	(0x01 << 2)
#define OSD_COLOR_MATRIX_32_ABGR	(0x02 << 2)
#define OSD_COLOR_MATRIX_32_BGRA	(0x03 << 2)

#define OSD_COLOR_MATRIX_24_RGB		(0x00 << 2)

#define OSD_COLOR_MATRIX_16_RGB655	(0x00 << 2)
#define OSD_COLOR_MATRIX_16_RGB565	(0x04 << 2)

#define OSD_MALI_COLOR_MODE_R8		(0 << 8)
#define OSD_MALI_COLOR_MODE_YUV422	(1 << 8)
#define OSD_MALI_COLOR_MODE_RGB565	(2 << 8)
#define OSD_MALI_COLOR_MODE_RGBA5551	(3 << 8)
#define OSD_MALI_COLOR_MODE_RGBA4444	(4 << 8)
#define OSD_MALI_COLOR_MODE_RGBA8888	(5 << 8)
#define OSD_MALI_COLOR_MODE_RGB888	(7 << 8)
#define OSD_MALI_COLOR_MODE_YUV422_10B	(8 << 8)
#define OSD_MALI_COLOR_MODE_RGBA1010102	(9 << 8)

#define OSD_INTERLACE_ENABLED	BIT(1)
#define OSD_INTERLACE_ODD	BIT(0)
#define OSD_INTERLACE_EVEN	(0)

/* OSDx_CTRL_STAT */
#define OSD_ENABLE		BIT(21)
#define OSD_MEM_LINEAR_ADDR	BIT(2)
#define OSD_BLK0_ENABLE		BIT(0)

#define OSD_GLOBAL_ALPHA_SHIFT	12

/* OSDx_CTRL_STAT2 */
#define OSD_DPATH_MALI_AFBCD	BIT(15)
#define OSD_REPLACE_EN		BIT(14)
#define OSD_REPLACE_SHIFT	6
#define OSD_PENDING_STAT_CLEAN	BIT(1)

struct meson_osd_regs {
	u32 ctrl_stat;
	u32 ctrl_stat2;
	u32 blk0_cfg[5];
	u32 blk1_cfg4;
	u32 blk2_cfg4;
	u32 fifo_ctrl;
	u32 unpack_ctrl;
	/* PPS offsets 0..11 match the VPP_OSD_VSC_PHASE_STEP bank. */
	u32 sc_base;
	u32 coef_idx;
	u32 coef;
};

extern const struct meson_osd_regs meson_osd_regs[MESON_MAX_OSDS];
void meson_viu_osd_afbc_select(struct meson_drm *priv, unsigned int index,
			      bool enable);

void meson_viu_osd1_reset(struct meson_drm *priv);
void meson_viu_g12a_enable_osd1_afbc(struct meson_drm *priv);
void meson_viu_g12a_disable_osd1_afbc(struct meson_drm *priv);
void meson_viu_gxm_enable_osd1_afbc(struct meson_drm *priv);
void meson_viu_gxm_disable_osd1_afbc(struct meson_drm *priv);
void meson_viu_init(struct meson_drm *priv);

#endif /* __MESON_VIU_H */
