// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2016 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
 * Copyright (C) 2014 Endless Mobile
 *
 * Written by:
 *     Jasper St. Pierre <jstpierre@mecheye.net>
 */

#include <linux/bitfield.h>
#include <linux/math64.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_device.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>

#include "meson_plane.h"
#include "meson_registers.h"
#include "meson_viu.h"
#include "meson_osd_afbcd.h"

/* OSD_SCI_WH_M1 */
#define SCI_WH_M1_W(w)			FIELD_PREP(GENMASK(28, 16), w)
#define SCI_WH_M1_H(h)			FIELD_PREP(GENMASK(12, 0), h)

/* OSD_SCO_H_START_END */
/* OSD_SCO_V_START_END */
#define SCO_HV_START(start)		FIELD_PREP(GENMASK(27, 16), start)
#define SCO_HV_END(end)			FIELD_PREP(GENMASK(11, 0), end)

/* OSD_SC_CTRL0 */
#define SC_CTRL0_PATH_EN		BIT(3)
#define SC_CTRL0_SEL_OSD1		BIT(2)

/* OSD_VSC_CTRL0 */
#define VSC_BANK_LEN(value)		FIELD_PREP(GENMASK(2, 0), value)
#define VSC_TOP_INI_RCV_NUM(value)	FIELD_PREP(GENMASK(6, 3), value)
#define VSC_TOP_RPT_L0_NUM(value)	FIELD_PREP(GENMASK(9, 8), value)
#define VSC_BOT_INI_RCV_NUM(value)	FIELD_PREP(GENMASK(14, 11), value)
#define VSC_BOT_RPT_L0_NUM(value)	FIELD_PREP(GENMASK(17, 16), value)
#define VSC_PROG_INTERLACE		BIT(23)
#define VSC_VERTICAL_SCALER_EN		BIT(24)

/* OSD_VSC_INI_PHASE */
#define VSC_INI_PHASE_BOT(bottom)	FIELD_PREP(GENMASK(31, 16), bottom)
#define VSC_INI_PHASE_TOP(top)		FIELD_PREP(GENMASK(15, 0), top)

/* OSD_HSC_CTRL0 */
#define HSC_BANK_LENGTH(value)		FIELD_PREP(GENMASK(2, 0), value)
#define HSC_INI_RCV_NUM0(value)		FIELD_PREP(GENMASK(6, 3), value)
#define HSC_RPT_P0_NUM0(value)		FIELD_PREP(GENMASK(9, 8), value)
#define HSC_HORIZ_SCALER_EN		BIT(22)

/* VPP_OSD_VSC_PHASE_STEP */
/* VPP_OSD_HSC_PHASE_STEP */
#define SC_PHASE_STEP(value)		FIELD_PREP(GENMASK(27, 0), value)

struct meson_plane {
	struct drm_plane base;
	struct meson_drm *priv;
	unsigned int index;
	bool enabled;
};
#define to_meson_plane(x) container_of(x, struct meson_plane, base)

#define FRAC_16_16(mult, div)    (((mult) << 16) / (div))

/* SC2's OSD1 PPS follows the blender; OSD2/3 PPS precede it. */
static void meson_plane_blend_size(struct meson_drm *priv,
				   struct drm_atomic_state *state,
				   struct drm_crtc_state *crtc_state,
				   unsigned int *width, unsigned int *height)
{
	struct drm_plane_state *primary;

	primary = drm_atomic_get_new_plane_state(state, priv->primary_plane);
	if (!primary)
		primary = priv->primary_plane->state;
	*width = crtc_state->mode.hdisplay;
	*height = crtc_state->mode.vdisplay;
	if (primary->crtc && primary->fb) {
		*width = drm_rect_width(&primary->src) >> 16;
		*height = drm_rect_height(&primary->src) >> 16;
	}
}

static struct drm_rect meson_plane_blend_dest(struct drm_plane_state *plane_state,
					     struct drm_crtc_state *crtc_state,
					     unsigned int width,
					     unsigned int height)
{
	struct drm_rect dest = plane_state->dst;

	dest.x1 = DIV_ROUND_CLOSEST_ULL((u64)dest.x1 * width,
					crtc_state->mode.hdisplay);
	dest.x2 = DIV_ROUND_CLOSEST_ULL((u64)dest.x2 * width,
					crtc_state->mode.hdisplay);
	dest.y1 = DIV_ROUND_CLOSEST_ULL((u64)dest.y1 * height,
					crtc_state->mode.vdisplay);
	dest.y2 = DIV_ROUND_CLOSEST_ULL((u64)dest.y2 * height,
					crtc_state->mode.vdisplay);
	return dest;
}

static int meson_plane_atomic_check(struct drm_plane *plane,
				    struct drm_atomic_state *state)
{
	struct drm_plane_state *new_plane_state = drm_atomic_get_new_plane_state(state,
										 plane);
	struct meson_plane *meson_plane = to_meson_plane(plane);
	struct meson_drm *priv = meson_plane->priv;
	struct drm_crtc *crtc = new_plane_state->crtc;
	struct drm_crtc_state *crtc_state;
	struct drm_plane_state *osd_state;
	struct drm_rect dest;
	unsigned int width, height, i;
	int ret;

	if (!crtc) {
		osd_state = drm_atomic_get_old_plane_state(state, plane);
		crtc = osd_state->crtc;
	}
	if (!crtc)
		return 0;

	crtc_state = drm_atomic_get_crtc_state(state, crtc);
	if (IS_ERR(crtc_state))
		return PTR_ERR(crtc_state);

	/* A primary scale/enable change changes every OSD's blend coordinates. */
	if (priv->sc2) {
		for (i = 0; i < MESON_MAX_OSDS; i++) {
			osd_state = drm_atomic_get_plane_state(state, priv->osd_planes[i]);
			if (IS_ERR(osd_state))
				return PTR_ERR(osd_state);
		}
	}
	if (!new_plane_state->crtc)
		return 0;

	ret = drm_atomic_helper_check_plane_state(new_plane_state, crtc_state,
			FRAC_16_16(1, 5), meson_plane->index ?
			FRAC_16_16(5, 1) : DRM_PLANE_NO_SCALING,
			meson_plane->index != 0, true);
	if (ret || !meson_plane->index || !new_plane_state->visible)
		return ret;

	/* The extra PPS consumes integer crop coordinates and progressive RGB. */
	if ((new_plane_state->src.x1 | new_plane_state->src.y1 |
	     new_plane_state->src.x2 | new_plane_state->src.y2) & 0xffff)
		return -EINVAL;
	meson_plane_blend_size(priv, state, crtc_state, &width, &height);
	dest = meson_plane_blend_dest(new_plane_state, crtc_state, width, height);
	if (drm_rect_width(&dest) <= 0 || drm_rect_height(&dest) <= 0 ||
	    dest.x2 > 4096 || dest.y2 > 4096 ||
	    ((u64)drm_rect_width(&new_plane_state->src) >> 16) >
		5 * drm_rect_width(&dest) ||
	    ((u64)drm_rect_height(&new_plane_state->src) >> 16) >
		5 * drm_rect_height(&dest))
		return -EINVAL;
	return 0;
}

#define MESON_MOD_AFBC_VALID_BITS (AFBC_FORMAT_MOD_BLOCK_SIZE_16x16 |	\
				   AFBC_FORMAT_MOD_BLOCK_SIZE_32x8 |	\
				   AFBC_FORMAT_MOD_YTR |		\
				   AFBC_FORMAT_MOD_SPARSE |		\
				   AFBC_FORMAT_MOD_SPLIT)

/* Takes a fixed 16.16 number and converts it to integer. */
static inline int64_t fixed16_to_int(int64_t value)
{
	return value >> 16;
}

static u32 meson_g12a_afbcd_line_stride(const struct meson_osd_state *osd)
{
	u32 line_stride = 0;

	switch (osd->format) {
	case DRM_FORMAT_RGB565:
		line_stride = ((osd->width << 4) + 127) >> 7;
		break;
	case DRM_FORMAT_RGB888:
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_ABGR8888:
		line_stride = ((osd->width << 5) + 127) >> 7;
		break;
	}

	return ((line_stride + 1) >> 1) << 1;
}

static void meson_plane_atomic_update(struct drm_plane *plane,
				      struct drm_atomic_state *state)
{
	struct meson_plane *meson_plane = to_meson_plane(plane);
	struct drm_plane_state *new_state = drm_atomic_get_new_plane_state(state,
									   plane);
	struct drm_rect dest = drm_plane_state_dest(new_state);
	struct meson_drm *priv = meson_plane->priv;
	struct meson_osd_state *osd = &priv->viu.osd[meson_plane->index];
	const struct meson_osd_regs *regs = &meson_osd_regs[meson_plane->index];
	struct drm_crtc_state *crtc_state =
		drm_atomic_get_new_crtc_state(state, new_state->crtc);
	unsigned int blend_w, blend_h;
	struct drm_framebuffer *fb = new_state->fb;
	struct drm_gem_dma_object *gem;
	unsigned long flags;
	int vsc_ini_rcv_num, vsc_ini_rpt_p0_num;
	int vsc_bot_rcv_num, vsc_bot_rpt_p0_num;
	int hsc_ini_rcv_num, hsc_ini_rpt_p0_num;
	int hf_phase_step, vf_phase_step;
	int src_w, src_h, dst_w, dst_h;
	int bot_ini_phase;
	int hf_bank_len;
	int vf_bank_len;
	u8 canvas_id;
	u32 global_alpha = 0x100;
	bool sc2_interlace = priv->sc2 && !meson_plane->index &&
		(new_state->crtc->mode.flags & DRM_MODE_FLAG_INTERLACE);

	if (!new_state->visible) {
		spin_lock_irqsave(&priv->drm->event_lock, flags);
		osd->enabled = false;
		meson_plane->enabled = false;
		spin_unlock_irqrestore(&priv->drm->event_lock, flags);
		return;
	}

	if (priv->sc2)
		global_alpha = DIV_ROUND_CLOSEST(new_state->alpha * 0x100,
					       DRM_BLEND_ALPHA_OPAQUE);

	/*
	 * Update Coordinates
	 * Update Formats
	 * Update Buffer
	 * Enable Plane
	 */
	spin_lock_irqsave(&priv->drm->event_lock, flags);

	/* Check if AFBC decoder is required for this buffer */
	if ((meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXM) ||
	     meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A)) &&
	    fb->modifier & DRM_FORMAT_MOD_ARM_AFBC(MESON_MOD_AFBC_VALID_BITS))
		osd->afbcd = true;
	else
		osd->afbcd = false;

	/* SC2 exposes the OSD's nine-bit global alpha through atomic state. */
	osd->ctrl_stat = OSD_ENABLE |
				   (global_alpha << OSD_GLOBAL_ALPHA_SHIFT) |
				   OSD_BLK0_ENABLE;

	osd->ctrl_stat2 = readl(priv->io_base + _REG(regs->ctrl_stat2));

	canvas_id = priv->canvas_id_osd[meson_plane->index];

	/* Set up BLK0 to point to the right canvas */
	osd->blk0_cfg[0] = canvas_id << OSD_CANVAS_SEL;

	if (osd->afbcd) {
		if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A)) {
			/* SC2's 3840x2176x4 output needs a 32-MiB virtual slot. */
			osd->blk1_cfg4 = MESON_G12A_AFBCD_OUT_ADDR *
				(1 + meson_plane->index * (priv->sc2 ? 2 : 1));
			osd->blk0_cfg[0] |= OSD_ENDIANNESS_BE;
			osd->ctrl_stat2 |= OSD_PENDING_STAT_CLEAN;
			osd->ctrl_stat |= VIU_OSD1_CFG_SYN_EN;
		}

		if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXM)) {
			osd->blk0_cfg[0] |= OSD_ENDIANNESS_LE;
			osd->ctrl_stat2 |= OSD_DPATH_MALI_AFBCD;
		}
	} else {
		osd->blk0_cfg[0] |= OSD_ENDIANNESS_LE;

		if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXM))
			osd->ctrl_stat2 &= ~OSD_DPATH_MALI_AFBCD;
	}

	/* On GXBB, Use the old non-HDR RGB2YUV converter */
	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXBB))
		osd->blk0_cfg[0] |= OSD_OUTPUT_COLOR_RGB;

	if (osd->afbcd &&
	    meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A)) {
		osd->blk0_cfg[0] |= OSD_MALI_SRC_EN |
			priv->afbcd.ops->fmt_to_blk_mode(fb->modifier,
							  fb->format->format);
	} else {
		switch (fb->format->format) {
		case DRM_FORMAT_XRGB8888:
		case DRM_FORMAT_ARGB8888:
			osd->blk0_cfg[0] |= OSD_BLK_MODE_32 |
						OSD_COLOR_MATRIX_32_ARGB;
			break;
		case DRM_FORMAT_XBGR8888:
		case DRM_FORMAT_ABGR8888:
			osd->blk0_cfg[0] |= OSD_BLK_MODE_32 |
						OSD_COLOR_MATRIX_32_ABGR;
			break;
		case DRM_FORMAT_RGB888:
			osd->blk0_cfg[0] |= OSD_BLK_MODE_24 |
						OSD_COLOR_MATRIX_24_RGB;
			break;
		case DRM_FORMAT_RGB565:
			osd->blk0_cfg[0] |= OSD_BLK_MODE_16 |
						OSD_COLOR_MATRIX_16_RGB565;
			break;
		}
	}

	switch (fb->format->format) {
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_XBGR8888:
		/* For XRGB, replace the pixel's alpha by 0xFF */
		osd->ctrl_stat2 |= OSD_REPLACE_EN;
		break;
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_ABGR8888:
		/* For ARGB, use the pixel's alpha */
		osd->ctrl_stat2 &= ~OSD_REPLACE_EN;
		break;
	}

	/* Default scaler parameters */
	vsc_bot_rcv_num = 0;
	vsc_bot_rpt_p0_num = 0;
	hf_bank_len = 4;
	vf_bank_len = 4;

	if (!meson_plane->index &&
	    (new_state->crtc->mode.flags & DRM_MODE_FLAG_INTERLACE)) {
		vsc_bot_rcv_num = 6;
		vsc_bot_rpt_p0_num = 2;
	}

	hsc_ini_rcv_num = hf_bank_len;
	vsc_ini_rcv_num = vf_bank_len;
	hsc_ini_rpt_p0_num = (hf_bank_len / 2) - 1;
	vsc_ini_rpt_p0_num = (vf_bank_len / 2) - 1;

	src_w = drm_rect_width(&new_state->src) >> 16;
	src_h = drm_rect_height(&new_state->src) >> 16;
	dst_w = new_state->crtc_w;
	dst_h = new_state->crtc_h;
	if (meson_plane->index) {
		meson_plane_blend_size(priv, state, crtc_state, &blend_w, &blend_h);
		dest = meson_plane_blend_dest(new_state, crtc_state, blend_w, blend_h);
		dst_w = drm_rect_width(&dest);
		dst_h = drm_rect_height(&dest);
	}

	/*
	 * When the output is interlaced, the OSD must switch between
	 * each field using the INTERLACE_SEL_ODD (0) of VIU_OSD1_BLK0_CFG_W0
	 * at each vsync.
	 * But the vertical scaler can provide such funtionnality if
	 * is configured for 2:1 scaling with interlace options enabled.
	 */
	if (!meson_plane->index &&
	    (new_state->crtc->mode.flags & DRM_MODE_FLAG_INTERLACE)) {
		dest.y1 /= 2;
		dest.y2 /= 2;
		dst_h /= 2;
	}

	hf_phase_step = div_u64((u64)src_w << 24, dst_w);
	vf_phase_step = div_u64((u64)src_h << 20, dst_h);

	if (!meson_plane->index &&
	    (new_state->crtc->mode.flags & DRM_MODE_FLAG_INTERLACE))
		bot_ini_phase = ((vf_phase_step / 2) >> 4);
	else
		bot_ini_phase = 0;

	if (sc2_interlace) {
		/* Progressive OSD input to top/bottom fields, as on G12B+. */
		vsc_bot_rcv_num = vf_bank_len;
		vsc_bot_rpt_p0_num = vsc_ini_rpt_p0_num;
		bot_ini_phase = vf_phase_step >> 5;
		/* Upscaling-only checks limit the field scaler ratio to 2:1. */
		if (bot_ini_phase >= BIT(16)) {
			vsc_bot_rpt_p0_num--;
			bot_ini_phase -= BIT(16);
		}
	}

	vf_phase_step = (vf_phase_step << 4);

	/* In interlaced mode, scaler is always active */
	if (sc2_interlace || src_h != dst_h || src_w != dst_w) {
		osd->sc_i_wh_m1 = SCI_WH_M1_W(src_w - 1) |
					   SCI_WH_M1_H(src_h - 1);
		osd->sc_o_h_start_end = SCO_HV_START(dest.x1) |
						 SCO_HV_END(dest.x2 - 1);
		osd->sc_o_v_start_end = SCO_HV_START(dest.y1) |
						 SCO_HV_END(dest.y2 - 1);
		/* Enable OSD Scaler */
		osd->sc_ctrl0 = SC_CTRL0_PATH_EN | SC_CTRL0_SEL_OSD1;
	} else {
		osd->sc_i_wh_m1 = 0;
		osd->sc_o_h_start_end = 0;
		osd->sc_o_v_start_end = 0;
		osd->sc_ctrl0 = 0;
	}

	/* In interlaced mode, vertical scaler is always active */
	if (sc2_interlace || src_h != dst_h) {
		osd->sc_v_ctrl0 =
					VSC_BANK_LEN(vf_bank_len) |
					VSC_TOP_INI_RCV_NUM(vsc_ini_rcv_num) |
					VSC_TOP_RPT_L0_NUM(vsc_ini_rpt_p0_num) |
					VSC_VERTICAL_SCALER_EN;

		if (!meson_plane->index &&
		    (new_state->crtc->mode.flags & DRM_MODE_FLAG_INTERLACE))
			osd->sc_v_ctrl0 |=
					VSC_BOT_INI_RCV_NUM(vsc_bot_rcv_num) |
					VSC_BOT_RPT_L0_NUM(vsc_bot_rpt_p0_num) |
					VSC_PROG_INTERLACE;

		if (sc2_interlace)
			/* G12B+ vendor setting for progressive-to-field scaling. */
			osd->sc_v_ctrl0 |= BIT(25);

		osd->sc_v_phase_step = SC_PHASE_STEP(vf_phase_step);
		osd->sc_v_ini_phase = VSC_INI_PHASE_BOT(bot_ini_phase);
		if (sc2_interlace && src_h == dst_h * 2)
			osd->sc_v_ini_phase |=
				VSC_INI_PHASE_TOP(0x8000) |
				VSC_INI_PHASE_BOT(0x8000);
	} else {
		osd->sc_v_ctrl0 = 0;
		osd->sc_v_phase_step = 0;
		osd->sc_v_ini_phase = 0;
	}

	/* Horizontal scaler is only used if width does not match */
	if (src_w != dst_w) {
		osd->sc_h_ctrl0 =
					HSC_BANK_LENGTH(hf_bank_len) |
					HSC_INI_RCV_NUM0(hsc_ini_rcv_num) |
					HSC_RPT_P0_NUM0(hsc_ini_rpt_p0_num) |
					HSC_HORIZ_SCALER_EN;
		osd->sc_h_phase_step = SC_PHASE_STEP(hf_phase_step);
		osd->sc_h_ini_phase = 0;
	} else {
		osd->sc_h_ctrl0 = 0;
		osd->sc_h_phase_step = 0;
		osd->sc_h_ini_phase = 0;
	}

	/*
	 * The format of these registers is (x2 << 16 | x1),
	 * where x2 is exclusive.
	 * e.g. +30x1920 would be (1919 << 16) | 30
	 */
	osd->blk0_cfg[1] =
				((fixed16_to_int(new_state->src.x2) - 1) << 16) |
				fixed16_to_int(new_state->src.x1);
	osd->blk0_cfg[2] =
				((fixed16_to_int(new_state->src.y2) - 1) << 16) |
				fixed16_to_int(new_state->src.y1);
	osd->blk0_cfg[3] = ((dest.x2 - 1) << 16) | dest.x1;
	osd->blk0_cfg[4] = ((dest.y2 - 1) << 16) | dest.y1;

	if (priv->sc2) {
		/*
		 * OSD memory scanout and the OSD blender precede the shared
		 * scaler. They consume a progressive frame, not a 540-line
		 * field; only the scaler output and VPP use field coordinates.
		 */
		osd->blk0_cfg[3] = (src_w - 1) << 16;
		osd->blk0_cfg[4] = (src_h - 1) << 16;
	}

	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A)) {
		osd->blend_scope_h = ((dest.x2 - 1) << 16) | dest.x1;
		osd->blend_scope_v = ((dest.y2 - 1) << 16) | dest.y1;
		osd->blend0_size = dst_h << 16 | dst_w;
		osd->blend1_size = dst_h << 16 | dst_w;

		if (priv->sc2 && !meson_plane->index) {
			osd->blend_scope_h = (src_w - 1) << 16;
			osd->blend_scope_v = (src_h - 1) << 16;
			osd->blend0_size = src_h << 16 | src_w;
			osd->blend1_size = src_h << 16 | src_w;
		}
	}

	/* Update Canvas with buffer address */
	gem = drm_fb_dma_get_gem_obj(fb, 0);

	osd->addr = gem->dma_addr + fb->offsets[0];
	osd->stride = fb->pitches[0];
	osd->height = fb->height;
	osd->width = fb->width;
	osd->modifier = fb->modifier;
	osd->format = fb->format->format;

	if (osd->afbcd) {
		/* Calculate decoder write stride */
		if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A))
			osd->blk2_cfg4 =
				meson_g12a_afbcd_line_stride(osd);
	}

	if (!meson_plane->enabled) {
		/* Reset OSD1 before enabling it on GXL+ SoCs */
		if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXM) ||
		    meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXL))
			meson_viu_osd1_reset(priv);

		meson_plane->enabled = true;
	}

	osd->enabled = true;

	spin_unlock_irqrestore(&priv->drm->event_lock, flags);
}

static void meson_plane_atomic_disable(struct drm_plane *plane,
				       struct drm_atomic_state *state)
{
	struct meson_plane *meson_plane = to_meson_plane(plane);
	struct meson_drm *priv = meson_plane->priv;
	struct meson_osd_state *osd = &priv->viu.osd[meson_plane->index];
	unsigned long flags;

	spin_lock_irqsave(&priv->drm->event_lock, flags);
	meson_plane->enabled = false;
	osd->enabled = false;
	spin_unlock_irqrestore(&priv->drm->event_lock, flags);
}

static const struct drm_plane_helper_funcs meson_plane_helper_funcs = {
	.atomic_check	= meson_plane_atomic_check,
	.atomic_disable	= meson_plane_atomic_disable,
	.atomic_update	= meson_plane_atomic_update,
};

static bool meson_plane_format_mod_supported(struct drm_plane *plane,
					     u32 format, u64 modifier)
{
	struct meson_plane *meson_plane = to_meson_plane(plane);
	struct meson_drm *priv = meson_plane->priv;
	int i;

	if (modifier == DRM_FORMAT_MOD_INVALID)
		return false;

	if (modifier == DRM_FORMAT_MOD_LINEAR)
		return true;

	if (!meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXM) &&
	    !meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A))
		return false;

	if (modifier & ~DRM_FORMAT_MOD_ARM_AFBC(MESON_MOD_AFBC_VALID_BITS))
		return false;

	for (i = 0 ; i < plane->modifier_count ; ++i)
		if (plane->modifiers[i] == modifier)
			break;

	if (i == plane->modifier_count) {
		DRM_DEBUG_KMS("Unsupported modifier\n");
		return false;
	}

	if (priv->afbcd.ops && priv->afbcd.ops->supported_fmt)
		return priv->afbcd.ops->supported_fmt(modifier, format);

	DRM_DEBUG_KMS("AFBC Unsupported\n");
	return false;
}

static const struct drm_plane_funcs meson_plane_funcs = {
	.update_plane		= drm_atomic_helper_update_plane,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.destroy		= drm_plane_cleanup,
	.reset			= drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_plane_destroy_state,
	.format_mod_supported   = meson_plane_format_mod_supported,
};

static const uint32_t supported_drm_formats[] = {
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_RGB565,
};

static const uint64_t format_modifiers_afbc_gxm[] = {
	DRM_FORMAT_MOD_ARM_AFBC(AFBC_FORMAT_MOD_BLOCK_SIZE_16x16 |
				AFBC_FORMAT_MOD_SPARSE |
				AFBC_FORMAT_MOD_YTR),
	/* SPLIT mandates SPARSE, RGB modes mandates YTR */
	DRM_FORMAT_MOD_ARM_AFBC(AFBC_FORMAT_MOD_BLOCK_SIZE_16x16 |
				AFBC_FORMAT_MOD_YTR |
				AFBC_FORMAT_MOD_SPARSE |
				AFBC_FORMAT_MOD_SPLIT),
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID,
};

static const uint64_t format_modifiers_afbc_g12a[] = {
	/*
	 * - TOFIX Support AFBC modifiers for YUV formats (16x16 + TILED)
	 * - SPLIT is mandatory for performances reasons when in 16x16
	 *   block size
	 * - 32x8 block size + SPLIT is mandatory with 4K frame size
	 *   for performances reasons
	 */
	DRM_FORMAT_MOD_ARM_AFBC(AFBC_FORMAT_MOD_BLOCK_SIZE_16x16 |
				AFBC_FORMAT_MOD_SPARSE |
				AFBC_FORMAT_MOD_SPLIT),
	DRM_FORMAT_MOD_ARM_AFBC(AFBC_FORMAT_MOD_BLOCK_SIZE_16x16 |
				AFBC_FORMAT_MOD_YTR |
				AFBC_FORMAT_MOD_SPARSE |
				AFBC_FORMAT_MOD_SPLIT),
	DRM_FORMAT_MOD_ARM_AFBC(AFBC_FORMAT_MOD_BLOCK_SIZE_32x8 |
				AFBC_FORMAT_MOD_SPARSE),
	DRM_FORMAT_MOD_ARM_AFBC(AFBC_FORMAT_MOD_BLOCK_SIZE_32x8 |
				AFBC_FORMAT_MOD_YTR |
				AFBC_FORMAT_MOD_SPARSE),
	DRM_FORMAT_MOD_ARM_AFBC(AFBC_FORMAT_MOD_BLOCK_SIZE_32x8 |
				AFBC_FORMAT_MOD_SPARSE |
				AFBC_FORMAT_MOD_SPLIT),
	DRM_FORMAT_MOD_ARM_AFBC(AFBC_FORMAT_MOD_BLOCK_SIZE_32x8 |
				AFBC_FORMAT_MOD_YTR |
				AFBC_FORMAT_MOD_SPARSE |
				AFBC_FORMAT_MOD_SPLIT),
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID,
};

static const uint64_t format_modifiers_default[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID,
};

int meson_plane_create(struct meson_drm *priv)
{
	const uint64_t *modifiers = format_modifiers_default;
	unsigned int index;
	int ret;

	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXM))
		modifiers = format_modifiers_afbc_gxm;
	else if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A))
		modifiers = format_modifiers_afbc_g12a;

	for (index = 0; index < (priv->sc2 ? MESON_MAX_OSDS : 1); index++) {
		struct meson_plane *meson_plane;
		struct drm_plane *plane;

		meson_plane = devm_kzalloc(priv->dev, sizeof(*meson_plane), GFP_KERNEL);
		if (!meson_plane)
			return -ENOMEM;
		meson_plane->priv = priv;
		meson_plane->index = index;
		plane = &meson_plane->base;
		ret = drm_universal_plane_init(priv->drm, plane, BIT(0),
				&meson_plane_funcs, supported_drm_formats,
				ARRAY_SIZE(supported_drm_formats), modifiers,
				index ? DRM_PLANE_TYPE_OVERLAY : DRM_PLANE_TYPE_PRIMARY,
				index ? "meson_osd%u" : "meson_primary_plane", index + 1);
		if (ret)
			return ret;
		drm_plane_helper_add(plane, &meson_plane_helper_funcs);
		if (priv->sc2) {
			ret = drm_plane_create_alpha_property(plane);
			if (ret)
				return ret;
		}
		/* RGB inputs are premultiplied; VD1 < VD2 < OSD1 < OSD2 < OSD3. */
		ret = drm_plane_create_zpos_immutable_property(plane,
					priv->sc2 ? index + 2 : 1);
		if (ret)
			return ret;
		priv->osd_planes[index] = plane;
		if (!index)
			priv->primary_plane = plane;
	}
	return 0;
}
