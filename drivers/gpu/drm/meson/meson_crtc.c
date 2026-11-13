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
#include <linux/soc/amlogic/meson-canvas.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_device.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "meson_crtc.h"
#include "meson_plane.h"
#include "meson_registers.h"
#include "meson_venc.h"
#include "meson_viu.h"
#include "meson_rdma.h"
#include "meson_vpp.h"
#include "meson_osd_afbcd.h"

/* CRTC definition */

struct meson_crtc {
	struct drm_crtc base;
	struct drm_pending_vblank_event *event;
	struct meson_drm *priv;
	void (*enable_osd1)(struct meson_drm *priv);
	void (*enable_vd1)(struct meson_drm *priv);
	void (*enable_osd1_afbc)(struct meson_drm *priv);
	void (*disable_osd1_afbc)(struct meson_drm *priv);
	bool vsync_forced;
	bool osd1_afbcd;
	struct {
		bool interlace;
		u32 vsc_ini_phase[2];
		u32 vsc_phase_ctrl[2];
	} video[2];
};
#define to_meson_crtc(x) container_of(x, struct meson_crtc, base)

/* CRTC */

static int meson_crtc_enable_vblank(struct drm_crtc *crtc)
{
	struct meson_crtc *meson_crtc = to_meson_crtc(crtc);
	struct meson_drm *priv = meson_crtc->priv;

	meson_venc_enable_vsync(priv);

	return 0;
}

static void meson_crtc_disable_vblank(struct drm_crtc *crtc)
{
	struct meson_crtc *meson_crtc = to_meson_crtc(crtc);
	struct meson_drm *priv = meson_crtc->priv;

	if (!meson_crtc->vsync_forced)
		meson_venc_disable_vsync(priv);
}

static const struct drm_crtc_funcs meson_crtc_funcs = {
	.atomic_destroy_state	= drm_atomic_helper_crtc_destroy_state,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.destroy		= drm_crtc_cleanup,
	.page_flip		= drm_atomic_helper_page_flip,
	.reset			= drm_atomic_helper_crtc_reset,
	.set_config             = drm_atomic_helper_set_config,
	.enable_vblank		= meson_crtc_enable_vblank,
	.disable_vblank		= meson_crtc_disable_vblank,

};

static void meson_g12a_crtc_atomic_enable(struct drm_crtc *crtc,
					  struct drm_atomic_state *state)
{
	struct meson_crtc *meson_crtc = to_meson_crtc(crtc);
	struct drm_crtc_state *crtc_state = crtc->state;
	struct meson_drm *priv = meson_crtc->priv;
	unsigned int vdisplay;

	DRM_DEBUG_DRIVER("\n");

	if (!crtc_state) {
		DRM_ERROR("Invalid crtc_state\n");
		return;
	}

	vdisplay = crtc_state->mode.vdisplay;
	if (crtc_state->mode.flags & DRM_MODE_FLAG_INTERLACE)
		vdisplay /= 2;

	/* VD1 Preblend vertical start/end */
	writel(FIELD_PREP(GENMASK(11, 0), 2303),
	       priv->io_base + _REG(VPP_PREBLEND_VD1_V_START_END));

	/* Setup Blender */
	writel(crtc_state->mode.hdisplay |
	       vdisplay << 16,
	       priv->io_base + _REG(VPP_POSTBLEND_H_SIZE));

	writel_relaxed(0 << 16 |
			(crtc_state->mode.hdisplay - 1),
			priv->io_base + _REG(VPP_OSD1_BLD_H_SCOPE));
	writel_relaxed(0 << 16 |
			(vdisplay - 1),
			priv->io_base + _REG(VPP_OSD1_BLD_V_SCOPE));
	writel_relaxed(crtc_state->mode.hdisplay << 16 |
			vdisplay,
			priv->io_base + _REG(VPP_OUT_H_V_SIZE));

	drm_crtc_vblank_on(crtc);
}

static void meson_crtc_atomic_enable(struct drm_crtc *crtc,
				     struct drm_atomic_state *state)
{
	struct meson_crtc *meson_crtc = to_meson_crtc(crtc);
	struct drm_crtc_state *crtc_state = crtc->state;
	struct meson_drm *priv = meson_crtc->priv;

	DRM_DEBUG_DRIVER("\n");

	if (!crtc_state) {
		DRM_ERROR("Invalid crtc_state\n");
		return;
	}

	/* Enable VPP Postblend */
	writel(crtc_state->mode.hdisplay,
	       priv->io_base + _REG(VPP_POSTBLEND_H_SIZE));

	/* VD1 Preblend vertical start/end */
	writel(FIELD_PREP(GENMASK(11, 0), 2303),
			priv->io_base + _REG(VPP_PREBLEND_VD1_V_START_END));

	writel_bits_relaxed(VPP_POSTBLEND_ENABLE, VPP_POSTBLEND_ENABLE,
			    priv->io_base + _REG(VPP_MISC));

	drm_crtc_vblank_on(crtc);
}

static void meson_crtc_write_osd_scaler(struct meson_drm *priv,
				       unsigned int index,
				       const struct meson_osd_state *osd)
{
	u32 base = meson_osd_regs[index].sc_base;

	/* All three SC2 PPS banks have the same offsets 0..11. */
	writel_relaxed(osd->sc_v_phase_step, priv->io_base + _REG(base));
	writel_relaxed(osd->sc_v_ini_phase, priv->io_base + _REG(base + 1));
	writel_relaxed(osd->sc_v_ctrl0, priv->io_base + _REG(base + 2));
	writel_relaxed(osd->sc_h_phase_step, priv->io_base + _REG(base + 3));
	writel_relaxed(osd->sc_h_ini_phase, priv->io_base + _REG(base + 4));
	writel_relaxed(osd->sc_h_ctrl0, priv->io_base + _REG(base + 5));
	writel_relaxed(osd->sc_ctrl0, priv->io_base + _REG(base + 8));
	writel_relaxed(osd->sc_i_wh_m1, priv->io_base + _REG(base + 9));
	writel_relaxed(osd->sc_o_h_start_end, priv->io_base + _REG(base + 10));
	writel_relaxed(osd->sc_o_v_start_end, priv->io_base + _REG(base + 11));
}

static void meson_crtc_sc2_output_scaler(struct meson_drm *priv, bool enabled)
{
	const struct drm_display_mode *mode = &priv->crtc->state->mode;
	u32 base = meson_osd_regs[0].sc_base;
	bool interlace = enabled && (mode->flags & DRM_MODE_FLAG_INTERLACE);

	if (enabled && priv->viu.osd[0].enabled) {
		meson_crtc_write_osd_scaler(priv, 0, &priv->viu.osd[0]);
		return;
	}
	/* Without a primary FB, the composition domain is the full mode. */
	writel_relaxed(interlace ? 2 << 24 : 0, priv->io_base + _REG(base));
	writel_relaxed(interlace ? 0x80008000 : 0, priv->io_base + _REG(base + 1));
	writel_relaxed(interlace ? BIT(25) | BIT(24) | BIT(23) |
			(1 << 16) | (4 << 11) | (1 << 8) | (4 << 3) | 4 : 0,
			priv->io_base + _REG(base + 2));
	writel_relaxed(0, priv->io_base + _REG(base + 3));
	writel_relaxed(0, priv->io_base + _REG(base + 4));
	writel_relaxed(0, priv->io_base + _REG(base + 5));
	writel_relaxed(interlace ? BIT(3) | BIT(2) : 0,
			priv->io_base + _REG(base + 8));
	writel_relaxed(interlace ? (mode->hdisplay - 1) << 16 |
			(mode->vdisplay - 1) : 0, priv->io_base + _REG(base + 9));
	writel_relaxed(interlace ? mode->hdisplay - 1 : 0,
			priv->io_base + _REG(base + 10));
	writel_relaxed(interlace ? mode->vdisplay / 2 - 1 : 0,
			priv->io_base + _REG(base + 11));
}

static void meson_crtc_sc2_osd_blend(struct meson_drm *priv)
{
	const struct drm_display_mode *mode = &priv->crtc->state->mode;
	u32 size = priv->viu.osd[0].enabled ? priv->viu.osd[0].blend0_size :
		   mode->vdisplay << 16 | mode->hdisplay;
	u32 control = VIU_OSD_BLEND_HOLD_LINES(4);
	unsigned int count = 0, index, input = 0;
	bool bypass;

	for (index = 0; index < MESON_MAX_OSDS; index++)
		count += priv->viu.osd[index].enabled;
	bypass = count == 1 && priv->viu.osd[0].enabled;
	/* Unused inputs must remain transparent, including the shift workaround. */
	for (index = 0; index < 4; index++) {
		writel_relaxed(0, priv->io_base +
			       _REG(VIU_OSD_BLEND_DIN0_SCOPE_H + index * 2));
		writel_relaxed(0xffffffff, priv->io_base +
			       _REG(VIU_OSD_BLEND_DIN0_SCOPE_V + index * 2));
	}
	if (!bypass) {
		/* DIN0 is below DIN2, which is below DIN3. */
		control |= VIU_OSD_BLEND_BLEN2_PREMULT_EN(3) |
			   VIU_OSD_BLEND_DIN_EN(5) | 0x4444;
	} else {
		/* Preserve the deployed single-OSD bypass path. */
		control |= VIU_OSD_BLEND_BLEN2_PREMULT_EN(1) |
			   VIU_OSD_BLEND_DIN0_BYPASS_TO_DOUT0 |
			   VIU_OSD_BLEND1_DIN3_BYPASS_TO_DOUT1 |
			   VIU_OSD_BLEND1_DOUT_BYPASS_TO_BLEND2;
	}
	for (index = 0; index < MESON_MAX_OSDS; index++) {
		const struct meson_osd_state *osd = &priv->viu.osd[index];
		unsigned int din;

		if (!osd->enabled)
			continue;
		din = !input ? 0 : input == count - 1 ? 3 : 2;
		input++;
		/* Inactive DINs select OSD4 for the SC2 shift workaround. */
		control &= ~(0xf << (din * 4));
		control |= VIU_OSD_BLEND_REORDER(din, index + 1) |
			   VIU_OSD_BLEND_DIN_EN(BIT(din));
		/* BSP divides premultiplied pixels before blend0/1 multiplication. */
		writel_bits_relaxed(BIT(28), bypass ?
			priv->viu.osd1_bypass_alpha_div :
			(osd->format == DRM_FORMAT_ARGB8888 ||
			 osd->format == DRM_FORMAT_ABGR8888 ? BIT(28) : 0),
			priv->io_base + _REG(meson_osd_regs[index].unpack_ctrl));
		/* SC2 scopes precede reordering and are indexed by OSD, not DIN. */
		writel_relaxed(osd->blend_scope_h, priv->io_base +
			       _REG(VIU_OSD_BLEND_DIN0_SCOPE_H + index * 2));
		writel_relaxed(osd->blend_scope_v, priv->io_base +
			       _REG(VIU_OSD_BLEND_DIN0_SCOPE_V + index * 2));
	}
	writel_relaxed(size, priv->io_base + _REG(VIU_OSD_BLEND_BLEND0_SIZE));
	writel_relaxed(size, priv->io_base + _REG(VIU_OSD_BLEND_BLEND1_SIZE));
	writel_relaxed(control, priv->io_base + _REG(VIU_OSD_BLEND_CTRL));
	writel_relaxed(bypass ? priv->viu.osd1_bypass_blend_ctrl1 :
			BIT(0) | (3 << 4) | BIT(12) | (3 << 16),
			priv->io_base + _REG(VIU_OSD_BLEND_CTRL1));
	/* DOUT is divided to straight alpha; VPP must multiply it once. */
	writel_relaxed(OSD_BLEND_PATH_SEL_ENABLE |
			(count ? OSD_BLEND_POSTBLD_SRC_OSD1 : 0),
			priv->io_base + _REG(OSD1_BLEND_SRC_CTRL));
	meson_crtc_sc2_output_scaler(priv, count != 0);
}

static void meson_crtc_sc2_update_osds(struct meson_drm *priv)
{
	u32 surface_mask = 0;
	unsigned int index, word;
	bool commit = false;

	for (index = 0; index < MESON_MAX_OSDS; index++) {
		const struct meson_osd_state *osd = &priv->viu.osd[index];

		commit |= osd->commit;
		if (osd->enabled && osd->afbcd)
			surface_mask |= BIT(index);
	}
	if (!commit)
		return;

	/* One replay table and one decoder command own the entire OSD commit. */
	meson_rdma_reset(priv);
	for (index = 0; index < MESON_MAX_OSDS; index++) {
		struct meson_osd_state *osd = &priv->viu.osd[index];
		const struct meson_osd_regs *regs = &meson_osd_regs[index];

		osd->commit = false;
		if (!osd->enabled) {
			writel_bits_relaxed(OSD_ENABLE | OSD_BLK0_ENABLE, 0,
					    priv->io_base + _REG(regs->ctrl_stat));
			meson_viu_osd_afbc_select(priv, index, false);
			if (index)
				writel_relaxed(0, priv->io_base + _REG(regs->sc_base + 8));
			continue;
		}
		writel_relaxed(osd->ctrl_stat | (osd->afbcd ? OSD_MEM_LINEAR_ADDR : 0),
			       priv->io_base + _REG(regs->ctrl_stat));
		writel_relaxed(osd->ctrl_stat2, priv->io_base + _REG(regs->ctrl_stat2));
		for (word = 0; word < ARRAY_SIZE(regs->blk0_cfg); word++)
			writel_relaxed(osd->blk0_cfg[word],
				       priv->io_base + _REG(regs->blk0_cfg[word]));
		if (osd->afbcd) {
			writel_relaxed(osd->blk1_cfg4, priv->io_base + _REG(regs->blk1_cfg4));
			writel_relaxed(osd->blk2_cfg4, priv->io_base + _REG(regs->blk2_cfg4));
		} else {
			meson_canvas_config(priv->canvas, priv->canvas_id_osd[index],
				osd->addr, osd->stride, osd->height,
				MESON_CANVAS_WRAP_NONE, MESON_CANVAS_BLKMODE_LINEAR, 0);
		}
		meson_viu_osd_afbc_select(priv, index, osd->afbcd);
		if (index)
			meson_crtc_write_osd_scaler(priv, index, osd);
	}
	meson_crtc_sc2_osd_blend(priv);
	meson_afbcd_sc2_update(priv, surface_mask);
}

static void meson_crtc_disable_osd1(struct meson_crtc *meson_crtc)
{
	struct meson_drm *priv = meson_crtc->priv;

	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A))
		writel_bits_relaxed(VIU_OSD1_POSTBLD_SRC_OSD1, 0,
				    priv->io_base + _REG(OSD1_BLEND_SRC_CTRL));
	else
		writel_bits_relaxed(VPP_OSD1_POSTBLEND, 0,
				    priv->io_base + _REG(VPP_MISC));

	writel_bits_relaxed(OSD_ENABLE | OSD_BLK0_ENABLE, 0,
			    priv->io_base + _REG(VIU_OSD1_CTRL_STAT));

	if (meson_crtc->osd1_afbcd) {
		if (meson_crtc->disable_osd1_afbc)
			meson_crtc->disable_osd1_afbc(priv);
		priv->afbcd.ops->reset(priv);
		priv->afbcd.ops->disable(priv);
		meson_crtc->osd1_afbcd = false;
	}
}

static void meson_crtc_disable_video(struct meson_crtc *meson_crtc,
				     unsigned int index)
{
	struct meson_drm *priv = meson_crtc->priv;

	if (priv->sc2) {
		/* video_sink disable_vd2_blend: stop only this layer's fetchers. */
		writel_relaxed(0, priv->io_base +
			       _REG(index ? VD2_BLEND_SRC_CTRL : VD1_BLEND_SRC_CTRL));
		writel_relaxed(0, priv->io_base +
			       _REG(meson_viu_afbc_reg(priv, index, AFBC_ENABLE)));
		writel_relaxed(0, priv->io_base +
			       _REG(meson_viu_mif_reg(priv, index ?
						  VD2_IF0_GEN_REG : VD1_IF0_GEN_REG)));
		meson_viu_video_afbc_select(priv, index, false);
	} else if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A)) {
		writel_relaxed(0, priv->io_base + _REG(VD1_BLEND_SRC_CTRL));
		writel_relaxed(0, priv->io_base + _REG(VD2_BLEND_SRC_CTRL));
		writel_relaxed(0, priv->io_base +
			       _REG(meson_viu_mif_reg(priv, VD1_IF0_GEN_REG)));
		writel_relaxed(0, priv->io_base +
			       _REG(meson_viu_mif_reg(priv, VD2_IF0_GEN_REG)));
	} else {
		writel_bits_relaxed(VPP_VD1_POSTBLEND | VPP_VD1_PREBLEND, 0,
				    priv->io_base + _REG(VPP_MISC));
	}

	meson_crtc->video[index].interlace = false;
}

static void meson_crtc_atomic_disable(struct drm_crtc *crtc,
				      struct drm_atomic_state *state)
{
	struct meson_crtc *meson_crtc = to_meson_crtc(crtc);
	struct meson_drm *priv = meson_crtc->priv;
	unsigned int i;

	DRM_DEBUG_DRIVER("\n");

	drm_crtc_vblank_off(crtc);

	spin_lock_irq(&crtc->dev->event_lock);
	if (priv->sc2) {
		for (i = 0; i < MESON_MAX_OSDS; i++) {
			priv->viu.osd[i].enabled = false;
			priv->viu.osd[i].commit = true;
		}
		meson_crtc_sc2_update_osds(priv);
	} else {
		priv->viu.osd[0].enabled = false;
		priv->viu.osd[0].commit = false;
		meson_crtc_disable_osd1(meson_crtc);
	}
	for (i = 0; i < (priv->sc2 ? 2 : 1); i++) {
		priv->viu.video[i].enabled = false;
		priv->viu.video[i].commit = false;
		meson_crtc_disable_video(meson_crtc, i);
	}
	meson_crtc->vsync_forced = false;
	meson_venc_disable_vsync(priv);

	if (!meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A))
		writel_bits_relaxed(VPP_POSTBLEND_ENABLE, 0,
				    priv->io_base + _REG(VPP_MISC));

	if (crtc->state->event && !crtc->state->active) {
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
	}
	spin_unlock_irq(&crtc->dev->event_lock);
}

static void meson_crtc_atomic_flush(struct drm_crtc *crtc,
				    struct drm_atomic_state *state)
{
	struct meson_crtc *meson_crtc = to_meson_crtc(crtc);
	struct meson_drm *priv = meson_crtc->priv;
	struct drm_crtc_state *crtc_state =
		drm_atomic_get_new_crtc_state(state, crtc);
	unsigned long flags;
	unsigned int i;

	if (crtc_state->event)
		WARN_ON(drm_crtc_vblank_get(crtc) != 0);

	/*
	 * Publish the completed shadows and their event together. The atomic
	 * helpers wait for the previous flip before either plane can update its
	 * shadow, so the IRQ must not signal this flip before atomic_flush.
	 */
	spin_lock_irqsave(&crtc->dev->event_lock, flags);
	for (i = 0; i < (priv->sc2 ? MESON_MAX_OSDS : 1); i++)
		if (drm_atomic_get_new_plane_state(state, priv->osd_planes[i]))
			priv->viu.osd[i].commit = true;
	for (i = 0; i < (priv->sc2 ? 2 : 1); i++)
		if (drm_atomic_get_new_plane_state(state, priv->overlay_planes[i]))
			priv->viu.video[i].commit = true;
	if (crtc_state->event) {
		meson_crtc->event = crtc_state->event;
		crtc_state->event = NULL;
	}
	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
}

static const struct drm_crtc_helper_funcs meson_crtc_helper_funcs = {
	.atomic_flush	= meson_crtc_atomic_flush,
	.atomic_enable	= meson_crtc_atomic_enable,
	.atomic_disable	= meson_crtc_atomic_disable,
};

static const struct drm_crtc_helper_funcs meson_g12a_crtc_helper_funcs = {
	.atomic_flush	= meson_crtc_atomic_flush,
	.atomic_enable	= meson_g12a_crtc_atomic_enable,
	.atomic_disable	= meson_crtc_atomic_disable,
};

static void meson_crtc_enable_osd1(struct meson_drm *priv)
{
	writel_bits_relaxed(VPP_OSD1_POSTBLEND, VPP_OSD1_POSTBLEND,
			    priv->io_base + _REG(VPP_MISC));
}

static void meson_crtc_g12a_enable_osd1_afbc(struct meson_drm *priv)
{
	writel_relaxed(priv->viu.osd[0].blk2_cfg4,
		       priv->io_base + _REG(VIU_OSD1_BLK2_CFG_W4));

	writel_bits_relaxed(OSD_MEM_LINEAR_ADDR, OSD_MEM_LINEAR_ADDR,
			    priv->io_base + _REG(VIU_OSD1_CTRL_STAT));

	writel_relaxed(priv->viu.osd[0].blk1_cfg4,
		       priv->io_base + _REG(VIU_OSD1_BLK1_CFG_W4));

	meson_viu_g12a_enable_osd1_afbc(priv);

	writel_bits_relaxed(OSD_MEM_LINEAR_ADDR, OSD_MEM_LINEAR_ADDR,
			    priv->io_base + _REG(VIU_OSD1_CTRL_STAT));

	writel_bits_relaxed(OSD_MALI_SRC_EN, OSD_MALI_SRC_EN,
			    priv->io_base + _REG(VIU_OSD1_BLK0_CFG_W0));
}

static void meson_g12a_crtc_enable_osd1(struct meson_drm *priv)
{
	writel_relaxed(priv->viu.osd[0].blend_scope_h,
		       priv->io_base +
		       _REG(VIU_OSD_BLEND_DIN0_SCOPE_H));
	writel_relaxed(priv->viu.osd[0].blend_scope_v,
		       priv->io_base +
		       _REG(VIU_OSD_BLEND_DIN0_SCOPE_V));
	writel_relaxed(priv->viu.osd[0].blend0_size,
		       priv->io_base +
		       _REG(VIU_OSD_BLEND_BLEND0_SIZE));
	writel_relaxed(priv->viu.osd[0].blend1_size,
		       priv->io_base +
		       _REG(VIU_OSD_BLEND_BLEND1_SIZE));
	writel_bits_relaxed(3 << 8, 3 << 8,
			    priv->io_base + _REG(OSD1_BLEND_SRC_CTRL));
}

static void meson_crtc_enable_vd1(struct meson_drm *priv)
{
	writel_bits_relaxed(VPP_VD1_PREBLEND | VPP_VD1_POSTBLEND |
			    VPP_COLOR_MNG_ENABLE,
			    VPP_VD1_PREBLEND | VPP_VD1_POSTBLEND |
			    VPP_COLOR_MNG_ENABLE,
			    priv->io_base + _REG(VPP_MISC));

	writel_bits_relaxed(VIU_CTRL0_AFBC_TO_VD1,
			    priv->viu.video[0].afbc ? VIU_CTRL0_AFBC_TO_VD1 : 0,
			    priv->io_base + _REG(VIU_MISC_CTRL0));
}

static void meson_g12a_crtc_enable_vd1(struct meson_drm *priv)
{
	writel_relaxed(VD_BLEND_PREBLD_SRC_VD1 |
		       VD_BLEND_PREBLD_PREMULT_EN |
		       VD_BLEND_POSTBLD_SRC_VD1 |
		       VD_BLEND_POSTBLD_PREMULT_EN,
		       priv->io_base + _REG(VD1_BLEND_SRC_CTRL));

	meson_viu_video_afbc_select(priv, 0, priv->viu.video[0].afbc);
}

static void meson_crtc_write_video_mif(struct meson_drm *priv,
				       const struct meson_video_state *vd,
				       unsigned int index)
{
	/* Legacy VD1/VD2 field addresses precede the SoC-specific translation. */
	unsigned int offset = index * (VD2_IF0_GEN_REG - VD1_IF0_GEN_REG);

	writel_relaxed(vd->if0_gen_reg, priv->io_base +
		       _REG(meson_viu_mif_reg(priv, VD1_IF0_GEN_REG + offset)));
	if (!index || priv->sc2)
		writel_relaxed(vd->if0_gen_reg2, priv->io_base +
			       _REG(meson_viu_mif_reg(priv, VD1_IF0_GEN_REG2 + offset)));
	writel_relaxed(vd->fmt_ctrl, priv->io_base +
		       _REG(meson_viu_mif_reg(priv, VIU_VD1_FMT_CTRL + offset)));
	writel_relaxed(vd->fmt_w, priv->io_base +
		       _REG(meson_viu_mif_reg(priv, VIU_VD1_FMT_W + offset)));
	writel_relaxed(vd->if0_canvas0, priv->io_base +
		       _REG(meson_viu_mif_reg(priv, VD1_IF0_CANVAS0 + offset)));
	writel_relaxed(vd->if0_canvas0, priv->io_base +
		       _REG(meson_viu_mif_reg(priv, VD1_IF0_CANVAS1 + offset)));
	writel_relaxed(vd->if0_luma_x0, priv->io_base +
		       _REG(meson_viu_mif_reg(priv, VD1_IF0_LUMA_X0 + offset)));
	writel_relaxed(vd->if0_luma_x0, priv->io_base +
		       _REG(meson_viu_mif_reg(priv, VD1_IF0_LUMA_X1 + offset)));
	writel_relaxed(vd->if0_luma_y0, priv->io_base +
		       _REG(meson_viu_mif_reg(priv, VD1_IF0_LUMA_Y0 + offset)));
	writel_relaxed(vd->if0_luma_y0, priv->io_base +
		       _REG(meson_viu_mif_reg(priv, VD1_IF0_LUMA_Y1 + offset)));
	writel_relaxed(vd->if0_chroma_x0, priv->io_base +
		       _REG(meson_viu_mif_reg(priv, VD1_IF0_CHROMA_X0 + offset)));
	writel_relaxed(vd->if0_chroma_x0, priv->io_base +
		       _REG(meson_viu_mif_reg(priv, VD1_IF0_CHROMA_X1 + offset)));
	writel_relaxed(vd->if0_chroma_y0, priv->io_base +
		       _REG(meson_viu_mif_reg(priv, VD1_IF0_CHROMA_Y0 + offset)));
	writel_relaxed(vd->if0_chroma_y0, priv->io_base +
		       _REG(meson_viu_mif_reg(priv, VD1_IF0_CHROMA_Y1 + offset)));
	writel_relaxed(vd->if0_repeat_loop, priv->io_base +
		       _REG(meson_viu_mif_reg(priv, VD1_IF0_RPT_LOOP + offset)));
	writel_relaxed(vd->if0_luma0_rpt_pat, priv->io_base +
		       _REG(meson_viu_mif_reg(priv, VD1_IF0_LUMA0_RPT_PAT + offset)));
	writel_relaxed(vd->if0_luma0_rpt_pat, priv->io_base +
		       _REG(meson_viu_mif_reg(priv, VD1_IF0_LUMA1_RPT_PAT + offset)));
	writel_relaxed(vd->if0_chroma0_rpt_pat, priv->io_base +
		       _REG(meson_viu_mif_reg(priv, VD1_IF0_CHROMA0_RPT_PAT + offset)));
	writel_relaxed(vd->if0_chroma0_rpt_pat, priv->io_base +
		       _REG(meson_viu_mif_reg(priv, VD1_IF0_CHROMA1_RPT_PAT + offset)));
	writel_relaxed(0, priv->io_base +
		       _REG(meson_viu_mif_reg(priv, VD1_IF0_LUMA_PSEL + offset)));
	writel_relaxed(0, priv->io_base +
		       _REG(meson_viu_mif_reg(priv, VD1_IF0_CHROMA_PSEL + offset)));
	if (!index || priv->sc2)
		writel_relaxed(vd->range_map_y, priv->io_base +
			       _REG(meson_viu_mif_reg(priv, VD1_IF0_RANGE_MAP_Y + offset)));
	if (!index || priv->sc2)
		writel_relaxed(vd->range_map_cb, priv->io_base +
			       _REG(meson_viu_mif_reg(priv, VD1_IF0_RANGE_MAP_CB + offset)));
	if (!index || priv->sc2)
		writel_relaxed(vd->range_map_cr, priv->io_base +
			       _REG(meson_viu_mif_reg(priv, VD1_IF0_RANGE_MAP_CR + offset)));
}

static void meson_crtc_update_video(struct meson_crtc *meson_crtc,
				    unsigned int index)
{
	struct meson_drm *priv = meson_crtc->priv;
	struct meson_video_state *vd = &priv->viu.video[index];
	const struct meson_vpp_video_regs *pps = &meson_vpp_video_regs[index];

	if (!vd->commit)
		return;
	if (!vd->enabled) {
		meson_crtc_disable_video(meson_crtc, index);
		vd->commit = false;
		return;
	}

	if (vd->afbc) {
		writel_relaxed(vd->afbc_head_addr,
			       priv->io_base +
			       _REG(meson_viu_afbc_reg(priv, index, AFBC_HEAD_BADDR)));
		writel_relaxed(vd->afbc_body_addr,
			       priv->io_base +
			       _REG(meson_viu_afbc_reg(priv, index, AFBC_BODY_BADDR)));
		writel_relaxed(vd->afbc_en,
			       priv->io_base +
			       _REG(meson_viu_afbc_reg(priv, index, AFBC_ENABLE)));
		writel_relaxed(vd->afbc_mode,
			       priv->io_base +
			       _REG(meson_viu_afbc_reg(priv, index, AFBC_MODE)));
		writel_relaxed(vd->afbc_size_in,
			       priv->io_base +
			       _REG(meson_viu_afbc_reg(priv, index, AFBC_SIZE_IN)));
		writel_relaxed(vd->afbc_dec_def_color,
			       priv->io_base +
			       _REG(meson_viu_afbc_reg(priv, index, AFBC_DEC_DEF_COLOR)));
		writel_relaxed(vd->afbc_conv_ctrl,
			       priv->io_base +
			       _REG(meson_viu_afbc_reg(priv, index, AFBC_CONV_CTRL)));
		writel_relaxed(vd->afbc_size_out,
			       priv->io_base +
			       _REG(meson_viu_afbc_reg(priv, index, AFBC_SIZE_OUT)));
		writel_relaxed(vd->afbc_vd_cfmt_ctrl,
			       priv->io_base +
			       _REG(meson_viu_afbc_reg(priv, index, AFBC_VD_CFMT_CTRL)));
		writel_relaxed(vd->afbc_vd_cfmt_w,
			       priv->io_base +
			       _REG(meson_viu_afbc_reg(priv, index, AFBC_VD_CFMT_W)));
		writel_relaxed(vd->afbc_mif_hor_scope,
			       priv->io_base +
			       _REG(meson_viu_afbc_reg(priv, index, AFBC_MIF_HOR_SCOPE)));
		writel_relaxed(vd->afbc_mif_ver_scope,
			       priv->io_base +
			       _REG(meson_viu_afbc_reg(priv, index, AFBC_MIF_VER_SCOPE)));
		writel_relaxed(vd->afbc_pixel_hor_scope,
			       priv->io_base+
			       _REG(meson_viu_afbc_reg(priv, index, AFBC_PIXEL_HOR_SCOPE)));
		writel_relaxed(vd->afbc_pixel_ver_scope,
			       priv->io_base +
			       _REG(meson_viu_afbc_reg(priv, index, AFBC_PIXEL_VER_SCOPE)));
		writel_relaxed(vd->afbc_vd_cfmt_h,
			       priv->io_base +
			       _REG(meson_viu_afbc_reg(priv, index, AFBC_VD_CFMT_H)));
	} else {
		switch (vd->planes) {
		case 3:
			meson_canvas_config(priv->canvas,
					    priv->canvas_id_vd[index][2],
					    vd->addr2,
					    vd->stride2,
					    vd->height2,
					    MESON_CANVAS_WRAP_NONE,
					    MESON_CANVAS_BLKMODE_LINEAR,
					    MESON_CANVAS_ENDIAN_SWAP64);
			fallthrough;
		case 2:
			meson_canvas_config(priv->canvas,
					    priv->canvas_id_vd[index][1],
					    vd->addr1,
					    vd->stride1,
					    vd->height1,
					    MESON_CANVAS_WRAP_NONE,
					    MESON_CANVAS_BLKMODE_LINEAR,
					    MESON_CANVAS_ENDIAN_SWAP64);
			fallthrough;
		case 1:
			meson_canvas_config(priv->canvas,
					    priv->canvas_id_vd[index][0],
					    vd->addr0,
					    vd->stride0,
					    vd->height0,
					    MESON_CANVAS_WRAP_NONE,
					    MESON_CANVAS_BLKMODE_LINEAR,
					    MESON_CANVAS_ENDIAN_SWAP64);
		}

		writel_relaxed(0, priv->io_base + _REG(meson_viu_afbc_reg(priv, index, AFBC_ENABLE)));
	}

	meson_crtc_write_video_mif(priv, vd, index);
	/* Older SoCs use the second MIF for VD1's two-picture path. */
	if (!priv->sc2)
		meson_crtc_write_video_mif(priv, vd, 1);

	writel_relaxed(VPP_VSC_BANK_LENGTH(4) | VPP_HSC_BANK_LENGTH(4) |
		       VPP_SC_VD_EN_ENABLE | VPP_SC_TOP_EN_ENABLE |
		       VPP_SC_HSC_EN_ENABLE | VPP_SC_VSC_EN_ENABLE,
		       priv->io_base + _REG(pps->sc_misc));
	if (!index) {
		writel_relaxed(vd->vpp_pic_in_height,
			       priv->io_base + _REG(VPP_PIC_IN_HEIGHT));
		writel_relaxed(vd->vpp_line_in_length,
			       priv->io_base + _REG(VPP_LINE_IN_LENGTH));
		writel_relaxed(vd->vpp_preblend_h_size,
			       priv->io_base + _REG(VPP_PREBLEND_H_SIZE));
	} else {
		/* SC2 vdx_scaler_setting owns this packed VD2 PPS input size. */
		writel_relaxed((vd->vpp_pic_in_height << 16) |
			       vd->vpp_line_in_length,
			       priv->io_base + _REG(VPP_VD2_HDR_IN_SIZE));
	}
	writel_relaxed(vd->postblend_h_start_end,
		       priv->io_base + _REG(pps->postblend_h_start_end));
	writel_relaxed(vd->postblend_v_start_end,
		       priv->io_base + _REG(pps->postblend_v_start_end));
	if (!priv->sc2) {
		writel_relaxed(vd->vpp_blend_vd2_h_start_end,
			       priv->io_base + _REG(VPP_BLEND_VD2_H_START_END));
		writel_relaxed(vd->vpp_blend_vd2_v_start_end,
			       priv->io_base + _REG(VPP_BLEND_VD2_V_START_END));
	}
	writel_relaxed(vd->vpp_vsc_region12_startp,
		       priv->io_base + _REG(pps->vsc_region12_startp));
	writel_relaxed(vd->vpp_vsc_region34_startp,
		       priv->io_base + _REG(pps->vsc_region34_startp));
	writel_relaxed(vd->vpp_vsc_region4_endp,
		       priv->io_base + _REG(pps->vsc_region4_endp));
	writel_relaxed(vd->vpp_vsc_start_phase_step,
		       priv->io_base + _REG(pps->vsc_start_phase_step));
	writel_relaxed(vd->vpp_hsc_region12_startp,
		       priv->io_base + _REG(pps->hsc_region12_startp));
	writel_relaxed(vd->vpp_hsc_region34_startp,
		       priv->io_base + _REG(pps->hsc_region34_startp));
	writel_relaxed(vd->vpp_hsc_region4_endp,
		       priv->io_base + _REG(pps->hsc_region4_endp));
	writel_relaxed(vd->vpp_hsc_start_phase_step,
		       priv->io_base + _REG(pps->hsc_start_phase_step));
	writel_relaxed(vd->vpp_hsc_region1_phase_slope,
		       priv->io_base + _REG(pps->hsc_region1_phase_slope));
	writel_relaxed(vd->vpp_hsc_region3_phase_slope,
		       priv->io_base + _REG(pps->hsc_region3_phase_slope));
	writel_relaxed(vd->vpp_hsc_phase_ctrl,
		       priv->io_base + _REG(pps->hsc_phase_ctrl));
	meson_crtc->video[index].interlace = vd->interlace;
	meson_crtc->video[index].vsc_ini_phase[0] = vd->vpp_vsc_ini_phase;
	meson_crtc->video[index].vsc_ini_phase[1] = vd->vpp_vsc_ini_phase_bottom;
	meson_crtc->video[index].vsc_phase_ctrl[0] = vd->vpp_vsc_phase_ctrl;
	meson_crtc->video[index].vsc_phase_ctrl[1] = vd->vpp_vsc_phase_ctrl_bottom;
	if (!vd->interlace) {
		writel_relaxed(vd->vpp_vsc_ini_phase,
			       priv->io_base + _REG(pps->vsc_ini_phase));
		writel_relaxed(vd->vpp_vsc_phase_ctrl,
			       priv->io_base + _REG(pps->vsc_phase_ctrl));
	}
	if (priv->sc2)
		writel_relaxed(vd->vpp_hsc_phase_ctrl,
			       priv->io_base + _REG(pps->hsc_phase_ctrl1));
	writel_relaxed(0x42, priv->io_base + _REG(pps->scale_coef_idx));

	if (index) {
		/* SC2 has fixed postblend port 1, behind OSD1, with opaque alpha. */
		writel_bits_relaxed(GENMASK(26, 18), 0x100 << 18,
				    priv->io_base + _REG(VPP_MISC));
		meson_viu_video_afbc_select(priv, index, vd->afbc);
		writel_relaxed(VD_BLEND_POSTBLD_SRC_VD2 | VD_BLEND_PATH_SEL_ENABLE,
			       priv->io_base + _REG(VD2_BLEND_SRC_CTRL));
	} else if (meson_crtc->enable_vd1) {
		meson_crtc->enable_vd1(priv);
	}
	vd->commit = false;
}

void meson_crtc_irq(struct meson_drm *priv)
{
	struct meson_crtc *meson_crtc = to_meson_crtc(priv->crtc);
	unsigned long flags;
	unsigned int i;

	drm_crtc_handle_vblank(priv->crtc);

	spin_lock_irqsave(&priv->drm->event_lock, flags);
	if (priv->sc2)
		meson_crtc_sc2_update_osds(priv);
	if (!priv->sc2 && priv->viu.osd[0].commit && !priv->viu.osd[0].enabled) {
		meson_crtc_disable_osd1(meson_crtc);
		priv->viu.osd[0].commit = false;
	}

	/* Update the OSD registers */
	if (!priv->sc2 && priv->viu.osd[0].enabled && priv->viu.osd[0].commit) {
		writel_relaxed(priv->viu.osd[0].ctrl_stat,
				priv->io_base + _REG(VIU_OSD1_CTRL_STAT));
		writel_relaxed(priv->viu.osd[0].ctrl_stat2,
				priv->io_base + _REG(VIU_OSD1_CTRL_STAT2));
		writel_relaxed(priv->viu.osd[0].blk0_cfg[0],
				priv->io_base + _REG(VIU_OSD1_BLK0_CFG_W0));
		writel_relaxed(priv->viu.osd[0].blk0_cfg[1],
				priv->io_base + _REG(VIU_OSD1_BLK0_CFG_W1));
		writel_relaxed(priv->viu.osd[0].blk0_cfg[2],
				priv->io_base + _REG(VIU_OSD1_BLK0_CFG_W2));
		writel_relaxed(priv->viu.osd[0].blk0_cfg[3],
				priv->io_base + _REG(VIU_OSD1_BLK0_CFG_W3));
		writel_relaxed(priv->viu.osd[0].blk0_cfg[4],
				priv->io_base + _REG(VIU_OSD1_BLK0_CFG_W4));

		if (priv->viu.osd[0].afbcd) {
			if (meson_crtc->enable_osd1_afbc)
				meson_crtc->enable_osd1_afbc(priv);
		} else if (meson_crtc->osd1_afbcd) {
			if (meson_crtc->disable_osd1_afbc)
				meson_crtc->disable_osd1_afbc(priv);
			priv->afbcd.ops->reset(priv);
			priv->afbcd.ops->disable(priv);
		}

		writel_relaxed(priv->viu.osd[0].sc_ctrl0,
				priv->io_base + _REG(VPP_OSD_SC_CTRL0));
		writel_relaxed(priv->viu.osd[0].sc_i_wh_m1,
				priv->io_base + _REG(VPP_OSD_SCI_WH_M1));
		writel_relaxed(priv->viu.osd[0].sc_o_h_start_end,
				priv->io_base + _REG(VPP_OSD_SCO_H_START_END));
		writel_relaxed(priv->viu.osd[0].sc_o_v_start_end,
				priv->io_base + _REG(VPP_OSD_SCO_V_START_END));
		writel_relaxed(priv->viu.osd[0].sc_v_ini_phase,
				priv->io_base + _REG(VPP_OSD_VSC_INI_PHASE));
		writel_relaxed(priv->viu.osd[0].sc_v_phase_step,
				priv->io_base + _REG(VPP_OSD_VSC_PHASE_STEP));
		writel_relaxed(priv->viu.osd[0].sc_h_ini_phase,
				priv->io_base + _REG(VPP_OSD_HSC_INI_PHASE));
		writel_relaxed(priv->viu.osd[0].sc_h_phase_step,
				priv->io_base + _REG(VPP_OSD_HSC_PHASE_STEP));
		writel_relaxed(priv->viu.osd[0].sc_h_ctrl0,
				priv->io_base + _REG(VPP_OSD_HSC_CTRL0));
		writel_relaxed(priv->viu.osd[0].sc_v_ctrl0,
				priv->io_base + _REG(VPP_OSD_VSC_CTRL0));

		if (!priv->viu.osd[0].afbcd)
			meson_canvas_config(priv->canvas, priv->canvas_id_osd[0],
					    priv->viu.osd[0].addr,
					    priv->viu.osd[0].stride,
					    priv->viu.osd[0].height,
					    MESON_CANVAS_WRAP_NONE,
					    MESON_CANVAS_BLKMODE_LINEAR, 0);

		/* Enable OSD1 */
		if (meson_crtc->enable_osd1)
			meson_crtc->enable_osd1(priv);

		if (priv->viu.osd[0].afbcd) {
			/* SC2 does not need the G12A per-frame reset workaround. */
			if (!priv->sc2 || !meson_crtc->osd1_afbcd)
				priv->afbcd.ops->reset(priv);
			priv->afbcd.ops->setup(priv);
			priv->afbcd.ops->enable(priv);
		}

		meson_crtc->osd1_afbcd = priv->viu.osd[0].afbcd;

		priv->viu.osd[0].commit = false;
	}

	for (i = 0; i < (priv->sc2 ? 2 : 1); i++)
		meson_crtc_update_video(meson_crtc, i);

	/* Use only applied phases: the plane shadows may hold an unflushed flip. */
	for (i = 0; i < (priv->sc2 ? 2 : 1); i++) {
		const struct meson_vpp_video_regs *pps = &meson_vpp_video_regs[i];
		bool bottom;

		if (!meson_crtc->video[i].interlace)
			continue;
		if (priv->venc.current_mode == MESON_VENC_MODE_HDMI &&
		    !priv->venc.hdmi_use_enci)
			bottom = FIELD_GET(GENMASK(28, 16),
				readl_relaxed(priv->io_base + _REG(ENCP_INFO_READ))) >= 562;
		else
			bottom = meson_venci_get_field(priv);

		/* CPU writes for the current field, not next-vsync RDMA. */
		writel_relaxed(meson_crtc->video[i].vsc_ini_phase[bottom],
			       priv->io_base + _REG(pps->vsc_ini_phase));
		writel_relaxed(meson_crtc->video[i].vsc_phase_ctrl[bottom],
			       priv->io_base + _REG(pps->vsc_phase_ctrl));
	}

	meson_crtc->vsync_forced = (priv->sc2 ? priv->afbcd.active_mask != 0 :
		meson_crtc->osd1_afbcd) ||
		meson_crtc->video[0].interlace || meson_crtc->video[1].interlace;
	if (meson_crtc->event) {
		drm_crtc_send_vblank_event(priv->crtc, meson_crtc->event);
		drm_crtc_vblank_put(priv->crtc);
		meson_crtc->event = NULL;
	}
	spin_unlock_irqrestore(&priv->drm->event_lock, flags);
}

int meson_crtc_create(struct meson_drm *priv)
{
	struct meson_crtc *meson_crtc;
	struct drm_crtc *crtc;
	int ret;

	meson_crtc = devm_kzalloc(priv->drm->dev, sizeof(*meson_crtc),
				  GFP_KERNEL);
	if (!meson_crtc)
		return -ENOMEM;

	meson_crtc->priv = priv;
	crtc = &meson_crtc->base;
	ret = drm_crtc_init_with_planes(priv->drm, crtc,
					priv->primary_plane, NULL,
					&meson_crtc_funcs, "meson_crtc");
	if (ret) {
		dev_err(priv->drm->dev, "Failed to init CRTC\n");
		return ret;
	}

	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A)) {
		meson_crtc->enable_osd1 = meson_g12a_crtc_enable_osd1;
		meson_crtc->enable_vd1 = meson_g12a_crtc_enable_vd1;
		meson_crtc->enable_osd1_afbc =
					meson_crtc_g12a_enable_osd1_afbc;
		meson_crtc->disable_osd1_afbc =
					meson_viu_g12a_disable_osd1_afbc;
		drm_crtc_helper_add(crtc, &meson_g12a_crtc_helper_funcs);
	} else {
		meson_crtc->enable_osd1 = meson_crtc_enable_osd1;
		meson_crtc->enable_vd1 = meson_crtc_enable_vd1;
		if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXM)) {
			meson_crtc->enable_osd1_afbc =
					meson_viu_gxm_enable_osd1_afbc;
			meson_crtc->disable_osd1_afbc =
					meson_viu_gxm_disable_osd1_afbc;
		}
		drm_crtc_helper_add(crtc, &meson_crtc_helper_funcs);
	}

	priv->crtc = crtc;

	return 0;
}
