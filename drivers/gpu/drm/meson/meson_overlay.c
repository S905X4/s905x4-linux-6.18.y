// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
 */

#include <linux/bitfield.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_device.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>

#include "meson_overlay.h"
#include "meson_registers.h"
#include "meson_viu.h"
#include "meson_vpp.h"

/* VD1_IF0_GEN_REG */
#define VD_URGENT_CHROMA		BIT(28)
#define VD_URGENT_LUMA			BIT(27)
#define VD_HOLD_LINES(lines)		FIELD_PREP(GENMASK(24, 19), lines)
#define VD_DEMUX_MODE_RGB		BIT(16)
#define VD_BYTES_PER_PIXEL(val)		FIELD_PREP(GENMASK(15, 14), val)
#define VD_CHRO_RPT_LASTL_CTRL		BIT(6)
#define VD_LITTLE_ENDIAN		BIT(4)
#define VD_SEPARATE_EN			BIT(1)
#define VD_ENABLE			BIT(0)

/* VD1_IF0_CANVAS0 */
#define CANVAS_ADDR2(addr)		FIELD_PREP(GENMASK(23, 16), addr)
#define CANVAS_ADDR1(addr)		FIELD_PREP(GENMASK(15, 8), addr)
#define CANVAS_ADDR0(addr)		FIELD_PREP(GENMASK(7, 0), addr)

/* VD1_IF0_LUMA_X0 VD1_IF0_CHROMA_X0 */
#define VD_X_START(value)		FIELD_PREP(GENMASK(14, 0), value)
#define VD_X_END(value)			FIELD_PREP(GENMASK(30, 16), value)

/* VD1_IF0_LUMA_Y0 VD1_IF0_CHROMA_Y0 */
#define VD_Y_START(value)		FIELD_PREP(GENMASK(12, 0), value)
#define VD_Y_END(value)			FIELD_PREP(GENMASK(28, 16), value)

/* VD1_IF0_GEN_REG2 */
#define VD_COLOR_MAP(value)		FIELD_PREP(GENMASK(1, 0), value)

/* VIU_VD1_FMT_CTRL */
#define VD_HORZ_Y_C_RATIO(value)	FIELD_PREP(GENMASK(22, 21), value)
#define VD_HORZ_FMT_EN			BIT(20)
#define VD_HORZ_INITIAL_PHASE(value)	FIELD_PREP(GENMASK(27, 24), value)
#define VD_VERT_RPT_LINE0		BIT(16)
#define VD_VERT_INITIAL_PHASE(value)	FIELD_PREP(GENMASK(11, 8), value)
#define VD_VERT_PHASE_STEP(value)	FIELD_PREP(GENMASK(7, 1), value)
#define VD_VERT_FMT_EN			BIT(0)

/* VPP_POSTBLEND_VD1_H_START_END */
#define VD_H_END(value)			FIELD_PREP(GENMASK(11, 0), value)
#define VD_H_START(value)		FIELD_PREP(GENMASK(27, 16), \
						   ((value) & GENMASK(13, 0)))

/* VPP_POSTBLEND_VD1_V_START_END */
#define VD_V_END(value)			FIELD_PREP(GENMASK(11, 0), value)
#define VD_V_START(value)		FIELD_PREP(GENMASK(27, 16), value)

/* VPP_BLEND_VD2_V_START_END */
#define VD2_V_END(value)		FIELD_PREP(GENMASK(11, 0), value)
#define VD2_V_START(value)		FIELD_PREP(GENMASK(27, 16), value)

/* VIU_VD1_FMT_W */
#define VD_V_WIDTH(value)		FIELD_PREP(GENMASK(11, 0), value)
#define VD_H_WIDTH(value)		FIELD_PREP(GENMASK(27, 16), value)

/* VPP_HSC_REGION12_STARTP VPP_HSC_REGION34_STARTP */
#define VD_REGION24_START(value)	FIELD_PREP(GENMASK(11, 0), value)
#define VD_REGION13_END(value)		FIELD_PREP(GENMASK(27, 16), value)

/* AFBC_ENABLE */
#define AFBC_DEC_ENABLE			BIT(8)
#define AFBC_FRM_START			BIT(0)

/* AFBC_MODE */
#define AFBC_HORZ_SKIP_UV(value)	FIELD_PREP(GENMASK(1, 0), value)
#define AFBC_VERT_SKIP_UV(value)	FIELD_PREP(GENMASK(3, 2), value)
#define AFBC_HORZ_SKIP_Y(value)		FIELD_PREP(GENMASK(5, 4), value)
#define AFBC_VERT_SKIP_Y(value)		FIELD_PREP(GENMASK(7, 6), value)
#define AFBC_COMPBITS_YUV(value)	FIELD_PREP(GENMASK(13, 8), value)
#define AFBC_COMPBITS_8BIT		0
#define AFBC_COMPBITS_10BIT		(2 | (2 << 2) | (2 << 4))
#define AFBC_BURST_LEN(value)		FIELD_PREP(GENMASK(15, 14), value)
#define AFBC_HOLD_LINE_NUM(value)	FIELD_PREP(GENMASK(22, 16), value)
#define AFBC_MIF_URGENT(value)		FIELD_PREP(GENMASK(25, 24), value)
#define AFBC_REV_MODE(value)		FIELD_PREP(GENMASK(27, 26), value)
#define AFBC_BLK_MEM_MODE		BIT(28)
#define AFBC_SCATTER_MODE		BIT(29)
#define AFBC_SOFT_RESET			BIT(31)

/* AFBC_SIZE_IN */
#define AFBC_HSIZE_IN(value)		FIELD_PREP(GENMASK(28, 16), value)
#define AFBC_VSIZE_IN(value)		FIELD_PREP(GENMASK(12, 0), value)

/* AFBC_DEC_DEF_COLOR */
#define AFBC_DEF_COLOR_Y(value)		FIELD_PREP(GENMASK(29, 20), value)
#define AFBC_DEF_COLOR_U(value)		FIELD_PREP(GENMASK(19, 10), value)
#define AFBC_DEF_COLOR_V(value)		FIELD_PREP(GENMASK(9, 0), value)

/* AFBC_CONV_CTRL */
#define AFBC_CONV_LBUF_LEN(value)	FIELD_PREP(GENMASK(11, 0), value)

/* AFBC_LBUF_DEPTH */
#define AFBC_DEC_LBUF_DEPTH(value)	FIELD_PREP(GENMASK(27, 16), value)
#define AFBC_MIF_LBUF_DEPTH(value)	FIELD_PREP(GENMASK(11, 0), value)

/* AFBC_OUT_XSCOPE/AFBC_SIZE_OUT */
#define AFBC_HSIZE_OUT(value)		FIELD_PREP(GENMASK(28, 16), value)
#define AFBC_VSIZE_OUT(value)		FIELD_PREP(GENMASK(12, 0), value)
#define AFBC_OUT_HORZ_BGN(value)	FIELD_PREP(GENMASK(28, 16), value)
#define AFBC_OUT_HORZ_END(value)	FIELD_PREP(GENMASK(12, 0), value)

/* AFBC_OUT_YSCOPE */
#define AFBC_OUT_VERT_BGN(value)	FIELD_PREP(GENMASK(28, 16), value)
#define AFBC_OUT_VERT_END(value)	FIELD_PREP(GENMASK(12, 0), value)

/* AFBC_VD_CFMT_CTRL */
#define AFBC_HORZ_RPT_PIXEL0		BIT(23)
#define AFBC_HORZ_Y_C_RATIO(value)	FIELD_PREP(GENMASK(22, 21), value)
#define AFBC_HORZ_FMT_EN		BIT(20)
#define AFBC_VERT_RPT_LINE0		BIT(16)
#define AFBC_VERT_INITIAL_PHASE(value)	FIELD_PREP(GENMASK(11, 8), value)
#define AFBC_VERT_PHASE_STEP(value)	FIELD_PREP(GENMASK(7, 1), value)
#define AFBC_VERT_FMT_EN		BIT(0)

/* AFBC_VD_CFMT_W */
#define AFBC_VD_V_WIDTH(value)		FIELD_PREP(GENMASK(11, 0), value)
#define AFBC_VD_H_WIDTH(value)		FIELD_PREP(GENMASK(27, 16), value)

/* AFBC_MIF_HOR_SCOPE */
#define AFBC_MIF_BLK_BGN_H(value)	FIELD_PREP(GENMASK(25, 16), value)
#define AFBC_MIF_BLK_END_H(value)	FIELD_PREP(GENMASK(9, 0), value)

/* AFBC_MIF_VER_SCOPE */
#define AFBC_MIF_BLK_BGN_V(value)	FIELD_PREP(GENMASK(27, 16), value)
#define AFBC_MIF_BLK_END_V(value)	FIELD_PREP(GENMASK(11, 0), value)

/* AFBC_PIXEL_HOR_SCOPE */
#define AFBC_DEC_PIXEL_BGN_H(value)	FIELD_PREP(GENMASK(28, 16), \
						   ((value) & GENMASK(12, 0)))
#define AFBC_DEC_PIXEL_END_H(value)	FIELD_PREP(GENMASK(12, 0), value)

/* AFBC_PIXEL_VER_SCOPE */
#define AFBC_DEC_PIXEL_BGN_V(value)	FIELD_PREP(GENMASK(28, 16), value)
#define AFBC_DEC_PIXEL_END_V(value)	FIELD_PREP(GENMASK(12, 0), value)

/* AFBC_VD_CFMT_H */
#define AFBC_VD_HEIGHT(value)		FIELD_PREP(GENMASK(12, 0), value)

struct meson_overlay {
	struct drm_plane base;
	struct meson_drm *priv;
	unsigned int index;
};
#define to_meson_overlay(x) container_of(x, struct meson_overlay, base)

#define FRAC_16_16(mult, div)    (((mult) << 16) / (div))

static int meson_overlay_atomic_check(struct drm_plane *plane,
				      struct drm_atomic_state *state)
{
	struct drm_plane_state *new_plane_state = drm_atomic_get_new_plane_state(state,
										 plane);
	struct drm_crtc_state *crtc_state;

	if (!new_plane_state->crtc)
		return 0;

	crtc_state = drm_atomic_get_crtc_state(state,
					       new_plane_state->crtc);
	if (IS_ERR(crtc_state))
		return PTR_ERR(crtc_state);

	return drm_atomic_helper_check_plane_state(new_plane_state,
						   crtc_state,
						   FRAC_16_16(1, 5),
						   FRAC_16_16(5, 1),
						   true, true);
}

/* Takes a fixed 16.16 number and converts it to integer. */
static inline int64_t fixed16_to_int(int64_t value)
{
	return value >> 16;
}

static const uint8_t skip_tab[6] = {
	0x24, 0x04, 0x68, 0x48, 0x28, 0x08,
};

static void meson_overlay_get_vertical_phase(unsigned int ratio_y, int *phase,
					     int *repeat, bool interlace,
					     bool bottom)
{
	int offset_in = 0;
	int offset_out = 0;
	int repeat_skip = 0;

	if (!interlace && ratio_y > (1 << 18))
		offset_out = (1 * ratio_y) >> 10;
	else if (interlace && bottom)
		offset_out = (2 * ratio_y) >> 10;

	while ((offset_in + (4 << 8)) <= offset_out) {
		repeat_skip++;
		offset_in += 4 << 8;
	}

	*phase = (offset_out - offset_in) >> 2;

	if (*phase > 0x100)
		repeat_skip++;

	*phase = *phase & 0xff;

	if (repeat_skip > 5)
		repeat_skip = 5;

	*repeat = skip_tab[repeat_skip];
}

static void meson_overlay_setup_scaler_params(struct meson_drm *priv,
					      struct drm_plane *plane,
					      bool interlace_mode)
{
	struct meson_overlay *meson_overlay = to_meson_overlay(plane);
	struct meson_video_state *vd = &priv->viu.video[meson_overlay->index];
	struct drm_crtc_state *crtc_state = priv->crtc->state;
	int video_top, video_left, video_width, video_height;
	struct drm_plane_state *state = plane->state;
	unsigned int vd_start_lines, vd_end_lines;
	unsigned int hd_start_lines, hd_end_lines;
	unsigned int crtc_height, crtc_width;
	unsigned int vsc_startp, vsc_endp;
	unsigned int hsc_startp, hsc_endp;
	unsigned int crop_top, crop_left;
	int vphase, vphase_repeat_skip;
	int vphase_bottom, vphase_repeat_skip_bottom;
	unsigned int ratio_x, ratio_y;
	int temp_height, temp_width;
	unsigned int w_in, h_in;
	int afbc_left, afbc_right;
	int afbc_top_src, afbc_bottom_src;
	int afbc_top, afbc_bottom;
	int temp, start, end;

	if (!crtc_state) {
		DRM_ERROR("Invalid crtc_state\n");
		return;
	}

	crtc_height = crtc_state->mode.vdisplay;
	crtc_width = crtc_state->mode.hdisplay;

	w_in = fixed16_to_int(state->src_w);
	h_in = fixed16_to_int(state->src_h);
	crop_top = fixed16_to_int(state->src_y);
	crop_left = fixed16_to_int(state->src_x);

	video_top = state->crtc_y;
	video_left = state->crtc_x;
	video_width = state->crtc_w;
	video_height = state->crtc_h;

	DRM_DEBUG("crtc_width %d crtc_height %d interlace %d\n",
		  crtc_width, crtc_height, interlace_mode);
	DRM_DEBUG("w_in %d h_in %d crop_top %d crop_left %d\n",
		  w_in, h_in, crop_top, crop_left);
	DRM_DEBUG("video top %d left %d width %d height %d\n",
		  video_top, video_left, video_width, video_height);

	ratio_x = (w_in << 18) / video_width;
	ratio_y = (h_in << 18) / video_height;

	if (ratio_x * video_width < (w_in << 18))
		ratio_x++;

	DRM_DEBUG("ratio x 0x%x y 0x%x\n", ratio_x, ratio_y);

	/* Vertical */

	start = video_top + video_height / 2 - ((h_in << 17) / ratio_y);
	end = (h_in << 18) / ratio_y + start - 1;

	if (video_top < 0 && start < 0)
		vd_start_lines = (-(start) * ratio_y) >> 18;
	else if (start < video_top)
		vd_start_lines = ((video_top - start) * ratio_y) >> 18;
	else
		vd_start_lines = 0;

	if (video_top < 0)
		temp_height = min_t(unsigned int,
				    video_top + video_height - 1,
				    crtc_height - 1);
	else
		temp_height = min_t(unsigned int,
				    video_top + video_height - 1,
				    crtc_height - 1) - video_top + 1;

	temp = vd_start_lines + (temp_height * ratio_y >> 18);
	vd_end_lines = (temp <= (h_in - 1)) ? temp : (h_in - 1);

	vd_start_lines += crop_top;
	vd_end_lines += crop_top;

	/*
	 * TOFIX: Input frames are handled and scaled like progressive frames,
	 * proper handling of interlaced field input frames need to be figured
	 * out using the proper framebuffer flags set by userspace.
	 */

	vsc_startp = max_t(int, start,
			   max_t(int, 0, video_top));
	vsc_endp = min_t(int, end,
			 min_t(int, crtc_height - 1,
			       video_top + video_height - 1));
	if (interlace_mode) {
		vsc_startp >>= 1;
		vsc_endp >>= 1;
	}

	DRM_DEBUG("vsc startp %d endp %d start_lines %d end_lines %d\n",
		 vsc_startp, vsc_endp, vd_start_lines, vd_end_lines);

	afbc_top = round_down(vd_start_lines, 4);
	afbc_bottom = round_up(vd_end_lines + 1, 4);
	afbc_top_src = 0;
	afbc_bottom_src = round_up(state->fb->height, 4);

	DRM_DEBUG("afbc top %d (src %d) bottom %d (src %d)\n",
		  afbc_top, afbc_top_src, afbc_bottom, afbc_bottom_src);

	/* Horizontal */

	start = video_left + video_width / 2 - ((w_in << 17) / ratio_x);
	end = (w_in << 18) / ratio_x + start - 1;

	if (video_left < 0 && start < 0)
		hd_start_lines = (-(start) * ratio_x) >> 18;
	else if (start < video_left)
		hd_start_lines = ((video_left - start) * ratio_x) >> 18;
	else
		hd_start_lines = 0;

	if (video_left < 0)
		temp_width = min_t(unsigned int,
				   video_left + video_width - 1,
				   crtc_width - 1);
	else
		temp_width = min_t(unsigned int,
				   video_left + video_width - 1,
				   crtc_width - 1) - video_left + 1;

	temp = hd_start_lines + (temp_width * ratio_x >> 18);
	hd_end_lines = (temp <= (w_in - 1)) ? temp : (w_in - 1);

	vd->vpp_line_in_length = hd_end_lines - hd_start_lines + 1;
	hsc_startp = max_t(int, start, max_t(int, 0, video_left));
	hsc_endp = min_t(int, end, min_t(int, crtc_width - 1,
					 video_left + video_width - 1));

	hd_start_lines += crop_left;
	hd_end_lines += crop_left;

	DRM_DEBUG("hsc startp %d endp %d start_lines %d end_lines %d\n",
		 hsc_startp, hsc_endp, hd_start_lines, hd_end_lines);

	if (!priv->sc2 &&
	    (hd_start_lines > 0 || hd_end_lines < state->fb->width - 1)) {
		afbc_left = 0;
		afbc_right = round_up(state->fb->width, 32);
	} else {
		afbc_left = round_down(hd_start_lines, 32);
		afbc_right = round_up(hd_end_lines + 1, 32);
	}

	DRM_DEBUG("afbc left %d right %d\n", afbc_left, afbc_right);

	/* One output field consumes the full progressive input frame. */
	if (interlace_mode)
		ratio_y <<= 1;

	meson_overlay_get_vertical_phase(ratio_y, &vphase, &vphase_repeat_skip,
					 interlace_mode, false);
	meson_overlay_get_vertical_phase(ratio_y, &vphase_bottom,
					 &vphase_repeat_skip_bottom,
					 interlace_mode, true);

	vd->vpp_vsc_ini_phase = vphase << 8;
	vd->vpp_vsc_phase_ctrl = (1 << 13) | (4 << 8) |
				       vphase_repeat_skip;
	vd->vpp_vsc_ini_phase_bottom = vphase_bottom << 8;
	vd->vpp_vsc_phase_ctrl_bottom = (1 << 13) | (4 << 8) |
					      vphase_repeat_skip_bottom;

	vd->if0_luma_x0 = VD_X_START(hd_start_lines) |
				    VD_X_END(hd_end_lines);
	vd->if0_chroma_x0 = VD_X_START(hd_start_lines >> 1) |
				      VD_X_END(hd_end_lines >> 1);

	vd->fmt_w =
			VD_H_WIDTH(hd_end_lines - hd_start_lines + 1) |
			VD_V_WIDTH(hd_end_lines/2 - hd_start_lines/2 + 1);

	vd->afbc_vd_cfmt_w =
			AFBC_VD_H_WIDTH(afbc_right - afbc_left) |
			AFBC_VD_V_WIDTH(afbc_right / 2 - afbc_left / 2);

	vd->afbc_vd_cfmt_h =
			AFBC_VD_HEIGHT((afbc_bottom - afbc_top) / 2);

	vd->afbc_mif_hor_scope = AFBC_MIF_BLK_BGN_H(afbc_left / 32) |
				AFBC_MIF_BLK_END_H((afbc_right / 32) - 1);

	vd->afbc_mif_ver_scope = AFBC_MIF_BLK_BGN_V(afbc_top / 4) |
				AFBC_MIF_BLK_END_V((afbc_bottom / 4) - 1);

	vd->afbc_size_out =
			AFBC_HSIZE_OUT(afbc_right - afbc_left) |
			AFBC_VSIZE_OUT(afbc_bottom - afbc_top);

	vd->afbc_pixel_hor_scope =
			AFBC_DEC_PIXEL_BGN_H(hd_start_lines - afbc_left) |
			AFBC_DEC_PIXEL_END_H(hd_end_lines - afbc_left);

	vd->afbc_pixel_ver_scope =
			AFBC_DEC_PIXEL_BGN_V(vd_start_lines - afbc_top) |
			AFBC_DEC_PIXEL_END_V(vd_end_lines - afbc_top);

	vd->afbc_size_in =
				AFBC_HSIZE_IN(round_up(state->fb->width, 32)) |
				AFBC_VSIZE_IN(afbc_bottom_src - afbc_top_src);

	vd->if0_luma_y0 = VD_Y_START(vd_start_lines) |
				    VD_Y_END(vd_end_lines);

	vd->if0_chroma_y0 = VD_Y_START(vd_start_lines >> 1) |
				      VD_Y_END(vd_end_lines >> 1);
	if (priv->sc2 && state->fb->format->format == DRM_FORMAT_UYVY)
		vd->if0_chroma_y0 = vd->if0_luma_y0;

	vd->vpp_pic_in_height = vd_end_lines - vd_start_lines + 1;

	vd->postblend_h_start_end = VD_H_START(hsc_startp) |
						  VD_H_END(hsc_endp);
	vd->vpp_blend_vd2_h_start_end = VD_H_START(hd_start_lines) |
					      VD_H_END(hd_end_lines);
	vd->vpp_hsc_region12_startp = VD_REGION13_END(0) |
					    VD_REGION24_START(hsc_startp);
	vd->vpp_hsc_region34_startp =
				VD_REGION13_END(hsc_startp) |
				VD_REGION24_START(hsc_endp - hsc_startp);
	vd->vpp_hsc_region4_endp = hsc_endp - hsc_startp;
	vd->vpp_hsc_start_phase_step = ratio_x << 6;
	vd->vpp_hsc_region1_phase_slope = 0;
	vd->vpp_hsc_region3_phase_slope = 0;
	/* SC2 widened the repeat count downwards to bit 20, also in 4-tap mode. */
	vd->vpp_hsc_phase_ctrl = (1 << (priv->sc2 ? 20 : 21)) |
				       (4 << 16);

	vd->vpp_line_in_length = hd_end_lines - hd_start_lines + 1;
	vd->vpp_preblend_h_size = hd_end_lines - hd_start_lines + 1;

	vd->postblend_v_start_end = VD_V_START(vsc_startp) |
						  VD_V_END(vsc_endp);
	vd->vpp_blend_vd2_v_start_end =
				VD2_V_START((vd_end_lines + 1) >> 1) |
				VD2_V_END(vd_end_lines);

	vd->vpp_vsc_region12_startp = 0;
	vd->vpp_vsc_region34_startp =
				VD_REGION13_END(vsc_endp - vsc_startp) |
				VD_REGION24_START(vsc_endp - vsc_startp);
	vd->vpp_vsc_region4_endp = vsc_endp - vsc_startp;
	vd->vpp_vsc_start_phase_step = ratio_y << 6;
}

static void meson_overlay_atomic_update(struct drm_plane *plane,
					struct drm_atomic_state *state)
{
	struct meson_overlay *meson_overlay = to_meson_overlay(plane);
	struct drm_plane_state *new_state = drm_atomic_get_new_plane_state(state,
									   plane);
	struct drm_framebuffer *fb = new_state->fb;
	struct meson_drm *priv = meson_overlay->priv;
	struct meson_video_state *vd = &priv->viu.video[meson_overlay->index];
	struct drm_gem_dma_object *gem;
	unsigned long flags;
	bool interlace_mode;

	DRM_DEBUG_DRIVER("\n");

	interlace_mode = new_state->crtc->mode.flags & DRM_MODE_FLAG_INTERLACE;

	spin_lock_irqsave(&priv->drm->event_lock, flags);
	vd->interlace = interlace_mode;

	if ((fb->modifier & DRM_FORMAT_MOD_AMLOGIC_FBC(0, 0)) ==
			    DRM_FORMAT_MOD_AMLOGIC_FBC(0, 0)) {
		vd->afbc = true;

		vd->afbc_mode = AFBC_MIF_URGENT(3) |
					  AFBC_HOLD_LINE_NUM(8) |
					  AFBC_BURST_LEN(2);

		if ((fb->modifier >> __fourcc_mod_amlogic_options_shift) &
		    AMLOGIC_FBC_OPTION_MEM_SAVING)
			vd->afbc_mode |= AFBC_BLK_MEM_MODE;

		if ((fb->modifier & __fourcc_mod_amlogic_layout_mask) ==
				AMLOGIC_FBC_LAYOUT_SCATTER)
			vd->afbc_mode |= AFBC_SCATTER_MODE;

		vd->afbc_en = 0x1600 | AFBC_DEC_ENABLE;
		if (priv->sc2)
			vd->afbc_en |= BIT(19); /* DOS compression */

		vd->afbc_conv_ctrl = AFBC_CONV_LBUF_LEN(256);
		if (priv->sc2)
			vd->afbc_conv_ctrl |= 2 << 12; /* YUV420 */

		vd->afbc_dec_def_color = AFBC_DEF_COLOR_Y(1023);

		/* 420: horizontal / 2, vertical / 4 */
		vd->afbc_vd_cfmt_ctrl = AFBC_HORZ_RPT_PIXEL0 |
						  AFBC_HORZ_Y_C_RATIO(1) |
						  AFBC_HORZ_FMT_EN |
						  AFBC_VERT_RPT_LINE0 |
						  AFBC_VERT_INITIAL_PHASE(12) |
						  AFBC_VERT_PHASE_STEP(8) |
						  AFBC_VERT_FMT_EN;

		switch (fb->format->format) {
		/* AFBC Only formats */
		case DRM_FORMAT_YUV420_10BIT:
			vd->afbc_mode |=
				AFBC_COMPBITS_YUV(AFBC_COMPBITS_10BIT);
			vd->afbc_dec_def_color |=
					AFBC_DEF_COLOR_U(512) |
					AFBC_DEF_COLOR_V(512);
			break;
		case DRM_FORMAT_YUV420_8BIT:
			vd->afbc_dec_def_color |=
					AFBC_DEF_COLOR_U(128) |
					AFBC_DEF_COLOR_V(128);
			break;
		}

		vd->if0_gen_reg = 0;
		vd->if0_canvas0 = 0;
		vd->fmt_ctrl = 0;
	} else {
		vd->afbc = false;

		vd->if0_gen_reg = VD_URGENT_CHROMA |
					    VD_URGENT_LUMA |
					    VD_HOLD_LINES(9) |
					    VD_CHRO_RPT_LASTL_CTRL |
					    VD_ENABLE;
	}

	/* Setup scaler params */
	meson_overlay_setup_scaler_params(priv, plane, interlace_mode);

	vd->if0_repeat_loop = 0;
	/* Output fields are selected by VPP phase, not by skipping MIF rows. */
	vd->if0_luma0_rpt_pat = 0;
	vd->if0_chroma0_rpt_pat = 0;
	vd->range_map_y = 0;
	vd->range_map_cb = 0;
	vd->range_map_cr = 0;

	/* Default values for RGB888/YUV444 */
	vd->if0_gen_reg2 = 0;
	vd->fmt_ctrl = 0;
	vd->uyvy = priv->sc2 && fb->format->format == DRM_FORMAT_UYVY;

	/* None will match for AFBC Only formats */
	switch (fb->format->format) {
	/* TOFIX DRM_FORMAT_RGB888 should be supported */
	case DRM_FORMAT_UYVY:
		vd->if0_gen_reg |= VD_BYTES_PER_PIXEL(1) | VD_LITTLE_ENDIAN;
		vd->if0_canvas0 =
			CANVAS_ADDR2(priv->canvas_id_vd[meson_overlay->index][0]) |
			CANVAS_ADDR1(priv->canvas_id_vd[meson_overlay->index][0]) |
			CANVAS_ADDR0(priv->canvas_id_vd[meson_overlay->index][0]);
		/* SC2 video_hw.c: 4:2:2 has no vertical formatter. */
		vd->fmt_ctrl = VD_HORZ_Y_C_RATIO(1) | VD_HORZ_FMT_EN |
			VD_VERT_RPT_LINE0 | VD_VERT_INITIAL_PHASE(12) |
			VD_VERT_PHASE_STEP(16);
		if (FIELD_GET(GENMASK(14, 0), vd->if0_luma_x0) & 1)
			vd->fmt_ctrl |= VD_HORZ_INITIAL_PHASE(8);
		break;
	case DRM_FORMAT_YUYV:
		vd->if0_gen_reg |= VD_BYTES_PER_PIXEL(1);
		vd->if0_canvas0 =
					CANVAS_ADDR2(priv->canvas_id_vd[meson_overlay->index][0]) |
					CANVAS_ADDR1(priv->canvas_id_vd[meson_overlay->index][0]) |
					CANVAS_ADDR0(priv->canvas_id_vd[meson_overlay->index][0]);
		vd->fmt_ctrl = VD_HORZ_Y_C_RATIO(1) | /* /2 */
					     VD_HORZ_FMT_EN |
					     VD_VERT_RPT_LINE0 |
					     VD_VERT_INITIAL_PHASE(12) |
					     VD_VERT_PHASE_STEP(16) | /* /2 */
					     VD_VERT_FMT_EN;
		break;
	case DRM_FORMAT_NV12:
	case DRM_FORMAT_NV21:
		vd->if0_gen_reg |= VD_SEPARATE_EN;
		vd->if0_canvas0 =
					CANVAS_ADDR2(priv->canvas_id_vd[meson_overlay->index][1]) |
					CANVAS_ADDR1(priv->canvas_id_vd[meson_overlay->index][1]) |
					CANVAS_ADDR0(priv->canvas_id_vd[meson_overlay->index][0]);
		if (fb->format->format == DRM_FORMAT_NV12)
			vd->if0_gen_reg2 = VD_COLOR_MAP(1);
		else
			vd->if0_gen_reg2 = VD_COLOR_MAP(2);
		vd->fmt_ctrl = VD_HORZ_Y_C_RATIO(1) | /* /2 */
					     VD_HORZ_FMT_EN |
					     VD_VERT_RPT_LINE0 |
					     VD_VERT_INITIAL_PHASE(12) |
					     VD_VERT_PHASE_STEP(8) | /* /4 */
					     VD_VERT_FMT_EN;
		break;
	case DRM_FORMAT_YUV444:
	case DRM_FORMAT_YUV422:
	case DRM_FORMAT_YUV420:
	case DRM_FORMAT_YUV411:
	case DRM_FORMAT_YUV410:
		vd->if0_gen_reg |= VD_SEPARATE_EN;
		vd->if0_canvas0 =
					CANVAS_ADDR2(priv->canvas_id_vd[meson_overlay->index][2]) |
					CANVAS_ADDR1(priv->canvas_id_vd[meson_overlay->index][1]) |
					CANVAS_ADDR0(priv->canvas_id_vd[meson_overlay->index][0]);
		switch (fb->format->format) {
		case DRM_FORMAT_YUV422:
			vd->fmt_ctrl =
					VD_HORZ_Y_C_RATIO(1) | /* /2 */
					VD_HORZ_FMT_EN |
					VD_VERT_RPT_LINE0 |
					VD_VERT_INITIAL_PHASE(12) |
					VD_VERT_PHASE_STEP(16) | /* /2 */
					VD_VERT_FMT_EN;
			break;
		case DRM_FORMAT_YUV420:
			vd->fmt_ctrl =
					VD_HORZ_Y_C_RATIO(1) | /* /2 */
					VD_HORZ_FMT_EN |
					VD_VERT_RPT_LINE0 |
					VD_VERT_INITIAL_PHASE(12) |
					VD_VERT_PHASE_STEP(8) | /* /4 */
					VD_VERT_FMT_EN;
			break;
		case DRM_FORMAT_YUV411:
			vd->fmt_ctrl =
					VD_HORZ_Y_C_RATIO(2) | /* /4 */
					VD_HORZ_FMT_EN |
					VD_VERT_RPT_LINE0 |
					VD_VERT_INITIAL_PHASE(12) |
					VD_VERT_PHASE_STEP(16) | /* /2 */
					VD_VERT_FMT_EN;
			break;
		case DRM_FORMAT_YUV410:
			vd->fmt_ctrl =
					VD_HORZ_Y_C_RATIO(2) | /* /4 */
					VD_HORZ_FMT_EN |
					VD_VERT_RPT_LINE0 |
					VD_VERT_INITIAL_PHASE(12) |
					VD_VERT_PHASE_STEP(8) | /* /4 */
					VD_VERT_FMT_EN;
			break;
		}
		break;
	}

	/* Update Canvas with buffer address */
	vd->planes = fb->format->num_planes;

	switch (vd->planes) {
	case 3:
		gem = drm_fb_dma_get_gem_obj(fb, 2);
		vd->addr2 = gem->dma_addr + fb->offsets[2];
		vd->stride2 = fb->pitches[2];
		vd->height2 =
			drm_format_info_plane_height(fb->format,
						fb->height, 2);
		DRM_DEBUG("plane 2 addr 0x%x stride %d height %d\n",
			 vd->addr2,
			 vd->stride2,
			 vd->height2);
		fallthrough;
	case 2:
		gem = drm_fb_dma_get_gem_obj(fb, 1);
		vd->addr1 = gem->dma_addr + fb->offsets[1];
		vd->stride1 = fb->pitches[1];
		vd->height1 =
			drm_format_info_plane_height(fb->format,
						fb->height, 1);
		DRM_DEBUG("plane 1 addr 0x%x stride %d height %d\n",
			 vd->addr1,
			 vd->stride1,
			 vd->height1);
		fallthrough;
	case 1:
		gem = drm_fb_dma_get_gem_obj(fb, 0);
		vd->addr0 = gem->dma_addr + fb->offsets[0];
		vd->stride0 = fb->pitches[0];
		vd->height0 =
			drm_format_info_plane_height(fb->format,
						     fb->height, 0);
		DRM_DEBUG("plane 0 addr 0x%x stride %d height %d\n",
			 vd->addr0,
			 vd->stride0,
			 vd->height0);
	}

	if (vd->afbc) {
		if (vd->afbc_mode & AFBC_SCATTER_MODE) {
			/*
			 * In Scatter mode, the header contains the physical
			 * body content layout, thus the body content
			 * size isn't needed.
			 */
			vd->afbc_head_addr = vd->addr0 >> 4;
			vd->afbc_body_addr = 0;
		} else {
			/* Default mode is 4k per superblock */
			unsigned long block_size = 4096;
			unsigned long body_size;

			/* 8bit mem saving mode is 3072bytes per superblock */
			if (vd->afbc_mode & AFBC_BLK_MEM_MODE)
				block_size = 3072;

			body_size = (ALIGN(vd->stride0, 64) / 64) *
				    (ALIGN(vd->height0, 32) / 32) *
				    block_size;

			vd->afbc_body_addr = vd->addr0 >> 4;
			/* Header is after body content */
			vd->afbc_head_addr = (vd->addr0 +
							body_size) >> 4;
		}
	}

	vd->enabled = true;

	spin_unlock_irqrestore(&priv->drm->event_lock, flags);

	DRM_DEBUG_DRIVER("\n");
}

static void meson_overlay_atomic_disable(struct drm_plane *plane,
				       struct drm_atomic_state *state)
{
	struct meson_overlay *meson_overlay = to_meson_overlay(plane);
	struct meson_drm *priv = meson_overlay->priv;
	struct meson_video_state *vd = &priv->viu.video[meson_overlay->index];
	unsigned long flags;

	DRM_DEBUG_DRIVER("\n");

	spin_lock_irqsave(&priv->drm->event_lock, flags);
	vd->enabled = false;
	spin_unlock_irqrestore(&priv->drm->event_lock, flags);
}

static const struct drm_plane_helper_funcs meson_overlay_helper_funcs = {
	.atomic_check	= meson_overlay_atomic_check,
	.atomic_disable	= meson_overlay_atomic_disable,
	.atomic_update	= meson_overlay_atomic_update,
};

static bool meson_overlay_format_mod_supported(struct drm_plane *plane,
					       u32 format, u64 modifier)
{
	if (format == DRM_FORMAT_UYVY)
		return to_meson_overlay(plane)->priv->sc2 &&
		       modifier == DRM_FORMAT_MOD_LINEAR;

	if (modifier == DRM_FORMAT_MOD_LINEAR &&
	    format != DRM_FORMAT_YUV420_8BIT &&
	    format != DRM_FORMAT_YUV420_10BIT)
		return true;

	if ((modifier & DRM_FORMAT_MOD_AMLOGIC_FBC(0, 0)) ==
			DRM_FORMAT_MOD_AMLOGIC_FBC(0, 0)) {
		unsigned int layout = modifier &
			DRM_FORMAT_MOD_AMLOGIC_FBC(
				__fourcc_mod_amlogic_layout_mask, 0);
		unsigned int options =
			(modifier >> __fourcc_mod_amlogic_options_shift) &
			__fourcc_mod_amlogic_options_mask;

		if (format != DRM_FORMAT_YUV420_8BIT &&
		    format != DRM_FORMAT_YUV420_10BIT) {
			DRM_DEBUG_KMS("%llx invalid format 0x%08x\n",
				      modifier, format);
			return false;
		}

		if (layout != AMLOGIC_FBC_LAYOUT_BASIC &&
		    layout != AMLOGIC_FBC_LAYOUT_SCATTER) {
			DRM_DEBUG_KMS("%llx invalid layout %x\n",
				      modifier, layout);
			return false;
		}

		if (options &&
		    options != AMLOGIC_FBC_OPTION_MEM_SAVING) {
			DRM_DEBUG_KMS("%llx invalid layout %x\n",
				      modifier, layout);
			return false;
		}

		return true;
	}

	DRM_DEBUG_KMS("invalid modifier %llx for format 0x%08x\n",
		      modifier, format);

	return false;
}

static const struct drm_plane_funcs meson_overlay_funcs = {
	.update_plane		= drm_atomic_helper_update_plane,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.destroy		= drm_plane_cleanup,
	.reset			= drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_plane_destroy_state,
	.format_mod_supported   = meson_overlay_format_mod_supported,
};

static const uint32_t supported_drm_formats[] = {
	DRM_FORMAT_YUYV,
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV21,
	DRM_FORMAT_YUV444,
	DRM_FORMAT_YUV422,
	DRM_FORMAT_YUV420,
	DRM_FORMAT_YUV411,
	DRM_FORMAT_YUV410,
	DRM_FORMAT_YUV420_8BIT, /* Amlogic FBC Only */
	DRM_FORMAT_YUV420_10BIT, /* Amlogic FBC Only */
	DRM_FORMAT_UYVY, /* SC2 only; excluded below on older SoCs. */
};

static const uint64_t format_modifiers[] = {
	DRM_FORMAT_MOD_AMLOGIC_FBC(AMLOGIC_FBC_LAYOUT_SCATTER,
				   AMLOGIC_FBC_OPTION_MEM_SAVING),
	DRM_FORMAT_MOD_AMLOGIC_FBC(AMLOGIC_FBC_LAYOUT_BASIC,
				   AMLOGIC_FBC_OPTION_MEM_SAVING),
	DRM_FORMAT_MOD_AMLOGIC_FBC(AMLOGIC_FBC_LAYOUT_SCATTER, 0),
	DRM_FORMAT_MOD_AMLOGIC_FBC(AMLOGIC_FBC_LAYOUT_BASIC, 0),
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID,
};

/* VD2 shares the sourced SC2 linear 4:2:2, 4:2:0 and DOS-FBC paths. */
static const uint32_t sc2_vd2_formats[] = {
	DRM_FORMAT_UYVY,
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV21,
	DRM_FORMAT_YUV420_8BIT,
	DRM_FORMAT_YUV420_10BIT,
};

static int meson_overlay_create_one(struct meson_drm *priv, unsigned int index)
{
	struct meson_overlay *meson_overlay;
	struct drm_plane *plane;
	int ret;

	meson_overlay = devm_kzalloc(priv->drm->dev, sizeof(*meson_overlay),
				   GFP_KERNEL);
	if (!meson_overlay)
		return -ENOMEM;

	meson_overlay->priv = priv;
	meson_overlay->index = index;
	plane = &meson_overlay->base;

	ret = drm_universal_plane_init(priv->drm, plane, 1,
				       &meson_overlay_funcs,
				       index ? sc2_vd2_formats : supported_drm_formats,
				       index ? ARRAY_SIZE(sc2_vd2_formats) :
					       ARRAY_SIZE(supported_drm_formats) - !priv->sc2,
				       format_modifiers,
				       DRM_PLANE_TYPE_OVERLAY,
				       "meson_video_vd%u", index + 1);
	if (ret)
		return ret;

	drm_plane_helper_add(plane, &meson_overlay_helper_funcs);

	/* Fixed postblend ports: VD1 behind VD2, both behind OSD1. */
	drm_plane_create_zpos_immutable_property(plane, index);
	priv->overlay_planes[index] = plane;

	return 0;
}

int meson_overlay_create(struct meson_drm *priv)
{
	int ret;

	ret = meson_overlay_create_one(priv, 0);
	if (ret || !priv->sc2)
		return ret;

	return meson_overlay_create_one(priv, 1);
}
