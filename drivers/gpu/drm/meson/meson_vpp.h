/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2016 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 */

/* Video Post Process */

#ifndef __MESON_VPP_H
#define __MESON_VPP_H

struct drm_rect;
struct meson_drm;

/* Mux VIU/VPP to ENCL */
#define MESON_VIU_VPP_MUX_ENCL	0x0
/* Mux VIU/VPP to ENCI */
#define MESON_VIU_VPP_MUX_ENCI	0x5
/* Mux VIU/VPP to ENCP */
#define MESON_VIU_VPP_MUX_ENCP	0xA

struct meson_vpp_video_regs {
	unsigned int scale_coef_idx;
	unsigned int scale_coef;
	unsigned int vsc_region12_startp;
	unsigned int vsc_region34_startp;
	unsigned int vsc_region4_endp;
	unsigned int vsc_start_phase_step;
	unsigned int vsc_phase_ctrl;
	unsigned int vsc_ini_phase;
	unsigned int hsc_region12_startp;
	unsigned int hsc_region34_startp;
	unsigned int hsc_region4_endp;
	unsigned int hsc_start_phase_step;
	unsigned int hsc_region1_phase_slope;
	unsigned int hsc_region3_phase_slope;
	unsigned int hsc_phase_ctrl;
	unsigned int sc_misc;
	unsigned int hsc_phase_ctrl1;
	unsigned int postblend_h_start_end;
	unsigned int postblend_v_start_end;
};

extern const struct meson_vpp_video_regs meson_vpp_video_regs[2];

void meson_vpp_setup_mux(struct meson_drm *priv, unsigned int mux);

void meson_vpp_setup_interlace_vscaler_osd1(struct meson_drm *priv,
					    struct drm_rect *input);
void meson_vpp_disable_interlace_vscaler_osd1(struct meson_drm *priv);

void meson_vpp_init(struct meson_drm *priv);

#endif /* __MESON_VPP_H */
