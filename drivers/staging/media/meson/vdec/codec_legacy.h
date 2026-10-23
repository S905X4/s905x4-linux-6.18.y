/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef __MESON_VDEC_CODEC_LEGACY_H_
#define __MESON_VDEC_CODEC_LEGACY_H_

#include "vdec.h"

/* Factory avs_multi image; the single avs image fixes its canvas IDs at zero. */
#define MESON_AVS_FIRMWARE_INDEX 0x4156534d

extern struct amvdec_codec_ops codec_vc1_ops;
extern struct amvdec_codec_ops codec_avs_ops;
extern struct amvdec_codec_ops codec_mjpeg_ops;

#endif
