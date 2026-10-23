/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2018 BayLibre, SAS
 * Author: Maxime Jourdan <mjourdan@baylibre.com>
 */

#ifndef __MESON_VDEC_VDEC_1_H_
#define __MESON_VDEC_VDEC_1_H_

#include "vdec.h"

extern struct amvdec_ops vdec_1_ops;
/* SC2 stopped run boundary: optional codec reset precedes the common core reset.
 * Any drain failure retains quarantined resources.
 */
int amvdec_vdec1_reset(struct amvdec_session *sess, u32 reset_mask);

#endif
