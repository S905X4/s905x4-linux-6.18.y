/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __MESON_SC2_DOS_H
#define __MESON_SC2_DOS_H

struct device;

enum meson_sc2_dos_engine {
	MESON_SC2_DOS_VDEC,
	MESON_SC2_DOS_HEVC,
};

/* Acquire before power/clocks or any DOS access; release only after DMA idle.
 * The same device/engine may nest leases; different owners receive -EBUSY.
 * A failed DMA drain must retain the lease and all DMA-addressable storage.
 */
int meson_sc2_dos_acquire(struct device *dev, enum meson_sc2_dos_engine engine);
void meson_sc2_dos_release(struct device *dev, enum meson_sc2_dos_engine engine);

#endif
