/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef MESON_SC2_STATEFUL_H
#define MESON_SC2_STATEFUL_H

struct device;
struct v4l2_device;
struct meson_sc2_hw;
struct meson_sc2_stateful;

/* The owner retains the shared engine until the last stateful file closes. */
int meson_sc2_stateful_register(struct device *dev, struct meson_sc2_hw *hw,
			       struct v4l2_device *owner,
			       struct meson_sc2_stateful **node);
void meson_sc2_stateful_unregister(struct meson_sc2_stateful *node);

#endif
