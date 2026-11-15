/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef MESON_SC2_VP9_API_H
#define MESON_SC2_VP9_API_H
#include "meson_sc2_hw.h"

/* One VP9 frame, without IVF/AMLV framing or a superframe index. The
 * stateful frontend/FFmpeg superframe splitter supplies independent AUs.
 */
struct meson_sc2_vp9_job {
	const void *data;
	size_t size;
	u64 timestamp;
	struct meson_sc2_hevc_job output;
};
struct meson_sc2_vp9_result {
	struct meson_sc2_surface *surface;
	u64 timestamp;
	u32 width, height;
	u8 depth;
	s32 poc;
	bool hidden;
};
/* Caller has aborted and joined its worker, and released old presentations.
 * Keep the committed references/workspace when restarting CAPTURE only.
 */
int meson_sc2_vp9_capture_restart(struct meson_sc2_hevc_context *context);
int meson_sc2_vp9_decode(struct meson_sc2_hevc_context *context,
			const struct meson_sc2_vp9_job *job,
			struct meson_sc2_vp9_result *result);
int meson_sc2_vp9_dequeue(struct meson_sc2_hevc_context *context, bool drain,
			 struct meson_sc2_vp9_result *result);
int meson_sc2_vp9_put_output(struct meson_sc2_hevc_context *context,
			    struct meson_sc2_surface *surface);
bool meson_sc2_vp9_surface_busy(struct meson_sc2_hevc_context *context,
			       struct meson_sc2_surface *surface);
void meson_sc2_vp9_adapt_prob(int prev_key, int key, int context,
			    unsigned int *previous, unsigned int *current_prob,
			    unsigned int *count);
#endif
