/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef MESON_SC2_AVS2_API_H
#define MESON_SC2_AVS2_API_H

#include "meson_sc2_hw.h"

/* 23 vendor reference/reorder slots and one independent background slot. */
#define MESON_SC2_AVS2_MIN_CAPTURE_BUFFERS 24
#define MESON_SC2_AVS2_MAX_WIDTH 3840U
#define MESON_SC2_AVS2_MAX_HEIGHT 2160U

/* One complete start-code-delimited picture AU or a header-only AU. Sequence
 * headers may precede the picture in earlier submissions. CAPTURE is AMF8/AMF0
 * FBC, never a DW8 surrogate.
 * output.destination and output.output_buffer must describe that same object,
 * or both be NULL for an actual firmware header probe before CAPTURE exists.
 * The caller keeps each surface alive while surface_busy() returns true.
 */
struct meson_sc2_avs2_job {
	const void *data;
	size_t size;
	u64 timestamp;
	struct meson_sc2_hevc_job output;
};

struct meson_sc2_avs2_result {
	struct meson_sc2_surface *surface;
	u64 timestamp;
	u32 width;
	u32 height;
	u8 depth;
	s32 poc;
	bool hidden;
};

/* -EPIPE reports parsed dimensions/depth in result without writing CAPTURE.
 * NULL-destination probes and mismatched allocations stop before DPB mutation;
 * the caller retains and replays the same access unit after reconfiguration.
 * -EAGAIN means the supplied destination is still retained by the DPB.
 * Successful decode does not imply display readiness: call dequeue separately.
 */
int meson_sc2_avs2_decode(struct meson_sc2_hevc_context *context,
			const struct meson_sc2_avs2_job *job,
			struct meson_sc2_avs2_result *result);
/* Only for a source-change CAPTURE stop after the worker and old presentations
 * have drained. Retires DPB/workspace but retains consumed encoded headers for
 * the queued -EPIPE AU. Ordinary STREAMOFF/seek uses context_reset instead.
 */
int meson_sc2_avs2_capture_restart(struct meson_sc2_hevc_context *context);
/* Acknowledge removal of the head SRC only after the frontend's abort check.
 * Header-only success commits its prefix; a consumed picture/error retires it.
 * An unconsumed source must instead survive CAPTURE restart or full reset.
 */
void meson_sc2_avs2_source_consumed(struct meson_sc2_hevc_context *context,
				    bool picture, int status);
/* Returns -EAGAIN when no output is ready. drain presents remaining reordered
 * pictures for DECODER_CMD_STOP without retiring reference/background state;
 * DECODER_CMD_START may resume dependent access units. STREAMOFF resets DPB.
 */
int meson_sc2_avs2_dequeue(struct meson_sc2_hevc_context *context, bool drain,
			 struct meson_sc2_avs2_result *result);
int meson_sc2_avs2_put_output(struct meson_sc2_hevc_context *context,
			    struct meson_sc2_surface *surface);
bool meson_sc2_avs2_surface_busy(struct meson_sc2_hevc_context *context,
			       struct meson_sc2_surface *surface);

#endif
