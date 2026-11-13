/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef MESON_SC2_REQUEST_HW_H
#define MESON_SC2_REQUEST_HW_H

#include <linux/dma-mapping.h>
#include <linux/types.h>
#include <linux/v4l2-controls.h>

struct device;
struct dma_buf;
struct meson_sc2_hw;
struct meson_sc2_hevc_context;
struct meson_hevc_syntax_request;
struct meson_av1_syntax_request;

/* References remain tied to one context and CAPTURE buffer. Main10 header/body
 * storage belongs to that buffer's exported vb2 allocation; MV stays private.
 */
struct meson_sc2_surface {
	void *cpu;
	dma_addr_t dma;
	size_t size;
	u32 header_size; /* HEVC_CM_HEADER_LENGTH, not the scatter allocation. */
	u32 header_alloc_size;
	u32 body_offset;
	u32 body_size;
	u32 mv_offset;
	u32 mv_size;
	dma_addr_t mv_dma;
	bool capture_owned;
	u32 width;
	u32 height;
	u8 depth;
	s32 poc;
	bool valid;
};

struct meson_sc2_hevc_job {
	const struct meson_hevc_syntax_request *syntax;
	struct meson_sc2_surface *destination;
	/* Indexed exactly like native decode.dpb[], resolved within this context. */
	struct meson_sc2_surface *references[16];
	dma_addr_t output_dma;
	size_t output_size;
	u32 output_stride;
	/* Negotiated V4L2 CAPTURE fourcc, separate from FBC reference depth. */
	u32 output_format;
	/* Caller-owned allocation; hardware pins it until DMA is quiescent. */
	struct dma_buf *output_buffer;
	u32 output_uv_offset;
};

/* AV1 uses the same engine, output pinning and abort lifetime as HEVC. */
struct meson_sc2_av1_job {
	struct meson_av1_syntax_request *syntax;
	struct meson_sc2_hevc_job output;
	struct meson_sc2_surface *references[8];
	u64 timestamp;
};

int meson_sc2_hw_create(struct device *dev, struct meson_sc2_hw **result);
void meson_sc2_hw_destroy(struct meson_sc2_hw *hw);
int meson_sc2_hevc_context_create(struct meson_sc2_hw *hw,
				 struct meson_sc2_hevc_context **result);
void meson_sc2_hevc_context_destroy(struct meson_sc2_hevc_context *context);
int meson_sc2_surface_alloc(struct meson_sc2_hw *hw,
			   struct meson_sc2_surface *surface,
			   u32 width, u32 height, u8 depth);
u32 meson_sc2_hevc_fbc_size(u32 width, u32 height);
int meson_sc2_hevc_fbc_bind(struct meson_sc2_hevc_context *context,
			   struct meson_sc2_surface *surface,
			   u32 width, u32 height, void *cpu, dma_addr_t dma,
			   size_t size, struct dma_buf *buffer);
/* Stateful VP9/AVS2 keep the actual 8/10-bit FBC depth; MV storage is
 * codec-sized (VP9 0x240, AVS2 0x200 bytes per 64x64 block).
 */
int meson_sc2_modern_fbc_bind(struct meson_sc2_hevc_context *context,
			     struct meson_sc2_surface *surface,
			     u32 width, u32 height, u8 depth, u32 mv_unit,
			     void *cpu, dma_addr_t dma, size_t size,
			     struct dma_buf *buffer);
int meson_sc2_av1_surface_alloc(struct meson_sc2_hw *hw,
			       struct meson_sc2_surface *surface,
			       u32 width, u32 height);
int meson_sc2_av1_fbc_bind(struct meson_sc2_hevc_context *context,
			  struct meson_sc2_surface *surface,
			  u32 width, u32 height, void *cpu, dma_addr_t dma,
			  size_t size, struct dma_buf *buffer);
void meson_sc2_surface_free(struct meson_sc2_hw *hw,
			   struct meson_sc2_surface *surface);
/* Blocking, called only by the frontend's serialized work item. Each job
 * establishes fresh parser state; full POC/list/storage authority is native
 * controls, not stale firmware RPM state. Abort wakes its bounded waits.
 */
int meson_sc2_hevc_decode(struct meson_sc2_hevc_context *context,
			  const struct meson_sc2_hevc_job *job);
void meson_sc2_hevc_abort(struct meson_sc2_hevc_context *context);
/* Call reset only after the frontend has flushed its decode worker. */
int meson_sc2_hevc_context_reset(struct meson_sc2_hevc_context *context);
bool meson_sc2_hevc_context_fatal(struct meson_sc2_hevc_context *context);
int meson_sc2_av1_decode(struct meson_sc2_hevc_context *context,
			 const struct meson_sc2_av1_job *job);

#endif
