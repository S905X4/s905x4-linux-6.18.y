// SPDX-License-Identifier: GPL-2.0-only
/* Native stateful VP9/AVS2 frontend for the existing shared SC2 engine. */
#include <linux/dma-buf.h>
#include <linux/fcntl.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "meson_sc2_stateful.h"
#include "meson_sc2_vp9_api.h"
#include "meson_sc2_avs2_api.h"

#ifndef V4L2_PIX_FMT_AVS2
#define V4L2_PIX_FMT_AVS2 v4l2_fourcc('A', 'V', 'S', '2')
#endif
#ifndef V4L2_PIX_FMT_AMLOGIC_FBC8
#define V4L2_PIX_FMT_AMLOGIC_FBC8 v4l2_fourcc('A', 'M', 'F', '8')
#endif

#define SC2_STATEFUL_NAME "meson-sc2-stateful"
#define SC2_STATEFUL_MAX_WIDTH 3840U
#define SC2_STATEFUL_MAX_HEIGHT 2160U
#define SC2_STATEFUL_SOURCE_SIZE (8U * 1024U * 1024U)
/* Leave room for the native parser's padding and VP9 framing guard. */
#define SC2_STATEFUL_MAX_SOURCE (16U * 1024U * 1024U - 16U * 1024U - 64U)
#define SC2_VP9_MIN_CAPTURE 12U

struct meson_sc2_stateful {
	struct device *dev;
	struct meson_sc2_hw *hw;
	struct v4l2_device *owner;
	struct v4l2_device v4l2;
	struct video_device video;
	struct v4l2_m2m_dev *m2m;
	struct workqueue_struct *workqueue;
	struct mutex lock;
	struct list_head contexts;
	bool removing;
};

struct sc2_stateful_buffer {
	struct v4l2_m2m_buffer m2m;
	struct list_head capture_entry;
	struct meson_sc2_surface surface;
	struct dma_buf *export;
	bool queued;
	/* A DONE buffer still owns one presentation until its next QBUF. */
	bool user_output;
};

struct sc2_stateful_picture {
	struct meson_sc2_surface *surface;
	u64 timestamp;
	u32 width;
	u32 height;
	u8 depth;
};

struct sc2_stateful_ctx {
	struct v4l2_fh fh;
	struct list_head entry;
	struct meson_sc2_stateful *node;
	struct meson_sc2_hevc_context *hwctx;
	struct v4l2_ctrl_handler controls;
	/* The worker never takes node->lock: STREAMOFF flushes it under that lock. */
	struct mutex state_lock;
	struct work_struct work;
	struct delayed_work retry_work;
	struct list_head captures;
	struct v4l2_pix_format_mplane source_fmt;
	struct v4l2_pix_format_mplane capture_fmt;
	struct sc2_stateful_picture pending;
	struct vb2_v4l2_buffer *drain_tail;
	u32 width;
	u32 height;
	u32 source_sequence;
	u32 capture_sequence;
	u8 depth;
	bool header_known;
	bool output_streaming;
	bool capture_streaming;
	bool pending_valid;
	bool poll_output;
	bool reconfigure;
	bool reconfigure_needs_off;
	/* Head AVS2 SRC still owes a replay, even after CAPTURE ON. */
	bool avs2_replay_pending;
	bool last_sent;
	bool draining;
	bool drain_ready;
	bool drain_exhausted;
	bool eos;
	bool decode_blocked;
	bool retry_wait;
	bool aborting;
	/* Snapshot for job_ready(), which is called with the M2M spinlock held. */
	bool ready;
};

static struct sc2_stateful_ctx *sc2_stateful_file(struct file *file)
{
	return container_of(file->private_data, struct sc2_stateful_ctx, fh);
}

static struct sc2_stateful_buffer *sc2_stateful_buffer(struct vb2_v4l2_buffer *vb)
{
	return container_of(vb, struct sc2_stateful_buffer, m2m.vb);
}

static bool sc2_stateful_avs2(struct sc2_stateful_ctx *ctx)
{
	return ctx->source_fmt.pixelformat == V4L2_PIX_FMT_AVS2;
}

static unsigned int sc2_stateful_min_capture(struct sc2_stateful_ctx *ctx)
{
	return sc2_stateful_avs2(ctx) ? MESON_SC2_AVS2_MIN_CAPTURE_BUFFERS :
		SC2_VP9_MIN_CAPTURE;
}

static bool sc2_stateful_busy(struct sc2_stateful_ctx *ctx,
			      struct meson_sc2_surface *surface)
{
	return sc2_stateful_avs2(ctx) ?
		meson_sc2_avs2_surface_busy(ctx->hwctx, surface) :
		meson_sc2_vp9_surface_busy(ctx->hwctx, surface);
}

static int sc2_stateful_put(struct sc2_stateful_ctx *ctx,
			    struct meson_sc2_surface *surface)
{
	return sc2_stateful_avs2(ctx) ?
		meson_sc2_avs2_put_output(ctx->hwctx, surface) :
		meson_sc2_vp9_put_output(ctx->hwctx, surface);
}

static int sc2_stateful_dequeue(struct sc2_stateful_ctx *ctx, bool drain,
				struct sc2_stateful_picture *picture)
{
	int ret;

	if (sc2_stateful_avs2(ctx)) {
		struct meson_sc2_avs2_result result = {};

		ret = meson_sc2_avs2_dequeue(ctx->hwctx, drain, &result);
		if (!ret)
			*picture = (struct sc2_stateful_picture) {
				.surface = result.surface, .timestamp = result.timestamp,
				.width = result.width, .height = result.height,
				.depth = result.depth,
			};
	} else {
		struct meson_sc2_vp9_result result = {};

		ret = meson_sc2_vp9_dequeue(ctx->hwctx, drain, &result);
		if (!ret)
			*picture = (struct sc2_stateful_picture) {
				.surface = result.surface, .timestamp = result.timestamp,
				.width = result.width, .height = result.height,
				.depth = result.depth,
			};
	}
	return ret;
}

static struct sc2_stateful_buffer *sc2_stateful_find(struct sc2_stateful_ctx *ctx,
						   struct meson_sc2_surface *surface)
{
	struct sc2_stateful_buffer *buf;

	if (!surface || !surface->cpu)
		return NULL;
	list_for_each_entry(buf, &ctx->captures, capture_entry)
		if (buf->surface.cpu == surface->cpu && buf->surface.dma == surface->dma)
			return buf;
	return NULL;
}

static struct sc2_stateful_buffer *sc2_stateful_destination(struct sc2_stateful_ctx *ctx)
{
	struct sc2_stateful_buffer *buf;

	list_for_each_entry(buf, &ctx->captures, capture_entry)
		if (buf->queued && !buf->user_output &&
		    !sc2_stateful_busy(ctx, &buf->surface))
			return buf;
	return NULL;
}

static void sc2_stateful_update_ready(struct sc2_stateful_ctx *ctx)
{
	struct sc2_stateful_buffer *buf;
	bool ready = false;

	if (ctx->aborting || ctx->node->removing || !ctx->output_streaming || ctx->eos)
		goto out;
	if (ctx->pending_valid) {
		buf = sc2_stateful_find(ctx, ctx->pending.surface);
		ready = ctx->capture_streaming && buf && buf->queued;
		goto out;
	}
	if (ctx->reconfigure || (ctx->draining && ctx->drain_ready)) {
		if (!ctx->capture_streaming || ctx->last_sent)
			goto out;
		ready = !ctx->drain_exhausted ||
			v4l2_m2m_num_dst_bufs_ready(ctx->fh.m2m_ctx);
		goto out;
	}
	if (ctx->poll_output && ctx->capture_streaming) {
		ready = true;
		goto out;
	}
	if (!v4l2_m2m_num_src_bufs_ready(ctx->fh.m2m_ctx) ||
	    ctx->decode_blocked || ctx->retry_wait)
		goto out;
	ready = !ctx->header_known ||
		(ctx->capture_streaming && sc2_stateful_destination(ctx));
out:
	WRITE_ONCE(ctx->ready, ready);
}

static void sc2_stateful_capture_format(struct sc2_stateful_ctx *ctx,
				       struct v4l2_pix_format_mplane *pix,
				       u32 fourcc)
{
	if (ctx->header_known)
		fourcc = ctx->depth == 10 ? V4L2_PIX_FMT_AMLOGIC_FBC10 :
			V4L2_PIX_FMT_AMLOGIC_FBC8;
	if (fourcc != V4L2_PIX_FMT_AMLOGIC_FBC10)
		fourcc = V4L2_PIX_FMT_AMLOGIC_FBC8;
	memset(pix, 0, sizeof(*pix));
	pix->pixelformat = fourcc;
	pix->width = ALIGN(ctx->width, 64);
	pix->height = ALIGN(ctx->height, 64);
	pix->field = V4L2_FIELD_NONE;
	pix->num_planes = 1;
	/* Scatter FBC has no linear row stride; DRM uses the padded pixel width. */
	pix->plane_fmt[0].sizeimage = meson_sc2_hevc_fbc_size(ctx->width, ctx->height);
	pix->colorspace = ctx->source_fmt.colorspace;
	pix->xfer_func = ctx->source_fmt.xfer_func;
	pix->ycbcr_enc = ctx->source_fmt.ycbcr_enc;
	pix->quantization = ctx->source_fmt.quantization;
}

static bool sc2_stateful_type(enum v4l2_buf_type type)
{
	return type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE ||
		type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
}

static int sc2_stateful_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct sc2_stateful_ctx *ctx = sc2_stateful_file(file);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	u32 size = pix->plane_fmt[0].sizeimage;

	if (!sc2_stateful_type(f->type))
		return -EINVAL;
	mutex_lock(&ctx->state_lock);
	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		sc2_stateful_capture_format(ctx, pix, pix->pixelformat);
	} else {
		if (pix->pixelformat != V4L2_PIX_FMT_AVS2)
			pix->pixelformat = V4L2_PIX_FMT_VP9;
		pix->width = clamp_t(u32, pix->width, 64, SC2_STATEFUL_MAX_WIDTH);
		pix->height = clamp_t(u32, pix->height, 64, SC2_STATEFUL_MAX_HEIGHT);
		pix->width = ALIGN(pix->width, 2);
		pix->height = ALIGN(pix->height, 2);
		pix->field = V4L2_FIELD_NONE;
		pix->num_planes = 1;
		memset(pix->plane_fmt, 0, sizeof(pix->plane_fmt));
		pix->plane_fmt[0].sizeimage = clamp_t(u32, size,
			SC2_STATEFUL_SOURCE_SIZE, SC2_STATEFUL_MAX_SOURCE);
		if (!pix->colorspace)
			pix->colorspace = V4L2_COLORSPACE_REC709;
		memset(pix->reserved, 0, sizeof(pix->reserved));
	}
	mutex_unlock(&ctx->state_lock);
	return 0;
}

static int sc2_stateful_get_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct sc2_stateful_ctx *ctx = sc2_stateful_file(file);

	if (!sc2_stateful_type(f->type))
		return -EINVAL;
	mutex_lock(&ctx->state_lock);
	f->fmt.pix_mp = V4L2_TYPE_IS_OUTPUT(f->type) ? ctx->source_fmt : ctx->capture_fmt;
	mutex_unlock(&ctx->state_lock);
	return 0;
}

static int sc2_stateful_set_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct sc2_stateful_ctx *ctx = sc2_stateful_file(file);
	struct vb2_queue *q;
	int ret;

	if (!sc2_stateful_type(f->type))
		return -EINVAL;
	q = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (vb2_is_busy(q) || (V4L2_TYPE_IS_OUTPUT(f->type) &&
		vb2_is_busy(v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx))))
		return -EBUSY;
	ret = sc2_stateful_try_fmt(file, priv, f);
	if (ret)
		return ret;
	mutex_lock(&ctx->state_lock);
	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		ctx->source_fmt = f->fmt.pix_mp;
		ctx->width = ctx->source_fmt.width;
		ctx->height = ctx->source_fmt.height;
		ctx->header_known = false;
		sc2_stateful_capture_format(ctx, &ctx->capture_fmt,
					    ctx->capture_fmt.pixelformat);
		v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx)->min_reqbufs_allocation =
			sc2_stateful_min_capture(ctx);
	} else {
		sc2_stateful_capture_format(ctx, &f->fmt.pix_mp,
					    f->fmt.pix_mp.pixelformat);
		ctx->capture_fmt = f->fmt.pix_mp;
	}
	mutex_unlock(&ctx->state_lock);
	return 0;
}

static int sc2_stateful_enum_fmt(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	if (f->index > 1 || !sc2_stateful_type(f->type))
		return -EINVAL;
	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		f->pixelformat = f->index ? V4L2_PIX_FMT_AVS2 : V4L2_PIX_FMT_VP9;
		f->flags = V4L2_FMT_FLAG_COMPRESSED | V4L2_FMT_FLAG_DYN_RESOLUTION;
	} else {
		f->pixelformat = f->index ? V4L2_PIX_FMT_AMLOGIC_FBC10 :
			V4L2_PIX_FMT_AMLOGIC_FBC8;
	}
	return 0;
}

static bool sc2_stateful_fourcc(u32 fourcc)
{
	return fourcc == V4L2_PIX_FMT_VP9 || fourcc == V4L2_PIX_FMT_AVS2 ||
		fourcc == V4L2_PIX_FMT_AMLOGIC_FBC8 || fourcc == V4L2_PIX_FMT_AMLOGIC_FBC10;
}

static int sc2_stateful_framesizes(struct file *file, void *priv,
				   struct v4l2_frmsizeenum *f)
{
	bool capture = f->pixel_format == V4L2_PIX_FMT_AMLOGIC_FBC8 ||
		f->pixel_format == V4L2_PIX_FMT_AMLOGIC_FBC10;

	if (f->index || !sc2_stateful_fourcc(f->pixel_format))
		return -EINVAL;
	f->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	f->stepwise = (struct v4l2_frmsize_stepwise) {
		.min_width = 64, .max_width = SC2_STATEFUL_MAX_WIDTH,
		.step_width = capture ? 64 : 2,
		.min_height = 64,
		.max_height = capture ? ALIGN(SC2_STATEFUL_MAX_HEIGHT, 64) :
			SC2_STATEFUL_MAX_HEIGHT,
		.step_height = capture ? 64 : 2,
	};
	return 0;
}

static int sc2_stateful_frameintervals(struct file *file, void *priv,
				       struct v4l2_frmivalenum *f)
{
	bool capture = f->pixel_format == V4L2_PIX_FMT_AMLOGIC_FBC8 ||
		f->pixel_format == V4L2_PIX_FMT_AMLOGIC_FBC10;

	if (f->index || !sc2_stateful_fourcc(f->pixel_format) ||
	    f->width < 64 || f->width > SC2_STATEFUL_MAX_WIDTH ||
	    f->height < 64 || f->height > (capture ?
		ALIGN(SC2_STATEFUL_MAX_HEIGHT, 64) : SC2_STATEFUL_MAX_HEIGHT) ||
	    !IS_ALIGNED(f->width, capture ? 64 : 2) ||
	    !IS_ALIGNED(f->height, capture ? 64 : 2))
		return -EINVAL;
	f->type = V4L2_FRMIVAL_TYPE_CONTINUOUS;
	f->stepwise.min = (struct v4l2_fract) { 1, 60 };
	f->stepwise.max = (struct v4l2_fract) { 1, 1 };
	f->stepwise.step = (struct v4l2_fract) { 0, 1 };
	return 0;
}

static int sc2_stateful_selection(struct file *file, void *priv,
				  struct v4l2_selection *s)
{
	struct sc2_stateful_ctx *ctx = sc2_stateful_file(file);
	int ret = 0;

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	mutex_lock(&ctx->state_lock);
	switch (s->target) {
	case V4L2_SEL_TGT_COMPOSE:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
		s->r = (struct v4l2_rect) { 0, 0, ctx->width, ctx->height };
		break;
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	case V4L2_SEL_TGT_COMPOSE_PADDED:
		s->r = (struct v4l2_rect) { 0, 0, ctx->capture_fmt.width,
			ctx->capture_fmt.height };
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&ctx->state_lock);
	return ret;
}

static int sc2_stateful_querycap(struct file *file, void *priv,
				 struct v4l2_capability *cap)
{
	strscpy(cap->driver, "meson-sc2-vdec", sizeof(cap->driver));
	strscpy(cap->card, "Amlogic SC2 VP9/AVS2", sizeof(cap->card));
	return 0;
}

static int sc2_stateful_volatile(struct v4l2_ctrl *ctrl)
{
	struct sc2_stateful_ctx *ctx = container_of(ctrl->handler,
		struct sc2_stateful_ctx, controls);

	mutex_lock(&ctx->state_lock);
	switch (ctrl->id) {
	case V4L2_CID_MIN_BUFFERS_FOR_CAPTURE:
		ctrl->val = sc2_stateful_min_capture(ctx);
		break;
	case V4L2_CID_MPEG_VIDEO_VP9_PROFILE:
		ctrl->val = ctx->header_known && ctx->depth == 10 ?
			V4L2_MPEG_VIDEO_VP9_PROFILE_2 : V4L2_MPEG_VIDEO_VP9_PROFILE_0;
		break;
	}
	mutex_unlock(&ctx->state_lock);
	return 0;
}

static const struct v4l2_ctrl_ops sc2_stateful_control_ops = {
	.g_volatile_ctrl = sc2_stateful_volatile,
};

static int sc2_stateful_controls(struct sc2_stateful_ctx *ctx)
{
	struct v4l2_ctrl *ctrl;

	v4l2_ctrl_handler_init(&ctx->controls, 3);
	ctrl = v4l2_ctrl_new_std(&ctx->controls, &sc2_stateful_control_ops,
		V4L2_CID_MIN_BUFFERS_FOR_CAPTURE, SC2_VP9_MIN_CAPTURE,
		MESON_SC2_AVS2_MIN_CAPTURE_BUFFERS, 1, SC2_VP9_MIN_CAPTURE);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE;
	ctrl = v4l2_ctrl_new_std(&ctx->controls, NULL,
		V4L2_CID_MIN_BUFFERS_FOR_OUTPUT, 1, 1, 1, 1);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	ctrl = v4l2_ctrl_new_std_menu(&ctx->controls, &sc2_stateful_control_ops,
		V4L2_CID_MPEG_VIDEO_VP9_PROFILE, V4L2_MPEG_VIDEO_VP9_PROFILE_2,
		BIT(V4L2_MPEG_VIDEO_VP9_PROFILE_1), V4L2_MPEG_VIDEO_VP9_PROFILE_0);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE;
	ctx->fh.ctrl_handler = &ctx->controls;
	return ctx->controls.error;
}

static int sc2_stateful_queue_setup(struct vb2_queue *q, unsigned int *count,
				    unsigned int *planes, unsigned int sizes[],
				    struct device *alloc_devs[])
{
	struct sc2_stateful_ctx *ctx = vb2_get_drv_priv(q);
	u32 size;
	int ret = 0;

	mutex_lock(&ctx->state_lock);
	size = V4L2_TYPE_IS_OUTPUT(q->type) ? ctx->source_fmt.plane_fmt[0].sizeimage :
		ctx->capture_fmt.plane_fmt[0].sizeimage;
	if (*planes) {
		ret = *planes == 1 && sizes[0] >= size ? 0 : -EINVAL;
		goto unlock;
	}
	if (!V4L2_TYPE_IS_OUTPUT(q->type))
		*count = max(*count, sc2_stateful_min_capture(ctx));
	*planes = 1;
	sizes[0] = size;
unlock:
	mutex_unlock(&ctx->state_lock);
	return ret;
}

static int sc2_stateful_buf_init(struct vb2_buffer *vb)
{
	struct sc2_stateful_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct sc2_stateful_buffer *buf = sc2_stateful_buffer(to_vb2_v4l2_buffer(vb));

	INIT_LIST_HEAD(&buf->capture_entry);
	if (!V4L2_TYPE_IS_OUTPUT(vb->type)) {
		mutex_lock(&ctx->state_lock);
		list_add_tail(&buf->capture_entry, &ctx->captures);
		mutex_unlock(&ctx->state_lock);
	}
	return 0;
}

static int sc2_stateful_buf_prepare(struct vb2_buffer *vb)
{
	struct sc2_stateful_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct sc2_stateful_buffer *buf = sc2_stateful_buffer(to_vb2_v4l2_buffer(vb));
	u32 size;
	u8 depth;
	int ret = 0;

	if (vb->planes[0].data_offset)
		return -EINVAL;
	if (V4L2_TYPE_IS_OUTPUT(vb->type)) {
		size = vb2_get_plane_payload(vb, 0);
		if (vb2_plane_size(vb, 0) < ctx->source_fmt.plane_fmt[0].sizeimage ||
		    !size || size > SC2_STATEFUL_MAX_SOURCE)
			return -EINVAL;
		to_vb2_v4l2_buffer(vb)->field = V4L2_FIELD_NONE;
		return 0;
	}
	if (vb->memory != VB2_MEMORY_MMAP)
		return -EOPNOTSUPP;
	mutex_lock(&ctx->state_lock);
	/* G_FMT already describes the next sequence, but old presentations may
	 * still need their original object requeued before the source-change LAST.
	 */
	if (ctx->reconfigure_needs_off && buf->surface.cpu) {
		vb2_set_plane_payload(vb, 0,
			meson_sc2_hevc_fbc_size(buf->surface.width, buf->surface.height));
		goto unlock;
	}
	size = ctx->capture_fmt.plane_fmt[0].sizeimage;
	depth = ctx->capture_fmt.pixelformat == V4L2_PIX_FMT_AMLOGIC_FBC10 ? 10 : 8;
	if (vb2_plane_size(vb, 0) < size) {
		ret = -EINVAL;
		goto unlock;
	}
	if (!buf->export) {
		buf->export = vb->vb2_queue->mem_ops->get_dmabuf(vb,
						vb->planes[0].mem_priv, O_RDWR);
		if (IS_ERR_OR_NULL(buf->export)) {
			ret = buf->export ? PTR_ERR(buf->export) : -ENOMEM;
			buf->export = NULL;
			goto unlock;
		}
	}
	/* Never rebind a queued/DONE DPB object just because it was prepared. */
	if (!buf->surface.cpu)
		ret = meson_sc2_modern_fbc_bind(ctx->hwctx, &buf->surface,
			ctx->width, ctx->height, depth, sc2_stateful_avs2(ctx) ? 0x200 : 0x240,
			vb2_plane_vaddr(vb, 0), vb2_dma_contig_plane_dma_addr(vb, 0),
			vb2_plane_size(vb, 0), buf->export);
	else if (buf->surface.width != ctx->width || buf->surface.height != ctx->height ||
		 buf->surface.depth != depth)
		ret = -EPIPE;
	if (!ret)
		vb2_set_plane_payload(vb, 0, size);
unlock:
	mutex_unlock(&ctx->state_lock);
	return ret;
}

static void sc2_stateful_buf_cleanup(struct vb2_buffer *vb)
{
	struct sc2_stateful_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct sc2_stateful_buffer *buf = sc2_stateful_buffer(to_vb2_v4l2_buffer(vb));

	if (V4L2_TYPE_IS_OUTPUT(vb->type))
		return;
	mutex_lock(&ctx->state_lock);
	list_del(&buf->capture_entry);
	/* Codec picture pins may outlive this CAPTURE binding. Fatal DMA retains all pins. */
	meson_sc2_surface_free(ctx->node->hw, &buf->surface);
	if (buf->export)
		dma_buf_put(buf->export);
	mutex_unlock(&ctx->state_lock);
}

static void sc2_stateful_fail(struct sc2_stateful_ctx *ctx)
{
	WRITE_ONCE(ctx->aborting, true);
	WRITE_ONCE(ctx->ready, false);
	meson_sc2_hevc_abort(ctx->hwctx);
	vb2_queue_error(v4l2_m2m_get_src_vq(ctx->fh.m2m_ctx));
	vb2_queue_error(v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx));
}

static void sc2_stateful_buf_queue(struct vb2_buffer *vb)
{
	struct sc2_stateful_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct sc2_stateful_buffer *buf = sc2_stateful_buffer(to_vb2_v4l2_buffer(vb));

	mutex_lock(&ctx->state_lock);
	if (!V4L2_TYPE_IS_OUTPUT(vb->type)) {
		if (buf->user_output) {
			if (sc2_stateful_put(ctx, &buf->surface))
				sc2_stateful_fail(ctx);
			buf->user_output = false;
		}
		buf->queued = true;
		ctx->decode_blocked = false;
	}
	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(vb));
	sc2_stateful_update_ready(ctx);
	mutex_unlock(&ctx->state_lock);
}

static void sc2_stateful_retry(struct work_struct *work)
{
	struct sc2_stateful_ctx *ctx = container_of(to_delayed_work(work),
		struct sc2_stateful_ctx, retry_work);

	mutex_lock(&ctx->state_lock);
	ctx->retry_wait = false;
	sc2_stateful_update_ready(ctx);
	mutex_unlock(&ctx->state_lock);
	v4l2_m2m_try_schedule(ctx->fh.m2m_ctx);
}

/* Caller has stopped submissions; a source-change CAPTURE stop is not a seek. */
static void sc2_stateful_quiesce(struct sc2_stateful_ctx *ctx, bool capture_stop)
{
	struct sc2_stateful_picture picture;
	struct sc2_stateful_buffer *buf;
	bool preserve, restart;
	int ret;

	WRITE_ONCE(ctx->aborting, true);
	WRITE_ONCE(ctx->ready, false);
	meson_sc2_hevc_abort(ctx->hwctx);
	cancel_delayed_work_sync(&ctx->retry_work);
	flush_work(&ctx->work);
	mutex_lock(&ctx->state_lock);
	preserve = capture_stop && !sc2_stateful_avs2(ctx) &&
		ctx->reconfigure && ctx->reconfigure_needs_off && ctx->last_sent;
	restart = capture_stop && sc2_stateful_avs2(ctx) &&
		ctx->avs2_replay_pending;
	if (!meson_sc2_hevc_context_fatal(ctx->hwctx)) {
		if (ctx->pending_valid)
			sc2_stateful_put(ctx, ctx->pending.surface);
		while (!sc2_stateful_dequeue(ctx, true, &picture))
			sc2_stateful_put(ctx, picture.surface);
		list_for_each_entry(buf, &ctx->captures, capture_entry)
			if (buf->user_output)
				sc2_stateful_put(ctx, &buf->surface);
	}
	if (preserve)
		ret = meson_sc2_vp9_capture_restart(ctx->hwctx);
	else if (restart)
		ret = meson_sc2_avs2_capture_restart(ctx->hwctx);
	else
		ret = meson_sc2_hevc_context_reset(ctx->hwctx);
	ctx->pending_valid = false;
	ctx->poll_output = false;
	ctx->retry_wait = false;
	ctx->decode_blocked = false;
	list_for_each_entry(buf, &ctx->captures, capture_entry) {
		buf->user_output = false;
		if (!preserve)
			buf->surface.valid = false;
	}
	if (ret)
		sc2_stateful_fail(ctx);
	mutex_unlock(&ctx->state_lock);
}

static void sc2_stateful_return_queue(struct sc2_stateful_ctx *ctx,
				      enum v4l2_buf_type type,
				      enum vb2_buffer_state state)
{
	struct vb2_v4l2_buffer *vb;

	while ((vb = V4L2_TYPE_IS_OUTPUT(type) ?
		v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx) :
		v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx))) {
		if (!V4L2_TYPE_IS_OUTPUT(type))
			sc2_stateful_buffer(vb)->queued = false;
		v4l2_m2m_buf_done(vb, state);
	}
}

static int sc2_stateful_start(struct vb2_queue *q, unsigned int count)
{
	struct sc2_stateful_ctx *ctx = vb2_get_drv_priv(q);
	struct sc2_stateful_buffer *buf;
	int ret = 0;

	mutex_lock(&ctx->state_lock);
	if (ctx->node->removing || meson_sc2_hevc_context_fatal(ctx->hwctx)) {
		ret = -EIO;
		goto unlock;
	}
	if (V4L2_TYPE_IS_OUTPUT(q->type)) {
		ctx->output_streaming = true;
		ctx->source_sequence = 0;
		ctx->draining = false;
		ctx->drain_ready = false;
		ctx->eos = false;
		v4l2_m2m_clear_state(ctx->fh.m2m_ctx);
	} else {
		if (vb2_get_num_buffers(q) < sc2_stateful_min_capture(ctx)) {
			ret = -EINVAL;
			goto unlock;
		}
		if (ctx->reconfigure_needs_off) {
			ret = -EPIPE;
			goto unlock;
		}
		list_for_each_entry(buf, &ctx->captures, capture_entry)
			if (buf->queued && (buf->surface.width != ctx->width ||
			    buf->surface.height != ctx->height || buf->surface.depth !=
			    (ctx->capture_fmt.pixelformat == V4L2_PIX_FMT_AMLOGIC_FBC10 ? 10 : 8))) {
				ret = -EPIPE;
				goto unlock;
			}
		ctx->capture_streaming = true;
		ctx->capture_sequence = 0;
		ctx->reconfigure = false;
		ctx->drain_exhausted = false;
		ctx->last_sent = false;
	}
	WRITE_ONCE(ctx->aborting, false);
	sc2_stateful_update_ready(ctx);
unlock:
	mutex_unlock(&ctx->state_lock);
	if (ret)
		sc2_stateful_return_queue(ctx, q->type, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void sc2_stateful_stop(struct vb2_queue *q)
{
	struct sc2_stateful_ctx *ctx = vb2_get_drv_priv(q);

	sc2_stateful_quiesce(ctx, !V4L2_TYPE_IS_OUTPUT(q->type));
	mutex_lock(&ctx->state_lock);
	sc2_stateful_return_queue(ctx, q->type, VB2_BUF_STATE_ERROR);
	if (V4L2_TYPE_IS_OUTPUT(q->type)) {
		ctx->output_streaming = false;
		ctx->header_known = false;
		ctx->reconfigure = false;
		ctx->reconfigure_needs_off = false;
		ctx->avs2_replay_pending = false;
		ctx->draining = false;
		ctx->drain_ready = false;
		ctx->drain_tail = NULL;
		ctx->eos = false;
		v4l2_m2m_clear_state(ctx->fh.m2m_ctx);
	} else {
		ctx->capture_streaming = false;
		ctx->reconfigure_needs_off = false;
		ctx->last_sent = false;
		ctx->drain_exhausted = false;
	}
	if (!meson_sc2_hevc_context_fatal(ctx->hwctx))
		WRITE_ONCE(ctx->aborting, false);
	sc2_stateful_update_ready(ctx);
	mutex_unlock(&ctx->state_lock);
}

static const struct vb2_ops sc2_stateful_queue_ops = {
	.queue_setup = sc2_stateful_queue_setup,
	.buf_init = sc2_stateful_buf_init,
	.buf_prepare = sc2_stateful_buf_prepare,
	.buf_cleanup = sc2_stateful_buf_cleanup,
	.buf_queue = sc2_stateful_buf_queue,
	.start_streaming = sc2_stateful_start,
	.stop_streaming = sc2_stateful_stop,
};

static int sc2_stateful_queue_init(void *priv, struct vb2_queue *src,
				   struct vb2_queue *dst)
{
	struct sc2_stateful_ctx *ctx = priv;
	struct vb2_queue *queues[] = { src, dst };
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(queues); i++) {
		struct vb2_queue *q = queues[i];

		q->type = i ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE :
			V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
		q->io_modes = VB2_MMAP | (i ? 0 : VB2_DMABUF);
		q->drv_priv = ctx;
		q->ops = &sc2_stateful_queue_ops;
		q->mem_ops = &vb2_dma_contig_memops;
		q->buf_struct_size = sizeof(struct sc2_stateful_buffer);
		q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
		q->lock = &ctx->node->lock;
		q->dev = ctx->node->dev;
		q->min_reqbufs_allocation = i ? sc2_stateful_min_capture(ctx) : 1;
		ret = vb2_queue_init(q);
		if (ret)
			return ret;
	}
	return 0;
}

static int sc2_stateful_decode(struct sc2_stateful_ctx *ctx,
			       struct vb2_v4l2_buffer *src,
			       struct sc2_stateful_buffer *dst,
			       struct sc2_stateful_picture *picture)
{
	struct meson_sc2_hevc_job output = {};
	struct dma_buf *input = src->vb2_buf.planes[0].dbuf;
	const void *data;
	size_t size = vb2_get_plane_payload(&src->vb2_buf, 0);
	int ret;

	if (input) {
		ret = dma_buf_begin_cpu_access(input, DMA_TO_DEVICE);
		if (ret)
			return ret;
	}
	data = vb2_plane_vaddr(&src->vb2_buf, 0);
	if (!data) {
		ret = -EFAULT;
		goto end_access;
	}
	if (dst) {
		output.destination = &dst->surface;
		output.output_dma = vb2_dma_contig_plane_dma_addr(&dst->m2m.vb.vb2_buf, 0);
		output.output_size = ctx->capture_fmt.plane_fmt[0].sizeimage;
		output.output_format = ctx->capture_fmt.pixelformat;
		output.output_buffer = dst->export;
	}
	if (sc2_stateful_avs2(ctx)) {
		struct meson_sc2_avs2_job job = {
			.data = data, .size = size, .timestamp = src->vb2_buf.timestamp,
			.output = output,
		};
		struct meson_sc2_avs2_result result = {};

		ret = meson_sc2_avs2_decode(ctx->hwctx, &job, &result);
		*picture = (struct sc2_stateful_picture) {
			.surface = result.surface, .timestamp = result.timestamp,
			.width = result.width, .height = result.height, .depth = result.depth,
		};
	} else {
		struct meson_sc2_vp9_job job = {
			.data = data, .size = size, .timestamp = src->vb2_buf.timestamp,
			.output = output,
		};
		struct meson_sc2_vp9_result result = {};

		/* Exactly one raw VP9 frame per OUTPUT AU; split superframes in userspace. */
		ret = meson_sc2_vp9_decode(ctx->hwctx, &job, &result);
		*picture = (struct sc2_stateful_picture) {
			.surface = result.surface, .timestamp = result.timestamp,
			.width = result.width, .height = result.height, .depth = result.depth,
		};
	}
end_access:
	if (input)
		dma_buf_end_cpu_access(input, DMA_TO_DEVICE);
	return ret;
}

static bool sc2_stateful_present(struct sc2_stateful_ctx *ctx)
{
	struct sc2_stateful_buffer *buf = sc2_stateful_find(ctx, ctx->pending.surface);
	struct vb2_v4l2_buffer *vb;

	if (!buf) {
		sc2_stateful_fail(ctx);
		return false;
	}
	/* show_existing may name a buffer still DONE or being scanned out. */
	if (!buf->queued || buf->user_output)
		return false;
	vb = &buf->m2m.vb;
	v4l2_m2m_dst_buf_remove_by_buf(ctx->fh.m2m_ctx, vb);
	buf->queued = false;
	buf->user_output = true;
	vb->vb2_buf.timestamp = ctx->pending.timestamp;
	vb->sequence = ctx->capture_sequence++;
	vb->field = V4L2_FIELD_NONE;
	vb->flags &= ~(V4L2_BUF_FLAG_LAST | V4L2_BUF_FLAG_ERROR |
		V4L2_BUF_FLAG_KEYFRAME | V4L2_BUF_FLAG_PFRAME | V4L2_BUF_FLAG_BFRAME);
	vb2_set_plane_payload(&vb->vb2_buf, 0,
		meson_sc2_hevc_fbc_size(buf->surface.width, buf->surface.height));
	ctx->pending_valid = false;
	ctx->decode_blocked = false;
	v4l2_m2m_buf_done(vb, VB2_BUF_STATE_DONE);
	return true;
}

static bool sc2_stateful_last(struct sc2_stateful_ctx *ctx, bool eos)
{
	struct vb2_v4l2_buffer *vb = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

	if (!vb)
		return false;
	sc2_stateful_buffer(vb)->queued = false;
	vb->flags &= ~V4L2_BUF_FLAG_ERROR;
	vb->flags |= V4L2_BUF_FLAG_LAST;
	vb->field = V4L2_FIELD_NONE;
	vb->sequence = ctx->capture_sequence++;
	vb->vb2_buf.timestamp = 0;
	vb2_set_plane_payload(&vb->vb2_buf, 0, 0);
	ctx->last_sent = true;
	v4l2_m2m_buf_done(vb, VB2_BUF_STATE_DONE);
	if (eos) {
		const struct v4l2_event event = { .type = V4L2_EVENT_EOS };

		ctx->eos = true;
		ctx->draining = false;
		v4l2_m2m_mark_stopped(ctx->fh.m2m_ctx);
		v4l2_event_queue_fh(&ctx->fh, &event);
	}
	return true;
}

static void sc2_stateful_worker(struct work_struct *work)
{
	struct sc2_stateful_ctx *ctx = container_of(work, struct sc2_stateful_ctx, work);
	struct sc2_stateful_picture picture = {};
	struct sc2_stateful_buffer *dst = NULL;
	struct vb2_v4l2_buffer *src;
	bool drain;
	int ret;

	mutex_lock(&ctx->state_lock);
	if (READ_ONCE(ctx->aborting) || READ_ONCE(ctx->node->removing))
		goto finish;
	if (ctx->pending_valid) {
		sc2_stateful_present(ctx);
		goto finish;
	}
	drain = ctx->reconfigure || (ctx->draining && ctx->drain_ready);
	if (ctx->capture_streaming && (ctx->poll_output || (drain && !ctx->drain_exhausted))) {
		ret = sc2_stateful_dequeue(ctx, drain, &ctx->pending);
		if (!ret) {
			ctx->pending_valid = true;
			sc2_stateful_present(ctx);
			goto finish;
		}
		if (ret != -EAGAIN) {
			sc2_stateful_fail(ctx);
			goto finish;
		}
		ctx->poll_output = false;
		if (drain)
			ctx->drain_exhausted = true;
	}
	if (drain) {
		if (ctx->capture_streaming && ctx->drain_exhausted && !ctx->last_sent)
			sc2_stateful_last(ctx, !ctx->reconfigure);
		goto finish;
	}
	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	if (!src || ctx->decode_blocked || ctx->retry_wait)
		goto finish;
	if (ctx->header_known) {
		if (!ctx->capture_streaming)
			goto finish;
		dst = sc2_stateful_destination(ctx);
		if (!dst)
			goto finish;
	}
	ret = sc2_stateful_decode(ctx, src, dst, &picture);
	/* Record the obligation even if CAPTURE stop raced the probe return. */
	if (sc2_stateful_avs2(ctx) && ret == -EPIPE)
		ctx->avs2_replay_pending = true;
	if (READ_ONCE(ctx->aborting))
		goto finish;
	if (meson_sc2_hevc_context_fatal(ctx->hwctx)) {
		sc2_stateful_fail(ctx);
		goto finish;
	}
	if (ret == -EPIPE) {
		struct v4l2_event event = {
			.type = V4L2_EVENT_SOURCE_CHANGE,
			.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
		};

		if (picture.width < 64 || picture.width > SC2_STATEFUL_MAX_WIDTH ||
		    picture.height < 64 || picture.height > SC2_STATEFUL_MAX_HEIGHT ||
		    (picture.depth != 8 && picture.depth != 10)) {
			ret = -EINVAL;
		} else {
			ctx->width = picture.width;
			ctx->height = picture.height;
			ctx->depth = picture.depth;
			ctx->header_known = true;
			sc2_stateful_capture_format(ctx, &ctx->capture_fmt, 0);
			ctx->reconfigure = true;
			ctx->reconfigure_needs_off = ctx->capture_streaming;
			ctx->last_sent = false;
			ctx->drain_exhausted = false;
			ctx->poll_output = true;
			v4l2_event_queue_fh(&ctx->fh, &event);
			/* Keep this exact source AU queued until CAPTURE is reconfigured. */
			goto finish;
		}
	}
	if (ret == -EBUSY) {
		/* Another native node owns DOS; do not consume or corrupt this AU. */
		ctx->retry_wait = true;
		queue_delayed_work(ctx->node->workqueue, &ctx->retry_work,
				   msecs_to_jiffies(10));
		goto finish;
	}
	if (ret == -EAGAIN) {
		ctx->decode_blocked = true;
		goto finish;
	}
	v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	if (sc2_stateful_avs2(ctx)) {
		meson_sc2_avs2_source_consumed(ctx->hwctx, !!picture.surface, ret);
		ctx->avs2_replay_pending = false;
	}
	src->sequence = ctx->source_sequence++;
	if (ctx->draining && src == ctx->drain_tail) {
		ctx->drain_ready = true;
		ctx->drain_tail = NULL;
		ctx->drain_exhausted = false;
	}
	v4l2_m2m_buf_done(src, ret ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
	/* Decode and presentation are independent, including hidden pictures. */
	ctx->poll_output = true;
finish:
	sc2_stateful_update_ready(ctx);
	mutex_unlock(&ctx->state_lock);
	v4l2_m2m_job_finish(ctx->node->m2m, ctx->fh.m2m_ctx);
}

static void sc2_stateful_run(void *priv)
{
	struct sc2_stateful_ctx *ctx = priv;

	queue_work(ctx->node->workqueue, &ctx->work);
}

static int sc2_stateful_ready(void *priv)
{
	struct sc2_stateful_ctx *ctx = priv;

	return READ_ONCE(ctx->ready) && !READ_ONCE(ctx->aborting) &&
		!READ_ONCE(ctx->node->removing);
}

static void sc2_stateful_abort(void *priv)
{
	struct sc2_stateful_ctx *ctx = priv;

	WRITE_ONCE(ctx->aborting, true);
	WRITE_ONCE(ctx->ready, false);
	meson_sc2_hevc_abort(ctx->hwctx);
}

static const struct v4l2_m2m_ops sc2_stateful_m2m_ops = {
	.device_run = sc2_stateful_run,
	.job_ready = sc2_stateful_ready,
	.job_abort = sc2_stateful_abort,
};

static int sc2_stateful_decoder_cmd(struct file *file, void *priv,
				    struct v4l2_decoder_cmd *cmd)
{
	struct sc2_stateful_ctx *ctx = sc2_stateful_file(file);
	int ret;

	ret = v4l2_m2m_ioctl_try_decoder_cmd(file, priv, cmd);
	if (ret)
		return ret;
	mutex_lock(&ctx->state_lock);
	if (ctx->aborting || ctx->node->removing) {
		ret = -EIO;
		goto unlock;
	}
	if (cmd->cmd == V4L2_DEC_CMD_STOP) {
		if (ctx->draining) {
			ret = -EBUSY;
			goto unlock;
		}
		if (!ctx->output_streaming || ctx->eos)
			goto unlock;
		ctx->draining = true;
		ctx->drain_tail = v4l2_m2m_last_src_buf(ctx->fh.m2m_ctx);
		ctx->drain_ready = !ctx->drain_tail;
		ctx->drain_exhausted = false;
		ctx->last_sent = false;
		ctx->fh.m2m_ctx->is_draining = true;
		ctx->fh.m2m_ctx->last_src_buf = ctx->drain_tail;
	} else {
		if (ctx->draining) {
			ret = -EBUSY;
			goto unlock;
		}
		if (ctx->eos) {
			/* START resumes the drained session; STREAMOFF performs a seek/reset. */
			ctx->eos = false;
			ctx->drain_ready = false;
			ctx->drain_exhausted = false;
			ctx->last_sent = false;
			v4l2_m2m_clear_state(ctx->fh.m2m_ctx);
			vb2_clear_last_buffer_dequeued(v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx));
		}
	}
	sc2_stateful_update_ready(ctx);
unlock:
	mutex_unlock(&ctx->state_lock);
	if (!ret)
		v4l2_m2m_try_schedule(ctx->fh.m2m_ctx);
	return ret;
}

static int sc2_stateful_subscribe(struct v4l2_fh *fh,
				  const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subscribe(fh, sub);
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 2, NULL);
	default:
		return v4l2_ctrl_subscribe_event(fh, sub);
	}
}

static const struct v4l2_ioctl_ops sc2_stateful_ioctl_ops = {
	.vidioc_querycap = sc2_stateful_querycap,
	.vidioc_enum_fmt_vid_cap = sc2_stateful_enum_fmt,
	.vidioc_enum_fmt_vid_out = sc2_stateful_enum_fmt,
	.vidioc_enum_framesizes = sc2_stateful_framesizes,
	.vidioc_enum_frameintervals = sc2_stateful_frameintervals,
	.vidioc_g_fmt_vid_cap_mplane = sc2_stateful_get_fmt,
	.vidioc_g_fmt_vid_out_mplane = sc2_stateful_get_fmt,
	.vidioc_try_fmt_vid_cap_mplane = sc2_stateful_try_fmt,
	.vidioc_try_fmt_vid_out_mplane = sc2_stateful_try_fmt,
	.vidioc_s_fmt_vid_cap_mplane = sc2_stateful_set_fmt,
	.vidioc_s_fmt_vid_out_mplane = sc2_stateful_set_fmt,
	.vidioc_g_selection = sc2_stateful_selection,
	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
	.vidioc_try_decoder_cmd = v4l2_m2m_ioctl_try_decoder_cmd,
	.vidioc_decoder_cmd = sc2_stateful_decoder_cmd,
	.vidioc_subscribe_event = sc2_stateful_subscribe,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static int sc2_stateful_open(struct file *file)
{
	struct meson_sc2_stateful *node = video_drvdata(file);
	struct sc2_stateful_ctx *ctx;
	int ret;

	if (mutex_lock_interruptible(&node->lock))
		return -ERESTARTSYS;
	if (node->removing) {
		ret = -ENODEV;
		goto unlock;
	}
	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		ret = -ENOMEM;
		goto unlock;
	}
	ctx->node = node;
	ctx->width = 1920;
	ctx->height = 1080;
	ctx->depth = 8;
	mutex_init(&ctx->state_lock);
	INIT_LIST_HEAD(&ctx->captures);
	INIT_WORK(&ctx->work, sc2_stateful_worker);
	INIT_DELAYED_WORK(&ctx->retry_work, sc2_stateful_retry);
	v4l2_fh_init(&ctx->fh, &node->video);
	ctx->source_fmt = (struct v4l2_pix_format_mplane) {
		.width = ctx->width, .height = ctx->height,
		.pixelformat = V4L2_PIX_FMT_VP9, .field = V4L2_FIELD_NONE,
		.num_planes = 1, .colorspace = V4L2_COLORSPACE_REC709,
		.plane_fmt[0].sizeimage = SC2_STATEFUL_SOURCE_SIZE,
	};
	sc2_stateful_capture_format(ctx, &ctx->capture_fmt, V4L2_PIX_FMT_AMLOGIC_FBC8);
	ret = sc2_stateful_controls(ctx);
	if (ret)
		goto free_controls;
	ret = meson_sc2_hevc_context_create(node->hw, &ctx->hwctx);
	if (ret)
		goto free_controls;
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(node->m2m, ctx, sc2_stateful_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto free_hw;
	}
	/* Header parsing needs OUTPUT only; presentation/drain can need no source. */
	ctx->fh.m2m_ctx->ignore_cap_streaming = true;
	v4l2_m2m_set_src_buffered(ctx->fh.m2m_ctx, true);
	v4l2_m2m_set_dst_buffered(ctx->fh.m2m_ctx, true);
	v4l2_fh_add(&ctx->fh, file);
	list_add_tail(&ctx->entry, &node->contexts);
	mutex_unlock(&node->lock);
	return 0;
free_hw:
	meson_sc2_hevc_context_destroy(ctx->hwctx);
free_controls:
	v4l2_ctrl_handler_free(&ctx->controls);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
unlock:
	mutex_unlock(&node->lock);
	return ret;
}

static int sc2_stateful_release(struct file *file)
{
	struct sc2_stateful_ctx *ctx = sc2_stateful_file(file);
	struct meson_sc2_stateful *node = ctx->node;

	mutex_lock(&node->lock);
	sc2_stateful_quiesce(ctx, false);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	list_del(&ctx->entry);
	v4l2_fh_del(&ctx->fh, file);
	v4l2_ctrl_handler_free(&ctx->controls);
	v4l2_fh_exit(&ctx->fh);
	meson_sc2_hevc_context_destroy(ctx->hwctx);
	kfree(ctx);
	mutex_unlock(&node->lock);
	return 0;
}

static const struct v4l2_file_operations sc2_stateful_fops = {
	.owner = THIS_MODULE,
	.open = sc2_stateful_open,
	.release = sc2_stateful_release,
	.unlocked_ioctl = video_ioctl2,
	.poll = v4l2_m2m_fop_poll,
	.mmap = v4l2_m2m_fop_mmap,
};

static void sc2_stateful_device_release(struct v4l2_device *v4l2)
{
	struct meson_sc2_stateful *node = container_of(v4l2,
		struct meson_sc2_stateful, v4l2);
	struct v4l2_device *owner = node->owner;

	v4l2_m2m_release(node->m2m);
	destroy_workqueue(node->workqueue);
	kfree(node);
	/* Only the existing platform frontend owns and ultimately destroys hw. */
	v4l2_device_put(owner);
}

int meson_sc2_stateful_register(struct device *dev, struct meson_sc2_hw *hw,
			       struct v4l2_device *owner,
			       struct meson_sc2_stateful **result)
{
	struct meson_sc2_stateful *node;
	int ret;

	if (!dev || !hw || !owner || !result)
		return -EINVAL;
	*result = NULL;
	node = kzalloc(sizeof(*node), GFP_KERNEL);
	if (!node)
		return -ENOMEM;
	node->dev = dev;
	node->hw = hw;
	node->owner = owner;
	mutex_init(&node->lock);
	INIT_LIST_HEAD(&node->contexts);
	node->workqueue = alloc_ordered_workqueue(SC2_STATEFUL_NAME, WQ_MEM_RECLAIM);
	if (!node->workqueue) {
		ret = -ENOMEM;
		goto free_node;
	}
	node->m2m = v4l2_m2m_init(&sc2_stateful_m2m_ops);
	if (IS_ERR(node->m2m)) {
		ret = PTR_ERR(node->m2m);
		goto free_workqueue;
	}
	ret = v4l2_device_register(dev, &node->v4l2);
	if (ret)
		goto free_m2m;
	v4l2_device_get(owner);
	node->v4l2.release = sc2_stateful_device_release;
	strscpy(node->video.name, SC2_STATEFUL_NAME, sizeof(node->video.name));
	node->video.v4l2_dev = &node->v4l2;
	node->video.fops = &sc2_stateful_fops;
	node->video.ioctl_ops = &sc2_stateful_ioctl_ops;
	node->video.lock = &node->lock;
	node->video.release = video_device_release_empty;
	node->video.vfl_dir = VFL_DIR_M2M;
	node->video.device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	video_set_drvdata(&node->video, node);
	ret = video_register_device(&node->video, VFL_TYPE_VIDEO, -1);
	if (ret) {
		v4l2_device_unregister(&node->v4l2);
		v4l2_device_put(&node->v4l2);
		return ret;
	}
	*result = node;
	return 0;
free_m2m:
	v4l2_m2m_release(node->m2m);
free_workqueue:
	destroy_workqueue(node->workqueue);
free_node:
	kfree(node);
	return ret;
}

void meson_sc2_stateful_unregister(struct meson_sc2_stateful *node)
{
	struct sc2_stateful_ctx *ctx;

	if (!node)
		return;
	mutex_lock(&node->lock);
	WRITE_ONCE(node->removing, true);
	list_for_each_entry(ctx, &node->contexts, entry) {
		sc2_stateful_quiesce(ctx, false);
		vb2_queue_error(v4l2_m2m_get_src_vq(ctx->fh.m2m_ctx));
		vb2_queue_error(v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx));
	}
	mutex_unlock(&node->lock);
	video_unregister_device(&node->video);
	v4l2_device_unregister(&node->v4l2);
	v4l2_device_put(&node->v4l2);
}
