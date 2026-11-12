// SPDX-License-Identifier: GPL-2.0-only
/* Amlogic SC2 stateless HEVC/AV1 Request decoder, one shared engine. */
#include <linux/dma-buf.h>
#include <linux/fcntl.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <media/media-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "meson_sc2_hw.h"
#include "meson_sc2_stateful.h"
#include "hevc_request_syntax.h"
#include "av1_request_syntax.h"

#define SC2_NAME "meson-sc2-hevc"
#define SC2_MAX_WIDTH 4096U
#define SC2_MAX_HEIGHT 2304U
#define SC2_MAX_SLICES 600U
#define SC2_MAX_ENTRIES 65536U
#define SC2_SOURCE_SIZE (8U * 1024U * 1024U)
#define SC2_REQUEST_DONE 0

struct sc2_dev {
	struct device *dev;
	struct meson_sc2_hw *hw;
	struct mutex lock;
	struct v4l2_device v4l2;
	struct media_device media;
	struct video_device video;
	struct v4l2_m2m_dev *m2m;
	struct workqueue_struct *workqueue;
	struct list_head contexts;
	struct meson_sc2_stateful *stateful;
	bool removing;
};

struct sc2_buffer {
	struct v4l2_m2m_buffer m2m;
	struct list_head capture_entry;
	struct meson_sc2_surface surface;
	struct dma_buf *output_buffer;
	u64 timestamp;
	u64 epoch;
	unsigned long request_flags;
};

struct sc2_ctx {
	struct v4l2_fh fh;
	struct list_head entry;
	struct sc2_dev *dev;
	struct v4l2_ctrl_handler controls;
	struct meson_sc2_hevc_context *hwctx;
	/* Never take the ioctl lock in the worker: STREAMOFF waits for it. */
	struct mutex state_lock;
	struct work_struct work;
	struct list_head captures;
	struct v4l2_pix_format_mplane source_fmt;
	struct v4l2_pix_format_mplane capture_fmt;
	u64 epoch;
	u32 source_sequence;
	u32 capture_sequence;
	bool aborting;
	struct meson_hevc_syntax_slice *slices;
	struct v4l2_ctrl_hevc_slice_params *slice_params;
};

static struct sc2_ctx *sc2_file_ctx(struct file *file)
{
	return container_of(file->private_data, struct sc2_ctx, fh);
}

static struct sc2_buffer *sc2_buffer(struct vb2_v4l2_buffer *vbuf)
{
	return container_of(vbuf, struct sc2_buffer, m2m.vb);
}

static struct v4l2_pix_format_mplane *sc2_format(struct sc2_ctx *ctx,
					      enum v4l2_buf_type type)
{
	return V4L2_TYPE_IS_OUTPUT(type) ? &ctx->source_fmt : &ctx->capture_fmt;
}

static void sc2_capture_format(struct sc2_ctx *ctx,
			       struct v4l2_pix_format_mplane *pix)
{
	bool fbc = pix->pixelformat == V4L2_PIX_FMT_AMLOGIC_FBC10 &&
		   ctx->source_fmt.pixelformat == V4L2_PIX_FMT_HEVC_SLICE;

	pix->pixelformat = fbc ? V4L2_PIX_FMT_AMLOGIC_FBC10 :
		pix->pixelformat == V4L2_PIX_FMT_NV21 ? V4L2_PIX_FMT_NV21 :
		V4L2_PIX_FMT_NV12;
	pix->width = ALIGN(ctx->source_fmt.width, 64);
	pix->height = ALIGN(ctx->source_fmt.height, 64);
	pix->field = V4L2_FIELD_NONE;
	pix->num_planes = 1;
	pix->plane_fmt[0].bytesperline = fbc ? 0 : pix->width;
	pix->plane_fmt[0].sizeimage = fbc ?
		meson_sc2_hevc_fbc_size(pix->width, pix->height) :
		pix->width * pix->height * 3 / 2;
	pix->colorspace = ctx->source_fmt.colorspace;
	pix->xfer_func = ctx->source_fmt.xfer_func;
	pix->ycbcr_enc = ctx->source_fmt.ycbcr_enc;
	pix->quantization = ctx->source_fmt.quantization;
	memset(pix->plane_fmt[0].reserved, 0,
	       sizeof(pix->plane_fmt[0].reserved));
	memset(pix->reserved, 0, sizeof(pix->reserved));
}

static int sc2_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct sc2_ctx *ctx = sc2_file_ctx(file);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	u32 size = pix->plane_fmt[0].sizeimage;
	u32 fourcc = pix->pixelformat;

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		memset(pix, 0, sizeof(*pix));
		pix->pixelformat = fourcc;
		sc2_capture_format(ctx, pix);
		return 0;
	}
	if (f->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return -EINVAL;
	if (pix->pixelformat != V4L2_PIX_FMT_AV1_FRAME)
		pix->pixelformat = V4L2_PIX_FMT_HEVC_SLICE;
	pix->width = clamp_t(u32, ALIGN(pix->width, 2), 64, SC2_MAX_WIDTH);
	pix->height = clamp_t(u32, ALIGN(pix->height, 2), 64, SC2_MAX_HEIGHT);
	if (pix->pixelformat == V4L2_PIX_FMT_AV1_FRAME) {
		pix->width = min(pix->width, 1920U);
		pix->height = min(pix->height, 1088U);
	}
	pix->field = V4L2_FIELD_NONE;
	pix->num_planes = 1;
	memset(pix->plane_fmt, 0, sizeof(pix->plane_fmt));
	/* The default is not a minimum for an explicit userspace capacity. */
	pix->plane_fmt[0].sizeimage = size ?
		clamp_t(u32, size, 4U * 1024U, 32U * 1024U * 1024U) :
		SC2_SOURCE_SIZE;
	if (!pix->colorspace)
		pix->colorspace = V4L2_COLORSPACE_REC709;
	memset(pix->reserved, 0, sizeof(pix->reserved));
	return 0;
}

static int sc2_get_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct sc2_ctx *ctx = sc2_file_ctx(file);

	f->fmt.pix_mp = *sc2_format(ctx, f->type);
	return 0;
}

static int sc2_set_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct sc2_ctx *ctx = sc2_file_ctx(file);
	struct vb2_queue *queue = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	int ret;

	if (vb2_is_busy(queue))
		return -EBUSY;
	if (V4L2_TYPE_IS_OUTPUT(f->type) &&
	    vb2_is_busy(v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx)))
		return -EBUSY;
	ret = sc2_try_fmt(file, priv, f);
	if (ret)
		return ret;
	*sc2_format(ctx, f->type) = f->fmt.pix_mp;
	if (V4L2_TYPE_IS_OUTPUT(f->type))
		sc2_capture_format(ctx, &ctx->capture_fmt);
	return 0;
}

static int sc2_enum_fmt(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	struct sc2_ctx *ctx = sc2_file_ctx(file);
	bool output = V4L2_TYPE_IS_OUTPUT(f->type);
	bool hevc = ctx->source_fmt.pixelformat == V4L2_PIX_FMT_HEVC_SLICE;
	unsigned int last = !output && hevc ? 2 : 1;

	if (f->index > last)
		return -EINVAL;
	if (output) {
		f->pixelformat = f->index ? V4L2_PIX_FMT_AV1_FRAME : V4L2_PIX_FMT_HEVC_SLICE;
		f->flags = V4L2_FMT_FLAG_COMPRESSED;
	} else {
		if (!f->index)
			f->pixelformat = V4L2_PIX_FMT_NV12;
		else if (f->index == 1 && hevc)
			f->pixelformat = V4L2_PIX_FMT_AMLOGIC_FBC10;
		else
			f->pixelformat = V4L2_PIX_FMT_NV21;
	}
	return 0;
}

static int sc2_enum_framesizes(struct file *file, void *priv,
			       struct v4l2_frmsizeenum *f)
{
	if (f->index || (f->pixel_format != V4L2_PIX_FMT_HEVC_SLICE &&
			f->pixel_format != V4L2_PIX_FMT_AV1_FRAME &&
			f->pixel_format != V4L2_PIX_FMT_NV12 &&
			f->pixel_format != V4L2_PIX_FMT_NV21 &&
			f->pixel_format != V4L2_PIX_FMT_AMLOGIC_FBC10))
		return -EINVAL;
	f->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	f->stepwise.min_width = 64;
	f->stepwise.max_width = f->pixel_format == V4L2_PIX_FMT_AV1_FRAME ?
		1920 : SC2_MAX_WIDTH;
	f->stepwise.step_width = f->pixel_format == V4L2_PIX_FMT_NV12 ||
		f->pixel_format == V4L2_PIX_FMT_NV21 ||
		f->pixel_format == V4L2_PIX_FMT_AMLOGIC_FBC10 ? 64 : 2;
	f->stepwise.min_height = 64;
	f->stepwise.max_height = f->pixel_format == V4L2_PIX_FMT_AV1_FRAME ?
		1088 : SC2_MAX_HEIGHT;
	f->stepwise.step_height = f->pixel_format == V4L2_PIX_FMT_NV12 ||
		f->pixel_format == V4L2_PIX_FMT_NV21 ||
		f->pixel_format == V4L2_PIX_FMT_AMLOGIC_FBC10 ? 64 : 2;
	return 0;
}

static int sc2_get_selection(struct file *file, void *priv,
			      struct v4l2_selection *s)
{
	struct sc2_ctx *ctx = sc2_file_ctx(file);

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	switch (s->target) {
	case V4L2_SEL_TGT_COMPOSE:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
		s->r = (struct v4l2_rect) { 0, 0, ctx->source_fmt.width,
					   ctx->source_fmt.height };
		return 0;
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	case V4L2_SEL_TGT_COMPOSE_PADDED:
		s->r = (struct v4l2_rect) { 0, 0, ctx->capture_fmt.width,
					   ctx->capture_fmt.height };
		return 0;
	default:
		return -EINVAL;
	}
}

static int sc2_querycap(struct file *file, void *priv,
			struct v4l2_capability *cap)
{
	strscpy(cap->driver, SC2_NAME, sizeof(cap->driver));
	strscpy(cap->card, "Amlogic SC2 HEVC/AV1 Request", sizeof(cap->card));
	return 0;
}

static int sc2_try_control(struct v4l2_ctrl *ctrl)
{
	if (ctrl->id == V4L2_CID_STATELESS_HEVC_SPS) {
		const struct v4l2_ctrl_hevc_sps *sps = ctrl->p_new.p_hevc_sps;

		if (sps->chroma_format_idc != 1 ||
		    (sps->bit_depth_luma_minus8 != 0 && sps->bit_depth_luma_minus8 != 2) ||
		    sps->bit_depth_chroma_minus8 != sps->bit_depth_luma_minus8 ||
		    sps->pic_width_in_luma_samples < 64 ||
		    sps->pic_width_in_luma_samples > SC2_MAX_WIDTH ||
		    sps->pic_height_in_luma_samples < 64 ||
		    sps->pic_height_in_luma_samples > SC2_MAX_HEIGHT ||
		    (sps->pic_width_in_luma_samples & 1) ||
		    (sps->pic_height_in_luma_samples & 1))
			return -EINVAL;
	}
	return 0;
}

static const struct v4l2_ctrl_ops sc2_ctrl_ops = {
	.try_ctrl = sc2_try_control,
};

static int sc2_controls_init(struct sc2_ctx *ctx)
{
	static const struct v4l2_ctrl_config configs[] = {
		{ .id = V4L2_CID_STATELESS_AV1_SEQUENCE },
		{ .id = V4L2_CID_STATELESS_AV1_FRAME },
		{ .id = V4L2_CID_STATELESS_AV1_TILE_GROUP_ENTRY,
		  .dims = { 256 }, .flags = V4L2_CTRL_FLAG_DYNAMIC_ARRAY },
		{ .id = V4L2_CID_STATELESS_AV1_FILM_GRAIN },
		{ .id = V4L2_CID_STATELESS_HEVC_SPS, .ops = &sc2_ctrl_ops },
		{ .id = V4L2_CID_STATELESS_HEVC_PPS },
		{ .id = V4L2_CID_STATELESS_HEVC_SCALING_MATRIX },
		{ .id = V4L2_CID_STATELESS_HEVC_DECODE_PARAMS },
		{ .id = V4L2_CID_STATELESS_HEVC_SLICE_PARAMS,
		  .dims = { SC2_MAX_SLICES }, .flags = V4L2_CTRL_FLAG_DYNAMIC_ARRAY },
		{ .id = V4L2_CID_STATELESS_HEVC_ENTRY_POINT_OFFSETS,
		  .dims = { SC2_MAX_ENTRIES }, .min = 0, .max = U32_MAX, .step = 1,
		  .flags = V4L2_CTRL_FLAG_DYNAMIC_ARRAY },
		{ .id = V4L2_CID_STATELESS_HEVC_DECODE_MODE,
		  .min = V4L2_STATELESS_HEVC_DECODE_MODE_FRAME_BASED,
		  .max = V4L2_STATELESS_HEVC_DECODE_MODE_FRAME_BASED,
		  .def = V4L2_STATELESS_HEVC_DECODE_MODE_FRAME_BASED },
		{ .id = V4L2_CID_STATELESS_HEVC_START_CODE,
		  .max = V4L2_STATELESS_HEVC_START_CODE_NONE,
		  .def = V4L2_STATELESS_HEVC_START_CODE_NONE },
	};
	struct v4l2_ctrl *ctrl;
	unsigned int i;

	v4l2_ctrl_handler_init(&ctx->controls, ARRAY_SIZE(configs) + 3);
	for (i = 0; i < ARRAY_SIZE(configs); i++)
		v4l2_ctrl_new_custom(&ctx->controls, &configs[i], NULL);
	ctrl = v4l2_ctrl_new_std_menu(&ctx->controls, NULL,
				    V4L2_CID_MPEG_VIDEO_HEVC_PROFILE,
				    V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10, 0,
				    V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	ctrl = v4l2_ctrl_new_std_menu(&ctx->controls, NULL,
				    V4L2_CID_MPEG_VIDEO_AV1_PROFILE,
				    V4L2_MPEG_VIDEO_AV1_PROFILE_MAIN, 0,
				    V4L2_MPEG_VIDEO_AV1_PROFILE_MAIN);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	ctrl = v4l2_ctrl_new_std(&ctx->controls, NULL,
				V4L2_CID_MIN_BUFFERS_FOR_CAPTURE, 1, 17, 1, 17);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	ctx->fh.ctrl_handler = &ctx->controls;
	return ctx->controls.error;
}

static void sc2_complete_request(struct sc2_ctx *ctx,
				 struct vb2_v4l2_buffer *vb)
{
	if (vb->vb2_buf.req_obj.req &&
	    !test_and_set_bit(SC2_REQUEST_DONE, &sc2_buffer(vb)->request_flags))
		v4l2_ctrl_request_complete(vb->vb2_buf.req_obj.req, &ctx->controls);
}

static int sc2_queue_setup(struct vb2_queue *q, unsigned int *count,
			   unsigned int *planes, unsigned int sizes[],
			   struct device *alloc_devs[])
{
	struct sc2_ctx *ctx = vb2_get_drv_priv(q);
	u32 size = sc2_format(ctx, q->type)->plane_fmt[0].sizeimage;

	if (*planes)
		return *planes == 1 && sizes[0] >= size ? 0 : -EINVAL;
	*planes = 1;
	sizes[0] = size;
	return 0;
}

static int sc2_buf_init(struct vb2_buffer *vb)
{
	struct sc2_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct sc2_buffer *buf = sc2_buffer(to_vb2_v4l2_buffer(vb));

	INIT_LIST_HEAD(&buf->capture_entry);
	if (V4L2_TYPE_IS_OUTPUT(vb->type))
		return 0;
	mutex_lock(&ctx->state_lock);
	list_add_tail(&buf->capture_entry, &ctx->captures);
	mutex_unlock(&ctx->state_lock);
	return 0;
}

static int sc2_buf_prepare(struct vb2_buffer *vb)
{
	struct sc2_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct sc2_buffer *buf = sc2_buffer(to_vb2_v4l2_buffer(vb));
	struct dma_buf *dmabuf;
	u32 size = sc2_format(ctx, vb->type)->plane_fmt[0].sizeimage;
	int ret = 0;

	if (vb2_plane_size(vb, 0) < size)
		return -EINVAL;
	if (V4L2_TYPE_IS_OUTPUT(vb->type)) {
		if (vb->planes[0].data_offset || !vb2_get_plane_payload(vb, 0))
			return -EINVAL;
		clear_bit(SC2_REQUEST_DONE, &buf->request_flags);
		return 0;
	}
	if (vb->planes[0].data_offset)
		return -EINVAL;
	/* Absolute scatter PFNs require driver-owned, physically contiguous storage. */
	if (ctx->capture_fmt.pixelformat == V4L2_PIX_FMT_AMLOGIC_FBC10 &&
	    vb->memory != VB2_MEMORY_MMAP)
		return -EOPNOTSUPP;
	mutex_lock(&ctx->state_lock);
	if (vb->memory == VB2_MEMORY_MMAP && buf->output_buffer) {
		dmabuf = buf->output_buffer;
	} else {
		/* Retain one export across requeues, including fatal DMA quarantine. */
		if (vb->memory == VB2_MEMORY_DMABUF) {
			dmabuf = vb->planes[0].dbuf;
			get_dma_buf(dmabuf);
		} else {
			dmabuf = vb->vb2_queue->mem_ops->get_dmabuf(vb,
						vb->planes[0].mem_priv, O_RDWR);
			if (IS_ERR_OR_NULL(dmabuf)) {
				ret = dmabuf ? PTR_ERR(dmabuf) : -ENOMEM;
				goto unlock;
			}
		}
		if (buf->output_buffer)
			dma_buf_put(buf->output_buffer);
		buf->output_buffer = dmabuf;
	}
	if (!buf->surface.cpu) {
		if (ctx->source_fmt.pixelformat == V4L2_PIX_FMT_AV1_FRAME)
			ret = meson_sc2_av1_surface_alloc(ctx->dev->hw, &buf->surface,
				ctx->source_fmt.width, ctx->source_fmt.height);
		else if (ctx->capture_fmt.pixelformat == V4L2_PIX_FMT_AMLOGIC_FBC10)
			ret = meson_sc2_hevc_fbc_bind(ctx->hwctx, &buf->surface,
				ctx->source_fmt.width, ctx->source_fmt.height,
				vb2_plane_vaddr(vb, 0), vb2_dma_contig_plane_dma_addr(vb, 0),
				vb2_plane_size(vb, 0), dmabuf);
		else
			ret = meson_sc2_surface_alloc(ctx->dev->hw, &buf->surface,
				ctx->source_fmt.width, ctx->source_fmt.height, 8);
	}
	if (ret)
		goto unlock;
	vb2_set_plane_payload(vb, 0, size);
unlock:
	mutex_unlock(&ctx->state_lock);
	return ret;
}

static void sc2_buf_cleanup(struct vb2_buffer *vb)
{
	struct sc2_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct sc2_buffer *buf = sc2_buffer(to_vb2_v4l2_buffer(vb));

	if (V4L2_TYPE_IS_OUTPUT(vb->type))
		return;
	mutex_lock(&ctx->state_lock);
	list_del(&buf->capture_entry);
	meson_sc2_surface_free(ctx->dev->hw, &buf->surface);
	if (buf->output_buffer)
		dma_buf_put(buf->output_buffer);
	mutex_unlock(&ctx->state_lock);
}

static void sc2_buf_queue(struct vb2_buffer *vb)
{
	struct sc2_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	if (V4L2_TYPE_IS_OUTPUT(vb->type))
		clear_bit(SC2_REQUEST_DONE,
			  &sc2_buffer(to_vb2_v4l2_buffer(vb))->request_flags);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(vb));
}

static int sc2_buf_out_validate(struct vb2_buffer *vb)
{
	to_vb2_v4l2_buffer(vb)->field = V4L2_FIELD_NONE;
	return 0;
}

static void sc2_buf_request_complete(struct vb2_buffer *vb)
{
	struct sc2_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	sc2_complete_request(ctx, to_vb2_v4l2_buffer(vb));
}

static int sc2_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct sc2_ctx *ctx = vb2_get_drv_priv(q);
	struct vb2_v4l2_buffer *vb;
	int ret;

	ret = meson_sc2_hevc_context_reset(ctx->hwctx);
	if (!ret) {
		WRITE_ONCE(ctx->aborting, false);
		return 0;
	}
	while ((vb = V4L2_TYPE_IS_OUTPUT(q->type) ?
		v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx) :
		v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx)))
		v4l2_m2m_buf_done(vb, VB2_BUF_STATE_QUEUED);
	return ret;
}
static void sc2_stop_streaming(struct vb2_queue *q)
{
	struct sc2_ctx *ctx = vb2_get_drv_priv(q);
	struct vb2_v4l2_buffer *vb;
	struct sc2_buffer *buf;

	WRITE_ONCE(ctx->aborting, true);
	meson_sc2_hevc_abort(ctx->hwctx);
	flush_work(&ctx->work);
	while ((vb = V4L2_TYPE_IS_OUTPUT(q->type) ?
		v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx) :
		v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx))) {
		if (V4L2_TYPE_IS_OUTPUT(q->type))
			sc2_complete_request(ctx, vb);
		v4l2_m2m_buf_done(vb, VB2_BUF_STATE_ERROR);
	}
	mutex_lock(&ctx->state_lock);
	ctx->epoch++;
	list_for_each_entry(buf, &ctx->captures, capture_entry)
		buf->surface.valid = false;
	mutex_unlock(&ctx->state_lock);
}

static const struct vb2_ops sc2_queue_ops = {
	.queue_setup = sc2_queue_setup,
	.buf_init = sc2_buf_init,
	.buf_prepare = sc2_buf_prepare,
	.buf_cleanup = sc2_buf_cleanup,
	.buf_queue = sc2_buf_queue,
	.buf_out_validate = sc2_buf_out_validate,
	.buf_request_complete = sc2_buf_request_complete,
	.start_streaming = sc2_start_streaming,
	.stop_streaming = sc2_stop_streaming,
};

static int sc2_queue_init(void *priv, struct vb2_queue *src,
			   struct vb2_queue *dst)
{
	struct sc2_ctx *ctx = priv;
	struct vb2_queue *queues[] = { src, dst };
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(queues); i++) {
		struct vb2_queue *q = queues[i];

		q->type = i ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE :
			      V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
		q->io_modes = VB2_MMAP | VB2_DMABUF;
		q->drv_priv = ctx;
		q->ops = &sc2_queue_ops;
		q->mem_ops = &vb2_dma_contig_memops;
		q->buf_struct_size = sizeof(struct sc2_buffer);
		q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
		q->lock = &ctx->dev->lock;
		q->dev = ctx->dev->dev;
		q->supports_requests = !i;
		q->requires_requests = !i;
		ret = vb2_queue_init(q);
		if (ret)
			return ret;
	}
	return 0;
}

/* Require explicit per-picture controls, never inherit an earlier request. */
static int sc2_request_validate(struct media_request *req)
{
	static const u32 required[] = {
		V4L2_CID_STATELESS_HEVC_SPS,
		V4L2_CID_STATELESS_HEVC_PPS,
		V4L2_CID_STATELESS_HEVC_DECODE_PARAMS,
		V4L2_CID_STATELESS_HEVC_DECODE_MODE,
		V4L2_CID_STATELESS_HEVC_START_CODE,
		V4L2_CID_STATELESS_HEVC_SLICE_PARAMS,
	};
	static const u32 av1_required[] = {
		V4L2_CID_STATELESS_AV1_SEQUENCE,
		V4L2_CID_STATELESS_AV1_FRAME,
		V4L2_CID_STATELESS_AV1_TILE_GROUP_ENTRY,
	};
	const u32 *ids = required;
	size_t count = ARRAY_SIZE(required);
	struct v4l2_ctrl_handler *hdl;
	struct media_request_object *obj;
	struct sc2_ctx *ctx = NULL;
	unsigned int i;
	int ret = -EINVAL;

	if (vb2_request_buffer_cnt(req) != 1)
		return -EINVAL;
	list_for_each_entry(obj, &req->objects, list) {
		struct vb2_buffer *vb;

		if (!vb2_request_object_is_buffer(obj))
			continue;
		vb = container_of(obj, struct vb2_buffer, req_obj);
		if (vb->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
			return -EINVAL;
		ctx = vb2_get_drv_priv(vb->vb2_queue);
	}
	if (!ctx)
		return -ENOENT;
	hdl = v4l2_ctrl_request_hdl_find(req, &ctx->controls);
	if (!hdl)
		return -ENOENT;
	if (ctx->source_fmt.pixelformat == V4L2_PIX_FMT_AV1_FRAME) {
		ids = av1_required;
		count = ARRAY_SIZE(av1_required);
	}
	for (i = 0; i < count; i++)
		if (!v4l2_ctrl_request_hdl_ctrl_find(hdl, ids[i]))
			goto put;
	ret = vb2_request_validate(req);
put:
	v4l2_ctrl_request_hdl_put(hdl);
	return ret;
}

static const struct media_device_ops sc2_media_ops = {
	.req_validate = sc2_request_validate,
	.req_queue = v4l2_m2m_request_queue,
};

static struct v4l2_ctrl_ref *sc2_request_control(struct v4l2_ctrl_handler *hdl,
						       u32 id)
{
	struct v4l2_ctrl_ref *ref;

	list_for_each_entry(ref, &hdl->ctrl_refs, node)
		if (ref->ctrl->id == id && ref->p_req_valid &&
		    !ref->p_req_array_enomem)
			return ref;
	return NULL;
}

static int sc2_prepare_job(struct sc2_ctx *ctx, struct vb2_v4l2_buffer *src,
			    struct sc2_buffer *dst,
			    struct meson_hevc_syntax_request *syntax,
			    struct meson_sc2_hevc_job *job,
			    struct v4l2_ctrl_handler *request_hdl)
{
	struct v4l2_ctrl_ref *slices, *entries;
	const struct v4l2_ctrl_hevc_decode_params *decode = syntax->decode;
	struct sc2_buffer *buf;
	const u8 *data = vb2_plane_vaddr(&src->vb2_buf, 0);
	size_t used = vb2_get_plane_payload(&src->vb2_buf, 0);
	size_t start = 0, entry = 0, i, j;
	unsigned int entry_count = 0;

	if (!data || !used)
		return -EINVAL;
	if (syntax->sps->pic_width_in_luma_samples != ctx->source_fmt.width ||
	    syntax->sps->pic_height_in_luma_samples != ctx->source_fmt.height ||
	    syntax->sps->bit_depth_luma_minus8 !=
		(ctx->capture_fmt.pixelformat == V4L2_PIX_FMT_AMLOGIC_FBC10 ? 2 : 0) ||
	    syntax->sps->bit_depth_chroma_minus8 != syntax->sps->bit_depth_luma_minus8 ||
	    syntax->sps->chroma_format_idc != 1)
		return -EINVAL;
	if (sc2_request_control(request_hdl,
				V4L2_CID_STATELESS_HEVC_DECODE_MODE)->p_req.p_s32[0] !=
	    V4L2_STATELESS_HEVC_DECODE_MODE_FRAME_BASED ||
	    sc2_request_control(request_hdl,
				V4L2_CID_STATELESS_HEVC_START_CODE)->p_req.p_s32[0] !=
	    V4L2_STATELESS_HEVC_START_CODE_NONE)
		return -EOPNOTSUPP;
	if ((syntax->sps->flags & V4L2_HEVC_SPS_FLAG_SCALING_LIST_ENABLED) &&
	    !syntax->scaling)
		return -ENOENT;
	slices = sc2_request_control(request_hdl,
				    V4L2_CID_STATELESS_HEVC_SLICE_PARAMS);
	entries = sc2_request_control(request_hdl,
				     V4L2_CID_STATELESS_HEVC_ENTRY_POINT_OFFSETS);
	if (!slices)
		return -ENOENT;
	syntax->num_slices = slices->p_req_elems;
	if (!syntax->num_slices || syntax->num_slices > SC2_MAX_SLICES)
		return -EINVAL;
	memcpy(ctx->slice_params, slices->p_req.p,
	       syntax->num_slices * sizeof(*ctx->slice_params));
	for (i = 0; i < syntax->num_slices; i++)
		if (ctx->slice_params[i].pic_struct != V4L2_HEVC_SEI_PIC_STRUCT_FRAME)
			return -EOPNOTSUPP;
	for (i = 0; i < decode->num_active_dpb_entries; i++)
		if (decode->dpb[i].field_pic)
			return -EOPNOTSUPP;
	if (entries) {
		entry_count = entries->p_req_elems;
		if (entry_count > SC2_MAX_ENTRIES)
			return -EINVAL;
	}
	/* FRAME_BASED native offsets are absolute within concatenated raw NALs. */
	for (i = 0; i < syntax->num_slices; i++) {
		struct v4l2_ctrl_hevc_slice_params *p = &ctx->slice_params[i];
		size_t end = p->bit_size / 8;

		if ((p->bit_size & 7) || end > used || end <= start + 2 ||
		    p->data_byte_offset < start + 2 || p->data_byte_offset >= end ||
		    p->num_entry_point_offsets > entry_count - entry ||
		    (data[start] & 0x81) || !(data[start + 1] & 7) ||
		    (data[start + 1] & 0xf8) ||
		    ((data[start] >> 1) & 0x3f) != p->nal_unit_type)
			return -EINVAL;
		p->bit_size -= start * 8;
		p->data_byte_offset -= start;
		ctx->slices[i] = (struct meson_hevc_syntax_slice) {
			.params = p,
			.data = data + start,
			.size = end - start,
			.entry_point_offsets = entries ? entries->p_req.p_u32 + entry : NULL,
			.num_entry_point_offsets = p->num_entry_point_offsets,
		};
		entry += p->num_entry_point_offsets;
		start = end;
	}
	if (start != used || (entry && entry != entry_count))
		return -EINVAL;
	syntax->slices = ctx->slices;
	if (decode->num_active_dpb_entries > ARRAY_SIZE(job->references))
		return -EINVAL;
	if (decode->flags & V4L2_HEVC_DECODE_PARAM_FLAG_IDR_PIC) {
		ctx->epoch++;
		list_for_each_entry(buf, &ctx->captures, capture_entry)
			buf->surface.valid = false;
	}
	for (i = 0; i < decode->num_active_dpb_entries; i++) {
		const struct v4l2_hevc_dpb_entry *dpb = &decode->dpb[i];
		struct meson_sc2_surface *found = NULL;

		for (j = 0; j < i; j++)
			if (decode->dpb[j].timestamp == dpb->timestamp)
				return -EINVAL;
		list_for_each_entry(buf, &ctx->captures, capture_entry) {
			if (buf == dst || !buf->surface.valid ||
			    buf->epoch != ctx->epoch || buf->timestamp != dpb->timestamp)
				continue;
			if (found || buf->surface.poc != dpb->pic_order_cnt_val)
				return -EINVAL;
			found = &buf->surface;
		}
		if (!found)
			return -ENOENT;
		job->references[i] = found;
	}
	list_for_each_entry(buf, &ctx->captures, capture_entry)
		if (buf != dst && buf->surface.valid && buf->epoch == ctx->epoch &&
		    buf->timestamp == src->vb2_buf.timestamp)
			return -EINVAL;
	job->syntax = syntax;
	job->destination = &dst->surface;
	job->output_dma = vb2_dma_contig_plane_dma_addr(&dst->m2m.vb.vb2_buf, 0);
	job->output_size = ctx->capture_fmt.plane_fmt[0].sizeimage;
	job->output_stride = ctx->capture_fmt.width;
	job->output_format = ctx->capture_fmt.pixelformat;
	job->output_uv_offset = job->output_stride * ctx->capture_fmt.height;
	job->output_buffer = dst->output_buffer;
	return 0;
}

static int sc2_decode_av1(struct sc2_ctx *ctx, struct vb2_v4l2_buffer *src,
			  struct sc2_buffer *dst, struct v4l2_ctrl_handler *hdl)
{
	struct meson_av1_syntax_request syntax = {};
	struct meson_sc2_av1_job job = { .syntax = &syntax };
	struct v4l2_ctrl_ref *tiles, *grain;
	const struct v4l2_ctrl_av1_frame *frame;
	struct sc2_buffer *buf;
	unsigned int i;

	syntax.sequence = sc2_request_control(hdl,
		V4L2_CID_STATELESS_AV1_SEQUENCE)->p_req.p;
	syntax.frame = sc2_request_control(hdl, V4L2_CID_STATELESS_AV1_FRAME)->p_req.p;
	frame = syntax.frame;
	if (frame->upscaled_width != ctx->source_fmt.width ||
	    frame->frame_height_minus_1 + 1 != ctx->source_fmt.height)
		return -EINVAL;
	tiles = sc2_request_control(hdl, V4L2_CID_STATELESS_AV1_TILE_GROUP_ENTRY);
	grain = sc2_request_control(hdl, V4L2_CID_STATELESS_AV1_FILM_GRAIN);
	syntax.tiles = tiles->p_req.p;
	syntax.num_tiles = tiles->p_req_elems;
	syntax.film_grain = grain ? grain->p_req.p : NULL;
	syntax.tile_data = vb2_plane_vaddr(&src->vb2_buf, 0);
	syntax.tile_data_size = vb2_get_plane_payload(&src->vb2_buf, 0);
	if (!syntax.tile_data || !syntax.tile_data_size)
		return -EINVAL;
	if (frame->frame_type != V4L2_AV1_KEY_FRAME) {
		for (i = 0; i < 8; i++) {
			list_for_each_entry(buf, &ctx->captures, capture_entry) {
				if (buf == dst || !buf->surface.valid ||
				    buf->epoch != ctx->epoch ||
				    buf->timestamp != frame->reference_frame_ts[i])
					continue;
				if (job.references[i])
					return -EINVAL;
				job.references[i] = &buf->surface;
			}
		}
	}
	job.output.destination = &dst->surface;
	job.output.output_dma = vb2_dma_contig_plane_dma_addr(&dst->m2m.vb.vb2_buf, 0);
	job.output.output_size = ctx->capture_fmt.plane_fmt[0].sizeimage;
	job.output.output_stride = ctx->capture_fmt.plane_fmt[0].bytesperline;
	job.output.output_format = ctx->capture_fmt.pixelformat;
	job.output.output_uv_offset = job.output.output_stride * ctx->capture_fmt.height;
	job.output.output_buffer = dst->output_buffer;
	job.timestamp = src->vb2_buf.timestamp;
	return meson_sc2_av1_decode(ctx->hwctx, &job);
}

static void sc2_worker(struct work_struct *work)
{
	struct sc2_ctx *ctx = container_of(work, struct sc2_ctx, work);
	struct vb2_v4l2_buffer *src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	struct vb2_v4l2_buffer *cap = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	struct meson_hevc_syntax_request syntax = {};
	struct meson_sc2_hevc_job job = {};
	struct v4l2_ctrl_handler *request_hdl = NULL;
	struct dma_buf *input = NULL;
	struct sc2_buffer *dst;
	bool cpu_access = false;
	bool av1 = ctx->source_fmt.pixelformat == V4L2_PIX_FMT_AV1_FRAME;
	int ret = -ECANCELED;

	if (!src || !cap)
		goto finish;
	dst = sc2_buffer(cap);
	mutex_lock(&ctx->state_lock);
	/* A later queued destination may still be an earlier job's reference.
	 * Retire its old contents in decode order, never during buf_prepare.
	 */
	dst->surface.valid = false;
	if (READ_ONCE(ctx->aborting) || READ_ONCE(ctx->dev->removing))
		goto unlock;
	request_hdl = v4l2_ctrl_request_hdl_find(src->vb2_buf.req_obj.req,
						&ctx->controls);
	if (!request_hdl) {
		ret = -ENOENT;
		goto unlock;
	}
	ret = v4l2_ctrl_request_setup(src->vb2_buf.req_obj.req, &ctx->controls);
	if (ret)
		goto unlock;
	input = src->vb2_buf.planes[0].dbuf;
	if (input) {
		ret = dma_buf_begin_cpu_access(input, DMA_TO_DEVICE);
		if (ret)
			goto unlock;
		cpu_access = true;
	}
	if (av1) {
		ret = sc2_decode_av1(ctx, src, dst, request_hdl);
	} else {
		/* QUEUED request values are immutable during hardware execution. */
		syntax.sps = sc2_request_control(request_hdl,
			V4L2_CID_STATELESS_HEVC_SPS)->p_req.p;
		syntax.pps = sc2_request_control(request_hdl,
			V4L2_CID_STATELESS_HEVC_PPS)->p_req.p;
		syntax.decode = sc2_request_control(request_hdl,
			V4L2_CID_STATELESS_HEVC_DECODE_PARAMS)->p_req.p;
		if (sc2_request_control(request_hdl, V4L2_CID_STATELESS_HEVC_SCALING_MATRIX))
			syntax.scaling = sc2_request_control(request_hdl,
				V4L2_CID_STATELESS_HEVC_SCALING_MATRIX)->p_req.p;
		ret = sc2_prepare_job(ctx, src, dst, &syntax, &job, request_hdl);
		if (!ret)
			ret = meson_sc2_hevc_decode(ctx->hwctx, &job);
	}
	if (cpu_access)
		dma_buf_end_cpu_access(input, DMA_TO_DEVICE);
	if (!ret && !READ_ONCE(ctx->aborting)) {
		dst->surface.valid = true;
		dst->surface.poc = av1 ? 0 : syntax.decode->pic_order_cnt_val;
		dst->timestamp = src->vb2_buf.timestamp;
		dst->epoch = ctx->epoch;
	} else {
		dst->surface.valid = false;
		if (!ret)
			ret = -ECANCELED;
	}
	if (meson_sc2_hevc_context_fatal(ctx->hwctx)) {
		WRITE_ONCE(ctx->aborting, true);
		vb2_queue_error(v4l2_m2m_get_src_vq(ctx->fh.m2m_ctx));
		vb2_queue_error(v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx));
	}
unlock:
	v4l2_ctrl_request_hdl_put(request_hdl);
	mutex_unlock(&ctx->state_lock);
finish:
	if (src) {
		sc2_complete_request(ctx, src);
		src->sequence = ctx->source_sequence++;
	}
	if (src && cap)
		v4l2_m2m_buf_copy_metadata(src, cap, true);
	if (cap) {
		cap->field = V4L2_FIELD_NONE;
		cap->sequence = ctx->capture_sequence++;
		if (ret)
			vb2_set_plane_payload(&cap->vb2_buf, 0, 0);
	}
	v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
	if (src)
		v4l2_m2m_buf_done(src, ret ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
	if (cap)
		v4l2_m2m_buf_done(cap, ret ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
	v4l2_m2m_job_finish(ctx->dev->m2m, ctx->fh.m2m_ctx);
}

static void sc2_device_run(void *priv)
{
	struct sc2_ctx *ctx = priv;

	queue_work(ctx->dev->workqueue, &ctx->work);
}

static int sc2_job_ready(void *priv)
{
	struct sc2_ctx *ctx = priv;

	return !READ_ONCE(ctx->aborting) && !READ_ONCE(ctx->dev->removing);
}

static void sc2_job_abort(void *priv)
{
	struct sc2_ctx *ctx = priv;

	WRITE_ONCE(ctx->aborting, true);
	meson_sc2_hevc_abort(ctx->hwctx);
}

static const struct v4l2_m2m_ops sc2_m2m_ops = {
	.device_run = sc2_device_run,
	.job_ready = sc2_job_ready,
	.job_abort = sc2_job_abort,
};

static const struct v4l2_ioctl_ops sc2_ioctl_ops = {
	.vidioc_querycap = sc2_querycap,
	.vidioc_enum_fmt_vid_cap = sc2_enum_fmt,
	.vidioc_enum_fmt_vid_out = sc2_enum_fmt,
	.vidioc_enum_framesizes = sc2_enum_framesizes,
	.vidioc_g_fmt_vid_cap_mplane = sc2_get_fmt,
	.vidioc_g_fmt_vid_out_mplane = sc2_get_fmt,
	.vidioc_try_fmt_vid_cap_mplane = sc2_try_fmt,
	.vidioc_try_fmt_vid_out_mplane = sc2_try_fmt,
	.vidioc_s_fmt_vid_cap_mplane = sc2_set_fmt,
	.vidioc_s_fmt_vid_out_mplane = sc2_set_fmt,
	.vidioc_g_selection = sc2_get_selection,
	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
	.vidioc_try_decoder_cmd = v4l2_m2m_ioctl_stateless_try_decoder_cmd,
	.vidioc_decoder_cmd = v4l2_m2m_ioctl_stateless_decoder_cmd,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static int sc2_open(struct file *file)
{
	struct sc2_dev *dev = video_drvdata(file);
	struct sc2_ctx *ctx;
	int ret;

	if (mutex_lock_interruptible(&dev->lock))
		return -ERESTARTSYS;
	if (dev->removing) {
		ret = -ENODEV;
		goto unlock;
	}
	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		ret = -ENOMEM;
		goto unlock;
	}
	ctx->dev = dev;
	ctx->epoch = 1;
	mutex_init(&ctx->state_lock);
	INIT_LIST_HEAD(&ctx->captures);
	INIT_WORK(&ctx->work, sc2_worker);
	ctx->slices = kvcalloc(SC2_MAX_SLICES, sizeof(*ctx->slices), GFP_KERNEL);
	ctx->slice_params = kvcalloc(SC2_MAX_SLICES,
				     sizeof(*ctx->slice_params), GFP_KERNEL);
	if (!ctx->slices || !ctx->slice_params) {
		ret = -ENOMEM;
		goto free_ctx;
	}
	v4l2_fh_init(&ctx->fh, &dev->video);
	ctx->source_fmt = (struct v4l2_pix_format_mplane) {
		.width = 1920, .height = 1080, .pixelformat = V4L2_PIX_FMT_HEVC_SLICE,
		.field = V4L2_FIELD_NONE, .num_planes = 1,
		.colorspace = V4L2_COLORSPACE_REC709,
		.plane_fmt[0].sizeimage = SC2_SOURCE_SIZE,
	};
	sc2_capture_format(ctx, &ctx->capture_fmt);
	ret = sc2_controls_init(ctx);
	if (ret)
		goto free_controls;
	ret = meson_sc2_hevc_context_create(dev->hw, &ctx->hwctx);
	if (ret)
		goto free_controls;
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(dev->m2m, ctx, sc2_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto free_hwctx;
	}
	v4l2_fh_add(&ctx->fh, file);
	list_add_tail(&ctx->entry, &dev->contexts);
	mutex_unlock(&dev->lock);
	return 0;
free_hwctx:
	meson_sc2_hevc_context_destroy(ctx->hwctx);
free_controls:
	v4l2_ctrl_handler_free(&ctx->controls);
	v4l2_fh_exit(&ctx->fh);
free_ctx:
	kvfree(ctx->slice_params);
	kvfree(ctx->slices);
	kfree(ctx);
unlock:
	mutex_unlock(&dev->lock);
	return ret;
}

static int sc2_release(struct file *file)
{
	struct sc2_ctx *ctx = sc2_file_ctx(file);
	struct sc2_dev *dev = ctx->dev;

	mutex_lock(&dev->lock);
	WRITE_ONCE(ctx->aborting, true);
	meson_sc2_hevc_abort(ctx->hwctx);
	flush_work(&ctx->work);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	list_del(&ctx->entry);
	v4l2_fh_del(&ctx->fh, file);
	v4l2_ctrl_handler_free(&ctx->controls);
	v4l2_fh_exit(&ctx->fh);
	meson_sc2_hevc_context_destroy(ctx->hwctx);
	kvfree(ctx->slice_params);
	kvfree(ctx->slices);
	kfree(ctx);
	mutex_unlock(&dev->lock);
	return 0;
}

static const struct v4l2_file_operations sc2_fops = {
	.owner = THIS_MODULE,
	.open = sc2_open,
	.release = sc2_release,
	.unlocked_ioctl = video_ioctl2,
	.poll = v4l2_m2m_fop_poll,
	.mmap = v4l2_m2m_fop_mmap,
};

/* Video-device references keep this container alive across platform unbind. */
static void sc2_device_release(struct v4l2_device *v4l2)
{
	struct sc2_dev *dev = container_of(v4l2, struct sc2_dev, v4l2);

	v4l2_m2m_release(dev->m2m);
	destroy_workqueue(dev->workqueue);
	meson_sc2_hw_destroy(dev->hw);
	media_device_cleanup(&dev->media);
	/* v4l2_device_disconnect() drops the platform-device reference. */
	kfree(dev);
}

static int sc2_probe(struct platform_device *pdev)
{
	struct sc2_dev *dev;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	dev->dev = &pdev->dev;
	mutex_init(&dev->lock);
	INIT_LIST_HEAD(&dev->contexts);
	ret = meson_sc2_hw_create(&pdev->dev, &dev->hw);
	if (ret)
		goto free_dev;
	dev->workqueue = alloc_ordered_workqueue(SC2_NAME, WQ_MEM_RECLAIM);
	if (!dev->workqueue) {
		ret = -ENOMEM;
		goto free_hw;
	}
	dev->m2m = v4l2_m2m_init(&sc2_m2m_ops);
	if (IS_ERR(dev->m2m)) {
		ret = PTR_ERR(dev->m2m);
		goto free_workqueue;
	}
	media_device_init(&dev->media);
	dev->media.dev = &pdev->dev;
	dev->media.ops = &sc2_media_ops;
	strscpy(dev->media.model, "Amlogic SC2 HEVC/AV1 Request", sizeof(dev->media.model));
	dev->v4l2.mdev = &dev->media;
	ret = v4l2_device_register(&pdev->dev, &dev->v4l2);
	if (ret)
		goto free_media;
	dev->v4l2.release = sc2_device_release;
	strscpy(dev->video.name, SC2_NAME, sizeof(dev->video.name));
	dev->video.v4l2_dev = &dev->v4l2;
	dev->video.fops = &sc2_fops;
	dev->video.ioctl_ops = &sc2_ioctl_ops;
	dev->video.lock = &dev->lock;
	dev->video.release = video_device_release_empty;
	dev->video.vfl_dir = VFL_DIR_M2M;
	dev->video.device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	video_set_drvdata(&dev->video, dev);
	ret = video_register_device(&dev->video, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto unregister_v4l2;
	ret = v4l2_m2m_register_media_controller(dev->m2m, &dev->video,
						MEDIA_ENT_F_PROC_VIDEO_DECODER);
	if (ret)
		goto unregister_video;
	ret = media_device_register(&dev->media);
	if (ret)
		goto unregister_controller;
	ret = meson_sc2_stateful_register(&pdev->dev, dev->hw, &dev->v4l2,
					 &dev->stateful);
	if (ret)
		goto unregister_media;
	platform_set_drvdata(pdev, dev);
	return 0;
unregister_media:
	media_device_unregister(&dev->media);
unregister_controller:
	v4l2_m2m_unregister_media_controller(dev->m2m);
unregister_video:
	video_unregister_device(&dev->video);
unregister_v4l2:
	v4l2_device_unregister(&dev->v4l2);
	v4l2_device_put(&dev->v4l2);
	return ret;
free_media:
	media_device_cleanup(&dev->media);
	v4l2_m2m_release(dev->m2m);
free_workqueue:
	destroy_workqueue(dev->workqueue);
free_hw:
	meson_sc2_hw_destroy(dev->hw);
free_dev:
	kfree(dev);
	return ret;
}

static void sc2_remove(struct platform_device *pdev)
{
	struct sc2_dev *dev = platform_get_drvdata(pdev);
	struct sc2_ctx *ctx;

	mutex_lock(&dev->lock);
	WRITE_ONCE(dev->removing, true);
	list_for_each_entry(ctx, &dev->contexts, entry) {
		WRITE_ONCE(ctx->aborting, true);
		meson_sc2_hevc_abort(ctx->hwctx);
		flush_work(&ctx->work);
		vb2_queue_error(v4l2_m2m_get_src_vq(ctx->fh.m2m_ctx));
		vb2_queue_error(v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx));
	}
	mutex_unlock(&dev->lock);
	meson_sc2_stateful_unregister(dev->stateful);
	media_device_unregister(&dev->media);
	v4l2_m2m_unregister_media_controller(dev->m2m);
	video_unregister_device(&dev->video);
	v4l2_device_unregister(&dev->v4l2);
	v4l2_device_put(&dev->v4l2);
}

static const struct of_device_id sc2_of_match[] = {
	{ .compatible = "amlogic,meson-sc2-hevc-decoder" },
	{}
};
MODULE_DEVICE_TABLE(of, sc2_of_match);

static struct platform_driver sc2_driver = {
	.probe = sc2_probe,
	.remove = sc2_remove,
	.driver = {
		.name = SC2_NAME,
		.of_match_table = sc2_of_match,
		/* A fatal DMA quarantine must not lose its genpd via sysfs unbind. */
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(sc2_driver);

MODULE_DESCRIPTION("Amlogic SC2 HEVC/AV1 Request and VP9/AVS2 stateful decoders");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("DMA_BUF");
