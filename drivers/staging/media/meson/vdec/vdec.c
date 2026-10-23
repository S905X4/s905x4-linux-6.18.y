// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018 BayLibre, SAS
 * Author: Maxime Jourdan <mjourdan@baylibre.com>
 */

#include <linux/of.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mfd/syscon.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-dev.h>
#include <media/videobuf2-dma-contig.h>
#include <linux/pm_runtime.h>
#include <linux/firmware/meson/meson_sm.h>
#include "dos_regs.h"

#include "vdec.h"
#include "esparser.h"
#include "vdec_helpers.h"

struct dummy_buf {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

static void vdec_free_canvas(struct amvdec_session *sess);

/* 16 MiB for parsed bitstream swap exchange */
#define SIZE_VIFIFO SZ_16M

static u32 get_output_size(u32 width, u32 height)
{
	return ALIGN(ALIGN(width, 32) * ALIGN(height, 32), SZ_64K);
}

u32 amvdec_get_output_width(struct amvdec_session *sess)
{
	return ALIGN(sess->fmt_out->capture_width ?: sess->width, 32);
}
EXPORT_SYMBOL_GPL(amvdec_get_output_width);

u32 amvdec_get_output_height(struct amvdec_session *sess)
{
	return ALIGN(sess->fmt_out->capture_height ?: sess->height, 32);
}
EXPORT_SYMBOL_GPL(amvdec_get_output_height);

u32 amvdec_get_output_size(struct amvdec_session *sess)
{
	return get_output_size(amvdec_get_output_width(sess),
			       amvdec_get_output_height(sess));
}
EXPORT_SYMBOL_GPL(amvdec_get_output_size);

static int vdec_codec_needs_recycle(struct amvdec_session *sess)
{
	struct amvdec_codec_ops *codec_ops = sess->fmt_out->codec_ops;

	return codec_ops->can_recycle && codec_ops->recycle;
}

static int vdec_recycle_thread(void *data)
{
	struct amvdec_session *sess = data;
	struct amvdec_core *core = sess->core;
	struct amvdec_codec_ops *codec_ops = sess->fmt_out->codec_ops;
	struct amvdec_buffer *tmp, *n;
	unsigned int index;

	while (!kthread_should_stop()) {
		mutex_lock(&sess->bufs_recycle_lock);
		list_for_each_entry_safe(tmp, n, &sess->bufs_recycle, list) {
			if (!codec_ops->can_recycle(core))
				break;


			for (index = 0; index < sess->num_dst_bufs; index++)
				if (sess->fw_idx_to_vb2_idx[index] == tmp->vb->index)
					break;
			if (index < sess->num_dst_bufs)
				codec_ops->recycle(core, index);
			list_del(&tmp->list);
			kfree(tmp);
		}
		mutex_unlock(&sess->bufs_recycle_lock);
		if (!sess->should_stop && sess->status != STATUS_STOPPED)
			schedule_work(&sess->esparser_queue_work);

		usleep_range(5000, 10000);
	}

	return 0;
}

static int vdec_poweron(struct amvdec_session *sess)
{
	int ret;
	struct amvdec_ops *vdec_ops = sess->fmt_out->vdec_ops;

	ret = clk_prepare_enable(sess->core->dos_parser_clk);
	if (ret)
		return ret;

	ret = clk_prepare_enable(sess->core->dos_clk);
	if (ret)
		goto disable_dos_parser;

	ret = vdec_ops->start(sess);
	if (ret)
		goto disable_dos;

	esparser_power_up(sess);

	return 0;

disable_dos:
	if (sess->core->quarantined)
		return ret;
	clk_disable_unprepare(sess->core->dos_clk);
disable_dos_parser:
	clk_disable_unprepare(sess->core->dos_parser_clk);

	return ret;
}

static void vdec_wait_inactive(struct amvdec_session *sess)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(5000);

	/* We consider 50ms with no IRQ to be inactive. */
	while (time_before(jiffies, deadline) &&
	       time_is_after_jiffies64(sess->last_irq_jiffies + msecs_to_jiffies(50)))
		msleep(25);
}

static int vdec_poweroff(struct amvdec_session *sess)
{
	struct amvdec_ops *vdec_ops = sess->fmt_out->vdec_ops;
	struct amvdec_codec_ops *codec_ops = sess->fmt_out->codec_ops;

	sess->should_stop = 1;
	vdec_wait_inactive(sess);
	if (codec_ops->drain)
		codec_ops->drain(sess);

	if (vdec_ops->stop(sess))
		return -EIO;
	clk_disable_unprepare(sess->core->dos_clk);
	clk_disable_unprepare(sess->core->dos_parser_clk);
	return 0;
}

static void
vdec_queue_recycle(struct amvdec_session *sess, struct vb2_buffer *vb)
{
	struct amvdec_buffer *new_buf;

	new_buf = kmalloc(sizeof(*new_buf), GFP_KERNEL);
	if (!new_buf)
		return;
	new_buf->vb = vb;

	mutex_lock(&sess->bufs_recycle_lock);
	list_add_tail(&new_buf->list, &sess->bufs_recycle);
	mutex_unlock(&sess->bufs_recycle_lock);
}

static void vdec_m2m_device_run(void *priv)
{
	struct amvdec_session *sess = priv;

	schedule_work(&sess->esparser_queue_work);
}

static void vdec_m2m_job_abort(void *priv)
{
	struct amvdec_session *sess = priv;

	v4l2_m2m_job_finish(sess->m2m_dev, sess->m2m_ctx);
}

static const struct v4l2_m2m_ops vdec_m2m_ops = {
	.device_run = vdec_m2m_device_run,
	.job_abort = vdec_m2m_job_abort,
};

static void process_num_buffers(struct vb2_queue *q,
				struct amvdec_session *sess,
				unsigned int *num_buffers,
				bool is_reqbufs)
{
	const struct amvdec_format *fmt_out = sess->fmt_out;
	unsigned int q_num_bufs = vb2_get_num_buffers(q);
	unsigned int buffers_total = q_num_bufs + *num_buffers;
	u32 min_buf_capture = v4l2_ctrl_g_ctrl(sess->ctrl_min_buf_capture);

	if (q_num_bufs + *num_buffers < min_buf_capture)
		*num_buffers = min_buf_capture - q_num_bufs;
	if (is_reqbufs && buffers_total < fmt_out->min_buffers)
		*num_buffers = fmt_out->min_buffers - q_num_bufs;
	if (buffers_total > fmt_out->max_buffers)
		*num_buffers = fmt_out->max_buffers - q_num_bufs;

	/* We need to program the complete CAPTURE buffer list
	 * in registers during start_streaming, and the firmwares
	 * are free to choose any of them to write frames to. As such,
	 * we need all of them to be queued into the driver
	 */
	sess->num_dst_bufs = q_num_bufs + *num_buffers;
	sess->capture_width = amvdec_get_output_width(sess);
	sess->capture_height = amvdec_get_output_height(sess);
	q->min_queued_buffers = max(fmt_out->min_buffers, sess->num_dst_bufs);
}

static int vdec_queue_setup(struct vb2_queue *q, unsigned int *num_buffers,
			    unsigned int *num_planes, unsigned int sizes[],
			    struct device *alloc_devs[])
{
	struct amvdec_session *sess = vb2_get_drv_priv(q);
	u32 output_size = amvdec_get_output_size(sess);

	if (*num_planes) {
		switch (q->type) {
		case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
			if (*num_planes != 1 ||
			    sizes[0] < sess->src_buffer_size)
				return -EINVAL;
			break;
		case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
			switch (sess->pixfmt_cap) {
			case V4L2_PIX_FMT_NV12M:
				if (*num_planes != 2 ||
				    sizes[0] < output_size ||
				    sizes[1] < output_size / 2)
					return -EINVAL;
				break;
			case V4L2_PIX_FMT_YUV420M:
				if (*num_planes != 3 ||
				    sizes[0] < output_size ||
				    sizes[1] < output_size / 4 ||
				    sizes[2] < output_size / 4)
					return -EINVAL;
				break;
			default:
				return -EINVAL;
			}

			process_num_buffers(q, sess, num_buffers, false);
			break;
		}

		return 0;
	}

	switch (q->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		sizes[0] = sess->src_buffer_size;
		*num_planes = 1;
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		switch (sess->pixfmt_cap) {
		case V4L2_PIX_FMT_NV12M:
			sizes[0] = output_size;
			sizes[1] = output_size / 2;
			*num_planes = 2;
			break;
		case V4L2_PIX_FMT_YUV420M:
			sizes[0] = output_size;
			sizes[1] = output_size / 4;
			sizes[2] = output_size / 4;
			*num_planes = 3;
			break;
		default:
			return -EINVAL;
		}

		process_num_buffers(q, sess, num_buffers, true);
		break;
	default:
		return -EINVAL;
	}

	sess->changed_format = 1;
	return 0;
}

static void vdec_vb2_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct amvdec_session *sess = vb2_get_drv_priv(vb->vb2_queue);
	struct v4l2_m2m_ctx *m2m_ctx = sess->m2m_ctx;

	if (vb->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE && sess->eos_completed) {
		unsigned int plane;

		for (plane = 0; plane < vb->num_planes; plane++)
			vb2_set_plane_payload(vb, plane, 0);
		v4l2_m2m_last_buffer_done(m2m_ctx, vbuf);
		return;
	}
	v4l2_m2m_buf_queue(m2m_ctx, vbuf);

	if (!sess->streamon_out)
		return;

	if (sess->streamon_cap &&
	    vb->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE &&
	    vdec_codec_needs_recycle(sess) &&
	    (sess->core->platform->revision != VDEC_REVISION_SC2 ||
	     sess->status == STATUS_RUNNING))
		vdec_queue_recycle(sess, vb);

	schedule_work(&sess->esparser_queue_work);
}

static void vdec_reset_decode_state(struct amvdec_session *sess)
{
	sess->should_stop = 0;
	sess->aborted = false;
	sess->eos_completed = false;
	sess->keyframe_found = 0;
	sess->last_offset = 0;
	sess->wrap_count = 0;
	sess->pixelaspect.numerator = 1;
	sess->pixelaspect.denominator = 1;
	sess->stream_write = 0;
	sess->stream_offset = 0;
	sess->status = STATUS_INIT;
	sess->core->cur_sess = sess;
	atomic_set(&sess->esparser_queued_bufs, 0);
	v4l2_ctrl_s_ctrl(sess->ctrl_min_buf_capture, 1);
}

static int vdec_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct amvdec_session *sess = vb2_get_drv_priv(q);
	struct amvdec_codec_ops *codec_ops = sess->fmt_out->codec_ops;
	struct amvdec_core *core = sess->core;
	struct vb2_v4l2_buffer *buf;
	int ret;

	if (core->cur_sess && core->cur_sess != sess) {
		ret = -EBUSY;
		goto bufs_done;
	}

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		sess->streamon_out = 1;
	else
		sess->streamon_cap = 1;

	if (!sess->streamon_out)
		return 0;

	if (sess->status == STATUS_NEEDS_RESUME &&
	    q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE &&
	    sess->changed_format) {
		codec_ops->resume(sess);
		if (sess->aborted)
			goto resume_failed;
		if (core->platform->revision == VDEC_REVISION_SC2 &&
		    !codec_ops->manages_processor &&
		    sess->fmt_out->needs_capture && !sess->processor_started) {
			amvdec_write_dos(core, MPSR, 1);
			sess->processor_started = true;
		}
		sess->status = STATUS_RUNNING;
		return 0;
	}

	if (sess->status == STATUS_RUNNING ||
	    sess->status == STATUS_NEEDS_RESUME ||
	    sess->status == STATUS_INIT)
		return 0;

	sess->vififo_size = SIZE_VIFIFO;
	sess->vififo_vaddr =
		dma_alloc_coherent(sess->core->dev, sess->vififo_size,
				   &sess->vififo_paddr, GFP_KERNEL);
	if (!sess->vififo_vaddr) {
		dev_err(sess->core->dev, "Failed to request VIFIFO buffer\n");
		ret = -ENOMEM;
		goto bufs_done;
	}

	vdec_reset_decode_state(sess);

	ret = vdec_poweron(sess);
	if (ret)
		goto vififo_free;

	sess->sequence_cap = 0;
	sess->sequence_out = 0;
	if (vdec_codec_needs_recycle(sess)) {
		sess->recycle_thread = kthread_run(vdec_recycle_thread, sess,
						 "vdec_recycle");
		if (IS_ERR(sess->recycle_thread)) {
			ret = PTR_ERR(sess->recycle_thread);
			vdec_poweroff(sess);
			goto vififo_free;
		}
	}

	if (sess->status == STATUS_INIT && sess->streamon_cap &&
	    sess->fmt_out->needs_capture && codec_ops->resume) {
		if (!sess->processor_started) {
			codec_ops->resume(sess);
			if (sess->aborted)
				goto resume_failed;
			if (core->platform->revision == VDEC_REVISION_SC2 &&
			    !codec_ops->manages_processor) {
				amvdec_write_dos(core, MPSR, 1);
				sess->processor_started = true;
			}
		}
		sess->status = STATUS_RUNNING;
	}
	schedule_work(&sess->esparser_queue_work);
	return 0;

resume_failed:
	ret = -EIO;
	if (!IS_ERR_OR_NULL(sess->recycle_thread))
		kthread_stop(sess->recycle_thread);
	if (vdec_poweroff(sess))
		return ret;
vififo_free:
	if (core->quarantined)
		return ret;
	vdec_free_canvas(sess);
	core->cur_sess = NULL;
	sess->status = STATUS_STOPPED;
	kfree(sess->priv);
	sess->priv = NULL;
	dma_free_coherent(sess->core->dev, sess->vififo_size,
			  sess->vififo_vaddr, sess->vififo_paddr);
bufs_done:
	while ((buf = v4l2_m2m_src_buf_remove(sess->m2m_ctx)))
		v4l2_m2m_buf_done(buf, VB2_BUF_STATE_QUEUED);
	while ((buf = v4l2_m2m_dst_buf_remove(sess->m2m_ctx)))
		v4l2_m2m_buf_done(buf, VB2_BUF_STATE_QUEUED);

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		sess->streamon_out = 0;
	else
		sess->streamon_cap = 0;

	return ret;
}

static void vdec_free_canvas(struct amvdec_session *sess)
{
	int i;

	for (i = 0; i < sess->canvas_num; ++i)
		meson_canvas_free(sess->core->canvas, sess->canvas_alloc[i]);

	sess->canvas_num = 0;
}

static void vdec_reset_timestamps(struct amvdec_session *sess)
{
	struct amvdec_timestamp *tmp, *n;

	list_for_each_entry_safe(tmp, n, &sess->timestamps, list) {
		list_del(&tmp->list);
		kfree(tmp);
	}
}

static void vdec_reset_bufs_recycle(struct amvdec_session *sess)
{
	struct amvdec_buffer *tmp, *n;

	list_for_each_entry_safe(tmp, n, &sess->bufs_recycle, list) {
		list_del(&tmp->list);
		kfree(tmp);
	}
}

static void vdec_stop_streaming(struct vb2_queue *q)
{
	struct amvdec_session *sess = vb2_get_drv_priv(q);
	struct amvdec_codec_ops *codec_ops = sess->fmt_out->codec_ops;
	struct amvdec_core *core = sess->core;
	struct vb2_v4l2_buffer *buf;
	sess->draining = false;
	cancel_delayed_work_sync(&sess->drain_work);

	if (sess->status == STATUS_RUNNING ||
	    sess->status == STATUS_INIT ||
	    (sess->status == STATUS_NEEDS_RESUME &&
	     (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE ||
	      !sess->streamon_out || !sess->streamon_cap))) {
		if (!IS_ERR_OR_NULL(sess->recycle_thread))
			kthread_stop(sess->recycle_thread);

		if (vdec_poweroff(sess))
			return;
		vdec_free_canvas(sess);
		dma_free_coherent(sess->core->dev, sess->vififo_size,
				  sess->vififo_vaddr, sess->vififo_paddr);
		vdec_reset_timestamps(sess);
		vdec_reset_bufs_recycle(sess);
		kfree(sess->priv);
		sess->priv = NULL;
		core->cur_sess = NULL;
		sess->status = STATUS_STOPPED;
	}

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		while ((buf = v4l2_m2m_src_buf_remove(sess->m2m_ctx)))
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);

		sess->streamon_out = 0;
	} else {
		/* Drain remaining refs if was still running */
		if (sess->status >= STATUS_RUNNING && codec_ops->drain)
			codec_ops->drain(sess);

		while ((buf = v4l2_m2m_dst_buf_remove(sess->m2m_ctx)))
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);

		sess->streamon_cap = 0;
	}
}

static int vdec_vb2_buf_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	vbuf->field = V4L2_FIELD_NONE;
	return 0;
}

static const struct vb2_ops vdec_vb2_ops = {
	.queue_setup = vdec_queue_setup,
	.start_streaming = vdec_start_streaming,
	.stop_streaming = vdec_stop_streaming,
	.buf_queue = vdec_vb2_buf_queue,
	.buf_prepare = vdec_vb2_buf_prepare,
};

static int
vdec_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	strscpy(cap->driver, file_to_amvdec_session(file)->core->platform->revision ==
		VDEC_REVISION_SC2 ? "meson-sc2-vdec" : "meson-vdec", sizeof(cap->driver));
	strscpy(cap->card, "Amlogic Video Decoder", sizeof(cap->card));
	strscpy(cap->bus_info, "platform:meson-vdec", sizeof(cap->bus_info));

	return 0;
}

static const struct amvdec_format *
find_format(const struct amvdec_format *fmts, u32 size, u32 pixfmt)
{
	unsigned int i;

	for (i = 0; i < size; i++) {
		if (fmts[i].pixfmt == pixfmt)
			return &fmts[i];
	}

	return NULL;
}

static unsigned int
vdec_supports_pixfmt_cap(const struct amvdec_format *fmt_out, u32 pixfmt_cap)
{
	int i;

	for (i = 0; fmt_out->pixfmts_cap[i]; i++)
		if (fmt_out->pixfmts_cap[i] == pixfmt_cap)
			return 1;

	return 0;
}

static const struct amvdec_format *
vdec_try_fmt_common(struct amvdec_session *sess, u32 size,
		    struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pixmp = &f->fmt.pix_mp;
	struct v4l2_plane_pix_format *pfmt = pixmp->plane_fmt;
	const struct amvdec_format *fmts = sess->core->platform->formats;
	const struct amvdec_format *fmt_out = NULL;
	u32 output_size = 0;
	u32 stride;

	memset(pfmt[0].reserved, 0, sizeof(pfmt[0].reserved));
	memset(pixmp->reserved, 0, sizeof(pixmp->reserved));

	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		fmt_out = find_format(fmts, size, pixmp->pixelformat);
		if (!fmt_out) {
			pixmp->pixelformat = fmts[0].pixfmt;
			fmt_out = &fmts[0];
		}
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		fmt_out = sess->fmt_out;
		break;
	default:
		return NULL;
	}

	pixmp->width = clamp(pixmp->width,
		sess->core->platform->revision == VDEC_REVISION_SC2 ? 64U : 256U,
		fmt_out->max_width);
	pixmp->height = clamp(pixmp->height,
		sess->core->platform->revision == VDEC_REVISION_SC2 ? 64U : 144U,
		fmt_out->max_height);
	stride = ALIGN(fmt_out->capture_width ?: pixmp->width, 32);
	output_size = get_output_size(stride,
			fmt_out->capture_height ?: pixmp->height);

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		if (!pfmt[0].sizeimage)
			pfmt[0].sizeimage = sess->src_buffer_size;
		pfmt[0].bytesperline = 0;
		pixmp->num_planes = 1;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		fmt_out = sess->fmt_out;
		if (!vdec_supports_pixfmt_cap(fmt_out, pixmp->pixelformat))
			pixmp->pixelformat = fmt_out->pixfmts_cap[0];

		memset(pfmt[1].reserved, 0, sizeof(pfmt[1].reserved));
		if (pixmp->pixelformat == V4L2_PIX_FMT_NV12M) {
			pfmt[0].sizeimage = output_size;
			pfmt[0].bytesperline = stride;

			pfmt[1].sizeimage = output_size / 2;
			pfmt[1].bytesperline = stride;
			pixmp->num_planes = 2;
		} else if (pixmp->pixelformat == V4L2_PIX_FMT_YUV420M) {
			pfmt[0].sizeimage = output_size;
			pfmt[0].bytesperline = stride;

			pfmt[1].sizeimage = output_size / 4;
			pfmt[1].bytesperline = stride / 2;

			pfmt[2].sizeimage = output_size / 4;
			pfmt[2].bytesperline = stride / 2;
			pixmp->num_planes = 3;
		}
	}

	if (pixmp->field == V4L2_FIELD_ANY)
		pixmp->field = V4L2_FIELD_NONE;

	return fmt_out;
}

static int vdec_try_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct amvdec_session *sess = file_to_amvdec_session(file);

	vdec_try_fmt_common(sess, sess->core->platform->num_formats, f);

	return 0;
}

static int vdec_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct amvdec_session *sess = file_to_amvdec_session(file);
	struct v4l2_pix_format_mplane *pixmp = &f->fmt.pix_mp;

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		pixmp->pixelformat = sess->pixfmt_cap;
	else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		pixmp->pixelformat = sess->fmt_out->pixfmt;

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		pixmp->width = sess->width;
		pixmp->height = sess->height;
		pixmp->colorspace = sess->colorspace;
		pixmp->ycbcr_enc = sess->ycbcr_enc;
		pixmp->quantization = sess->quantization;
		pixmp->xfer_func = sess->xfer_func;
		pixmp->field = sess->field ?: V4L2_FIELD_NONE;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		pixmp->width = sess->width;
		pixmp->height = sess->height;
	}

	vdec_try_fmt_common(sess, sess->core->platform->num_formats, f);

	return 0;
}

static int vdec_s_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct amvdec_session *sess = file_to_amvdec_session(file);
	struct v4l2_pix_format_mplane *pixmp = &f->fmt.pix_mp;
	u32 num_formats = sess->core->platform->num_formats;
	const struct amvdec_format *fmt_out;
	struct v4l2_pix_format_mplane orig_pixmp;
	struct v4l2_format format;
	u32 pixfmt_out = 0, pixfmt_cap = 0;
	struct vb2_queue *queue = v4l2_m2m_get_vq(sess->m2m_ctx, f->type);

	if (!queue)
		return -EINVAL;
	if (vb2_is_busy(queue))
		return -EBUSY;
	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE &&
	    vb2_is_busy(v4l2_m2m_get_vq(sess->m2m_ctx,
				      V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)))
		return -EBUSY;
	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE &&
	    sess->status != STATUS_STOPPED) {
		pixmp->width = sess->width;
		pixmp->height = sess->height;
	}

	orig_pixmp = *pixmp;

	fmt_out = vdec_try_fmt_common(sess, num_formats, f);
	if (!fmt_out)
		return -EINVAL;

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		pixfmt_out = pixmp->pixelformat;
		pixfmt_cap = sess->pixfmt_cap;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		pixfmt_cap = pixmp->pixelformat;
		pixfmt_out = sess->fmt_out->pixfmt;
	}

	memset(&format, 0, sizeof(format));

	format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	format.fmt.pix_mp.pixelformat = pixfmt_out;
	format.fmt.pix_mp.width = orig_pixmp.width;
	format.fmt.pix_mp.height = orig_pixmp.height;
	vdec_try_fmt_common(sess, num_formats, &format);

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		sess->width = format.fmt.pix_mp.width;
		sess->height = format.fmt.pix_mp.height;
		sess->colorspace = pixmp->colorspace;
		sess->ycbcr_enc = pixmp->ycbcr_enc;
		sess->quantization = pixmp->quantization;
		sess->xfer_func = pixmp->xfer_func;
		sess->src_buffer_size = pixmp->plane_fmt[0].sizeimage;
		sess->visible_width = 0;
		sess->visible_height = 0;
		sess->crop_left = 0;
		sess->crop_top = 0;
		sess->fmt_out = fmt_out;
	}

	memset(&format, 0, sizeof(format));

	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	format.fmt.pix_mp.pixelformat = pixfmt_cap;
	format.fmt.pix_mp.width = pixmp->width;
	format.fmt.pix_mp.height = pixmp->height;
	vdec_try_fmt_common(sess, num_formats, &format);

	sess->width = format.fmt.pix_mp.width;
	sess->height = format.fmt.pix_mp.height;

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		sess->fmt_out = fmt_out;
	else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		sess->pixfmt_cap = format.fmt.pix_mp.pixelformat;

	return 0;
}

static int vdec_enum_fmt(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	struct amvdec_session *sess = file_to_amvdec_session(file);
	const struct vdec_platform *platform = sess->core->platform;
	const struct amvdec_format *fmt_out;

	memset(f->reserved, 0, sizeof(f->reserved));

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		if (f->index >= platform->num_formats)
			return -EINVAL;

		fmt_out = &platform->formats[f->index];
		f->pixelformat = fmt_out->pixfmt;
		f->flags = fmt_out->flags;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		fmt_out = sess->fmt_out;
		if (f->index >= 4 || !fmt_out->pixfmts_cap[f->index])
			return -EINVAL;

		f->pixelformat = fmt_out->pixfmts_cap[f->index];
	} else {
		return -EINVAL;
	}

	return 0;
}

static int vdec_enum_framesizes(struct file *file, void *fh,
				struct v4l2_frmsizeenum *fsize)
{
	struct amvdec_session *sess = file_to_amvdec_session(file);
	const struct amvdec_format *formats = sess->core->platform->formats;
	const struct amvdec_format *fmt;
	u32 num_formats = sess->core->platform->num_formats;

	fmt = find_format(formats, num_formats, fsize->pixel_format);
	if (!fmt && sess->core->platform->revision == VDEC_REVISION_SC2 &&
	    vdec_supports_pixfmt_cap(sess->fmt_out, fsize->pixel_format))
		fmt = sess->fmt_out;
	if (!fmt || fsize->index)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;

	fsize->stepwise.min_width = sess->core->platform->revision ==
		VDEC_REVISION_SC2 ? 64 : 256;
	fsize->stepwise.max_width = fmt->max_width;
	fsize->stepwise.step_width = 1;
	fsize->stepwise.min_height = sess->core->platform->revision ==
		VDEC_REVISION_SC2 ? 64 : 144;
	fsize->stepwise.max_height = fmt->max_height;
	fsize->stepwise.step_height = 1;

	return 0;
}

static void vdec_sc2_eos_work(struct work_struct *work)
{
	struct amvdec_session *sess = container_of(to_delayed_work(work),
						 struct amvdec_session, drain_work);
	struct amvdec_codec_ops *codec = sess->fmt_out->codec_ops;
	const u8 *sequence;
	u32 length, end, read, remaining;
	int ret;

	if (!READ_ONCE(sess->draining))
		return;
	/* VB2 stop holds the queue mutex while canceling this work. Never wait
	 * for that mutex here, and never block DQBUF/QBUF in DECODER_CMD.
	 */
	if (!mutex_trylock(&sess->lock))
		goto retry;
	if (sess->eos_completed || sess->status == STATUS_STOPPED) {
		sess->draining = false;
		goto unlock;
	}
	if (time_after_eq(jiffies, sess->drain_deadline)) {
		amvdec_abort(sess);
		sess->draining = false;
		goto unlock;
	}
	/* STOP first consumes every OUTPUT buffer queued before the command. */
	if (v4l2_m2m_num_src_bufs_ready(sess->m2m_ctx)) {
		schedule_work(&sess->esparser_queue_work);
		goto unlock_retry;
	}
	if (codec->drain_step) {
		sess->should_stop = 1;
		WRITE_ONCE(sess->eos_queued, true);
		ret = codec->drain_step(sess);
		if (ret) {
			if (ret < 0)
				amvdec_abort(sess);
			sess->draining = false;
			goto unlock;
		}
		goto unlock_retry;
	}
	if (!sess->eos_queued) {
		if (!codec->eos_sequence) {
			amvdec_abort(sess);
			sess->draining = false;
			goto unlock;
		}
		sequence = codec->eos_sequence(&length);
		end = length;
		while (end && !sequence[end - 1])
			end--;
		sess->drain_padding = length - end;
		sess->should_stop = 1;
		WRITE_ONCE(sess->eos_queued, true);
		ret = esparser_queue_eos(sess->core, sequence, length);
		dev_dbg(sess->core->dev,
			"EOS queue ret=%d len=%u state=%u src=%u pending=%d\n",
			ret, length, sess->status,
			v4l2_m2m_num_src_bufs_ready(sess->m2m_ctx),
			atomic_read(&sess->esparser_queued_bufs));
		if (ret == -EAGAIN) {
			WRITE_ONCE(sess->eos_queued, false);
			goto unlock_retry;
		}
		if (ret < 0) {
			amvdec_abort(sess);
			sess->draining = false;
			goto unlock;
		}
	}
	/* A manual VIFIFO retains a partial fetch burst of zero padding, so its
	 * level need not reach zero. Require the read pointer to have crossed
	 * every coded byte and the sequence-end marker, every tracked input
	 * presentation to be returned, and the output mailbox to be acknowledged.
	 * This does not replace the separate DMC/DMA idle proof at STREAMOFF.
	 * H264 uses its explicit frame-status EOS acknowledgement instead.
	 */
	read = amvdec_read_dos(sess->core, VLD_MEM_VIFIFO_RP);
	if (sess->fmt_out->pixfmt != V4L2_PIX_FMT_H264 &&
	    read >= sess->vififo_paddr &&
	    read < sess->vififo_paddr + sess->vififo_size) {
		remaining = (sess->stream_write + sess->vififo_size -
			     (read - sess->vififo_paddr)) % sess->vififo_size;
		if (remaining <= sess->drain_padding &&
		    !atomic_read(&sess->esparser_queued_bufs) &&
		    !amvdec_read_dos(sess->core, AV_SCRATCH_9)) {
			amvdec_eos(sess);
			sess->draining = false;
			goto unlock;
		}
	}
unlock_retry:
	if (sess->eos_queued)
		dev_dbg(sess->core->dev,
			"EOS wait done=%u state=%u pending=%d scratch=%08x/%08x/%08x fifo=%08x/%08x/%08x\n",
			sess->eos_completed, sess->status,
			atomic_read(&sess->esparser_queued_bufs),
			amvdec_read_dos(sess->core, AV_SCRATCH_0),
			amvdec_read_dos(sess->core, AV_SCRATCH_1),
			amvdec_read_dos(sess->core, AV_SCRATCH_A),
			amvdec_read_dos(sess->core, VLD_MEM_VIFIFO_RP),
			amvdec_read_dos(sess->core, VLD_MEM_VIFIFO_WP),
			amvdec_read_dos(sess->core, VLD_MEM_VIFIFO_LEVEL));
	mutex_unlock(&sess->lock);
retry:
	if (READ_ONCE(sess->draining))
		schedule_delayed_work(&sess->drain_work, msecs_to_jiffies(5));
	return;
unlock:
	mutex_unlock(&sess->lock);
}

/* LAST transfers a surface to userspace; it cannot stay in firmware's free
 * pool across START. Reset both processor and pool ownership after a drain.
 * Keep VIFIFO allocated and let the normal source/capture handshake rebuild
 * canvases only from buffers actually returned by userspace.
 */
static int vdec_sc2_restart(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	int ret;

	if (sess->aborted)
		return -EIO;
	if (!IS_ERR_OR_NULL(sess->recycle_thread)) {
		kthread_stop(sess->recycle_thread);
		sess->recycle_thread = NULL;
	}
	ret = vdec_poweroff(sess);
	if (ret)
		return ret;
	vdec_free_canvas(sess);
	vdec_reset_timestamps(sess);
	vdec_reset_bufs_recycle(sess);
	kfree(sess->priv);
	sess->priv = NULL;
	vdec_reset_decode_state(sess);
	ret = vdec_poweron(sess);
	if (ret)
		goto fail;
	if (vdec_codec_needs_recycle(sess)) {
		sess->recycle_thread = kthread_run(vdec_recycle_thread, sess,
						 "vdec_recycle");
		if (IS_ERR(sess->recycle_thread)) {
			ret = PTR_ERR(sess->recycle_thread);
			if (vdec_poweroff(sess))
				return ret;
			goto fail;
		}
	}
	if (sess->status == STATUS_INIT && sess->fmt_out->needs_capture &&
	    sess->processor_started)
		sess->status = STATUS_RUNNING;
	schedule_work(&sess->esparser_queue_work);
	return 0;

fail:
	amvdec_abort(sess);
	if (!core->quarantined) {
		vdec_free_canvas(sess);
		kfree(sess->priv);
		sess->priv = NULL;
		dma_free_coherent(core->dev, sess->vififo_size,
				  sess->vififo_vaddr, sess->vififo_paddr);
		core->cur_sess = NULL;
		sess->status = STATUS_STOPPED;
	}
	return ret;
}

static int
vdec_decoder_cmd(struct file *file, void *fh, struct v4l2_decoder_cmd *cmd)
{
	struct amvdec_session *sess = file_to_amvdec_session(file);
	struct amvdec_codec_ops *codec_ops = sess->fmt_out->codec_ops;
	struct device *dev = sess->core->dev;
	int ret;

	ret = v4l2_m2m_ioctl_try_decoder_cmd(file, fh, cmd);
	if (ret)
		return ret;

	if (!(sess->streamon_out & sess->streamon_cap))
		return 0;

	if (cmd->cmd == V4L2_DEC_CMD_START) {
		if (sess->draining)
			return -EBUSY;
		if (sess->core->platform->revision == VDEC_REVISION_SC2 &&
		    sess->eos_completed) {
			ret = vdec_sc2_restart(sess);
			if (ret)
				return ret;
		}
		sess->eos_queued = false;
		v4l2_m2m_clear_state(sess->m2m_ctx);
		sess->should_stop = 0;
		sess->eos_completed = false;
		return 0;
	}

	/* Should not happen */
	if (cmd->cmd != V4L2_DEC_CMD_STOP)
		return -EINVAL;

	dev_dbg(dev, "Received V4L2_DEC_CMD_STOP\n");
	if (sess->core->platform->revision == VDEC_REVISION_SC2) {
		if (sess->draining || sess->eos_completed)
			return 0;
		sess->draining = true;
		sess->eos_queued = false;
		sess->drain_padding = 0;
		sess->drain_deadline = jiffies + msecs_to_jiffies(5000);
		schedule_delayed_work(&sess->drain_work, 0);
		return 0;
	}

	sess->should_stop = 1;

	v4l2_m2m_mark_stopped(sess->m2m_ctx);

	if (codec_ops->drain) {
		vdec_wait_inactive(sess);
		codec_ops->drain(sess);
	} else if (codec_ops->eos_sequence) {
		u32 len;
		const u8 *data = codec_ops->eos_sequence(&len);

		esparser_queue_eos(sess->core, data, len);
		vdec_wait_inactive(sess);
	}

	return ret;
}

static int vdec_subscribe_event(struct v4l2_fh *fh,
				const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_EOS:
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subscribe_event(fh, sub);
	default:
		return -EINVAL;
	}
}

static int vdec_g_pixelaspect(struct file *file, void *fh, int type,
			      struct v4l2_fract *f)
{
	struct amvdec_session *sess = file_to_amvdec_session(file);

	if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	*f = sess->pixelaspect;
	return 0;
}

static int vdec_g_selection(struct file *file, void *fh,
			    struct v4l2_selection *selection)
{
	struct amvdec_session *sess = file_to_amvdec_session(file);

	if (selection->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE &&
	    selection->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	selection->r.left = 0;
	selection->r.top = 0;
	switch (selection->target) {
	case V4L2_SEL_TGT_COMPOSE:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
		selection->r.left = sess->crop_left;
		selection->r.top = sess->crop_top;
		selection->r.width = sess->visible_width ?: sess->width;
		selection->r.height = sess->visible_height ?: sess->height;
		return 0;
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	case V4L2_SEL_TGT_COMPOSE_PADDED:
		selection->r.width = amvdec_get_output_width(sess);
		selection->r.height = amvdec_get_output_height(sess);
		return 0;
	default:
		return -EINVAL;
	}
}

static int vdec_g_parm(struct file *file, void *fh, struct v4l2_streamparm *parm)
{
	struct amvdec_session *sess = file_to_amvdec_session(file);

	if (!sess->timeperframe.numerator || !sess->timeperframe.denominator)
		return -EINVAL;
	if (parm->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE ||
	    parm->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		parm->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
		parm->parm.capture.timeperframe = sess->timeperframe;
	} else if (parm->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE ||
		   parm->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		parm->parm.output.capability = V4L2_CAP_TIMEPERFRAME;
		parm->parm.output.timeperframe = sess->timeperframe;
	} else {
		return -EINVAL;
	}
	return 0;
}

static int vdec_s_parm(struct file *file, void *fh, struct v4l2_streamparm *parm)
{
	struct amvdec_session *sess = file_to_amvdec_session(file);
	struct v4l2_fract period;

	if (parm->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return -EINVAL;
	period = parm->parm.output.timeperframe;
	if (!period.numerator || !period.denominator)
		return -EINVAL;
	sess->timeperframe = period;
	return vdec_g_parm(file, fh, parm);
}

static const struct v4l2_ioctl_ops vdec_ioctl_ops = {
	.vidioc_querycap = vdec_querycap,
	.vidioc_enum_fmt_vid_cap = vdec_enum_fmt,
	.vidioc_enum_fmt_vid_out = vdec_enum_fmt,
	.vidioc_s_fmt_vid_cap_mplane = vdec_s_fmt,
	.vidioc_s_fmt_vid_out_mplane = vdec_s_fmt,
	.vidioc_g_fmt_vid_cap_mplane = vdec_g_fmt,
	.vidioc_g_fmt_vid_out_mplane = vdec_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane = vdec_try_fmt,
	.vidioc_try_fmt_vid_out_mplane = vdec_try_fmt,
	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
	.vidioc_enum_framesizes = vdec_enum_framesizes,
	.vidioc_subscribe_event = vdec_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
	.vidioc_try_decoder_cmd = v4l2_m2m_ioctl_try_decoder_cmd,
	.vidioc_decoder_cmd = vdec_decoder_cmd,
	.vidioc_g_pixelaspect = vdec_g_pixelaspect,
	.vidioc_g_selection = vdec_g_selection,
	.vidioc_g_parm = vdec_g_parm,
	.vidioc_s_parm = vdec_s_parm,
};

static int m2m_queue_init(void *priv, struct vb2_queue *src_vq,
			  struct vb2_queue *dst_vq)
{
	struct amvdec_session *sess = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->ops = &vdec_vb2_ops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->drv_priv = sess;
	src_vq->buf_struct_size = sizeof(struct dummy_buf);
	src_vq->min_queued_buffers = 1;
	src_vq->dev = sess->core->dev;
	src_vq->lock = &sess->lock;
	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->ops = &vdec_vb2_ops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->drv_priv = sess;
	dst_vq->buf_struct_size = sizeof(struct dummy_buf);
	dst_vq->min_queued_buffers = 1;
	dst_vq->dev = sess->core->dev;
	dst_vq->lock = &sess->lock;
	return vb2_queue_init(dst_vq);
}

static int vdec_init_ctrls(struct amvdec_session *sess)
{
	struct v4l2_ctrl_handler *ctrl_handler = &sess->ctrl_handler;
	int ret;

	ret = v4l2_ctrl_handler_init(ctrl_handler, 1);
	if (ret)
		return ret;

	sess->ctrl_min_buf_capture =
		v4l2_ctrl_new_std(ctrl_handler, NULL,
				  V4L2_CID_MIN_BUFFERS_FOR_CAPTURE, 1, 32, 1,
				  1);

	ret = ctrl_handler->error;
	if (ret) {
		v4l2_ctrl_handler_free(ctrl_handler);
		return ret;
	}

	return 0;
}

static int vdec_open(struct file *file)
{
	struct amvdec_core *core = video_drvdata(file);
	struct device *dev = core->dev;
	const struct amvdec_format *formats = core->platform->formats;
	struct amvdec_session *sess;
	int ret;

	sess = kzalloc(sizeof(*sess), GFP_KERNEL);
	if (!sess)
		return -ENOMEM;

	sess->core = core;

	sess->m2m_dev = v4l2_m2m_init(&vdec_m2m_ops);
	if (IS_ERR(sess->m2m_dev)) {
		dev_err(dev, "Fail to v4l2_m2m_init\n");
		ret = PTR_ERR(sess->m2m_dev);
		goto err_free_sess;
	}

	sess->m2m_ctx = v4l2_m2m_ctx_init(sess->m2m_dev, sess, m2m_queue_init);
	if (IS_ERR(sess->m2m_ctx)) {
		dev_err(dev, "Fail to v4l2_m2m_ctx_init\n");
		ret = PTR_ERR(sess->m2m_ctx);
		goto err_m2m_release;
	}

	ret = vdec_init_ctrls(sess);
	if (ret)
		goto err_m2m_ctx_release;

	sess->pixfmt_cap = formats[0].pixfmts_cap[0];
	sess->fmt_out = &formats[0];
	sess->width = 1280;
	sess->height = 720;
	sess->pixelaspect.numerator = 1;
	sess->pixelaspect.denominator = 1;
	sess->src_buffer_size = SZ_1M;

	INIT_LIST_HEAD(&sess->timestamps);
	INIT_LIST_HEAD(&sess->bufs_recycle);
	INIT_WORK(&sess->esparser_queue_work, esparser_queue_all_src);
	INIT_DELAYED_WORK(&sess->drain_work, vdec_sc2_eos_work);
	mutex_init(&sess->lock);
	mutex_init(&sess->bufs_recycle_lock);
	spin_lock_init(&sess->ts_spinlock);

	v4l2_fh_init(&sess->fh, core->vdev_dec);
	sess->fh.ctrl_handler = &sess->ctrl_handler;
	v4l2_fh_add(&sess->fh, file);
	sess->fh.m2m_ctx = sess->m2m_ctx;

	return 0;

err_m2m_ctx_release:
	v4l2_m2m_ctx_release(sess->m2m_ctx);
err_m2m_release:
	v4l2_m2m_release(sess->m2m_dev);
err_free_sess:
	kfree(sess);
	return ret;
}

static int vdec_close(struct file *file)
{
	struct amvdec_session *sess = file_to_amvdec_session(file);
	if (sess->core->cur_sess == sess && !sess->core->quarantined) {
		mutex_lock(&sess->lock);
		sess->streamon_out = 0;
		vdec_stop_streaming(v4l2_m2m_get_vq(sess->m2m_ctx,
					 V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE));
		mutex_unlock(&sess->lock);
	}
	cancel_work_sync(&sess->esparser_queue_work);
	sess->draining = false;
	cancel_delayed_work_sync(&sess->drain_work);

	if (sess->core->quarantined && sess->core->cur_sess == sess)
		return 0;
	v4l2_m2m_ctx_release(sess->m2m_ctx);
	v4l2_m2m_release(sess->m2m_dev);
	v4l2_fh_del(&sess->fh, file);
	v4l2_fh_exit(&sess->fh);

	mutex_destroy(&sess->lock);
	mutex_destroy(&sess->bufs_recycle_lock);

	kfree(sess);

	return 0;
}

static long vdec_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct amvdec_session *sess = file_to_amvdec_session(file);

	if (sess->core->quarantined)
		return -EIO;
	return video_ioctl2(file, cmd, arg);
}

static const struct v4l2_file_operations vdec_fops = {
	.owner = THIS_MODULE,
	.open = vdec_open,
	.release = vdec_close,
	.unlocked_ioctl = vdec_ioctl,
	.poll = v4l2_m2m_fop_poll,
	.mmap = v4l2_m2m_fop_mmap,
};

static irqreturn_t vdec_isr(int irq, void *data)
{
	struct amvdec_core *core = data;
	struct amvdec_session *sess = core->cur_sess;
	if (!sess)
		return IRQ_NONE;

	sess->last_irq_jiffies = get_jiffies_64();

	return sess->fmt_out->codec_ops->isr(sess);
}

static irqreturn_t vdec_threaded_isr(int irq, void *data)
{
	struct amvdec_core *core = data;
	struct amvdec_session *sess = core->cur_sess;
	if (!sess)
		return IRQ_NONE;

	return sess->fmt_out->codec_ops->threaded_isr(sess);
}

static const struct of_device_id vdec_dt_match[] = {
	{ .compatible = "amlogic,gxbb-vdec",
	  .data = &vdec_platform_gxbb },
	{ .compatible = "amlogic,gxm-vdec",
	  .data = &vdec_platform_gxm },
	{ .compatible = "amlogic,gxl-vdec",
	  .data = &vdec_platform_gxl },
	{ .compatible = "amlogic,gxlx-vdec",
	  .data = &vdec_platform_gxlx },
	{ .compatible = "amlogic,g12a-vdec",
	  .data = &vdec_platform_g12a },
	{ .compatible = "amlogic,sm1-vdec",
	  .data = &vdec_platform_sm1 },
	{ .compatible = "amlogic,meson-sc2-vdec",
	  .data = &vdec_platform_sc2 },
	{}
};
MODULE_DEVICE_TABLE(of, vdec_dt_match);

static void vdec_disable_pm(void *data)
{
	pm_runtime_disable(data);
}

static int vdec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct video_device *vdev;
	struct amvdec_core *core;
	const struct of_device_id *of_id;
	struct resource *res;
	struct device_node *np;
	bool sc2;
	int irq;
	int ret;

	core = devm_kzalloc(dev, sizeof(*core), GFP_KERNEL);
	if (!core)
		return -ENOMEM;

	core->dev = dev;
	platform_set_drvdata(pdev, core);
	of_id = of_match_node(vdec_dt_match, dev->of_node);
	core->platform = of_id->data;
	sc2 = core->platform->revision == VDEC_REVISION_SC2;
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	if (sc2) {
		if (!IS_REACHABLE(CONFIG_MESON_SM))
			return -EOPNOTSUPP;
		/* Native HEVC owns the shared aperture reservation. The arbiter
		 * serializes all accesses, including secure loading and resets.
		 */
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dos");
		if (!res || resource_size(res) < SZ_64K)
			return -EINVAL;
		core->dos_base = devm_ioremap(dev, res->start, resource_size(res));
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dmc");
		if (!res || resource_size(res) < 0x100)
			return -EINVAL;
		core->dmc_base = devm_ioremap(dev, res->start, resource_size(res));
		if (!core->dos_base || !core->dmc_base)
			return -ENOMEM;
		np = of_find_compatible_node(NULL, NULL, "amlogic,meson-sc2-sm");
		if (!np)
			return -ENODEV;
		core->sm = meson_sm_get(np);
		of_node_put(np);
		if (!core->sm)
			return -EPROBE_DEFER;
		pm_runtime_enable(dev);
		ret = devm_add_action_or_reset(dev, vdec_disable_pm, dev);
		if (ret)
			return ret;
	} else {

	core->dos_base = devm_platform_ioremap_resource_byname(pdev, "dos");
	if (IS_ERR(core->dos_base))
		return PTR_ERR(core->dos_base);

	core->esparser_base = devm_platform_ioremap_resource_byname(pdev, "esparser");
	if (IS_ERR(core->esparser_base))
		return PTR_ERR(core->esparser_base);

	core->regmap_ao =
		syscon_regmap_lookup_by_phandle(dev->of_node,
						"amlogic,ao-sysctrl");
	if (IS_ERR(core->regmap_ao)) {
		dev_err(dev, "Couldn't regmap AO sysctrl\n");
		return PTR_ERR(core->regmap_ao);
	}
	}

	core->canvas = meson_canvas_get(dev);
	if (IS_ERR(core->canvas))
		return PTR_ERR(core->canvas);

	/* The platform revision was selected before resource acquisition. */

	if (core->platform->revision == VDEC_REVISION_G12A ||
	    core->platform->revision == VDEC_REVISION_SM1) {
		core->vdec_hevcf_clk = devm_clk_get(dev, "vdec_hevcf");
		if (IS_ERR(core->vdec_hevcf_clk))
			return -EPROBE_DEFER;
	}

	if (!sc2) {
		core->dos_parser_clk = devm_clk_get(dev, "dos_parser");
		if (IS_ERR(core->dos_parser_clk))
			return PTR_ERR(core->dos_parser_clk);
	}

	core->dos_clk = devm_clk_get(dev, "dos");
	if (IS_ERR(core->dos_clk))
		return -EPROBE_DEFER;

	core->vdec_1_clk = devm_clk_get(dev, "vdec_1");
	if (IS_ERR(core->vdec_1_clk))
		return -EPROBE_DEFER;

	if (!sc2) {
		core->vdec_hevc_clk = devm_clk_get(dev, "vdec_hevc");
		if (IS_ERR(core->vdec_hevc_clk))
			return PTR_ERR(core->vdec_hevc_clk);
	}

	irq = platform_get_irq_byname(pdev, "vdec");
	if (irq < 0)
		return irq;
	core->irq = irq;

	ret = devm_request_threaded_irq(core->dev, irq, vdec_isr,
					vdec_threaded_isr, IRQF_ONESHOT,
					"vdec", core);
	if (ret)
		return ret;
	if (sc2)
		disable_irq(irq);

	if (!sc2) {
		ret = esparser_init(pdev, core);
		if (ret)
			return ret;
	}

	ret = v4l2_device_register(dev, &core->v4l2_dev);
	if (ret) {
		dev_err(dev, "Couldn't register v4l2 device\n");
		return -ENOMEM;
	}

	vdev = video_device_alloc();
	if (!vdev) {
		ret = -ENOMEM;
		goto err_vdev_release;
	}

	core->vdev_dec = vdev;
	core->dev_dec = dev;
	mutex_init(&core->lock);

	strscpy(vdev->name, "meson-video-decoder", sizeof(vdev->name));
	vdev->release = video_device_release;
	vdev->fops = &vdec_fops;
	vdev->ioctl_ops = &vdec_ioctl_ops;
	vdev->vfl_dir = VFL_DIR_M2M;
	vdev->v4l2_dev = &core->v4l2_dev;
	vdev->lock = &core->lock;
	vdev->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;

	video_set_drvdata(vdev, core);

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(dev, "Failed registering video device\n");
		goto err_vdev_release;
	}

	return 0;

err_vdev_release:
	video_device_release(vdev);
	v4l2_device_unregister(&core->v4l2_dev);
	return ret;
}

static void vdec_remove(struct platform_device *pdev)
{
	struct amvdec_core *core = platform_get_drvdata(pdev);

	video_unregister_device(core->vdev_dec);
	v4l2_device_unregister(&core->v4l2_dev);
}

static struct platform_driver meson_vdec_driver = {
	.probe = vdec_probe,
	.remove = vdec_remove,
	.driver = {
		.name = "meson-vdec",
		.of_match_table = vdec_dt_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(meson_vdec_driver);

MODULE_DESCRIPTION("Meson video decoder driver for GXBB/GXL/GXM/G12/SM1/SC2");
MODULE_AUTHOR("Maxime Jourdan <mjourdan@baylibre.com>");
MODULE_LICENSE("GPL");
