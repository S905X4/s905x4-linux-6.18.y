// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Native SC2 DI post-engine, field-rate edge interpolation.
 * Register programming derived from Amlogic di_multi/di_hw_v3.c and
 * deinterlace_hw.c (Copyright (C) 2017 Amlogic, Inc.).
 * This driver does not implement the vendor temporal MA/MC algorithms.
 */
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/soc/amlogic/meson-canvas.h>
#include <linux/workqueue.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#define DI_NAME "meson-sc2-di"
#define DI_POST_CTRL 0x1701
#define DI_POST_SIZE 0x1702
#define DI_EI_CTRL3 0x1719
#define DI_INTR_CTRL 0x1730
#define DI_TOP_POST_CTRL 0x17c6
#define DI_POST_GL_CTRL 0x17c9
#define DI_POST_GL_THD 0x17ca
#define VIUB_SW_RESET 0x2001
#define VIUB_SW_RESET0 0x2002
#define VIUB_GCLK_CTRL0 0x2007
#define VIUB_GCLK_CTRL3 0x200a
#define DIWR_CANVAS 0x20f2
#define DIWR_X 0x20f4
#define DIWR_Y 0x20f5
#define DIWR_CTRL 0x20f6
#define IF0_BASE 0x5600
#define IF_GEN 0x00
#define IF_CANVAS 0x01
#define IF_X 0x03
#define IF_Y 0x04
#define IF_CX 0x05
#define IF_CY 0x06
#define IF_RPT 0x0b
#define IF_YPAT 0x0c
#define IF_CPAT 0x0d
#define IF_DUMMY 0x12
#define IF_FIFO 0x13
#define IF_GEN2 0x19
#define IF_GEN3 0x1c
#define IF_FMT 0x1d
#define IF_FMT_W 0x1e
#define DI_IRQ_POST BIT(2)
#define DI_IRQ_MASK (0xcfff0000 & ~BIT(18))

struct sc2_di;
struct sc2_di_ctx {
	struct v4l2_fh fh;
	struct sc2_di *di;
	struct list_head list;
	struct file *file;
	struct v4l2_pix_format_mplane fmt[2];
	struct v4l2_rect crop;
	struct v4l2_fract period;
	unsigned int field;
	u32 sequence;
	bool aborting;
};

struct sc2_di {
	struct device *dev;
	void __iomem *vcbus;
	void __iomem *power;
	struct clk_bulk_data clks[4];
	struct meson_canvas *canvas;
	u8 canvas_id[4];
	struct mutex lock;
	spinlock_t contexts_lock;
	struct list_head contexts;
	bool fatal;
	bool removing;
	struct v4l2_device v4l2;
	struct video_device video;
	struct v4l2_m2m_dev *m2m;
	struct work_struct work;
	struct completion complete;
	struct completion released;
	/* ACTIVE VB2 ownership retains both allocations and imported mappings. */
	struct vb2_v4l2_buffer *quarantine_src;
	struct vb2_v4l2_buffer *quarantine_dst;
	struct sc2_di_ctx *active;
	int irq;
	u32 saved_power, saved_gate0, saved_gate3;
};

static inline struct sc2_di_ctx *di_ctx(void *fh)
{
	return container_of(fh, struct sc2_di_ctx, fh);
}

static void di_write(struct sc2_di *di, u32 reg, u32 value)
{
	writel(value, di->vcbus + reg * 4);
}

static u32 di_read(struct sc2_di *di, u32 reg)
{
	return readl(di->vcbus + reg * 4);
}

static void di_update(struct sc2_di *di, u32 reg, u32 mask, u32 value)
{
	di_write(di, reg, (di_read(di, reg) & ~mask) | (value & mask));
}

static int di_stop(struct sc2_di *di)
{
	u32 value;
	int ret;

	di_write(di, DI_INTR_CTRL, 0xcfff0000);
	di_write(di, DI_POST_GL_CTRL, 0xc0000001);
	di_update(di, IF0_BASE + IF_GEN, BIT(0), 0);
	di_update(di, DIWR_CTRL, BIT(0), 0);
	/* SC2 DIWR pending_ddr_wrrsp must clear before returning a surface. */
	ret = readl_poll_timeout(di->vcbus + DIWR_CTRL * 4, value,
				 !(value & BIT(31)), 10, 100000);
	if (ret) {
		/* Vendor post-only asynchronous reset, never the shared VPU reset. */
		di_update(di, VIUB_SW_RESET, 0x483c0, 0x483c0);
		di_update(di, VIUB_SW_RESET, 0x483c0, 0);
		di_update(di, VIUB_SW_RESET0, 0x3001, 0x3001);
		di_update(di, VIUB_SW_RESET0, 0x3001, 0);
		ret = readl_poll_timeout(di->vcbus + DIWR_CTRL * 4, value,
					 !(value & BIT(31)), 10, 100000);
	}
	di_write(di, DI_POST_CTRL, 0x80000001);
	return ret;
}

static int di_canvas(struct sc2_di *di, unsigned int id, dma_addr_t addr,
		     u32 stride, u32 height)
{
	return meson_canvas_config(di->canvas, di->canvas_id[id], addr,
		stride, height, MESON_CANVAS_WRAP_NONE,
		MESON_CANVAS_BLKMODE_LINEAR, 0);
}

static int di_program(struct sc2_di_ctx *ctx,
		      struct vb2_v4l2_buffer *src,
		      struct vb2_v4l2_buffer *dst)
{
	struct sc2_di *di = ctx->di;
	const struct v4l2_rect *r = &ctx->crop;
	unsigned int bottom = (src->field == V4L2_FIELD_INTERLACED_BT) ^ ctx->field;
	unsigned int p, burst_len = 2;
	int ret;

	/* Decoder-backed frames retain their full canvas and field selection.
	 * Crop scopes and repeat patterns select source rows without CPU copies.
	 */
	for (p = 0; p < 2; p++) {
		u32 pitch = ctx->fmt[0].plane_fmt[p].bytesperline;
		u32 height = ctx->fmt[0].height >> p;
		dma_addr_t addr = vb2_dma_contig_plane_dma_addr(&src->vb2_buf, p) +
			src->vb2_buf.planes[p].data_offset;

		if (!IS_ALIGNED(pitch, 64))
			burst_len = 1;
		ret = di_canvas(di, p, addr, pitch, height);
		if (ret)
			return ret;
		addr = vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, p) +
			dst->vb2_buf.planes[p].data_offset;
		ret = di_canvas(di, p + 2, addr,
			ctx->fmt[1].plane_fmt[p].bytesperline, r->height >> p);
		if (ret)
			return ret;
	}

	di_write(di, DI_POST_GL_CTRL, 0x40200001);
	/* The newer EI filter requires at least 80 output pixels. */
	di_write(di, DI_EI_CTRL3, (r->width >= 80 ? BIT(31) : 0) | 0x13);
	di_write(di, IF0_BASE + IF_GEN3, (3 << 4) | (burst_len << 1));
	/* SC2 set_di_mif_v3: chroma map 1=VU, 2=UV. */
	di_write(di, IF0_BASE + IF_GEN2,
		 ctx->fmt[0].pixelformat == V4L2_PIX_FMT_NV21M ? 1 : 2);
	di_write(di, IF0_BASE + IF_CANVAS, di->canvas_id[0] |
		 (di->canvas_id[1] << 8) | (di->canvas_id[1] << 16));
	di_write(di, IF0_BASE + IF_X,
		 ((r->left + r->width - 1) << 16) | r->left);
	di_write(di, IF0_BASE + IF_Y,
		 ((r->top + r->height - 1) << 16) | (r->top + bottom));
	di_write(di, IF0_BASE + IF_CX,
		 ((r->left / 2 + r->width / 2 - 1) << 16) | (r->left / 2));
	di_write(di, IF0_BASE + IF_CY,
		 ((r->top / 2 + r->height / 2 - 1) << 16) |
		 (r->top / 2 + bottom));
	di_write(di, IF0_BASE + IF_RPT, 0x1111);
	di_write(di, IF0_BASE + IF_YPAT, 0x80);
	di_write(di, IF0_BASE + IF_CPAT, 0x80);
	di_write(di, IF0_BASE + IF_DUMMY, 0x00808000);
	di_write(di, IF0_BASE + IF_FIFO, 0x120);
	/* Ordinary IF0 field mode: phase0, step8; no decoder phase override. */
	di_write(di, IF0_BASE + IF_FMT,
		 BIT(21) | BIT(20) | BIT(17) | (8 << 1) | BIT(0));
	di_write(di, IF0_BASE + IF_FMT_W, (r->width << 16) | (r->width / 2));
	di_write(di, IF0_BASE + IF_GEN,
		 BIT(29) | BIT(25) | (8 << 19) | BIT(18) | BIT(6) |
		 BIT(4) | BIT(1) | BIT(0));
	di_write(di, DIWR_X, r->width - 1);
	di_write(di, DIWR_Y, r->height - 1);
	di_write(di, DIWR_CANVAS, di->canvas_id[2] | (di->canvas_id[3] << 8));
	/* SC2 set_wrmif_simple_v3: bit17=0 writes VU, bit17=1 writes UV.
	 * Input and output order are independent hardware MIF selections.
	 */
	/* The writer, not the destination canvas, orders linear output bytes. */
	di_write(di, DIWR_CTRL, BIT(0) | BIT(2) | BIT(3) | (3 << 4) |
		 (2 << 22) |
		 (2 << 26) |
		 (ctx->fmt[1].pixelformat == V4L2_PIX_FMT_NV12M ? BIT(17) : 0));
	di_write(di, DI_POST_GL_THD, 8 << 16);
	di_write(di, DI_POST_SIZE, ((r->height - 1) << 16) | (r->width - 1));
	/* post + EI + mux + DDR; blend and MC deliberately disabled. */
	di_write(di, DI_POST_CTRL,
		 BIT(0) | BIT(2) | BIT(3) | BIT(4) | (bottom << 29));
	di_write(di, DI_TOP_POST_CTRL, BIT(30) | BIT(0));
	di_write(di, DI_INTR_CTRL, DI_IRQ_MASK | 0xffff);
	di_write(di, DI_POST_GL_CTRL, 0x80200001);
	return 0;
}

static irqreturn_t di_irq(int irq, void *data)
{
	struct sc2_di *di = data;
	u32 status = di_read(di, DI_INTR_CTRL);

	if (!(status & DI_IRQ_POST))
		return IRQ_NONE;
	di_write(di, DI_INTR_CTRL, DI_IRQ_MASK | DI_IRQ_POST);
	complete(&di->complete);
	return IRQ_HANDLED;
}
static void di_latch_fatal(struct sc2_di *di)
{
	struct sc2_di_ctx *ctx;
	unsigned long flags;

	WRITE_ONCE(di->fatal, true);
	spin_lock_irqsave(&di->contexts_lock, flags);
	list_for_each_entry(ctx, &di->contexts, list) {
		vb2_queue_error(v4l2_m2m_get_src_vq(ctx->fh.m2m_ctx));
		vb2_queue_error(v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx));
	}
	spin_unlock_irqrestore(&di->contexts_lock, flags);
}

static void di_quarantine(struct sc2_di *di,
			  struct vb2_v4l2_buffer *src,
			  struct vb2_v4l2_buffer *dst)
{
	u32 value;

	di_latch_fatal(di);
	di->quarantine_src = src;
	di->quarantine_dst = dst;
	pm_runtime_get_noresume(di->dev);
	dev_err(di->dev, "DMA failed to drain; quarantining ACTIVE surfaces and power until idle\n");
	/* No buf_done/job_finish while quarantined. The running M2M job owns
	 * the context; VB2 owns the allocations, DMABUF attachments and mapped
	 * SG tables. abort/STREAMOFF/close wait for this worker, and remove
	 * waits for all jobs before releasing any devm/supplier resources.
	 * A permanently wedged bus deliberately blocks teardown rather than
	 * unmapping live DMA. No shared VPU reset or new DMA job is allowed.
	 */
	while (readl_poll_timeout(di->vcbus + DIWR_CTRL * 4, value,
				 !(value & BIT(31)), 1000, 100000))
		msleep(100);
	di->quarantine_src = NULL;
	di->quarantine_dst = NULL;
	pm_runtime_put_noidle(di->dev);
}


static void di_work(struct work_struct *work)
{
	struct sc2_di *di = container_of(work, struct sc2_di, work);
	struct sc2_di_ctx *ctx = READ_ONCE(di->active);
	struct vb2_v4l2_buffer *src, *dst;
	enum vb2_buffer_state state = VB2_BUF_STATE_DONE;
	bool started;
	int ret;

	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	reinit_completion(&di->complete);
	ret = READ_ONCE(ctx->aborting) || READ_ONCE(di->fatal) ||
		READ_ONCE(di->removing) ? -ECANCELED : di_program(ctx, src, dst);
	/* A context may already have been queued when another one went fatal.
	 * Such a dispatch returns errors without touching any hardware register.
	 */
	started = !ret;
	if (started && !wait_for_completion_timeout(&di->complete, msecs_to_jiffies(250)))
		ret = -ETIMEDOUT;
	if (READ_ONCE(ctx->aborting))
		ret = -ECANCELED;
	if (started && di_stop(di)) {
		di_quarantine(di, src, dst);
		ret = -EIO;
	}
	if (ret) {
		state = VB2_BUF_STATE_ERROR;
		if (ret != -ECANCELED)
			dev_err_ratelimited(di->dev, "EI field job failed: %d\n", ret);
	}

	v4l2_m2m_buf_copy_metadata(src, dst, false);
	dst->field = V4L2_FIELD_NONE;
	dst->sequence = ctx->sequence++;
	if (ctx->field)
		dst->vb2_buf.timestamp += div_u64((u64)NSEC_PER_SEC * ctx->period.numerator,
						(u64)ctx->period.denominator * 2);
	v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
	v4l2_m2m_buf_done(dst, state);
	if (ret || ++ctx->field == 2) {
		ctx->field = 0;
		v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		v4l2_m2m_buf_done(src, state);
	}
	WRITE_ONCE(di->active, NULL);
	v4l2_m2m_job_finish(di->m2m, ctx->fh.m2m_ctx);
}

static void di_run(void *priv)
{
	struct sc2_di_ctx *ctx = priv;

	WRITE_ONCE(ctx->di->active, ctx);
	queue_work(system_long_wq, &ctx->di->work);
}

static void di_abort(void *priv)
{
	struct sc2_di_ctx *ctx = priv;

	WRITE_ONCE(ctx->aborting, true);
	complete(&ctx->di->complete);
	flush_work(&ctx->di->work);
}

static int di_job_ready(void *priv)
{
	struct sc2_di_ctx *ctx = priv;

	return !READ_ONCE(ctx->di->fatal) && !READ_ONCE(ctx->di->removing) &&
		!READ_ONCE(ctx->aborting);
}

static const struct v4l2_m2m_ops di_m2m_ops = {
	.device_run = di_run,
	.job_ready = di_job_ready,
	.job_abort = di_abort,
};

static unsigned int di_index(enum v4l2_buf_type type)
{
	return V4L2_TYPE_IS_OUTPUT(type) ? 0 : 1;
}

static int di_queue_setup(struct vb2_queue *vq, unsigned int *nbufs,
			  unsigned int *nplanes, unsigned int sizes[],
			  struct device *alloc_devs[])
{
	struct sc2_di_ctx *ctx = vb2_get_drv_priv(vq);
	struct v4l2_pix_format_mplane *f = &ctx->fmt[di_index(vq->type)];
	unsigned int p;

	if (READ_ONCE(ctx->di->fatal) || READ_ONCE(ctx->di->removing))
		return -EIO;

	if (*nplanes) {
		if (*nplanes != 2)
			return -EINVAL;
		for (p = 0; p < 2; p++)
			if (sizes[p] < f->plane_fmt[p].sizeimage)
				return -EINVAL;
	} else {
		*nplanes = 2;
		for (p = 0; p < 2; p++)
			sizes[p] = f->plane_fmt[p].sizeimage;
	}
	return 0;
}

static int di_buf_prepare(struct vb2_buffer *vb)
{
	struct sc2_di_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct v4l2_pix_format_mplane *f = &ctx->fmt[di_index(vb->type)];
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	unsigned int p;

	if (READ_ONCE(ctx->di->fatal) || READ_ONCE(ctx->di->removing))
		return -EIO;

	if (V4L2_TYPE_IS_OUTPUT(vb->type) &&
	    vbuf->field != V4L2_FIELD_INTERLACED_TB &&
	    vbuf->field != V4L2_FIELD_INTERLACED_BT)
		return -EINVAL;
	for (p = 0; p < 2; p++) {
		u32 off = vb->planes[p].data_offset;
		dma_addr_t addr = vb2_dma_contig_plane_dma_addr(vb, p) + off;

		if (off > vb2_plane_size(vb, p) ||
		    vb2_plane_size(vb, p) - off < f->plane_fmt[p].sizeimage ||
		    !IS_ALIGNED(addr, 8) || upper_32_bits(addr))
			return -EINVAL;
		if (V4L2_TYPE_IS_OUTPUT(vb->type) &&
		    vb2_get_plane_payload(vb, p) < off + f->plane_fmt[p].sizeimage)
			return -EINVAL;
		if (!V4L2_TYPE_IS_OUTPUT(vb->type))
			vb2_set_plane_payload(vb, p, off + f->plane_fmt[p].sizeimage);
	}
	return 0;
}

static void di_buf_queue(struct vb2_buffer *vb)
{
	struct sc2_di_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(vb));
}

static int di_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct sc2_di_ctx *ctx = vb2_get_drv_priv(vq);
	struct vb2_v4l2_buffer *buf;
	int ret;

	if (READ_ONCE(ctx->di->fatal) || READ_ONCE(ctx->di->removing)) {
		ret = -EIO;
		goto fail;
	}

	if (ctx->fmt[1].width != ctx->crop.width ||
	    ctx->fmt[1].height != ctx->crop.height) {
		ret = -EINVAL;
		goto fail;
	}
	ret = pm_runtime_resume_and_get(ctx->di->dev);
	if (ret < 0)
		goto fail;
	WRITE_ONCE(ctx->aborting, false);
	ctx->field = 0;
	ctx->sequence = 0;
	return 0;
fail:
	while ((buf = V4L2_TYPE_IS_OUTPUT(vq->type) ?
		v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx) :
		v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx)))
		v4l2_m2m_buf_done(buf, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void di_stop_streaming(struct vb2_queue *vq)
{
	struct sc2_di_ctx *ctx = vb2_get_drv_priv(vq);
	struct vb2_v4l2_buffer *buf;

	if (READ_ONCE(ctx->di->active) == ctx)
		di_abort(ctx);
	while ((buf = V4L2_TYPE_IS_OUTPUT(vq->type) ?
		v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx) :
		v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx)))
		v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);
	ctx->field = 0;
	pm_runtime_put_sync(ctx->di->dev);
}

static const struct vb2_ops di_qops = {
	.queue_setup = di_queue_setup,
	.buf_prepare = di_buf_prepare,
	.buf_queue = di_buf_queue,
	.start_streaming = di_start_streaming,
	.stop_streaming = di_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

static int di_queue_init(void *priv, struct vb2_queue *src, struct vb2_queue *dst)
{
	struct sc2_di_ctx *ctx = priv;
	struct vb2_queue *q[2] = { src, dst };
	unsigned int i;
	int ret;

	for (i = 0; i < 2; i++) {
		q[i]->type = i ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE :
				V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
		q[i]->io_modes = VB2_MMAP | VB2_DMABUF;
		q[i]->drv_priv = ctx;
		q[i]->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
		q[i]->ops = &di_qops;
		q[i]->mem_ops = &vb2_dma_contig_memops;
		q[i]->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
		q[i]->lock = &ctx->di->lock;
		q[i]->dev = ctx->di->dev;
		ret = vb2_queue_init(q[i]);
		if (ret)
			return ret;
	}
	return 0;
}

static int di_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	strscpy(cap->driver, DI_NAME, sizeof(cap->driver));
	strscpy(cap->card, "SC2 hardware field-rate EI", sizeof(cap->card));
	return 0;
}

static int di_enum_fmt(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	if (f->index > 1)
		return -EINVAL;
	f->pixelformat = f->index ? V4L2_PIX_FMT_NV21M : V4L2_PIX_FMT_NV12M;
	return 0;
}

static int di_try_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *p = &f->fmt.pix_mp;
	unsigned int i;
	u32 alignment = V4L2_TYPE_IS_OUTPUT(f->type) ? 32 : 64;

	if (p->pixelformat != V4L2_PIX_FMT_NV21M)
		p->pixelformat = V4L2_PIX_FMT_NV12M;
	p->width = clamp_t(u32, ALIGN(p->width, 2), 32, 1920);
	p->height = clamp_t(u32, ALIGN(p->height, 4), 32, 1088);
	p->num_planes = 2;
	if (!V4L2_TYPE_IS_OUTPUT(f->type))
		p->field = V4L2_FIELD_NONE;
	else if (p->field != V4L2_FIELD_INTERLACED_BT)
		p->field = V4L2_FIELD_INTERLACED_TB;
	for (i = 0; i < 2; i++) {
		/* SC2 DIWR uses 64-byte bursts; a 32-byte-only stride aliases rows. */
		p->plane_fmt[i].bytesperline = ALIGN(clamp_t(u32,
			p->plane_fmt[i].bytesperline, p->width, 4096), alignment);
		p->plane_fmt[i].sizeimage = p->plane_fmt[i].bytesperline * (p->height >> i);
	}
	if (p->colorspace == V4L2_COLORSPACE_DEFAULT)
		p->colorspace = V4L2_COLORSPACE_REC709;
	return 0;
}

static int di_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	f->fmt.pix_mp = di_ctx(file_to_v4l2_fh(file))->fmt[di_index(f->type)];
	return 0;
}

static int di_s_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct sc2_di_ctx *ctx = di_ctx(file_to_v4l2_fh(file));
	unsigned int index = di_index(f->type);

	if (vb2_is_busy(v4l2_m2m_get_src_vq(ctx->fh.m2m_ctx)) ||
	    vb2_is_busy(v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx)))
		return -EBUSY;
	di_try_fmt(file, fh, f);
	ctx->fmt[index] = f->fmt.pix_mp;
	if (!index)
		ctx->crop = (struct v4l2_rect) { 0, 0, f->fmt.pix_mp.width, f->fmt.pix_mp.height };
	return 0;
}

static int di_g_selection(struct file *file, void *fh, struct v4l2_selection *s)
{
	struct sc2_di_ctx *ctx = di_ctx(file_to_v4l2_fh(file));

	if (!V4L2_TYPE_IS_OUTPUT(s->type))
		return -EINVAL;
	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
		s->r = ctx->crop;
		return 0;
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		s->r = (struct v4l2_rect) { 0, 0, ctx->fmt[0].width, ctx->fmt[0].height };
		return 0;
	default:
		return -EINVAL;
	}
}

static int di_s_selection(struct file *file, void *fh, struct v4l2_selection *s)
{
	struct sc2_di_ctx *ctx = di_ctx(file_to_v4l2_fh(file));

	if (!V4L2_TYPE_IS_OUTPUT(s->type) || s->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;
	if (vb2_is_busy(v4l2_m2m_get_src_vq(ctx->fh.m2m_ctx)) ||
	    vb2_is_busy(v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx)))
		return -EBUSY;
	if (s->r.left < 0 || s->r.top < 0 || s->r.width < 32 || s->r.height < 32 ||
	    !IS_ALIGNED(s->r.left, 8) || !IS_ALIGNED(s->r.top, 4) ||
	    !IS_ALIGNED(s->r.width, 2) || !IS_ALIGNED(s->r.height, 4) ||
	    s->r.left + s->r.width > ctx->fmt[0].width ||
	    s->r.top + s->r.height > ctx->fmt[0].height)
		return -EINVAL;
	ctx->crop = s->r;
	return 0;
}

static int di_g_parm(struct file *file, void *fh, struct v4l2_streamparm *p)
{
	struct sc2_di_ctx *ctx = di_ctx(file_to_v4l2_fh(file));

	if (!V4L2_TYPE_IS_OUTPUT(p->type))
		return -EINVAL;
	memset(&p->parm, 0, sizeof(p->parm));
	p->parm.output.capability = V4L2_CAP_TIMEPERFRAME;
	p->parm.output.timeperframe = ctx->period;
	return 0;
}

static int di_s_parm(struct file *file, void *fh, struct v4l2_streamparm *p)
{
	struct sc2_di_ctx *ctx = di_ctx(file_to_v4l2_fh(file));
	struct v4l2_fract t = p->parm.output.timeperframe;

	if (!V4L2_TYPE_IS_OUTPUT(p->type) || !t.numerator || !t.denominator ||
	    t.numerator > 1000000 || t.denominator > 1000000)
		return -EINVAL;
	ctx->period = t;
	return di_g_parm(file, fh, p);
}

static const struct v4l2_ioctl_ops di_ioctl_ops = {
	.vidioc_querycap = di_querycap,
	.vidioc_enum_fmt_vid_cap = di_enum_fmt,
	.vidioc_enum_fmt_vid_out = di_enum_fmt,
	.vidioc_g_fmt_vid_cap_mplane = di_g_fmt,
	.vidioc_g_fmt_vid_out_mplane = di_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane = di_try_fmt,
	.vidioc_try_fmt_vid_out_mplane = di_try_fmt,
	.vidioc_s_fmt_vid_cap_mplane = di_s_fmt,
	.vidioc_s_fmt_vid_out_mplane = di_s_fmt,
	.vidioc_g_selection = di_g_selection,
	.vidioc_s_selection = di_s_selection,
	.vidioc_g_parm = di_g_parm,
	.vidioc_s_parm = di_s_parm,
	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
};

static int di_open(struct file *file)
{
	struct sc2_di *di = video_drvdata(file);
	struct sc2_di_ctx *ctx;
	struct v4l2_format f = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE };
	unsigned long flags;
	int ret;

	mutex_lock(&di->lock);
	if (di->removing || di->fatal) {
		ret = -ENODEV;
		goto unlock;
	}
	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		ret = -ENOMEM;
		goto unlock;
	}
	ctx->di = di;
	ctx->file = file;
	ctx->period = (struct v4l2_fract) { 1, 25 };
	v4l2_fh_init(&ctx->fh, &di->video);
	f.fmt.pix_mp.width = 1920;
	f.fmt.pix_mp.height = 1088;
	di_try_fmt(file, NULL, &f);
	ctx->fmt[0] = f.fmt.pix_mp;
	ctx->fmt[1] = f.fmt.pix_mp;
	ctx->fmt[1].field = V4L2_FIELD_NONE;
	ctx->crop = (struct v4l2_rect) { 0, 0, 1920, 1088 };
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(di->m2m, ctx, di_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		v4l2_fh_exit(&ctx->fh);
		kfree(ctx);
		goto unlock;
	}
	v4l2_fh_add(&ctx->fh, file);
	spin_lock_irqsave(&di->contexts_lock, flags);
	list_add_tail(&ctx->list, &di->contexts);
	spin_unlock_irqrestore(&di->contexts_lock, flags);
	ret = 0;
unlock:
	mutex_unlock(&di->lock);
	return ret;
}

static int di_release(struct file *file)
{
	struct sc2_di_ctx *ctx = di_ctx(file_to_v4l2_fh(file));
	struct sc2_di *di = ctx->di;
	unsigned long flags;

	mutex_lock(&di->lock);
	spin_lock_irqsave(&di->contexts_lock, flags);
	list_del(&ctx->list);
	spin_unlock_irqrestore(&di->contexts_lock, flags);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
	mutex_unlock(&di->lock);
	kfree(ctx);
	return 0;
}

static const struct v4l2_file_operations di_fops = {
	.owner = THIS_MODULE,
	.open = di_open,
	.release = di_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

static int di_runtime_resume(struct device *dev)
{
	struct sc2_di *di = dev_get_drvdata(dev);
	u32 value;
	int ret = clk_bulk_prepare_enable(ARRAY_SIZE(di->clks), di->clks);

	if (ret)
		return ret;
	value = readl(di->power + 0x54);
	di->saved_power = value & GENMASK(29, 28);
	writel(value & ~GENMASK(29, 28), di->power + 0x54);
	di->saved_gate0 = di_read(di, VIUB_GCLK_CTRL0) & (BIT(15) | BIT(9));
	di->saved_gate3 = di_read(di, VIUB_GCLK_CTRL3) & GENMASK(5, 2);
	di_update(di, VIUB_GCLK_CTRL0, BIT(15) | BIT(9), BIT(15) | BIT(9));
	di_update(di, VIUB_GCLK_CTRL3, GENMASK(5, 2), 0);
	di_write(di, 0x1704, 0x00ff0100);
	di_write(di, 0x1705, 0x5a0a0f2d);
	di_write(di, 0x1706, 0x050a0a5d);
	di_update(di, 0x2028, GENMASK(31, 30), GENMASK(31, 30));
	di_write(di, DI_INTR_CTRL, 0xcfffffff);
	enable_irq(di->irq);
	return 0;
}

static int di_runtime_suspend(struct device *dev)
{
	struct sc2_di *di = dev_get_drvdata(dev);
	u32 value;
	int ret = di_stop(di);

	if (ret)
		return ret;
	disable_irq(di->irq);
	di_update(di, VIUB_GCLK_CTRL0, BIT(15) | BIT(9), di->saved_gate0);
	di_update(di, VIUB_GCLK_CTRL3, GENMASK(5, 2), di->saved_gate3);
	value = readl(di->power + 0x54);
	writel((value & ~GENMASK(29, 28)) | di->saved_power, di->power + 0x54);
	clk_bulk_disable_unprepare(ARRAY_SIZE(di->clks), di->clks);
	return 0;
}

static void di_unmap(void *data)
{
	iounmap(data);
}

static void di_v4l2_release(struct v4l2_device *v4l2)
{
	struct sc2_di *di = container_of(v4l2, struct sc2_di, v4l2);

	/* Last video/file/VMA reference, not merely unregistering the node. */
	v4l2_m2m_release(di->m2m);
	complete(&di->released);
}

static int di_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct platform_device *vpu, *canvas;
	struct device_node *np;
	struct sc2_di *di;
	unsigned int i;
	int ret;

	di = devm_kzalloc(dev, sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;
	di->dev = dev;
	platform_set_drvdata(pdev, di);
	mutex_init(&di->lock);
	spin_lock_init(&di->contexts_lock);
	INIT_LIST_HEAD(&di->contexts);
	init_completion(&di->released);
	init_completion(&di->complete);
	INIT_WORK(&di->work, di_work);
	np = of_parse_phandle(dev->of_node, "amlogic,vpu", 0);
	if (!np)
		return -EINVAL;
	vpu = of_find_device_by_node(np);
	if (!vpu) {
		of_node_put(np);
		return -EPROBE_DEFER;
	}
	/* Explicit VPU supplier, not a second resource owner or independent reset.
	 * Registers used here are DI-only; display fetch registers are untouched.
	 */
	if (!device_link_add(dev, &vpu->dev,
		DL_FLAG_PM_RUNTIME | DL_FLAG_AUTOREMOVE_CONSUMER)) {
		put_device(&vpu->dev);
		of_node_put(np);
		return -EINVAL;
	}
	put_device(&vpu->dev);
	di->vcbus = of_iomap(np, 0);
	di->power = of_iomap(np, 3);
	of_node_put(np);
	if (!di->vcbus || !di->power) {
		if (di->vcbus)
			iounmap(di->vcbus);
		if (di->power)
			iounmap(di->power);
		return -ENOMEM;
	}
	ret = devm_add_action_or_reset(dev, di_unmap, di->vcbus);
	if (ret) {
		iounmap(di->power);
		return ret;
	}
	ret = devm_add_action_or_reset(dev, di_unmap, di->power);
	if (ret)
		return ret;
	di->clks[0].id = "vpu";
	di->clks[1].id = "vapb";
	di->clks[2].id = "clkb";
	di->clks[3].id = "intr";
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(di->clks), di->clks);
	if (ret)
		return ret;
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	np = of_parse_phandle(dev->of_node, "amlogic,canvas", 0);
	if (!np)
		return -EINVAL;
	canvas = of_find_device_by_node(np);
	of_node_put(np);
	if (!canvas)
		return -EPROBE_DEFER;
	/* Supplier unbind must wait for our final file/VMA references too. */
	if (!device_link_add(dev, &canvas->dev, DL_FLAG_AUTOREMOVE_CONSUMER)) {
		put_device(&canvas->dev);
		return -EINVAL;
	}
	put_device(&canvas->dev);
	di->canvas = meson_canvas_get(dev);
	if (IS_ERR(di->canvas))
		return PTR_ERR(di->canvas);
	for (i = 0; i < ARRAY_SIZE(di->canvas_id); i++) {
		ret = meson_canvas_alloc(di->canvas, &di->canvas_id[i]);
		if (ret)
			goto free_canvas;
	}
	di->irq = platform_get_irq_byname(pdev, "post");
	if (di->irq < 0) {
		ret = di->irq;
		goto free_canvas;
	}
	ret = devm_request_irq(dev, di->irq, di_irq, IRQF_NO_AUTOEN, DI_NAME, di);
	if (ret)
		goto free_canvas;
	ret = v4l2_device_register(dev, &di->v4l2);
	if (ret)
		goto free_canvas;
	di->m2m = v4l2_m2m_init(&di_m2m_ops);
	if (IS_ERR(di->m2m)) {
		ret = PTR_ERR(di->m2m);
		goto unreg_v4l2;
	}
	di->v4l2.release = di_v4l2_release;
	strscpy(di->video.name, DI_NAME, sizeof(di->video.name));
	di->video.v4l2_dev = &di->v4l2;
	di->video.fops = &di_fops;
	di->video.ioctl_ops = &di_ioctl_ops;
	di->video.release = video_device_release_empty;
	di->video.lock = &di->lock;
	di->video.vfl_dir = VFL_DIR_M2M;
	di->video.device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	video_set_drvdata(&di->video, di);
	pm_runtime_enable(dev);
	ret = video_register_device(&di->video, VFL_TYPE_VIDEO, -1);
	if (ret) {
		pm_runtime_disable(dev);
		goto unreg_v4l2;
	}
	dev_info(dev, "native SC2 hardware EI, NV12M/NV21M, two progressive outputs per frame\n");
	return 0;
unreg_v4l2:
	v4l2_device_unregister(&di->v4l2);
	v4l2_device_put(&di->v4l2);
free_canvas:
	while (i--)
		meson_canvas_free(di->canvas, di->canvas_id[i]);
	return ret;
}

static void di_remove(struct platform_device *pdev)
{
	struct sc2_di *di = platform_get_drvdata(pdev);
	struct sc2_di_ctx *ctx;
	unsigned int i;

	mutex_lock(&di->lock);
	WRITE_ONCE(di->removing, true);
	list_for_each_entry(ctx, &di->contexts, list) {
		WRITE_ONCE(ctx->aborting, true);
		vb2_queue_error(v4l2_m2m_get_src_vq(ctx->fh.m2m_ctx));
		vb2_queue_error(v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx));
	}
	complete(&di->complete);
	v4l2_m2m_suspend(di->m2m);
	flush_work(&di->work);
	list_for_each_entry(ctx, &di->contexts, list) {
		v4l2_m2m_streamoff(ctx->file, ctx->fh.m2m_ctx,
			V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
		v4l2_m2m_streamoff(ctx->file, ctx->fh.m2m_ctx,
			V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	}
	mutex_unlock(&di->lock);
	video_unregister_device(&di->video);
	v4l2_device_unregister(&di->v4l2);
	v4l2_device_put(&di->v4l2);
	/* Keep the devm container, MMIO, IRQ, clocks and supplier links alive
	 * through final close/munmap. This also orders supplier unbind after
	 * all users: the consumer remove callback cannot return before then.
	 */
	wait_for_completion(&di->released);
	pm_runtime_disable(di->dev);
	for (i = 0; i < ARRAY_SIZE(di->canvas_id); i++)
		meson_canvas_free(di->canvas, di->canvas_id[i]);
}

static const struct dev_pm_ops di_pm_ops = {
	SET_RUNTIME_PM_OPS(di_runtime_suspend, di_runtime_resume, NULL)
};
static const struct of_device_id di_of_match[] = {
	{ .compatible = "amlogic,meson-sc2-di" },
	{ }
};
MODULE_DEVICE_TABLE(of, di_of_match);
static struct platform_driver di_driver = {
	.probe = di_probe,
	.remove = di_remove,
	.driver = {
		.name = DI_NAME,
		.of_match_table = di_of_match,
		.pm = &di_pm_ops,
	},
};
module_platform_driver(di_driver);
MODULE_DESCRIPTION("Amlogic SC2 field-rate hardware edge-interpolation deinterlacer");
MODULE_LICENSE("GPL");
