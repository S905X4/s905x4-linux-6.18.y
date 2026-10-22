// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SC2 VDEC1 single-instance MPEG4, VC1, AVS and MJPEG protocols.
 *
 * Register transactions and workspace relocations adapted from Amlogic's
 * vmpeg4.c, vvc1.c, avs.c and vmjpeg.c (Copyright 2017 Amlogic, Inc.),
 * also present in CoreELEC media_modules-aml a387dea216e416f975377152d374557d448093c8.
 * Firmware owns reconstruction, references and display reordering. This is
 * deliberately not the similarly named multi-instance firmware protocol.
 */
#include <linux/bitops.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/soc/amlogic/meson-canvas.h>
#include <media/v4l2-event.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#include "codec_legacy.h"
#include "dos_regs.h"
#include "vdec_helpers.h"

#define LEGACY_WIDTH 1920
#define LEGACY_HEIGHT 1088
#define LEGACY_BUFFERIN AV_SCRATCH_8
#define LEGACY_BUFFEROUT AV_SCRATCH_9
#define LEGACY_OFFSET AV_SCRATCH_C
#define LEGACY_WORKSPACE AV_SCRATCH_F
#define LEGACY_CANVAS_ACCESS 0x0014
#define LEGACY_SCRATCH_M 0x2758
#define LEGACY_SCRATCH_N 0x275c

enum legacy_codec { LEGACY_MPEG4, LEGACY_VC1, LEGACY_AVS, LEGACY_MJPEG };

struct codec_legacy {
	enum legacy_codec codec;
	struct amvdec_session *sess;
	struct delayed_work recycle_work;
	unsigned long pending_recycle;
	void *workspace;
	dma_addr_t workspace_dma;
	u32 workspace_size;
	u32 buffers;
	u32 canvases[8];
	u32 error_count;
	bool configured;
	u8 wmv_sequence[4];
	u8 mpeg4_time_bits;
	bool wmv_sequence_valid;
	bool failed;
	bool source_wait;
	bool metadata_ready;
};

static void legacy_abort(struct amvdec_session *sess);
static irqreturn_t legacy_threaded_isr(struct amvdec_session *sess);

static bool legacy_old_frames_drained(struct amvdec_session *sess)
{
	struct vb2_queue *q = v4l2_m2m_get_dst_vq(sess->m2m_ctx);
	unsigned long flags;
	bool empty;

	spin_lock_irqsave(&q->done_lock, flags);
	empty = list_empty(&q->done_list);
	spin_unlock_irqrestore(&q->done_lock, flags);
	return empty;
}

static void legacy_recycle_worker(struct work_struct *work)
{
	struct codec_legacy *ctx = container_of(to_delayed_work(work),
						struct codec_legacy, recycle_work);
	struct amvdec_core *core = ctx->sess->core;
	unsigned long pending;
	u32 idx;

	if (READ_ONCE(ctx->failed))
		return;
	if ((ctx->codec == LEGACY_MPEG4 && amvdec_read_dos(core, AV_SCRATCH_L)) ||
	    ((ctx->codec == LEGACY_VC1 || ctx->codec == LEGACY_AVS) &&
	     amvdec_read_dos(core, AV_SCRATCH_7))) {
		legacy_abort(ctx->sess);
		return;
	}
	if (READ_ONCE(ctx->source_wait) && legacy_old_frames_drained(ctx->sess)) {
		WRITE_ONCE(ctx->source_wait, false);
		legacy_threaded_isr(ctx->sess);
	}
	pending = READ_ONCE(ctx->pending_recycle);
	if (pending && !amvdec_read_dos(core, LEGACY_BUFFERIN)) {
		idx = __ffs(pending);
		clear_bit(idx, &ctx->pending_recycle);
		amvdec_write_dos(core, LEGACY_BUFFERIN,
			ctx->codec == LEGACY_MJPEG ? idx + 1 : ~(u32)BIT(idx));
	}
	if (!READ_ONCE(ctx->failed))
		schedule_delayed_work(&ctx->recycle_work,
			msecs_to_jiffies(READ_ONCE(ctx->pending_recycle) ? 1 : 10));
}

/* AVS derives buffers 4..7 by adding eight to the first four canvas IDs. */
static int legacy_canvas_range(struct amvdec_session *sess, unsigned int count)
{
	u8 allocated[256], first = 0;
	unsigned int n = 0, run = 0, i;
	int ret;

	while (n < ARRAY_SIZE(allocated)) {
		ret = meson_canvas_alloc(sess->core->canvas, &allocated[n]);
		if (ret)
			break;
		if (!run || allocated[n] != (unsigned int)allocated[n - 1] + 1) {
			first = allocated[n];
			run = 1;
		} else {
			run++;
		}
		n++;
		if (run == count) {
			for (i = 0; i < n - count; i++)
				meson_canvas_free(sess->core->canvas, allocated[i]);
			for (i = 0; i < count; i++)
				sess->canvas_alloc[sess->canvas_num++] = first + i;
			return 0;
		}
	}
	for (i = 0; i < n; i++)
		meson_canvas_free(sess->core->canvas, allocated[i]);
	return -ENOSPC;
}

/* The JPEG ABI programs first-field canvases through scratch4..7, but only
 * fixed absolute second-field IDs are evidenced by the vendor host. Reserve
 * those IDs independently; DRM legitimately owns low canvas IDs at probe.
 */
static int mjpeg_canvas_reserve(struct amvdec_session *sess)
{
	u8 allocated[256], primary[8], id;
	unsigned int n = 0, first = 0, i;
	u32 secondary = 0;
	int ret;

	while (n < ARRAY_SIZE(allocated)) {
		ret = meson_canvas_alloc(sess->core->canvas, &id);
		if (ret)
			break;
		allocated[n++] = id;
		if (id >= 12 && id <= 25)
			secondary |= BIT(id - 12);
		else if (first < ARRAY_SIZE(primary))
			primary[first++] = id;
		if (secondary == GENMASK(13, 0) && first == ARRAY_SIZE(primary))
			break;
	}
	if (secondary != GENMASK(13, 0) || first != ARRAY_SIZE(primary)) {
		for (i = 0; i < n; i++)
			meson_canvas_free(sess->core->canvas, allocated[i]);
		return -EBUSY;
	}
	for (i = 0; i < n; i++) {
		id = allocated[i];
		if ((id < 12 || id > 25) && !memchr(primary, id, sizeof(primary)))
			meson_canvas_free(sess->core->canvas, id);
	}
	for (i = 0; i < ARRAY_SIZE(primary); i++)
		sess->canvas_alloc[sess->canvas_num++] = primary[i];
	for (i = 12; i <= 25; i++)
		sess->canvas_alloc[sess->canvas_num++] = i;
	return 0;
}

static int legacy_canvas_config(struct amvdec_session *sess, u8 id,
				dma_addr_t dma, u32 stride, u32 height)
{
	int ret;

	if (upper_32_bits(dma))
		return -ERANGE;
	ret = meson_canvas_config(sess->core->canvas, id, dma, stride,
				  height, MESON_CANVAS_WRAP_NONE,
				  MESON_CANVAS_BLKMODE_LINEAR,
				  MESON_CANVAS_ENDIAN_SWAP64);
	if (ret)
		return ret;
	/* G12A+ VDEC-local canvas block-mode override, linear mode. */
	amvdec_write_dos(sess->core, LEGACY_CANVAS_ACCESS,
			 BIT(11) | BIT(8) | id);
	return 0;
}

static int legacy_set_canvases(struct amvdec_session *sess)
{
	static const u8 mjpeg_second[] = { 12, 14, 22, 24 };
	struct codec_legacy *ctx = sess->priv;
	struct v4l2_m2m_buffer *buf;
	u32 count = ctx->buffers * 2;
	u32 i = 0, start = sess->canvas_num, y, uv;
	dma_addr_t y_dma, uv_dma;
	int ret;

	if (sess->pixfmt_cap != V4L2_PIX_FMT_NV12M ||
	    sess->fmt_out->capture_width != LEGACY_WIDTH ||
	    sess->fmt_out->capture_height != LEGACY_HEIGHT)
		return -EINVAL;
	ret = ctx->codec == LEGACY_MJPEG ? mjpeg_canvas_reserve(sess) :
		legacy_canvas_range(sess, count);
	if (ret)
		return ret;

	v4l2_m2m_for_each_dst_buf(sess->m2m_ctx, buf) {
		if (i >= ctx->buffers) {
			ret = -EINVAL;
			goto fail;
		}
		y = sess->canvas_alloc[start + i * 2];
		uv = sess->canvas_alloc[start + i * 2 + 1];
		y_dma = vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0);
		uv_dma = vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 1);
		if (vb2_plane_size(&buf->vb.vb2_buf, 0) < LEGACY_WIDTH * LEGACY_HEIGHT ||
		    vb2_plane_size(&buf->vb.vb2_buf, 1) < LEGACY_WIDTH * LEGACY_HEIGHT / 2) {
			ret = -EINVAL;
			goto fail;
		}
		ret = legacy_canvas_config(sess, y, y_dma, LEGACY_WIDTH, LEGACY_HEIGHT);
		if (ret)
			goto fail;
		ret = legacy_canvas_config(sess, uv, uv_dma, LEGACY_WIDTH, LEGACY_HEIGHT / 2);
		if (ret)
			goto fail;
		ctx->canvases[i] = y | uv << 8 | uv << 16;
		if (ctx->codec == LEGACY_MJPEG) {
			u32 field_height = DIV_ROUND_UP(sess->height, 2);
			u8 second = mjpeg_second[i];

			ret = legacy_canvas_config(sess, second,
					 y_dma + LEGACY_WIDTH * field_height,
					 LEGACY_WIDTH, LEGACY_HEIGHT / 2);
			if (ret)
				goto fail;
			ret = legacy_canvas_config(sess, second + 1,
					 uv_dma + LEGACY_WIDTH * field_height / 2,
					 LEGACY_WIDTH, LEGACY_HEIGHT / 4);
			if (ret)
				goto fail;
		}
		sess->fw_idx_to_vb2_idx[i++] = buf->vb.vb2_buf.index;
	}
	if (i == ctx->buffers)
		return 0;
	ret = -EINVAL;
fail:
	while (sess->canvas_num > start)
		meson_canvas_free(sess->core->canvas,
				  sess->canvas_alloc[--sess->canvas_num]);
	return ret;
}

static void legacy_init_scaler(struct amvdec_core *core)
{
	static const u32 coeff[] = {
		0x20402000, 0x20402000, 0x1f3f2101, 0x1f3f2101,
		0x1e3e2202, 0x1e3e2202, 0x1d3d2303, 0x1d3d2303,
		0x1c3c2404, 0x1c3c2404, 0x1b3b2505, 0x1b3b2505,
		0x1a3a2606, 0x1a3a2606, 0x19392707, 0x19392707,
		0x18382808, 0x18382808, 0x17372909, 0x17372909,
		0x16362a0a, 0x16362a0a, 0x15352b0b, 0x15352b0b,
		0x14342c0c, 0x14342c0c, 0x13332d0d, 0x13332d0d,
		0x12322e0e, 0x12322e0e, 0x11312f0f, 0x11312f0f,
		0x10303010,
	};
	unsigned int i;

	amvdec_write_dos(core, PSCALE_CTRL, 0xc000);
	amvdec_write_dos(core, PSCALE_BMEM_ADDR, 0);
	for (i = 0; i < ARRAY_SIZE(coeff); i++) {
		amvdec_write_dos(core, PSCALE_BMEM_DAT, 0);
		amvdec_write_dos(core, PSCALE_BMEM_DAT, coeff[i]);
	}
	for (i = 37; i <= 43; i += 2) {
		amvdec_write_dos(core, PSCALE_BMEM_ADDR, i * 2);
		amvdec_write_dos(core, PSCALE_BMEM_DAT, 8);
		amvdec_write_dos(core, PSCALE_BMEM_DAT, 0x60000000);
	}
	for (i = 36; i <= 42; i += 2) {
		amvdec_write_dos(core, PSCALE_BMEM_ADDR, i * 2 + 1);
		amvdec_write_dos(core, PSCALE_BMEM_DAT, 0x10000);
	}
	amvdec_write_dos(core, DOS_SW_RESET0, BIT(10));
	amvdec_write_dos(core, DOS_SW_RESET0, 0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_write_dos(core, PSCALE_RST, 7);
	amvdec_write_dos(core, PSCALE_RST, 0);
}

static int legacy_configure(struct amvdec_session *sess)
{
	struct codec_legacy *ctx = sess->priv;
	struct amvdec_core *core = sess->core;
	u32 i, reset = BIT(7) | BIT(6);
	int ret;

	if (ctx->codec == LEGACY_VC1 || ctx->codec == LEGACY_AVS)
		reset |= BIT(4);
	amvdec_write_dos(core, DOS_SW_RESET0, reset);
	amvdec_write_dos(core, DOS_SW_RESET0, 0);
	if (ctx->codec == LEGACY_VC1 || ctx->codec == LEGACY_AVS) {
		amvdec_read_dos(core, DOS_SW_RESET0);
		amvdec_write_dos(core, DOS_SW_RESET0, reset);
		amvdec_write_dos(core, DOS_SW_RESET0, 0);
		amvdec_write_dos(core, DOS_SW_RESET0, BIT(9) | BIT(8));
		amvdec_write_dos(core, DOS_SW_RESET0, 0);
	}
	ret = legacy_set_canvases(sess);
	if (ret)
		return ret;

	amvdec_write_dos(core, POWER_CTL_VLD, BIT(4));
	amvdec_clear_dos_bits(core, VLD_MEM_VIFIFO_CONTROL, GENMASK(23, 16));
	amvdec_write_dos_bits(core, VLD_MEM_VIFIFO_CONTROL, 2 << 16 | 8 << 18);
	for (i = 0; i < min(ctx->buffers, 4U); i++)
		amvdec_write_dos(core, (ctx->codec == LEGACY_MJPEG ?
				 AV_SCRATCH_4 : AV_SCRATCH_0) + 4 * i,
				 ctx->canvases[i]);
	amvdec_write_dos(core, PSCALE_CTRL, 0);
	amvdec_write_dos(core, LEGACY_BUFFERIN, 0);
	amvdec_write_dos(core, LEGACY_BUFFEROUT, 0);
	/* Bit17 selects interleaved chroma; bit16=0 selects UV, not VU. */
	amvdec_write_dos_bits(core, MDEC_PIC_DC_CTRL, BIT(17));
	amvdec_clear_dos_bits(core, MDEC_PIC_DC_CTRL, BIT(16));

	switch (ctx->codec) {
	case LEGACY_MPEG4:
		for (i = 4; i < 8; i++)
			amvdec_write_dos(core, AV_SCRATCH_G + 4 * (i - 4),
					 ctx->canvases[i]);
		amvdec_write_dos(core, AV_SCRATCH_L, 0);
		amvdec_write_dos(core, LEGACY_WORKSPACE,
				 lower_32_bits(ctx->workspace_dma) - 0x02b00000);
		amvdec_write_dos(core, AV_SCRATCH_A, 0);
		amvdec_write_dos(core, AV_SCRATCH_7, sess->width << 16 | sess->height);
		amvdec_write_dos(core, AV_SCRATCH_E, 0);
		amvdec_write_dos(core, MDEC_PIC_DC_THRESH, 0x404038aa);
		break;
	case LEGACY_VC1:
		amvdec_write_dos(core, LEGACY_WORKSPACE,
				 lower_32_bits(ctx->workspace_dma) - 0x01f00000);
		amvdec_write_dos(core, AV_SCRATCH_7, 0);
		amvdec_write_dos(core, AV_SCRATCH_4,
				 sess->fmt_out->pixfmt == V4L2_PIX_FMT_VC1_ANNEX_G);
		break;
	case LEGACY_AVS:
		amvdec_write_dos(core, LEGACY_WORKSPACE,
				 lower_32_bits(ctx->workspace_dma) - 0x01a00000);
		amvdec_write_dos(core, AV_SCRATCH_H, 1);
		amvdec_write_dos(core, AV_SCRATCH_G, 0);
		amvdec_write_dos(core, AV_SCRATCH_7, 0);
		amvdec_write_dos(core, AV_SCRATCH_D, 0);
		/* No userdata DMA is requested; decoding does not depend on it. */
		amvdec_write_dos(core, LEGACY_SCRATCH_M, 0);
		amvdec_write_dos(core, LEGACY_SCRATCH_N, 0);
		break;
	case LEGACY_MJPEG:
		amvdec_write_dos(core, AV_SCRATCH_0, 12);
		amvdec_write_dos(core, AV_SCRATCH_1, 0x031a);
		legacy_init_scaler(core);
		amvdec_write_dos(core, MCPU_INTR_MSK, 0xffff);
		amvdec_write_dos(core, AV_SCRATCH_2, sess->height << 4 | 0x8000);
		amvdec_write_dos(core, VDEC_ASSIST_AMR1_INT8, 8);
		break;
	}
	amvdec_write_dos(core, ASSIST_MBOX1_CLR_REG, 1);
	amvdec_write_dos(core, ASSIST_MBOX1_MASK, 1);
	ctx->configured = true;
	schedule_delayed_work(&ctx->recycle_work, msecs_to_jiffies(10));
	return 0;
}

static int legacy_start(struct amvdec_session *sess, enum legacy_codec codec)
{
	struct codec_legacy *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->codec = codec;
	ctx->sess = sess;
	INIT_DELAYED_WORK(&ctx->recycle_work, legacy_recycle_worker);
	ctx->buffers = codec == LEGACY_MPEG4 || codec == LEGACY_AVS ? 8 : 4;
	ctx->workspace_size = codec == LEGACY_AVS ? SZ_4M :
			      codec == LEGACY_VC1 ? SZ_2M : SZ_1M;
	if (codec != LEGACY_MJPEG) {
		ctx->workspace = dma_alloc_coherent(sess->core->dev,
				ctx->workspace_size, &ctx->workspace_dma, GFP_KERNEL);
		if (!ctx->workspace) {
			kfree(ctx);
			return -ENOMEM;
		}
		if (upper_32_bits(ctx->workspace_dma + ctx->workspace_size - 1)) {
			ret = -ERANGE;
			goto free;
		}
	}
	sess->priv = ctx;
	sess->visible_width = sess->width;
	sess->visible_height = sess->height;
	sess->keyframe_found = codec == LEGACY_MJPEG;
	if (!sess->streamon_cap ||
	    v4l2_m2m_num_dst_bufs_ready(sess->m2m_ctx) != sess->num_dst_bufs) {
		amvdec_src_change(sess, sess->width, sess->height, ctx->buffers);
		return 0;
	}
	ret = legacy_configure(sess);
	if (!ret)
		return 0;
	sess->priv = NULL;
free:
	if (ctx->workspace)
		dma_free_coherent(sess->core->dev, ctx->workspace_size,
				  ctx->workspace, ctx->workspace_dma);
	kfree(ctx);
	return ret;
}

static int mpeg4_start(struct amvdec_session *s) { return legacy_start(s, LEGACY_MPEG4); }
static int vc1_start(struct amvdec_session *s) { return legacy_start(s, LEGACY_VC1); }
static int avs_start(struct amvdec_session *s) { return legacy_start(s, LEGACY_AVS); }
static int mjpeg_start(struct amvdec_session *s) { return legacy_start(s, LEGACY_MJPEG); }

static int legacy_stop(struct amvdec_session *sess)
{
	struct codec_legacy *ctx = sess->priv;

	WRITE_ONCE(ctx->failed, true);
	cancel_delayed_work_sync(&ctx->recycle_work);
	/* Engine stop has already drained VDEC's DMC channel before freeing DMA. */
	if (ctx->workspace) {
		dma_free_coherent(sess->core->dev, ctx->workspace_size,
				  ctx->workspace, ctx->workspace_dma);
		ctx->workspace = NULL;
	}
	return 0;
}

static int legacy_can_recycle(struct amvdec_core *core)
{
	struct codec_legacy *ctx = core->cur_sess->priv;

	return ctx->configured && !READ_ONCE(ctx->failed);
}

static void legacy_recycle(struct amvdec_core *core, u32 idx)
{
	struct codec_legacy *ctx = core->cur_sess->priv;

	if (idx >= ctx->buffers || READ_ONCE(ctx->failed))
		return;
	set_bit(idx, &ctx->pending_recycle);
	mod_delayed_work(system_wq, &ctx->recycle_work, 0);
}

static void legacy_resume(struct amvdec_session *sess)
{
	struct codec_legacy *ctx = sess->priv;

	if (!ctx->configured && legacy_configure(sess)) {
		WRITE_ONCE(ctx->failed, true);
		amvdec_abort(sess);
	}
}

static void legacy_abort(struct amvdec_session *sess)
{
	struct codec_legacy *ctx = sess->priv;

	WRITE_ONCE(ctx->failed, true);
	amvdec_write_dos(sess->core, ASSIST_MBOX1_MASK, 0);
	amvdec_write_dos(sess->core, MPSR, 0);
	amvdec_abort(sess);
}

static struct v4l2_fract legacy_aspect(enum legacy_codec codec,
				     u32 width, u32 height, u32 ratio)
{
	static const struct v4l2_fract sar[] = {
		{ 1, 1 }, { 1, 1 }, { 12, 11 }, { 10, 11 }, { 16, 11 },
		{ 40, 33 }, { 24, 11 }, { 20, 11 }, { 32, 11 }, { 80, 33 },
		{ 18, 11 }, { 15, 11 }, { 64, 33 }, { 160, 99 },
	};
	struct v4l2_fract result;
	u32 dar_num, dar_den;

	if (codec == LEGACY_AVS) {
		switch (ratio) {
		case 2: dar_num = 4; dar_den = 3; break;
		case 3: dar_num = 16; dar_den = 9; break;
		case 4: dar_num = 221; dar_den = 100; break;
		default: return sar[0];
		}
		result.numerator = height * dar_num;
		result.denominator = width * dar_den;
		return result;
	}
	if (ratio > 0xf && (ratio & 0xff) && ((ratio >> 8) & 0xff)) {
		result.numerator = (ratio >> 8) & 0xff;
		result.denominator = ratio & 0xff;
		return result;
	}
	return sar[ratio < ARRAY_SIZE(sar) ? ratio : 0];
}

static void legacy_dimensions(struct amvdec_session *sess, u32 width, u32 height)
{
	struct codec_legacy *ctx = sess->priv;

	/* Fixed backing canvases remain valid across visible-size changes. */
	if (!width || !height || width > LEGACY_WIDTH || height > LEGACY_HEIGHT) {
		legacy_abort(sess);
		return;
	}
	if (width != sess->width || height != sess->height) {
		sess->width = width;
		sess->height = height;
		sess->visible_width = width;
		sess->visible_height = height;
		ctx->metadata_ready = false;
	}
}

/* Amlogic single VC1 microcode's escaped length/checksum header. The V4L2
 * Annex-L boundary remains standard RCV; proprietary framing stays here.
 */
static void legacy_wmv_header(u8 *dst, u8 code, u32 bytes)
{
	u32 sum = 0;
	unsigned int i;

	dst[0] = 0; dst[1] = 0; dst[2] = 1; dst[3] = code;
	dst[4] = 0; dst[5] = bytes >> 16; dst[6] = 0x88;
	dst[7] = bytes >> 8; dst[8] = bytes; dst[9] = 0x88;
	dst[10] = 0xff; dst[11] = 0xff; dst[12] = 0x88;
	dst[13] = 0xff; dst[14] = 0xff; dst[15] = 0x88;
	for (i = 4; i < 16; i++)
		sum += dst[i];
	dst[16] = sum >> 8; dst[17] = sum; dst[18] = 0x88;
	dst[19] = sum >> 8; dst[20] = sum; dst[21] = 0x88;
}

static int legacy_es_prepare_input(struct amvdec_session *sess,
				   struct vb2_v4l2_buffer *vbuf, u32 *payload_size)
{
	struct codec_legacy *ctx = sess->priv;
	const u8 *data = vb2_plane_vaddr(&vbuf->vb2_buf, 0);
	u32 i, pictures = 0;

	if (!data)
		return -EINVAL;
	for (i = 0; i + 4 <= *payload_size; i++) {
		if (data[i] || data[i + 1] || data[i + 2] != 1)
			continue;
		/* VC1 field-start 0x0c belongs to its frame-start 0x0d.
		 * An AVS B3/B6 picture header likewise owns the whole frame:
		 * field-coded slices share that header and one BUFFEROUT result.
		 * Two display field views must not consume two picture tokens.
		 */
		if (ctx->codec == LEGACY_VC1 ? data[i + 3] == 0x0d :
		    data[i + 3] == 0xb3 || data[i + 3] == 0xb6)
			pictures++;
		i += 3;
	}
	sess->input_frames = pictures;
	return 0;
}

static int vc1_prepare_input(struct amvdec_session *sess,
			     struct vb2_v4l2_buffer *vbuf, u32 *payload_size)
{
	struct codec_legacy *ctx = sess->priv;
	struct vb2_buffer *vb = &vbuf->vb2_buf;
	u8 *data = vb2_plane_vaddr(vb, 0);
	u32 size = *payload_size, pos = 0, frames = 0, frame_size;
	u32 width = sess->width, height = sess->height;
	bool sequence;

	if (sess->fmt_out->pixfmt != V4L2_PIX_FMT_VC1_ANNEX_L)
		return legacy_es_prepare_input(sess, vbuf, payload_size);
	if (!data || size < 8)
		return -EINVAL;
	sequence = size >= 36 && data[3] == 0xc5;
	if (sequence) {
		if (get_unaligned_le32(data + 4) != 4 ||
		    get_unaligned_le32(data + 20) != 12)
			return -EINVAL;
		height = get_unaligned_le32(data + 12);
		width = get_unaligned_le32(data + 16);
		if (!width || width > LEGACY_WIDTH || !height || height > LEGACY_HEIGHT)
			return -EINVAL;
		pos = 36;
	} else if (!ctx->wmv_sequence_valid) {
		return -EINVAL;
	}
	/* Validate the complete packet and its expansion before touching bytes. */
	while (pos < size) {
		if (size - pos < 8)
			return -EINVAL;
		frame_size = get_unaligned_le32(data + pos) & 0x7fffffff;
		if (!frame_size || frame_size > 0xffffff || frame_size > size - pos - 8)
			return -EINVAL;
		pos += frame_size + 8;
		frames++;
	}
	if ((u64)size + frames * 14ULL - (sequence ? 6 : 0) > vb2_plane_size(vb, 0))
		return -ENOSPC;
	if (sequence) {
		memcpy(ctx->wmv_sequence, data + 8, 4);
		ctx->wmv_sequence_valid = true;
		memmove(data + 30, data + 36, size - 36);
		size -= 6;
		legacy_wmv_header(data, 0x10, 8);
		put_unaligned_be16(width, data + 22);
		put_unaligned_be16(height, data + 24);
		memcpy(data + 26, ctx->wmv_sequence, 4);
		pos = 30;
	} else {
		pos = 0;
	}
	while (pos < size) {
		frame_size = get_unaligned_le32(data + pos) & 0x7fffffff;
		memmove(data + pos + 22, data + pos + 8, size - pos - 8);
		legacy_wmv_header(data + pos, 0x0d, frame_size);
		size += 14;
		pos += frame_size + 22;
	}
	*payload_size = size;
	sess->input_frames = frames;
	return 0;
}

struct legacy_bits {
	const u8 *data;
	u32 size;
	u32 pos;
	bool error;
};

static u32 legacy_get_bits(struct legacy_bits *bits, unsigned int count)
{
	u32 value = 0;

	if (count > 32 || count > bits->size - bits->pos) {
		bits->error = true;
		return 0;
	}
	while (count--) {
		value = value << 1 | (bits->data[bits->pos / 8] >>
					(7 - bits->pos % 8) & 1);
		bits->pos++;
	}
	return value;
}

static void legacy_skip_bits(struct legacy_bits *bits, unsigned int count)
{
	if (count > bits->size - bits->pos)
		bits->error = true;
	else
		bits->pos += count;
}

static int mpeg4_vol_time_bits(struct legacy_bits *bits)
{
	u32 version = 1, shape, resolution;

	legacy_skip_bits(bits, 9); /* random_accessible_vol, object type */
	if (legacy_get_bits(bits, 1)) {
		version = legacy_get_bits(bits, 4);
		legacy_skip_bits(bits, 3);
	}
	if (legacy_get_bits(bits, 4) == 15)
		legacy_skip_bits(bits, 16); /* extended pixel aspect */
	if (legacy_get_bits(bits, 1)) {
		legacy_skip_bits(bits, 3); /* chroma_format, low_delay */
		if (legacy_get_bits(bits, 1))
			legacy_skip_bits(bits, 79); /* vbv_parameters */
	}
	shape = legacy_get_bits(bits, 2);
	if (shape == 3 && version != 1)
		legacy_skip_bits(bits, 4);
	if (legacy_get_bits(bits, 1) != 1)
		return -EINVAL;
	resolution = legacy_get_bits(bits, 16);
	if (bits->error || !resolution || legacy_get_bits(bits, 1) != 1)
		return -EINVAL;
	return max(1, fls(resolution - 1));
}

/* Count coded VOPs, not start codes: packed Xvid can contain several pictures
 * and an uncoded dummy VOP. All actual pictures retain the OUTPUT's token.
 * Timing syntax follows ISO/IEC 14496-2 VOL/VOP headers; decoding stays in HW.
 */
static int mpeg4_prepare_input(struct amvdec_session *sess,
			      struct vb2_v4l2_buffer *vbuf, u32 *payload_size)
{
	struct codec_legacy *ctx = sess->priv;
	const u8 *data = vb2_plane_vaddr(&vbuf->vb2_buf, 0);
	u32 size = *payload_size, i, pictures = 0;
	int time_bits = ctx->mpeg4_time_bits;

	if (!data)
		return -EINVAL;
	for (i = 0; i + 4 <= size; i++) {
		struct legacy_bits bits;
		u8 code;

		if (data[i] || data[i + 1] || data[i + 2] != 1)
			continue;
		code = data[i + 3];
		bits = (struct legacy_bits) {
			.data = data + i + 4, .size = (size - i - 4) * 8,
		};
		if ((code & 0xf0) == 0x20) {
			time_bits = mpeg4_vol_time_bits(&bits);
			if (time_bits < 0)
				return time_bits;
		} else if (code == 0xb6) {
			if (!time_bits)
				return -EINVAL; /* no valid VOL has established timing */
			legacy_skip_bits(&bits, 2); /* vop_coding_type */
			while (legacy_get_bits(&bits, 1))
				; /* modulo_time_base */
			if (legacy_get_bits(&bits, 1) != 1)
				return -EINVAL;
			legacy_skip_bits(&bits, time_bits);
			if (legacy_get_bits(&bits, 1) != 1)
				return -EINVAL;
			pictures += legacy_get_bits(&bits, 1); /* vop_coded */
			if (bits.error)
				return -EINVAL;
		}
		i += 3;
	}
	ctx->mpeg4_time_bits = time_bits;
	sess->input_frames = pictures;
	return 0;
}

static int mjpeg_prepare_input(struct amvdec_session *sess,
			       struct vb2_v4l2_buffer *vbuf, u32 *payload_size)
{
	static const u8 second_offset[] = { 12, 14, 22, 24 };
	struct codec_legacy *ctx = sess->priv;
	const u8 *data = vb2_plane_vaddr(&vbuf->vb2_buf, 0);
	u32 size = *payload_size, pos = 2, width = 0, height = 0, i;
	bool fields = false;

	/* Single MJPEG has no references: serialize compressed pictures so that
	 * a source change can update the field canvas split without racing DMA.
	 */
	if (!ctx->configured || atomic_read(&sess->esparser_queued_bufs))
		return -EAGAIN;
	if (!data || size < 4 || data[0] != 0xff || data[1] != 0xd8)
		return -EINVAL;
	while (pos + 4 <= size) {
		u32 length, marker;

		if (data[pos++] != 0xff)
			return -EINVAL;
		while (pos < size && data[pos] == 0xff)
			pos++;
		if (pos >= size)
			return -EINVAL;
		marker = data[pos++];
		if (marker == 0xda)
			break;
		if (pos + 2 > size)
			return -EINVAL;
		length = get_unaligned_be16(data + pos);
		if (length < 2 || length > size - pos)
			return -EINVAL;
		if (marker >= 0xc0 && marker <= 0xcf && marker != 0xc4 &&
		    marker != 0xc8 && marker != 0xcc) {
			/* Baseline sequential JPEG only; do not advertise progressive. */
			if (marker != 0xc0 || length < 8 || data[pos + 2] != 8)
				return -EINVAL;
			height = get_unaligned_be16(data + pos + 3);
			width = get_unaligned_be16(data + pos + 5);
		}
		pos += length;
	}
	/* AVI field JPEG consists of two complete JPEG images in one packet. */
	for (i = pos; i + 3 < size; i++) {
		if (data[i] == 0xff && data[i + 1] == 0xd9 &&
		    data[i + 2] == 0xff && data[i + 3] == 0xd8) {
			height *= 2;
			fields = true;
			break;
		}
	}
	if (!width || !height || width > LEGACY_WIDTH || height > LEGACY_HEIGHT)
		return -EINVAL;
	if ((width != sess->width || height != sess->height) &&
	    !legacy_old_frames_drained(sess))
		return -EAGAIN;
	legacy_dimensions(sess, width, height);
	for (i = 0; i < ctx->buffers; i++) {
		struct vb2_buffer *vb = vb2_get_buffer(v4l2_m2m_get_dst_vq(sess->m2m_ctx),
						 sess->fw_idx_to_vb2_idx[i]);
		u32 stride = LEGACY_WIDTH << fields;
		u8 second = second_offset[i];
		int ret;

		if (!vb)
			return -EINVAL;
		/* Aliasing the two field canvases onto alternate rows performs the
		 * weave in the decoder's DMA engine, not through a CPU pixel copy.
		 */
		ret = legacy_canvas_config(sess, ctx->canvases[i] & 0xff,
			vb2_dma_contig_plane_dma_addr(vb, 0), stride,
			LEGACY_HEIGHT >> fields);
		if (ret)
			return ret;
		ret = legacy_canvas_config(sess, (ctx->canvases[i] >> 8) & 0xff,
			vb2_dma_contig_plane_dma_addr(vb, 1), stride,
			(LEGACY_HEIGHT / 2) >> fields);
		if (ret)
			return ret;
		ret = legacy_canvas_config(sess, second,
			vb2_dma_contig_plane_dma_addr(vb, 0) + LEGACY_WIDTH,
			stride, LEGACY_HEIGHT / 2);
		if (ret)
			return ret;
		ret = legacy_canvas_config(sess, second + 1,
			vb2_dma_contig_plane_dma_addr(vb, 1) + LEGACY_WIDTH,
			stride, LEGACY_HEIGHT / 4);
		if (ret)
			return ret;
	}
	amvdec_write_dos(sess->core, AV_SCRATCH_2, height << 4 | 0x8000);
	return 0;
}


static void legacy_timestamp(struct amvdec_session *sess, u32 offset,
			     struct vb2_v4l2_buffer *buf)
{
	struct amvdec_timestamp *ts, *match = NULL;
	unsigned long flags;
	bool retired = false;

	spin_lock_irqsave(&sess->ts_spinlock, flags);
	list_for_each_entry(ts, &sess->timestamps, list) {
		if ((s32)(offset - ts->offset) >= 0 &&
		    (s32)(offset - ts->end_offset) < 0) {
			match = ts;
			break;
		}
	}
	if (buf) {
		buf->vb2_buf.timestamp = match ? match->ts : 0;
		buf->timecode = match ? match->tc : (struct v4l2_timecode) { 0 };
		buf->flags = match ? match->flags : 0;
	}
	if (match && !--match->remaining) {
		list_del(&match->list);
		retired = true;
	}
	spin_unlock_irqrestore(&sess->ts_spinlock, flags);
	if (retired) {
		kfree(match);
		atomic_dec(&sess->esparser_queued_bufs);
	}
}

static void legacy_complete(struct amvdec_session *sess, u32 idx, u32 offset,
			    u32 field, u32 type, bool damaged)
{
	struct vb2_v4l2_buffer *buf;

	buf = v4l2_m2m_dst_buf_remove_by_idx(sess->m2m_ctx,
					  sess->fw_idx_to_vb2_idx[idx]);
	if (!buf) {
		legacy_abort(sess);
		return;
	}
	legacy_timestamp(sess, offset, buf);
	/* Firmware emits display order; packet PTS follows the stream offset,
	 * not the order in which compressed buffers were submitted.
	 */
	buf->sequence = sess->sequence_cap++;
	buf->field = field;
	sess->field = field;
	buf->flags &= ~(V4L2_BUF_FLAG_KEYFRAME | V4L2_BUF_FLAG_PFRAME | V4L2_BUF_FLAG_BFRAME);
	buf->flags |= type == 0 ? V4L2_BUF_FLAG_KEYFRAME :
		      type == 1 ? V4L2_BUF_FLAG_PFRAME : V4L2_BUF_FLAG_BFRAME;
	if (damaged)
		buf->flags |= V4L2_BUF_FLAG_ERROR;
	vb2_set_plane_payload(&buf->vb2_buf, 0, LEGACY_WIDTH * LEGACY_HEIGHT);
	vb2_set_plane_payload(&buf->vb2_buf, 1, LEGACY_WIDTH * LEGACY_HEIGHT / 2);
	v4l2_m2m_buf_done(buf, VB2_BUF_STATE_DONE);
	schedule_work(&sess->esparser_queue_work);
}

static irqreturn_t legacy_isr(struct amvdec_session *sess)
{
	amvdec_write_dos(sess->core, ASSIST_MBOX1_CLR_REG, 1);
	return IRQ_WAKE_THREAD;
}

static irqreturn_t legacy_threaded_isr(struct amvdec_session *sess)
{
	struct codec_legacy *ctx = sess->priv;
	struct amvdec_core *core = sess->core;
	u32 reg, idx, type, field = V4L2_FIELD_NONE, errors = 0;
	u32 offset, size, width = sess->width, height = sess->height;
	struct v4l2_fract sar = sess->pixelaspect;
	bool metadata_changed;

	if (!ctx || ctx->failed)
		return IRQ_HANDLED;
	if ((ctx->codec == LEGACY_MPEG4 && amvdec_read_dos(core, AV_SCRATCH_L)) ||
	    ((ctx->codec == LEGACY_VC1 || ctx->codec == LEGACY_AVS) &&
	     amvdec_read_dos(core, AV_SCRATCH_7))) {
		legacy_abort(sess);
		return IRQ_HANDLED;
	}
	reg = amvdec_read_dos(core, LEGACY_BUFFEROUT);
	if (!reg)
		return IRQ_HANDLED;
	idx = ctx->codec == LEGACY_MPEG4 ? reg & 7 : ((reg & 7) - 1) & 7;
	if (idx >= ctx->buffers) {
		legacy_abort(sess);
		return IRQ_HANDLED;
	}
	type = ctx->codec == LEGACY_MJPEG ? 0 : (reg >> 3) & 7;
	if (ctx->codec == LEGACY_MPEG4) {
		size = amvdec_read_dos(core, AV_SCRATCH_7);
		width = size >> 16;
		height = size & 0xffff;
	} else if (ctx->codec == LEGACY_VC1) {
		width = amvdec_read_dos(core, AV_SCRATCH_J);
		height = amvdec_read_dos(core, AV_SCRATCH_K);
	} else if (ctx->codec == LEGACY_AVS) {
		width = amvdec_read_dos(core, AV_SCRATCH_1);
		height = amvdec_read_dos(core, AV_SCRATCH_2);
	}
	if (ctx->codec != LEGACY_MJPEG) {
		errors = amvdec_read_dos(core, AV_SCRATCH_6);
		sar = legacy_aspect(ctx->codec, width, height, amvdec_read_dos(core,
			ctx->codec == LEGACY_MPEG4 ? AV_SCRATCH_5 : AV_SCRATCH_0));
		if (reg & BIT(7)) {
			bool top_first = reg & BIT(6);

			if (ctx->codec == LEGACY_VC1)
				top_first = !top_first;
			field = top_first ? V4L2_FIELD_INTERLACED_TB : V4L2_FIELD_INTERLACED_BT;
		}
	} else if (reg & BIT(5)) {
		/* Field canvas aliases already produce alternating raster rows. */
		bool top_first = reg & BIT(4);

		if (reg & BIT(7))
			top_first = !top_first;
		field = top_first ? V4L2_FIELD_INTERLACED_TB : V4L2_FIELD_INTERLACED_BT;
	}
	metadata_changed = !ctx->metadata_ready || width != sess->width ||
		height != sess->height || field != sess->field ||
		sar.numerator != sess->pixelaspect.numerator ||
		sar.denominator != sess->pixelaspect.denominator;
	if (metadata_changed && !legacy_old_frames_drained(sess)) {
		WRITE_ONCE(ctx->source_wait, true);
		return IRQ_HANDLED;
	}
	legacy_dimensions(sess, width, height);
	if (ctx->failed)
		return IRQ_HANDLED;
	sess->pixelaspect = sar;
	sess->field = field;
	if (metadata_changed) {
		static const struct v4l2_event event = {
			.type = V4L2_EVENT_SOURCE_CHANGE,
			.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
		};

		ctx->metadata_ready = true;
		v4l2_event_queue_fh(&sess->fh, &event);
	}
	offset = amvdec_read_dos(core, ctx->codec == LEGACY_MJPEG ?
				 AV_SCRATCH_A : LEGACY_OFFSET);
	if (!sess->keyframe_found && type != 0) {
		/* Keep the surface queued while firmware discards pre-intra output. */
		legacy_recycle(core, idx);
		legacy_timestamp(sess, offset, NULL);
	} else {
		sess->keyframe_found = 1;
		legacy_complete(sess, idx, offset, field, type,
				 errors != ctx->error_count);
	}
	ctx->error_count = errors;
	amvdec_write_dos(core, LEGACY_BUFFEROUT, 0);
	return IRQ_HANDLED;
}

static const u8 mpeg4_eos[SZ_1K] = { 0, 0, 1, 0xb1 };
static const u8 vc1_eos[SZ_1K] = { 0, 0, 1, 0x0a };
static const u8 avs_eos[SZ_1K] = { 0, 0, 1, 0xb1 };
static const u8 mjpeg_eos[SZ_1K] = { 0xff, 0xd9 };
static const u8 *mpeg4_end(u32 *len) { *len = sizeof(mpeg4_eos); return mpeg4_eos; }
static const u8 *vc1_end(u32 *len) { *len = sizeof(vc1_eos); return vc1_eos; }
static const u8 *avs_end(u32 *len) { *len = sizeof(avs_eos); return avs_eos; }
static const u8 *mjpeg_end(u32 *len) { *len = sizeof(mjpeg_eos); return mjpeg_eos; }

struct amvdec_codec_ops codec_mpeg4_ops = {
	.start = mpeg4_start, .stop = legacy_stop,
	.resume = legacy_resume,
	.prepare_input = mpeg4_prepare_input,
	.isr = legacy_isr, .threaded_isr = legacy_threaded_isr,
	.can_recycle = legacy_can_recycle, .recycle = legacy_recycle,
	.eos_sequence = mpeg4_end,
};
struct amvdec_codec_ops codec_vc1_ops = {
	.start = vc1_start, .stop = legacy_stop,
	.resume = legacy_resume, .prepare_input = vc1_prepare_input,
	.isr = legacy_isr, .threaded_isr = legacy_threaded_isr,
	.can_recycle = legacy_can_recycle, .recycle = legacy_recycle,
	.eos_sequence = vc1_end,
};
struct amvdec_codec_ops codec_avs_ops = {
	.start = avs_start, .stop = legacy_stop,
	.resume = legacy_resume,
	.prepare_input = legacy_es_prepare_input,
	.isr = legacy_isr, .threaded_isr = legacy_threaded_isr,
	.can_recycle = legacy_can_recycle, .recycle = legacy_recycle,
	.eos_sequence = avs_end,
};
struct amvdec_codec_ops codec_mjpeg_ops = {
	.start = mjpeg_start, .stop = legacy_stop,
	.resume = legacy_resume,
	.prepare_input = mjpeg_prepare_input,
	.isr = legacy_isr, .threaded_isr = legacy_threaded_isr,
	.can_recycle = legacy_can_recycle, .recycle = legacy_recycle,
	.eos_sequence = mjpeg_end,
};
