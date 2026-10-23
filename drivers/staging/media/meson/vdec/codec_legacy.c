// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SC2 VDEC1 VC1, factory AVSM and MJPEG protocols.
 *
 * Register transactions and workspace relocations adapted from Amlogic's
 * vvc1.c, avs_multi.c and vmjpeg.c (Copyright 2017 Amlogic, Inc.),
 * also present in CoreELEC media_modules-aml a387dea216e416f975377152d374557d448093c8.
 * AVSM uses MULTI run/checkpoint commands but the factory image publishes the
 * avs.c display/geometry ABI. Its BUFFEROUT, not the run slot, owns output.
 */
#include <linux/bitops.h>
#include <linux/dma-mapping.h>
#include <linux/iopoll.h>
#include <linux/mutex.h>
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
#include "vdec_1.h"

#define LEGACY_WIDTH 1920
#define LEGACY_HEIGHT 1088
#define LEGACY_BUFFERIN AV_SCRATCH_8
#define LEGACY_BUFFEROUT AV_SCRATCH_9
#define LEGACY_OFFSET AV_SCRATCH_C
#define LEGACY_WORKSPACE AV_SCRATCH_F
#define LEGACY_CANVAS_ACCESS 0x0014
#define LEGACY_SCRATCH_M 0x2758
#define LEGACY_SCRATCH_N 0x275c
#define LEGACY_AVS_WORKSPACE_SIZE SZ_4M
#define LEGACY_AVS_USERDATA_SIZE SZ_8K
#define LEGACY_AVS_WORKSPACE_BASE 0x01a00000
#define LEGACY_AVS_DECODE_STATUS_INFO 5
#define AVSM_LMEM_SIZE (0x500 * 2)
#define AVSM_LMEM_OFFSET ALIGN(LEGACY_AVS_WORKSPACE_SIZE + LEGACY_AVS_USERDATA_SIZE, PAGE_SIZE)
#define AVSM_DMA_SIZE ALIGN(AVSM_LMEM_OFFSET + AVSM_LMEM_SIZE, PAGE_SIZE)
#define AVSM_REG(word) ((word) * 4)
#define AVSM_BIT_COUNT AVSM_REG(0xc1a)
#define AVSM_CONTROL AVSM_REG(0xc1e)
#define AVSM_REC_CANVAS AVSM_REG(0x9b2)
#define AVSM_CANVAS_DATAL AVSM_REG(0x9ea)
#define AVSM_CANVAS_DATAH AVSM_REG(0x9eb)
#define AVSM_CANVAS_ADDR AVSM_REG(0x9ec)
#define AVSM_CANVAS_CFG0 AVSM_REG(0x9ed)

/* avs_multi.c vavs_save_regs/vavs_restore_regs, vendor word addresses.
 * Scratch4/B are per-run commands, not saved context. I is private LMEM DMA.
 */
static const u16 avsm_context_regs[] = {
	AVSM_REG(0x9c0), AVSM_REG(0x9c1), AVSM_REG(0x9c2), AVSM_REG(0x9c3),
	AVSM_REG(0x9c5), AVSM_REG(0x9c6), AVSM_REG(0x9c7), AVSM_REG(0x9c8),
	AVSM_REG(0x9c9), AVSM_REG(0x9ca), AVSM_REG(0x9cc), AVSM_REG(0x9cd),
	AVSM_REG(0x9ce), AVSM_REG(0x9cf), AVSM_REG(0x9d0), AVSM_REG(0x9d1),
	AVSM_REG(0x9d2), AVSM_REG(0xc09), AVSM_REG(0xc1a),
	AVSM_REG(0x9b2), AVSM_REG(0x9b0), AVSM_REG(0x9b1),
	AVSM_REG(0x992), AVSM_REG(0x990), AVSM_REG(0x991),
	AVSM_REG(0x993), AVSM_REG(0x994), AVSM_REG(0x995),
	AVSM_REG(0xc04), AVSM_REG(0xc1e), AVSM_REG(0xc38),
	AVSM_REG(0xc0c), AVSM_REG(0xc0d), AVSM_REG(0xe00),
	AVSM_REG(0xe0e), AVSM_REG(0xe0c), AVSM_REG(0xc0a),
	AVSM_REG(0xe04), AVSM_REG(0xe19), AVSM_REG(0xe1a),
	AVSM_REG(0xe1b), AVSM_REG(0xc39), AVSM_REG(0x952),
	AVSM_REG(0x908), AVSM_REG(0xc3d), AVSM_REG(0xc18),
};

struct avsm_picture {
	u32 offset;
	u32 info;
	u32 width;
	u32 height;
	struct v4l2_fract aspect;
	/* Completion authority is PIC_DONE/REC or an actual BUFFEROUT. */
	bool valid;
	bool displayed;
	bool consumer;
};

enum legacy_codec { LEGACY_VC1 = 1, LEGACY_AVS, LEGACY_MJPEG };

struct codec_legacy {
	enum legacy_codec codec;
	struct amvdec_session *sess;
	struct delayed_work recycle_work;
	/* Serialize BUFFEROUT publication/acknowledgement with BUFFERIN writes. */
	struct mutex output_lock;
	unsigned long pending_recycle;
	void *workspace;
	dma_addr_t workspace_dma;
	u32 workspace_size;
	u32 buffers;
	u32 canvases[8];
	u32 error_count;
	bool configured;
	u32 mjpeg_width;
	u32 mjpeg_height;
	bool mjpeg_fields;
	u8 wmv_sequence[4];
	bool wmv_sequence_valid;
	bool failed;
	bool source_wait;
	bool metadata_ready;
	bool mjpeg_input_dumped;
	u32 avsm_regs[ARRAY_SIZE(avsm_context_regs)];
	struct avsm_picture avsm_pics[8];
	u8 avsm_display[8];
	u8 avsm_display_head;
	u8 avsm_display_count;
	u16 avsm_decode_count;
	u32 avsm_recycled;
	u32 avsm_run_recycled;
	u32 avsm_busy;
	u32 avsm_committed;
	u32 avsm_run_end;
	u32 avsm_empty_at;
	u32 avsm_frame_offset;
	u32 avsm_frame_bytes;
	u32 avsm_frame_pictures;
	u32 avsm_frame_info;
	bool avsm_input_pending;
	bool avsm_running;
	bool avsm_context_valid;
	bool avsm_empty;
	bool avsm_low_delay;
	bool avsm_plus;
	bool avsm_modern_header;
};

static void legacy_abort(struct amvdec_session *sess);
static irqreturn_t legacy_threaded_isr(struct amvdec_session *sess);
static irqreturn_t legacy_process_output(struct amvdec_session *sess);
static void avsm_service(struct amvdec_session *sess);
static irqreturn_t avsm_process_output(struct amvdec_session *sess);

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

static void legacy_debug_state(struct amvdec_session *sess, const char *reason)
{
	struct codec_legacy *ctx = sess->priv;
	struct amvdec_core *core = sess->core;
	unsigned int i;

	dev_dbg(core->dev,
		"legacy %s codec=%u configured=%u started=%u queued=%d stream=%#x displayed=%u recycle=%#lx\n",
		reason, ctx->codec, ctx->configured, sess->processor_started,
		atomic_read(&sess->esparser_queued_bufs), sess->stream_offset,
		sess->sequence_cap, READ_ONCE(ctx->pending_recycle));
	dev_dbg(core->dev,
		"legacy state mpsr=%#x fifo control=%#x current=%#x wp=%#x rp=%#x level=%#x pic_dc=%#x canvas_mode=%#x\n",
		amvdec_read_dos(core, MPSR),
		amvdec_read_dos(core, VLD_MEM_VIFIFO_CONTROL),
		amvdec_read_dos(core, VLD_MEM_VIFIFO_CURR_PTR),
		amvdec_read_dos(core, VLD_MEM_VIFIFO_WP),
		amvdec_read_dos(core, VLD_MEM_VIFIFO_RP),
		amvdec_read_dos(core, VLD_MEM_VIFIFO_LEVEL),
		amvdec_read_dos(core, MDEC_PIC_DC_CTRL),
		amvdec_read_dos(core, LEGACY_CANVAS_ACCESS));
	for (i = 0; i < 24; i++)
		dev_dbg(core->dev, "legacy scratch[%u]=%#x\n", i,
			amvdec_read_dos(core, AV_SCRATCH_0 + 4 * i));
}

static void legacy_recycle_worker(struct work_struct *work)
{
	struct codec_legacy *ctx = container_of(to_delayed_work(work),
						struct codec_legacy, recycle_work);
	struct amvdec_core *core = ctx->sess->core;
	unsigned long pending;
	u32 idx;

	mutex_lock(&ctx->output_lock);
	if (READ_ONCE(ctx->failed))
		goto unlock;
	if (ctx->codec == LEGACY_AVS) {
		avsm_service(ctx->sess);
		goto unlock;
	}
	if (ctx->codec == LEGACY_MJPEG && !ctx->mjpeg_input_dumped &&
	    READ_ONCE(ctx->sess->stream_offset)) {
		ctx->mjpeg_input_dumped = true;
		legacy_debug_state(ctx->sess, "first input");
	}
	if (ctx->codec == LEGACY_VC1 && amvdec_read_dos(core, AV_SCRATCH_7)) {
		legacy_abort(ctx->sess);
		goto unlock;
	}
	if (READ_ONCE(ctx->source_wait) && legacy_old_frames_drained(ctx->sess)) {
		WRITE_ONCE(ctx->source_wait, false);
		legacy_process_output(ctx->sess);
	}
	if (READ_ONCE(ctx->failed))
		goto unlock;
	pending = READ_ONCE(ctx->pending_recycle);
	if (pending && !amvdec_read_dos(core, LEGACY_BUFFERIN)) {
		idx = __ffs(pending);
		clear_bit(idx, &ctx->pending_recycle);
		amvdec_write_dos(core, LEGACY_BUFFERIN,
			ctx->codec == LEGACY_MJPEG ? idx + 1 : ~(u32)BIT(idx));
	}
unlock:
	mutex_unlock(&ctx->output_lock);
	if (!READ_ONCE(ctx->failed))
		schedule_delayed_work(&ctx->recycle_work,
			msecs_to_jiffies(READ_ONCE(ctx->pending_recycle) ? 1 : 10));
}

/* JPEG adds twelve to the first bank's canvas IDs. */
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

static int legacy_canvas_config(struct amvdec_session *sess, u8 id,
				dma_addr_t dma, u32 stride, u32 height)
{
	int ret;

	if (upper_32_bits(dma))
		return -ERANGE;
	ret = meson_canvas_config(sess->core->canvas, id, dma, stride,
				  height, MESON_CANVAS_WRAP_NONE,
				  MESON_CANVAS_BLKMODE_LINEAR, MESON_CANVAS_ENDIAN_SWAP64);
	if (ret)
		return ret;
	/* The VDEC-local canvas mode must follow the global DMC canvas entry. */
	amvdec_write_dos(sess->core, LEGACY_CANVAS_ACCESS,
			 BIT(11) | BIT(8) | id);
	return 0;
}

static int legacy_set_canvases(struct amvdec_session *sess)
{
	struct codec_legacy *ctx = sess->priv;
	struct v4l2_m2m_buffer *buf;
	bool planar = ctx->codec == LEGACY_MJPEG;
	u32 planes = planar ? 3 : 2, count = ctx->buffers * planes;
	u32 i = 0, start = sess->canvas_num, p;
	int ret;

	if ((planar ? sess->pixfmt_cap != V4L2_PIX_FMT_YUV420M :
		      sess->pixfmt_cap != V4L2_PIX_FMT_NV12M) ||
	    sess->fmt_out->capture_width != LEGACY_WIDTH ||
	    sess->fmt_out->capture_height != LEGACY_HEIGHT)
		return -EINVAL;
	/* SC2 JPEG adds twelve to each primary canvas component for field two,
	 * as observed in PSCALE_CANVAS_WR_ADDR. Own both banks after relocation.
	 */
	ret = legacy_canvas_range(sess, planar ? count * 2 : count);
	if (ret)
		return ret;

	v4l2_m2m_for_each_dst_buf(sess->m2m_ctx, buf) {
		if (i >= ctx->buffers) {
			ret = -EINVAL;
			goto fail;
		}
		ctx->canvases[i] = 0;
		for (p = 0; p < planes; p++) {
			dma_addr_t dma = vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, p);
			u32 stride = LEGACY_WIDTH >> (planar && p);
			u32 height = LEGACY_HEIGHT >> !!p;
			u8 id = sess->canvas_alloc[start + i * planes + p];

			if (vb2_plane_size(&buf->vb.vb2_buf, p) < stride * height) {
				ret = -EINVAL;
				goto fail;
			}
			ret = legacy_canvas_config(sess, id, dma, stride, height);
			if (ret)
				goto fail;
			ctx->canvases[i] |= id << (8 * p);
			if (ctx->codec == LEGACY_MJPEG) {
				u32 field_height = DIV_ROUND_UP(sess->height, 2) >> !!p;
				u8 second = id + 12;

				ret = legacy_canvas_config(sess, second,
						 dma + stride * field_height, stride, height / 2);
				if (ret)
					goto fail;
			}
		}
		if (!planar)
			ctx->canvases[i] |= (ctx->canvases[i] & 0xff00) << 8;
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
	if (ctx->codec != LEGACY_AVS)
		for (i = 0; i < min(ctx->buffers, 4U); i++)
			amvdec_write_dos(core, (ctx->codec == LEGACY_MJPEG ?
					 AV_SCRATCH_4 : AV_SCRATCH_0) + 4 * i,
					 ctx->canvases[i]);
	amvdec_write_dos(core, PSCALE_CTRL, 0);
	amvdec_write_dos(core, LEGACY_BUFFERIN, 0);
	amvdec_write_dos(core, LEGACY_BUFFEROUT, 0);
	/* Bit17 selects interleaved chroma; bit16=0 selects UV, not VU. */
	if (sess->pixfmt_cap == V4L2_PIX_FMT_YUV420M)
		amvdec_clear_dos_bits(core, MDEC_PIC_DC_CTRL, BIT(17));
	else
		amvdec_write_dos_bits(core, MDEC_PIC_DC_CTRL, BIT(17));
	amvdec_clear_dos_bits(core, MDEC_PIC_DC_CTRL, BIT(16));



	switch (ctx->codec) {
	case LEGACY_VC1:
		amvdec_write_dos(core, LEGACY_WORKSPACE,
				 lower_32_bits(ctx->workspace_dma) - 0x01f00000);
		amvdec_write_dos(core, AV_SCRATCH_7, 0);
		amvdec_write_dos(core, AV_SCRATCH_4,
				 sess->fmt_out->pixfmt == V4L2_PIX_FMT_VC1_ANNEX_G);
		break;
	case LEGACY_AVS:
		/* The four dynamic-table words occupy SCRATCH0..3. These register
		 * constants are byte addresses; i is a surface index, not a register
		 * offset. */
		for (i = 0; i < ctx->buffers; i += 2)
			amvdec_write_dos(core, AV_SCRATCH_0 + 4 * (i >> 1),
				(ctx->canvases[i] & 0xffff) |
				(ctx->canvases[i + 1] & 0xffff) << 16);

		amvdec_write_dos(core, AV_SCRATCH_7, 0);
		amvdec_write_dos(core, AV_SCRATCH_D, 0);
		amvdec_write_dos(core, AV_SCRATCH_E, 0);
		amvdec_write_dos(core, LEGACY_SCRATCH_M, 0);
		amvdec_write_dos(core, LEGACY_WORKSPACE,
				 lower_32_bits(ctx->workspace_dma) -
				 LEGACY_AVS_WORKSPACE_BASE);
		/* Factory AVSM stream mode; run commands follow committed input. */
		amvdec_write_dos(core, AV_SCRATCH_6, 2);
		amvdec_write_dos(core, AV_SCRATCH_H, 0);
		amvdec_write_dos(core, AV_SCRATCH_I,
			lower_32_bits(ctx->workspace_dma + AVSM_LMEM_OFFSET));
		amvdec_write_dos(core, AV_SCRATCH_J, 0);
		amvdec_write_dos(core, AV_SCRATCH_G, 1);
		/* The image requires its own 8 KiB userdata DMA ring. Like vendor
		 * AV_SCRATCH_N = userdata_dma - buf_offset, this is relative to F.
		 */
		amvdec_write_dos(core, LEGACY_SCRATCH_N,
				 LEGACY_AVS_WORKSPACE_BASE + LEGACY_AVS_WORKSPACE_SIZE);
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
	legacy_debug_state(sess, "configured");
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
	mutex_init(&ctx->output_lock);
	INIT_DELAYED_WORK(&ctx->recycle_work, legacy_recycle_worker);
	ctx->buffers = codec == LEGACY_AVS ? 8 : 4;
	ctx->workspace_size = codec == LEGACY_AVS ?
			      AVSM_DMA_SIZE :
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

static int vc1_start(struct amvdec_session *s) { return legacy_start(s, LEGACY_VC1); }
static int avs_start(struct amvdec_session *s)
{
	if (s->fmt_out->firmware_index != MESON_AVS_FIRMWARE_INDEX)
		return -EINVAL;
	return legacy_start(s, LEGACY_AVS);
}
static int mjpeg_start(struct amvdec_session *s) { return legacy_start(s, LEGACY_MJPEG); }

static int legacy_stop(struct amvdec_session *sess)
{
	struct codec_legacy *ctx = sess->priv;

	legacy_debug_state(sess, "stopping");
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

	legacy_debug_state(sess, "aborting");
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

static int mjpeg_prepare_input(struct amvdec_session *sess,
			       struct vb2_v4l2_buffer *vbuf, u32 *payload_size)
{
	struct codec_legacy *ctx = sess->priv;
	const u8 *data = vb2_plane_vaddr(&vbuf->vb2_buf, 0);
	u32 size = *payload_size, pos = 2, width = 0, height = 0, i;
	bool fields = false;

	if (!ctx->configured)
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
	/* The single-instance firmware consumes a stream, not one submitted
	 * picture at a time. Keep its lookahead supplied while the canvas layout
	 * is unchanged; only a real layout change needs outstanding DMA drained.
	 */
	if (width == ctx->mjpeg_width && height == ctx->mjpeg_height &&
	    fields == ctx->mjpeg_fields)
		return 0;
	if (atomic_read(&sess->esparser_queued_bufs))
		return -EAGAIN;
	if ((width != sess->width || height != sess->height) &&
	    !legacy_old_frames_drained(sess))
		return -EAGAIN;
	legacy_dimensions(sess, width, height);
	for (i = 0; i < ctx->buffers; i++) {
		struct vb2_buffer *vb = vb2_get_buffer(v4l2_m2m_get_dst_vq(sess->m2m_ctx),
						 sess->fw_idx_to_vb2_idx[i]);
		u32 p;
		int ret;

		if (!vb)
			return -EINVAL;
		/* Aliasing the two field canvases onto alternate rows performs the
		 * weave in the decoder's DMA engine, not through a CPU pixel copy.
		 */
		for (p = 0; p < 3; p++) {
			u32 pitch = LEGACY_WIDTH >> !!p;
			u32 stride = pitch << fields;
			u32 canvas_height = LEGACY_HEIGHT >> !!p;
			u8 primary = (ctx->canvases[i] >> (8 * p)) & 0xff;
			u8 second = primary + 12;
			dma_addr_t dma = vb2_dma_contig_plane_dma_addr(vb, p);

			ret = legacy_canvas_config(sess, primary, dma, stride,
						 canvas_height >> fields);
			if (ret)
				return ret;
			ret = legacy_canvas_config(sess, second, dma + pitch,
						 stride, canvas_height / 2);
			if (ret)
				return ret;
		}
	}
	amvdec_write_dos(sess->core, AV_SCRATCH_2, height << 4 | 0x8000);
	ctx->mjpeg_width = width;
	ctx->mjpeg_height = height;
	ctx->mjpeg_fields = fields;
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
	if (sess->pixfmt_cap == V4L2_PIX_FMT_YUV420M) {
		vb2_set_plane_payload(&buf->vb2_buf, 1, LEGACY_WIDTH * LEGACY_HEIGHT / 4);
		vb2_set_plane_payload(&buf->vb2_buf, 2, LEGACY_WIDTH * LEGACY_HEIGHT / 4);
	} else {
		vb2_set_plane_payload(&buf->vb2_buf, 1, LEGACY_WIDTH * LEGACY_HEIGHT / 2);
	}
	v4l2_m2m_buf_done(buf, VB2_BUF_STATE_DONE);
	schedule_work(&sess->esparser_queue_work);
}

static void avsm_abort(struct amvdec_session *sess, const char *reason, u32 index)
{
	struct codec_legacy *ctx = sess->priv;
	struct amvdec_core *core = sess->core;
	u32 valid = 0, displayed = 0, consumer = 0, i;

	for (i = 0; i < ctx->buffers; i++) {
		valid |= ctx->avsm_pics[i].valid ? BIT(i) : 0;
		displayed |= ctx->avsm_pics[i].displayed ? BIT(i) : 0;
		consumer |= ctx->avsm_pics[i].consumer ? BIT(i) : 0;
	}
	dev_err(core->dev,
		"AVSM failure=%s slot=%u h=%#x bo=%#x rec=%#x scratch4=%#x g=%#x offset=%#x wh=%#x/%#x running=%u count=%u busy=%#x valid=%#x displayed=%#x consumer=%#x recycle=%#x sent=%#x\n",
		reason, index, amvdec_read_dos(core, AV_SCRATCH_H),
		amvdec_read_dos(core, LEGACY_BUFFEROUT),
		amvdec_read_dos(core, AVSM_REC_CANVAS),
		amvdec_read_dos(core, AV_SCRATCH_4),
		amvdec_read_dos(core, AV_SCRATCH_G),
		amvdec_read_dos(core, LEGACY_OFFSET),
		amvdec_read_dos(core, AV_SCRATCH_1),
		amvdec_read_dos(core, AV_SCRATCH_2), ctx->avsm_running,
		ctx->avsm_decode_count, ctx->avsm_busy, valid, displayed,
		consumer, ctx->avsm_recycled, ctx->avsm_run_recycled);
	dev_err(core->dev,
		"AVSM input committed=%#x run_end=%#x context=%u rp=%#x wp=%#x level=%#x bits=%#x queued=%d display_queue=%u\n",
		ctx->avsm_committed, ctx->avsm_run_end, ctx->avsm_context_valid,
		amvdec_read_dos(core, VLD_MEM_VIFIFO_RP),
		amvdec_read_dos(core, VLD_MEM_VIFIFO_WP),
		amvdec_read_dos(core, VLD_MEM_VIFIFO_LEVEL),
		amvdec_read_dos(core, AVSM_BIT_COUNT),
		atomic_read(&sess->esparser_queued_bufs), ctx->avsm_display_count);
	legacy_abort(sess);
}


static void avsm_reset_processor(struct amvdec_core *core)
{
	/* amvdec_start/amvdec_stop require these bus reads around CPU reset. */
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_write_dos(core, DOS_SW_RESET0, BIT(12) | BIT(11));
	amvdec_write_dos(core, DOS_SW_RESET0, 0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);
}

static int avsm_stop_processor(struct amvdec_session *sess)
{
	struct codec_legacy *ctx = sess->priv;
	struct amvdec_core *core = sess->core;
	u32 value;
	int ret;

	amvdec_write_dos(core, MPSR, 0);
	amvdec_write_dos(core, CPSR, 0);
	ctx->avsm_running = false;
	WRITE_ONCE(sess->processor_started, false);
	ret = readl_poll_timeout(core->dos_base + IMEM_DMA_CTRL, value,
				 !(value & BIT(15)), 10, 100000);
	if (!ret)
		ret = readl_poll_timeout(core->dos_base + LMEM_DMA_CTRL, value,
					 !(value & BIT(15)), 10, 100000);
	if (!ret)
		avsm_reset_processor(core);
	return ret;
}

static void avsm_queue_display(struct codec_legacy *ctx, unsigned int index)
{
	unsigned int tail;

	if (index >= ctx->buffers || !ctx->avsm_pics[index].valid ||
	    ctx->avsm_pics[index].displayed || ctx->avsm_display_count == 8) {
		avsm_abort(ctx->sess, "display-queue", index);
		return;
	}
	tail = (ctx->avsm_display_head + ctx->avsm_display_count) & 7;
	ctx->avsm_display[tail] = index;
	ctx->avsm_display_count++;
	ctx->avsm_pics[index].displayed = true;
	ctx->avsm_pics[index].consumer = true;
}

static bool avsm_has_timestamp(struct amvdec_session *sess, u32 offset)
{
	struct amvdec_timestamp *ts;
	unsigned long flags;
	unsigned int matches = 0;

	spin_lock_irqsave(&sess->ts_spinlock, flags);
	list_for_each_entry(ts, &sess->timestamps, list) {
		if (ts->remaining && (s32)(offset - ts->offset) >= 0 &&
		    (s32)(offset - ts->end_offset) < 0)
			matches++;
	}
	spin_unlock_irqrestore(&sess->ts_spinlock, flags);
	return matches == 1;
}

static void avsm_publish(struct amvdec_session *sess)
{
	struct codec_legacy *ctx = sess->priv;

	while (ctx->avsm_display_count && !ctx->failed) {
		u32 index = ctx->avsm_display[ctx->avsm_display_head];
		struct avsm_picture *pic = &ctx->avsm_pics[index];
		u32 field = V4L2_FIELD_NONE;
		bool change;

		if (pic->info & BIT(7))
			field = pic->info & BIT(6) ? V4L2_FIELD_INTERLACED_TB :
						     V4L2_FIELD_INTERLACED_BT;
		change = !ctx->metadata_ready || sess->width != pic->width ||
			sess->height != pic->height || sess->field != field ||
			sess->pixelaspect.numerator != pic->aspect.numerator ||
			sess->pixelaspect.denominator != pic->aspect.denominator;
		if (change && !legacy_old_frames_drained(sess))
			return;
		legacy_dimensions(sess, pic->width, pic->height);
		if (ctx->failed)
			return;
		/* Offset belongs to BUFFEROUT's picture, including delayed I/P
		 * output. Never publish a zero-PTS substitute for a missing token.
		 */
		if (!avsm_has_timestamp(sess, pic->offset)) {
			dev_err(sess->core->dev, "AVSM unmatched display token slot=%u offset=%#x\n",
				index, pic->offset);
			avsm_abort(sess, "display-token", index);
			return;
		}
		sess->pixelaspect = pic->aspect;
		sess->field = field;
		if (change) {
			static const struct v4l2_event event = {
				.type = V4L2_EVENT_SOURCE_CHANGE,
				.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
			};

			ctx->metadata_ready = true;
			v4l2_event_queue_fh(&sess->fh, &event);
		}
		ctx->avsm_display_head = (ctx->avsm_display_head + 1) & 7;
		ctx->avsm_display_count--;
		pic->consumer = true;
		/* Vendor field views use this same raster; native CAPTURE describes
		 * both fields in one interlaced buffer, without copying pixels.
		 */
		legacy_complete(sess, index, pic->offset, field,
				(pic->info >> 3) & 7, false);
	}
}

static void avsm_start_run(struct amvdec_session *sess, u32 slot)
{
	struct codec_legacy *ctx = sess->priv;
	struct amvdec_core *core = sess->core;
	u32 i, rp, wp;
	const char *reason = "run-reset";

	/* A frame job owns one coded access unit and its zero-filled fetch tail. */
	if (amvdec_vdec1_reset(sess, 0xfffffff0))
		goto fail;
	/* vdec_prepare_input resets and powers VLD before restoring its FIFO. */
	amvdec_write_dos(core, VLD_MEM_VIFIFO_CONTROL, 0);
	amvdec_write_dos(core, DOS_SW_RESET0, BIT(3) | BIT(4) | BIT(5));
	amvdec_write_dos(core, DOS_SW_RESET0, 0);
	amvdec_write_dos(core, POWER_CTL_VLD, BIT(4));
	rp = lower_32_bits(sess->vififo_paddr) +
		ctx->avsm_frame_offset % sess->vififo_size;
	wp = lower_32_bits(sess->vififo_paddr) +
		((ctx->avsm_frame_offset % sess->vififo_size +
		  ctx->avsm_frame_bytes + SZ_1K) & ~7U) % sess->vififo_size;
	amvdec_write_dos(core, VLD_MEM_VIFIFO_START_PTR,
		lower_32_bits(sess->vififo_paddr));
	amvdec_write_dos(core, VLD_MEM_VIFIFO_END_PTR,
		lower_32_bits(sess->vififo_paddr) + sess->vififo_size - 8);
	amvdec_write_dos(core, VLD_MEM_VIFIFO_CURR_PTR, rp);
	amvdec_write_dos(core, VLD_MEM_VIFIFO_CONTROL, 1);
	amvdec_write_dos(core, VLD_MEM_VIFIFO_CONTROL, 0);
	amvdec_write_dos(core, VLD_MEM_VIFIFO_BUF_CNTL, MEM_BUFCTRL_MANUAL);
	amvdec_write_dos(core, VLD_MEM_VIFIFO_RP, rp);
	amvdec_write_dos(core, VLD_MEM_VIFIFO_WP, wp);
	amvdec_write_dos(core, VLD_MEM_VIFIFO_BUF_CNTL, MEM_BUFCTRL_MANUAL | 1);
	amvdec_write_dos(core, VLD_MEM_VIFIFO_BUF_CNTL, MEM_BUFCTRL_MANUAL);
	amvdec_write_dos(core, VLD_MEM_VIFIFO_CONTROL,
			 (0x11 << 16) | BIT(10) | (7 << 3));
	/* vavs_prot_init resets the codec after preparing its input reservoir. */
	amvdec_write_dos(core, DOS_SW_RESET0, BIT(7) | BIT(6) | BIT(4));
	amvdec_write_dos(core, DOS_SW_RESET0, 0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_write_dos(core, DOS_SW_RESET0, BIT(7) | BIT(6) | BIT(4));
	amvdec_write_dos(core, DOS_SW_RESET0, 0);
	amvdec_write_dos(core, DOS_SW_RESET0, BIT(9) | BIT(8));
	amvdec_write_dos(core, DOS_SW_RESET0, 0);
	amvdec_write_dos(core, POWER_CTL_VLD, BIT(4));
	amvdec_clear_dos_bits(core, VLD_MEM_VIFIFO_CONTROL, GENMASK(23, 16));
	amvdec_write_dos_bits(core, VLD_MEM_VIFIFO_CONTROL, 2 << 16 | 8 << 18);
	if (ctx->avsm_context_valid) {
		for (i = 0; i < ARRAY_SIZE(avsm_context_regs); i++)
			amvdec_write_dos(core, avsm_context_regs[i], ctx->avsm_regs[i]);
	} else {
		for (i = 0; i < ctx->buffers; i += 2)
			amvdec_write_dos(core, AV_SCRATCH_0 + 4 * (i >> 1),
				(ctx->canvases[i] & 0xffff) |
				(ctx->canvases[i + 1] & 0xffff) << 16);
		amvdec_write_dos(core, AV_SCRATCH_7, 0);
		amvdec_write_dos(core, AV_SCRATCH_D, 0);
		amvdec_write_dos(core, AV_SCRATCH_E, 0);
		amvdec_write_dos(core, LEGACY_SCRATCH_M, 0);
		amvdec_write_dos(core, LEGACY_WORKSPACE,
			lower_32_bits(ctx->workspace_dma) - LEGACY_AVS_WORKSPACE_BASE);
		amvdec_write_dos(core, LEGACY_BUFFERIN, 0);
		amvdec_write_dos(core, LEGACY_BUFFEROUT, 0);
		amvdec_write_dos(core, LEGACY_SCRATCH_N,
			LEGACY_AVS_WORKSPACE_BASE + LEGACY_AVS_WORKSPACE_SIZE);
	}
	/* SC2's full core reset clears its decoder-local LUT. Populate the
	 * imported surfaces there, as config_cav_lut_ex(VDEC_1) does. The
	 * global DMC table alone is not the decoder's lookup authority. */
	amvdec_write_dos(core, AVSM_CANVAS_CFG0, 0);
	for (i = 0; i < ctx->buffers; i++) {
		struct vb2_buffer *vb = vb2_get_buffer(
			v4l2_m2m_get_dst_vq(sess->m2m_ctx), sess->fw_idx_to_vb2_idx[i]);
		u32 p;

		if (!vb)
			goto fail;
		for (p = 0; p < 2; p++) {
			dma_addr_t dma = vb2_dma_contig_plane_dma_addr(vb, p);
			u32 width = LEGACY_WIDTH / 8;

			amvdec_write_dos(core, AVSM_CANVAS_DATAL,
				lower_32_bits(dma >> 3) | ((width & 7) << 29));
			amvdec_write_dos(core, AVSM_CANVAS_DATAH,
				(width >> 3) | ((LEGACY_HEIGHT >> p) << 9) |
				(MESON_CANVAS_ENDIAN_SWAP64 << 26));
			amvdec_write_dos(core, AVSM_CANVAS_ADDR,
				(ctx->canvases[i] >> (8 * p)) & 0xff);
		}
	}
	amvdec_write_dos(core, AV_SCRATCH_4, slot);
	amvdec_write_dos(core, AV_SCRATCH_B, 0);
	amvdec_write_dos(core, AV_SCRATCH_6, 1);
	/* vavs_prot_init uses firmware recovery mode 1 on every run. */
	amvdec_write_dos(core, AV_SCRATCH_G, 1);
	amvdec_write_dos(core, AV_SCRATCH_I,
		lower_32_bits(ctx->workspace_dma + AVSM_LMEM_OFFSET));
	amvdec_write_dos(core, AV_SCRATCH_J, 0);
	if (ctx->avsm_context_valid)
		amvdec_write_dos(core, LEGACY_SCRATCH_N, 0);
	amvdec_write_dos(core, PSCALE_CTRL, 0);
	amvdec_write_dos_bits(core, MDEC_PIC_DC_CTRL, BIT(17));
	amvdec_clear_dos_bits(core, MDEC_PIC_DC_CTRL, BIT(16) | BIT(31));
	/* Must precede vdec_enable_input(), also on resumed runs. */
	amvdec_write_dos(core, AVSM_CONTROL, 0x4c80);
	amvdec_write_dos_bits(core, VLD_MEM_VIFIFO_CONTROL, BIT(2) | BIT(1));
	amvdec_write_dos(core, AVSM_BIT_COUNT, ctx->avsm_frame_bytes * 8);
	ctx->avsm_run_recycled = ctx->avsm_recycled;
	amvdec_write_dos(core, AV_SCRATCH_H,
		ctx->avsm_decode_count | ((~ctx->avsm_run_recycled & 0xffff) << 16));
	ctx->avsm_run_end = ctx->avsm_committed;
	amvdec_write_dos(core, ASSIST_MBOX1_CLR_REG, 1);
	amvdec_write_dos(core, ASSIST_MBOX1_MASK, 1);
	avsm_reset_processor(core);
	ctx->avsm_running = true;
	WRITE_ONCE(sess->processor_started, true);
	amvdec_write_dos(core, MPSR, 1);
	return;
fail:
	avsm_abort(sess, reason, slot);
}

static void avsm_service(struct amvdec_session *sess)
{
	struct codec_legacy *ctx = sess->priv;
	u32 i;

	if (ctx->failed)
		return;
	if (ctx->configured && amvdec_read_dos(sess->core, AV_SCRATCH_7)) {
		avsm_abort(sess, "service-sos", U32_MAX);
		return;
	}

	for (i = 0; i < ctx->buffers; i++) {
		if (test_and_clear_bit(i, &ctx->pending_recycle)) {
			ctx->avsm_pics[i].consumer = false;
			ctx->avsm_recycled |= BIT(i);
		}
	}
	avsm_publish(sess);
	if (ctx->failed || !ctx->configured || ctx->avsm_running ||
	    !ctx->avsm_input_pending)
		return;
	for (i = 0; i < ctx->buffers; i++) {
		struct avsm_picture *pic = &ctx->avsm_pics[i];

		if (!(ctx->avsm_busy & ~ctx->avsm_recycled & BIT(i)) &&
		    !pic->consumer && (!pic->valid || pic->displayed)) {
			avsm_start_run(sess, i);
			return;
		}
	}
}

static irqreturn_t avsm_process_output(struct amvdec_session *sess)
{
	struct codec_legacy *ctx = sess->priv;
	struct amvdec_core *core = sess->core;
	u32 status = amvdec_read_dos(core, AV_SCRATCH_H);
	u32 result = status & 0xff;
	u32 reg, index = U32_MAX, type, rec, i;
	bool terminal = result >= 1 && result <= 4;
	struct avsm_picture *pic;
	const char *reason = "irq-sos";

	if (status == LEGACY_AVS_DECODE_STATUS_INFO) {
		amvdec_write_dos(core, AV_SCRATCH_H, 0);
		return IRQ_HANDLED;
	}
	if (amvdec_read_dos(core, AV_SCRATCH_E) & 0xffff) {
		amvdec_write_dos(core, AV_SCRATCH_E, 0);
		return IRQ_HANDLED;
	}
	if (amvdec_read_dos(core, LEGACY_SCRATCH_N) & BIT(15)) {
		amvdec_write_dos(core, LEGACY_SCRATCH_N, 0);
		return IRQ_HANDLED;
	}
	if (amvdec_read_dos(core, AV_SCRATCH_7))
		goto fail;
	if (terminal) {
		reason = "completion-idle";
		if (!ctx->avsm_running) {
			/* A stopped run can leave its mailbox status latched until the
			 * threaded handler acknowledges it. Ignore that stale event. */
			amvdec_write_dos(core, AV_SCRATCH_H, 0);
			return IRQ_HANDLED;
		}
		reason = "processor-stop";
		if (avsm_stop_processor(sess))
			goto fail;
		/* Prevent the stopped run's terminal status from retriggering the
		 * mailbox before the next run programs its command word. */
		amvdec_write_dos(core, AV_SCRATCH_H, 0);
		/* avs_multi ISR: low byte is the result, next byte is the
		 * SKIP continuation start code, upper half is a busy bitmap.
		 * No part of H encodes the displayed picture's slot.
		 */
		ctx->avsm_busy = status >> 16;
		reason = "busy-range";
		if (ctx->avsm_busy & ~GENMASK(ctx->buffers - 1, 0))
			goto fail;
		if (ctx->avsm_frame_pictures && result == 1) {
			/* Only PIC_DONE proves that REC contains a complete raster.
			 * EMPTY can also name a partially reconstructed surface.
			 */
			rec = amvdec_read_dos(core, AVSM_REC_CANVAS) & 0xffff;
			for (index = 0; index < ctx->buffers; index++)
				if ((ctx->canvases[index] & 0xffff) == rec)
					break;
			reason = "rec-unowned";
			if (index == ctx->buffers)
				goto fail;
			reason = "rec-not-busy";
			if (!(ctx->avsm_busy & BIT(index)))
				goto fail;
			pic = &ctx->avsm_pics[index];
			reason = "rec-lifetime";
			if (pic->consumer || (pic->valid && !pic->displayed))
				goto fail;
			*pic = (struct avsm_picture) {
				.valid = true, .offset = ctx->avsm_frame_offset,
				.info = ctx->avsm_frame_info,
				.width = amvdec_read_dos(core, AV_SCRATCH_1),
				.height = amvdec_read_dos(core, AV_SCRATCH_2),
			};
			pic->aspect = legacy_aspect(LEGACY_AVS, pic->width, pic->height,
				amvdec_read_dos(core, AV_SCRATCH_0));
		}
	}
	reg = amvdec_read_dos(core, LEGACY_BUFFEROUT);
	if (reg) {
		/* BUFFEROUT names a displayed surface. Its token was attached to
		 * that surface when the frame job completed, before later reordering. */
		index = ((reg & 7) + (((reg >> 8) & 3) << 3) - 1) & 0x1f;
		type = (reg >> 3) & 7;
		reason = "display-identity";
		if (index >= ctx->buffers || type > 2)
			goto fail;
		pic = &ctx->avsm_pics[index];
		reason = "display-lifetime";
		if (pic->displayed || pic->consumer) {
			/* A repeated BUFFEROUT for a surface still owned by CAPTURE is
			 * stale mailbox state, not a second presentation. */
			amvdec_write_dos(core, LEGACY_BUFFEROUT, 0);
			goto terminal_complete;
		}
		reason = "display-incomplete";
		if (!pic->valid)
			goto fail;
		pic->info = reg;
		/* avs.c AVS_PIC_WIDTH/HEIGHT, not avs_multi's packed PIC_INFO.
		 * The live image reports the two words separately.
		 */
		pic->width = amvdec_read_dos(core, AV_SCRATCH_1);
		pic->height = amvdec_read_dos(core, AV_SCRATCH_2);
		pic->aspect = legacy_aspect(LEGACY_AVS, pic->width, pic->height,
			amvdec_read_dos(core, AV_SCRATCH_0));
		dev_dbg(core->dev,
			"legacy output codec=2 reg=%#x index=%u offset=%#x type=%u repeat=%u errors=0 status=%#x rec=%#x dc_status=%#x anc0=%#x anc1=%#x anc2=%#x\n",
			reg, index, pic->offset, type,
			amvdec_read_dos(core, AV_SCRATCH_A), status,
			amvdec_read_dos(core, AVSM_REC_CANVAS),
			amvdec_read_dos(core, MDEC_PIC_DC_STATUS),
			amvdec_read_dos(core, AVSM_REG(0x990)),
			amvdec_read_dos(core, AVSM_REG(0x991)),
			amvdec_read_dos(core, AVSM_REG(0x992)));
		avsm_queue_display(ctx, index);
		if (ctx->failed)
			return IRQ_HANDLED;
		amvdec_write_dos(core, LEGACY_BUFFEROUT, 0);
	}
terminal_complete:
	if (terminal && ctx->avsm_frame_pictures && (result == 2 || result == 3)) {
		reason = "incomplete-picture";
		goto fail;
	}
	if (!terminal) {
		avsm_publish(sess);
		return IRQ_HANDLED;
	}
	if (++ctx->avsm_decode_count == 0)
		ctx->avsm_decode_count++;
	for (i = 0; i < ARRAY_SIZE(avsm_context_regs); i++)
		ctx->avsm_regs[i] = amvdec_read_dos(core, avsm_context_regs[i]);
	ctx->avsm_recycled &= ~ctx->avsm_run_recycled;
	ctx->avsm_context_valid = true;
	ctx->avsm_empty = true;
	ctx->avsm_empty_at = ctx->avsm_run_end;
	ctx->avsm_input_pending = false;
	schedule_work(&sess->esparser_queue_work);
	avsm_service(sess);
	return IRQ_HANDLED;
fail:
	avsm_abort(sess, reason, index);
	return IRQ_HANDLED;
}

/* The pending reference has no BUFFEROUT yet. Preserve its coded header
 * metadata, rather than borrowing flags from a different displayed picture.
 */
static u32 avsm_read_bits(const u8 *data, u32 bytes, u32 *pos, u32 count)
{
	u32 value = 0;

	if (count > 32 || *pos > bytes * 8 || count > bytes * 8 - *pos) {
		*pos = U32_MAX;
		return 0;
	}
	while (count) {
		u32 bits = min(8 - (*pos & 7), count);

		value = (value << bits) |
			((data[*pos >> 3] >> (8 - (*pos & 7) - bits)) &
			 ((1U << bits) - 1));
		*pos += bits;
		count -= bits;
	}
	return value;
}

static int avsm_picture_info(struct codec_legacy *ctx, const u8 *data,
			     u32 bytes, u32 *info)
{
	u32 i;

	for (i = 0; i + 4 <= bytes; i++) {
		const u8 *header;
		u32 size, pos, peek, type, progressive, structure = 1, top;
		u8 code;

		if (data[i] || data[i + 1] || data[i + 2] != 1)
			continue;
		code = data[i + 3];
		header = data + i + 4;
		size = bytes - i - 4;
		if (code == 0xb0) {
			/* low_delay is sequence-header bit 89, MSB first. */
			if (size < 12)
				return -EINVAL;
			ctx->avsm_plus = header[0] == 0x48;
			ctx->avsm_low_delay = header[11] & BIT(6);
		}
		if (code != 0xb3 && code != 0xb6)
			continue;
		/* AVS+ adds marker_bit and seven bbv_delay_extension bits. */
		pos = ctx->avsm_plus ? 24 : 16;
		type = code == 0xb6 ? avsm_read_bits(header, size, &pos, 2) : 0;
		if (type > 2)
			return -EINVAL;
		if (code == 0xb3) {
			if (avsm_read_bits(header, size, &pos, 1))
				avsm_read_bits(header, size, &pos, 24);
			/* FFmpeg cavsdec's compatibility with marker-less old clips. */
			peek = pos;
			if (ctx->avsm_low_delay ||
			    !(avsm_read_bits(header, size, &peek, 9) & 1))
				ctx->avsm_modern_header = true;
			peek = pos;
			if (avsm_read_bits(header, size, &peek, 11) & 3)
				ctx->avsm_modern_header = true;
			if (ctx->avsm_plus || ctx->avsm_modern_header)
				avsm_read_bits(header, size, &pos, 1);
		}
		avsm_read_bits(header, size, &pos, 8); /* picture_distance */
		if (ctx->avsm_low_delay) {
			u32 zeros = 0;

			while (!avsm_read_bits(header, size, &pos, 1))
				if (pos == U32_MAX || ++zeros > 31)
					return -EINVAL;
			avsm_read_bits(header, size, &pos, zeros);
		}
		progressive = avsm_read_bits(header, size, &pos, 1);
		if (!progressive)
			structure = avsm_read_bits(header, size, &pos, 1);
		if (!structure && code == 0xb6)
			avsm_read_bits(header, size, &pos, 1);
		top = avsm_read_bits(header, size, &pos, 1);
		if (pos == U32_MAX)
			return -EINVAL;
		*info = type << 3 | (!progressive ? BIT(7) : 0) |
			(top ? BIT(6) : 0);
		return 0;
	}
	return 0; /* Header-only OUTPUT carries no picture token. */
}

static int avsm_queue_input(struct amvdec_session *sess,
			    struct vb2_v4l2_buffer *vbuf)
{
	struct codec_legacy *ctx = sess->priv;
	struct vb2_buffer *vb = &vbuf->vb2_buf;
	u32 bytes = vb2_get_plane_payload(vb, 0), offset, physical, span;
	const u8 *data = vb2_plane_vaddr(vb, 0);
	int ret = 0;

	mutex_lock(&ctx->output_lock);
	if (!ctx->configured || ctx->avsm_input_pending) {
		mutex_unlock(&ctx->output_lock);
		return -EAGAIN;
	}
	if (ctx->failed) {
		ret = -EIO;
		goto error;
	}
	if (!data || !bytes || bytes > sess->vififo_size - SZ_1K - 64) {
		ret = -EINVAL;
		goto error;
	}
	ret = legacy_es_prepare_input(sess, vbuf, &bytes);
	if (ret)
		goto error;
	/* This format does not advertise CONTINUOUS_BYTESTREAM: an OUTPUT
	 * contains at most one coded picture, including its preceding headers. */
	if (sess->input_frames > 1) {
		ret = -EINVAL;
		goto error;
	}
	ret = avsm_picture_info(ctx, data, bytes, &ctx->avsm_frame_info);
	if (ret)
		goto error;
	span = ALIGN(bytes + SZ_1K, 64);
	offset = sess->stream_offset;
	physical = offset % sess->vififo_size;
	if (physical + span > sess->vififo_size) {
		offset += sess->vififo_size - physical;
		physical = 0;
	}
	sess->input_size = bytes;
	ret = amvdec_add_ts(sess, vb->timestamp, vbuf->timecode, offset, vbuf->flags);
	if (ret)
		goto error;
	memcpy(sess->vififo_vaddr + physical, data, bytes);
	memset(sess->vififo_vaddr + physical + bytes, 0, span - bytes);
	dma_wmb();
	sess->stream_offset = offset + span;
	sess->stream_write = (physical + span) % sess->vififo_size;
	ctx->avsm_frame_offset = offset;
	ctx->avsm_frame_bytes = bytes;
	ctx->avsm_frame_pictures = sess->input_frames;
	ctx->avsm_committed = sess->stream_offset;
	ctx->avsm_input_pending = true;
	ctx->avsm_empty = false;
	if (sess->input_frames)
		atomic_inc(&sess->esparser_queued_bufs);
	v4l2_m2m_src_buf_remove_by_buf(sess->m2m_ctx, vbuf);
	vbuf->flags = 0;
	vbuf->field = V4L2_FIELD_NONE;
	vbuf->sequence = sess->sequence_out++;
	v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_DONE);
	avsm_service(sess);
	mutex_unlock(&ctx->output_lock);
	return 0;
error:
	mutex_unlock(&ctx->output_lock);
	v4l2_m2m_src_buf_remove_by_buf(sess->m2m_ctx, vbuf);
	v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
	return ret;
}

static int avsm_drain_step(struct amvdec_session *sess)
{
	struct codec_legacy *ctx = sess->priv;
	int result = 0;

	mutex_lock(&ctx->output_lock);
	avsm_service(sess);
	if (ctx->failed) {
		result = -EIO;
		goto unlock;
	}
	/* At idle, only the single completed, undisplayed I/P named by ANC0
	 * may be the delayed reference tail. EMPTY alone never completes a raster.
	 */
	if (!ctx->avsm_running && ctx->avsm_empty &&
	    ctx->avsm_empty_at == ctx->avsm_committed &&
	    ctx->avsm_committed == READ_ONCE(sess->stream_offset)) {
		u32 i, tail = U32_MAX;

		if (ctx->avsm_display_count)
			goto unlock;
		for (i = 0; i < ctx->buffers; i++) {
			struct avsm_picture *pic = &ctx->avsm_pics[i];

			if (!pic->valid || pic->displayed)
				continue;
			if (tail != U32_MAX || ((pic->info >> 3) & 7) > 1) {
				result = -ENODATA;
				goto unlock;
			}
			tail = i;
		}
		if (tail != U32_MAX) {
			if (atomic_read(&sess->esparser_queued_bufs) != 1 ||
			    (amvdec_read_dos(sess->core, AVSM_REG(0x990)) & 0xffff) !=
			    (ctx->canvases[tail] & 0xffff)) {
				result = -ENODATA;
				goto unlock;
			}
			avsm_queue_display(ctx, tail);
		}
		avsm_publish(sess);
		if (!ctx->failed && !ctx->avsm_display_count &&
		    !atomic_read(&sess->esparser_queued_bufs)) {
			amvdec_eos(sess);
			result = 1;
		} else if (!ctx->failed && !ctx->avsm_display_count) {
			dev_err(sess->core->dev,
				"AVSM EMPTY leaves %d input tokens without completed display\n",
				atomic_read(&sess->esparser_queued_bufs));
			result = -ENODATA;
		}
	}
unlock:
	mutex_unlock(&ctx->output_lock);
	return result;
}

static irqreturn_t legacy_isr(struct amvdec_session *sess)
{
	amvdec_write_dos(sess->core, ASSIST_MBOX1_CLR_REG, 1);
	return IRQ_WAKE_THREAD;
}

static irqreturn_t legacy_process_output(struct amvdec_session *sess)
{
	struct codec_legacy *ctx = sess->priv;
	struct amvdec_core *core = sess->core;
	u32 reg, idx, type, field = V4L2_FIELD_NONE, errors = 0;
	u32 offset, width = sess->width, height = sess->height;
	struct v4l2_fract sar = sess->pixelaspect;
	bool metadata_changed;

	if (!ctx || ctx->failed)
		return IRQ_HANDLED;
	if (ctx->codec == LEGACY_AVS)
		return avsm_process_output(sess);
	if (ctx->codec == LEGACY_VC1 && amvdec_read_dos(core, AV_SCRATCH_7)) {
		legacy_abort(sess);
		return IRQ_HANDLED;
	}
	reg = amvdec_read_dos(core, LEGACY_BUFFEROUT);
	if (!reg)
		return IRQ_HANDLED;
	idx = ((reg & 7) - 1) & 7;
	if (idx >= ctx->buffers) {
		legacy_abort(sess);
		return IRQ_HANDLED;
	}
	type = ctx->codec == LEGACY_MJPEG ? 0 : (reg >> 3) & 7;
	if (ctx->codec == LEGACY_VC1) {
		width = amvdec_read_dos(core, AV_SCRATCH_J);
		height = amvdec_read_dos(core, AV_SCRATCH_K);
	}
	if (ctx->codec != LEGACY_MJPEG) {
		errors = amvdec_read_dos(core, AV_SCRATCH_6);
		sar = legacy_aspect(ctx->codec, width, height,
			amvdec_read_dos(core, AV_SCRATCH_0));
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
	dev_dbg(core->dev,
		"legacy output codec=%u reg=%#x index=%u offset=%#x type=%u repeat=%u errors=%u\n",
		ctx->codec, reg, idx, offset, type,
		amvdec_read_dos(core, AV_SCRATCH_A), errors);
	if (ctx->codec == LEGACY_MJPEG)
		dev_dbg(core->dev,
			"legacy JPEG scaler ctrl=%#x ctrl1=%#x ctrl2=%#x canvas_wr=%#x pic_dc=%#x\n",
			amvdec_read_dos(core, PSCALE_CTRL),
			amvdec_read_dos(core, 0x24a4), /* PSCALE_CTRL1 */
			amvdec_read_dos(core, 0x24b8), /* PSCALE_CTRL2 */
			amvdec_read_dos(core, 0x24b4), /* PSCALE_CANVAS_WR_ADDR */
			amvdec_read_dos(core, MDEC_PIC_DC_CTRL));
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

static irqreturn_t legacy_threaded_isr(struct amvdec_session *sess)
{
	struct codec_legacy *ctx = sess->priv;
	irqreturn_t ret;

	if (!ctx)
		return IRQ_HANDLED;
	mutex_lock(&ctx->output_lock);
	ret = legacy_process_output(sess);
	mutex_unlock(&ctx->output_lock);
	return ret;
}

static const u8 vc1_eos[SZ_1K] = { 0, 0, 1, 0x0a };
static const u8 mjpeg_eos[SZ_1K] = { 0xff, 0xd9 };
static const u8 *vc1_end(u32 *len) { *len = sizeof(vc1_eos); return vc1_eos; }
static const u8 *mjpeg_end(u32 *len) { *len = sizeof(mjpeg_eos); return mjpeg_eos; }

struct amvdec_codec_ops codec_vc1_ops = {
	.start = vc1_start, .stop = legacy_stop,
	.resume = legacy_resume, .prepare_input = vc1_prepare_input,
	.isr = legacy_isr, .threaded_isr = legacy_threaded_isr,
	.can_recycle = legacy_can_recycle, .recycle = legacy_recycle,
	.eos_sequence = vc1_end,
};
struct amvdec_codec_ops codec_avs_ops = {
	.start = avs_start, .stop = legacy_stop,
	.manages_processor = true,
	.resume = legacy_resume,
	.queue_input = avsm_queue_input,
	.isr = legacy_isr, .threaded_isr = legacy_threaded_isr,
	.can_recycle = legacy_can_recycle, .recycle = legacy_recycle,
	.drain_step = avsm_drain_step,
};
struct amvdec_codec_ops codec_mjpeg_ops = {
	.start = mjpeg_start, .stop = legacy_stop,
	.resume = legacy_resume,
	.prepare_input = mjpeg_prepare_input,
	.isr = legacy_isr, .threaded_isr = legacy_threaded_isr,
	.can_recycle = legacy_can_recycle, .recycle = legacy_recycle,
	.eos_sequence = mjpeg_end,
};
