// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018 BayLibre, SAS
 * Author: Maxime Jourdan <mjourdan@baylibre.com>
 */

#include <linux/iopoll.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-event.h>

#include "codec_mpeg12.h"
#include "dos_regs.h"
#include "vdec_helpers.h"
#include "esparser.h"
#include "vdec_1.h"

#define SIZE_WORKSPACE		SZ_128K
/* Offset substracted by the firmware from the workspace paddr */
#define WORKSPACE_OFFSET	(5 * SZ_1K)

/* map firmware registers to known MPEG1/2 functions */
#define MREG_SEQ_INFO		AV_SCRATCH_4
	#define MPEG2_SEQ_DAR_MASK	GENMASK(3, 0)
	#define MPEG2_DAR_4_3		2
	#define MPEG2_DAR_16_9		3
	#define MPEG2_DAR_221_100	4
#define MREG_PIC_INFO		AV_SCRATCH_5
#define MREG_PIC_WIDTH		AV_SCRATCH_6
#define MREG_PIC_HEIGHT		AV_SCRATCH_7
#define MREG_BUFFERIN		AV_SCRATCH_8
#define MREG_BUFFEROUT		AV_SCRATCH_9
#define MREG_CMD		AV_SCRATCH_A
#define MREG_CO_MV_START	AV_SCRATCH_B
#define MREG_ERROR_COUNT	AV_SCRATCH_C
#define MREG_FRAME_OFFSET	AV_SCRATCH_D
#define MREG_WAIT_BUFFER	AV_SCRATCH_E
#define MREG_FATAL_ERROR	AV_SCRATCH_F

#define PICINFO_PROG		0x00008000
#define PICINFO_TOP_FIRST	0x00002000

struct codec_mpeg12 {
	/* Buffer for the MPEG1/2 Workspace */
	void	  *workspace_vaddr;
	dma_addr_t workspace_paddr;
};

static const u8 eos_sequence[SZ_1K] = { 0x00, 0x00, 0x01, 0xB7 };

static const u8 *codec_mpeg12_eos_sequence(u32 *len)
{
	*len = ARRAY_SIZE(eos_sequence);
	return eos_sequence;
}

static int codec_mpeg12_can_recycle(struct amvdec_core *core)
{
	return !amvdec_read_dos(core, MREG_BUFFERIN);
}

static void codec_mpeg12_recycle(struct amvdec_core *core, u32 buf_idx)
{
	amvdec_write_dos(core, MREG_BUFFERIN, buf_idx + 1);
}

static int codec_mpeg12_start(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_mpeg12 *mpeg12;
	int ret;

	mpeg12 = kzalloc(sizeof(*mpeg12), GFP_KERNEL);
	if (!mpeg12)
		return -ENOMEM;

	/* Allocate some memory for the MPEG1/2 decoder's state */
	mpeg12->workspace_vaddr = dma_alloc_coherent(core->dev, SIZE_WORKSPACE,
						     &mpeg12->workspace_paddr,
						     GFP_KERNEL);
	if (!mpeg12->workspace_vaddr) {
		dev_err(core->dev, "Failed to request MPEG 1/2 Workspace\n");
		ret = -ENOMEM;
		goto free_mpeg12;
	}

	ret = amvdec_set_canvases(sess, (u32[]){ AV_SCRATCH_0, 0 },
				(u32[]){ 8, 0 });
	if (ret)
		goto free_workspace;

	amvdec_write_dos(core, POWER_CTL_VLD, BIT(4));
	amvdec_write_dos(core, MREG_CO_MV_START,
			 mpeg12->workspace_paddr + WORKSPACE_OFFSET);

	amvdec_write_dos(core, MPEG1_2_REG, 0);
	amvdec_write_dos(core, PSCALE_CTRL, 0);
	amvdec_write_dos(core, PIC_HEAD_INFO, 0x380);
	amvdec_write_dos(core, M4_CONTROL_REG, 0);
	amvdec_write_dos(core, MREG_BUFFERIN, 0);
	amvdec_write_dos(core, MREG_BUFFEROUT, 0);
	amvdec_write_dos(core, MREG_CMD,
			 (sess->width << 16) | sess->height);
	amvdec_write_dos(core, MREG_ERROR_COUNT, 0);
	amvdec_write_dos(core, MREG_FATAL_ERROR, 0);
	amvdec_write_dos(core, MREG_WAIT_BUFFER, 0);

	sess->keyframe_found = 1;
	sess->priv = mpeg12;

	return 0;

free_workspace:
	dma_free_coherent(core->dev, SIZE_WORKSPACE, mpeg12->workspace_vaddr,
			  mpeg12->workspace_paddr);
free_mpeg12:
	kfree(mpeg12);

	return ret;
}

static void codec_mpeg12_resume(struct amvdec_session *sess)
{
	if (amvdec_set_canvases(sess, (u32[]){ AV_SCRATCH_0, 0 },
				(u32[]){ 8, 0 }))
		amvdec_abort(sess);
}

static int codec_mpeg12_stop(struct amvdec_session *sess)
{
	struct codec_mpeg12 *mpeg12 = sess->priv;
	struct amvdec_core *core = sess->core;

	if (mpeg12->workspace_vaddr)
		dma_free_coherent(core->dev, SIZE_WORKSPACE,
				  mpeg12->workspace_vaddr,
				  mpeg12->workspace_paddr);

	return 0;
}

static void codec_mpeg12_update_dar(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	u32 seq = amvdec_read_dos(core, MREG_SEQ_INFO);
	u32 ar = seq & MPEG2_SEQ_DAR_MASK;

	switch (ar) {
	case MPEG2_DAR_4_3:
		amvdec_set_par_from_dar(sess, 4, 3);
		break;
	case MPEG2_DAR_16_9:
		amvdec_set_par_from_dar(sess, 16, 9);
		break;
	case MPEG2_DAR_221_100:
		amvdec_set_par_from_dar(sess, 221, 100);
		break;
	default:
		sess->pixelaspect.numerator = 1;
		sess->pixelaspect.denominator = 1;
		break;
	}
}

static irqreturn_t codec_mpeg12_threaded_isr(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	u32 reg;
	u32 pic_info;
	u32 is_progressive;
	u32 buffer_index;
	u32 field = V4L2_FIELD_NONE;
	u32 offset;

	amvdec_write_dos(core, ASSIST_MBOX1_CLR_REG, 1);
	reg = amvdec_read_dos(core, MREG_FATAL_ERROR);
	if (reg == 1) {
		dev_err(core->dev, "MPEG1/2 fatal error\n");
		amvdec_abort(sess);
		return IRQ_HANDLED;
	}

	reg = amvdec_read_dos(core, MREG_BUFFEROUT);
	if (!reg)
		return IRQ_HANDLED;
	dev_dbg(core->dev, "MPEG12 IRQ output=%08x pic=%08x offset=%08x\n",
		reg, amvdec_read_dos(core, MREG_PIC_INFO),
		amvdec_read_dos(core, MREG_FRAME_OFFSET));

	/* Unclear what this means */
	if ((reg & GENMASK(23, 17)) == GENMASK(23, 17)) {
		if (sess->should_stop)
			amvdec_eos(sess);
		goto end;
	}

	pic_info = amvdec_read_dos(core, MREG_PIC_INFO);
	is_progressive = pic_info & PICINFO_PROG;

	if (!is_progressive)
		field = (pic_info & PICINFO_TOP_FIRST) ?
			V4L2_FIELD_INTERLACED_TB :
			V4L2_FIELD_INTERLACED_BT;
	sess->field = field;

	codec_mpeg12_update_dar(sess);
	buffer_index = ((reg & 0xf) - 1) & 7;
	offset = amvdec_read_dos(core, MREG_FRAME_OFFSET);
	amvdec_dst_buf_done_idx(sess, buffer_index, offset, field);

end:
	amvdec_write_dos(core, MREG_BUFFEROUT, 0);
	return IRQ_HANDLED;
}

static irqreturn_t codec_mpeg12_isr(struct amvdec_session *sess)
{
	return IRQ_WAKE_THREAD;
}

struct amvdec_codec_ops codec_mpeg12_ops = {
	.start = codec_mpeg12_start,
	.stop = codec_mpeg12_stop,
	.isr = codec_mpeg12_isr,
	.threaded_isr = codec_mpeg12_threaded_isr,
	.can_recycle = codec_mpeg12_can_recycle,
	.recycle = codec_mpeg12_recycle,
	.eos_sequence = codec_mpeg12_eos_sequence,
	.resume = codec_mpeg12_resume,
};

/* SC2 uses the separately authenticated MPEG12 MULTI firmware. Registers here
 * are byte offsets of the vendor vmpeg12_multi/VLD register definitions.
 * Its SCRATCH8 is input mode, NEVER the SINGLE buffer-recycle mailbox.
 */
#define MPEG12_MULTI_WORKSPACE SZ_256K
#define MPEG12_MULTI_CC_SIZE (5 * SZ_1K)
#define MPEG12_MULTI_BUFFERS 8
#define MPEG12_MULTI_SWAP_ADDR 0x3154
#define MPEG12_MULTI_SWAP_CTL 0x3158
#define MPEG12_MULTI_F_CODE 0x3008
#define MPEG12_MULTI_SLICE_TYPE 0x3010
#define MPEG12_MULTI_MB_INFO 0x30b0
#define MPEG12_MULTI_VCOP_CTRL 0x3800
#define MPEG12_MULTI_REC_CANVAS 0x26c8
#define MPEG12_MULTI_ANC2_CANVAS (ANC0_CANVAS_ADDR + 8)
#define MPEG12_MULTI_PIC_DONE 1
#define MPEG12_MULTI_DATA_EMPTY 2
#define MPEG12_MULTI_SEQ_END 3
#define MPEG12_MULTI_DATA_REQUEST 4
#define MPEG12_MULTI_PIC_ERROR BIT(31)
#define MPEG12_MULTI_PIC_TYPE GENMASK(17, 16)
#define MPEG12_MULTI_TYPE_P BIT(16)
#define MPEG12_MULTI_TYPE_B BIT(17)
#define MPEG12_MULTI_SEQ_EXT BIT(31)
#define MPEG12_MULTI_SEQ_PROG BIT(16)

static const u32 mpeg12_context_regs[] = {
	MREG_SEQ_INFO, MREG_PIC_WIDTH, MREG_PIC_HEIGHT, MPEG1_2_REG,
	PIC_HEAD_INFO, MPEG12_MULTI_F_CODE, MPEG12_MULTI_SLICE_TYPE,
	MPEG12_MULTI_MB_INFO, MPEG12_MULTI_VCOP_CTRL, AV_SCRATCH_H,
};

struct mpeg12_multi_picture {
	u64 timestamp;
	struct v4l2_timecode timecode;
	u32 flags;
	u32 field;
	u32 packet_offset;
	bool pending;
	bool displayed;
	bool packet_done;
};

struct mpeg12_multi_preroll {
	struct list_head list;
	u32 offset;
	u32 end;
	u32 type;
};

struct mpeg12_multi {
	struct mutex lock;
	struct list_head preroll;
	u32 input_refs;
	void *workspace;
	dma_addr_t workspace_dma;
	void *cc;
	dma_addr_t cc_dma;
	void *swap;
	dma_addr_t swap_dma;
	u32 context[ARRAY_SIZE(mpeg12_context_regs)];
	u32 canvas[MPEG12_MULTI_BUFFERS];
	struct mpeg12_multi_picture picture[MPEG12_MULTI_BUFFERS];
	int refs[2];
	int active;
	int error;
	u32 committed;
	u32 job_end;
	u32 retained;
	u32 swap_wrap;
	u32 userdata_wp;
	u32 width;
	u32 height;
	u32 decoded;
	u32 pending;
	u32 eos_start;
	bool ready;
	bool running;
	bool context_valid;
	bool swap_valid;
	bool empty;
	bool eos_queued;
	bool terminal;
};

static int mpeg12_multi_run(struct amvdec_session *sess);

static int mpeg12_multi_pause(struct amvdec_session *sess)
{
	struct mpeg12_multi *m = sess->priv;
	struct amvdec_core *core = sess->core;
	u32 value;
	int ret;

	amvdec_write_dos(core, MPSR, 0);
	amvdec_write_dos(core, CPSR, 0);
	sess->processor_started = false;
	m->running = false;
	ret = read_poll_timeout(amvdec_read_dos, value, !(value & BIT(15)),
				10, 100000, false, core, IMEM_DMA_CTRL);
	if (ret)
		return ret;
	ret = read_poll_timeout(amvdec_read_dos, value, !(value & BIT(15)),
				10, 100000, false, core, LMEM_DMA_CTRL);
	if (ret)
		return ret;
	/* amvdec_stop(): reset the stopped processors, preserving IMEM. */
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_write_dos(core, DOS_SW_RESET0, BIT(12) | BIT(11));
	amvdec_write_dos(core, DOS_SW_RESET0, 0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	return 0;
}

static int mpeg12_multi_swap(struct amvdec_session *sess, bool save)
{
	struct mpeg12_multi *m = sess->priv;
	struct amvdec_core *core = sess->core;
	u32 value, rp;
	int ret;

	amvdec_write_dos(core, VLD_MEM_VIFIFO_CONTROL, save ? BIT(15) : 0);
	amvdec_write_dos(core, MPEG12_MULTI_SWAP_ADDR, m->swap_dma);
	amvdec_write_dos(core, MPEG12_MULTI_SWAP_CTL, save ? 3 : 1);
	ret = read_poll_timeout(amvdec_read_dos, value, !(value & BIT(7)),
				10, 100000, false, core, MPEG12_MULTI_SWAP_CTL);
	if (ret)
		return ret;
	amvdec_write_dos(core, MPEG12_MULTI_SWAP_CTL, 0);
	if (save) {
		rp = amvdec_read_dos(core, VLD_MEM_VIFIFO_RP);
		if (rp < sess->vififo_paddr ||
		    rp >= sess->vififo_paddr + sess->vififo_size)
			return -EIO;
		WRITE_ONCE(m->retained, rp - sess->vififo_paddr);
		m->swap_wrap = amvdec_read_dos(core, VLD_MEM_VIFIFO_WRAP_COUNT);
		m->swap_valid = true;
	} else {
		amvdec_write_dos(core, VLD_MEM_VIFIFO_WRAP_COUNT, m->swap_wrap);
	}
	return 0;
}

static bool mpeg12_multi_timestamps_empty(struct amvdec_session *sess)
{
	unsigned long flags;
	bool empty;

	spin_lock_irqsave(&sess->ts_spinlock, flags);
	empty = list_empty(&sess->timestamps);
	spin_unlock_irqrestore(&sess->ts_spinlock, flags);
	return empty;
}

/* Bind the decoded picture now; references are presented later. No nearest
 * offset, most-recent-PTS fallback, or second lookup at final flush is valid.
 */
static int mpeg12_multi_timestamp(struct amvdec_session *sess,
				struct mpeg12_multi_picture *pic, u32 offset)
{
	struct amvdec_timestamp *ts;
	unsigned long flags;
	int ret = -ENOENT;

	spin_lock_irqsave(&sess->ts_spinlock, flags);
	list_for_each_entry(ts, &sess->timestamps, list) {
		if (!ts->remaining ||
		    (u32)(offset - ts->offset) >= (u32)(ts->end_offset - ts->offset))
			continue;
		pic->timestamp = ts->ts;
		pic->timecode = ts->tc;
		pic->flags = ts->flags;
		pic->packet_offset = ts->offset;
		pic->packet_done = !--ts->remaining;
		if (pic->packet_done) {
			list_del(&ts->list);
			kfree(ts);
		}
		ret = 0;
		break;
	}
	spin_unlock_irqrestore(&sess->ts_spinlock, flags);
	return ret;
}

/* The MULTI parser can consume leading pictures without PIC_DONE. Only
 * explicitly classified startup packets, wholly behind a real completed
 * picture's exact packet, may lose their non-display tokens this way.
 */
static int mpeg12_multi_retire_preroll(struct amvdec_session *sess,
				     u32 offset, u32 type, bool drop)
{
	struct mpeg12_multi *m = sess->priv;
	struct mpeg12_multi_preroll *p, *next;
	struct amvdec_timestamp *ts;
	unsigned long flags;
	bool found;
	int ret = drop ? -EPROTO : 0;

	list_for_each_entry_safe(p, next, &m->preroll, list) {
		if (p->offset == offset) {
			if (!drop || p->type != type)
				return -EPROTO;
			list_del(&p->list);
			kfree(p);
			return 0;
		}
		if ((s32)(p->end - offset) > 0)
			break;
		found = false;
		spin_lock_irqsave(&sess->ts_spinlock, flags);
		list_for_each_entry(ts, &sess->timestamps, list) {
			if (ts->offset != p->offset || ts->end_offset != p->end)
				continue;
			if (ts->remaining != 1)
				break;
			list_del(&ts->list);
			kfree(ts);
			atomic_dec(&sess->esparser_queued_bufs);
			found = true;
			break;
		}
		spin_unlock_irqrestore(&sess->ts_spinlock, flags);
		if (!found)
			return -ENOENT;
		list_del(&p->list);
		kfree(p);
	}
	return ret;
}

static int mpeg12_multi_present(struct amvdec_session *sess, int index)
{
	struct mpeg12_multi *m = sess->priv;
	struct mpeg12_multi_picture *pic;
	struct vb2_v4l2_buffer *vb;
	u32 size = amvdec_get_output_size(sess);

	if (index < 0 || index >= MPEG12_MULTI_BUFFERS)
		return -EINVAL;
	pic = &m->picture[index];
	if (!pic->pending || pic->displayed || !m->pending)
		return -EPROTO;
	vb = v4l2_m2m_dst_buf_remove_by_idx(sess->m2m_ctx,
					 sess->fw_idx_to_vb2_idx[index]);
	if (!vb)
		return -ENOBUFS;
	/* Same VB2 metadata contract as dst_buf_done(), with saved picture PTS. */
	vb2_set_plane_payload(&vb->vb2_buf, 0, size);
	vb2_set_plane_payload(&vb->vb2_buf, 1, size / 2);
	vb->vb2_buf.timestamp = pic->timestamp;
	vb->timecode = pic->timecode;
	vb->flags = pic->flags;
	vb->field = pic->field;
	vb->sequence = sess->sequence_cap++;
	pic->pending = false;
	pic->displayed = true;
	m->pending--;
	if (pic->packet_done)
		atomic_dec(&sess->esparser_queued_bufs);
	v4l2_m2m_buf_done(vb, VB2_BUF_STATE_DONE);
	schedule_work(&sess->esparser_queue_work);
	return 0;
}

static int mpeg12_multi_dimensions(struct amvdec_session *sess)
{
	struct mpeg12_multi *m = sess->priv;
	u32 width = amvdec_read_dos(sess->core, MREG_PIC_WIDTH);
	u32 height = amvdec_read_dos(sess->core, MREG_PIC_HEIGHT);
	const struct v4l2_event ev = {
		.type = V4L2_EVENT_SOURCE_CHANGE,
		.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
	};

	if (!width || !height || width > sess->fmt_out->max_width ||
	    height > sess->fmt_out->max_height ||
	    width > sess->capture_width || height > sess->capture_height)
		return -EINVAL;
	m->width = width;
	m->height = height;
	if (width != sess->width || height != sess->height) {
		sess->width = width;
		sess->height = height;
		sess->visible_width = width;
		sess->visible_height = height;
		v4l2_event_queue_fh(&sess->fh, &ev);
	}
	return 0;
}

static int mpeg12_multi_run(struct amvdec_session *sess)
{
	struct mpeg12_multi *m = sess->priv;
	struct amvdec_core *core = sess->core;
	u32 rp, size, fifo = 0;
	unsigned int i;
	int ret, index = -1;

	if (m->error)
		return m->error;
	if (!m->ready || m->running || m->terminal ||
	    (!m->swap_valid && !m->committed) ||
	    (m->empty && m->job_end == m->committed))
		return 0;
	for (i = 0; i < MPEG12_MULTI_BUFFERS; i++) {
		if (i != m->refs[0] && i != m->refs[1] &&
		    !m->picture[i].pending && !m->picture[i].displayed) {
			index = i;
			break;
		}
	}
	if (index < 0)
		return 0;
	/* vmpeg12_multi.run(): retain IMEM, quiescing DMA/DMC before reset. */
	ret = amvdec_vdec1_reset(sess, 0xfffffff0);
	if (ret)
		return ret;
	amvdec_write_dos_bits(core, POWER_CTL_VLD, BIT(4));
	if (m->swap_valid) {
		ret = mpeg12_multi_swap(sess, false);
		if (ret)
			return ret;
		/* Snapshot the restored reservoir BEFORE CONTROL/WP can refill it. */
		fifo = amvdec_read_dos(core, VLD_MEM_VIFIFO_LEVEL);
	} else {
		amvdec_write_dos(core, VLD_MEM_VIFIFO_START_PTR, sess->vififo_paddr);
		amvdec_write_dos(core, VLD_MEM_VIFIFO_END_PTR,
				 sess->vififo_paddr + sess->vififo_size - 8);
		amvdec_write_dos(core, VLD_MEM_VIFIFO_CURR_PTR, sess->vififo_paddr);
		amvdec_write_dos(core, VLD_MEM_VIFIFO_CONTROL, 1);
		amvdec_write_dos(core, VLD_MEM_VIFIFO_CONTROL, 0);
		amvdec_write_dos(core, VLD_MEM_VIFIFO_BUF_CNTL, MEM_BUFCTRL_MANUAL);
		amvdec_write_dos(core, VLD_MEM_VIFIFO_RP, sess->vififo_paddr);
	}
	rp = amvdec_read_dos(core, VLD_MEM_VIFIFO_RP);
	amvdec_write_dos(core, VLD_MEM_VIFIFO_WP,
			 sess->vififo_paddr + sess->stream_write);
	amvdec_write_dos(core, VLD_MEM_VIFIFO_CONTROL,
			 (0x11 << 16) | BIT(10) | (7 << 3));
	/* rp and fifo describe the same pre-refill restored checkpoint. */
	if (rp < sess->vififo_paddr ||
	    rp >= sess->vififo_paddr + sess->vififo_size)
		return -EIO;
	size = (sess->stream_write + sess->vififo_size -
		(rp - sess->vififo_paddr)) % sess->vififo_size;
	size += fifo;
	if (!size)
		return 0;
	/* MULTI workspace base contains LMEM/CC/matrix/MV partitions; unlike
	 * SINGLE, CO_MV_START must NOT have the 5KiB workspace offset added.
	 */
	amvdec_write_dos(core, MREG_CO_MV_START, m->workspace_dma);
	amvdec_write_dos(core, AV_SCRATCH_0, m->cc_dma);
	amvdec_write_dos(core, AV_SCRATCH_2,
			 m->refs[0] < 0 ? U32_MAX : m->canvas[m->refs[0]]);
	amvdec_write_dos(core, AV_SCRATCH_3,
			 m->refs[1] < 0 ? U32_MAX : m->canvas[m->refs[1]]);
	amvdec_write_dos(core, MPEG12_MULTI_REC_CANVAS, m->canvas[index]);
	amvdec_write_dos(core, MPEG12_MULTI_ANC2_CANVAS, m->canvas[index]);
	/* The vendor restores these saved registers even without valid MPEG
	 * context; zero-initialized values prevent another codec's leftovers.
	 */
	for (i = 0; i < ARRAY_SIZE(mpeg12_context_regs); i++)
		amvdec_write_dos(core, mpeg12_context_regs[i], m->context[i]);
	amvdec_write_dos(core, MPEG1_2_REG,
			 m->context_valid ? m->context[3] : 0);
	amvdec_write_dos(core, PIC_HEAD_INFO,
			 m->context_valid ? m->context[4] : 0x380);
	amvdec_write_dos(core, PSCALE_CTRL, 0);
	amvdec_write_dos(core, M4_CONTROL_REG, 0);
	amvdec_write_dos(core, MREG_BUFFEROUT, 0);
	amvdec_write_dos(core, MREG_CMD, (m->width << 16) | m->height);
	amvdec_write_dos(core, MREG_PIC_WIDTH, m->width);
	amvdec_write_dos(core, MREG_PIC_HEIGHT, m->height);
	amvdec_write_dos(core, MREG_ERROR_COUNT, 0);
	/* bit1: parameter notify; bit0: separate CMA userdata buffer. */
	amvdec_write_dos(core, MREG_FATAL_ERROR, 3);
	amvdec_write_dos(core, MREG_WAIT_BUFFER, 0);
	amvdec_write_dos(core, AV_SCRATCH_G, 0);
	amvdec_write_dos(core, AV_SCRATCH_J,
			 m->context_valid ? 0 : m->userdata_wp);
	amvdec_write_dos(core, MREG_BUFFERIN, m->context_valid ? BIT(6) : 0);
	amvdec_set_output_format(sess);
	amvdec_write_dos(core, ASSIST_MBOX1_CLR_REG, 1);
	amvdec_write_dos(core, ASSIST_MBOX1_MASK, 1);
	m->active = index;
	m->job_end = m->committed;
	m->empty = false;
	dma_wmb();
	amvdec_write_dos_bits(core, VLD_MEM_VIFIFO_CONTROL,
			      MEM_CTRL_EMPTY_EN | MEM_CTRL_FILL_EN);
	m->running = true;
	sess->processor_started = true;
	amvdec_write_dos(core, MPSR, 1);
	return 0;
}

static int mpeg12_multi_picture_done(struct amvdec_session *sess, u32 status)
{
	struct mpeg12_multi *m = sess->priv;
	struct amvdec_core *core = sess->core;
	struct mpeg12_multi_picture *pic;
	u32 info = amvdec_read_dos(core, MREG_PIC_INFO);
	u32 seq = amvdec_read_dos(core, MREG_SEQ_INFO);
	u32 offset = amvdec_read_dos(core, MREG_FRAME_OFFSET);
	u32 rec = amvdec_read_dos(core, MPEG12_MULTI_REC_CANVAS);
	u32 type = info & MPEG12_MULTI_PIC_TYPE;
	bool drop = (!m->decoded && type) ||
		    (type == MPEG12_MULTI_TYPE_B && m->refs[0] < 0);
	unsigned int i;
	int index = m->active, display = index, ret;

	/* ERROR_COUNT accumulates parser errors from firmware-skipped prefix
	 * pictures as well. PIC_INFO.ERROR describes THIS completed picture,
	 * as in vmpeg12_multi; a cumulative count cannot invalidate a clean IP.
	 */
	if (index < 0 || index >= MPEG12_MULTI_BUFFERS ||
	    rec != m->canvas[index] || info & MPEG12_MULTI_PIC_ERROR ||
	    type == MPEG12_MULTI_PIC_TYPE)
		return -EIO;
	if ((seq & MPEG12_MULTI_SEQ_EXT) && (seq & MPEG12_MULTI_SEQ_PROG) &&
	    !(info & PICINFO_PROG))
		return -EPROTO;
	ret = mpeg12_multi_dimensions(sess);
	if (ret)
		return ret;
	pic = &m->picture[index];
	ret = mpeg12_multi_timestamp(sess, pic, offset);
	if (ret)
		return ret;
	ret = mpeg12_multi_retire_preroll(sess, pic->packet_offset, type, drop);
	if (ret)
		return ret;
	if (drop) {
		/* Real completion, but no usable random-access references yet. */
		if (pic->packet_done)
			atomic_dec(&sess->esparser_queued_bufs);
		display = -1;
		goto checkpoint;
	}
	pic->field = V4L2_FIELD_NONE;
	/* MPEG-1 has no sequence/picture coding extension: its absent
	 * progressive_frame bit does not describe an interlaced picture.
	 */
	if ((seq & MPEG12_MULTI_SEQ_EXT) &&
	    (!(info & PICINFO_PROG) || !(seq & MPEG12_MULTI_SEQ_PROG)))
		pic->field = info & PICINFO_TOP_FIRST ? V4L2_FIELD_INTERLACED_TB :
						      V4L2_FIELD_INTERLACED_BT;
	sess->field = pic->field;
	pic->flags &= ~(V4L2_BUF_FLAG_KEYFRAME | V4L2_BUF_FLAG_PFRAME |
			V4L2_BUF_FLAG_BFRAME);
	pic->flags |= !type ? V4L2_BUF_FLAG_KEYFRAME :
		      type == MPEG12_MULTI_TYPE_P ? V4L2_BUF_FLAG_PFRAME :
						   V4L2_BUF_FLAG_BFRAME;
	pic->pending = true;
	m->pending++;
	m->decoded++;
	/* update_reference(): the first I is shown immediately, the second
	 * reference is delayed, later references release the preceding one.
	 * Display ownership remains independent of these TWO reference holds.
	 */
	if (type != MPEG12_MULTI_TYPE_B) {
		if (m->refs[1] < 0) {
			m->refs[1] = index;
		} else if (m->refs[0] < 0) {
			m->refs[0] = m->refs[1];
			m->refs[1] = index;
			display = -1;
		} else {
			m->refs[0] = m->refs[1];
			m->refs[1] = index;
			display = m->refs[0];
		}
	}
checkpoint:
	/* SEQ_END invalidates the MPEG context, but still reports its actual
	 * completed picture. It is not a license to emit a stale final surface.
	 */
	m->context_valid = status != MPEG12_MULTI_SEQ_END;
	if (m->context_valid)
		for (i = 0; i < ARRAY_SIZE(mpeg12_context_regs); i++)
			m->context[i] = amvdec_read_dos(core, mpeg12_context_regs[i]);
	ret = mpeg12_multi_swap(sess, true);
	if (ret)
		return ret;
	codec_mpeg12_update_dar(sess);
	if (display >= 0) {
		ret = mpeg12_multi_present(sess, display);
		if (ret)
			return ret;
	}
	return 0;
}

static irqreturn_t mpeg12_multi_threaded_isr(struct amvdec_session *sess)
{
	struct mpeg12_multi *m = sess->priv;
	struct amvdec_core *core = sess->core;
	u32 status, value, rp, remaining;
	int ret = 0;

	mutex_lock(&m->lock);
	amvdec_write_dos(core, ASSIST_MBOX1_CLR_REG, 1);
	if (m->error || !m->running)
		goto unlock;
	if (amvdec_read_dos(core, MREG_FATAL_ERROR) == 1) {
		ret = -EIO;
		goto fail;
	}
	value = amvdec_read_dos(core, AV_SCRATCH_G);
	if (value == 1) {
		ret = mpeg12_multi_dimensions(sess);
		if (ret)
			goto fail;
		amvdec_write_dos(core, AV_SCRATCH_G, 0);
		goto unlock;
	}
	value = amvdec_read_dos(core, AV_SCRATCH_J);
	if (value & BIT(16)) {
		if ((value & 0x7fff) >= MPEG12_MULTI_CC_SIZE) {
			ret = -EOVERFLOW;
			goto fail;
		}
		m->userdata_wp = value & ~BIT(16);
		amvdec_write_dos(core, AV_SCRATCH_J, 0);
		goto unlock;
	}
	status = amvdec_read_dos(core, MREG_BUFFEROUT);
	/* The vendor stream-input path leaves DATA_REQUEST to VLD refill.
	 * Only frame-input mode has a host chunk/VIFF_BIT_CNT continuation.
	 */
	if (!status || status == MPEG12_MULTI_DATA_REQUEST)
		goto unlock;
	if (status != MPEG12_MULTI_PIC_DONE &&
	    status != MPEG12_MULTI_SEQ_END && status != MPEG12_MULTI_DATA_EMPTY) {
		ret = -EPROTO;
		goto fail;
	}
	ret = mpeg12_multi_pause(sess);
	if (ret)
		goto fail;
	if (status == MPEG12_MULTI_DATA_EMPTY) {
		m->empty = true;
		/* Do not checkpoint an incomplete picture. New input replays the
		 * previous completed-picture MPEG/VLD checkpoint, not this state.
		 */
	} else {
		ret = mpeg12_multi_picture_done(sess, status);
		if (ret)
			goto fail;
	}
	/* SEQ_END or empty are firmware events, never elapsed-time EOF.
	 * Both must have reached the real B7 marker in the latest input run.
	 * An incomplete picture retains its unmatched timestamp and cannot end.
	 */
	if ((status == MPEG12_MULTI_SEQ_END || status == MPEG12_MULTI_DATA_EMPTY) &&
	    m->eos_queued && m->job_end == m->committed &&
	    m->committed == m->eos_start + sizeof(eos_sequence) &&
	    mpeg12_multi_timestamps_empty(sess)) {
		rp = amvdec_read_dos(core, VLD_MEM_VIFIFO_RP);
		if (rp < sess->vififo_paddr ||
		    rp >= sess->vififo_paddr + sess->vififo_size) {
			ret = -EIO;
			goto fail;
		}
		remaining = (sess->stream_write + sess->vififo_size -
			     (rp - sess->vififo_paddr)) % sess->vififo_size;
		if (remaining <= sizeof(eos_sequence) - 4)
			m->terminal = true;
	}
	ret = mpeg12_multi_run(sess);
	if (ret)
		goto fail;
	goto unlock;
fail:
	m->error = ret;
	if (m->running)
		mpeg12_multi_pause(sess);
	dev_err(core->dev, "MPEG12 MULTI failed: %d output=%08x offset=%08x\n",
		ret, amvdec_read_dos(core, MREG_BUFFEROUT),
		amvdec_read_dos(core, MREG_FRAME_OFFSET));
unlock:
	mutex_unlock(&m->lock);
	if (ret)
		amvdec_abort(sess);
	schedule_work(&sess->esparser_queue_work);
	return IRQ_HANDLED;
}

/* Track source coded order, not asynchronous decoder progress. Once two IP
 * references have been submitted there are no startup packets left to mark.
 * This only classifies complete packet headers; it never changes payloads or
 * authorizes display, and malformed/ambiguous packets keep normal strict tokens.
 */
static int mpeg12_multi_prepare_input(struct amvdec_session *sess,
				     struct vb2_v4l2_buffer *vbuf,
				     u32 *payload_size)
{
	struct mpeg12_multi *m = sess->priv;
	struct mpeg12_multi_preroll *p;
	const u8 *data = vb2_plane_vaddr(&vbuf->vb2_buf, 0);
	u32 code = U32_MAX, type = 0, pictures = 0, i;
	bool slice = false, header = false;
	int ret = 0;

	if (!data)
		return -EFAULT;

	mutex_lock(&m->lock);
	if (m->input_refs >= 2)
		goto unlock;
	for (i = 0; i < *payload_size; i++) {
		code = (code << 8) | data[i];
		if ((code & 0xffffff00) != 0x100)
			continue;
		if (!(code & 0xff)) {
			pictures++;
			if (*payload_size - i > 2) {
				type = (data[i + 2] >> 3) & 7;
				header = true;
			}
		} else if ((code & 0xff) <= 0xaf && pictures) {
			slice = true;
		}
	}
	if (pictures != 1 || !header || !slice || type < 1 || type > 3)
		goto unlock;
	if (type == 1 || (type == 2 && m->input_refs)) {
		m->input_refs++;
		goto unlock;
	}
	p = kmalloc(sizeof(*p), GFP_KERNEL);
	if (!p) {
		ret = -ENOMEM;
		goto unlock;
	}
	p->offset = sess->stream_offset;
	p->end = p->offset + *payload_size;
	p->type = (type - 1) << 16;
	list_add_tail(&p->list, &m->preroll);
unlock:
	mutex_unlock(&m->lock);
	return ret;
}

static int mpeg12_multi_start(struct amvdec_session *sess)
{
	struct mpeg12_multi *m;
	struct device *dev = sess->core->dev;

	if (sess->core->platform->revision != VDEC_REVISION_SC2 ||
	    (sess->pixfmt_cap != V4L2_PIX_FMT_NV12M &&
	     sess->pixfmt_cap != V4L2_PIX_FMT_NV21M))
		return -EINVAL;
	m = kzalloc(sizeof(*m), GFP_KERNEL);
	if (!m)
		return -ENOMEM;
	mutex_init(&m->lock);
	INIT_LIST_HEAD(&m->preroll);
	m->refs[0] = -1;
	m->refs[1] = -1;
	m->active = -1;
	m->workspace = dma_alloc_coherent(dev, MPEG12_MULTI_WORKSPACE,
					  &m->workspace_dma, GFP_KERNEL);
	if (!m->workspace)
		goto free_context;
	m->cc = dma_alloc_coherent(dev, MPEG12_MULTI_CC_SIZE,
				   &m->cc_dma, GFP_KERNEL);
	if (!m->cc)
		goto free_workspace;
	m->swap = dma_alloc_coherent(dev, PAGE_SIZE, &m->swap_dma, GFP_KERNEL);
	if (!m->swap)
		goto free_cc;
	sess->priv = m;
	sess->keyframe_found = 1;
	amvdec_src_change(sess, sess->width, sess->height, MPEG12_MULTI_BUFFERS);
	return 0;
free_cc:
	dma_free_coherent(dev, MPEG12_MULTI_CC_SIZE, m->cc, m->cc_dma);
free_workspace:
	dma_free_coherent(dev, MPEG12_MULTI_WORKSPACE, m->workspace,
			  m->workspace_dma);
free_context:
	kfree(m);
	return -ENOMEM;
}

static int mpeg12_multi_stop(struct amvdec_session *sess)
{
	struct mpeg12_multi *m = sess->priv;
	struct mpeg12_multi_preroll *p, *next;
	struct device *dev = sess->core->dev;

	/* vdec_1_sc2_stop has disabled IRQs and proved processor/DMC DMA idle. */
	list_for_each_entry_safe(p, next, &m->preroll, list) {
		list_del(&p->list);
		kfree(p);
	}
	dma_free_coherent(dev, PAGE_SIZE, m->swap, m->swap_dma);
	dma_free_coherent(dev, MPEG12_MULTI_CC_SIZE, m->cc, m->cc_dma);
	dma_free_coherent(dev, MPEG12_MULTI_WORKSPACE, m->workspace,
			  m->workspace_dma);
	return 0;
}

static void mpeg12_multi_resume(struct amvdec_session *sess)
{
	struct mpeg12_multi *m = sess->priv;
	unsigned int i;
	int ret = 0;

	mutex_lock(&m->lock);
	if (!m->ready) {
		if (sess->num_dst_bufs != MPEG12_MULTI_BUFFERS ||
		    (sess->pixfmt_cap != V4L2_PIX_FMT_NV12M &&
		     sess->pixfmt_cap != V4L2_PIX_FMT_NV21M)) {
			ret = -EINVAL;
			goto unlock;
		}
		ret = amvdec_set_canvases(sess,
			(u32[]){ ANC0_CANVAS_ADDR, 0 },
			(u32[]){ MPEG12_MULTI_BUFFERS, 0 });
		if (ret)
			goto unlock;
		/* Save immutable owned canvas identities BEFORE ANC2 becomes REC. */
		for (i = 0; i < MPEG12_MULTI_BUFFERS; i++)
			m->canvas[i] = amvdec_read_dos(sess->core,
						     ANC0_CANVAS_ADDR + 4 * i);
		m->ready = true;
	}
	ret = mpeg12_multi_run(sess);
unlock:
	if (ret)
		m->error = ret;
	mutex_unlock(&m->lock);
	if (ret)
		amvdec_abort(sess);
}

static void mpeg12_multi_input_committed(struct amvdec_session *sess,
					u32 offset, u32 bytes)
{
	struct mpeg12_multi *m = sess->priv;
	int ret;

	mutex_lock(&m->lock);
	m->committed = offset + bytes;
	ret = mpeg12_multi_run(sess);
	if (ret)
		m->error = ret;
	mutex_unlock(&m->lock);
	if (ret)
		amvdec_abort(sess);
}

static u32 mpeg12_multi_input_read_pointer(struct amvdec_session *sess)
{
	struct mpeg12_multi *m = sess->priv;

	/* Keep bytes needed by the last completed-picture replay checkpoint. */
	return sess->vififo_paddr + READ_ONCE(m->retained);
}

static int mpeg12_multi_can_recycle(struct amvdec_core *core)
{
	return 1;
}

static void mpeg12_multi_recycle(struct amvdec_core *core, u32 index)
{
	struct amvdec_session *sess = core->cur_sess;
	struct mpeg12_multi *m = sess->priv;
	int ret;

	mutex_lock(&m->lock);
	if (index >= MPEG12_MULTI_BUFFERS) {
		ret = -EINVAL;
	} else {
		m->picture[index].displayed = false;
		ret = mpeg12_multi_run(sess);
	}
	if (ret)
		m->error = ret;
	mutex_unlock(&m->lock);
	if (ret)
		amvdec_abort(sess);
}

static int mpeg12_multi_drain_step(struct amvdec_session *sess)
{
	struct mpeg12_multi *m = sess->priv;
	int ret = 0, index;

	mutex_lock(&m->lock);
	if (m->error) {
		ret = m->error;
		goto unlock;
	}
	if (!m->eos_queued) {
		m->eos_start = sess->stream_offset;
		ret = esparser_queue_eos(sess->core, eos_sequence,
					 sizeof(eos_sequence));
		if (ret == -EAGAIN) {
			ret = 0;
			goto unlock;
		}
		if (ret < 0)
			goto unlock;
		m->committed = sess->stream_offset;
		m->eos_queued = true;
	}
	ret = mpeg12_multi_run(sess);
	if (ret || !m->terminal || m->running)
		goto unlock;
	if (!mpeg12_multi_timestamps_empty(sess)) {
		ret = -EPROTO;
		goto unlock;
	}
	/* flush_output(): refs[1] is the delayed final IP, not an ANC index.
	 * The initial single I was already presented and must not be repeated.
	 */
	index = m->refs[1];
	if (index >= 0 && m->picture[index].pending) {
		ret = mpeg12_multi_present(sess, index);
		if (ret)
			goto unlock;
	}
	m->refs[0] = -1;
	m->refs[1] = -1;
	if (m->pending || atomic_read(&sess->esparser_queued_bufs)) {
		ret = -EPROTO;
		goto unlock;
	}
	amvdec_eos(sess);
	ret = 1;
unlock:
	if (ret < 0)
		m->error = ret;
	mutex_unlock(&m->lock);
	return ret;
}

struct amvdec_codec_ops codec_mpeg12_multi_ops = {
	.manages_processor = true,
	.start = mpeg12_multi_start,
	.stop = mpeg12_multi_stop,
	.isr = codec_mpeg12_isr,
	.threaded_isr = mpeg12_multi_threaded_isr,
	.can_recycle = mpeg12_multi_can_recycle,
	.recycle = mpeg12_multi_recycle,
	.resume = mpeg12_multi_resume,
	.prepare_input = mpeg12_multi_prepare_input,
	.input_committed = mpeg12_multi_input_committed,
	.input_read_pointer = mpeg12_multi_input_read_pointer,
	.drain_step = mpeg12_multi_drain_step,
};
