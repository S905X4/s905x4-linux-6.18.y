// SPDX-License-Identifier: GPL-2.0-or-later
/* SC2 factory mmpeg4_mc_5 MULTI protocol, adapted from Amlogic's
 * vmpeg4_multi.c (Copyright 2017 Amlogic, Inc.). Not the SINGLE mailbox ABI.
 */
#include <linux/iopoll.h>
#include <linux/firmware/meson/meson_sm.h>
#include <linux/jhash.h>
#include <linux/math64.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <media/v4l2-event.h>
#include <media/v4l2-mem2mem.h>

#include "codec_mpeg4.h"
#include "dos_regs.h"
#include "vdec_helpers.h"
#include "vdec_1.h"
#include "esparser.h"

#define MPEG4_BUFFERS 8
#define MPEG4_WORKSPACE (12 * SZ_64K)
#define MPEG4_DMA_SIZE (MPEG4_WORKSPACE + PAGE_SIZE)
/* Factory CTX_LMEM_SWAP_OFFSET=0, CTX_QUANT_MATRIX_OFFSET=0x800;
 * DCAC scratch begins at 0x1000. Fingerprints below inspect owned memory only.
 */
#define MPEG4_LMEM_SIZE SZ_2K
#define MPEG4_REF0 AV_SCRATCH_1
#define MPEG4_REF1 AV_SCRATCH_2
#define MPEG4_RATE AV_SCRATCH_3
#define MPEG4_RATIO AV_SCRATCH_5
#define MPEG4_ERRORS AV_SCRATCH_6
#define MPEG4_WH AV_SCRATCH_7
#define MPEG4_INPUT AV_SCRATCH_8
#define MPEG4_OUTPUT AV_SCRATCH_9
#define MPEG4_REPEAT AV_SCRATCH_A
#define MPEG4_OFFSET AV_SCRATCH_C
#define MPEG4_SYS_RATE AV_SCRATCH_E
#define MPEG4_WORK AV_SCRATCH_F
#define MPEG4_INFO AV_SCRATCH_H
#define MPEG4_DEBUG 0x2758
#define MPEG4_REC 0x26c8
#define MPEG4_ANC2 (ANC0_CANVAS_ADDR + 8)
#define MPEG4_SWAP_ADDR 0x3154
#define MPEG4_SWAP_CTL 0x3158
#define MPEG4_VCOP 0x3800
#define MPEG4_SLICE_QP 0x3028
#define MPEG4_MB_INFO 0x30b0
#define MPEG4_DC_AC 0x3814
#define MPEG4_IQIDCT 0x3838
#define MPEG4_RESYNC 0x3038
#define MPEG4_RV_COUNT 0x3830
#define MPEG4_INTERLACE BIT(7)
#define MPEG4_TOP_FIRST BIT(6)

static const u8 mpeg4_eos[SZ_1K] = { 0, 0, 1, 0xb1 };

/* vmpeg4_save_hw_context()/vmpeg4_hw_ctx_restore(). LMEM and quantization
 * matrices are saved by the firmware inside the persistent workspace.
 */
static const u32 mpeg4_context_regs[] = {
	MPEG1_2_REG, MPEG4_VCOP, PIC_HEAD_INFO, MPEG4_SLICE_QP,
	MPEG4_WH, MPEG4_MB_INFO, MPEG4_DC_AC,
	MPEG4_IQIDCT, MPEG4_RESYNC, MPEG4_RV_COUNT,
};

struct mpeg4_picture {
	u64 timestamp;
	struct v4l2_timecode timecode;
	struct v4l2_fract aspect;
	u32 flags;
	u32 field;
	u32 width;
	u32 height;
	u32 offset;
	bool packet_done;
	bool pending;
	bool consumer;
};

struct codec_mpeg4 {
	struct mutex lock;
	void *workspace;
	dma_addr_t workspace_dma;
	u32 context[ARRAY_SIZE(mpeg4_context_regs)];
	u32 canvas[MPEG4_BUFFERS];
	struct mpeg4_picture picture[MPEG4_BUFFERS];
	int refs[2];
	int active;
	int display;
	int error;
	u32 committed;
	u32 job_end;
	u32 retained;
	u32 swap_wrap;
	u32 pending;
	u32 decoded;
	u8 time_bits;
	bool ready;
	bool running;
	bool context_valid;
	bool swap_valid;
	bool empty;
	bool terminal;
	bool eos_queued;
};

static int mpeg4_run(struct amvdec_session *sess);

/* Both amvdec_stop() and amvdec_start() pulse the stopped CPUs. The
 * start-side pulse is after VLD/MPEG context restore, not a core reset.
 */
static void mpeg4_reset_cpus(struct amvdec_core *core)
{
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_write_dos(core, DOS_SW_RESET0, BIT(12) | BIT(11));
	amvdec_write_dos(core, DOS_SW_RESET0, 0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);
}

static int mpeg4_pause(struct amvdec_session *sess)
{
	struct codec_mpeg4 *m = sess->priv;
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
	/* amvdec_stop(): stopped-processor reset, retaining authenticated IMEM. */
	mpeg4_reset_cpus(core);
	return 0;
}

static int mpeg4_swap(struct amvdec_session *sess, bool save)
{
	struct codec_mpeg4 *m = sess->priv;
	struct amvdec_core *core = sess->core;
	u32 value, rp;
	int ret;

	amvdec_write_dos(core, VLD_MEM_VIFIFO_CONTROL, save ? BIT(15) : 0);
	amvdec_write_dos(core, MPEG4_SWAP_ADDR,
			 m->workspace_dma + MPEG4_WORKSPACE);
	amvdec_write_dos(core, MPEG4_SWAP_CTL, save ? 3 : 1);
	ret = read_poll_timeout(amvdec_read_dos, value, !(value & BIT(7)),
				10, 100000, false, core, MPEG4_SWAP_CTL);
	if (ret)
		return ret;
	amvdec_write_dos(core, MPEG4_SWAP_CTL, 0);
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

static bool mpeg4_timestamps_empty(struct amvdec_session *sess)
{
	unsigned long flags;
	bool empty;

	spin_lock_irqsave(&sess->ts_spinlock, flags);
	empty = list_empty(&sess->timestamps);
	spin_unlock_irqrestore(&sess->ts_spinlock, flags);
	return empty;
}

/* vdec_has_more_input(): an EMPTY is final only if the hardware was
 * given the writer's latest WP. Unmatched coded tokens still forbid EOF.
 */
static bool mpeg4_terminal_empty(struct amvdec_session *sess)
{
	struct codec_mpeg4 *m = sess->priv;

	return m->eos_queued && sess->draining && !m->running &&
		m->job_end == m->committed &&
		m->committed == READ_ONCE(sess->stream_offset) &&
		amvdec_read_dos(sess->core, VLD_MEM_VIFIFO_WP) ==
			sess->vififo_paddr + READ_ONCE(sess->stream_write) &&
		mpeg4_timestamps_empty(sess);
}

/* One true completion consumes the next committed coded-VOP token before
 * display/reference reorder. Only I/P offsets have vendor source authority;
 * cross-check those against that exact token, never search another token.
 */
static int mpeg4_timestamp(struct amvdec_session *sess,
			   struct mpeg4_picture *pic, u32 offset, bool check_offset)
{
	struct amvdec_timestamp *ts;
	unsigned long flags;
	u32 start = 0, end = 0, remaining = 0;
	int ret = -ENOENT;

	spin_lock_irqsave(&sess->ts_spinlock, flags);
	if (!list_empty(&sess->timestamps)) {
		ts = list_first_entry(&sess->timestamps, struct amvdec_timestamp, list);
		start = ts->offset;
		end = ts->end_offset;
		remaining = ts->remaining;
		if (!ts->remaining || (check_offset &&
		    (u32)(offset - ts->offset) >= (u32)(ts->end_offset - ts->offset))) {
			ret = -EPROTO;
			goto unlock;
		}
		pic->timestamp = ts->ts;
		pic->timecode = ts->tc;
		pic->flags = ts->flags;
		pic->packet_done = !--ts->remaining;
		if (pic->packet_done) {
			list_del(&ts->list);
			kfree(ts);
		}
		ret = 0;
	}
unlock:
	spin_unlock_irqrestore(&sess->ts_spinlock, flags);
	dev_dbg(sess->core->dev,
		"MPEG4 token ret=%d hw=%08x check=%u head=%08x..%08x remaining=%u\n",
		ret, offset, check_offset, start, end, remaining);
	return ret;
}

static bool mpeg4_old_frames_drained(struct amvdec_session *sess)
{
	struct vb2_queue *q = v4l2_m2m_get_dst_vq(sess->m2m_ctx);
	unsigned long flags;
	bool empty;

	spin_lock_irqsave(&q->done_lock, flags);
	empty = list_empty(&q->done_list);
	spin_unlock_irqrestore(&q->done_lock, flags);
	return empty;
}

/* A held reference's metadata belongs to that picture, not the newer picture
 * whose completion makes it displayable. Wait for the old-format done queue.
 */
static int mpeg4_present(struct amvdec_session *sess, int index)
{
	struct codec_mpeg4 *m = sess->priv;
	struct mpeg4_picture *pic = &m->picture[index];
	struct vb2_v4l2_buffer *vb;
	u32 size = amvdec_get_output_size(sess);
	bool change;

	if (!pic->pending || pic->consumer || !m->pending)
		return -EPROTO;
	change = pic->width != sess->width || pic->height != sess->height ||
		pic->field != sess->field ||
		pic->aspect.numerator != sess->pixelaspect.numerator ||
		pic->aspect.denominator != sess->pixelaspect.denominator;
	if (change && !mpeg4_old_frames_drained(sess))
		return -EAGAIN;
	vb = v4l2_m2m_dst_buf_remove_by_idx(sess->m2m_ctx,
					 sess->fw_idx_to_vb2_idx[index]);
	if (!vb)
		return -ENOBUFS;
	if (change) {
		const struct v4l2_event ev = {
			.type = V4L2_EVENT_SOURCE_CHANGE,
			.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
		};

		sess->width = pic->width;
		sess->height = pic->height;
		sess->visible_width = pic->width;
		sess->visible_height = pic->height;
		sess->field = pic->field;
		sess->pixelaspect = pic->aspect;
		v4l2_event_queue_fh(&sess->fh, &ev);
	}
	vb2_set_plane_payload(&vb->vb2_buf, 0, size);
	vb2_set_plane_payload(&vb->vb2_buf, 1, size / 2);
	vb->vb2_buf.timestamp = pic->timestamp;
	vb->timecode = pic->timecode;
	vb->flags = pic->flags;
	vb->field = pic->field;
	vb->sequence = sess->sequence_cap++;
	pic->pending = false;
	pic->consumer = true;
	m->pending--;
	if (pic->packet_done)
		atomic_dec(&sess->esparser_queued_bufs);
	v4l2_m2m_buf_done(vb, VB2_BUF_STATE_DONE);
	schedule_work(&sess->esparser_queue_work);
	return 0;
}

static int mpeg4_run(struct amvdec_session *sess)
{
	struct codec_mpeg4 *m = sess->priv;
	struct amvdec_core *core = sess->core;
	u64 sys_rate = 0;
	u32 rp, size, fifo = 0;
	unsigned int i;
	int ret, index = -1;

	if (m->error)
		return m->error;
	if (m->display >= 0) {
		ret = mpeg4_present(sess, m->display);
		if (ret == -EAGAIN) {
			dev_dbg(core->dev, "MPEG4 wait old-format display=%d pending=%u\n",
				m->display, m->pending);
			return 0;
		}
		if (ret)
			return ret;
		m->display = -1;
	}
	if (!m->ready || m->running || m->terminal || !m->committed ||
	    (m->empty && m->job_end == m->committed) ||
	    (m->eos_queued && mpeg4_timestamps_empty(sess))) {
		dev_dbg(core->dev,
			"MPEG4 defer ready=%u running=%u terminal=%u empty=%u committed=%08x job=%08x retained=%08x decoded=%u pending=%u\n",
			m->ready, m->running, m->terminal, m->empty,
			m->committed, m->job_end, m->retained, m->decoded, m->pending);
		return 0;
	}
	for (i = 0; i < MPEG4_BUFFERS; i++) {
		if (i != m->refs[0] && i != m->refs[1] &&
		    !m->picture[i].pending && !m->picture[i].consumer) {
			index = i;
			break;
		}
	}
	if (index < 0) {
		dev_dbg(core->dev, "MPEG4 wait surface refs=%d/%d display=%d pending=%u\n",
			m->refs[0], m->refs[1], m->display, m->pending);
		for (i = 0; i < MPEG4_BUFFERS; i++)
			dev_dbg(core->dev, "MPEG4 surface=%u pending=%u consumer=%u pts=%llu\n",
				i, m->picture[i].pending, m->picture[i].consumer,
				m->picture[i].timestamp);
		return 0;
	}
	/* MP4_SYS_RATE is the negotiated frame duration in 96 kHz ticks,
	 * shared by vmpeg4_hw_ctx_restore()'s stream and frame input modes.
	 * Round to the nearest tick as Kodi does when constructing sysinfo.rate;
	 * leave it unspecified only when no frame interval was negotiated.
	 */
	if (sess->timeperframe.denominator) {
		sys_rate = DIV_ROUND_CLOSEST_ULL(96000ULL * sess->timeperframe.numerator,
					       sess->timeperframe.denominator);
		if (sys_rate > U32_MAX)
			return -ERANGE;
	}
	/* vdec_reset_core(): reset VLD/FIFO/MC/DBLK/PIC_DC only. In
	 * particular, IQIDCT and its quantization state must survive a run;
	 * the broad startup reset is not a per-picture context switch.
	 */
	ret = amvdec_vdec1_reset(sess, 0);
	if (ret)
		return ret;
	amvdec_write_dos_bits(core, POWER_CTL_VLD, BIT(4));
	if (m->swap_valid) {
		ret = mpeg4_swap(sess, false);
		if (ret)
			return ret;
		/* Take the restored-reservoir snapshot before publishing WP. */
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
	if (rp < sess->vififo_paddr ||
	    rp >= sess->vififo_paddr + sess->vififo_size)
		return -EIO;
	size = (sess->stream_write + sess->vififo_size -
		(rp - sess->vififo_paddr)) % sess->vififo_size + fifo;
	dev_dbg(core->dev,
		"MPEG4 run active=%d refs=%d/%d context=%u swap=%u rp=%08x fifo=%08x size=%08x write=%08x committed=%08x retained=%08x\n",
		index, m->refs[0], m->refs[1], m->context_valid, m->swap_valid,
		rp, fifo, size, sess->stream_write, m->committed, m->retained);
	if (!size)
		return 0;
	amvdec_write_dos(core, VLD_MEM_VIFIFO_WP,
			 sess->vififo_paddr + sess->stream_write);
	amvdec_write_dos(core, VLD_MEM_VIFIFO_CONTROL,
			 (0x11 << 16) | BIT(10) | (7 << 3));
	amvdec_write_dos(core, MPEG4_REF0, m->refs[0] < 0 ?
		(m->refs[1] < 0 ? U32_MAX : m->canvas[m->refs[1]]) :
		m->canvas[m->refs[0]]);
	amvdec_write_dos(core, MPEG4_REF1,
			 m->refs[1] < 0 ? U32_MAX : m->canvas[m->refs[1]]);
	amvdec_write_dos(core, MPEG4_REC, m->canvas[index]);
	amvdec_write_dos(core, MPEG4_ANC2, m->canvas[index]);
	amvdec_write_dos(core, MPEG4_WORK, m->workspace_dma);
	amvdec_write_dos(core, PSCALE_CTRL, 0);
	amvdec_write_dos(core, MPEG4_OUTPUT, 0x10000);
	amvdec_write_dos(core, MPEG4_REPEAT, 0);
	amvdec_write_dos(core, MPEG4_INFO, 0);
	amvdec_write_dos(core, MPEG4_DEBUG, 0);
	amvdec_write_dos(core, MPEG4_ERRORS, 0);
	amvdec_write_dos_bits(core, MDEC_PIC_DC_CTRL, BIT(17));
	amvdec_clear_dos_bits(core, MDEC_PIC_DC_CTRL, BIT(16));
	amvdec_write_dos(core, MDEC_PIC_DC_THRESH, 0x404038aa);
	/* Match vmpeg4_hw_ctx_restore() ordering. Do not initialize IQIDCT
	 * from an invalid context: its initial state belongs to core startup.
	 */
	amvdec_write_dos(core, MPEG4_WH, m->context_valid ? m->context[4] :
			 (sess->width << 16) | sess->height);
	amvdec_write_dos(core, MPEG4_SYS_RATE, sys_rate);
	if (m->context_valid) {
		for (i = 6; i < ARRAY_SIZE(mpeg4_context_regs); i++)
			amvdec_write_dos(core, mpeg4_context_regs[i], m->context[i]);
	}
	amvdec_write_dos(core, MPEG1_2_REG,
			 m->context_valid ? m->context[0] : 1);
	for (i = 1; i < 4; i++)
		amvdec_write_dos(core, mpeg4_context_regs[i], m->context[i]);
	amvdec_write_dos(core, MPEG4_MB_INFO, m->context[5]);
	/* Stream input, no fabricated chunk offset/bit count. */
	amvdec_write_dos(core, MPEG4_INPUT, m->context_valid ? BIT(6) : 0);
	amvdec_write_dos(core, ASSIST_MBOX1_CLR_REG, 1);
	amvdec_write_dos(core, ASSIST_MBOX1_MASK, 1);
	m->active = index;
	m->job_end = m->committed;
	m->empty = false;
	dma_wmb();
	amvdec_write_dos_bits(core, VLD_MEM_VIFIFO_CONTROL,
			      MEM_CTRL_EMPTY_EN | MEM_CTRL_FILL_EN);
	mpeg4_reset_cpus(core);
	dev_dbg(core->dev, "MPEG4 memory start lmem=%08x quant=%08x interval=%u/%u\n",
		jhash(m->workspace, MPEG4_LMEM_SIZE, 0),
		jhash(m->workspace + MPEG4_LMEM_SIZE, MPEG4_LMEM_SIZE, 0),
		sess->timeperframe.numerator, sess->timeperframe.denominator);
	m->running = true;
	sess->processor_started = true;
	amvdec_write_dos(core, MPSR, 1);
	return 0;
}

static int mpeg4_picture_done(struct amvdec_session *sess, u32 info)
{
	static const struct v4l2_fract sar[] = {
		{ 1, 1 }, { 1, 1 }, { 12, 11 }, { 10, 11 }, { 16, 11 }, { 40, 33 },
	};
	struct codec_mpeg4 *m = sess->priv;
	struct amvdec_core *core = sess->core;
	struct mpeg4_picture *pic;
	u32 type = (info >> 3) & 7;
	u32 wh = amvdec_read_dos(core, MPEG4_WH);
	u32 ratio = amvdec_read_dos(core, MPEG4_RATIO) & 0xffff;
	int index = m->active, display = index, ret;
	unsigned int i;

	if (index < 0 || index >= MPEG4_BUFFERS || type > 2 ||
	    amvdec_read_dos(core, MPEG4_REC) != m->canvas[index] ||
	    amvdec_read_dos(core, MPEG4_ERRORS))
		return -EPROTO;
	if ((!m->decoded && type) || (type == 2 && m->refs[0] < 0))
		return -EPROTO;
	pic = &m->picture[index];
	pic->width = wh >> 16;
	pic->height = wh & 0xffff;
	if (!pic->width || !pic->height ||
	    pic->width > sess->fmt_out->max_width ||
	    pic->height > sess->fmt_out->max_height ||
	    pic->width > sess->capture_width || pic->height > sess->capture_height)
		return -EINVAL;
	pic->aspect = (struct v4l2_fract) { 1, 1 };
	if (ratio < ARRAY_SIZE(sar))
		pic->aspect = sar[ratio];
	else if (ratio > 0xf && (ratio >> 8) && (ratio & 0xff))
		pic->aspect = (struct v4l2_fract) { ratio >> 8, ratio & 0xff };
	pic->field = !(info & MPEG4_INTERLACE) ? V4L2_FIELD_NONE :
		info & MPEG4_TOP_FIRST ? V4L2_FIELD_INTERLACED_TB :
					 V4L2_FIELD_INTERLACED_BT;
	pic->offset = amvdec_read_dos(core, MPEG4_OFFSET);
	ret = mpeg4_timestamp(sess, pic, pic->offset, type != 2);
	if (ret)
		return ret;
	pic->flags &= ~(V4L2_BUF_FLAG_KEYFRAME | V4L2_BUF_FLAG_PFRAME |
			V4L2_BUF_FLAG_BFRAME);
	pic->flags |= !type ? V4L2_BUF_FLAG_KEYFRAME :
		      type == 1 ? V4L2_BUF_FLAG_PFRAME : V4L2_BUF_FLAG_BFRAME;
	pic->pending = true;
	m->pending++;
	m->decoded++;
	/* update_ref(): unlike MPEG12, MPEG4 delays even the first reference. */
	if (type != 2) {
		if (m->refs[1] < 0) {
			m->refs[1] = index;
			display = -1;
		} else if (m->refs[0] < 0) {
			m->refs[0] = m->refs[1];
			m->refs[1] = index;
			display = m->refs[0];
		} else {
			m->refs[0] = m->refs[1];
			m->refs[1] = index;
			display = m->refs[0];
		}
	}
	for (i = 0; i < ARRAY_SIZE(mpeg4_context_regs); i++)
		m->context[i] = amvdec_read_dos(core, mpeg4_context_regs[i]);
	m->context_valid = true;
	if (display >= 0)
		m->display = display;
	return 0;
}

static irqreturn_t mpeg4_threaded_isr(struct amvdec_session *sess)
{
	struct codec_mpeg4 *m = sess->priv;
	struct amvdec_core *core = sess->core;
	u32 status;
	int ret = 0;

	mutex_lock(&m->lock);
	amvdec_write_dos(core, ASSIST_MBOX1_CLR_REG, 1);
	if (m->error || !m->running)
		goto unlock;
	if (amvdec_read_dos(core, MPEG4_INFO) == 1) {
		amvdec_write_dos(core, MPEG4_INFO, 0);
		goto unlock;
	}
	if (amvdec_read_dos(core, MPEG4_DEBUG)) {
		amvdec_write_dos(core, MPEG4_DEBUG, 0);
		goto unlock;
	}
	status = amvdec_read_dos(core, MPEG4_OUTPUT);
	dev_dbg(core->dev,
		"MPEG4 irq output=%08x active=%d refs=%d/%d display=%d decoded=%u pending=%u committed=%08x job=%08x retained=%08x context=%u swap=%u eos=%u\n",
		status, m->active, m->refs[0], m->refs[1], m->display,
		m->decoded, m->pending, m->committed, m->job_end, m->retained,
		m->context_valid, m->swap_valid, m->eos_queued);
	if (status == 0x10000)
		goto unlock;
	/* vmpeg4_save_hw_context() runs in the completion IRQ, before
	 * vmpeg4_work() stops/resets the CPUs. EMPTY is not a completion and
	 * must retain the previous picture's register and input checkpoint.
	 */
	if (status != 2) {
		ret = mpeg4_picture_done(sess, status);
		if (ret)
			goto fail;
	}
	ret = mpeg4_pause(sess);
	if (ret)
		goto fail;
	dev_dbg(core->dev, "MPEG4 memory stopped output=%08x lmem=%08x quant=%08x\n",
		status, jhash(m->workspace, MPEG4_LMEM_SIZE, 0),
		jhash(m->workspace + MPEG4_LMEM_SIZE, MPEG4_LMEM_SIZE, 0));
	if (status == 2) {
		/* Stream AGAIN: replay the previous completed-picture checkpoint
		 * only after genuinely new input. Do not save partial picture state.
		 */
		m->empty = true;
		if (mpeg4_terminal_empty(sess))
			m->terminal = true;
	} else {
		/* The generic vendor input checkpoint is saved after amvdec_stop(),
		 * separately from the pre-stop MPEG picture/register snapshot.
		 */
		ret = mpeg4_swap(sess, true);
		if (ret)
			goto fail;
		if (mpeg4_terminal_empty(sess)) {
			m->empty = true;
			m->terminal = true;
		}
	}
	ret = mpeg4_run(sess);
	if (ret)
		goto fail;
	goto unlock;
fail:
	m->error = ret;
	if (m->running)
		mpeg4_pause(sess);
	dev_err(core->dev, "MPEG4 MULTI failed: %d output=%08x offset=%08x\n",
		ret, amvdec_read_dos(core, MPEG4_OUTPUT),
		amvdec_read_dos(core, MPEG4_OFFSET));
unlock:
	mutex_unlock(&m->lock);
	if (ret)
		amvdec_abort(sess);
	schedule_work(&sess->esparser_queue_work);
	return IRQ_HANDLED;
}

static irqreturn_t mpeg4_isr(struct amvdec_session *sess)
{
	return IRQ_WAKE_THREAD;
}

static int mpeg4_start(struct amvdec_session *sess)
{
	struct codec_mpeg4 *m;
	int ret = -ENOMEM;

	if (sess->core->platform->revision != VDEC_REVISION_SC2 ||
	    sess->pixfmt_cap != V4L2_PIX_FMT_NV12M)
		return -EINVAL;
	m = kzalloc(sizeof(*m), GFP_KERNEL);
	if (!m)
		return -ENOMEM;
	mutex_init(&m->lock);
	m->refs[0] = -1;
	m->refs[1] = -1;
	m->active = -1;
	m->display = -1;
	m->workspace = dma_alloc_coherent(sess->core->dev, MPEG4_DMA_SIZE,
					 &m->workspace_dma, GFP_KERNEL);
	if (!m->workspace)
		goto free_context;
	if (upper_32_bits(m->workspace_dma + MPEG4_DMA_SIZE - 1)) {
		ret = -ERANGE;
		goto free_workspace;
	}
	/* First-run workspace clear is REQUIRED by vmpeg4_hw_ctx_restore(). */
	memset(m->workspace, 0, MPEG4_DMA_SIZE);
	sess->priv = m;
	sess->keyframe_found = 1;
	amvdec_src_change(sess, sess->width, sess->height, MPEG4_BUFFERS);
	return 0;
free_workspace:
	dma_free_coherent(sess->core->dev, MPEG4_DMA_SIZE, m->workspace,
			  m->workspace_dma);
free_context:
	kfree(m);
	return ret;
}

static int mpeg4_stop(struct amvdec_session *sess)
{
	struct codec_mpeg4 *m = sess->priv;

	/* Generic engine stop has disabled IRQs and proved DMA/DMC idle. */
	dma_free_coherent(sess->core->dev, MPEG4_DMA_SIZE, m->workspace,
			  m->workspace_dma);
	return 0;
}

static void mpeg4_resume(struct amvdec_session *sess)
{
	struct codec_mpeg4 *m = sess->priv;
	unsigned int i;
	int ret = 0;

	mutex_lock(&m->lock);
	if (!m->ready) {
		if (sess->num_dst_bufs != MPEG4_BUFFERS ||
		    sess->pixfmt_cap != V4L2_PIX_FMT_NV12M) {
			ret = -EINVAL;
			goto unlock;
		}
		ret = amvdec_set_canvases(sess, (u32[]){ ANC0_CANVAS_ADDR, 0 },
					 (u32[]){ MPEG4_BUFFERS, 0 });
		if (ret)
			goto unlock;
		for (i = 0; i < MPEG4_BUFFERS; i++)
			m->canvas[i] = amvdec_read_dos(sess->core,
						     ANC0_CANVAS_ADDR + 4 * i);
		m->ready = true;
	}
	ret = mpeg4_run(sess);
unlock:
	if (ret)
		m->error = ret;
	mutex_unlock(&m->lock);
	if (ret)
		amvdec_abort(sess);
}

static void mpeg4_input_committed(struct amvdec_session *sess, u32 offset, u32 bytes)
{
	struct codec_mpeg4 *m = sess->priv;
	int ret;

	mutex_lock(&m->lock);
	m->committed = offset + bytes;
	dev_dbg(sess->core->dev, "MPEG4 commit offset=%08x bytes=%u stream=%08x write=%08x\n",
		offset, bytes, sess->stream_offset, sess->stream_write);
	ret = mpeg4_run(sess);
	if (ret)
		m->error = ret;
	mutex_unlock(&m->lock);
	if (ret)
		amvdec_abort(sess);
}

static u32 mpeg4_input_read_pointer(struct amvdec_session *sess)
{
	struct codec_mpeg4 *m = sess->priv;

	return sess->vififo_paddr + READ_ONCE(m->retained);
}

static int mpeg4_can_recycle(struct amvdec_core *core)
{
	return 1;
}

static void mpeg4_recycle(struct amvdec_core *core, u32 index)
{
	struct amvdec_session *sess = core->cur_sess;
	struct codec_mpeg4 *m = sess->priv;
	int ret;

	mutex_lock(&m->lock);
	if (index >= MPEG4_BUFFERS || !m->picture[index].consumer) {
		ret = -EINVAL;
	} else {
		m->picture[index].consumer = false;
		ret = mpeg4_run(sess);
	}
	if (ret)
		m->error = ret;
	mutex_unlock(&m->lock);
	if (ret)
		amvdec_abort(sess);
}

static int mpeg4_drain_step(struct amvdec_session *sess)
{
	struct codec_mpeg4 *m = sess->priv;
	int ret = 0, index;

	mutex_lock(&m->lock);
	if (m->error) {
		ret = m->error;
		goto unlock;
	}
	if (!m->eos_queued) {
		ret = esparser_queue_eos(sess->core, mpeg4_eos, sizeof(mpeg4_eos));
		if (ret == -EAGAIN) {
			ret = 0;
			goto unlock;
		}
		if (ret < 0)
			goto unlock;
		m->committed = sess->stream_offset;
		m->eos_queued = true;
		m->terminal = false;
	}
	/* vmpeg4_work(): genuine stream AGAIN plus no more submitted input
	 * permits flush_output(refs[1]). There is no SINGLE guessed ANC tail,
	 * synthetic VOP, or timeout-derived completion.
	 */
	dev_dbg(sess->core->dev,
		"MPEG4 drain running=%u empty=%u terminal=%u decoded=%u pending=%u refs=%d/%d display=%d committed=%08x job=%08x stream=%08x write=%08x retained=%08x queued=%d\n",
		m->running, m->empty, m->terminal, m->decoded, m->pending,
		m->refs[0], m->refs[1], m->display, m->committed, m->job_end,
		sess->stream_offset, sess->stream_write, m->retained,
		atomic_read(&sess->esparser_queued_bufs));
	if (mpeg4_terminal_empty(sess))
		m->terminal = true;
	ret = mpeg4_run(sess);
	if (ret || !m->terminal || m->running)
		goto unlock;
	index = m->refs[1];
	if (index >= 0 && m->picture[index].pending) {
		ret = mpeg4_present(sess, index);
		if (ret == -EAGAIN) {
			ret = 0;
			goto unlock;
		}
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

struct mpeg4_bits {
	const u8 *data;
	u32 size;
	u32 pos;
	bool error;
};

static u32 mpeg4_get_bits(struct mpeg4_bits *bits, unsigned int count)
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

static void mpeg4_skip_bits(struct mpeg4_bits *bits, unsigned int count)
{
	if (count > bits->size - bits->pos)
		bits->error = true;
	else
		bits->pos += count;
}

static int mpeg4_vol_time_bits(struct mpeg4_bits *bits)
{
	u32 version = 1, shape, resolution;

	mpeg4_skip_bits(bits, 9);
	if (mpeg4_get_bits(bits, 1)) {
		version = mpeg4_get_bits(bits, 4);
		mpeg4_skip_bits(bits, 3);
	}
	if (mpeg4_get_bits(bits, 4) == 15)
		mpeg4_skip_bits(bits, 16);
	if (mpeg4_get_bits(bits, 1)) {
		mpeg4_skip_bits(bits, 3);
		if (mpeg4_get_bits(bits, 1))
			mpeg4_skip_bits(bits, 79);
	}
	shape = mpeg4_get_bits(bits, 2);
	if (shape == 3 && version != 1)
		mpeg4_skip_bits(bits, 4);
	if (mpeg4_get_bits(bits, 1) != 1)
		return -EINVAL;
	resolution = mpeg4_get_bits(bits, 16);
	if (bits->error || !resolution || mpeg4_get_bits(bits, 1) != 1)
		return -EINVAL;
	return max(1, fls(resolution - 1));
}

/* Preserve coded-VOP token accounting, including packed Xvid's uncoded dummy.
 * This parser supplies metadata only; reconstruction remains entirely VDEC1.
 */
static int mpeg4_prepare_input(struct amvdec_session *sess,
			      struct vb2_v4l2_buffer *vbuf, u32 *payload_size)
{
	struct codec_mpeg4 *m = sess->priv;
	const u8 *data = vb2_plane_vaddr(&vbuf->vb2_buf, 0);
	u32 size = *payload_size, i, pictures = 0;
	int time_bits = m->time_bits;

	if (!data)
		return -EINVAL;
	for (i = 0; i + 4 <= size; i++) {
		struct mpeg4_bits bits;
		u8 code;

		if (data[i] || data[i + 1] || data[i + 2] != 1)
			continue;
		code = data[i + 3];
		bits = (struct mpeg4_bits) {
			.data = data + i + 4, .size = (size - i - 4) * 8,
		};
		if ((code & 0xf0) == 0x20) {
			time_bits = mpeg4_vol_time_bits(&bits);
			if (time_bits < 0)
				return time_bits;
		} else if (code == 0xb6) {
			if (!time_bits)
				return -EINVAL;
			mpeg4_skip_bits(&bits, 2);
			while (mpeg4_get_bits(&bits, 1))
				;
			if (mpeg4_get_bits(&bits, 1) != 1)
				return -EINVAL;
			mpeg4_skip_bits(&bits, time_bits);
			if (mpeg4_get_bits(&bits, 1) != 1)
				return -EINVAL;
			pictures += mpeg4_get_bits(&bits, 1);
			if (bits.error)
				return -EINVAL;
		}
		i += 3;
	}
	m->time_bits = time_bits;
	sess->input_frames = pictures;
	return 0;
}


struct amvdec_codec_ops codec_mpeg4_ops = {
	.manages_processor = true,
	.start = mpeg4_start,
	.stop = mpeg4_stop,
	.isr = mpeg4_isr,
	.threaded_isr = mpeg4_threaded_isr,
	.can_recycle = mpeg4_can_recycle,
	.recycle = mpeg4_recycle,
	.resume = mpeg4_resume,
	.prepare_input = mpeg4_prepare_input,
	.input_committed = mpeg4_input_committed,
	.input_read_pointer = mpeg4_input_read_pointer,
	.drain_step = mpeg4_drain_step,
};
