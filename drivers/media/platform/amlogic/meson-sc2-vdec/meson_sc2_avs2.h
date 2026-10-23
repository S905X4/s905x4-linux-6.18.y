/* SPDX-License-Identifier: GPL-2.0-only */
/* Included by meson_sc2_hw.c, after its private engine/DMA helpers.
 * AVS2 P2 uses the vendor RPM/DPB protocol, not a Request syntax shim.
 * Firmware 27 on core 2 parses the actual compressed access unit. All retained
 * pictures are native MMU/FBC objects; double-write is deliberately disabled.
 */
#include "meson_sc2_avs2_api.h"
#include "meson_sc2_avs2_global.h"

#define A2_REG(n) ((n) * 4U)
#define A2_SHIFT_PROTECT A2_REG(0x313a)
#define A2_PARSER_RESULT3 A2_REG(0x311b)
#define A2_MPRED_CTRL9 A2_REG(0x325b)
#define A2_DBLK_CFGD A2_REG(0x350d)
#define A2_MCRCC_PERF A2_REG(0x34f3)
#define A2_DECOMP_PERF A2_REG(0x34c5)

/* TM2-revB/SC2 4096x2304 workspace from vendor specification 4. DBLK
 * parameter/data capacities are 0x8100/0x88800/0x88800, each rounded to a
 * page. Earlier-generation 0x80000 data buffers are too small for SC2.
 * RPM/LMEM reserve the full 0x600-word bank. VBH has two 0x2800 banks.
 */
#define A2_IPP       0x000000
#define A2_ALF       0x004000
#define A2_RCS       0x005000
#define A2_SPS       0x006000
#define A2_PPS       0x007000
#define A2_SWAP2     0x00a000
#define A2_DBLK_PARA 0x00b000
#define A2_DBLK_DATA 0x014000
#define A2_DBLK_DATA2 0x09d000
#define A2_VBH       0x126000
#define A2_MPRED_ABOVE 0x12c000
#define A2_RPM       0x130000
#define A2_LMEM      0x132000
#define A2_MAP       0x134000
#define A2_CUVA      0x144000
#define A2_WORK_SIZE 0x145000

struct sc2_avs2_picture {
	struct meson_sc2_surface *surface;
	struct meson_sc2_surface storage;
	u64 timestamp;
	u8 depth;
	bool valid;
};

struct sc2_avs2_state {
	struct avs2_decoder dec;
	struct sc2_avs2_picture picture[AVS2_MAX_BUFFER_NUM];
	u32 iqit[1024];
	u32 start_flag, decode_count;
	bool iqit_valid;
};

static const u16 sc2_avs2_wq4[16] = {
	64, 64, 64, 68, 64, 64, 68, 72,
	64, 68, 76, 80, 72, 76, 84, 96,
};
static const u16 sc2_avs2_wq8[64] = {
	64, 64, 64, 64, 68, 68, 72, 76,
	64, 64, 64, 68, 72, 76, 84, 92,
	64, 64, 68, 72, 76, 80, 88, 100,
	64, 68, 72, 80, 84, 92, 100, 112,
	68, 72, 80, 84, 92, 104, 112, 128,
	76, 80, 84, 92, 104, 116, 132, 152,
	96, 100, 104, 116, 124, 140, 164, 188,
	104, 108, 116, 128, 152, 172, 192, 216,
};

static void sc2_avs2_reset(struct meson_sc2_hevc_context *ctx)
{
	struct sc2_avs2_state *st = ctx->avs2;

	if (!st)
		return;
	memset(st, 0, sizeof(*st));
	init_avs2_decoder(&st->dec);
	st->dec.ref_maxbuffer = REF_MAXBUFFER;
	/* Establish stable frame indices before the first sequence notification:
	 * prepare_header may flush the DPB even before the first picture header.
	 */
	avs2_init_global_buffers(&st->dec);
	memset(ctx->work, 0, A2_WORK_SIZE);
}

/* Decoder failure retires its references, not presentations already handed
 * to userspace. Their QBUF must still release the original slot ownership.
 */
static void sc2_avs2_error_reset(struct meson_sc2_hevc_context *ctx)
{
	struct sc2_avs2_state *st = ctx->avs2;
	int held[AVS2_MAX_BUFFER_NUM];
	unsigned int i;

	for (i = 0; i < AVS2_MAX_BUFFER_NUM; i++) {
		held[i] = st->dec.frm_pool[i].vf_ref;
		if (!held[i])
			memset(&st->picture[i], 0, sizeof(st->picture[i]));
	}
	init_avs2_decoder(&st->dec);
	avs2_init_global_buffers(&st->dec);
	for (i = 0; i < AVS2_MAX_BUFFER_NUM; i++)
		st->dec.frm_pool[i].vf_ref = held[i];
	st->start_flag = 0;
	st->decode_count = 0;
	st->iqit_valid = false;
	memset(ctx->work, 0, A2_WORK_SIZE);
}

static void sc2_avs2_release(struct meson_sc2_hevc_context *ctx)
{
	kfree(ctx->avs2);
	ctx->avs2 = NULL;
}

static int sc2_avs2_alloc(struct meson_sc2_hevc_context *ctx)
{
	BUILD_BUG_ON(A2_WORK_SIZE > SC2_WORK_SIZE);
	BUILD_BUG_ON(AVS2_MAX_BUFFER_NUM > 32);
	if (ctx->avs2)
		return 0;
	ctx->avs2 = kzalloc(sizeof(*ctx->avs2), GFP_KERNEL);
	if (!ctx->avs2)
		return -ENOMEM;
	sc2_avs2_reset(ctx);
	return 0;
}

/* Match the manager's reuse criterion, not just referred_by_others: F pictures
 * and output-pending nonreferences can carry a zero reference flag while their
 * coding-order/POC identity is still live. Background storage is independent.
 */
static bool sc2_avs2_frame_busy(struct avs2_decoder *dec,
			       struct avs2_frame_s *f)
{
	return dec->hc.cur_pic == f || dec->f_bg == f || f->bg_flag ||
		f->vf_ref || f->to_prepare_disp || f->is_output != -1 ||
		f->imgcoi_ref >= -256;
}

static bool sc2_avs2_busy_locked(struct sc2_avs2_state *st,
				struct meson_sc2_surface *surface)
{
	unsigned int i;

	if (!st)
		return false;
	for (i = 0; i < AVS2_MAX_BUFFER_NUM; i++)
		if (st->picture[i].surface == surface && st->picture[i].valid &&
		    sc2_avs2_frame_busy(&st->dec, &st->dec.frm_pool[i]))
			return true;
	return false;
}

bool meson_sc2_avs2_surface_busy(struct meson_sc2_hevc_context *ctx,
			       struct meson_sc2_surface *surface)
{
	bool busy;

	if (!ctx || !surface)
		return false;
	mutex_lock(&ctx->hw->lock);
	busy = ctx->hw->fatal || sc2_avs2_busy_locked(ctx->avs2, surface);
	mutex_unlock(&ctx->hw->lock);
	return busy;
}

static void sc2_avs2_parser(struct meson_sc2_hevc_context *ctx, size_t length)
{
	struct meson_sc2_hw *hw = ctx->hw;
	struct sc2_avs2_state *st = ctx->avs2;
	u32 b = lower_32_bits(ctx->work_dma);
	u32 in = lower_32_bits(ctx->input_dma);
	u32 above;
	unsigned int i;

	sc2_write(hw, HEVCD_IPP_LINEBUFF_BASE, b + A2_IPP);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_1, b + A2_RPM);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_2, b + A2_ALF);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_3, b + A2_RCS);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_4, b + A2_SPS);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_5, b + A2_PPS);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_7, b + A2_MAP);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_8, b + A2_SWAP2);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_9, b + A2_LMEM);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_A, b + A2_CUVA);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_B, (SZ_4K >> 4) << 16);
	sc2_write(hw, HEVC_DBLK_CFG3, 0x404010);
	sc2_write(hw, HEVC_DBLK_CFG4, b + A2_DBLK_PARA);
	sc2_write(hw, HEVC_DBLK_CFG5, b + A2_DBLK_DATA);
	sc2_write(hw, HEVC_DBLK_CFGE, b + A2_DBLK_DATA2);
	sc2_write(hw, HEVC_SAO_MMU_VH0_ADDR, b + A2_VBH);
	sc2_write(hw, HEVC_SAO_MMU_VH1_ADDR, b + A2_VBH + 0x2800);
	sc2_write(hw, HEVC_MPRED_ABV_START_ADDR, b + A2_MPRED_ABOVE);
	sc2_write(hw, HEVC_SAO_CTRL5,
		  (sc2_read(hw, HEVC_SAO_CTRL5) | BIT(10)) &
		  ~(BIT(9) | BIT(15) | BIT(7)));
	sc2_write(hw, HEVCD_MPP_DECOMP_CTL1, BIT(4));
	sc2_write(hw, HEVCD_MPP_DECOMP_CTL2, 0);
	sc2_write(hw, HEVC_STREAM_CONTROL, 0x0eb14070);
	sc2_write(hw, HEVC_STREAM_START_ADDR, in);
	sc2_write(hw, HEVC_STREAM_END_ADDR, in + SC2_INPUT_SIZE);
	sc2_write(hw, HEVC_STREAM_RD_PTR, in);
	sc2_write(hw, HEVC_STREAM_WR_PTR,
		  in + round_down(length + SC2_INPUT_PADDING, 64));
	sc2_write(hw, HEVC_STREAM_FIFO_CTL,
		  sc2_read(hw, HEVC_STREAM_FIFO_CTL) | BIT(29));
	sc2_write(hw, HEVC_PARSER_INT_CONTROL,
		  (sc2_read(hw, HEVC_PARSER_INT_CONTROL) & ~(7U << 29)) |
		  (3U << 29) | BIT(24) | BIT(22) | BIT(7) | BIT(4) | BIT(0));
	sc2_write(hw, HEVC_SHIFT_STATUS,
		  (sc2_read(hw, HEVC_SHIFT_STATUS) & ~BIT(1)) | BIT(0));
	sc2_write(hw, HEVC_SHIFT_CONTROL,
		  (6 << 20) | (3 << 6) | (2 << 4) | (2 << 1) | 1);
	sc2_write(hw, A2_SHIFT_PROTECT, BIT(29));
	sc2_write(hw, HEVC_SHIFT_STARTCODE, 0x00000100);
	sc2_write(hw, HEVC_SHIFT_EMULATECODE, 0);
	sc2_write(hw, HEVC_SHIFT_BYTE_COUNT, 0);
	sc2_write(hw, HEVC_CABAC_CONTROL, 1);
	sc2_write(hw, HEVC_PARSER_CORE_CONTROL, 1);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_0, 0);
	sc2_write(hw, HEVC_IQIT_SCALELUT_WR_ADDR, 0);
	for (i = 0; i < ARRAY_SIZE(st->iqit); i++)
		sc2_write(hw, HEVC_IQIT_SCALELUT_DATA,
			  st->iqit_valid ? st->iqit[i] : 0);
	if (!st->iqit_valid) {
		sc2_write(hw, HEVC_IQIT_SCALELUT_WR_ADDR, 64);
		for (i = 0; i < ARRAY_SIZE(sc2_avs2_wq4); i++)
			sc2_write(hw, HEVC_IQIT_SCALELUT_DATA, sc2_avs2_wq4[i]);
		sc2_write(hw, HEVC_IQIT_SCALELUT_WR_ADDR, 0);
		for (i = 0; i < ARRAY_SIZE(sc2_avs2_wq8); i++)
			sc2_write(hw, HEVC_IQIT_SCALELUT_DATA, sc2_avs2_wq8[i]);
	}
	sc2_write(hw, HEVC_PARSER_CMD_WRITE, BIT(16));
	for (i = 0; i < ARRAY_SIZE(sc2_parser_commands); i++)
		sc2_write(hw, HEVC_PARSER_CMD_WRITE, sc2_parser_commands[i]);
	sc2_write(hw, HEVC_PARSER_CMD_SKIP_0, 0x0000090b);
	sc2_write(hw, HEVC_PARSER_CMD_SKIP_1, 0x1b14140f);
	sc2_write(hw, HEVC_PARSER_CMD_SKIP_2, 0x001b1910);
	sc2_write(hw, HEVC_PARSER_IF_CONTROL, BIT(9) | BIT(5) | BIT(2) | BIT(0));
	sc2_write(hw, HEVC_MPRED_INT_STATUS, BIT(31));
	sc2_write(hw, A2_PARSER_RESULT3, U32_MAX);
	/* Vendor multi-instance initialization resets MPRED after enabling the
	 * parser interfaces, then restores its above-buffer latch three times.
	 * The shared core reset before parser setup does not replace this step.
	 */
	for (i = 0; i < 8; i++)
		above = sc2_read(hw, HEVC_MPRED_ABV_START_ADDR);
	sc2_write(hw, SC2_DOS_SW_RESET3, BIT(18));
	sc2_write(hw, SC2_DOS_SW_RESET3, 0);
	sc2_write(hw, HEVC_MPRED_ABV_START_ADDR, above);
	sc2_write(hw, HEVC_MPRED_ABV_START_ADDR, above);
	sc2_write(hw, HEVC_MPRED_ABV_START_ADDR, above);
	sc2_write(hw, HEVCD_IPP_TOP_CNTL, 1);
	sc2_write(hw, HEVCD_IPP_TOP_CNTL, 2);
	sc2_write(hw, HEVC_DBLK_CFGB,
		  (sc2_read(hw, HEVC_DBLK_CFGB) & ~0x30f) | BIT(8) | 2);
	sc2_write(hw, HEVC_DBLK_CFG0, 1);
	sc2_write(hw, A2_MCRCC_PERF, 1);
	sc2_write(hw, A2_MCRCC_PERF, 0);
	sc2_write(hw, A2_DECOMP_PERF, 1);
	sc2_write(hw, A2_DECOMP_PERF, 0);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_E, 1);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_F, 0);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_G, 0);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_H, 0);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_I, 8);
	/* Vendor frame-based mode includes the new-generation chip feature byte.
	 * Bits 16..23 retain sequence/I readiness across loaded firmware jobs.
	 */
	sc2_write(hw, HEVC_ASSIST_SCRATCH_J, 0x80000002U | (st->start_flag << 16));
	sc2_write(hw, HEVC_ASSIST_SCRATCH_K, 0);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_L, 0);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_M, st->decode_count);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_N, length);
	sc2_write(hw, HEVC_PSCALE_CTRL, 0);
	sc2_write(hw, SC2_MBOX_CLEAR, 1);
	sc2_write(hw, SC2_MBOX_MASK, 1);
	sc2_write(hw, HEVC_STREAM_CONTROL, sc2_read(hw, HEVC_STREAM_CONTROL) | 1);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_0, 0xf1); /* AVS2_SEARCH_NEW_PIC */
}

static u32 sc2_avs2_canvas(struct avs2_frame_s *f)
{
	return f->index * 0x010101U;
}

static int sc2_avs2_mc(struct sc2_avs2_state *st, struct meson_sc2_hw *hw)
{
	struct avs2_decoder *d = &st->dec;
	unsigned int i;

	for (i = 0; i < AVS2_MAX_BUFFER_NUM; i++) {
		struct sc2_avs2_picture *p = &st->picture[i];

		sc2_write(hw, HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR, BIT(1) | (i << 8));
		sc2_write(hw, HEVCD_MPP_ANC2AXI_TBL_DATA,
			  p->valid ? lower_32_bits(p->storage.dma) >> 5 : 0);
	}
	sc2_write(hw, HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR, 1);
	sc2_write(hw, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, 1);
	for (i = 0; i < 32; i++)
		sc2_write(hw, HEVCD_MPP_ANC_CANVAS_DATA_ADDR, 0);
	if (d->f_bg) {
		if (d->f_bg->index < 0 || d->f_bg->index >= AVS2_MAX_BUFFER_NUM ||
		    !st->picture[d->f_bg->index].valid)
			return -ENOENT;
		sc2_write(hw, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, (15 << 8) | 1);
		sc2_write(hw, HEVCD_MPP_ANC_CANVAS_DATA_ADDR, sc2_avs2_canvas(d->f_bg));
		sc2_write(hw, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, (31 << 8) | 1);
		sc2_write(hw, HEVCD_MPP_ANC_CANVAS_DATA_ADDR, sc2_avs2_canvas(d->f_bg));
	}
	if (d->img.type == I_IMG)
		return 0;
	for (i = 0; i < d->img.num_of_references; i++) {
		struct avs2_frame_s *f = d->fref[i];

		if (f->index < 0 || f->index >= AVS2_MAX_BUFFER_NUM ||
		    !st->picture[f->index].valid || f->error_mark)
			return -ENOENT;
	}
	if (d->img.type == B_IMG) {
		sc2_write(hw, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, 1);
		sc2_write(hw, HEVCD_MPP_ANC_CANVAS_DATA_ADDR, sc2_avs2_canvas(d->fref[1]));
		sc2_write(hw, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, (16 << 8) | 1);
		sc2_write(hw, HEVCD_MPP_ANC_CANVAS_DATA_ADDR, sc2_avs2_canvas(d->fref[0]));
	} else {
		sc2_write(hw, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, 1);
		for (i = 0; i < d->img.num_of_references; i++)
			sc2_write(hw, HEVCD_MPP_ANC_CANVAS_DATA_ADDR, sc2_avs2_canvas(d->fref[i]));
		if (d->img.type == F_IMG) {
			sc2_write(hw, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, (16 << 8) | 1);
			for (i = 0; i < d->img.num_of_references; i++)
				sc2_write(hw, HEVCD_MPP_ANC_CANVAS_DATA_ADDR,
					  sc2_avs2_canvas(d->fref[i]));
		}
	}
	return 0;
}

static void sc2_avs2_mcrcc(struct avs2_decoder *d, struct meson_sc2_hw *hw)
{
	u32 a, b;

	sc2_write(hw, HEVCD_MCRCC_CTL1, 2);
	if (d->img.type == I_IMG) {
		sc2_write(hw, HEVCD_MCRCC_CTL1, 0);
		return;
	}
	sc2_write(hw, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR,
		  d->img.type == P_IMG ? 2 : 0);
	a = sc2_read(hw, HEVCD_MPP_ANC_CANVAS_DATA_ADDR) & 0xffff;
	a |= a << 16;
	sc2_write(hw, HEVCD_MCRCC_CTL2, a);
	if (d->img.type == B_IMG || d->img.type == F_IMG)
		sc2_write(hw, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, (16 << 8) | 2);
	b = sc2_read(hw, HEVCD_MPP_ANC_CANVAS_DATA_ADDR) & 0xffff;
	b |= b << 16;
	if ((d->img.type == B_IMG || d->img.type == F_IMG) && a == b) {
		b = sc2_read(hw, HEVCD_MPP_ANC_CANVAS_DATA_ADDR) & 0xffff;
		b |= b << 16;
	}
	sc2_write(hw, HEVCD_MCRCC_CTL3, b);
	sc2_write(hw, HEVCD_MCRCC_CTL1, 0xff0);
}

static void sc2_avs2_mpred(struct avs2_decoder *d, struct meson_sc2_hw *hw)
{
	struct avs2_frame_s *cur = d->hc.cur_pic;
	struct avs2_frame_s *col = d->img.num_of_references ? d->fref[0] :
		(d->f_bg ? d->f_bg : cur);
	u32 unit = d->lcu_size_log2 == 6 ? 0x200 :
		   d->lcu_size_log2 == 5 ? 0x80 : 0x20;
	unsigned int i;

	sc2_write(hw, HEVC_MPRED_MV_WR_START_ADDR, cur->mpred_mv_wr_start_addr);
	sc2_write(hw, HEVC_MPRED_MV_RD_START_ADDR, col->mpred_mv_wr_start_addr);
	sc2_write(hw, A2_MPRED_CTRL9,
		  ((d->bk_img_is_top_field & 1) << 13) |
		  ((d->hd.background_picture_enable & 1) << 12) |
		  ((d->hd.curr_RPS.num_of_ref & 7) << 8) |
		  ((d->hd.b_pmvr_enabled & 1) << 6) |
		  ((d->img.is_top_field & 1) << 5) |
		  ((d->img.is_field_sequence & 1) << 4) |
		  ((d->img.typeb & 7) << 1) |
		  (d->hd.background_reference_enable & 1));
	sc2_write(hw, HEVC_MPRED_CUR_POC, d->img.pic_distance);
	sc2_write(hw, HEVC_MPRED_COL_POC, d->img.imgtr_next_P);
	/* Pair order is a hardware latch requirement, not two independent arrays. */
	for (i = 0; i < 7; i++) {
		sc2_write(hw, HEVC_MPRED_L0_REF00_POC + i * 4,
			  d->fref[i]->imgtr_fwRefDistance);
		sc2_write(hw, HEVC_MPRED_L1_REF00_POC + i * 4,
			  col->ref_poc[i]);
	}
	sc2_write(hw, HEVC_MPRED_MV_RD_END_ADDR,
		  col->mpred_mv_wr_start_addr + d->lcu_x_num * d->lcu_y_num * unit);
}

static void sc2_avs2_alf_coeff(struct meson_sc2_hw *hw,
			     const struct ALFParam_s *a, unsigned int group,
			     bool last)
{
	s32 c[9], sum = 0;
	unsigned int i;

	for (i = 0; i < 8; i++) {
		c[i] = a->coeffmulti[group][i];
		sum += 2 * c[i];
	}
	c[8] = 64 - sum + a->coeffmulti[group][8];
	sc2_write(hw, A2_DBLK_CFGD,
		  ((c[4] & 0xf) << 28) | ((c[3] & 0x7f) << 21) |
		  ((c[2] & 0x7f) << 14) | ((c[1] & 0x7f) << 7) | (c[0] & 0x7f));
	sc2_write(hw, A2_DBLK_CFGD,
		  (last ? BIT(31) : 0) | ((c[8] & 0x7f) << 24) |
		  ((c[7] & 0x7f) << 17) | ((c[6] & 0x7f) << 10) |
		  ((c[5] & 0x7f) << 3) | ((c[4] >> 4) & 7));
}

static void sc2_avs2_filters(struct avs2_decoder *d, struct meson_sc2_hw *hw)
{
	union param_u *p = &d->param;
	struct ALFParam_s *y = &d->m_alfPictureParam[0];
	u32 v, table = 0, bin = 0;
	unsigned int i, groups = y->alf_flag ? y->filters_per_group : 1;

	v = sc2_read(hw, HEVC_DBLK_CFG1) & ~(0xf0000 | 3);
	v |= (d->input.sample_bit_depth == 10 ? 0xa0000 : 0) |
		(p->p.lcu_size == 6 ? 0 : p->p.lcu_size == 5 ? 1 : 2);
	sc2_write(hw, HEVC_DBLK_CFG1, v);
	sc2_write(hw, HEVC_DBLK_CFG2, (d->img.height << 16) | d->img.width);
	sc2_write(hw, HEVC_DBLK_CFG9,
		  ((d->input.crossSliceLoopFilter & 1) << 27) |
		  ((p->p.loop_filter_disable & 1) << 26) |
		  ((d->input.useNSQT & 1) << 25) | ((d->img.type & 7) << 22) |
		  ((p->p.alpha_c_offset & 0x1f) << 17) |
		  ((p->p.beta_offset & 0x1f) << 12) |
		  ((p->p.chroma_quant_param_delta_cb & 0x3f) << 6) |
		  (p->p.chroma_quant_param_delta_cr & 0x3f));
	/* DBLK's ALF port consumes Cr, Cb, two Y-bin words, then Y groups. */
	sc2_avs2_alf_coeff(hw, &d->m_alfPictureParam[2], 0, false);
	sc2_avs2_alf_coeff(hw, &d->m_alfPictureParam[1], 0, false);
	for (i = 0; i < 16; i++) {
		if (i && y->filters_per_group > 1 && y->filterPattern[i])
			bin++;
		table |= (bin & 0xf) << ((i & 7) * 4);
		if ((i & 7) == 7) {
			sc2_write(hw, A2_DBLK_CFGD, table);
			table = 0;
		}
	}
	for (i = 0; i < groups; i++)
		sc2_avs2_alf_coeff(hw, y, i, i == groups - 1);
}

static int sc2_avs2_program(struct meson_sc2_hevc_context *ctx,
			  const struct meson_sc2_avs2_job *job)
{
	struct sc2_avs2_state *st = ctx->avs2;
	struct avs2_decoder *d = &st->dec;
	struct meson_sc2_hw *hw = ctx->hw;
	struct meson_sc2_surface *s = job->output.destination;
	__le32 *map = ctx->work + A2_MAP;
	u32 body = DIV_ROUND_UP(s->width, 64) * DIV_ROUND_UP(s->height, 32) *
		   (d->input.sample_bit_depth == 10 ? 4096 : 3200);
	u32 pages = DIV_ROUND_UP(body, SZ_4K), value;
	unsigned int i;
	int ret;

	if (body > s->body_size || pages > SC2_MAP_ENTRIES ||
	    d->lcu_total * (d->lcu_size_log2 == 6 ? 0x200 :
			   d->lcu_size_log2 == 5 ? 0x80 : 0x20) > s->mv_size)
		return -EOVERFLOW;
	for (i = 0; i < SC2_MAP_ENTRIES; i++)
		map[i] = cpu_to_le32((s->dma + s->body_offset +
				     (i < pages ? i * SZ_4K : 0)) >> 12);
	ret = sc2_avs2_mc(st, hw);
	if (ret)
		return ret;
	sc2_avs2_mcrcc(d, hw);
	sc2_avs2_mpred(d, hw);
	sc2_avs2_filters(d, hw);
	sc2_write(hw, HEVC_SAO_CTRL0,
		  (sc2_read(hw, HEVC_SAO_CTRL0) & ~0xf) | d->lcu_size_log2);
	sc2_write(hw, HEVC_SAO_Y_START_ADDR, U32_MAX);
	sc2_write(hw, HEVC_SAO_C_START_ADDR, U32_MAX);
	sc2_write(hw, HEVC_SAO_Y_LENGTH, 0);
	sc2_write(hw, HEVC_SAO_C_LENGTH, 0);
	sc2_write(hw, HEVC_CM_BODY_START_ADDR, lower_32_bits(s->dma + s->body_offset));
	sc2_write(hw, HEVC_CM_HEADER_START_ADDR, lower_32_bits(s->dma));
	value = sc2_read(hw, HEVC_SAO_CTRL1) & ~(0x3000 | 0xff0 | 3 | (3 << 14));
	sc2_write(hw, HEVC_SAO_CTRL1, value | (2 << 14) | 0xff0 | BIT(1));
	value = sc2_read(hw, HEVCD_IPP_AXIIF_CONFIG) & ~(0x30 | 0xf | (3 << 8));
	sc2_write(hw, HEVCD_IPP_AXIIF_CONFIG, value | 0xf | (2 << 8));
	value = sc2_read(hw, HEVC_SAO_CTRL5) & ~(BIT(9) | BIT(15) | (0xffU << 16));
	if (d->input.sample_bit_depth == 8)
		value |= BIT(9);
	sc2_write(hw, HEVC_SAO_CTRL5, value | BIT(10));
	sc2_write(hw, HEVCD_MPP_DECOMP_CTL1, BIT(4));
	sc2_write(hw, HEVCD_MPP_DECOMP_CTL2, body >> 5);
	sc2_write(hw, HEVC_CM_BODY_LENGTH, body);
	sc2_write(hw, HEVC_CM_HEADER_OFFSET, body);
	sc2_write(hw, HEVC_CM_HEADER_LENGTH, s->header_size);
	d->hc.cur_pic->comp_body_size = body;
	dma_wmb();
	return 0;
}

static int sc2_avs2_header(struct meson_sc2_hevc_context *ctx,
			   const struct meson_sc2_avs2_job *job,
			   struct meson_sc2_avs2_result *result, u32 status,
			   const u8 *pending, unsigned int pending_count)
{
	struct sc2_avs2_state *st = ctx->avs2;
	struct avs2_decoder *d = &st->dec;
	struct meson_sc2_surface *s = job->output.destination;
	const __le16 *raw = ctx->work + A2_RPM;
	struct avs2_frame_s *f;
	u32 depth;
	unsigned int i;
	int ret;

	dma_rmb();
	for (i = 0; i < ARRAY_SIZE(d->param.l.data); i++)
		d->param.l.data[i] = le16_to_cpu(raw[(i & ~3U) + 3 - (i & 3)]);
	depth = (d->param.p.profile_id == BASELINE10_PROFILE ||
		 d->param.p.profile_id == SCENE10_PROFILE) &&
		d->param.p.encoding_precision == 2 ? 10 : 8;
	result->width = d->param.p.horizontal_size;
	result->height = d->param.p.vertical_size;
	result->depth = depth;
	if (!result->width || !result->height ||
	    result->width > MESON_SC2_AVS2_MAX_WIDTH ||
	    result->height > MESON_SC2_AVS2_MAX_HEIGHT || d->param.p.chroma_format != 1 ||
	    d->param.p.lcu_size < 4 || d->param.p.lcu_size > 6 ||
	    d->param.p.num_of_ref_cur > MAXREF || d->param.p.num_to_remove_cur > MAXREF)
		return -EBADMSG;
	if (!s || s->width != result->width || s->height != result->height ||
	    s->depth != depth)
		return -EPIPE;
	/* Sequence/edit/end notifications can retire DPB entries. Defer them
	 * until geometry is accepted, so a probe or source change is replayable.
	 */
	for (i = 0; i < pending_count; i++) {
		avs2_prepare_header(d, pending[i]);
		if (d->error)
			return d->error;
	}
	avs2_prepare_header(d, status == 0x22 ? I_PICTURE_START_CODE : PB_PICTURE_START_CODE);
	if (d->error || d->bufmgr_error_flag)
		return d->error ?: -EBADMSG;
	ret = avs2_process_header(d);
	if (ret < 0)
		return ret;
	f = d->hc.cur_pic;
	if (!f || f->index < 0 || f->index >= AVS2_MAX_BUFFER_NUM)
		return -EPROTO;
	/* A CAPTURE object may be reassigned only after its old DPB identity died. */
	for (i = 0; i < AVS2_MAX_BUFFER_NUM; i++)
		if (st->picture[i].surface == s)
			st->picture[i].valid = false;
	st->picture[f->index] = (struct sc2_avs2_picture) {
		.surface = s, .storage = *s, .timestamp = job->timestamp,
		.depth = depth, .valid = true,
	};
	f->header_adr = s->dma;
	f->mpred_mv_wr_start_addr = lower_32_bits(s->mv_dma);
	f->mc_canvas_y = f->index;
	f->mc_canvas_u_v = f->index;
	f->pic_w = result->width;
	f->pic_h = result->height;
	f->bit_depth = depth;
	f->double_write_mode = 0;
	f->lcu_total = d->lcu_total;
	f->pts64 = job->timestamp;
	f->decode_idx = st->decode_count;
	f->error_mark = 0;
	f->mmu_alloc_flag = 1;
	return sc2_avs2_program(ctx, job);
}

static int sc2_avs2_run(struct meson_sc2_hevc_context *ctx,
			const struct meson_sc2_avs2_job *job,
			struct meson_sc2_avs2_result *result, bool *picture)
{
	struct meson_sc2_hw *hw = ctx->hw;
	struct sc2_avs2_state *st = ctx->avs2;
	unsigned long deadline = jiffies + SC2_JOB_TIMEOUT, remaining;
	u32 status, sequence, code, debug_tag, header_history = 0;
	u8 pending[64];
	unsigned int notifications = 0, pending_count = 0, i;
	int ret;

	*picture = false;
	sc2_avs2_parser(ctx, job->size);
	dma_wmb();
	sc2_write(hw, SC2_DOS_SW_RESET3, BIT(12) | BIT(11));
	sc2_write(hw, SC2_DOS_SW_RESET3, 0);
	sc2_read(hw, SC2_DOS_SW_RESET3);
	usleep_range(10, 20);
	sc2_write(hw, HEVC_MPSR, 1);
	for (;;) {
		if (atomic_read(&ctx->aborted))
			return -ECANCELED;
		if (time_after_eq(jiffies, deadline)) {
			ret = -ETIMEDOUT;
			goto failed;
		}
		sequence = smp_load_acquire(&hw->irq_sequence);
		status = sc2_read(hw, HEVC_ASSIST_SCRATCH_0) & 0xff;
		/* The vendor hard IRQ handles DEBUG_REG1 before interpreting status:
		 * firmware may pause for this acknowledgment while status still holds
		 * its internal sequence/I/PB parsing state, before RPM is exported.
		 */
		debug_tag = sc2_read(hw, HEVC_ASSIST_SCRATCH_G);
		if (debug_tag) {
			dev_info_ratelimited(hw->dev,
				"AVS2 firmware debug tag=%#x value=%#x status=%#x bytes=%u\n",
				debug_tag, sc2_read(hw, HEVC_ASSIST_SCRATCH_H),
				status, sc2_read(hw, HEVC_SHIFT_BYTE_COUNT));
			/* Never acknowledge a firmware-requested pause as normal debug. */
			if (debug_tag & BIT(17)) {
				ret = -EIO;
				goto failed;
			}
			sc2_write(hw, HEVC_ASSIST_SCRATCH_G, 0);
			if (!(debug_tag & BIT(16)))
				continue;
		}
		if (status >= 0x21 && status <= 0x25) {
			header_history = (header_history << 8) | status;
			if (++notifications > 64) {
				ret = -EPROTO;
				goto failed;
			}
			if (status == 0x22 || status == 0x23) {
				if (*picture) {
					ret = -EBADMSG;
					goto failed;
				}
				ret = sc2_avs2_header(ctx, job, result, status,
						     pending, pending_count);
				if (ret)
					goto failed;
				*picture = true;
				pending_count = 0;
			} else {
				code = status == 0x21 ? SEQUENCE_HEADER_CODE :
				       status == 0x24 ? SEQUENCE_END_CODE :
				       sc2_read(hw, HEVC_ASSIST_SCRATCH_J) & 0xff;
				if (code != SEQUENCE_HEADER_CODE && code != SEQUENCE_END_CODE &&
				    code != VIDEO_EDIT_CODE) {
					ret = -EBADMSG;
					goto failed;
				}
				pending[pending_count++] = code;
				if (code == SEQUENCE_HEADER_CODE)
					st->start_flag |= 1;
			}
			dma_wmb();
			sc2_write(hw, HEVC_ASSIST_SCRATCH_0, 0xff);
			continue;
		}
		if (status == 0x30) {
			u32 lcu = sc2_read(hw, HEVC_PARSER_LCU_START) & 0xffffff;

			if (!*picture || lcu + 1 != st->dec.lcu_total ||
			    sc2_read(hw, HEVC_SHIFT_BYTE_COUNT) > job->size + SC2_INPUT_PADDING ||
			    (sc2_read(hw, SC2_SAO_MMU_STATUS) >> 16) >
			    DIV_ROUND_UP(st->dec.hc.cur_pic->comp_body_size, SZ_4K)) {
				ret = -EIO;
				goto failed;
			}
			st->dec.hc.cur_pic->decoded_lcu = st->dec.lcu_total;
			st->start_flag |= 3;
			return 0;
		}
		if (status >= 0x31 && status <= 0x36) {
			if (status == 0x34 || status == 0x35) {
				ret = status == 0x34 ? -ETIMEDOUT : -EOVERFLOW;
				goto failed;
			}
			/* Header-only access units legitimately exhaust input without
			 * a decoded picture. Exhaustion during a picture is corruption.
			 */
			if (!*picture && notifications &&
			    (status == 0x32 || status == 0x33 || status == 0x36)) {
				for (i = 0; i < pending_count; i++) {
					avs2_prepare_header(&st->dec, pending[i]);
					if (st->dec.error) {
						ret = st->dec.error;
						goto failed;
					}
				}
				result->hidden = true;
				return 0;
			}
			ret = -EBADMSG;
			goto failed;
		}
		remaining = deadline - jiffies;
		if ((long)remaining <= 0) {
			ret = -ETIMEDOUT;
			goto failed;
		}
		wait_event_timeout(hw->wait, atomic_read(&ctx->aborted) ||
			smp_load_acquire(&hw->irq_sequence) != sequence, remaining);
	}
failed:
	/* Sample before drain/reset destroys the firmware's rejection context.
	 * EPIPE is the expected geometry probe, not a decode failure.
	 */
	if (ret != -EPIPE) {
		const __le16 *raw = ctx->work + A2_RPM;
		union param_u *p = &st->dec.param;

		dma_rmb();
		for (i = 0; i < ARRAY_SIZE(p->l.data); i++)
			p->l.data[i] = le16_to_cpu(raw[(i & ~3U) + 3 - (i & 3)]);
		dev_err_ratelimited(hw->dev,
			"AVS2 ret=%d status=%#x notifications=%u history=%#x picture=%u mode=%#x debug=%#x/%#x rpmcmd=%#x bytes=%u/%zu RPM profile=%#x size=%ux%u chroma=%u precision=%u/%u lcu=%u/%u refs=%u remove=%u coi=%u\n",
			ret, sc2_read(hw, HEVC_ASSIST_SCRATCH_0), notifications,
			header_history, *picture, sc2_read(hw, HEVC_ASSIST_SCRATCH_J),
			sc2_read(hw, HEVC_ASSIST_SCRATCH_G),
			sc2_read(hw, HEVC_ASSIST_SCRATCH_H),
			sc2_read(hw, HEVC_ASSIST_SCRATCH_F),
			sc2_read(hw, HEVC_SHIFT_BYTE_COUNT), job->size,
			p->p.profile_id, p->p.horizontal_size, p->p.vertical_size,
			p->p.chroma_format, p->p.sample_precision,
			p->p.encoding_precision, p->p.lcu_size,
			p->p.g_uiMaxSizeInBit, p->p.num_of_ref_cur,
			p->p.num_to_remove_cur, p->p.coding_order);
		dev_err_ratelimited(hw->dev,
			"AVS2 parser pc=%#x lmem=%#x cmd=%#x fetch=%#x header=%#x irq=%#x shift=%#x cabac=%#x level=%u rd=%#x wr=%#x\n",
			sc2_read(hw, A2_REG(0x3308)),
			sc2_read(hw, SC2_LMEM_DMA_CTRL),
			sc2_read(hw, HEVC_PARSER_CMD_STATUS),
			sc2_read(hw, HEVC_PARSER_CMD_FETCH),
			sc2_read(hw, HEVC_PARSER_HEADER_INFO),
			sc2_read(hw, HEVC_PARSER_INT_STATUS),
			sc2_read(hw, HEVC_SHIFT_STATUS),
			sc2_read(hw, HEVC_CABAC_CONTROL),
			sc2_read(hw, HEVC_STREAM_LEVEL),
			sc2_read(hw, HEVC_STREAM_RD_PTR),
			sc2_read(hw, HEVC_STREAM_WR_PTR));
	}
	return ret;
}

int meson_sc2_avs2_decode(struct meson_sc2_hevc_context *ctx,
			const struct meson_sc2_avs2_job *job,
			struct meson_sc2_avs2_result *result)
{
	struct meson_sc2_hw *hw;
	struct sc2_avs2_state *st;
	struct sc2_surface_owner *owner = NULL;
	struct meson_sc2_surface *s;
	s32 fw_status = -EIO;
	bool recovered, picture = false, started = false;
	u32 old_start_flag;
	unsigned int i;
	int ret, drain;

	if (!ctx || !job || !result || !job->data || !job->size ||
	    job->size > SC2_INPUT_SIZE - SC2_INPUT_PADDING ||
	    (!!job->output.destination != !!job->output.output_buffer))
		return -EINVAL;
	memset(result, 0, sizeof(*result));
	hw = ctx->hw;
	s = job->output.destination;
	mutex_lock(&hw->lock);
	if (hw->fatal || atomic_read(&ctx->aborted)) {
		ret = hw->fatal ? -EIO : -ECANCELED;
		goto unlock;
	}
	ret = sc2_avs2_alloc(ctx);
	if (ret)
		goto unlock;
	st = ctx->avs2;
	old_start_flag = st->start_flag;
	if (s && sc2_avs2_busy_locked(st, s)) {
		ret = -EAGAIN;
		goto unlock;
	}
	if (s) {
		owner = sc2_owner(hw, s);
		if (!owner || !s->capture_owned ||
		    owner->capture_buffer != job->output.output_buffer ||
		    owner->context != ctx || job->output.output_size < s->size ||
		    s->width > MESON_SC2_AVS2_MAX_WIDTH ||
		    s->height > MESON_SC2_AVS2_MAX_HEIGHT ||
		    !s->width || !s->height || (s->depth != 8 && s->depth != 10)) {
			ret = -EINVAL;
			goto unlock;
		}
	}
	memcpy(ctx->input, job->data, job->size);
	memset(ctx->input + job->size, 0, SC2_INPUT_PADDING);
	if (s) {
		ret = sc2_pin_output(hw, &job->output);
		if (ret)
			goto unlock;
	}
	ret = sc2_power_on(hw);
	if (ret)
		goto unpin;
	started = true;
	ret = sc2_prepare(hw);
	if (!ret) {
		ret = meson_sm_call(hw->firmware, SM_VIDEO_FW_LOAD, &fw_status,
				    27, 2, 0, 0, 0);
		if (!ret && fw_status)
			ret = -EIO;
	}
	if (!ret)
		ret = sc2_avs2_run(ctx, job, result, &picture);
	if (!ret) {
		/* Preserve firmware-updated sequence/per-picture weighting RAM
		 * before the shared engine reset or another codec can overwrite it.
		 */
		sc2_write(hw, HEVC_IQIT_SCALELUT_RD_ADDR, 0);
		for (i = 0; i < ARRAY_SIZE(st->iqit); i++)
			st->iqit[i] = sc2_read(hw, HEVC_IQIT_SCALELUT_DATA);
		st->iqit_valid = true;
	}
	drain = sc2_drain(hw, &recovered);
	if (drain) {
		sc2_quarantine(hw, drain);
		ret = -EIO;
		goto unlock;
	}
	if (!ret && (recovered || atomic_read(&ctx->aborted)))
		ret = recovered ? -EIO : -ECANCELED;
	dma_rmb();
	if (!ret && picture && (!sc2_surface_guards(s) ||
	    memchr_inv(owner->private_mv + s->mv_size, SC2_GUARD_BYTE, SC2_GUARD_SIZE)))
		ret = -EIO;
	if (!ret && picture) {
		struct avs2_frame_s *f = st->dec.hc.cur_pic;

		s->poc = st->dec.img.pic_distance;
		s->valid = true;
		owner->surface.poc = s->poc;
		owner->surface.valid = true;
		result->surface = s;
		result->timestamp = job->timestamp;
		result->poc = s->poc;
		result->hidden = st->dec.img.typeb == BACKGROUND_IMG &&
			!st->dec.hd.background_picture_output_flag;
		avs2_post_process(&st->dec);
		if (st->dec.error || f->error_mark)
			ret = st->dec.error ?: -EBADMSG;
		st->dec.hc.cur_pic = NULL;
		st->decode_count++;
	}
	sc2_power_off(hw);
 unpin:
	sc2_unpin_output(hw);
	if (ret == -EPIPE) {
		st->start_flag = old_start_flag;
	} else if (ret && started && !hw->fatal) {
		if (s) {
			s->valid = false;
			owner->surface.valid = false;
		}
		sc2_avs2_error_reset(ctx);
	}
 unlock:
	mutex_unlock(&hw->lock);
	return ret;
}

int meson_sc2_avs2_dequeue(struct meson_sc2_hevc_context *ctx, bool drain,
			 struct meson_sc2_avs2_result *result)
{
	struct sc2_avs2_state *st;
	struct avs2_frame_s *best = NULL;
	struct sc2_avs2_picture *p;
	unsigned int i;
	int ret = -EAGAIN;

	if (!ctx || !result)
		return -EINVAL;
	mutex_lock(&ctx->hw->lock);
	st = ctx->avs2;
	if (ctx->hw->fatal) {
		ret = -EIO;
		goto out;
	}
	if (!st)
		goto out;
	if (drain) {
		ret = avs2_drain_output(&st->dec);
		if (ret < 0)
			goto out;
		ret = -EAGAIN;
	}
	for (i = 0; i < AVS2_MAX_BUFFER_NUM; i++) {
		struct avs2_frame_s *f = &st->dec.frm_pool[i];

		if (f->to_prepare_disp > 0 && (!best || f->to_prepare_disp < best->to_prepare_disp))
			best = f;
	}
	if (!best)
		goto out;
	p = &st->picture[best->index];
	if (!p->valid) {
		ret = -EIO;
		goto out;
	}
	*result = (struct meson_sc2_avs2_result) {
		.surface = p->surface, .timestamp = p->timestamp,
		.width = best->pic_w, .height = best->pic_h, .depth = p->depth,
		.poc = best->imgtr_fwRefDistance_bak,
	};
	best->to_prepare_disp = 0;
	best->vf_ref++;
	ret = 0;
 out:
	mutex_unlock(&ctx->hw->lock);
	return ret;
}

int meson_sc2_avs2_put_output(struct meson_sc2_hevc_context *ctx,
			    struct meson_sc2_surface *surface)
{
	struct sc2_avs2_state *st;
	unsigned int i;
	int ret = -ENOENT;

	if (!ctx || !surface)
		return -EINVAL;
	mutex_lock(&ctx->hw->lock);
	st = ctx->avs2;
	if (ctx->hw->fatal) {
		ret = -EIO;
		goto out;
	}
	if (!st)
		goto out;
	for (i = 0; i < AVS2_MAX_BUFFER_NUM; i++)
		if (st->picture[i].surface == surface && st->picture[i].valid &&
		    st->dec.frm_pool[i].vf_ref > 0) {
			st->dec.frm_pool[i].vf_ref--;
			ret = 0;
			break;
		}
 out:
	mutex_unlock(&ctx->hw->lock);
	return ret;
}
