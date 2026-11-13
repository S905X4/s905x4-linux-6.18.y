// SPDX-License-Identifier: GPL-2.0
/* Controls-authoritative AV1 syntax. See av1_request_syntax.h for contract. */
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include "av1_request_syntax.h"

struct av1_bits {
	u8 *data;
	size_t capacity, bytes;
	unsigned int bit;
	int status;
	struct meson_av1_syntax_error *error;
};

struct av1_syntax {
	const struct meson_av1_syntax_request *r;
	const struct v4l2_ctrl_av1_frame *primary;
	unsigned int cols_log2, rows_log2;
	bool intra, lossless;
};

static void fail(struct av1_bits *b, int code, const char *reason)
{
	if (!b->status) {
		b->status = code;
		if (b->error)
			b->error->reason = reason;
	}
}

#define CHECK(b, condition, code, reason) do { \
	if (!(condition)) { fail(b, code, reason); return; } \
} while (0)
#define SF(s, name) (!!((s)->r->sequence->flags & V4L2_AV1_SEQUENCE_FLAG_##name))
#define FF(s, name) (!!((s)->r->frame->flags & V4L2_AV1_FRAME_FLAG_##name))

static void put(struct av1_bits *b, unsigned int n, u32 value)
{
	unsigned int i;

	if (b->status)
		return;
	CHECK(b, n <= 32 && (n == 32 || (value >> n) == 0),
	      -ERANGE, "fixed-width field out of range");
	for (i = n; i; i--) {
		CHECK(b, b->bytes != SIZE_MAX, -ERANGE, "output size overflow");
		if (b->data) {
			CHECK(b, b->bytes < b->capacity, -ENOSPC, "destination capacity");
			if (!b->bit)
				b->data[b->bytes] = 0;
			b->data[b->bytes] |= ((value >> (i - 1)) & 1) << (7 - b->bit);
		}
		if (++b->bit == 8) {
			b->bit = 0;
			b->bytes++;
		}
	}
}

static void signed_bits(struct av1_bits *b, unsigned int n, s32 value)
{
	CHECK(b, n && n < 32 && value >= -(1 << (n - 1)) &&
	      value < (1 << (n - 1)), -ERANGE, "signed field out of range");
	put(b, n, (u32)value & ((1U << n) - 1));
}

static void align_bits(struct av1_bits *b, bool trailing)
{
	if (trailing)
		put(b, 1, 1);
	while (b->bit && !b->status)
		put(b, 1, 0);
}

static unsigned int tile_log2(unsigned int block, unsigned int target)
{
	unsigned int n = 0;

	while ((block << n) < target)
		n++;
	return n;
}

static unsigned int dimension_bits(unsigned int maximum_minus_one)
{
	return max(1U, tile_log2(1, maximum_minus_one + 1));
}

static void ns(struct av1_bits *b, unsigned int n, unsigned int value)
{
	unsigned int w, m;

	CHECK(b, n && value < n, -ERANGE, "non-symmetric field out of range");
	if (n == 1)
		return;
	w = tile_log2(1, n);
	m = (1U << w) - n;
	if (value < m)
		put(b, w - 1, value);
	else {
		put(b, w - 1, (value + m) >> 1);
		put(b, 1, (value + m) & 1);
	}
}

static void increment(struct av1_bits *b, unsigned int value,
		      unsigned int low, unsigned int high)
{
	unsigned int i;

	CHECK(b, value >= low && value <= high, -EINVAL, "increment field out of range");
	for (i = low; i < high; i++) {
		put(b, 1, value > i);
		if (value == i)
			break;
	}
}

static void subexp(struct av1_bits *b, unsigned int n, unsigned int value)
{
	unsigned int i = 0, mk = 0, bits, a;

	CHECK(b, value < n, -ERANGE, "subexponential field out of range");
	for (;;) {
		bits = i ? 3 + i - 1 : 3;
		a = 1U << bits;
		if (n <= mk + 3 * a) {
			ns(b, n - mk, value - mk);
			return;
		}
		put(b, 1, value >= mk + a);
		if (value < mk + a) {
			put(b, bits, value - mk);
			return;
		}
		mk += a;
		i++;
	}
}

static void bytes(struct av1_bits *b, const u8 *data, size_t size)
{
	if (b->status)
		return;
	CHECK(b, !b->bit, -EINVAL, "unaligned payload");
	CHECK(b, size <= SIZE_MAX - b->bytes, -ERANGE, "output size overflow");
	if (b->data) {
		CHECK(b, b->bytes <= b->capacity && size <= b->capacity - b->bytes,
		      -ENOSPC, "destination capacity");
		memcpy(b->data + b->bytes, data, size);
	}
	b->bytes += size;
}

static void obu_prefix(struct av1_bits *b, unsigned int type, size_t size)
{
	unsigned int i;
	u64 value = size;

	CHECK(b, value <= ((1ULL << 56) - 1), -ERANGE, "OBU exceeds eight-byte leb128");
	put(b, 8, (type << 3) | 2);
	for (i = 0; i < 8; i++) {
		put(b, 8, (value & 127) | (value > 127 ? 128 : 0));
		value >>= 7;
		if (!value)
			break;
	}
}

static void sequence_header(struct av1_bits *b, struct av1_syntax *s)
{
	const struct v4l2_ctrl_av1_sequence *q = s->r->sequence;
	unsigned int wb = dimension_bits(q->max_frame_width_minus_1);
	unsigned int hb = dimension_bits(q->max_frame_height_minus_1);

	put(b, 3, 0);
	put(b, 1, SF(s, STILL_PICTURE));
	put(b, 1, 0); /* full header, including for reduced-still inputs */
	put(b, 1, 0); /* timing_info_present_flag */
	put(b, 1, 0); /* initial_display_delay_present_flag */
	put(b, 5, 0); /* one unfiltered operating point */
	put(b, 12, 0);
	put(b, 5, 31); /* unconstrained level */
	put(b, 1, 0); /* tier */
	put(b, 4, wb - 1);
	put(b, 4, hb - 1);
	put(b, wb, q->max_frame_width_minus_1);
	put(b, hb, q->max_frame_height_minus_1);
	put(b, 1, 0); /* frame_id_numbers_present_flag */
	put(b, 1, SF(s, USE_128X128_SUPERBLOCK));
	put(b, 1, SF(s, ENABLE_FILTER_INTRA));
	put(b, 1, SF(s, ENABLE_INTRA_EDGE_FILTER));
	put(b, 1, SF(s, ENABLE_INTERINTRA_COMPOUND));
	put(b, 1, SF(s, ENABLE_MASKED_COMPOUND));
	put(b, 1, SF(s, ENABLE_WARPED_MOTION));
	put(b, 1, SF(s, ENABLE_DUAL_FILTER));
	put(b, 1, SF(s, ENABLE_ORDER_HINT));
	if (SF(s, ENABLE_ORDER_HINT)) {
		put(b, 1, SF(s, ENABLE_JNT_COMP));
		put(b, 1, SF(s, ENABLE_REF_FRAME_MVS));
	}
	put(b, 1, 1); /* per-frame screen content tools */
	put(b, 1, 1); /* per-frame integer MV */
	if (SF(s, ENABLE_ORDER_HINT))
		put(b, 3, q->order_hint_bits - 1);
	put(b, 1, SF(s, ENABLE_SUPERRES));
	put(b, 1, SF(s, ENABLE_CDEF));
	put(b, 1, SF(s, ENABLE_RESTORATION));
	put(b, 1, q->bit_depth == 10);
	put(b, 1, 0); /* monochrome */
	put(b, 1, 0); /* color description unspecified */
	put(b, 1, SF(s, COLOR_RANGE));
	put(b, 2, 0); /* chroma_sample_position unspecified */
	put(b, 1, SF(s, SEPARATE_UV_DELTA_Q));
	put(b, 1, SF(s, FILM_GRAIN_PARAMS_PRESENT));
	align_bits(b, true);
}

static void tile_info(struct av1_bits *b, struct av1_syntax *s)
{
	const struct v4l2_ctrl_av1_frame *f = s->r->frame;
	const struct v4l2_av1_tile_info *t = &f->tile_info;
	unsigned int context_id = t->context_update_tile_id;
	unsigned int shift = SF(s, USE_128X128_SUPERBLOCK) ? 5 : 4;
	unsigned int mi_cols = 2 * ((f->frame_width_minus_1 + 8) >> 3);
	unsigned int mi_rows = 2 * ((f->frame_height_minus_1 + 8) >> 3);
	unsigned int cols = (mi_cols + (1U << shift) - 1) >> shift;
	unsigned int rows = (mi_rows + (1U << shift) - 1) >> shift;
	unsigned int max_width = 4096 >> (shift + 2);
	unsigned int area = (4096 * 2304) >> (2 * (shift + 2));
	unsigned int min_cols = tile_log2(max_width, cols);
	unsigned int max_cols = tile_log2(1, min(cols, 64U));
	unsigned int max_rows = tile_log2(1, min(rows, 64U));
	unsigned int min_tiles = max(min_cols, tile_log2(area, rows * cols));
	unsigned int i, pos = 0, widest = 0, width, height, min_rows;
	bool uniform = t->flags & V4L2_AV1_TILE_INFO_FLAG_UNIFORM_TILE_SPACING;

	CHECK(b, !(t->flags & ~1U), -EOPNOTSUPP, "unknown tile flags");
	CHECK(b, t->tile_cols && t->tile_cols <= 64 && t->tile_rows &&
	      t->tile_rows <= 64 && t->tile_cols * t->tile_rows <= 512,
	      -EINVAL, "tile grid out of range");
	put(b, 1, uniform);
	if (uniform) {
		for (s->cols_log2 = min_cols; s->cols_log2 <= max_cols; s->cols_log2++) {
			width = DIV_ROUND_UP(cols, 1U << s->cols_log2);
			if (DIV_ROUND_UP(cols, width) == t->tile_cols &&
			    t->width_in_sbs_minus_1[0] == width - 1)
				break;
		}
		increment(b, s->cols_log2, min_cols, max_cols);
		min_rows = min_tiles > s->cols_log2 ? min_tiles - s->cols_log2 : 0;
		for (s->rows_log2 = min_rows; s->rows_log2 <= max_rows; s->rows_log2++) {
			height = DIV_ROUND_UP(rows, 1U << s->rows_log2);
			if (DIV_ROUND_UP(rows, height) == t->tile_rows &&
			    t->height_in_sbs_minus_1[0] == height - 1)
				break;
		}
		increment(b, s->rows_log2, min_rows, max_rows);
	} else {
		s->cols_log2 = tile_log2(1, t->tile_cols);
		s->rows_log2 = tile_log2(1, t->tile_rows);
	}
	if (b->status)
		return;
	for (i = 0; i < t->tile_cols; i++) {
		CHECK(b, pos < cols && t->mi_col_starts[i] == (pos << shift),
		      -EINVAL, "tile column starts");
		CHECK(b, t->width_in_sbs_minus_1[i] < min(cols - pos, max_width),
		      -EINVAL, "tile width exceeds grid or maximum");
		width = t->width_in_sbs_minus_1[i] + 1;
		if (uniform) {
			CHECK(b, width == min(cols - pos, DIV_ROUND_UP(cols, 1U << s->cols_log2)),
			      -EINVAL, "uniform tile width mismatch");
		} else {
			ns(b, min(cols - pos, max_width), width - 1);
		}
		widest = max(widest, width);
		pos += width;
	}
	CHECK(b, pos == cols && t->mi_col_starts[t->tile_cols] == mi_cols,
	      -EINVAL, "tile columns do not cover frame");
	if (!uniform)
		area = min_tiles ? (rows * cols) >> (min_tiles + 1) : rows * cols;
	height = max(area / widest, 1U);
	pos = 0;
	for (i = 0; i < t->tile_rows; i++) {
		CHECK(b, pos < rows && t->mi_row_starts[i] == (pos << shift),
		      -EINVAL, "tile row starts");
		CHECK(b, t->height_in_sbs_minus_1[i] < rows - pos,
		      -EINVAL, "tile height exceeds grid");
		width = t->height_in_sbs_minus_1[i] + 1;
		if (uniform) {
			CHECK(b, width == min(rows - pos, DIV_ROUND_UP(rows, 1U << s->rows_log2)),
			      -EINVAL, "uniform tile height mismatch");
		} else {
			ns(b, min(rows - pos, height), width - 1);
		}
		pos += width;
	}
	CHECK(b, pos == rows && t->mi_row_starts[t->tile_rows] == mi_rows,
	      -EINVAL, "tile rows do not cover frame");
	CHECK(b, t->tile_cols * t->tile_rows <= 256 || s->r->metadata,
	      -ENODATA, "native ABI omits ninth context-update tile ID bit");
	if (s->r->metadata) {
		context_id = s->r->metadata->context_update_tile_id;
		CHECK(b, (context_id & 255) == t->context_update_tile_id,
		      -EINVAL, "wide context-update tile ID differs from native low byte");
	}
	if (s->cols_log2 || s->rows_log2) {
		CHECK(b, context_id < t->tile_cols * t->tile_rows &&
		      t->tile_size_bytes >= 1 && t->tile_size_bytes <= 4,
		      -EINVAL, "tile context or size-byte count");
		put(b, s->cols_log2 + s->rows_log2, context_id);
		put(b, 2, t->tile_size_bytes - 1);
	} else {
		CHECK(b, !context_id && !t->tile_size_bytes,
		      -EINVAL, "single tile context or size-byte count");
	}
}

static void delta_q(struct av1_bits *b, int value)
{
	put(b, 1, value != 0);
	if (value)
		signed_bits(b, 7, value);
}

static void quantization(struct av1_bits *b, struct av1_syntax *s)
{
	const struct v4l2_av1_quantization *q = &s->r->frame->quantization;
	bool diff = q->flags & V4L2_AV1_QUANTIZATION_FLAG_DIFF_UV_DELTA;

	CHECK(b, !(q->flags & ~7U), -EOPNOTSUPP, "unknown quantization flags");
	put(b, 8, q->base_q_idx);
	delta_q(b, q->delta_q_y_dc);
	if (SF(s, SEPARATE_UV_DELTA_Q))
		put(b, 1, diff);
	else
		CHECK(b, !diff, -EINVAL, "UV delta requires separate UV sequence flag");
	delta_q(b, q->delta_q_u_dc);
	delta_q(b, q->delta_q_u_ac);
	if (diff) {
		delta_q(b, q->delta_q_v_dc);
		delta_q(b, q->delta_q_v_ac);
	} else {
		CHECK(b, q->delta_q_v_dc == q->delta_q_u_dc && q->delta_q_v_ac == q->delta_q_u_ac,
		      -EINVAL, "inferred V quantizer mismatch");
	}
	put(b, 1, !!(q->flags & V4L2_AV1_QUANTIZATION_FLAG_USING_QMATRIX));
	if (q->flags & V4L2_AV1_QUANTIZATION_FLAG_USING_QMATRIX) {
		put(b, 4, q->qm_y);
		put(b, 4, q->qm_u);
		if (SF(s, SEPARATE_UV_DELTA_Q))
			put(b, 4, q->qm_v);
		else
			CHECK(b, q->qm_v == q->qm_u, -EINVAL, "inferred V qmatrix mismatch");
	}
}

static void segmentation(struct av1_bits *b, struct av1_syntax *s)
{
	const struct v4l2_av1_segmentation *g = &s->r->frame->segmentation;
	const struct v4l2_av1_quantization *q = &s->r->frame->quantization;
	static const u8 bits[8] = { 8, 6, 6, 6, 6, 3, 0, 0 };
	bool enabled = g->flags & V4L2_AV1_SEGMENTATION_FLAG_ENABLED;
	bool update = g->flags & V4L2_AV1_SEGMENTATION_FLAG_UPDATE_DATA;
	bool map = g->flags & V4L2_AV1_SEGMENTATION_FLAG_UPDATE_MAP;
	bool temporal = g->flags & V4L2_AV1_SEGMENTATION_FLAG_TEMPORAL_UPDATE;
	unsigned int i, j, last = 0;
	bool preskip = false;

	CHECK(b, !(g->flags & ~31U), -EOPNOTSUPP, "unknown segmentation flags");
	put(b, 1, enabled);
	if (enabled) {
		if (!s->primary) {
			CHECK(b, map && !temporal && update, -EINVAL, "primary-none segmentation inference");
		} else {
			put(b, 1, map);
			if (map)
				put(b, 1, temporal);
			else
				CHECK(b, !temporal, -EINVAL, "temporal segmentation without map update");
			put(b, 1, update);
		}
	}
	s->lossless = !q->delta_q_y_dc && !q->delta_q_u_dc && !q->delta_q_u_ac &&
		!q->delta_q_v_dc && !q->delta_q_v_ac;
	for (i = 0; i < 8; i++) {
		int index = q->base_q_idx;

		if (g->feature_enabled[i])
			last = i;
		if (g->feature_enabled[i] & 0xe0)
			preskip = true;
		for (j = 0; j < 8; j++) {
			bool on = g->feature_enabled[i] & (1 << j);
			int value = g->feature_data[i][j];

			CHECK(b, enabled || !on, -EINVAL, "features with segmentation disabled");
			if (enabled && update) {
				put(b, 1, on);
				if (on && bits[j]) {
					if (j < 5) {
						CHECK(b, value >= -((1 << bits[j]) - 1) && value <= (1 << bits[j]) - 1,
						      -ERANGE, "segmentation feature magnitude");
						signed_bits(b, bits[j] + 1, value);
					} else {
						put(b, bits[j], value);
					}
				}
			} else if (enabled) {
				CHECK(b, s->primary && on == !!(s->primary->segmentation.feature_enabled[i] & (1 << j)) &&
				      (!on || value == s->primary->segmentation.feature_data[i][j]),
				      -EINVAL, "inherited segmentation differs from primary reference");
			}
			CHECK(b, (on && bits[j]) || !value, -EINVAL, "inferred segmentation feature must be zero");
		}
		if (enabled && (g->feature_enabled[i] & 1))
			index += g->feature_data[i][0];
		if (clamp(index, 0, 255))
			s->lossless = false;
	}
	CHECK(b, g->last_active_seg_id == last &&
	      !!(g->flags & V4L2_AV1_SEGMENTATION_FLAG_SEG_ID_PRE_SKIP) == preskip,
	      -EINVAL, "derived segmentation state mismatch");
}

static void filters(struct av1_bits *b, struct av1_syntax *s)
{
	const struct v4l2_ctrl_av1_frame *f = s->r->frame;
	const struct v4l2_av1_quantization *q = &f->quantization;
	const struct v4l2_av1_loop_filter *lf = &f->loop_filter;
	const struct v4l2_av1_cdef *c = &f->cdef;
	const struct v4l2_av1_loop_restoration *lr = &f->loop_restoration;
	static const s8 defaults[8] = { 1, 0, 0, 0, -1, 0, -1, -1 };
	static const u8 lr_wire[4] = { 0, 2, 3, 1 };
	bool dq = q->flags & V4L2_AV1_QUANTIZATION_FLAG_DELTA_Q_PRESENT;
	bool dlf = lf->flags & V4L2_AV1_LOOP_FILTER_FLAG_DELTA_LF_PRESENT;
	bool enabled = lf->flags & V4L2_AV1_LOOP_FILTER_FLAG_DELTA_ENABLED;
	bool update = lf->flags & V4L2_AV1_LOOP_FILTER_FLAG_DELTA_UPDATE;
	bool uses_lr = false, chroma_lr = false;
	unsigned int i;

	CHECK(b, !(lf->flags & ~15U), -EOPNOTSUPP, "unknown loop filter flags");
	if (q->base_q_idx)
		put(b, 1, dq);
	else
		CHECK(b, !dq, -EINVAL, "delta Q on zero base quantizer");
	if (dq) {
		put(b, 2, q->delta_q_res);
		if (!FF(s, ALLOW_INTRABC))
			put(b, 1, dlf);
		else
			CHECK(b, !dlf, -EINVAL, "delta LF with intrabc");
	} else {
		CHECK(b, !dlf, -EINVAL, "delta LF without delta Q");
	}
	if (dlf) {
		put(b, 2, lf->delta_lf_res);
		put(b, 1, !!(lf->flags & V4L2_AV1_LOOP_FILTER_FLAG_DELTA_LF_MULTI));
	} else {
		CHECK(b, !lf->delta_lf_res && !(lf->flags & V4L2_AV1_LOOP_FILTER_FLAG_DELTA_LF_MULTI),
		      -EINVAL, "inferred delta LF state mismatch");
	}
	if (!s->lossless && !FF(s, ALLOW_INTRABC)) {
		put(b, 6, lf->level[0]);
		put(b, 6, lf->level[1]);
		if (lf->level[0] || lf->level[1]) {
			put(b, 6, lf->level[2]);
			put(b, 6, lf->level[3]);
		} else {
			CHECK(b, !lf->level[2] && !lf->level[3], -EINVAL, "inferred chroma LF levels");
		}
		put(b, 3, lf->sharpness);
		put(b, 1, enabled);
		if (enabled)
			put(b, 1, update);
	} else {
		CHECK(b, !lf->level[0] && !lf->level[1], -EINVAL, "loop filtering on lossless/intrabc");
	}
	for (i = 0; i < 10; i++) {
		int value = i < 8 ? lf->ref_deltas[i] : lf->mode_deltas[i - 8];
		int prev = i < 8 ? defaults[i] : 0;
		bool present = !s->lossless && !FF(s, ALLOW_INTRABC) && enabled;

		if (present && s->primary)
			prev = i < 8 ? s->primary->loop_filter.ref_deltas[i] :
				s->primary->loop_filter.mode_deltas[i - 8];
		if (present && update) {
			put(b, 1, value != prev);
			if (value != prev)
				signed_bits(b, 7, value);
		} else {
			CHECK(b, value == prev, -EINVAL, "inherited loop filter delta mismatch");
		}
	}
	if (!s->lossless && !FF(s, ALLOW_INTRABC) && SF(s, ENABLE_CDEF)) {
		put(b, 2, c->damping_minus_3);
		put(b, 2, c->bits);
		CHECK(b, c->bits <= 3, -EINVAL, "CDEF strength count");
		for (i = 0; i < (1U << c->bits); i++) {
			put(b, 4, c->y_pri_strength[i]);
			put(b, 2, c->y_sec_strength[i]);
			put(b, 4, c->uv_pri_strength[i]);
			put(b, 2, c->uv_sec_strength[i]);
		}
	} else {
		CHECK(b, !c->bits && !c->damping_minus_3 && !c->y_pri_strength[0] &&
		      !c->y_sec_strength[0] && !c->uv_pri_strength[0] && !c->uv_sec_strength[0],
		      -EINVAL, "inferred CDEF state mismatch");
	}
	CHECK(b, !(lr->flags & ~3U), -EOPNOTSUPP, "unknown restoration flags");
	for (i = 0; i < 3; i++) {
		CHECK(b, (unsigned int)lr->frame_restoration_type[i] < 4,
		      -EINVAL, "restoration type");
		if (lr->frame_restoration_type[i]) {
			uses_lr = true;
			if (i)
				chroma_lr = true;
		}
	}
	CHECK(b, !!(lr->flags & V4L2_AV1_LOOP_RESTORATION_FLAG_USES_LR) == uses_lr &&
	      !!(lr->flags & V4L2_AV1_LOOP_RESTORATION_FLAG_USES_CHROMA_LR) == chroma_lr,
	      -EINVAL, "derived restoration flags mismatch");
	if (!(s->lossless && f->upscaled_width == f->frame_width_minus_1 + 1) &&
	    !FF(s, ALLOW_INTRABC) && SF(s, ENABLE_RESTORATION)) {
		for (i = 0; i < 3; i++)
			put(b, 2, lr_wire[lr->frame_restoration_type[i]]);
		if (uses_lr) {
			increment(b, lr->lr_unit_shift, SF(s, USE_128X128_SUPERBLOCK) ? 1 : 0, 2);
			if (chroma_lr)
				put(b, 1, lr->lr_uv_shift);
			else
				CHECK(b, !lr->lr_uv_shift, -EINVAL, "chroma restoration shift without chroma LR");
			if (b->status)
				return;
			for (i = 0; i < 3; i++)
				CHECK(b, lr->loop_restoration_size[i] ==
				      (1U << (6 + lr->lr_unit_shift - (i ? lr->lr_uv_shift : 0))),
				      -EINVAL, "derived restoration unit size mismatch");
		}
	} else {
		CHECK(b, !uses_lr, -EINVAL, "restoration disabled by sequence/lossless/intrabc");
	}
}

static int relative_dist(unsigned int a, unsigned int z, unsigned int bits)
{
	unsigned int sign = 1U << (bits - 1);
	unsigned int diff = (a - z) & ((1U << bits) - 1);

	return (int)(diff & (sign - 1)) - (int)(diff & sign);
}

static void skip_mode(struct av1_bits *b, struct av1_syntax *s)
{
	const struct v4l2_ctrl_av1_frame *f = s->r->frame;
	unsigned int bits = s->r->sequence->order_hint_bits;
	int forward = -1, backward = -1, second = -1, i;
	unsigned int hints[7];
	bool allowed = false;

	if (!s->intra && FF(s, REFERENCE_SELECT) && SF(s, ENABLE_ORDER_HINT)) {
		for (i = 0; i < 7; i++) {
			int dist;

			hints[i] = s->r->references[f->ref_frame_idx[i]].order_hint;
			dist = relative_dist(hints[i], f->order_hint, bits);
			if (dist < 0 && (forward < 0 || relative_dist(hints[i], hints[forward], bits) > 0))
				forward = i;
			if (dist > 0 && (backward < 0 || relative_dist(hints[i], hints[backward], bits) < 0))
				backward = i;
		}
		if (forward >= 0 && backward < 0) {
			for (i = 0; i < 7; i++)
				if (relative_dist(hints[i], hints[forward], bits) < 0 &&
				    (second < 0 || relative_dist(hints[i], hints[second], bits) > 0))
					second = i;
			backward = second;
		}
		allowed = forward >= 0 && backward >= 0;
	}
	CHECK(b, FF(s, SKIP_MODE_ALLOWED) == allowed, -EINVAL, "derived skip mode allowed mismatch");
	if (allowed)
		put(b, 1, FF(s, SKIP_MODE_PRESENT));
	else
		CHECK(b, !FF(s, SKIP_MODE_PRESENT), -EINVAL, "skip mode present but not allowed");
	if (FF(s, SKIP_MODE_PRESENT))
		CHECK(b, f->skip_mode_frame[0] == min(forward, backward) + 1 &&
		      f->skip_mode_frame[1] == max(forward, backward) + 1,
		      -EINVAL, "derived skip mode reference pair mismatch");
}

static unsigned int recenter(unsigned int r, unsigned int value)
{
	if (value > 2 * r)
		return value;
	if (value >= r)
		return 2 * (value - r);
	return 2 * (r - value) - 1;
}

static void global_param(struct av1_bits *b, struct av1_syntax *s,
			 unsigned int type, unsigned int ref, unsigned int index)
{
	unsigned int abs_bits = 12, precision = 15;
	int identity = index % 3 == 2 ? 65536 : 0;
	/* Keep the true primary's predictor: the backend retains firmware GMC
	 * state with the same completed physical picture as this frame snapshot.
	 * Parser restart is not a reason to recenter against identity. */
	s32 previous = s->primary ? s->primary->global_motion.params[ref][index] : identity;
	s64 current = s->r->frame->global_motion.params[ref][index];
	s64 predictor;
	unsigned int shift, maximum, n, v, r, symbol;

	if (index < 2) {
		if (type == V4L2_AV1_WARP_MODEL_TRANSLATION) {
			abs_bits = 9 - !FF(s, ALLOW_HIGH_PRECISION_MV);
			precision = 3 - !FF(s, ALLOW_HIGH_PRECISION_MV);
		} else {
			abs_bits = 12;
			precision = 6;
		}
	}
	shift = 16 - precision;
	maximum = 1U << abs_bits;
	n = 2 * maximum + 1;
	current -= identity;
	CHECK(b, !(current & ((1U << shift) - 1)), -EINVAL, "global motion fixed-point precision");
	current >>= shift;
	/* AV1 uses an arithmetic shift BEFORE subtracting the identity offset. */
	predictor = ((s64)previous >> shift) - (identity >> shift);
	CHECK(b, current >= -(s64)maximum && current <= maximum &&
	      predictor >= -(s64)maximum && predictor <= maximum,
	      -ERANGE, "global motion parameter or predictor range");
	v = current + maximum;
	r = predictor + maximum;
	symbol = 2 * r <= n ? recenter(r, v) : recenter(n - 1 - r, n - 1 - v);
	subexp(b, n, symbol);
}

static void global_motion(struct av1_bits *b, struct av1_syntax *s)
{
	const struct v4l2_av1_global_motion *g = &s->r->frame->global_motion;
	unsigned int ref, i, type, expected;

	for (ref = 1; ref < 8; ref++) {
		type = g->type[ref];
		CHECK(b, type <= V4L2_AV1_WARP_MODEL_AFFINE, -EINVAL, "global motion type");
		expected = type ? V4L2_AV1_GLOBAL_MOTION_FLAG_IS_GLOBAL : 0;
		if (type == V4L2_AV1_WARP_MODEL_ROTZOOM)
			expected |= V4L2_AV1_GLOBAL_MOTION_FLAG_IS_ROT_ZOOM;
		if (type == V4L2_AV1_WARP_MODEL_TRANSLATION)
			expected |= V4L2_AV1_GLOBAL_MOTION_FLAG_IS_TRANSLATION;
		CHECK(b, g->flags[ref] == expected, -EINVAL, "global motion flags/type mismatch");
		if (s->intra)
			CHECK(b, !type, -EINVAL, "global motion on intra frame");
		else {
			put(b, 1, type != 0);
			if (type) {
				put(b, 1, type == V4L2_AV1_WARP_MODEL_ROTZOOM);
				if (type != V4L2_AV1_WARP_MODEL_ROTZOOM)
					put(b, 1, type == V4L2_AV1_WARP_MODEL_TRANSLATION);
			}
		}
		if (type >= V4L2_AV1_WARP_MODEL_ROTZOOM) {
			global_param(b, s, type, ref, 2);
			global_param(b, s, type, ref, 3);
			if (type == V4L2_AV1_WARP_MODEL_AFFINE) {
				global_param(b, s, type, ref, 4);
				global_param(b, s, type, ref, 5);
			} else {
				CHECK(b, (s64)g->params[ref][4] == -(s64)g->params[ref][3] &&
				      g->params[ref][5] == g->params[ref][2],
				      -EINVAL, "rotzoom inferred parameters mismatch");
			}
		} else {
			for (i = 2; i < 6; i++)
				CHECK(b, g->params[ref][i] == (i % 3 == 2 ? 65536 : 0),
				      -EINVAL, "identity/translation inferred matrix mismatch");
		}
		if (type) {
			global_param(b, s, type, ref, 0);
			global_param(b, s, type, ref, 1);
		} else {
			CHECK(b, !g->params[ref][0] && !g->params[ref][1],
			      -EINVAL, "identity global motion translation mismatch");
		}
	}
	/* invalid is a derived shear-validity result, not a wire syntax bit.
	 * Firmware derives it from the emitted resolved matrix, as AV1 requires.
	 */
}

static void grain_points(struct av1_bits *b, unsigned int count, unsigned int maximum,
			 const u8 *values, const u8 *scales)
{
	unsigned int i;

	CHECK(b, count <= maximum, -EINVAL, "film grain scaling point count");
	put(b, 4, count);
	for (i = 0; i < count; i++) {
		CHECK(b, !i || values[i] > values[i - 1], -EINVAL, "film grain points not increasing");
		put(b, 8, values[i]);
		put(b, 8, scales[i]);
	}
}

static void film_grain(struct av1_bits *b, struct av1_syntax *s)
{
	const struct v4l2_ctrl_av1_film_grain *g = s->r->film_grain;
	unsigned int i, luma, chroma;
	bool from_luma;

	if (!SF(s, FILM_GRAIN_PARAMS_PRESENT) || (!FF(s, SHOW_FRAME) && !FF(s, SHOWABLE_FRAME))) {
		CHECK(b, !g || !(g->flags & V4L2_AV1_FILM_GRAIN_FLAG_APPLY_GRAIN),
		      -EINVAL, "film grain not enabled for this frame");
		return;
	}
	CHECK(b, g, -ENODATA, "missing resolved film grain control");
	CHECK(b, !(g->flags & ~31U), -EOPNOTSUPP, "unknown film grain flags");
	put(b, 1, !!(g->flags & V4L2_AV1_FILM_GRAIN_FLAG_APPLY_GRAIN));
	if (!(g->flags & V4L2_AV1_FILM_GRAIN_FLAG_APPLY_GRAIN))
		return;
	put(b, 16, g->grain_seed);
	/* Native grain controls are resolved values. Explicit update avoids an
	 * otherwise unobservable inherited grain reference and changes no pixels.
	 */
	if (s->r->frame->frame_type == V4L2_AV1_INTER_FRAME)
		put(b, 1, 1);
	grain_points(b, g->num_y_points, 14, g->point_y_value, g->point_y_scaling);
	from_luma = g->flags & V4L2_AV1_FILM_GRAIN_FLAG_CHROMA_SCALING_FROM_LUMA;
	put(b, 1, from_luma);
	if (from_luma || !g->num_y_points) {
		CHECK(b, !g->num_cb_points && !g->num_cr_points,
		      -EINVAL, "inferred chroma grain point count mismatch");
	} else {
		grain_points(b, g->num_cb_points, 10, g->point_cb_value, g->point_cb_scaling);
		grain_points(b, g->num_cr_points, 10, g->point_cr_value, g->point_cr_scaling);
	}
	put(b, 2, g->grain_scaling_minus_8);
	put(b, 2, g->ar_coeff_lag);
	CHECK(b, g->ar_coeff_lag <= 3, -EINVAL, "film grain AR lag");
	luma = 2 * g->ar_coeff_lag * (g->ar_coeff_lag + 1);
	chroma = luma + !!g->num_y_points;
	if (g->num_y_points)
		for (i = 0; i < luma; i++)
			put(b, 8, g->ar_coeffs_y_plus_128[i]);
	if (from_luma || g->num_cb_points)
		for (i = 0; i < chroma; i++)
			put(b, 8, g->ar_coeffs_cb_plus_128[i]);
	if (from_luma || g->num_cr_points)
		for (i = 0; i < chroma; i++)
			put(b, 8, g->ar_coeffs_cr_plus_128[i]);
	put(b, 2, g->ar_coeff_shift_minus_6);
	put(b, 2, g->grain_scale_shift);
	if (g->num_cb_points) {
		put(b, 8, g->cb_mult);
		put(b, 8, g->cb_luma_mult);
		put(b, 9, g->cb_offset);
	}
	if (g->num_cr_points) {
		put(b, 8, g->cr_mult);
		put(b, 8, g->cr_luma_mult);
		put(b, 9, g->cr_offset);
	}
	put(b, 1, !!(g->flags & V4L2_AV1_FILM_GRAIN_FLAG_OVERLAP));
	put(b, 1, !!(g->flags & V4L2_AV1_FILM_GRAIN_FLAG_CLIP_TO_RESTRICTED_RANGE));
}

static void frame_size(struct av1_bits *b, struct av1_syntax *s)
{
	const struct v4l2_ctrl_av1_sequence *q = s->r->sequence;
	const struct v4l2_ctrl_av1_frame *f = s->r->frame;
	bool render = f->render_width_minus_1 + 1U != f->upscaled_width ||
		f->render_height_minus_1 != f->frame_height_minus_1;
	unsigned int i;

	/* Resolved dimensions replace size-from-reference shorthand. */
	if (!s->intra && FF(s, FRAME_SIZE_OVERRIDE) && !FF(s, ERROR_RESILIENT_MODE))
		for (i = 0; i < 7; i++)
			put(b, 1, 0); /* found_ref */
	if (FF(s, FRAME_SIZE_OVERRIDE)) {
		put(b, dimension_bits(q->max_frame_width_minus_1), f->upscaled_width - 1);
		put(b, dimension_bits(q->max_frame_height_minus_1), f->frame_height_minus_1);
	} else {
		CHECK(b, f->upscaled_width == q->max_frame_width_minus_1 + 1U &&
		      f->frame_height_minus_1 == q->max_frame_height_minus_1,
		      -EINVAL, "frame dimensions require override");
	}
	if (SF(s, ENABLE_SUPERRES))
		put(b, 1, FF(s, USE_SUPERRES));
	if (FF(s, USE_SUPERRES))
		put(b, 3, f->superres_denom - 9);
	put(b, 1, render);
	if (render) {
		put(b, 16, f->render_width_minus_1);
		put(b, 16, f->render_height_minus_1);
	}
}

static void frame_header(struct av1_bits *b, struct av1_syntax *s)
{
	const struct v4l2_ctrl_av1_frame *f = s->r->frame;
	const struct v4l2_ctrl_av1_sequence *q = s->r->sequence;
	unsigned int i;

	put(b, 1, 0); /* show_existing_frame: not a native coded request */
	put(b, 2, f->frame_type);
	put(b, 1, FF(s, SHOW_FRAME));
	if (!FF(s, SHOW_FRAME))
		put(b, 1, FF(s, SHOWABLE_FRAME));
	else
		CHECK(b, FF(s, SHOWABLE_FRAME) == (f->frame_type != V4L2_AV1_KEY_FRAME),
		      -EINVAL, "inferred showable frame mismatch");
	if (f->frame_type == V4L2_AV1_SWITCH_FRAME ||
	    (f->frame_type == V4L2_AV1_KEY_FRAME && FF(s, SHOW_FRAME))) {
		CHECK(b, FF(s, ERROR_RESILIENT_MODE), -EINVAL, "inferred error resilient mode mismatch");
	} else {
		put(b, 1, FF(s, ERROR_RESILIENT_MODE));
	}
	put(b, 1, FF(s, DISABLE_CDF_UPDATE));
	put(b, 1, FF(s, ALLOW_SCREEN_CONTENT_TOOLS));
	if (FF(s, ALLOW_SCREEN_CONTENT_TOOLS))
		put(b, 1, FF(s, FORCE_INTEGER_MV));
	else
		CHECK(b, s->intra || !FF(s, FORCE_INTEGER_MV), -EINVAL, "integer MV without screen tools");
	if (f->frame_type == V4L2_AV1_SWITCH_FRAME)
		CHECK(b, FF(s, FRAME_SIZE_OVERRIDE), -EINVAL, "switch frame requires size override");
	else
		put(b, 1, FF(s, FRAME_SIZE_OVERRIDE));
	if (SF(s, ENABLE_ORDER_HINT))
		put(b, q->order_hint_bits, f->order_hint);
	if (!s->intra && !FF(s, ERROR_RESILIENT_MODE))
		put(b, 3, f->primary_ref_frame);
	if (f->frame_type == V4L2_AV1_SWITCH_FRAME ||
	    (f->frame_type == V4L2_AV1_KEY_FRAME && FF(s, SHOW_FRAME))) {
		CHECK(b, f->refresh_frame_flags == 255, -EINVAL, "inferred refresh flags mismatch");
	} else {
		CHECK(b, f->frame_type != V4L2_AV1_INTRA_ONLY_FRAME || f->refresh_frame_flags != 255,
		      -EINVAL, "intra-only frame refreshes all references");
		put(b, 8, f->refresh_frame_flags);
	}
	if ((!s->intra || f->refresh_frame_flags != 255) &&
	    SF(s, ENABLE_ORDER_HINT) && FF(s, ERROR_RESILIENT_MODE))
		for (i = 0; i < 8; i++)
			put(b, q->order_hint_bits, s->r->references[i].order_hint);
	if (!s->intra) {
		if (SF(s, ENABLE_ORDER_HINT))
			put(b, 1, 0); /* explicit refs, no short signaling */
		for (i = 0; i < 7; i++)
			put(b, 3, f->ref_frame_idx[i]);
	}
	frame_size(b, s);
	if (s->intra) {
		CHECK(b, FF(s, FORCE_INTEGER_MV), -EINVAL, "intra frame integer MV inference");
		if (FF(s, ALLOW_SCREEN_CONTENT_TOOLS) && !FF(s, USE_SUPERRES))
			put(b, 1, FF(s, ALLOW_INTRABC));
		else
			CHECK(b, !FF(s, ALLOW_INTRABC), -EINVAL, "intrabc requires screen tools without superres");
	} else {
		CHECK(b, !FF(s, ALLOW_INTRABC), -EINVAL, "intrabc on inter frame");
		if (!FF(s, FORCE_INTEGER_MV))
			put(b, 1, FF(s, ALLOW_HIGH_PRECISION_MV));
		else
			CHECK(b, !FF(s, ALLOW_HIGH_PRECISION_MV), -EINVAL, "high precision with integer MV");
		CHECK(b, (unsigned int)f->interpolation_filter <= V4L2_AV1_INTERPOLATION_FILTER_SWITCHABLE,
		      -EINVAL, "interpolation filter");
		put(b, 1, f->interpolation_filter == V4L2_AV1_INTERPOLATION_FILTER_SWITCHABLE);
		if (f->interpolation_filter != V4L2_AV1_INTERPOLATION_FILTER_SWITCHABLE)
			put(b, 2, f->interpolation_filter);
		put(b, 1, FF(s, IS_MOTION_MODE_SWITCHABLE));
		if (!FF(s, ERROR_RESILIENT_MODE) && SF(s, ENABLE_REF_FRAME_MVS))
			put(b, 1, FF(s, USE_REF_FRAME_MVS));
		else
			CHECK(b, !FF(s, USE_REF_FRAME_MVS), -EINVAL, "reference MVs disabled");
	}
	if (!FF(s, DISABLE_CDF_UPDATE))
		put(b, 1, FF(s, DISABLE_FRAME_END_UPDATE_CDF));
	else
		CHECK(b, FF(s, DISABLE_FRAME_END_UPDATE_CDF), -EINVAL, "CDF end update inference");
	tile_info(b, s);
	quantization(b, s);
	segmentation(b, s);
	filters(b, s);
	if (s->lossless)
		CHECK(b, f->tx_mode == V4L2_AV1_TX_MODE_ONLY_4X4, -EINVAL, "lossless transform mode");
	else {
		CHECK(b, f->tx_mode == V4L2_AV1_TX_MODE_LARGEST || f->tx_mode == V4L2_AV1_TX_MODE_SELECT,
		      -EINVAL, "transform mode");
		put(b, 1, f->tx_mode == V4L2_AV1_TX_MODE_SELECT);
	}
	if (!s->intra)
		put(b, 1, FF(s, REFERENCE_SELECT));
	else
		CHECK(b, !FF(s, REFERENCE_SELECT), -EINVAL, "compound reference mode on intra frame");
	skip_mode(b, s);
	if (!s->intra && !FF(s, ERROR_RESILIENT_MODE) && SF(s, ENABLE_WARPED_MOTION))
		put(b, 1, FF(s, ALLOW_WARPED_MOTION));
	else
		CHECK(b, !FF(s, ALLOW_WARPED_MOTION), -EINVAL, "warped motion disabled");
	put(b, 1, FF(s, REDUCED_TX_SET));
	global_motion(b, s);
	film_grain(b, s);
	align_bits(b, true);
}

static void prepare(struct av1_bits *b, struct av1_syntax *s)
{
	const struct meson_av1_syntax_request *r = s->r;
	const struct v4l2_ctrl_av1_sequence *q = r->sequence;
	const struct v4l2_ctrl_av1_frame *f = r->frame;
	unsigned int i;
	bool needs_slots;

	CHECK(b, q && f, -ENODATA, "missing sequence or frame control");
	CHECK(b, !(q->flags & ~0xfffffU) && !(f->flags & ~0xfffffU),
	      -EOPNOTSUPP, "unknown sequence/frame tool flags");
	CHECK(b, !q->seq_profile && (q->bit_depth == 8 || q->bit_depth == 10) &&
	      !SF(s, MONO_CHROME) && SF(s, SUBSAMPLING_X) && SF(s, SUBSAMPLING_Y),
	      -EOPNOTSUPP, "only AV1 Main 8/10-bit 4:2:0 supported");
	CHECK(b, SF(s, ENABLE_ORDER_HINT) ? q->order_hint_bits >= 1 && q->order_hint_bits <= 8 :
	      !q->order_hint_bits, -EINVAL, "sequence order hint width");
	CHECK(b, SF(s, ENABLE_ORDER_HINT) || (!SF(s, ENABLE_JNT_COMP) && !SF(s, ENABLE_REF_FRAME_MVS)),
	      -EINVAL, "order-hint-dependent sequence tools");
	CHECK(b, (unsigned int)f->frame_type <= V4L2_AV1_SWITCH_FRAME, -EINVAL, "frame type");
	s->intra = f->frame_type == V4L2_AV1_KEY_FRAME || f->frame_type == V4L2_AV1_INTRA_ONLY_FRAME;
	CHECK(b, !SF(s, STILL_PICTURE) || (f->frame_type == V4L2_AV1_KEY_FRAME && FF(s, SHOW_FRAME)),
	      -EINVAL, "still-picture sequence requires shown key frame");
	CHECK(b, f->upscaled_width && f->upscaled_width <= q->max_frame_width_minus_1 + 1U &&
	      f->frame_width_minus_1 < 65536 && f->frame_height_minus_1 <= q->max_frame_height_minus_1,
	      -EINVAL, "frame dimensions exceed sequence");
	CHECK(b, SF(s, ENABLE_ORDER_HINT) ? f->order_hint < (1U << q->order_hint_bits) : !f->order_hint,
	      -EINVAL, "current order hint range");
	if (FF(s, USE_SUPERRES)) {
		CHECK(b, SF(s, ENABLE_SUPERRES) && f->superres_denom >= 9 && f->superres_denom <= 16,
		      -EINVAL, "superresolution denominator or sequence tool");
	} else {
		CHECK(b, f->superres_denom == 8, -EINVAL, "no-superres denominator must be eight");
	}
	CHECK(b, f->frame_width_minus_1 + 1 ==
	      (f->upscaled_width * 8 + f->superres_denom / 2) / f->superres_denom,
	      -EINVAL, "derived superresolution width mismatch");
	CHECK(b, f->primary_ref_frame <= 7, -EINVAL, "primary reference selector");
	if (s->intra || FF(s, ERROR_RESILIENT_MODE))
		CHECK(b, f->primary_ref_frame == 7, -EINVAL, "primary reference must be NONE");
	needs_slots = !s->intra || ((!s->intra || f->refresh_frame_flags != 255) &&
		SF(s, ENABLE_ORDER_HINT) && FF(s, ERROR_RESILIENT_MODE));
	CHECK(b, !needs_slots || (r->references && r->num_references == 8),
	      -ENODATA, "missing eight immutable reference slot snapshots");
	CHECK(b, !r->references || r->num_references == 8,
	      -EINVAL, "reference slot accessible count must be eight");
	if (r->references) {
		for (i = 0; i < 8; i++) {
			const struct meson_av1_syntax_reference *ref = &r->references[i];

			CHECK(b, SF(s, ENABLE_ORDER_HINT) ? ref->order_hint < (1U << q->order_hint_bits) :
			      !ref->order_hint, -EINVAL, "reference slot order hint range");
			CHECK(b, !ref->valid || ref->frame, -ENODATA, "valid reference lacks resolved frame snapshot");
			CHECK(b, !ref->valid || ref->frame->order_hint == ref->order_hint,
			      -EINVAL, "reference snapshot order hint mismatch");
		}
	}
	if (!s->intra) {
		for (i = 0; i < 7; i++) {
			const struct meson_av1_syntax_reference *ref;

			CHECK(b, f->ref_frame_idx[i] >= 0 && f->ref_frame_idx[i] < 8,
			      -EINVAL, "reference slot index");
			ref = &r->references[f->ref_frame_idx[i]];
			CHECK(b, ref->valid, -ENODATA, "referenced slot is not retained");
			CHECK(b, ref->timestamp == f->reference_frame_ts[f->ref_frame_idx[i]],
			      -EINVAL, "reference timestamp differs from retained picture");
			CHECK(b, f->order_hints[i + 1] == ref->order_hint,
			      -EINVAL, "reference-type order hint differs from slot snapshot");
		}
		if (f->primary_ref_frame != 7)
			s->primary = r->references[f->ref_frame_idx[f->primary_ref_frame]].frame;
	}
}

static void tile_group(struct av1_bits *b, struct av1_syntax *s)
{
	const struct meson_av1_syntax_request *r = s->r;
	const struct v4l2_av1_tile_info *t = &r->frame->tile_info;
	size_t i, end = 0;
	unsigned int j;

	CHECK(b, r->tiles && r->tile_data, -ENODATA, "missing tile control or accessible payload");
	CHECK(b, r->num_tiles == t->tile_cols * t->tile_rows, -EINVAL, "exact tile control count");
	if (r->num_tiles > 1)
		put(b, 1, 0); /* one group spans the whole frame */
	align_bits(b, false);
	for (i = 0; i < r->num_tiles; i++) {
		const struct v4l2_ctrl_av1_tile_group_entry *tile = &r->tiles[i];

		if (b->error)
			b->error->tile = i;
		CHECK(b, tile->tile_row == i / t->tile_cols && tile->tile_col == i % t->tile_cols,
		      -EINVAL, "tile controls not in complete raster order");
		CHECK(b, tile->tile_size && tile->tile_offset >= end &&
		      tile->tile_offset <= r->tile_data_size &&
		      tile->tile_size <= r->tile_data_size - tile->tile_offset,
		      -EINVAL, "tile span empty, overlapping, or outside accessible payload");
		end = (size_t)tile->tile_offset + tile->tile_size;
		if (i + 1 < r->num_tiles) {
			CHECK(b, t->tile_size_bytes == 4 ||
			      tile->tile_size - 1 < (1U << (8 * t->tile_size_bytes)),
			      -ERANGE, "tile size does not fit native tile-size byte count");
			for (j = 0; j < t->tile_size_bytes; j++)
				put(b, 8, ((tile->tile_size - 1) >> (8 * j)) & 255);
		}
		bytes(b, r->tile_data + tile->tile_offset, tile->tile_size);
		if (b->status)
			return;
	}
	if (b->error)
		b->error->tile = SIZE_MAX;
}

typedef void (*av1_body)(struct av1_bits *, struct av1_syntax *);

static void obu(struct av1_bits *b, struct av1_syntax *s, unsigned int type, av1_body body)
{
	struct av1_bits count = { .error = b->error };

	if (b->status)
		return;
	body(&count, s);
	if (count.status) {
		b->status = count.status;
		return;
	}
	obu_prefix(b, type, count.bytes);
	body(b, s);
}

static void stream(struct av1_bits *b, struct av1_syntax *s)
{
	prepare(b, s);
	if (b->status)
		return;
	if (s->r->emit_sequence_header)
		obu(b, s, 1, sequence_header);
	obu(b, s, 3, frame_header);
	obu(b, s, 4, tile_group);
}

int meson_av1_request_syntax(const struct meson_av1_syntax_request *request,
			     u8 *destination, size_t capacity, size_t *length,
			     struct meson_av1_syntax_error *error)
{
	struct av1_bits b = { .data = destination, .capacity = capacity, .error = error };
	struct av1_syntax s = { .r = request };

	if (length)
		*length = 0;
	if (error) {
		error->tile = SIZE_MAX;
		error->reason = NULL;
	}
	if (!request || !length || !destination) {
		fail(&b, -EINVAL, "missing request, destination, or length");
		return b.status;
	}
	stream(&b, &s);
	if (b.status)
		return b.status;
	*length = b.bytes;
	return 0;
}
