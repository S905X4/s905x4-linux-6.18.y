// SPDX-License-Identifier: GPL-2.0
/* Controls-to-Annex-B normalization; see hevc_request_syntax.h for authority. */
#include <linux/bits.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/string.h>

#include "hevc_request_syntax.h"

#define SPS(f) (!!(sps->flags & V4L2_HEVC_SPS_FLAG_##f))
#define PPS(f) (!!(pps->flags & V4L2_HEVC_PPS_FLAG_##f))
#define SL(f) (!!(sl->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_##f))
#define MAX_DPB V4L2_HEVC_DPB_ENTRIES_NUM_MAX

struct syntax_writer {
	u8 *dst;
	size_t capacity;
	size_t size;
	u8 byte;
	u8 bits;
	u8 zeros;
	int error;
};

struct syntax_rps {
	u8 before[MAX_DPB];
	u8 after[MAX_DPB];
	u8 lt[MAX_DPB];
	u8 nb;
	u8 na;
	u8 nl;
	u16 used;
};

/* All byte emission is bounded, including inserted emulation prevention. */
static void raw_byte(struct syntax_writer *w, u8 byte)
{
	if (w->error)
		return;
	if (w->size == w->capacity) {
		w->error = -ENOSPC;
		return;
	}
	w->dst[w->size++] = byte;
}

static void ebsp_byte(struct syntax_writer *w, u8 byte)
{
	if (w->zeros == 2 && byte <= 3) {
		raw_byte(w, 3);
		w->zeros = 0;
	}
	raw_byte(w, byte);
	w->zeros = byte ? 0 : w->zeros + 1;
}

static void put_bits(struct syntax_writer *w, unsigned int n, u32 value)
{
	while (n) {
		w->byte = (w->byte << 1) | ((value >> --n) & 1);
		if (++w->bits == 8) {
			ebsp_byte(w, w->byte);
			w->bits = 0;
			w->byte = 0;
		}
	}
}

static void put_flag(struct syntax_writer *w, bool value)
{
	put_bits(w, 1, value);
}

static unsigned int bit_width(u32 value)
{
	unsigned int n = 0;

	while (value) {
		n++;
		value >>= 1;
	}
	return n;
}

static void put_ue(struct syntax_writer *w, u32 value)
{
	u64 code = (u64)value + 1;
	unsigned int n = bit_width(value == U32_MAX ? U32_MAX : value + 1);

	/* U32_MAX is a legal 65-bit ue(v), not a wrapped zero codeword. */
	if (code == (1ULL << 32)) {
		put_bits(w, 32, 0);
		put_flag(w, 1);
		put_bits(w, 32, 0);
		return;
	}
	put_bits(w, n - 1, 0);
	put_bits(w, n, code);
}

static void put_se(struct syntax_writer *w, s32 value)
{
	u64 code = value <= 0 ? -(s64)value * 2 : (s64)value * 2 - 1;

	if (code > U32_MAX) {
		w->error = -ERANGE;
		return;
	}
	put_ue(w, code);
}

/* Both rbsp_trailing_bits() and slice byte_alignment() have this form. */
static void align_one(struct syntax_writer *w)
{
	put_flag(w, 1);
	while (w->bits)
		put_flag(w, 0);
}

static void start_nal(struct syntax_writer *w, u8 type, u8 temporal_id)
{
	raw_byte(w, 0);
	raw_byte(w, 0);
	raw_byte(w, 0);
	raw_byte(w, 1);
	raw_byte(w, type << 1); /* forbidden_zero_bit=0, nuh_layer_id=0 */
	raw_byte(w, temporal_id);
	w->zeros = 0;
}

static int fail(struct meson_hevc_syntax_error *error, int code,
		const char *reason)
{
	if (error)
		error->reason = reason;
	return code;
}

static void profile_tier_level(struct syntax_writer *w,
			       const struct v4l2_ctrl_hevc_sps *sps)
{
	unsigned int i;
	u8 profile = sps->bit_depth_luma_minus8 ? 2 : 1;

	put_bits(w, 2, 0); /* profile_space */
	put_flag(w, 1); /* high tier: no accidental bitrate restriction */
	put_bits(w, 5, profile);
	put_bits(w, 32, 1U << (31 - profile));
	put_flag(w, 1); /* progressive_source_flag */
	put_flag(w, 0); /* interlaced_source_flag */
	put_flag(w, 1); /* non_packed_constraint_flag */
	put_flag(w, 1); /* frame_only_constraint_flag */
	/* Main/Main10: reserved_zero_43bits and inbld_flag. */
	put_bits(w, 32, 0);
	put_bits(w, 12, 0);
	put_bits(w, 8, 186); /* level 6.2 */
	for (i = 0; i < sps->sps_max_sub_layers_minus1; i++)
		put_bits(w, 2, 0); /* sublayer profile/level absent */
	if (sps->sps_max_sub_layers_minus1)
		for (i = sps->sps_max_sub_layers_minus1; i < 8; i++)
			put_bits(w, 2, 0);
}

static void ordering(struct syntax_writer *w,
		     const struct v4l2_ctrl_hevc_sps *sps)
{
	/* Native controls carry the active/highest layer values only. */
	put_flag(w, 0); /* infer the same envelope for lower sublayers */
	put_ue(w, sps->sps_max_dec_pic_buffering_minus1);
	put_ue(w, sps->sps_max_num_reorder_pics);
	put_ue(w, sps->sps_max_latency_increase_plus1);
}

static void write_vps(struct syntax_writer *w,
		      const struct v4l2_ctrl_hevc_sps *sps)
{
	start_nal(w, 32, 1);
	put_bits(w, 4, sps->video_parameter_set_id);
	put_bits(w, 2, 3); /* base_layer_internal/available */
	put_bits(w, 6, 0); /* max_layers_minus1 */
	put_bits(w, 3, sps->sps_max_sub_layers_minus1);
	put_flag(w, 1); /* temporal_id_nesting */
	put_bits(w, 16, 0xffff);
	profile_tier_level(w, sps);
	ordering(w, sps);
	put_bits(w, 6, 0); /* max_layer_id */
	put_ue(w, 0); /* num_layer_sets_minus1 */
	put_flag(w, 0); /* timing_info_present */
	put_flag(w, 0); /* extension_flag */
	align_one(w);
}

static void scaling_lists(struct syntax_writer *w,
			  const struct v4l2_ctrl_hevc_scaling_matrix *m)
{
	unsigned int size, matrix, diagonal, x, y, n;
	const u8 *list;
	int previous, dc, delta;

	for (size = 0; size < 4; size++) {
		n = size ? 8 : 4;
		for (matrix = 0; matrix < (size == 3 ? 2 : 6); matrix++) {
			if (!size)
				list = m->scaling_list_4x4[matrix];
			else if (size == 1)
				list = m->scaling_list_8x8[matrix];
			else if (size == 2)
				list = m->scaling_list_16x16[matrix];
			else
				list = m->scaling_list_32x32[matrix];
			put_flag(w, 1); /* explicit coefficients, no hidden defaults */
			previous = 8;
			if (size > 1) {
				dc = size == 2 ? m->scaling_list_dc_coef_16x16[matrix] :
					m->scaling_list_dc_coef_32x32[matrix];
				if (!dc) {
					w->error = -EINVAL;
					return;
				}
				put_se(w, dc - 8);
				previous = dc;
			}
			/* HEVC diagonal scan: (0,0), (0,1), (1,0), ... */
			for (diagonal = 0; diagonal < 2 * n - 1; diagonal++) {
				for (x = 0; x < n; x++) {
					if (diagonal < x || diagonal - x >= n)
						continue;
					y = diagonal - x;
					delta = (int)list[y * n + x] - previous;
					if (delta < -128)
						delta += 256;
					if (delta > 127)
						delta -= 256;
					put_se(w, delta);
					previous = list[y * n + x];
				}
			}
		}
	}
}

static void write_sps(struct syntax_writer *w,
		      const struct meson_hevc_syntax_request *r)
{
	const struct v4l2_ctrl_hevc_sps *sps = r->sps;

	start_nal(w, 33, 1);
	put_bits(w, 4, sps->video_parameter_set_id);
	put_bits(w, 3, sps->sps_max_sub_layers_minus1);
	put_flag(w, 1);
	profile_tier_level(w, sps);
	put_ue(w, sps->seq_parameter_set_id);
	put_ue(w, sps->chroma_format_idc);
	put_ue(w, sps->pic_width_in_luma_samples);
	put_ue(w, sps->pic_height_in_luma_samples);
	put_flag(w, 0); /* conformance_window: external display metadata */
	put_ue(w, sps->bit_depth_luma_minus8);
	put_ue(w, sps->bit_depth_chroma_minus8);
	put_ue(w, sps->log2_max_pic_order_cnt_lsb_minus4);
	ordering(w, sps);
	put_ue(w, sps->log2_min_luma_coding_block_size_minus3);
	put_ue(w, sps->log2_diff_max_min_luma_coding_block_size);
	put_ue(w, sps->log2_min_luma_transform_block_size_minus2);
	put_ue(w, sps->log2_diff_max_min_luma_transform_block_size);
	put_ue(w, sps->max_transform_hierarchy_depth_inter);
	put_ue(w, sps->max_transform_hierarchy_depth_intra);
	put_flag(w, SPS(SCALING_LIST_ENABLED));
	if (SPS(SCALING_LIST_ENABLED))
		put_flag(w, 0); /* effective matrices belong to the mutable PPS */
	put_flag(w, SPS(AMP_ENABLED));
	put_flag(w, SPS(SAMPLE_ADAPTIVE_OFFSET));
	put_flag(w, SPS(PCM_ENABLED));
	if (SPS(PCM_ENABLED)) {
		put_bits(w, 4, sps->pcm_sample_bit_depth_luma_minus1);
		put_bits(w, 4, sps->pcm_sample_bit_depth_chroma_minus1);
		put_ue(w, sps->log2_min_pcm_luma_coding_block_size_minus3);
		put_ue(w, sps->log2_diff_max_min_pcm_luma_coding_block_size);
		put_flag(w, SPS(PCM_LOOP_FILTER_DISABLED));
	}
	put_ue(w, 0); /* num_short_term_ref_pic_sets */
	put_flag(w, SPS(LONG_TERM_REF_PICS_PRESENT));
	if (SPS(LONG_TERM_REF_PICS_PRESENT))
		put_ue(w, 0); /* num_long_term_ref_pics_sps */
	put_flag(w, SPS(SPS_TEMPORAL_MVP_ENABLED));
	put_flag(w, SPS(STRONG_INTRA_SMOOTHING_ENABLED));
	put_flag(w, 0); /* VUI */
	put_flag(w, 0); /* SPS extension */
	align_one(w);
}

static void write_pps(struct syntax_writer *w,
		      const struct meson_hevc_syntax_request *r)
{
	const struct v4l2_ctrl_hevc_pps *pps = r->pps;
	const struct v4l2_ctrl_hevc_sps *sps = r->sps;
	unsigned int i;

	start_nal(w, 34, 1);
	put_ue(w, pps->pic_parameter_set_id);
	put_ue(w, r->sps->seq_parameter_set_id);
	put_flag(w, PPS(DEPENDENT_SLICE_SEGMENT_ENABLED));
	put_flag(w, PPS(OUTPUT_FLAG_PRESENT));
	put_bits(w, 3, pps->num_extra_slice_header_bits);
	put_flag(w, PPS(SIGN_DATA_HIDING_ENABLED));
	put_flag(w, PPS(CABAC_INIT_PRESENT));
	put_ue(w, pps->num_ref_idx_l0_default_active_minus1);
	put_ue(w, pps->num_ref_idx_l1_default_active_minus1);
	put_se(w, pps->init_qp_minus26);
	put_flag(w, PPS(CONSTRAINED_INTRA_PRED));
	put_flag(w, PPS(TRANSFORM_SKIP_ENABLED));
	put_flag(w, PPS(CU_QP_DELTA_ENABLED));
	if (PPS(CU_QP_DELTA_ENABLED))
		put_ue(w, pps->diff_cu_qp_delta_depth);
	put_se(w, pps->pps_cb_qp_offset);
	put_se(w, pps->pps_cr_qp_offset);
	put_flag(w, PPS(PPS_SLICE_CHROMA_QP_OFFSETS_PRESENT));
	put_flag(w, PPS(WEIGHTED_PRED));
	put_flag(w, PPS(WEIGHTED_BIPRED));
	put_flag(w, PPS(TRANSQUANT_BYPASS_ENABLED));
	put_flag(w, PPS(TILES_ENABLED));
	put_flag(w, PPS(ENTROPY_CODING_SYNC_ENABLED));
	if (PPS(TILES_ENABLED)) {
		put_ue(w, pps->num_tile_columns_minus1);
		put_ue(w, pps->num_tile_rows_minus1);
		put_flag(w, PPS(UNIFORM_SPACING));
		if (!PPS(UNIFORM_SPACING)) {
			for (i = 0; i < pps->num_tile_columns_minus1; i++)
				put_ue(w, pps->column_width_minus1[i]);
			for (i = 0; i < pps->num_tile_rows_minus1; i++)
				put_ue(w, pps->row_height_minus1[i]);
		}
		put_flag(w, PPS(LOOP_FILTER_ACROSS_TILES_ENABLED));
	}
	put_flag(w, PPS(PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED));
	put_flag(w, PPS(DEBLOCKING_FILTER_CONTROL_PRESENT));
	if (PPS(DEBLOCKING_FILTER_CONTROL_PRESENT)) {
		put_flag(w, PPS(DEBLOCKING_FILTER_OVERRIDE_ENABLED));
		put_flag(w, PPS(PPS_DISABLE_DEBLOCKING_FILTER));
		if (!PPS(PPS_DISABLE_DEBLOCKING_FILTER)) {
			put_se(w, pps->pps_beta_offset_div2);
			put_se(w, pps->pps_tc_offset_div2);
		}
	}
	put_flag(w, SPS(SCALING_LIST_ENABLED));
	if (SPS(SCALING_LIST_ENABLED))
		scaling_lists(w, r->scaling);
	put_flag(w, 1); /* lists_modification_present: resolved lists always writable */
	put_ue(w, pps->log2_parallel_merge_level_minus2);
	put_flag(w, PPS(SLICE_SEGMENT_HEADER_EXTENSION_PRESENT));
	put_flag(w, 0); /* PPS extension */
	align_one(w);
}

static bool is_idr(u8 type)
{
	return type == 19 || type == 20;
}

static bool is_irap(u8 type)
{
	return type >= 16 && type <= 23;
}

static void sort_refs(u8 *indices, unsigned int count,
		      const struct v4l2_ctrl_hevc_decode_params *d,
		      bool descending)
{
	unsigned int i, j;
	u8 index;
	s32 poc;

	for (i = 1; i < count; i++) {
		index = indices[i];
		poc = d->dpb[index].pic_order_cnt_val;
		for (j = i; j; j--) {
			s32 prior = d->dpb[indices[j - 1]].pic_order_cnt_val;

			if (descending ? prior >= poc : prior <= poc)
				break;
			indices[j] = indices[j - 1];
		}
		indices[j] = index;
	}
}

static int prepare_rps(const struct meson_hevc_syntax_request *r,
		       struct syntax_rps *out,
		       struct meson_hevc_syntax_error *error)
{
	const struct v4l2_ctrl_hevc_decode_params *d = r->decode;
	const struct v4l2_ctrl_hevc_sps *sps = r->sps;
	const u8 *indices[3] = { d->poc_st_curr_before, d->poc_st_curr_after,
				d->poc_lt_curr };
	unsigned int counts[3] = { d->num_poc_st_curr_before,
				   d->num_poc_st_curr_after, d->num_poc_lt_curr };
	unsigned int i, j, set, index;
	s64 delta, prior;

	memset(out, 0, sizeof(*out));
	if (d->num_active_dpb_entries > MAX_DPB)
		return fail(error, -EINVAL, "decode.num_active_dpb_entries exceeds 16");
	for (set = 0; set < 3; set++) {
		if (counts[set] > MAX_DPB)
			return fail(error, -EINVAL, "decode current RPS count exceeds 16");
		for (i = 0; i < counts[set]; i++) {
			index = indices[set][i];
			if (index >= d->num_active_dpb_entries || (out->used & BIT(index)))
				return fail(error, -EINVAL, "decode.poc_* is not a unique native DPB index");
			delta = (s64)d->dpb[index].pic_order_cnt_val - d->pic_order_cnt_val;
			if ((set == 0 && delta >= 0) || (set == 1 && delta <= 0) ||
			    (!!(d->dpb[index].flags & V4L2_HEVC_DPB_ENTRY_LONG_TERM_REFERENCE) !=
			     (set == 2)))
				return fail(error, -EINVAL, "decode current RPS classification disagrees with DPB POC/flags");
			out->used |= BIT(index);
		}
	}
	for (i = 0; i < d->num_active_dpb_entries; i++) {
		if (d->dpb[i].field_pic)
			return fail(error, -EOPNOTSUPP, "DPB field pictures are not progressive Main/Main10");
		if (d->dpb[i].flags & ~V4L2_HEVC_DPB_ENTRY_LONG_TERM_REFERENCE)
			return fail(error, -EOPNOTSUPP, "unknown DPB entry flags");
		for (j = 0; j < i; j++)
			if (d->dpb[i].pic_order_cnt_val == d->dpb[j].pic_order_cnt_val)
				return fail(error, -EINVAL, "duplicate full DPB POC");
		if (d->dpb[i].flags & V4L2_HEVC_DPB_ENTRY_LONG_TERM_REFERENCE) {
			if (!SPS(LONG_TERM_REF_PICS_PRESENT))
				return fail(error, -EINVAL, "long-term DPB without SPS long-term flag");
			out->lt[out->nl++] = i;
		} else {
			delta = (s64)d->dpb[i].pic_order_cnt_val - d->pic_order_cnt_val;
			if (!delta || delta > U32_MAX || delta < -(s64)U32_MAX)
				return fail(error, -ERANGE, "short-term DPB POC delta is zero or exceeds ue(v)");
			if (delta < 0)
				out->before[out->nb++] = i;
			else
				out->after[out->na++] = i;
		}
	}
	sort_refs(out->before, out->nb, d, true);
	sort_refs(out->after, out->na, d, false);
	for (set = 0; set < 2; set++) {
		const u8 *refs = set ? out->after : out->before;
		unsigned int count = set ? out->na : out->nb;

		prior = d->pic_order_cnt_val;
		for (i = 0; i < count; i++) {
			delta = d->dpb[refs[i]].pic_order_cnt_val;
			if ((set ? delta - prior : prior - delta) > 32768)
				return fail(error, -ERANGE, "explicit short-term RPS step exceeds 32768");
			prior = delta;
		}
	}
	/* Nondecreasing delta_poc_msb_cycle_lt, including following pictures. */
	sort_refs(out->lt, out->nl, d, true);
	return 0;
}

static void write_st_rps(struct syntax_writer *w, const struct syntax_rps *rps,
			 const struct v4l2_ctrl_hevc_decode_params *d)
{
	unsigned int i, set;
	s64 prior, delta;
	const u8 *indices;
	unsigned int count;

	put_flag(w, 0); /* short_term_ref_pic_set_sps_flag; zero SPS tables */
	put_ue(w, rps->nb);
	put_ue(w, rps->na);
	for (set = 0; set < 2; set++) {
		indices = set ? rps->after : rps->before;
		count = set ? rps->na : rps->nb;
		prior = 0;
		for (i = 0; i < count; i++) {
			delta = (s64)d->dpb[indices[i]].pic_order_cnt_val - d->pic_order_cnt_val;
			put_ue(w, (set ? delta - prior : prior - delta) - 1);
			put_flag(w, rps->used & BIT(indices[i]));
			prior = delta;
		}
	}
}

static int write_lt_rps(struct syntax_writer *w, const struct syntax_rps *rps,
			const struct v4l2_ctrl_hevc_decode_params *d,
			unsigned int poc_bits,
			struct meson_hevc_syntax_error *error)
{
	u32 mask = (1U << poc_bits) - 1;
	s64 current_msb = (s64)d->pic_order_cnt_val - ((u32)d->pic_order_cnt_val & mask);
	s64 reference_msb, cycle, previous = 0;
	unsigned int i, j;
	u8 index;

	put_ue(w, rps->nl); /* num_long_term_pics; no SPS LT table */
	for (i = 0; i < rps->nl; i++) {
		index = rps->lt[i];
		put_bits(w, poc_bits, (u32)d->dpb[index].pic_order_cnt_val & mask);
		put_flag(w, rps->used & BIT(index));
		reference_msb = (s64)d->dpb[index].pic_order_cnt_val -
			((u32)d->dpb[index].pic_order_cnt_val & mask);
		cycle = (current_msb - reference_msb) / (1U << poc_bits);
		if (cycle < 0) {
			/* LSB-only syntax can express a future-MSB LT if unambiguous. */
			for (j = 0; j < d->num_active_dpb_entries; j++)
				if (j != index && (((u32)d->dpb[j].pic_order_cnt_val & mask) ==
					((u32)d->dpb[index].pic_order_cnt_val & mask)))
					return fail(error, -EOPNOTSUPP,
						"future-MSB long-term reference has ambiguous POC LSB");
			put_flag(w, 0);
		} else {
			put_flag(w, 1);
			put_ue(w, cycle - previous);
			previous = cycle;
		}
	}
	return 0;
}

static unsigned int reference_list(const struct syntax_rps *rps, bool list1,
				   u8 *list)
{
	const u8 *groups[3] = { list1 ? rps->after : rps->before,
				list1 ? rps->before : rps->after, rps->lt };
	unsigned int counts[3] = { list1 ? rps->na : rps->nb,
				  list1 ? rps->nb : rps->na, rps->nl };
	unsigned int group, i, count = 0;

	for (group = 0; group < 3; group++)
		for (i = 0; i < counts[group]; i++)
			if (rps->used & BIT(groups[group][i]))
				list[count++] = groups[group][i];
	return count;
}

static int write_list(struct syntax_writer *w, const struct syntax_rps *rps,
		      const u8 *resolved, unsigned int active, bool list1,
		      struct meson_hevc_syntax_error *error)
{
	u8 list[MAX_DPB];
	unsigned int count = reference_list(rps, list1, list);
	unsigned int i, j;

	if (!count)
		return fail(error, -EINVAL, "inter slice has empty current RPS");
	if (count > 1)
		put_flag(w, 1); /* explicit list_modification_flag_lX */
	for (i = 0; i < active; i++) {
		for (j = 0; j < count; j++)
			if (list[j] == resolved[i])
				break;
		if (j == count)
			return fail(error, -EINVAL, "slice.ref_idx_lX is not in decode current RPS");
		if (count > 1)
			put_bits(w, bit_width(count - 1), j);
	}
	return 0;
}

/* This reader consumes only the original prefix fields absent from the ABI.
 * It never parses or reuses the original RPS, weights or CABAC header layout.
 */
struct prefix_reader {
	const u8 *data;
	size_t pos;
	size_t end;
	u8 byte;
	u8 left;
	u8 zeros;
	int error;
};

static u32 read_bits(struct prefix_reader *r, unsigned int count)
{
	u32 value = 0;

	while (count--) {
		if (!r->left) {
			if (r->pos == r->end) {
				r->error = -EINVAL;
				return 0;
			}
			r->byte = r->data[r->pos++];
			if (r->zeros == 2 && r->byte == 3) {
				if (r->pos == r->end || r->data[r->pos] > 3) {
					r->error = -EINVAL;
					return 0;
				}
				r->zeros = 0;
				r->byte = r->data[r->pos++];
			}
			r->zeros = r->byte ? 0 : r->zeros + 1;
			r->left = 8;
		}
		value = (value << 1) | ((r->byte >> --r->left) & 1);
	}
	return value;
}

static u32 read_ue(struct prefix_reader *r)
{
	unsigned int zeros = 0;

	while (!read_bits(r, 1)) {
		if (r->error || ++zeros > 31) {
			r->error = -EINVAL;
			return 0;
		}
	}
	return ((1U << zeros) - 1) + read_bits(r, zeros);
}

static int original_prefix(const struct meson_hevc_syntax_request *request,
			   const struct meson_hevc_syntax_slice *slice,
			   unsigned int address_bits, u8 *output, u8 *extra,
			   struct meson_hevc_syntax_error *error)
{
	const struct v4l2_ctrl_hevc_slice_params *sl = slice->params;
	const struct v4l2_ctrl_hevc_pps *pps = request->pps;
	struct prefix_reader reader = { .data = slice->data };
	size_t prefix = 0, end;
	bool first, dependent = false;
	u32 address = 0;

	if (!slice->data)
		return fail(error, -ENODATA, "original slice NAL absent");
	if (!sl->bit_size || (sl->bit_size & 7))
		return fail(error, -EINVAL, "slice.bit_size is not a whole nonempty NAL");
	end = sl->bit_size / 8;
	if (end > slice->size || sl->data_byte_offset >= end)
		return fail(error, -EINVAL, "slice data_byte_offset/bit_size exceeds original NAL span");
	if (end >= 4 && !slice->data[0] && !slice->data[1]) {
		if (slice->data[2] == 1)
			prefix = 3;
		else if (end >= 5 && !slice->data[2] && slice->data[3] == 1)
			prefix = 4;
	}
	if (sl->data_byte_offset <= prefix + 2)
		return fail(error, -EINVAL, "slice payload offset overlaps NAL header");
	if (slice->data[prefix] != (sl->nal_unit_type << 1) ||
	    slice->data[prefix + 1] != sl->nuh_temporal_id_plus1)
		return fail(error, -EINVAL, "original NAL type/layer/temporal ID disagrees with controls");
	reader.pos = prefix + 2;
	reader.end = sl->data_byte_offset;
	first = read_bits(&reader, 1);
	if (is_irap(sl->nal_unit_type) &&
	    read_bits(&reader, 1) !=
	    !!(request->decode->flags & V4L2_HEVC_DECODE_PARAM_FLAG_NO_OUTPUT_OF_PRIOR))
		return fail(error, -EINVAL, "original no_output_of_prior_pics disagrees with decode control");
	if (read_ue(&reader) != pps->pic_parameter_set_id)
		return fail(error, -EINVAL, "original slice PPS ID disagrees with control");
	if (!first) {
		if (PPS(DEPENDENT_SLICE_SEGMENT_ENABLED))
			dependent = read_bits(&reader, 1);
		address = read_bits(&reader, address_bits);
	}
	if (first != !sl->slice_segment_addr || address != sl->slice_segment_addr ||
	    dependent != SL(DEPENDENT_SLICE_SEGMENT))
		return fail(error, -EINVAL, "original first/dependent/address disagrees with slice control");
	*extra = 0;
	*output = 1;
	if (!dependent) {
		*extra = read_bits(&reader, pps->num_extra_slice_header_bits);
		if (read_ue(&reader) != sl->slice_type)
			return fail(error, -EINVAL, "original slice_type disagrees with slice control");
		if (PPS(OUTPUT_FLAG_PRESENT))
			*output = read_bits(&reader, 1);
	}
	if (reader.error)
		return fail(error, reader.error, "original prefix truncated or malformed EPB/Exp-Golomb");
	return 0;
}

static int weight_list(struct syntax_writer *w,
		       const s8 *luma_weight, const s8 *luma_offset,
		       const s8 chroma_weight[][2], const s8 chroma_offset[][2],
		       unsigned int count, unsigned int chroma_denom,
		       unsigned int *total_flags,
		       struct meson_hevc_syntax_error *error)
{
	unsigned int i, component;
	u16 luma = 0, chroma = 0;
	int weight, delta;

	for (i = 0; i < count; i++) {
		if (luma_weight[i] || luma_offset[i]) {
			luma |= BIT(i);
			(*total_flags)++;
		}
		put_flag(w, luma & BIT(i));
	}
	for (i = 0; i < count; i++) {
		if (chroma_weight[i][0] || chroma_weight[i][1] ||
		    chroma_offset[i][0] || chroma_offset[i][1]) {
			chroma |= BIT(i);
			*total_flags += 2;
		}
		put_flag(w, chroma & BIT(i));
	}
	if (*total_flags > 24)
		return fail(error, -EINVAL, "weighted prediction exceeds 24 signalled weight flags");
	for (i = 0; i < count; i++) {
		if (luma & BIT(i)) {
			put_se(w, luma_weight[i]);
			put_se(w, luma_offset[i]);
		}
		if (chroma & BIT(i)) {
			for (component = 0; component < 2; component++) {
				weight = (1 << chroma_denom) + chroma_weight[i][component];
				/* Invert the resolved, clipped ChromaOffsetLx. Selecting
				 * this preimage preserves the exact resolved offset.
				 */
				delta = chroma_offset[i][component] +
					((128 * weight) >> chroma_denom) - 128;
				delta = clamp(delta, -512, 511);
				if (clamp(delta - ((128 * weight) >> chroma_denom) + 128,
					  -128, 127) != chroma_offset[i][component])
					return fail(error, -EINVAL, "resolved chroma offset has no legal delta_chroma_offset preimage");
				put_se(w, chroma_weight[i][component]);
				put_se(w, delta);
			}
		}
	}
	return 0;
}

static int write_weights(struct syntax_writer *w,
			 const struct v4l2_ctrl_hevc_slice_params *sl,
			 struct meson_hevc_syntax_error *error)
{
	const struct v4l2_hevc_pred_weight_table *p = &sl->pred_weight_table;
	int chroma_denom = p->luma_log2_weight_denom + p->delta_chroma_log2_weight_denom;
	unsigned int total = 0;
	int ret;

	if (p->luma_log2_weight_denom > 7 || chroma_denom < 0 || chroma_denom > 7)
		return fail(error, -EINVAL, "weighted prediction denominator outside 0..7");
	put_ue(w, p->luma_log2_weight_denom);
	put_se(w, p->delta_chroma_log2_weight_denom);
	ret = weight_list(w, p->delta_luma_weight_l0, p->luma_offset_l0,
			  p->delta_chroma_weight_l0, p->chroma_offset_l0,
			  sl->num_ref_idx_l0_active_minus1 + 1, chroma_denom,
			  &total, error);
	if (ret || sl->slice_type != V4L2_HEVC_SLICE_TYPE_B)
		return ret;
	return weight_list(w, p->delta_luma_weight_l1, p->luma_offset_l1,
			   p->delta_chroma_weight_l1, p->chroma_offset_l1,
			   sl->num_ref_idx_l1_active_minus1 + 1, chroma_denom,
			   &total, error);
}

static int write_slice(struct syntax_writer *w,
		       const struct meson_hevc_syntax_request *r,
		       const struct meson_hevc_syntax_slice *slice,
		       const struct syntax_rps *rps, unsigned int address_bits,
		       struct meson_hevc_syntax_error *error)
{
	const struct v4l2_ctrl_hevc_sps *sps = r->sps;
	const struct v4l2_ctrl_hevc_pps *pps = r->pps;
	const struct v4l2_ctrl_hevc_slice_params *sl = slice->params;
	u8 output, extra;
	unsigned int i, bits = 0, collocated_count;
	size_t payload_size, entry_sum = 0;
	bool disabled = SL(SLICE_DEBLOCKING_FILTER_DISABLED);
	int ret;

	ret = original_prefix(r, slice, address_bits, &output, &extra, error);
	if (ret)
		return ret;
	payload_size = sl->bit_size / 8 - sl->data_byte_offset;
	if (sl->num_entry_point_offsets != slice->num_entry_point_offsets)
		return fail(error, -EINVAL, "slice entry-point count disagrees with supplied control span");
	if (sl->num_entry_point_offsets && !slice->entry_point_offsets)
		return fail(error, -ENODATA, "entry-point offset control absent");
	if (sl->num_entry_point_offsets &&
	    !PPS(TILES_ENABLED) && !PPS(ENTROPY_CODING_SYNC_ENABLED))
		return fail(error, -EINVAL, "entry points without tiles or WPP");
	for (i = 0; i < sl->num_entry_point_offsets; i++) {
		u32 bytes = slice->entry_point_offsets[i];

		if (!bytes || bytes >= payload_size - entry_sum)
			return fail(error, -EINVAL, "entry-point byte lengths exceed preserved CABAC payload");
		entry_sum += bytes;
		bits = max(bits, bit_width(bytes - 1));
	}
	start_nal(w, sl->nal_unit_type, sl->nuh_temporal_id_plus1);
	put_flag(w, !sl->slice_segment_addr);
	if (is_irap(sl->nal_unit_type))
		put_flag(w, r->decode->flags & V4L2_HEVC_DECODE_PARAM_FLAG_NO_OUTPUT_OF_PRIOR);
	put_ue(w, pps->pic_parameter_set_id);
	if (sl->slice_segment_addr) {
		if (PPS(DEPENDENT_SLICE_SEGMENT_ENABLED))
			put_flag(w, SL(DEPENDENT_SLICE_SEGMENT));
		put_bits(w, address_bits, sl->slice_segment_addr);
	}
	if (!SL(DEPENDENT_SLICE_SEGMENT)) {
		put_bits(w, pps->num_extra_slice_header_bits, extra);
		put_ue(w, sl->slice_type);
		if (PPS(OUTPUT_FLAG_PRESENT))
			put_flag(w, output);
		if (!is_idr(sl->nal_unit_type)) {
			put_bits(w, sps->log2_max_pic_order_cnt_lsb_minus4 + 4,
				 (u32)sl->slice_pic_order_cnt);
			write_st_rps(w, rps, r->decode);
			if (SPS(LONG_TERM_REF_PICS_PRESENT)) {
				ret = write_lt_rps(w, rps, r->decode,
						   sps->log2_max_pic_order_cnt_lsb_minus4 + 4,
						   error);
				if (ret)
					return ret;
			}
			if (SPS(SPS_TEMPORAL_MVP_ENABLED))
				put_flag(w, SL(SLICE_TEMPORAL_MVP_ENABLED));
		}
		if (SPS(SAMPLE_ADAPTIVE_OFFSET)) {
			put_flag(w, SL(SLICE_SAO_LUMA));
			put_flag(w, SL(SLICE_SAO_CHROMA));
		}
		if (sl->slice_type != V4L2_HEVC_SLICE_TYPE_I) {
			put_flag(w, 1); /* active-reference override */
			put_ue(w, sl->num_ref_idx_l0_active_minus1);
			if (sl->slice_type == V4L2_HEVC_SLICE_TYPE_B)
				put_ue(w, sl->num_ref_idx_l1_active_minus1);
			ret = write_list(w, rps, sl->ref_idx_l0,
					 sl->num_ref_idx_l0_active_minus1 + 1, false, error);
			if (ret)
				return ret;
			if (sl->slice_type == V4L2_HEVC_SLICE_TYPE_B) {
				ret = write_list(w, rps, sl->ref_idx_l1,
						 sl->num_ref_idx_l1_active_minus1 + 1, true, error);
				if (ret)
					return ret;
				put_flag(w, SL(MVD_L1_ZERO));
			}
			if (PPS(CABAC_INIT_PRESENT))
				put_flag(w, SL(CABAC_INIT));
			if (SL(SLICE_TEMPORAL_MVP_ENABLED)) {
				if (sl->slice_type == V4L2_HEVC_SLICE_TYPE_B)
					put_flag(w, SL(COLLOCATED_FROM_L0));
				collocated_count = (sl->slice_type != V4L2_HEVC_SLICE_TYPE_B ||
					SL(COLLOCATED_FROM_L0)) ? sl->num_ref_idx_l0_active_minus1 + 1 :
					sl->num_ref_idx_l1_active_minus1 + 1;
				if (sl->collocated_ref_idx >= collocated_count)
					return fail(error, -EINVAL, "collocated_ref_idx exceeds selected resolved list");
				if (collocated_count > 1)
					put_ue(w, sl->collocated_ref_idx);
			}
			if ((PPS(WEIGHTED_PRED) && sl->slice_type == V4L2_HEVC_SLICE_TYPE_P) ||
			    (PPS(WEIGHTED_BIPRED) && sl->slice_type == V4L2_HEVC_SLICE_TYPE_B)) {
				ret = write_weights(w, sl, error);
				if (ret)
					return ret;
			}
			put_ue(w, sl->five_minus_max_num_merge_cand);
		}
		put_se(w, sl->slice_qp_delta);
		if (PPS(PPS_SLICE_CHROMA_QP_OFFSETS_PRESENT)) {
			put_se(w, sl->slice_cb_qp_offset);
			put_se(w, sl->slice_cr_qp_offset);
		}
		if (PPS(DEBLOCKING_FILTER_CONTROL_PRESENT) &&
		    PPS(DEBLOCKING_FILTER_OVERRIDE_ENABLED)) {
			put_flag(w, 1); /* override with resolved slice values */
			put_flag(w, disabled);
			if (!disabled) {
				put_se(w, sl->slice_beta_offset_div2);
				put_se(w, sl->slice_tc_offset_div2);
			}
		}
		if (PPS(PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED) &&
		    (SL(SLICE_SAO_LUMA) || SL(SLICE_SAO_CHROMA) || !disabled))
			put_flag(w, SL(SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED));
	}
	if (PPS(TILES_ENABLED) || PPS(ENTROPY_CODING_SYNC_ENABLED)) {
		put_ue(w, sl->num_entry_point_offsets);
		if (sl->num_entry_point_offsets) {
			bits = max(bits, 1U);
			put_ue(w, bits - 1);
			for (i = 0; i < sl->num_entry_point_offsets; i++)
				put_bits(w, bits, slice->entry_point_offsets[i] - 1);
		}
	}
	if (PPS(SLICE_SEGMENT_HEADER_EXTENSION_PRESENT))
		put_ue(w, 0); /* reserved extension, no Main/Main10 decoding tools */
	align_one(w);
	/* Alignment ends in a nonzero byte. Neither the old nor new header
	 * can create/remove an EPB across the CABAC boundary. Preserve the
	 * already escaped payload verbatim, including entry points and tail.
	 */
	if (w->error)
		return fail(error, w->error, "destination capacity exhausted in slice header");
	if (payload_size > w->capacity - w->size)
		return fail(error, -ENOSPC, "destination capacity exhausted in preserved CABAC payload");
	memcpy(w->dst + w->size, slice->data + sl->data_byte_offset, payload_size);
	w->size += payload_size;
	return 0;
}

static int validate_parameters(const struct meson_hevc_syntax_request *r,
			       unsigned int *picture_ctbs,
			       struct meson_hevc_syntax_error *error)
{
	const struct v4l2_ctrl_hevc_sps *sps = r->sps;
	const struct v4l2_ctrl_hevc_pps *pps = r->pps;
	unsigned int min_cb, ctb, min_tb, max_tb, width, height, sum, i;

	if (sps->chroma_format_idc != 1 || SPS(SEPARATE_COLOUR_PLANE) ||
	    (sps->bit_depth_luma_minus8 != 0 && sps->bit_depth_luma_minus8 != 2) ||
	    sps->bit_depth_luma_minus8 != sps->bit_depth_chroma_minus8)
		return fail(error, -EOPNOTSUPP, "only single-layer progressive Main/Main10 4:2:0 is representable here");
	if (sps->flags & ~GENMASK_ULL(8, 0))
		return fail(error, -EOPNOTSUPP, "unknown SPS coding tool flags");
	if (pps->flags & ~GENMASK_ULL(20, 0))
		return fail(error, -EOPNOTSUPP, "unknown PPS coding tool flags");
	if (!sps->pic_width_in_luma_samples || !sps->pic_height_in_luma_samples ||
	    sps->video_parameter_set_id > 15 || sps->seq_parameter_set_id > 15 ||
	    sps->sps_max_sub_layers_minus1 > 6 ||
	    sps->log2_max_pic_order_cnt_lsb_minus4 > 12 ||
	    sps->sps_max_dec_pic_buffering_minus1 > 15 ||
	    sps->sps_max_num_reorder_pics > sps->sps_max_dec_pic_buffering_minus1)
		return fail(error, -EINVAL, "SPS dimensions/IDs/temporal layers/POC/DPB envelope outside HEVC range");
	if (SPS(SCALING_LIST_ENABLED) && !r->scaling)
		return fail(error, -ENODATA, "resolved scaling matrix control absent while scaling enabled");
	if (sps->log2_min_luma_coding_block_size_minus3 > 3 ||
	    sps->log2_diff_max_min_luma_coding_block_size > 3 ||
	    sps->log2_min_luma_transform_block_size_minus2 > 3 ||
	    sps->log2_diff_max_min_luma_transform_block_size > 3)
		return fail(error, -EINVAL, "SPS coding/transform block logarithm outside HEVC range");
	min_cb = sps->log2_min_luma_coding_block_size_minus3 + 3;
	ctb = min_cb + sps->log2_diff_max_min_luma_coding_block_size;
	min_tb = sps->log2_min_luma_transform_block_size_minus2 + 2;
	max_tb = min_tb + sps->log2_diff_max_min_luma_transform_block_size;
	if (ctb < 4 || ctb > 6 || min_tb >= min_cb || max_tb > min(ctb, 5U) ||
	    sps->max_transform_hierarchy_depth_inter > ctb - min_tb ||
	    sps->max_transform_hierarchy_depth_intra > ctb - min_tb)
		return fail(error, -EINVAL, "inconsistent SPS coding/transform geometry or hierarchy depth");
	if ((sps->pic_width_in_luma_samples & ((1U << min_cb) - 1)) ||
	    (sps->pic_height_in_luma_samples & ((1U << min_cb) - 1)))
		return fail(error, -EINVAL, "coded dimensions are not minimum-CB aligned");
	if (SPS(PCM_ENABLED) &&
	    (sps->pcm_sample_bit_depth_luma_minus1 > sps->bit_depth_luma_minus8 + 7 ||
	     sps->pcm_sample_bit_depth_chroma_minus1 > sps->bit_depth_chroma_minus8 + 7 ||
	     sps->log2_min_pcm_luma_coding_block_size_minus3 > 2 ||
	     sps->log2_diff_max_min_pcm_luma_coding_block_size > 2 ||
	     sps->log2_min_pcm_luma_coding_block_size_minus3 + 3 < min_cb ||
	     sps->log2_min_pcm_luma_coding_block_size_minus3 + 3 +
		     sps->log2_diff_max_min_pcm_luma_coding_block_size > min(ctb, 5U)))
		return fail(error, -EINVAL, "PCM depth/block geometry inconsistent with SPS");
	if (pps->pic_parameter_set_id > 63 || pps->num_extra_slice_header_bits > 7 ||
	    pps->num_ref_idx_l0_default_active_minus1 > 14 ||
	    pps->num_ref_idx_l1_default_active_minus1 > 14 ||
	    pps->init_qp_minus26 < -(26 + 6 * sps->bit_depth_luma_minus8) ||
	    pps->init_qp_minus26 > 25 ||
	    pps->pps_cb_qp_offset < -12 || pps->pps_cb_qp_offset > 12 ||
	    pps->pps_cr_qp_offset < -12 || pps->pps_cr_qp_offset > 12 ||
	    pps->pps_beta_offset_div2 < -6 || pps->pps_beta_offset_div2 > 6 ||
	    pps->pps_tc_offset_div2 < -6 || pps->pps_tc_offset_div2 > 6 ||
	    pps->log2_parallel_merge_level_minus2 > ctb - 2 ||
	    (PPS(CU_QP_DELTA_ENABLED) && pps->diff_cu_qp_delta_depth > ctb - min_cb))
		return fail(error, -EINVAL, "PPS IDs/reference counts/QP/deblock/merge geometry outside HEVC range");
	if (!PPS(DEBLOCKING_FILTER_CONTROL_PRESENT) &&
	    (PPS(DEBLOCKING_FILTER_OVERRIDE_ENABLED) || PPS(PPS_DISABLE_DEBLOCKING_FILTER) ||
	     pps->pps_beta_offset_div2 || pps->pps_tc_offset_div2))
		return fail(error, -EINVAL, "PPS deblock values contradict absent deblock control");
	width = DIV_ROUND_UP(sps->pic_width_in_luma_samples, 1U << ctb);
	height = DIV_ROUND_UP(sps->pic_height_in_luma_samples, 1U << ctb);
	*picture_ctbs = width * height;
	if (PPS(TILES_ENABLED)) {
		if (pps->num_tile_columns_minus1 >= 20 || pps->num_tile_rows_minus1 >= 22 ||
		    pps->num_tile_columns_minus1 >= width || pps->num_tile_rows_minus1 >= height)
			return fail(error, -EINVAL, "tile count exceeds control arrays or picture CTB geometry");
		if (!PPS(UNIFORM_SPACING)) {
			sum = 0;
			for (i = 0; i < pps->num_tile_columns_minus1; i++)
				sum += pps->column_width_minus1[i] + 1;
			if (sum >= width)
				return fail(error, -EINVAL, "explicit tile columns leave no final column");
			sum = 0;
			for (i = 0; i < pps->num_tile_rows_minus1; i++)
				sum += pps->row_height_minus1[i] + 1;
			if (sum >= height)
				return fail(error, -EINVAL, "explicit tile rows leave no final row");
		}
	}
	if (r->decode->flags & ~(V4L2_HEVC_DECODE_PARAM_FLAG_IRAP_PIC |
			       V4L2_HEVC_DECODE_PARAM_FLAG_IDR_PIC |
			       V4L2_HEVC_DECODE_PARAM_FLAG_NO_OUTPUT_OF_PRIOR))
		return fail(error, -EOPNOTSUPP, "unknown decode control flags");
	return 0;
}

static int validate_slice(const struct meson_hevc_syntax_request *r,
			  const struct v4l2_ctrl_hevc_slice_params *sl,
			  unsigned int picture_ctbs,
			  struct meson_hevc_syntax_error *error)
{
	const struct v4l2_ctrl_hevc_sps *sps = r->sps;
	const struct v4l2_ctrl_hevc_pps *pps = r->pps;
	bool disabled = SL(SLICE_DEBLOCKING_FILTER_DISABLED);
	bool inferred_disabled = PPS(DEBLOCKING_FILTER_CONTROL_PRESENT) &&
		PPS(PPS_DISABLE_DEBLOCKING_FILTER);
	int qp = pps->init_qp_minus26 + 26 + sl->slice_qp_delta;

	if ((sl->flags & ~GENMASK_ULL(9, 0)) || SL(USE_INTEGER_MV) ||
	    sl->slice_act_y_qp_offset || sl->slice_act_cb_qp_offset || sl->slice_act_cr_qp_offset)
		return fail(error, -EOPNOTSUPP, "slice SCC/unknown coding tools are not represented by Main/Main10 controls");
	if (sl->colour_plane_id || (sl->pic_struct != 0 && sl->pic_struct != 7 && sl->pic_struct != 8))
		return fail(error, -EOPNOTSUPP, "separate-colour-plane or field picture syntax");
	if (sl->nal_unit_type > 21 || (sl->nal_unit_type > 9 && sl->nal_unit_type < 16) ||
	    !sl->nuh_temporal_id_plus1 ||
	    sl->nuh_temporal_id_plus1 > sps->sps_max_sub_layers_minus1 + 1 ||
	    sl->slice_type > V4L2_HEVC_SLICE_TYPE_I ||
	    sl->slice_segment_addr >= picture_ctbs ||
	    sl->num_ref_idx_l0_active_minus1 > 14 || sl->num_ref_idx_l1_active_minus1 > 14 ||
	    sl->five_minus_max_num_merge_cand > 4)
		return fail(error, -EINVAL, "slice NAL/type/temporal/reference/address/merge value outside HEVC range");
	if (sl->slice_pic_order_cnt != r->decode->pic_order_cnt_val ||
	    (is_idr(sl->nal_unit_type) && sl->slice_pic_order_cnt) ||
	    is_idr(sl->nal_unit_type) != !!(r->decode->flags & V4L2_HEVC_DECODE_PARAM_FLAG_IDR_PIC) ||
	    is_irap(sl->nal_unit_type) != !!(r->decode->flags & V4L2_HEVC_DECODE_PARAM_FLAG_IRAP_PIC) ||
	    (is_irap(sl->nal_unit_type) && sl->slice_type != V4L2_HEVC_SLICE_TYPE_I))
		return fail(error, -EINVAL, "slice POC/IDR/IRAP state disagrees with decode control");
	if (is_idr(sl->nal_unit_type) && r->decode->num_active_dpb_entries)
		return fail(error, -EINVAL, "IDR cannot retain supplied DPB references");
	if (SL(DEPENDENT_SLICE_SEGMENT) &&
	    (!sl->slice_segment_addr || !PPS(DEPENDENT_SLICE_SEGMENT_ENABLED)))
		return fail(error, -EINVAL, "dependent slice has no independent predecessor geometry/PPS permission");
	if ((!SPS(SAMPLE_ADAPTIVE_OFFSET) && (SL(SLICE_SAO_LUMA) || SL(SLICE_SAO_CHROMA))) ||
	    (SL(SLICE_TEMPORAL_MVP_ENABLED) &&
	     (!SPS(SPS_TEMPORAL_MVP_ENABLED) || is_idr(sl->nal_unit_type))) ||
	    (!PPS(CABAC_INIT_PRESENT) && SL(CABAC_INIT)))
		return fail(error, -EINVAL, "slice SAO/TMVP/CABAC flag contradicts parameter controls");
	if (qp < -6 * sps->bit_depth_luma_minus8 || qp > 51 ||
	    sl->slice_cb_qp_offset < -12 || sl->slice_cb_qp_offset > 12 ||
	    sl->slice_cr_qp_offset < -12 || sl->slice_cr_qp_offset > 12 ||
	    sl->slice_cb_qp_offset + pps->pps_cb_qp_offset < -12 ||
	    sl->slice_cb_qp_offset + pps->pps_cb_qp_offset > 12 ||
	    sl->slice_cr_qp_offset + pps->pps_cr_qp_offset < -12 ||
	    sl->slice_cr_qp_offset + pps->pps_cr_qp_offset > 12 ||
	    (!PPS(PPS_SLICE_CHROMA_QP_OFFSETS_PRESENT) &&
	     (sl->slice_cb_qp_offset || sl->slice_cr_qp_offset)))
		return fail(error, -EINVAL, "slice QP/chroma offset outside legal or inferred range");
	if (sl->slice_beta_offset_div2 < -6 || sl->slice_beta_offset_div2 > 6 ||
	    sl->slice_tc_offset_div2 < -6 || sl->slice_tc_offset_div2 > 6)
		return fail(error, -EINVAL, "slice deblock offsets outside -6..6");
	if (!PPS(DEBLOCKING_FILTER_CONTROL_PRESENT) || !PPS(DEBLOCKING_FILTER_OVERRIDE_ENABLED)) {
		if (disabled != inferred_disabled || (!disabled &&
		    (sl->slice_beta_offset_div2 != pps->pps_beta_offset_div2 ||
		     sl->slice_tc_offset_div2 != pps->pps_tc_offset_div2)))
			return fail(error, -EINVAL, "resolved slice deblocking contradicts inferred PPS values");
	}
	if ((!PPS(PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED) &&
	     SL(SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED)) ||
	    (PPS(PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED) && disabled &&
	     !SL(SLICE_SAO_LUMA) && !SL(SLICE_SAO_CHROMA) &&
	     !SL(SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED)))
		return fail(error, -EINVAL, "slice loop-filter flag contradicts inferred PPS value");
	return 0;
}

static bool same_inherited_slice(const struct v4l2_ctrl_hevc_slice_params *a,
				 const struct v4l2_ctrl_hevc_slice_params *b)
{
	u64 mask = ~V4L2_HEVC_SLICE_PARAMS_FLAG_DEPENDENT_SLICE_SEGMENT;
	unsigned int n;

	if ((a->flags & mask) != (b->flags & mask) ||
	    a->slice_type != b->slice_type || a->slice_pic_order_cnt != b->slice_pic_order_cnt ||
	    a->slice_qp_delta != b->slice_qp_delta || a->slice_cb_qp_offset != b->slice_cb_qp_offset ||
	    a->slice_cr_qp_offset != b->slice_cr_qp_offset ||
	    a->slice_beta_offset_div2 != b->slice_beta_offset_div2 ||
	    a->slice_tc_offset_div2 != b->slice_tc_offset_div2)
		return false;
	if (a->slice_type == V4L2_HEVC_SLICE_TYPE_I)
		return true;
	if (a->num_ref_idx_l0_active_minus1 != b->num_ref_idx_l0_active_minus1 ||
	    a->five_minus_max_num_merge_cand != b->five_minus_max_num_merge_cand ||
	    a->collocated_ref_idx != b->collocated_ref_idx ||
	    memcmp(&a->pred_weight_table, &b->pred_weight_table, sizeof(a->pred_weight_table)))
		return false;
	n = a->num_ref_idx_l0_active_minus1 + 1;
	if (memcmp(a->ref_idx_l0, b->ref_idx_l0, n))
		return false;
	if (a->slice_type == V4L2_HEVC_SLICE_TYPE_B) {
		if (a->num_ref_idx_l1_active_minus1 != b->num_ref_idx_l1_active_minus1)
			return false;
		n = a->num_ref_idx_l1_active_minus1 + 1;
		if (memcmp(a->ref_idx_l1, b->ref_idx_l1, n))
			return false;
	}
	return true;
}

int meson_hevc_request_syntax(const struct meson_hevc_syntax_request *r,
			      u8 *destination, size_t capacity, size_t *length,
			      struct meson_hevc_syntax_error *error)
{
	struct syntax_writer writer = { .dst = destination, .capacity = capacity };
	struct syntax_rps rps;
	const struct v4l2_ctrl_hevc_slice_params *sl, *first = NULL, *independent = NULL;
	unsigned int picture_ctbs, address_bits;
	size_t i;
	int ret;

	if (error) {
		error->slice = SIZE_MAX;
		error->reason = NULL;
	}
	if (!length)
		return fail(error, -EINVAL, "output length pointer absent");
	*length = 0;
	if (!r || !r->sps || !r->pps || !r->decode || !r->slices || !r->num_slices)
		return fail(error, -ENODATA, "mandatory SPS/PPS/decode/slice controls absent");
	if (!destination)
		return fail(error, -EINVAL, "destination buffer absent");
	ret = validate_parameters(r, &picture_ctbs, error);
	if (ret)
		return ret;
	ret = prepare_rps(r, &rps, error);
	if (ret)
		return ret;
	address_bits = bit_width(picture_ctbs - 1);
	write_vps(&writer, r->sps);
	write_sps(&writer, r);
	write_pps(&writer, r);
	if (writer.error)
		return fail(error, writer.error, writer.error == -EINVAL ?
			    "scaling-list DC coefficient is zero (legal range 1..255)" :
			    "destination capacity exhausted in parameter sets");
	for (i = 0; i < r->num_slices; i++) {
		if (error)
			error->slice = i;
		sl = r->slices[i].params;
		if (!sl)
			return fail(error, -ENODATA, "slice control absent");
		ret = validate_slice(r, sl, picture_ctbs, error);
		if (ret)
			return ret;
		if (!i) {
			if (sl->slice_segment_addr ||
			    (sl->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_DEPENDENT_SLICE_SEGMENT))
				return fail(error, -EINVAL, "request must start with first independent slice");
			first = sl;
		} else if (!sl->slice_segment_addr || sl->nal_unit_type != first->nal_unit_type ||
			   sl->nuh_temporal_id_plus1 != first->nuh_temporal_id_plus1) {
			return fail(error, -EINVAL, "request slices do not describe one picture/NAL layer");
		}
		if (sl->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_DEPENDENT_SLICE_SEGMENT) {
			if (!independent || !same_inherited_slice(sl, independent))
				return fail(error, -EINVAL, "dependent slice controls disagree with inherited independent header");
		} else {
			independent = sl;
		}
		ret = write_slice(&writer, r, &r->slices[i], &rps, address_bits, error);
		if (ret)
			return ret;
	}
	*length = writer.size;
	if (error)
		error->slice = SIZE_MAX;
	return 0;
}
