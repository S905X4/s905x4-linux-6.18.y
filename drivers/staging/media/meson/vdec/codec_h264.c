// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2019 BayLibre, SAS
 * Author: Maxime Jourdan <mjourdan@baylibre.com>
 */

#include <linux/iopoll.h>
#include <linux/sort.h>
#include <linux/vmalloc.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "vdec_helpers.h"
#include "dos_regs.h"
#include "codec_h264.h"
#include "esparser.h"
#include "vdec_1.h"

#define SIZE_EXT_FW	(20 * SZ_1K)
#define SIZE_WORKSPACE	0x1ee000
#define SIZE_SEI	(8 * SZ_1K)

/*
 * Offset added by the firmware which must be substracted
 * from the workspace phyaddr
 */
#define WORKSPACE_BUF_OFFSET	0x1000000

/* ISR status */
#define CMD_MASK		GENMASK(7, 0)
#define CMD_SRC_CHANGE		1
#define CMD_FRAMES_READY	2
#define CMD_FATAL_ERROR		6
#define CMD_BAD_WIDTH		7
#define CMD_BAD_HEIGHT		8

#define SEI_DATA_READY	BIT(15)

/* Picture type */
#define SINGLE_PIC_TOP_BOT	5
#define SINGLE_PIC_BOT_TOP	6

/* Size of Motion Vector per macroblock */
#define MB_MV_SIZE	96

/* Frame status data */
#define PIC_STRUCT_BIT	5
#define PIC_STRUCT_MASK	GENMASK(2, 0)
#define BUF_IDX_MASK	GENMASK(4, 0)
#define SINGLE_ERROR_FLAG	BIT(9)
#define OFFSET_BIT	16
#define OFFSET_MASK	GENMASK(15, 0)

/* Bitstream parsed data */
#define MB_TOTAL_BIT	8
#define MB_TOTAL_MASK	GENMASK(15, 0)
#define MB_WIDTH_MASK	GENMASK(7, 0)
#define MAX_REF_BIT	24
#define MAX_REF_MASK	GENMASK(6, 0)
#define AR_IDC_BIT	16
#define AR_IDC_MASK	GENMASK(7, 0)
#define AR_PRESENT_FLAG	BIT(0)
#define AR_EXTEND	0xff

/* H.264 end_of_sequence/end_of_stream RBSPs and parser padding. SINGLE
 * requires its frame-status EOS bit; MULTI requires terminal SEARCH_BUFEMPTY
 * and the completed-picture ledger. Padding alone completes neither path.
 */
static const u8 eos_sequence[SZ_4K] = {
	0x00, 0x00, 0x00, 0x01, 0x0a, 0x80,
	0x00, 0x00, 0x00, 0x01, 0x0b, 0x80,
};

static const u8 *codec_h264_eos_sequence(u32 *len)
{
	*len = ARRAY_SIZE(eos_sequence);
	return eos_sequence;
}

struct h264_multi;

struct codec_h264 {
	struct h264_multi *multi;
	/* H.264 decoder requires an extended firmware */
	void      *ext_fw_vaddr;
	dma_addr_t ext_fw_paddr;

	/* Buffer for the H.264 Workspace */
	void      *workspace_vaddr;
	dma_addr_t workspace_paddr;

	/* Buffer for the H.264 references MV */
	void      *ref_vaddr;
	dma_addr_t ref_paddr;
	u32	   ref_size;

	/* Buffer for parsed SEI data */
	void      *sei_vaddr;
	dma_addr_t sei_paddr;

	u32 mb_width;
	u32 mb_height;
	u32 max_refs;
	u32 seq_info;
	u32 max_dpb;
	u32 mv_bytes;
	u32 level;
};

/* Single firmware scratch0 distinguishes the SPS reference count, level
 * DPB capacity and actual surface reservoir. Do not substitute one for the
 * other: B-picture reordering can need more DPB slots than active references.
 */
static u32 codec_h264_max_dpb(u32 level, u32 mb_width, u32 mb_height)
{
	u32 capacity;
	u32 size = ALIGN(mb_width, 4) * ALIGN(mb_height, 4) * 3840;

	switch (level) {
	case 10: capacity = 1485; break;
	case 11: capacity = 3375; break;
	case 12: case 13: case 20: capacity = 8910; break;
	case 21: capacity = 17820; break;
	case 22: case 30: capacity = 30375; break;
	case 31: capacity = 67500; break;
	case 32: capacity = 76800; break;
	case 40: case 41: case 42: capacity = 122880; break;
	case 50: capacity = 414000; break;
	case 51: case 52: capacity = 691200; break;
	default: return 16;
	}
	return min(DIV_ROUND_UP(capacity * 1024, size), 16U);
}

static const struct v4l2_fract par_table[] = {
	{ 1, 1 },   { 1, 1 },    { 12, 11 }, { 10, 11 },
	{ 16, 11 }, { 40, 33 },  { 24, 11 }, { 20, 11 },
	{ 32, 11 }, { 80, 33 },  { 18, 11 }, { 15, 11 },
	{ 64, 33 }, { 160, 99 }, { 4, 3 },   { 3, 2 },
	{ 2, 1 }
};

/*
 * Host DPB algorithms ported from Amlogic h264_multi/h264_dpb.c.
 * Copyright (C) 2017 Amlogic, Inc. GPL-2.0-or-later.
 * VFM, concealment, MVC and heuristic fast-output paths are not used here.
 * Only PIC_DATA_DONE pictures enter this DPB; both fields share a surface.
 */
static int meson_h264_dpb_prepare_display_buf(struct amvdec_session *, struct FrameStore *);
static int meson_h264_dpb_release_buf_spec_num(struct amvdec_session *, int);
static int meson_h264_dpb_get_free_buf_idx(struct amvdec_session *);

static void meson_h264_dpb_adaptive_memory_management(struct h264_dpb_stru *p_H264_Dpb,
				       struct StorablePicture *p);
static int meson_h264_dpb_allocate_colocate_buf(struct h264_dpb_stru *p_H264_Dpb);
static void meson_h264_dpb_bufmgr_h264_remove_unused_frame(struct h264_dpb_stru *p_H264_Dpb,
	u8 force_flag);
static void meson_h264_dpb_bufmgr_post(struct h264_dpb_stru *p_H264_Dpb);
static void meson_h264_dpb_check_frame_store_same_pic_num(struct DecodedPictureBuffer *p_Dpb,
	struct StorablePicture *p, struct Slice *currSlice);
static void meson_h264_dpb_check_num_ref(struct DecodedPictureBuffer *p_Dpb);
static inline int meson_h264_dpb_compare_fs_by_frame_num_desc(const void *arg1,
		const void *arg2);
static inline int meson_h264_dpb_compare_fs_by_lt_pic_idx_asc(const void *arg1,
		const void *arg2);
static inline int meson_h264_dpb_compare_fs_by_poc_asc(const void *arg1, const void *arg2);
static inline int meson_h264_dpb_compare_fs_by_poc_desc(const void *arg1, const void *arg2);
static inline int meson_h264_dpb_compare_pic_by_lt_pic_num_asc(const void *arg1,
		const void *arg2);
static inline int meson_h264_dpb_compare_pic_by_pic_num_desc(const void *arg1,
		const void *arg2);
static inline int meson_h264_dpb_compare_pic_by_poc_asc(const void *arg1, const void *arg2);
static inline int meson_h264_dpb_compare_pic_by_poc_desc(const void *arg1, const void *arg2);
static void meson_h264_dpb_decode_poc(struct VideoParameters *p_Vid, struct Slice *pSlice);
static void meson_h264_dpb_dpb_combine_field(struct h264_dpb_stru *p_H264_Dpb,
			      struct FrameStore *fs);
static void meson_h264_dpb_dpb_init_global(struct h264_dpb_stru *p_H264_Dpb,
	int id, int actual_dpb_size, int max_reference_size);
static int meson_h264_dpb_dpb_print(int index, int flags, const char *format, ...);
static int meson_h264_dpb_dpb_print_cont(int index, int flags, const char *format, ...);
static void meson_h264_dpb_dpb_split_field(struct h264_dpb_stru *p_H264_Dpb,
			    struct FrameStore *fs);
static void meson_h264_dpb_dump_dpb(struct DecodedPictureBuffer *d, u8 force);
static void meson_h264_dpb_flush_dpb(struct h264_dpb_stru *p_H264_Dpb);
static void meson_h264_dpb_free_picture(struct h264_dpb_stru *p_H264_Dpb,
			 struct StorablePicture *pic);
static void meson_h264_dpb_gen_field_ref_ids(struct VideoParameters *p_Vid,
			      struct StorablePicture *p);
static void meson_h264_dpb_gen_pic_list_from_frame_list(enum PictureStructure currStructure,
		struct FrameStore **fs_list, int list_idx,
		struct StorablePicture **list,
		char *list_size, int long_term);
static int meson_h264_dpb_get_long_term_flag_by_buf_spec_num(struct h264_dpb_stru *p_H264_Dpb,
	int buf_spec_num);
static struct StorablePicture *meson_h264_dpb_get_long_term_pic(struct Slice *currSlice,
		struct DecodedPictureBuffer *p_Dpb, int LongtermPicNum);
static struct StorablePicture *meson_h264_dpb_get_new_pic(struct h264_dpb_stru *p_H264_Dpb,
		enum PictureStructure structure, unsigned char is_output);
static int meson_h264_dpb_get_pic_num_x(struct StorablePicture *p,
			 int difference_of_pic_nums_minus1);
static struct StorablePicture *meson_h264_dpb_get_short_term_pic(struct Slice *currSlice,
		struct DecodedPictureBuffer *p_Dpb, int picNum);
static int meson_h264_dpb_h264_slice_header_process(struct h264_dpb_stru *p_H264_Dpb, int *frame_num_gap);
static void meson_h264_dpb_idr_memory_management(struct h264_dpb_stru *p_H264_Dpb,
				  struct StorablePicture *p);
static void meson_h264_dpb_init_colocate_buf(struct h264_dpb_stru *p_H264_Dpb, int count);
static void meson_h264_dpb_init_dpb(struct h264_dpb_stru *p_H264_Dpb, int type);
static void meson_h264_dpb_init_dummy_fs(void);
static void meson_h264_dpb_init_lists_b_slice(struct Slice *currSlice);
static void meson_h264_dpb_init_lists_i_slice(struct Slice *currSlice);
static void meson_h264_dpb_init_lists_p_slice(struct Slice *currSlice);
static void meson_h264_dpb_init_mbaff_lists(struct h264_dpb_stru *p_H264_Dpb,
			     struct Slice *currSlice);
static void meson_h264_dpb_init_picture(struct h264_dpb_stru *p_H264_Dpb,
			 struct Slice *currSlice,
			 struct StorablePicture *dec_picture);
static void meson_h264_dpb_insert_picture_in_dpb(struct h264_dpb_stru *p_H264_Dpb,
				  struct FrameStore *fs,
				  struct StorablePicture *p,
				  unsigned char data_flag);
static inline int meson_h264_dpb_is_long_ref(struct StorablePicture *s);
static int meson_h264_dpb_is_long_term_reference(struct FrameStore *fs);
static inline int meson_h264_dpb_is_short_ref(struct StorablePicture *s);
static int meson_h264_dpb_is_short_term_reference(struct DecodedPictureBuffer *p_Dpb,
				   struct FrameStore *fs);
static int meson_h264_dpb_is_used_for_reference(struct FrameStore *fs);
static void meson_h264_dpb_mark_pic_long_term(struct DecodedPictureBuffer *p_Dpb,
			       struct StorablePicture *p,
			       int long_term_frame_idx, int picNumX);
static void meson_h264_dpb_mm_assign_long_term_frame_idx(struct DecodedPictureBuffer *p_Dpb,
		struct StorablePicture *p, int difference_of_pic_nums_minus1,
		int long_term_frame_idx);
static void meson_h264_dpb_mm_mark_current_picture_long_term(struct DecodedPictureBuffer
		*p_Dpb, struct StorablePicture *p, int long_term_frame_idx);
static void meson_h264_dpb_mm_unmark_all_long_term_for_reference(struct DecodedPictureBuffer
		*p_Dpb);
static void meson_h264_dpb_mm_unmark_all_short_term_for_reference(struct DecodedPictureBuffer
		*p_Dpb);
static void meson_h264_dpb_mm_unmark_long_term_for_reference(struct DecodedPictureBuffer
		*p_Dpb, struct StorablePicture *p, int long_term_pic_num);
static void meson_h264_dpb_mm_unmark_short_term_for_reference(struct DecodedPictureBuffer
		*p_Dpb, struct StorablePicture *p,
		int difference_of_pic_nums_minus1);
static void meson_h264_dpb_mm_update_max_long_term_frame_idx(struct DecodedPictureBuffer
		*p_Dpb, int max_long_term_frame_idx_plus1);
static int meson_h264_dpb_output_frames(struct h264_dpb_stru *dpb, unsigned char flush);
static void meson_h264_dpb_qsort(void *base, size_t num, size_t width,
                  int (*compare)(const void *, const void *));
static void meson_h264_dpb_ref_pic_list_reordering(struct h264_dpb_stru *p_H264_Dpb,
				struct Slice *currSlice);
static int meson_h264_dpb_release_colocate_buf(struct h264_dpb_stru *p_H264_Dpb, int index);
static int meson_h264_dpb_release_picture(struct h264_dpb_stru *p_H264_Dpb,
		   struct StorablePicture *pic);
static void meson_h264_dpb_remove_frame_from_dpb(struct h264_dpb_stru *p_H264_Dpb, int pos);
static int meson_h264_dpb_remove_unused_frame_from_dpb(struct h264_dpb_stru *p_H264_Dpb);
static void meson_h264_dpb_reorder_lists(struct Slice *currSlice);
static void meson_h264_dpb_reorder_long_term(struct Slice *currSlice,
				struct StorablePicture **RefPicListX,
				int num_ref_idx_lX_active_minus1,
			      int LongTermPicNum, int *refIdxLX);
static void meson_h264_dpb_reorder_ref_pic_list(struct Slice *currSlice, int cur_list);
static void meson_h264_dpb_reorder_short_term(struct Slice *currSlice, int cur_list,
				int num_ref_idx_lX_active_minus1,
				int picNumLX, int *refIdxLX);
static void meson_h264_dpb_set_frame_output_flag(struct h264_dpb_stru *p_H264_Dpb, int index);
static void meson_h264_dpb_slice_prepare(struct h264_dpb_stru *p_H264_Dpb,
		struct DecodedPictureBuffer *p_Dpb,
		struct VideoParameters *p_Vid,
		struct SPSParameters *sps, struct Slice *pSlice);
static void meson_h264_dpb_sliding_window_memory_management(
		struct DecodedPictureBuffer *p_Dpb,
		struct StorablePicture *p);
static int meson_h264_dpb_store_picture_in_dpb(struct h264_dpb_stru *p_H264_Dpb,
			  struct StorablePicture *p,
			  unsigned char data_flag);
static void meson_h264_dpb_unmark1(struct DecodedPictureBuffer *p_Dpb,
	unsigned int curr_frame_num, int i);
static void meson_h264_dpb_unmark2(struct DecodedPictureBuffer *p_Dpb,
	int curr_pic_num, int i);
static void meson_h264_dpb_unmark3_bottom(struct DecodedPictureBuffer *p_Dpb,
	unsigned int curr_frame_num, int curr_pic_num, int mark_current, int i);
static void meson_h264_dpb_unmark3_top(struct DecodedPictureBuffer *p_Dpb,
	unsigned int curr_frame_num, int curr_pic_num, int mark_current, int i);
static void meson_h264_dpb_unmark_for_long_term_reference(struct FrameStore *fs);
static void meson_h264_dpb_unmark_for_reference(struct DecodedPictureBuffer *p_Dpb,
				 struct FrameStore *fs);
static void meson_h264_dpb_unmark_long_term_field_for_reference_by_frame_idx(
	struct DecodedPictureBuffer *p_Dpb, enum PictureStructure structure,
	int long_term_frame_idx, int mark_current, unsigned int curr_frame_num,
	int curr_pic_num);
static void meson_h264_dpb_unmark_long_term_frame_for_reference_by_frame_idx(
	struct DecodedPictureBuffer *p_Dpb, int long_term_frame_idx);
static int meson_h264_dpb_unmark_one_error_out_frame(struct h264_dpb_stru *p_H264_Dpb);
static int meson_h264_dpb_unmark_one_out_frame(struct h264_dpb_stru *p_H264_Dpb);
static void meson_h264_dpb_update_ltref_list(struct DecodedPictureBuffer *p_Dpb);
static void meson_h264_dpb_update_pic_num(struct h264_dpb_stru *p_H264_Dpb);
static void meson_h264_dpb_update_ref_list(struct DecodedPictureBuffer *p_Dpb);





#define FRAME_NUM_MAX_SIZE 0x10000

static int meson_h264_dpb_dpb_print(int index, int flags, const char *format, ...)
{
    struct va_format message;
    va_list args;

    if (flags)
        return 0;
    va_start(args, format);
    message.fmt = format;
    message.va = &args;
    pr_debug("meson-h264: %pV", &message);
    va_end(args);
    return 0;
}

static int meson_h264_dpb_dpb_print_cont(int index, int flags, const char *format, ...)
{
    struct va_format message;
    va_list args;

    if (flags)
        return 0;
    va_start(args, format);
    message.fmt = format;
    message.va = &args;
    pr_debug("meson-h264: %pV", &message);
    va_end(args);
    return 0;
}



#define CHECK_VALID(list_size, mark) {\
	if (list_size > MAX_LIST_SIZE || list_size < 0) { \
		meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_ERROR, \
		"%s(%d): listXsize[%d] %d is larger than max size\r\n",\
		__func__, __LINE__, mark, list_size);\
		list_size = 0; \
		p_H264_Dpb->dpb_error_flag = __LINE__;\
	} \
	}

static struct DecRefPicMarking_s
	dummy_dec_ref_pic_marking_buffer
	[DEC_REF_PIC_MARKING_BUFFER_NUM_MAX];
static struct StorablePicture dummy_pic;
static struct FrameStore dummy_fs;
static struct StorablePicture *meson_h264_dpb_get_new_pic(
	struct h264_dpb_stru *p_H264_Dpb,
	enum PictureStructure structure, unsigned char is_output);

static void meson_h264_dpb_init_dummy_fs(void)
{
	dummy_fs.frame = &dummy_pic;
	dummy_fs.top_field = &dummy_pic;
	dummy_fs.bottom_field = &dummy_pic;

	dummy_pic.top_field = &dummy_pic;
	dummy_pic.bottom_field = &dummy_pic;
	dummy_pic.frame = &dummy_pic;

	dummy_pic.dec_ref_pic_marking_buffer =
		&dummy_dec_ref_pic_marking_buffer[0];
}

enum {
	LIST_0 = 0,
	LIST_1 = 1,
	BI_PRED = 2,
	BI_PRED_L0 = 3,
	BI_PRED_L1 = 4
};

static void meson_h264_dpb_ref_pic_list_reordering(struct h264_dpb_stru *p_H264_Dpb,
				struct Slice *currSlice)
{
	unsigned int list, i, j;

	for (list = 0; list < 2; list++) {
		const u16 *cmd = list ? p_H264_Dpb->dpb_param.mmco.l1_reorder_cmd :
			p_H264_Dpb->dpb_param.mmco.l0_reorder_cmd;

		currSlice->ref_pic_list_reordering_flag[list] = 0;
		if (currSlice->slice_type == I_SLICE ||
		    (list && currSlice->slice_type != B_SLICE))
			continue;
		/* h264_multi_validate_rpm checked command/operand bounds first. */
		for (i = 0, j = 0; i < REORDERING_COMMAND_MAX_SIZE; i++) {
			unsigned int op = cmd[j++];

			currSlice->modification_of_pic_nums_idc[list][i] = op;
			/* The 33rd entry is the terminator after 32 modifications. */
			if (op == 3)
				break;
			currSlice->ref_pic_list_reordering_flag[list] = 1;
			if (op < 2)
				currSlice->abs_diff_pic_num_minus1[list][i] = cmd[j++];
			else
				currSlice->long_term_pic_idx[list][i] = cmd[j++];
		}
	}
}

static void meson_h264_dpb_slice_prepare(struct h264_dpb_stru *p_H264_Dpb,
		struct DecodedPictureBuffer *p_Dpb,
		struct VideoParameters *p_Vid,
		struct SPSParameters *sps, struct Slice *pSlice)
{
	int i, j;
	
	unsigned short *mmco_cmd = &p_H264_Dpb->dpb_param.mmco.mmco_cmd[0];
	
	sps->pic_order_cnt_type =
		p_H264_Dpb->dpb_param.l.data[PIC_ORDER_CNT_TYPE];
	sps->log2_max_pic_order_cnt_lsb_minus4 =
		p_H264_Dpb->dpb_param.l.data[LOG2_MAX_PIC_ORDER_CNT_LSB] - 4;
	sps->num_ref_frames_in_pic_order_cnt_cycle =
		p_H264_Dpb->
		dpb_param.l.data[NUM_REF_FRAMES_IN_PIC_ORDER_CNT_CYCLE];
	for (i = 0; i < 128; i++)
		sps->offset_for_ref_frame[i] =
			(short) p_H264_Dpb->
			dpb_param.mmco.offset_for_ref_frame_base[i];
	sps->offset_for_non_ref_pic =
		(short) p_H264_Dpb->dpb_param.l.data[OFFSET_FOR_NON_REF_PIC];
	sps->offset_for_top_to_bottom_field =
		(short) p_H264_Dpb->dpb_param.l.data
		[OFFSET_FOR_TOP_TO_BOTTOM_FIELD];

	pSlice->frame_num = p_H264_Dpb->dpb_param.dpb.frame_num;
	pSlice->idr_flag =
		(p_H264_Dpb->dpb_param.dpb.NAL_info_mmco & 0x1f)
		== 5 ? 1 : 0;
	pSlice->nal_reference_idc =
		(p_H264_Dpb->dpb_param.dpb.NAL_info_mmco >> 5)
		& 0x3;
	pSlice->pic_order_cnt_lsb =
		p_H264_Dpb->dpb_param.dpb.pic_order_cnt_lsb;
	pSlice->field_pic_flag = 0;
	pSlice->bottom_field_flag = 0;
	pSlice->delta_pic_order_cnt_bottom = val(
		p_H264_Dpb->dpb_param.dpb.delta_pic_order_cnt_bottom);
	pSlice->delta_pic_order_cnt[0] = val(
		p_H264_Dpb->dpb_param.dpb.delta_pic_order_cnt_0);
	pSlice->delta_pic_order_cnt[1] = val(
		p_H264_Dpb->dpb_param.dpb.delta_pic_order_cnt_1);

	p_Vid->max_frame_num = 1 <<
	(p_H264_Dpb->dpb_param.l.data[LOG2_MAX_FRAME_NUM]);

	pSlice->structure = (p_H264_Dpb->
		dpb_param.l.data[NEW_PICTURE_STRUCTURE] == 3) ?
		FRAME : p_H264_Dpb->dpb_param.l.data[NEW_PICTURE_STRUCTURE];
	if (pSlice->structure == FRAME) {
		pSlice->field_pic_flag = 0;
		pSlice->bottom_field_flag = 0;
	} else {
		pSlice->field_pic_flag = 1;
		if (pSlice->structure == TOP_FIELD)
			pSlice->bottom_field_flag = 0;
		else
			pSlice->bottom_field_flag = 1;
	}
	pSlice->pic_struct = p_H264_Dpb->dpb_param.l.data[PICTURE_STRUCT];

	sps->num_ref_frames = p_H264_Dpb->
		dpb_param.l.data[MAX_REFERENCE_FRAME_NUM];
	sps->profile_idc =
		 (p_H264_Dpb->dpb_param.l.data[PROFILE_IDC_MMCO] >> 8) & 0xff;
	
	if (pSlice->idr_flag) {
		pSlice->long_term_reference_flag = mmco_cmd[0] & 1;
		pSlice->no_output_of_prior_pics_flag = (mmco_cmd[0] >> 1) & 1;
		meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
		PRINT_FLAG_DPB_DETAIL,
		"IDR: long_term_reference_flag %d no_output_of_prior_pics_flag %d\r\n",
		pSlice->long_term_reference_flag,
		pSlice->no_output_of_prior_pics_flag);

		p_H264_Dpb->long_term_reference_flag = pSlice->long_term_reference_flag;

		meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
		PRINT_FLAG_DPB_DETAIL,
		"idr set pre_frame_num(%d) to frame_num (%d)\n",
		p_Vid->pre_frame_num, pSlice->frame_num);

		p_Vid->pre_frame_num = pSlice->frame_num;
	} else if (p_H264_Dpb->mDPB.first_pic_done == 0) {

		p_Vid->pre_frame_num = pSlice->frame_num;
	}
	
	sps->log2_max_frame_num_minus4 =
		p_H264_Dpb->dpb_param.l.data[LOG2_MAX_FRAME_NUM] - 4;
	sps->frame_num_gap_allowed = p_H264_Dpb->dpb_param.l.data[FRAME_NUM_GAP_ALLOWED];

	p_Vid->non_conforming_stream =
		p_H264_Dpb->dpb_param.l.data[NON_CONFORMING_STREAM];
	p_Vid->recovery_point =
		p_H264_Dpb->dpb_param.l.data[RECOVERY_POINT];
	switch (p_H264_Dpb->dpb_param.l.data[SLICE_TYPE]) {
	case I_Slice:
		pSlice->slice_type = I_SLICE;
		break;
	case P_Slice:
		pSlice->slice_type = P_SLICE;
		break;
	case B_Slice:
		pSlice->slice_type = B_SLICE;
		break;
	default:
		pSlice->slice_type = NUM_SLICE_TYPES;
		break;
	}

	pSlice->num_ref_idx_active[LIST_0] =
		p_H264_Dpb->dpb_param.dpb.num_ref_idx_l0_active_minus1 +
		1;
	
	pSlice->num_ref_idx_active[LIST_1] =
		p_H264_Dpb->dpb_param.dpb.num_ref_idx_l1_active_minus1 +
		1;

	pSlice->p_Vid = p_Vid;
	pSlice->p_Dpb = p_Dpb;

	pSlice->first_mb_in_slice =
		p_H264_Dpb->dpb_param.l.data[FIRST_MB_IN_SLICE];
	pSlice->mode_8x8_flags = p_H264_Dpb->dpb_param.l.data[MODE_8X8_FLAGS];
	pSlice->picture_structure_mmco =
		p_H264_Dpb->dpb_param.dpb.picture_structure_mmco;
	meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
	"%s slice_type is %d, num_ref_idx_active[0,1]=%d,%d nal_reference_idc %d pic struct 0x%x(mmco stru 0x%x)\n",
		  __func__, pSlice->slice_type,
		  pSlice->num_ref_idx_active[LIST_0],
		  pSlice->num_ref_idx_active[LIST_1],
		  pSlice->nal_reference_idc,
		  pSlice->structure,
		  pSlice->picture_structure_mmco);
	if (pSlice->num_ref_idx_active[LIST_0] >= MAX_LIST_SIZE) {
		pSlice->num_ref_idx_active[LIST_0] = MAX_LIST_SIZE - 1;
		p_H264_Dpb->dpb_error_flag = __LINE__;
	}
	if (pSlice->num_ref_idx_active[LIST_1] >= MAX_LIST_SIZE) {
		pSlice->num_ref_idx_active[LIST_1] = MAX_LIST_SIZE - 1;
		p_H264_Dpb->dpb_error_flag = __LINE__;
	}

	pSlice->adaptive_ref_pic_buffering_flag = 0;
	if (pSlice->nal_reference_idc) {
		for (i = 0, j = 0; i < 44; j++) {
			unsigned short val;
			struct DecRefPicMarking_s *tmp_drpm =
				&pSlice->dec_ref_pic_marking_buffer[j];
			memset(tmp_drpm, 0, sizeof(struct DecRefPicMarking_s));
			val = tmp_drpm->
				memory_management_control_operation =
					mmco_cmd[i++];
			tmp_drpm->Next = NULL;
			if (j > 0) {
				pSlice->
				dec_ref_pic_marking_buffer[j - 1].Next =
					tmp_drpm;
			}
			if (val == 0 || i >= 44)
				break;
			pSlice->adaptive_ref_pic_buffering_flag = 1;
			if ((val == 1) || (val == 3)) {
				tmp_drpm->difference_of_pic_nums_minus1 =
					mmco_cmd[i++];
			}
			if (val == 2)
				tmp_drpm->long_term_pic_num = mmco_cmd[i++];
			if (i >= 44)
				break;
			if ((val == 3) || (val == 6))
				tmp_drpm->long_term_frame_idx = mmco_cmd[i++];
			if (val == 4) {
				tmp_drpm->max_long_term_frame_idx_plus1 =
					mmco_cmd[i++];
			}
			meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
				PRINT_FLAG_DPB_DETAIL,
				"dec_ref_pic_marking_buffer[%d]:operation %x diff_pic_minus1 %x long_pic_num %x long_frame_idx %x max_long_frame_idx_plus1 %x\n",
				j,
				tmp_drpm->memory_management_control_operation,
				tmp_drpm->difference_of_pic_nums_minus1,
				tmp_drpm->long_term_pic_num,
				tmp_drpm->long_term_frame_idx,
				tmp_drpm->max_long_term_frame_idx_plus1);
		}
	}

	meson_h264_dpb_ref_pic_list_reordering(p_H264_Dpb, pSlice);

	p_H264_Dpb->vui_status = p_H264_Dpb->dpb_param.l.data[VUI_STATUS];
	p_H264_Dpb->aspect_ratio_idc =
		p_H264_Dpb->dpb_param.l.data[ASPECT_RATIO_IDC];
	p_H264_Dpb->aspect_ratio_sar_width =
		p_H264_Dpb->dpb_param.l.data[ASPECT_RATIO_SAR_WIDTH];
	p_H264_Dpb->aspect_ratio_sar_height =
		p_H264_Dpb->dpb_param.l.data[ASPECT_RATIO_SAR_HEIGHT];

	p_H264_Dpb->fixed_frame_rate_flag = p_H264_Dpb->dpb_param.l.data[
		FIXED_FRAME_RATE_FLAG];
	p_H264_Dpb->num_units_in_tick =
		p_H264_Dpb->dpb_param.l.data[NUM_UNITS_IN_TICK];
	p_H264_Dpb->time_scale = val(&p_H264_Dpb->dpb_param.l.data[TIME_SCALE]);

	p_H264_Dpb->bitstream_restriction_flag =
		(p_H264_Dpb->dpb_param.l.data[SPS_FLAGS2] >> 3) & 0x1;
	p_H264_Dpb->num_reorder_frames =
		p_H264_Dpb->dpb_param.l.data[NUM_REORDER_FRAMES];
	p_H264_Dpb->max_dec_frame_buffering =
		p_H264_Dpb->dpb_param.l.data[MAX_BUFFER_FRAME];

	meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
		"%s return\n", __func__);
}

static void meson_h264_dpb_decode_poc(struct VideoParameters *p_Vid, struct Slice *pSlice)
{
	struct h264_dpb_stru *p_H264_Dpb = container_of(p_Vid,
					struct h264_dpb_stru, mVideo);
	struct SPSParameters *active_sps = p_Vid->active_sps;
	int i;
	
	unsigned int MaxPicOrderCntLsb = (1 <<
		(active_sps->log2_max_pic_order_cnt_lsb_minus4 + 4));

	meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
		PRINT_FLAG_DEBUG_POC,
		"%s:pic_order_cnt_type %d, idr_flag %d last_has_mmco_5 %d last_pic_bottom_field %d pic_order_cnt_lsb %d PrevPicOrderCntLsb %d\r\n",
		__func__,
		active_sps->pic_order_cnt_type,
		pSlice->idr_flag,
		p_Vid->last_has_mmco_5,
		p_Vid->last_pic_bottom_field,
		pSlice->pic_order_cnt_lsb,
		p_Vid->PrevPicOrderCntLsb
	);

	meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DEBUG_POC,
	"%s:field_pic_flag %d, bottom_field_flag %d frame_num %d PreviousFrameNum %d PreviousFrameNumOffset %d ax_frame_num %d num_ref_frames_in_pic_order_cnt_cycle %d offset_for_non_ref_pic %d\r\n",
		__func__,
		pSlice->field_pic_flag,
		pSlice->bottom_field_flag,
		pSlice->frame_num,
		p_Vid->PreviousFrameNum,
		p_Vid->PreviousFrameNumOffset,
		p_Vid->max_frame_num,
		active_sps->num_ref_frames_in_pic_order_cnt_cycle,
		active_sps->offset_for_non_ref_pic
	);

	meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DEBUG_POC,
	"%s: delta_pic_order_cnt %d %d nal_reference_idc %d\r\n",
	__func__,
	pSlice->delta_pic_order_cnt[0], pSlice->delta_pic_order_cnt[1],
	pSlice->nal_reference_idc
	);

	switch (active_sps->pic_order_cnt_type) {
	case 0: 
		
		if (pSlice->idr_flag) {
			p_Vid->PrevPicOrderCntMsb = 0;
			p_Vid->PrevPicOrderCntLsb = 0;
		}
		
		if (pSlice->pic_order_cnt_lsb < p_Vid->PrevPicOrderCntLsb &&
		    (p_Vid->PrevPicOrderCntLsb - pSlice->pic_order_cnt_lsb) >=
		    (MaxPicOrderCntLsb / 2))
			pSlice->PicOrderCntMsb = p_Vid->PrevPicOrderCntMsb +
					MaxPicOrderCntLsb;
		else if (pSlice->pic_order_cnt_lsb >
				p_Vid->PrevPicOrderCntLsb &&
			 (pSlice->pic_order_cnt_lsb -
				p_Vid->PrevPicOrderCntLsb)  >
				 (MaxPicOrderCntLsb / 2))
			pSlice->PicOrderCntMsb = p_Vid->PrevPicOrderCntMsb -
					MaxPicOrderCntLsb;
		else
			pSlice->PicOrderCntMsb = p_Vid->PrevPicOrderCntMsb;

		if (pSlice->field_pic_flag == 0) {
			
			pSlice->toppoc = pSlice->PicOrderCntMsb +
					pSlice->pic_order_cnt_lsb;
			pSlice->bottompoc = pSlice->toppoc +
					pSlice->delta_pic_order_cnt_bottom;
			pSlice->ThisPOC = pSlice->framepoc =
				(pSlice->toppoc < pSlice->bottompoc) ?
				 pSlice->toppoc : pSlice->bottompoc;
					
		} else if (pSlice->bottom_field_flag == 0) {
			
			pSlice->ThisPOC = pSlice->toppoc =
				pSlice->PicOrderCntMsb +
				pSlice->pic_order_cnt_lsb;
		} else {
			
			pSlice->ThisPOC = pSlice->bottompoc =
				pSlice->PicOrderCntMsb +
				pSlice->pic_order_cnt_lsb;
		}
		pSlice->framepoc = pSlice->ThisPOC;

		p_Vid->ThisPOC = pSlice->ThisPOC;

		p_Vid->PreviousFrameNum = pSlice->frame_num;

		if (pSlice->nal_reference_idc) {
			p_Vid->PrevPicOrderCntLsb = pSlice->pic_order_cnt_lsb;
			p_Vid->PrevPicOrderCntMsb = pSlice->PicOrderCntMsb;
		}

		break;

	case 1: 
		
		if (pSlice->idr_flag) {
			p_Vid->FrameNumOffset = 0;   
			if (pSlice->frame_num)
				meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
					PRINT_FLAG_DPB_DETAIL,
					"frame_num not equal to zero in IDR picture %d",
					-1020);
		} else {
			if (p_Vid->last_has_mmco_5) {
				p_Vid->PreviousFrameNumOffset = 0;
				p_Vid->PreviousFrameNum = 0;
			}
			if (pSlice->frame_num < p_Vid->PreviousFrameNum) {
				
				p_Vid->FrameNumOffset =
					p_Vid->PreviousFrameNumOffset +
						p_Vid->max_frame_num;
			} else {
				p_Vid->FrameNumOffset =
					p_Vid->PreviousFrameNumOffset;
			}
		}

		if (active_sps->num_ref_frames_in_pic_order_cnt_cycle)
			pSlice->AbsFrameNum =
				p_Vid->FrameNumOffset + pSlice->frame_num;
		else
			pSlice->AbsFrameNum = 0;
		if ((!pSlice->nal_reference_idc) && pSlice->AbsFrameNum > 0)
			pSlice->AbsFrameNum--;

		p_Vid->ExpectedDeltaPerPicOrderCntCycle = 0;

		if (active_sps->num_ref_frames_in_pic_order_cnt_cycle)
			for (i = 0; i < (int) active_sps->
				num_ref_frames_in_pic_order_cnt_cycle; i++) {
				p_Vid->ExpectedDeltaPerPicOrderCntCycle +=
					active_sps->offset_for_ref_frame[i];
				meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
					PRINT_FLAG_DEBUG_POC,
					"%s: offset_for_ref_frame %d\r\n",
					__func__,
					active_sps->
					offset_for_ref_frame[i]);
			}

		if (pSlice->AbsFrameNum) {
			p_Vid->PicOrderCntCycleCnt =
				(pSlice->AbsFrameNum - 1) /
				active_sps->
				num_ref_frames_in_pic_order_cnt_cycle;
			p_Vid->FrameNumInPicOrderCntCycle =
				(pSlice->AbsFrameNum - 1) %
				active_sps->
				num_ref_frames_in_pic_order_cnt_cycle;
			p_Vid->ExpectedPicOrderCnt =
				p_Vid->PicOrderCntCycleCnt *
				p_Vid->ExpectedDeltaPerPicOrderCntCycle;
			for (i = 0; i <= (int)p_Vid->
				FrameNumInPicOrderCntCycle; i++) {
				p_Vid->ExpectedPicOrderCnt +=
					active_sps->offset_for_ref_frame[i];
				meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
					PRINT_FLAG_DEBUG_POC,
					"%s: offset_for_ref_frame %d\r\n",
					__func__,
					active_sps->
					offset_for_ref_frame[i]);
			}
		} else
			p_Vid->ExpectedPicOrderCnt = 0;

		if (!pSlice->nal_reference_idc)
			p_Vid->ExpectedPicOrderCnt +=
				active_sps->offset_for_non_ref_pic;

		if (pSlice->field_pic_flag == 0) {
			
			pSlice->toppoc = p_Vid->ExpectedPicOrderCnt +
				pSlice->delta_pic_order_cnt[0];
			pSlice->bottompoc = pSlice->toppoc +
				active_sps->offset_for_top_to_bottom_field +
				pSlice->delta_pic_order_cnt[1];
			pSlice->ThisPOC = pSlice->framepoc =
				(pSlice->toppoc < pSlice->bottompoc) ?
				pSlice->toppoc : pSlice->bottompoc;
				
		} else if (pSlice->bottom_field_flag == 0) {
			
			pSlice->ThisPOC = pSlice->toppoc =
				p_Vid->ExpectedPicOrderCnt +
				pSlice->delta_pic_order_cnt[0];
		} else {
			
			pSlice->ThisPOC = pSlice->bottompoc =
				p_Vid->ExpectedPicOrderCnt +
				active_sps->offset_for_top_to_bottom_field +
				pSlice->delta_pic_order_cnt[0];
		}
		pSlice->framepoc = pSlice->ThisPOC;

		p_Vid->PreviousFrameNum = pSlice->frame_num;
		p_Vid->PreviousFrameNumOffset = p_Vid->FrameNumOffset;

		break;

	case 2: 
		if (pSlice->idr_flag) { 
			p_Vid->FrameNumOffset = 0;   
			pSlice->ThisPOC = pSlice->framepoc = pSlice->toppoc =
				pSlice->bottompoc = 0;
			if (pSlice->frame_num)
				meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
					PRINT_FLAG_DPB_DETAIL,
					"frame_num not equal to zero in IDR picture %d",
					-1020);
		} else {
			if (p_Vid->last_has_mmco_5) {
				p_Vid->PreviousFrameNum = 0;
				p_Vid->PreviousFrameNumOffset = 0;
			}
			if (pSlice->frame_num < p_Vid->PreviousFrameNum)
				p_Vid->FrameNumOffset =
					p_Vid->PreviousFrameNumOffset +
					p_Vid->max_frame_num;
			else
				p_Vid->FrameNumOffset =
					p_Vid->PreviousFrameNumOffset;

			pSlice->AbsFrameNum = p_Vid->FrameNumOffset +
				pSlice->frame_num;
			if (!pSlice->nal_reference_idc)
				pSlice->ThisPOC =
					(2 * pSlice->AbsFrameNum - 1);
			else
				pSlice->ThisPOC = (2 * pSlice->AbsFrameNum);

			if (pSlice->field_pic_flag == 0)
				pSlice->toppoc = pSlice->bottompoc =
					pSlice->framepoc = pSlice->ThisPOC;
			else if (pSlice->bottom_field_flag == 0)
				pSlice->toppoc = pSlice->framepoc =
				pSlice->ThisPOC;
			else
				pSlice->bottompoc = pSlice->framepoc =
				pSlice->ThisPOC;
		}

		p_Vid->PreviousFrameNum = pSlice->frame_num;
		p_Vid->PreviousFrameNumOffset = p_Vid->FrameNumOffset;
		break;

	default:

		break;
	}
}



static void meson_h264_dpb_dpb_init_global(struct h264_dpb_stru *p_H264_Dpb,
	int id, int actual_dpb_size, int max_reference_size)
{
	int i;

	meson_h264_dpb_init_dummy_fs();

	memset(&p_H264_Dpb->mDPB, 0, sizeof(struct DecodedPictureBuffer));

	memset(&p_H264_Dpb->mSlice, 0, sizeof(struct Slice));
	memset(&p_H264_Dpb->mVideo, 0, sizeof(struct VideoParameters));
	memset(&p_H264_Dpb->mSPS, 0, sizeof(struct SPSParameters));

	for (i = 0; i < DPB_SIZE_MAX; i++) {
		memset(&(p_H264_Dpb->mFrameStore[i]), 0,
			sizeof(struct FrameStore));
	}

	for (i = 0; i < MAX_PIC_BUF_NUM; i++) {
		memset(&(p_H264_Dpb->m_PIC[i]), 0,
			sizeof(struct StorablePicture));
		p_H264_Dpb->m_PIC[i].index = i;
	}
	p_H264_Dpb->decoder_index = id;

	p_H264_Dpb->mDPB.size = actual_dpb_size;
	p_H264_Dpb->max_reference_size = max_reference_size;
	p_H264_Dpb->poc_even_odd_flag = 0;
}

static void meson_h264_dpb_init_picture(struct h264_dpb_stru *p_H264_Dpb,
			 struct Slice *currSlice,
			 struct StorablePicture *dec_picture)
{
	
	meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
		  "%s dec_picture %p\n", __func__, dec_picture);
	dec_picture->top_poc = currSlice->toppoc;
	dec_picture->bottom_poc = currSlice->bottompoc;
	dec_picture->frame_poc = currSlice->framepoc;
	switch (currSlice->structure) {
	case TOP_FIELD: {
		dec_picture->poc = currSlice->toppoc;
		
		break;
	}
	case BOTTOM_FIELD: {
		dec_picture->poc = currSlice->bottompoc;
		
		break;
	}
	case FRAME: {
		dec_picture->poc = currSlice->framepoc;
		break;
	}
	default:
		meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
			  "p_Vid->structure not initialized %d\n", 235);
	}

	dec_picture->used_for_reference = (currSlice->nal_reference_idc != 0);
	dec_picture->idr_flag = currSlice->idr_flag;
	dec_picture->no_output_of_prior_pics_flag =
		currSlice->no_output_of_prior_pics_flag;
	dec_picture->long_term_reference_flag =
		currSlice->long_term_reference_flag;
	dec_picture->adaptive_ref_pic_buffering_flag =
		currSlice->adaptive_ref_pic_buffering_flag;
	dec_picture->dec_ref_pic_marking_buffer =
		&currSlice->dec_ref_pic_marking_buffer[0];

	dec_picture->pic_num   = currSlice->frame_num;
	dec_picture->frame_num = currSlice->frame_num;

	dec_picture->coded_frame = (currSlice->structure == FRAME);

	if ((currSlice->picture_structure_mmco & 0x3) == 3) {
		dec_picture->mb_aff_frame_flag = 1;
		meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
			"%s, picture_structure_mmco is %x, set mb_aff_frame_flag to 1\n",
			__func__,
			currSlice->picture_structure_mmco);
	}

	if (currSlice->pic_struct < PIC_INVALID) {
		dec_picture->pic_struct = currSlice->pic_struct;
	} else {
		dec_picture->pic_struct = PIC_INVALID;
	}

	meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
			  "%s pic_struct = %d\n", __func__, dec_picture->pic_struct);
}



static struct StorablePicture *meson_h264_dpb_get_new_pic(struct h264_dpb_stru *p_H264_Dpb,
		enum PictureStructure structure, unsigned char is_output)
{
	struct StorablePicture *s = NULL;
	struct StorablePicture *pic;
	struct VideoParameters *p_Vid = &(p_H264_Dpb->mVideo);
	
	int ii = 0;

	for (ii = 0; ii < MAX_PIC_BUF_NUM; ii++) {
		pic = &(p_H264_Dpb->m_PIC[ii]);
		if (pic->is_used == 0) {
			pic->is_used = 1;
			s = pic;
			break;
		}
	}

	if (s) {
		s->buf_spec_is_alloced = 0;
		s->buf_spec_num = -1;
		s->colocated_buf_index = -1;
		s->proc_flag = 0;
		s->pic_num   = 0;
		s->frame_num = 0;
		s->long_term_frame_idx = 0;
		s->long_term_pic_num   = 0;
		s->used_for_reference  = 0;
		s->is_long_term        = 0;
		s->non_existing        = 0;
		s->is_output           = 0;
		s->pre_output          = 0;
		s->max_slice_id        = 0;
		s->data_flag &= ~(ERROR_FLAG | NODISP_FLAG);

		s->structure = structure;

		s->coded_frame  = 0;
		s->mb_aff_frame_flag  = 0;

		s->top_poc = s->bottom_poc = s->poc = 0;
		s->seiHasTone_mapping = 0;
		s->frame_mbs_only_flag = p_Vid->active_sps->frame_mbs_only_flag;


	} else
		p_H264_Dpb->buf_alloc_fail = 1;
	meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
			"%s %p\n", __func__, s);
	return s;
}

static void meson_h264_dpb_free_picture(struct h264_dpb_stru *p_H264_Dpb,
			 struct StorablePicture *pic)
{
	if (pic == NULL || pic->index < 0 ||
		pic->index >= MAX_PIC_BUF_NUM)
		return;
	meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
			"%s %p %d\n", __func__, pic, pic->index);
	
	p_H264_Dpb->m_PIC[pic->index].is_used = 0;
}

static void meson_h264_dpb_gen_field_ref_ids(struct VideoParameters *p_Vid,
			      struct StorablePicture *p)
{
	int i, j;
	struct h264_dpb_stru *p_H264_Dpb = container_of(p_Vid,
					struct h264_dpb_stru, mVideo);
	
	meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
					"%s\n", __func__);

	for (j = 0; j < p_Vid->iSliceNumOfCurrPic; j++) {
		if (p->listX[j][LIST_0]) {
			p->listXsize[j][LIST_0] =
				p_Vid->ppSliceList[j]->listXsize[LIST_0];
			for (i = 0; i < p->listXsize[j][LIST_0]; i++)
				p->listX[j][LIST_0][i] =
				p_Vid->ppSliceList[j]->listX[LIST_0][i];
		}
		if (p->listX[j][LIST_1]) {
			p->listXsize[j][LIST_1] =
				p_Vid->ppSliceList[j]->listXsize[LIST_1];
			for (i = 0; i < p->listXsize[j][LIST_1]; i++)
				p->listX[j][LIST_1][i] =
				p_Vid->ppSliceList[j]->listX[LIST_1][i];
		}
	}
}

static void meson_h264_dpb_init_dpb(struct h264_dpb_stru *p_H264_Dpb, int type)
{
	unsigned int i;
	struct VideoParameters *p_Vid  = &p_H264_Dpb->mVideo;
	struct DecodedPictureBuffer *p_Dpb = &p_H264_Dpb->mDPB;
	struct SPSParameters *active_sps = &p_H264_Dpb->mSPS;

	p_Vid->active_sps = active_sps;
	meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
		"%s\n", __func__);

	p_Dpb->p_Vid = p_Vid;
	if (p_Dpb->init_done) {
		
		if (p_Vid->no_reference_picture) {
			meson_h264_dpb_free_picture(p_H264_Dpb, p_Vid->no_reference_picture);
			p_Vid->no_reference_picture = NULL;
		}
		p_Dpb->init_done = 0;
	}

	p_Dpb->num_ref_frames = active_sps->num_ref_frames;
	
	meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
		  "%s dpb_size is %d  num_ref_frames = %d (%d)\n",
		  __func__, p_Dpb->size,
		  p_Dpb->num_ref_frames,
		  active_sps->num_ref_frames);
	if (active_sps->num_ref_frames == 0xffff) {
		meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, 0,
		  "!!!Warning, num_ref_frames = %d is invalid\n",
		  active_sps->num_ref_frames);
	}

	p_Dpb->used_size = 0;
	p_Dpb->last_picture = NULL;

	p_Dpb->ref_frames_in_buffer = 0;
	p_Dpb->ltref_frames_in_buffer = 0;

	for (i = 0; i < p_Dpb->size; i++) {
		p_Dpb->fs[i] = &(p_H264_Dpb->mFrameStore[i]);
		
		p_Dpb->fs[i]->index       = i;
		p_Dpb->fs_ref[i]   = NULL;
		p_Dpb->fs_ltref[i] = NULL;
		p_Dpb->fs[i]->layer_id = 0; 
	}

	if (!p_Vid->no_reference_picture) {
		p_Vid->no_reference_picture = meson_h264_dpb_get_new_pic(p_H264_Dpb,
					      FRAME,

		1);
		p_Vid->no_reference_picture->top_field =
			p_Vid->no_reference_picture;
		p_Vid->no_reference_picture->bottom_field =
			p_Vid->no_reference_picture;
		p_Vid->no_reference_picture->frame =
			p_Vid->no_reference_picture;
	}
	p_Dpb->last_output_poc = INT_MIN;

	p_Vid->last_has_mmco_5 = 0;

	meson_h264_dpb_init_colocate_buf(p_H264_Dpb, p_H264_Dpb->max_reference_size);

	p_Dpb->init_done = 1;

}

static void meson_h264_dpb_dpb_split_field(struct h264_dpb_stru *p_H264_Dpb,
			    struct FrameStore *fs)
{
	struct StorablePicture *fs_top = NULL, *fs_btm = NULL;
	struct StorablePicture *frame = fs->frame;

	meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
		"%s %p %p\n", __func__, fs, frame);

	fs->poc = frame->poc;

	if (!frame->frame_mbs_only_flag) {
		fs_top = fs->top_field = meson_h264_dpb_get_new_pic(p_H264_Dpb,
			TOP_FIELD,

			1);
		fs_btm = fs->bottom_field = meson_h264_dpb_get_new_pic(p_H264_Dpb,
			BOTTOM_FIELD,

			1);
		if (fs_top == NULL || fs_btm == NULL)
			return;

		fs_top->buf_spec_num = frame->buf_spec_num;
		fs_btm->buf_spec_num = frame->buf_spec_num;

		fs_top->colocated_buf_index = frame->colocated_buf_index;
		fs_btm->colocated_buf_index = frame->colocated_buf_index;

		fs_top->data_flag = frame->data_flag;
		fs_btm->data_flag = frame->data_flag;
		fs_top->poc = frame->top_poc;
		fs_btm->poc = frame->bottom_poc;

		fs_top->frame_poc =  frame->frame_poc;

		fs_top->bottom_poc = fs_btm->bottom_poc =  frame->bottom_poc;
		fs_top->top_poc    = fs_btm->top_poc    =  frame->top_poc;
		fs_btm->frame_poc  = frame->frame_poc;

		fs_top->used_for_reference = fs_btm->used_for_reference
					     = frame->used_for_reference;
		fs_top->is_long_term = fs_btm->is_long_term
				       = frame->is_long_term;
		fs->long_term_frame_idx = fs_top->long_term_frame_idx
					  = fs_btm->long_term_frame_idx
					    = frame->long_term_frame_idx;

		fs_top->coded_frame = fs_btm->coded_frame = 1;
		fs_top->mb_aff_frame_flag = fs_btm->mb_aff_frame_flag
					    = frame->mb_aff_frame_flag;

		frame->top_field    = fs_top;
		frame->bottom_field = fs_btm;
		frame->frame         = frame;
		fs_top->bottom_field = fs_btm;
		fs_top->frame        = frame;
		fs_top->top_field = fs_top;
		fs_btm->top_field = fs_top;
		fs_btm->frame     = frame;
		fs_btm->bottom_field = fs_btm;

		fs_top->chroma_format_idc = fs_btm->chroma_format_idc =
						    frame->chroma_format_idc;
		fs_top->iCodingType = fs_btm->iCodingType = frame->iCodingType;
	} else {
		fs->top_field       = NULL;
		fs->bottom_field    = NULL;
		frame->top_field    = NULL;
		frame->bottom_field = NULL;
		frame->frame = frame;
	}

}

static void meson_h264_dpb_dpb_combine_field(struct h264_dpb_stru *p_H264_Dpb,
			      struct FrameStore *fs)
{

	meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
			"%s\n", __func__);

	if (!fs->frame) {
		fs->frame = meson_h264_dpb_get_new_pic(p_H264_Dpb,
			FRAME,

			1);
	}
	if (!fs->frame)
		return;

	fs->frame->buf_spec_num = fs->top_field->buf_spec_num;
	fs->frame->offset_delimiter = fs->top_field->offset_delimiter;
	fs->offset_delimiter = fs->top_field->offset_delimiter;
	fs->frame->colocated_buf_index = fs->top_field->colocated_buf_index;
	fs->frame->data_flag = fs->top_field->data_flag;
	fs->frame->slice_type = fs->top_field->slice_type;
	if (fs->bottom_field)
		fs->frame->data_flag |= (fs->bottom_field->data_flag & 0xf0);

	if (fs->bottom_field) {
		fs->poc = fs->frame->poc = fs->frame->frame_poc = imin(
				fs->top_field->poc, fs->bottom_field->poc);

		fs->bottom_field->frame_poc = fs->top_field->frame_poc = fs->frame->poc;

		fs->bottom_field->top_poc = fs->frame->top_poc = fs->top_field->poc;
		fs->top_field->bottom_poc = fs->frame->bottom_poc =
				fs->bottom_field->poc;

		fs->frame->used_for_reference = (fs->top_field->used_for_reference &&
						 fs->bottom_field->used_for_reference);
		fs->frame->is_long_term = (fs->top_field->is_long_term &&
					   fs->bottom_field->is_long_term);
	}

	if (fs->frame->is_long_term)
		fs->frame->long_term_frame_idx = fs->long_term_frame_idx;

	fs->frame->top_field    = fs->top_field;
	if (fs->bottom_field)
		fs->frame->bottom_field = fs->bottom_field;
	fs->frame->frame = fs->frame;

	fs->frame->coded_frame = 0;

	fs->frame->chroma_format_idc = fs->top_field->chroma_format_idc;
	fs->frame->frame_cropping_flag = fs->top_field->frame_cropping_flag;
	if (fs->frame->frame_cropping_flag) {
		fs->frame->frame_crop_top_offset =
			fs->top_field->frame_crop_top_offset;
		fs->frame->frame_crop_bottom_offset =
			fs->top_field->frame_crop_bottom_offset;
		fs->frame->frame_crop_left_offset =
			fs->top_field->frame_crop_left_offset;
		fs->frame->frame_crop_right_offset =
			fs->top_field->frame_crop_right_offset;
	}
	if (fs->bottom_field) {
		fs->top_field->frame = fs->bottom_field->frame = fs->frame;
		fs->top_field->top_field = fs->top_field;
		fs->top_field->bottom_field = fs->bottom_field;
		fs->bottom_field->top_field = fs->top_field;
		fs->bottom_field->bottom_field = fs->bottom_field;
	}

	fs->frame->iCodingType = fs->top_field->iCodingType;
	if (fs->bottom_field && fs->top_field->poc < fs->bottom_field->poc) {
		fs->pts = fs->top_field->pts;
		fs->pts64 = fs->top_field->pts64;
		
		if ((fs->frame->slice_type == B_SLICE)
			&& (!fs->bottom_field->pts) &&(!fs->bottom_field->pts64)) {
			fs->pts =  0;
			fs->pts64 = 0;
		}
		fs->offset_delimiter = fs->top_field->offset_delimiter;
		fs->decoded_frame_size = fs->top_field->pic_size + fs->bottom_field->pic_size;
	} else if (fs->bottom_field) {
		fs->pts = fs->bottom_field->pts;
		fs->pts64 = fs->bottom_field->pts64;
		fs->offset_delimiter = fs->bottom_field->offset_delimiter;
		fs->decoded_frame_size = fs->top_field->pic_size + fs->bottom_field->pic_size;
	}
	
}


static void meson_h264_dpb_insert_picture_in_dpb(struct h264_dpb_stru *p_H264_Dpb,
				  struct FrameStore *fs,
				  struct StorablePicture *p,
				  unsigned char data_flag)
{
	
	struct VideoParameters *p_Vid = &p_H264_Dpb->mVideo;

	meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
		"%s %p %p\n", __func__, fs, p);
	p_H264_Dpb->dpb_frame_count++;
	fs->dpb_frame_count = p_H264_Dpb->dpb_frame_count;

	p->data_flag = data_flag;
	fs->data_flag |= data_flag;
	fs->buf_spec_num = p->buf_spec_num;
	fs->colocated_buf_index = p->colocated_buf_index;
	p->slice_type = p_H264_Dpb->mSlice.slice_type;
	switch (p->structure) {
	case FRAME:
		fs->frame = p;
		fs->is_used = 3;
		fs->slice_type = p->slice_type;
		fs->frame_size = p->frame_size;
		fs->offset_delimiter = p->offset_delimiter;
		fs->decoded_frame_size = p->pic_size;
		if (p->used_for_reference) {
			fs->is_reference = 3;
			fs->is_orig_reference = 3;
			if (p->is_long_term) {
				fs->is_long_term = 3;
				fs->long_term_frame_idx =
					p->long_term_frame_idx;
			}
		}
		fs->pts = p->pts;
		fs->pts64 = p->pts64;
		fs->layer_id = p->layer_id;

		meson_h264_dpb_dpb_split_field(p_H264_Dpb, fs);
		
		break;
	case TOP_FIELD:
		fs->top_field = p;
		fs->is_used |= 1;
		fs->layer_id = p->layer_id;
		if (fs->frame_size == 0) {
			fs->slice_type = p->slice_type;

		}
		fs->frame_size += p->frame_size;
		if (p->used_for_reference) {
			fs->is_reference |= 1;
			fs->is_orig_reference |= 1;
			if (p->is_long_term) {
				fs->is_long_term |= 1;
				fs->long_term_frame_idx =
					p->long_term_frame_idx;
			}
		}
		if (fs->is_used == 3) {
			
			meson_h264_dpb_dpb_combine_field(p_H264_Dpb, fs);
		} else {
			fs->poc = p->poc;
		}
		meson_h264_dpb_gen_field_ref_ids(p_Vid, p);
		break;
	case BOTTOM_FIELD:
		fs->bottom_field = p;
		fs->is_used |= 2;
		fs->layer_id = p->layer_id;
		if (fs->frame_size == 0) {
			fs->slice_type = p->slice_type;

		}
		fs->frame_size += p->frame_size;
		if (p->used_for_reference) {
			fs->is_reference |= 2;
			fs->is_orig_reference |= 2;
			if (p->is_long_term) {
				fs->is_long_term |= 2;
				fs->long_term_frame_idx =
					p->long_term_frame_idx;
			}
		}
		if (fs->is_used == 3) {
			
			meson_h264_dpb_dpb_combine_field(p_H264_Dpb, fs);
		} else {
			fs->poc = p->poc;
		}
		meson_h264_dpb_gen_field_ref_ids(p_Vid, p);
		break;
	}
	fs->frame_num = p->pic_num;
	fs->recovery_frame = p->recovery_frame;

	fs->is_output = p->is_output;
	fs->pre_output = p->pre_output;

	fs->max_mv = p->max_mv;
	fs->avg_mv = p->avg_mv;
	fs->min_mv = p->min_mv;

	fs->max_qp = p->max_qp;
	fs->avg_qp = p->avg_qp;
	fs->min_qp = p->min_qp;

	fs->max_skip = p->max_skip;
	fs->avg_skip = p->avg_skip;
	fs->min_skip = p->min_skip;

	fs->timestamp = p->timestamp;
	
}



static void meson_h264_dpb_unmark_for_reference(struct DecodedPictureBuffer *p_Dpb,
				 struct FrameStore *fs)
{
	struct h264_dpb_stru *p_H264_Dpb = container_of(p_Dpb,
				struct h264_dpb_stru, mDPB);
	meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
		"%s %p %p %p %p\n", __func__,
		fs, fs->frame, fs->top_field, fs->bottom_field);
	
	if (fs->is_used & 1) {
		if (fs->top_field)
			fs->top_field->used_for_reference = 0;
	}
	if (fs->is_used & 2) {
		if (fs->bottom_field)
			fs->bottom_field->used_for_reference = 0;
	}
	if (fs->is_used == 3) {
		if (fs->top_field && fs->bottom_field) {
			fs->top_field->used_for_reference = 0;
			fs->bottom_field->used_for_reference = 0;
		}
		fs->frame->used_for_reference = 0;
	}

	fs->is_reference = 0;

}

static void meson_h264_dpb_unmark_for_long_term_reference(struct FrameStore *fs)
{
	if (fs->is_used & 1) {
		if (fs->top_field) {
			fs->top_field->used_for_reference = 0;
			fs->top_field->is_long_term = 0;
		}
	}
	if (fs->is_used & 2) {
		if (fs->bottom_field) {
			fs->bottom_field->used_for_reference = 0;
			fs->bottom_field->is_long_term = 0;
		}
	}
	if (fs->is_used == 3) {
		if (fs->top_field && fs->bottom_field) {
			fs->top_field->used_for_reference = 0;
			fs->top_field->is_long_term = 0;
			fs->bottom_field->used_for_reference = 0;
			fs->bottom_field->is_long_term = 0;
		}
		fs->frame->used_for_reference = 0;
		fs->frame->is_long_term = 0;
	}

	fs->is_reference = 0;
	fs->is_long_term = 0;
}

static int meson_h264_dpb_get_long_term_flag_by_buf_spec_num(struct h264_dpb_stru *p_H264_Dpb,
	int buf_spec_num)
{
	struct DecodedPictureBuffer *p_Dpb = &p_H264_Dpb->mDPB;
	unsigned int i;

	for (i = 0; i < p_Dpb->used_size; i++) {
		if (p_Dpb->fs[i]->buf_spec_num == buf_spec_num)
			return p_Dpb->fs[i]->is_long_term;
	}
	return -1;
}

static void meson_h264_dpb_update_pic_num(struct h264_dpb_stru *p_H264_Dpb)
{
	unsigned int i;
	struct Slice *currSlice = &p_H264_Dpb->mSlice;
	struct VideoParameters *p_Vid = currSlice->p_Vid;
	struct DecodedPictureBuffer *p_Dpb = currSlice->p_Dpb;
	struct SPSParameters *active_sps = p_Vid->active_sps;
	int add_top = 0, add_bottom = 0;
	int max_frame_num = 1 << (active_sps->log2_max_frame_num_minus4 + 4);

	if (currSlice->structure == FRAME) {
		for (i = 0; i < p_Dpb->ref_frames_in_buffer; i++) {
			if (p_Dpb->fs_ref[i] == NULL ||
				p_Dpb->fs_ref[i]->frame == NULL) {
				p_H264_Dpb->dpb_error_flag = __LINE__;
				continue;
			}
			if (p_Dpb->fs_ref[i]->is_used == 3) {
				if ((p_Dpb->fs_ref[i]->frame->
					used_for_reference) &&
				    (!p_Dpb->fs_ref[i]->frame->
					is_long_term)) {
					if (p_Dpb->fs_ref[i]->frame_num >
						currSlice->frame_num) {
						p_Dpb->fs_ref[i]->
						frame_num_wrap =
						p_Dpb->fs_ref[i]->frame_num
							- max_frame_num;
					} else {
						p_Dpb->fs_ref[i]->
						frame_num_wrap =
						p_Dpb->fs_ref[i]->frame_num;
					}
					p_Dpb->fs_ref[i]->frame->pic_num =
					p_Dpb->fs_ref[i]->frame_num_wrap;
				}
			}
		}
		
		for (i = 0; i < p_Dpb->ltref_frames_in_buffer; i++) {
			if (p_Dpb->fs_ltref[i] == NULL ||
				p_Dpb->fs_ltref[i]->frame == NULL) {
				p_H264_Dpb->dpb_error_flag = __LINE__;
				continue;
			}
			if (p_Dpb->fs_ltref[i]->is_used == 3) {
				if (p_Dpb->fs_ltref[i]->frame->is_long_term) {
					p_Dpb->fs_ltref[i]->frame->
						long_term_pic_num =
						p_Dpb->fs_ltref[i]->frame->
							long_term_frame_idx;
				}
			}
		}
	} else {
		if (currSlice->structure == TOP_FIELD) {
			add_top    = 1;
			add_bottom = 0;
		} else {
			add_top    = 0;
			add_bottom = 1;
		}

		for (i = 0; i < p_Dpb->ref_frames_in_buffer; i++) {
			if (p_Dpb->fs_ref[i] == NULL) {
				p_H264_Dpb->dpb_error_flag = __LINE__;
				continue;
			}
			if (p_Dpb->fs_ref[i]->is_reference) {
				if (p_Dpb->fs_ref[i]->frame_num > currSlice->
					frame_num) {
					p_Dpb->fs_ref[i]->frame_num_wrap =
					p_Dpb->fs_ref[i]->frame_num -
					max_frame_num;
				} else {
					p_Dpb->fs_ref[i]->frame_num_wrap =
					p_Dpb->fs_ref[i]->frame_num;
				}
				if (p_Dpb->fs_ref[i]->is_reference & 1) {
					if (p_Dpb->fs_ref[i]->top_field
						== NULL) {
						p_H264_Dpb->dpb_error_flag =
							__LINE__;
						continue;
					}
					p_Dpb->fs_ref[i]->top_field->
					pic_num = (2 * p_Dpb->fs_ref[i]->
						frame_num_wrap) + add_top;
				}
				if (p_Dpb->fs_ref[i]->is_reference & 2) {
					if (p_Dpb->fs_ref[i]->bottom_field
						== NULL) {
						p_H264_Dpb->dpb_error_flag =
							__LINE__;
						continue;
					}
					p_Dpb->fs_ref[i]->bottom_field->
					pic_num = (2 * p_Dpb->fs_ref[i]->
						frame_num_wrap) + add_bottom;
				}
			}
		}
		
		for (i = 0; i < p_Dpb->ltref_frames_in_buffer; i++) {
			if (p_Dpb->fs_ltref[i] == NULL) {
				p_H264_Dpb->dpb_error_flag = __LINE__;
				continue;
			}
			if (p_Dpb->fs_ltref[i]->is_long_term & 1) {
				if (p_Dpb->fs_ltref[i]->top_field == NULL) {
					p_H264_Dpb->dpb_error_flag = __LINE__;
					continue;
				}
				p_Dpb->fs_ltref[i]->top_field->
					long_term_pic_num = 2 *
					p_Dpb->fs_ltref[i]->top_field->
					long_term_frame_idx + add_top;
			}
			if (p_Dpb->fs_ltref[i]->is_long_term & 2) {
				if (p_Dpb->fs_ltref[i]->bottom_field == NULL) {
					p_H264_Dpb->dpb_error_flag = __LINE__;
					continue;
				}
				p_Dpb->fs_ltref[i]->bottom_field->
					long_term_pic_num = 2 *
					p_Dpb->fs_ltref[i]->bottom_field->
					long_term_frame_idx + add_bottom;
			}
		}
	}
}

static void meson_h264_dpb_remove_frame_from_dpb(struct h264_dpb_stru *p_H264_Dpb, int pos)
{
	struct DecodedPictureBuffer *p_Dpb = &p_H264_Dpb->mDPB;
	struct FrameStore *fs = p_Dpb->fs[pos];
	struct FrameStore *tmp;
	unsigned int i;

	meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
		"%s pos %d %p\n", __func__, pos, fs);

	switch (fs->is_used) {
	case 3:
		meson_h264_dpb_free_picture(p_H264_Dpb, fs->frame);
		meson_h264_dpb_free_picture(p_H264_Dpb, fs->top_field);
		meson_h264_dpb_free_picture(p_H264_Dpb, fs->bottom_field);
		fs->frame = NULL;
		fs->top_field = NULL;
		fs->bottom_field = NULL;
		break;
	case 2:
		meson_h264_dpb_free_picture(p_H264_Dpb, fs->bottom_field);
		fs->bottom_field = NULL;
		break;
	case 1:
		meson_h264_dpb_free_picture(p_H264_Dpb, fs->top_field);
		fs->top_field = NULL;
		break;
	case 0:
		break;
	default:
		meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
			  "invalid frame store type %x", 500);
	}
	fs->data_flag = 0;
	fs->is_used = 0;
	fs->is_long_term = 0;
	fs->is_reference = 0;
	fs->is_orig_reference = 0;
	fs->frame_size = 0;
	
	tmp = p_Dpb->fs[pos];

	for (i = pos; i < p_Dpb->used_size - 1; i++)
		p_Dpb->fs[i] = p_Dpb->fs[i + 1];
	p_Dpb->fs[p_Dpb->used_size - 1] = tmp;

	if (p_Dpb->used_size)
		p_Dpb->used_size--;
}

static int meson_h264_dpb_is_used_for_reference(struct FrameStore *fs)
{
	if (fs->is_reference)
		return 1;

	if (fs->is_used == 3) { 
		if (fs->frame->used_for_reference)
			return 1;
	}

	if (fs->is_used & 1) { 
		if (fs->top_field) {
			if (fs->top_field->used_for_reference)
				return 1;
		}
	}

	if (fs->is_used & 2) { 
		if (fs->bottom_field) {
			if (fs->bottom_field->used_for_reference)
				return 1;
		}
	}
	return 0;
}

static int meson_h264_dpb_remove_unused_frame_from_dpb(struct h264_dpb_stru *p_H264_Dpb)
{
	unsigned int i;
	struct DecodedPictureBuffer *p_Dpb = &p_H264_Dpb->mDPB;
	struct StorablePicture *dec_picture = p_H264_Dpb->mVideo.dec_picture;

	for (i = 0; i < p_Dpb->used_size; i++) {
		if ((!meson_h264_dpb_is_used_for_reference(p_Dpb->fs[i])) &&
		    (p_Dpb->fs[i]->colocated_buf_index >= 0) &&
		    /* MMCO may unmark the first field while its mate owns the slot. */
		    (!dec_picture || dec_picture->buf_spec_is_alloced ||
		     dec_picture->colocated_buf_index != p_Dpb->fs[i]->colocated_buf_index)) {
			meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
			PRINT_FLAG_DPB_DETAIL,
			"meson_h264_dpb_release_colocate_buf[%d] for fs[%d]\n",
			p_Dpb->fs[i]->colocated_buf_index, i);

			meson_h264_dpb_release_colocate_buf(p_H264_Dpb,
				p_Dpb->fs[i]->colocated_buf_index); 
			p_Dpb->fs[i]->colocated_buf_index = -1;
		}
	}

	for (i = 0; i < p_Dpb->used_size; i++) {
		if (p_Dpb->fs[i]->is_output &&
			(!meson_h264_dpb_is_used_for_reference(p_Dpb->fs[i]))) {
			meson_h264_dpb_release_buf_spec_num(p_H264_Dpb->vdec,
				p_Dpb->fs[i]->buf_spec_num);
			p_Dpb->fs[i]->buf_spec_num = -1;
			meson_h264_dpb_remove_frame_from_dpb(p_H264_Dpb, i);

			meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
				PRINT_FLAG_DPB_DETAIL, "%s[%d]\n",
				__func__, i);

			return 1;
		}
	}
	return 0;
}

static int meson_h264_dpb_unmark_one_error_out_frame(struct h264_dpb_stru *p_H264_Dpb)
{
	int ret = 0;
	unsigned i;
	struct DecodedPictureBuffer *p_Dpb = &p_H264_Dpb->mDPB;

	for (i = 0; i < p_Dpb->used_size; i++) {
		if (p_Dpb->fs[i]->is_output &&
			((p_Dpb->fs[i]->data_flag & ERROR_FLAG) ||
			(p_Dpb->fs[i]->data_flag & NULL_FLAG))
			) {
			meson_h264_dpb_unmark_for_reference(p_Dpb, p_Dpb->fs[i]);

			ret = 1;
			break;
		}
	}
	return ret;
}

static int meson_h264_dpb_unmark_one_out_frame(struct h264_dpb_stru *p_H264_Dpb)
{
	int ret = 0;
	unsigned i;
	struct DecodedPictureBuffer *p_Dpb = &p_H264_Dpb->mDPB;

	for (i = 0; i < p_Dpb->used_size; i++) {
		if (p_Dpb->fs[i]->is_output) {
			meson_h264_dpb_unmark_for_reference(p_Dpb, p_Dpb->fs[i]);

			ret = 1;
		}
	}
	return ret;
}

static void meson_h264_dpb_bufmgr_h264_remove_unused_frame(struct h264_dpb_stru *p_H264_Dpb,
	u8 force_flag)
{
	struct DecodedPictureBuffer *p_Dpb = &p_H264_Dpb->mDPB;
	int ret = 0;
	unsigned char removed_flag = 0;
	do {
		ret = meson_h264_dpb_remove_unused_frame_from_dpb(p_H264_Dpb);
		if (ret != 0)
			removed_flag = 1;
	} while (ret != 0);
	if (removed_flag) {
		meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
			PRINT_FLAG_DPB_DETAIL, "%s\r\n", __func__);
		meson_h264_dpb_dump_dpb(p_Dpb, 0);
	} else if (force_flag == 2) {
		if (meson_h264_dpb_unmark_one_out_frame(p_H264_Dpb)) {
			meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
				0, "%s, Warning, force unmark one frame\r\n",
				__func__);
			meson_h264_dpb_update_ref_list(p_Dpb);
			meson_h264_dpb_remove_unused_frame_from_dpb(p_H264_Dpb);
			meson_h264_dpb_dump_dpb(p_Dpb, 0);
		}
	} else if (force_flag == 1) {
		if (meson_h264_dpb_unmark_one_error_out_frame(p_H264_Dpb)) {
			meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
				0, "%s, unmark error frame\r\n",
				__func__);
			meson_h264_dpb_update_ref_list(p_Dpb);
			meson_h264_dpb_remove_unused_frame_from_dpb(p_H264_Dpb);
			meson_h264_dpb_dump_dpb(p_Dpb, 0);
		}
	}
}





static int meson_h264_dpb_output_frames(struct h264_dpb_stru *dpb, unsigned char flush)
{
    struct DecodedPictureBuffer *d = &dpb->mDPB;
    int pos = -1, poc = INT_MAX;
    unsigned int i, pending = 0;

    for (i = 0; i < d->used_size; i++) {
        struct FrameStore *f = d->fs[i];

        if (f->is_output || f->pre_output)
            continue;
        /* Never expose an undecoded complementary field. */
        if (f->is_used != 3)
            return 0;
        if ((dpb->first_insert_frame == FirstInsertFrm_IDLE ||
             dpb->first_insert_frame == FirstInsertFrm_RESET) &&
            d->last_output_poc == INT_MIN) {
            dpb->first_insert_frame = FirstInsertFrm_OUT;
            dpb->first_output_poc = f->poc;
        }
        pending++;
        if (f->poc < poc) {
            poc = f->poc;
            pos = i;
        }
    }
    if (pos < 0 || (!flush && pending < dpb->reorder_output))
        return 0;
    if (dpb->first_insert_frame == FirstInsertFrm_OUT) {
        if (poc < dpb->first_output_poc)
            d->fs[pos]->data_flag |= NODISP_FLAG;
        else if (d->last_output_poc != INT_MIN)
            dpb->first_insert_frame = FirstInsertFrm_SKIPDONE;
    }
    if (meson_h264_dpb_prepare_display_buf(dpb->vdec, d->fs[pos]) < 0)
        return 0;
    d->fs[pos]->pre_output = 1;
    d->last_output_poc = poc;
    return 1;
}

static void meson_h264_dpb_flush_dpb(struct h264_dpb_stru *p_H264_Dpb)
{
	
	struct DecodedPictureBuffer *p_Dpb = &p_H264_Dpb->mDPB;
	unsigned int i;

	meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
		"%s\n", __func__);

	if (!p_Dpb->init_done)
		return;

	for (i = 0; i < p_Dpb->used_size; i++) {
		meson_h264_dpb_unmark_for_reference(p_Dpb, p_Dpb->fs[i]);

	}

	while (meson_h264_dpb_remove_unused_frame_from_dpb(p_H264_Dpb))
		;

	while (meson_h264_dpb_output_frames(p_H264_Dpb, 1))
		;

	p_Dpb->last_output_poc = INT_MIN;
}

static int meson_h264_dpb_is_short_term_reference(struct DecodedPictureBuffer *p_Dpb,
				   struct FrameStore *fs)
{
	struct h264_dpb_stru *p_H264_Dpb = container_of(p_Dpb,
			struct h264_dpb_stru, mDPB);
	if (fs->is_used == 3) { 
		if ((fs->frame->used_for_reference) &&
			(!fs->frame->is_long_term)) {
			meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
				PRINT_FLAG_DPB_DETAIL, "[[%s 1]]",
				__func__);
			return 1;
		}
	}

	if (fs->is_used & 1) { 
		if (fs->top_field) {
			if ((fs->top_field->used_for_reference) &&
				(!fs->top_field->is_long_term)) {
				meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
				PRINT_FLAG_DPB_DETAIL, "[[%s 2]]",
				__func__);
				return 1;
			}
		}
	}

	if (fs->is_used & 2) { 
		if (fs->bottom_field) {
			if ((fs->bottom_field->used_for_reference) &&
			    (!fs->bottom_field->is_long_term)) {
				meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
				PRINT_FLAG_DPB_DETAIL, "[[%s 3]]",
				__func__);
				return 1;
			}
		}
	}
	return 0;
}

static int meson_h264_dpb_is_long_term_reference(struct FrameStore *fs)
{

	if (fs->is_used == 3) { 
		if ((fs->frame->used_for_reference) &&
			(fs->frame->is_long_term)) {
			return 1;
		}
	}

	if (fs->is_used & 1) { 
		if (fs->top_field) {
			if ((fs->top_field->used_for_reference) &&
				(fs->top_field->is_long_term)) {
				return 1;
			}
		}
	}

	if (fs->is_used & 2) { 
		if (fs->bottom_field) {
			if ((fs->bottom_field->used_for_reference) &&
			    (fs->bottom_field->is_long_term)) {
				return 1;
			}
		}
	}
	return 0;
}

static void meson_h264_dpb_update_ref_list(struct DecodedPictureBuffer *p_Dpb)
{
	unsigned int i, j;

	struct h264_dpb_stru *p_H264_Dpb = container_of(p_Dpb,
		struct h264_dpb_stru, mDPB);
	meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
		"%s (%d, %d)\n", __func__, p_Dpb->size, p_Dpb->used_size);
	for (i = 0, j = 0; i < p_Dpb->used_size; i++) {
		meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
			  "fs[%d]: fs %p frame %p is_reference %d %d %d\n",
			  i, p_Dpb->fs[i], p_Dpb->fs[i]->frame,
			  p_Dpb->fs[i]->frame != NULL ?
			  p_Dpb->fs[i]->frame->used_for_reference : 0,
			  p_Dpb->fs[i]->top_field != NULL ?
			  p_Dpb->fs[i]->top_field->used_for_reference :
			  0,
			  p_Dpb->fs[i]->bottom_field != NULL ?
			  p_Dpb->fs[i]->bottom_field->used_for_reference : 0);
		if (meson_h264_dpb_is_short_term_reference(p_Dpb, p_Dpb->fs[i])) {
			meson_h264_dpb_dpb_print_cont(p_H264_Dpb->decoder_index,
				PRINT_FLAG_DPB_DETAIL,
			"fs_ref[%d]=fs[%d]: fs %p\n", j, i, p_Dpb->fs[i]);
			p_Dpb->fs_ref[j++] = p_Dpb->fs[i];
		}
	}

	p_Dpb->ref_frames_in_buffer = j;
	meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
		"%s dpb size is %d, %d\n", __func__, p_Dpb->size, j);
	while (j < p_Dpb->size) {

		p_Dpb->fs_ref[j++] = NULL;
	}
	for (i = 0; i < DPB_SIZE_MAX; i++) {
		if (p_Dpb->fs_ref[i] == NULL)
			p_Dpb->fs_ref[i] = &dummy_fs;
	}
}

static void meson_h264_dpb_update_ltref_list(struct DecodedPictureBuffer *p_Dpb)
{
	unsigned int i, j;
	struct h264_dpb_stru *p_H264_Dpb = container_of(p_Dpb,
		struct h264_dpb_stru, mDPB);

	meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
		"%s\n", __func__);
	for (i = 0, j = 0; i < p_Dpb->used_size; i++) {
		if (meson_h264_dpb_is_long_term_reference(p_Dpb->fs[i]))
			p_Dpb->fs_ltref[j++] = p_Dpb->fs[i];
	}

	p_Dpb->ltref_frames_in_buffer = j;

	while (j < p_Dpb->size)
		p_Dpb->fs_ltref[j++] = NULL;
	for (i = 0; i < DPB_SIZE_MAX; i++) {
		if (p_Dpb->fs_ltref[i] == NULL)
			p_Dpb->fs_ltref[i] = &dummy_fs;
	}
}

static void meson_h264_dpb_idr_memory_management(struct h264_dpb_stru *p_H264_Dpb,
				  struct StorablePicture *p)
{
	struct DecodedPictureBuffer *p_Dpb = &p_H264_Dpb->mDPB;

	meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
	"%s ref_frames_in_buffer %d ltref_frames_in_buffer %d\n",
	__func__, p_Dpb->ref_frames_in_buffer,
	p_Dpb->ltref_frames_in_buffer);

	if (p->no_output_of_prior_pics_flag) {
		int i;
		for (i = 0; i < p_Dpb->used_size; i++) {
			meson_h264_dpb_unmark_for_reference(p_Dpb, p_Dpb->fs[i]);
			if (p_Dpb->fs[i]->is_long_term)
				meson_h264_dpb_unmark_for_long_term_reference(p_Dpb->fs[i]);
			if (!p_Dpb->fs[i]->is_output && !p_Dpb->fs[i]->pre_output)
				meson_h264_dpb_set_frame_output_flag(p_H264_Dpb,
					p_Dpb->fs[i]->index);
		}
	} else {
		meson_h264_dpb_flush_dpb(p_H264_Dpb);
	}
	p_Dpb->last_picture = NULL;

	meson_h264_dpb_update_ref_list(p_Dpb);
	meson_h264_dpb_update_ltref_list(p_Dpb);
	p_Dpb->last_output_poc = INT_MIN;

	if (p->long_term_reference_flag) {
		p_Dpb->max_long_term_pic_idx = 0;
		p->is_long_term           = 1;
		p->long_term_frame_idx    = 0;
	} else {
		p_Dpb->max_long_term_pic_idx = -1;
		p->is_long_term           = 0;
	}

}

static void meson_h264_dpb_sliding_window_memory_management(
		struct DecodedPictureBuffer *p_Dpb,
		struct StorablePicture *p)
{
	unsigned int i;
	struct h264_dpb_stru *p_H264_Dpb = container_of(p_Dpb,
		struct h264_dpb_stru, mDPB);
	unsigned char slide_flag = 0;
	unsigned int sliding_margin = imax(
		1, p_Dpb->num_ref_frames) - p_Dpb->ltref_frames_in_buffer;
	
	meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
	"%s ref_frames_in_buffer %d ltref_frames_in_buffer %d\n",
	__func__, p_Dpb->ref_frames_in_buffer,
	p_Dpb->ltref_frames_in_buffer);

	if (p_Dpb->ref_frames_in_buffer == sliding_margin)
		slide_flag = 1;

	if (slide_flag) {
		for (i = 0; i < p_Dpb->used_size; i++) {
			if (p_Dpb->fs[i]->is_reference &&
				(!(p_Dpb->fs[i]->is_long_term))) {
				meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
				PRINT_FLAG_DPB_DETAIL, "unmark %d\n", i);
				meson_h264_dpb_unmark_for_reference(p_Dpb, p_Dpb->fs[i]);
				meson_h264_dpb_update_ref_list(p_Dpb);
				break;
			}
		}
	}

	p->is_long_term = 0;
}

static void meson_h264_dpb_check_num_ref(struct DecodedPictureBuffer *p_Dpb)
{
	if ((int)(p_Dpb->ltref_frames_in_buffer +
			p_Dpb->ref_frames_in_buffer) >
			imax(1, p_Dpb->num_ref_frames)) {
		struct h264_dpb_stru *p_H264_Dpb = container_of(p_Dpb,
					 struct h264_dpb_stru, mDPB);
		meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
			  "Max. number of reference frames exceeded. Invalid stream. lt %d ref %d mum_ref %d\n",
			  p_Dpb->ltref_frames_in_buffer,
			  p_Dpb->ref_frames_in_buffer,
			  p_Dpb->num_ref_frames);
	}
}

static void meson_h264_dpb_dump_dpb(struct DecodedPictureBuffer *d, u8 force)
{
    if (force)
        pr_debug("meson-h264: DPB used=%u refs=%u long=%u last_poc=%d\n",
                 d->used_size, d->ref_frames_in_buffer,
                 d->ltref_frames_in_buffer, d->last_output_poc);
}

static int meson_h264_dpb_get_pic_num_x(struct StorablePicture *p,
			 int difference_of_pic_nums_minus1)
{
	int currPicNum;

	if (p->structure == FRAME)
		currPicNum = p->frame_num;
	else
		currPicNum = 2 * p->frame_num + 1;

	return currPicNum - (difference_of_pic_nums_minus1 + 1);
}

static void meson_h264_dpb_mm_unmark_short_term_for_reference(struct DecodedPictureBuffer
		*p_Dpb, struct StorablePicture *p,
		int difference_of_pic_nums_minus1)
{
	struct h264_dpb_stru *p_H264_Dpb =
		container_of(p_Dpb, struct h264_dpb_stru, mDPB);
	int picNumX;

	unsigned int i;

	picNumX = meson_h264_dpb_get_pic_num_x(p, difference_of_pic_nums_minus1);

	for (i = 0; i < p_Dpb->ref_frames_in_buffer; i++) {
		if (p_Dpb->fs_ref[i] == NULL) {
			p_H264_Dpb->dpb_error_flag = __LINE__;
			continue;
		}
		if (p->structure == FRAME) {
			if ((p_Dpb->fs_ref[i]->is_reference == 3) &&
			    (p_Dpb->fs_ref[i]->is_long_term == 0)) {
				if (p_Dpb->fs_ref[i]->frame == NULL) {
					p_H264_Dpb->dpb_error_flag = __LINE__;
					continue;
				}
				if (p_Dpb->fs_ref[i]->frame->pic_num ==
					picNumX) {
					meson_h264_dpb_unmark_for_reference(p_Dpb,
						p_Dpb->fs_ref[i]);
					return;
				}
			}
		} else {
			if ((p_Dpb->fs_ref[i]->is_reference & 1) &&
			    (!(p_Dpb->fs_ref[i]->is_long_term & 1))) {
				if (p_Dpb->fs_ref[i]->top_field == NULL) {
					p_H264_Dpb->dpb_error_flag = __LINE__;
					continue;
				}
				if (p_Dpb->fs_ref[i]->top_field->pic_num ==
					picNumX) {
					p_Dpb->fs_ref[i]->
					top_field->used_for_reference = 0;
					p_Dpb->fs_ref[i]->is_reference &= 2;
					if ((p_Dpb->fs_ref[i]->is_used == 3)
						&& p_Dpb->fs_ref[i]->frame
					) {
						p_Dpb->fs_ref[i]->frame->
							used_for_reference = 0;
					}
					return;
				}
			}
			if ((p_Dpb->fs_ref[i]->is_reference & 2) &&
			    (!(p_Dpb->fs_ref[i]->is_long_term & 2))) {
				if (p_Dpb->fs_ref[i]->bottom_field == NULL) {
					p_H264_Dpb->dpb_error_flag = __LINE__;
					continue;
				}
				if (p_Dpb->fs_ref[i]->bottom_field->pic_num ==
					picNumX) {
					p_Dpb->fs_ref[i]->bottom_field->
					used_for_reference = 0;
					p_Dpb->fs_ref[i]->is_reference &= 1;
					if ((p_Dpb->fs_ref[i]->is_used == 3)
						&& p_Dpb->fs_ref[i]->frame
						) {
						p_Dpb->fs_ref[i]->frame->
						used_for_reference = 0;
					}
					return;
				}
			}
		}
	}
}

static void meson_h264_dpb_mm_unmark_long_term_for_reference(struct DecodedPictureBuffer
		*p_Dpb, struct StorablePicture *p, int long_term_pic_num)
{
	unsigned int i;

	for (i = 0; i < p_Dpb->ltref_frames_in_buffer; i++) {
		if (p->structure == FRAME) {
			if ((p_Dpb->fs_ltref[i]->is_reference == 3) &&
			    (p_Dpb->fs_ltref[i]->is_long_term == 3)) {
				if (p_Dpb->fs_ltref[i]->frame->
					long_term_pic_num ==
					long_term_pic_num) {
					meson_h264_dpb_unmark_for_long_term_reference(
						p_Dpb->fs_ltref[i]);
				}
			}
		} else {
			if ((p_Dpb->fs_ltref[i]->is_reference & 1) &&
			    ((p_Dpb->fs_ltref[i]->is_long_term & 1))) {
				if (p_Dpb->fs_ltref[i]->top_field->
					long_term_pic_num ==
					long_term_pic_num) {
					p_Dpb->fs_ltref[i]->top_field->
						used_for_reference = 0;
					p_Dpb->fs_ltref[i]->top_field->
						is_long_term = 0;
					p_Dpb->fs_ltref[i]->is_reference &= 2;
					p_Dpb->fs_ltref[i]->is_long_term &= 2;
					if (p_Dpb->fs_ltref[i]->is_used == 3) {
						p_Dpb->fs_ltref[i]->frame->
							used_for_reference = 0;
						p_Dpb->fs_ltref[i]->frame->
							is_long_term = 0;
					}
					return;
				}
			}
			if ((p_Dpb->fs_ltref[i]->is_reference & 2) &&
			    ((p_Dpb->fs_ltref[i]->is_long_term & 2))) {
				if (p_Dpb->fs_ltref[i]->bottom_field->
					long_term_pic_num ==
					long_term_pic_num) {
					p_Dpb->fs_ltref[i]->bottom_field->
						used_for_reference = 0;
					p_Dpb->fs_ltref[i]->bottom_field->
						is_long_term = 0;
					p_Dpb->fs_ltref[i]->is_reference &= 1;
					p_Dpb->fs_ltref[i]->is_long_term &= 1;
					if (p_Dpb->fs_ltref[i]->is_used == 3) {
						p_Dpb->fs_ltref[i]->frame->
							used_for_reference = 0;
						p_Dpb->fs_ltref[i]->frame->
							is_long_term = 0;
					}
					return;
				}
			}
		}
	}
}

static void meson_h264_dpb_unmark_long_term_frame_for_reference_by_frame_idx(
	struct DecodedPictureBuffer *p_Dpb, int long_term_frame_idx)
{
	unsigned int i;

	for (i = 0; i < p_Dpb->ltref_frames_in_buffer; i++) {
		if (p_Dpb->fs_ltref[i]->long_term_frame_idx ==
			long_term_frame_idx)
			meson_h264_dpb_unmark_for_long_term_reference(p_Dpb->fs_ltref[i]);
	}
}

static void meson_h264_dpb_unmark1(struct DecodedPictureBuffer *p_Dpb,
	unsigned int curr_frame_num, int i)
{
	if (p_Dpb->last_picture) {

			meson_h264_dpb_unmark_for_long_term_reference(p_Dpb->fs_ltref[i]);

	}
}

static void meson_h264_dpb_unmark2(struct DecodedPictureBuffer *p_Dpb,
	int curr_pic_num, int i)
{
	if ((p_Dpb->fs_ltref[i]->frame_num) !=
		(unsigned int)(curr_pic_num >> 1))
		meson_h264_dpb_unmark_for_long_term_reference(p_Dpb->fs_ltref[i]);
}

static void meson_h264_dpb_unmark3_top(struct DecodedPictureBuffer *p_Dpb,
	unsigned int curr_frame_num, int curr_pic_num, int mark_current, int i)
{
	if (p_Dpb->fs_ltref[i]->is_long_term == 3) {
		meson_h264_dpb_unmark_for_long_term_reference(p_Dpb->fs_ltref[i]);
	} else {
		if (p_Dpb->fs_ltref[i]->is_long_term == 1) {
			meson_h264_dpb_unmark_for_long_term_reference(p_Dpb->fs_ltref[i]);
		} else {
			if (mark_current)
				meson_h264_dpb_unmark1(p_Dpb, curr_frame_num, i);
			else
				meson_h264_dpb_unmark2(p_Dpb, curr_pic_num, i);
		}
	}
}

static void meson_h264_dpb_unmark3_bottom(struct DecodedPictureBuffer *p_Dpb,
	unsigned int curr_frame_num, int curr_pic_num, int mark_current, int i)
{
	if (p_Dpb->fs_ltref[i]->is_long_term == 2) {
		meson_h264_dpb_unmark_for_long_term_reference(p_Dpb->fs_ltref[i]);
	} else {
		if (mark_current)
			meson_h264_dpb_unmark1(p_Dpb, curr_frame_num, i);
		else
			meson_h264_dpb_unmark2(p_Dpb, curr_pic_num, i);
	}
}

static void meson_h264_dpb_unmark_long_term_field_for_reference_by_frame_idx(
	struct DecodedPictureBuffer *p_Dpb, enum PictureStructure structure,
	int long_term_frame_idx, int mark_current, unsigned int curr_frame_num,
	int curr_pic_num)
{
	struct VideoParameters *p_Vid = p_Dpb->p_Vid;
	unsigned int i;

	if (curr_pic_num < 0)
		curr_pic_num += (2 * p_Vid->max_frame_num);

	for (i = 0; i < p_Dpb->ltref_frames_in_buffer; i++) {
		if (p_Dpb->fs_ltref[i]->long_term_frame_idx ==
			long_term_frame_idx) {
			if (structure == TOP_FIELD)
				meson_h264_dpb_unmark3_top(p_Dpb, curr_frame_num,
					curr_pic_num, mark_current, i);

			if (structure == BOTTOM_FIELD)
				meson_h264_dpb_unmark3_bottom(p_Dpb, curr_frame_num,
					curr_pic_num, mark_current, i);
		}
	}
}

static void meson_h264_dpb_mark_pic_long_term(struct DecodedPictureBuffer *p_Dpb,
			       struct StorablePicture *p,
			       int long_term_frame_idx, int picNumX)
{
	struct h264_dpb_stru *p_H264_Dpb = container_of(p_Dpb,
				struct h264_dpb_stru, mDPB);
	unsigned int i;
	int add_top, add_bottom;

	if (p->structure == FRAME) {
		for (i = 0; i < p_Dpb->ref_frames_in_buffer; i++) {
			if (p_Dpb->fs_ref[i] == NULL) {
				p_H264_Dpb->dpb_error_flag = __LINE__;
				continue;
			}
			if (p_Dpb->fs_ref[i]->is_reference == 3) {
				if (p_Dpb->fs_ref[i]->frame == NULL) {
					p_H264_Dpb->dpb_error_flag = __LINE__;
					continue;
				}
				if ((!p_Dpb->fs_ref[i]->frame->
					is_long_term) &&
				    (p_Dpb->fs_ref[i]->frame->pic_num ==
					picNumX)) {
					p_Dpb->fs_ref[i]->
						long_term_frame_idx =
					p_Dpb->fs_ref[i]->frame->
						long_term_frame_idx =
						long_term_frame_idx;
					p_Dpb->fs_ref[i]->frame->
						long_term_pic_num =
						long_term_frame_idx;
					p_Dpb->fs_ref[i]->frame->
						is_long_term = 1;

					if (p_Dpb->fs_ref[i]->top_field &&
					    p_Dpb->fs_ref[i]->bottom_field) {
						p_Dpb->fs_ref[i]->top_field->
						long_term_frame_idx =
							p_Dpb->fs_ref[i]->
							bottom_field->
							long_term_frame_idx =
							long_term_frame_idx;
						p_Dpb->fs_ref[i]->top_field->
							long_term_pic_num =
							long_term_frame_idx;
						p_Dpb->fs_ref[i]->
							bottom_field->
							long_term_pic_num =
							long_term_frame_idx;

						p_Dpb->fs_ref[i]->top_field->
							is_long_term =
							p_Dpb->fs_ref[i]->
							bottom_field->
							is_long_term
							= 1;

					}
					p_Dpb->fs_ref[i]->is_long_term = 3;
					return;
				}
			}
		}
		meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
			  "Warning: reference frame for long term marking not found\n");
	} else {
		if (p->structure == TOP_FIELD) {
			add_top    = 1;
			add_bottom = 0;
		} else {
			add_top    = 0;
			add_bottom = 1;
		}
		for (i = 0; i < p_Dpb->ref_frames_in_buffer; i++) {
			if (p_Dpb->fs_ref[i] == NULL) {
				p_H264_Dpb->dpb_error_flag = __LINE__;
				continue;
			}
			if (p_Dpb->fs_ref[i]->is_reference & 1) {
				if (p_Dpb->fs_ref[i]->top_field == NULL) {
					p_H264_Dpb->dpb_error_flag = __LINE__;
					continue;
				}
				if ((!p_Dpb->fs_ref[i]->top_field->
					is_long_term) &&
				    (p_Dpb->fs_ref[i]->top_field->pic_num ==
					picNumX)) {
					if ((p_Dpb->fs_ref[i]->
						is_long_term) &&
					    (p_Dpb->fs_ref[i]->
						long_term_frame_idx !=
						long_term_frame_idx)) {
						meson_h264_dpb_dpb_print(p_H264_Dpb->
						decoder_index,
						PRINT_FLAG_DPB_DETAIL,
						"Warning: assigning long_term_frame_idx different from other field\n");
					}

					p_Dpb->fs_ref[i]->
						long_term_frame_idx =
						p_Dpb->fs_ref[i]->top_field->
						long_term_frame_idx
						= long_term_frame_idx;
					p_Dpb->fs_ref[i]->top_field->
						long_term_pic_num =
						2 * long_term_frame_idx +
						add_top;
					p_Dpb->fs_ref[i]->top_field->
						is_long_term = 1;
					p_Dpb->fs_ref[i]->is_long_term |= 1;
					if ((p_Dpb->fs_ref[i]->is_long_term
						== 3)
						&& p_Dpb->fs_ref[i]->frame
						) {
						p_Dpb->fs_ref[i]->frame->
							is_long_term = 1;
						p_Dpb->fs_ref[i]->frame->
							long_term_frame_idx =
							p_Dpb->fs_ref[i]->
							frame->
							long_term_pic_num =
							long_term_frame_idx;
					}
					return;
				}
			}
			if (p_Dpb->fs_ref[i]->is_reference & 2) {
				if (p_Dpb->fs_ref[i]->bottom_field == NULL) {
					p_H264_Dpb->dpb_error_flag = __LINE__;
					continue;
				}
				if ((!p_Dpb->fs_ref[i]->bottom_field->
					is_long_term) &&
				    (p_Dpb->fs_ref[i]->bottom_field->pic_num
					== picNumX)) {
					if ((p_Dpb->fs_ref[i]->
						is_long_term) &&
					    (p_Dpb->fs_ref[i]->
						long_term_frame_idx !=
						long_term_frame_idx)) {
						meson_h264_dpb_dpb_print(p_H264_Dpb->
						decoder_index,
						PRINT_FLAG_DPB_DETAIL,
						"Warning: assigning long_term_frame_idx different from other field\n");
					}

					p_Dpb->fs_ref[i]->
						long_term_frame_idx =
						p_Dpb->fs_ref[i]->bottom_field
						->long_term_frame_idx
						= long_term_frame_idx;
					p_Dpb->fs_ref[i]->bottom_field->
						long_term_pic_num = 2 *
						long_term_frame_idx +
						add_bottom;
					p_Dpb->fs_ref[i]->bottom_field->
						is_long_term = 1;
					p_Dpb->fs_ref[i]->is_long_term |= 2;
					if ((p_Dpb->fs_ref[i]->
						is_long_term == 3)
						&& p_Dpb->fs_ref[i]->frame
						) {
						p_Dpb->fs_ref[i]->frame->
							is_long_term = 1;
						p_Dpb->fs_ref[i]->frame->
							long_term_frame_idx =
							p_Dpb->fs_ref[i]->
							frame->
							long_term_pic_num =
							long_term_frame_idx;
					}
					return;
				}
			}
		}
		meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
			  "Warning: reference field for long term marking not found\n");
	}
}

static void meson_h264_dpb_mm_assign_long_term_frame_idx(struct DecodedPictureBuffer *p_Dpb,
		struct StorablePicture *p, int difference_of_pic_nums_minus1,
		int long_term_frame_idx)
{
	struct h264_dpb_stru *p_H264_Dpb = container_of(p_Dpb,
			struct h264_dpb_stru, mDPB);
	int picNumX = meson_h264_dpb_get_pic_num_x(p, difference_of_pic_nums_minus1);

	if (p->structure == FRAME) {
		meson_h264_dpb_unmark_long_term_frame_for_reference_by_frame_idx(p_Dpb,
				long_term_frame_idx);
	} else {
		unsigned int i;
		enum PictureStructure structure = FRAME;

		for (i = 0; i < p_Dpb->ref_frames_in_buffer; i++) {
			if (p_Dpb->fs_ref[i] == NULL) {
				p_H264_Dpb->dpb_error_flag = __LINE__;
				continue;
			}
			if (p_Dpb->fs_ref[i]->is_reference & 1) {
				if (p_Dpb->fs_ref[i]->top_field == NULL) {
					p_H264_Dpb->dpb_error_flag = __LINE__;
					continue;
				}
				if (p_Dpb->fs_ref[i]->top_field->
					pic_num == picNumX) {
					structure = TOP_FIELD;
					break;
				}
			}
			if (p_Dpb->fs_ref[i]->is_reference & 2) {
				if (p_Dpb->fs_ref[i]->bottom_field == NULL) {
					p_H264_Dpb->dpb_error_flag = __LINE__;
					continue;
				}
				if (p_Dpb->fs_ref[i]->bottom_field->
					pic_num == picNumX) {
					structure = BOTTOM_FIELD;
					break;
				}
			}
		}
		if (structure == FRAME) {
			meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
				  PRINT_FLAG_DPB_DETAIL,
				  "field for long term marking not found %d",
				  200);
		}

		meson_h264_dpb_unmark_long_term_field_for_reference_by_frame_idx(p_Dpb,
				structure,
				long_term_frame_idx, 0, 0, picNumX);
	}

	meson_h264_dpb_mark_pic_long_term(p_Dpb, p, long_term_frame_idx, picNumX);
}

static void meson_h264_dpb_mm_update_max_long_term_frame_idx(struct DecodedPictureBuffer
		*p_Dpb, int max_long_term_frame_idx_plus1)
{
	unsigned int i;

	p_Dpb->max_long_term_pic_idx = max_long_term_frame_idx_plus1 - 1;

	for (i = 0; i < p_Dpb->ltref_frames_in_buffer; i++) {
		if (p_Dpb->fs_ltref[i]->long_term_frame_idx >
			p_Dpb->max_long_term_pic_idx) {
			meson_h264_dpb_unmark_for_long_term_reference(p_Dpb->fs_ltref[i]);
		}
	}
}

static void meson_h264_dpb_mm_unmark_all_long_term_for_reference(struct DecodedPictureBuffer
		*p_Dpb)
{
	meson_h264_dpb_mm_update_max_long_term_frame_idx(p_Dpb, 0);
}

static void meson_h264_dpb_mm_unmark_all_short_term_for_reference(struct DecodedPictureBuffer
		*p_Dpb)
{
	unsigned int i;

	for (i = 0; i < p_Dpb->ref_frames_in_buffer; i++)
		meson_h264_dpb_unmark_for_reference(p_Dpb, p_Dpb->fs_ref[i]);
	meson_h264_dpb_update_ref_list(p_Dpb);
}

static void meson_h264_dpb_mm_mark_current_picture_long_term(struct DecodedPictureBuffer
		*p_Dpb, struct StorablePicture *p, int long_term_frame_idx)
{
	
	if (p->structure == FRAME) {
		meson_h264_dpb_unmark_long_term_frame_for_reference_by_frame_idx(p_Dpb,
				long_term_frame_idx);
	} else {
		meson_h264_dpb_unmark_long_term_field_for_reference_by_frame_idx(p_Dpb,
				p->structure, long_term_frame_idx,
				1, p->pic_num, 0);
	}

	p->is_long_term = 1;
	p->long_term_frame_idx = long_term_frame_idx;
}

static void meson_h264_dpb_adaptive_memory_management(struct h264_dpb_stru *p_H264_Dpb,
				       struct StorablePicture *p)
{
	struct DecodedPictureBuffer *p_Dpb = &p_H264_Dpb->mDPB;
	struct DecRefPicMarking_s *tmp_drpm;
	struct VideoParameters *p_Vid = p_Dpb->p_Vid;

	meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
				"%s\n", __func__);
	p_Vid->last_has_mmco_5 = 0;

	while (p->dec_ref_pic_marking_buffer) {
		tmp_drpm = p->dec_ref_pic_marking_buffer;
		switch (tmp_drpm->memory_management_control_operation) {
		case 0:
			if (tmp_drpm->Next != NULL)
				meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
					PRINT_FLAG_ERROR,
					"error, memory_management_control_operation = 0 not last operation in buffer\n");
			break;
		case 1:
			meson_h264_dpb_mm_unmark_short_term_for_reference(p_Dpb, p,
				tmp_drpm->difference_of_pic_nums_minus1);
			meson_h264_dpb_update_ref_list(p_Dpb);
			break;
		case 2:
			meson_h264_dpb_mm_unmark_long_term_for_reference(p_Dpb, p,
				tmp_drpm->long_term_pic_num);
			meson_h264_dpb_update_ltref_list(p_Dpb);
			break;
		case 3:
			meson_h264_dpb_mm_assign_long_term_frame_idx(p_Dpb, p,
				tmp_drpm->difference_of_pic_nums_minus1,
				tmp_drpm->long_term_frame_idx);
			meson_h264_dpb_update_ref_list(p_Dpb);
			meson_h264_dpb_update_ltref_list(p_Dpb);
			break;
		case 4:
			meson_h264_dpb_mm_update_max_long_term_frame_idx(p_Dpb,
				tmp_drpm->max_long_term_frame_idx_plus1);
			meson_h264_dpb_update_ltref_list(p_Dpb);
			break;
		case 5:
			meson_h264_dpb_mm_unmark_all_short_term_for_reference(p_Dpb);
			meson_h264_dpb_mm_unmark_all_long_term_for_reference(p_Dpb);
			p_Vid->last_has_mmco_5 = 1;
			break;
		case 6:
			meson_h264_dpb_mm_mark_current_picture_long_term(p_Dpb, p,
				tmp_drpm->long_term_frame_idx);
			meson_h264_dpb_check_num_ref(p_Dpb);
			break;
		default:
			meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
				PRINT_FLAG_ERROR,
				"error, invalid memory_management_control_operation in buffer\n");
		}
		p->dec_ref_pic_marking_buffer = tmp_drpm->Next;
		
	}
	if (p_Vid->last_has_mmco_5) {
		p->pic_num = p->frame_num = 0;

		switch (p->structure) {
		case TOP_FIELD: {
			
			p->poc = p->top_poc = 0;
			break;
		}
		case BOTTOM_FIELD: {
			
			p->poc = p->bottom_poc = 0;
			break;
		}
		case FRAME: {
			p->top_poc    -= p->poc;
			p->bottom_poc -= p->poc;

			p->poc = imin(p->top_poc, p->bottom_poc);
			
			break;
		}
		}
		p->frame_poc = p->poc;
		
		meson_h264_dpb_flush_dpb(p_H264_Dpb);
	}
}

static int meson_h264_dpb_store_picture_in_dpb(struct h264_dpb_stru *p_H264_Dpb,
			  struct StorablePicture *p,
			  unsigned char data_flag)
{
	
	struct VideoParameters *p_Vid = &p_H264_Dpb->mVideo;
	struct DecodedPictureBuffer *p_Dpb = &p_H264_Dpb->mDPB;
	struct FrameStore *pair = NULL;
	unsigned int i, frame_outside_count = 0;
	meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
		"%s p_Vid %p\n", __func__, p_Vid);

	/* Header processing already paired by frame_num, parity and reference
	 * class. Retain that surface identity across MMCO5's frame_num reset.
	 */
	if (!p->buf_spec_is_alloced) {
		pair = p_Dpb->last_picture;
		if (!pair || pair->buf_spec_num != p->buf_spec_num ||
		    pair->colocated_buf_index != p->colocated_buf_index ||
		    !((p->structure == TOP_FIELD && pair->is_used == 2) ||
		      (p->structure == BOTTOM_FIELD && pair->is_used == 1)))
			return -EILSEQ;
	}

	p_Vid->last_has_mmco_5 = 0;
	p_Vid->last_pic_bottom_field = (p->structure == BOTTOM_FIELD);
	if (p->idr_flag) {
		if (!pair)
			meson_h264_dpb_idr_memory_management(p_H264_Dpb, p);
		else {
			/* IDR marking belongs to the pair, not a second DPB flush. */
			p->is_long_term = p->long_term_reference_flag;
			p->long_term_frame_idx = 0;
		}
		if (p_H264_Dpb->first_insert_frame == FirstInsertFrm_OUT)
			p_H264_Dpb->first_insert_frame = FirstInsertFrm_SKIPDONE;
	} else {

		if (p->used_for_reference &&
			(p->adaptive_ref_pic_buffering_flag))
			meson_h264_dpb_adaptive_memory_management(p_H264_Dpb, p);
	}

	if (pair) {
		meson_h264_dpb_insert_picture_in_dpb(p_H264_Dpb, pair, p, data_flag);
		meson_h264_dpb_update_ref_list(p_Dpb);
		meson_h264_dpb_update_ltref_list(p_Dpb);
		p_Dpb->last_picture = NULL;
		p_Dpb->first_pic_done = 1;
		while (meson_h264_dpb_output_frames(p_H264_Dpb, 0))
			;
		return 0;
	}

	if ((!p->idr_flag) && (p->used_for_reference &&
			       (!p->adaptive_ref_pic_buffering_flag))) {
		meson_h264_dpb_sliding_window_memory_management(p_Dpb, p);
	}

	if (p_Vid->conceal_mode != 0) {
		for (i = 0; i < p_Dpb->size; i++)
			if (p_Dpb->fs[i]->is_reference)
				p_Dpb->fs[i]->concealment_reference = 1;
	}

	while (meson_h264_dpb_remove_unused_frame_from_dpb(p_H264_Dpb))
		;

	while (meson_h264_dpb_output_frames(p_H264_Dpb, 0))
		;

	if ((p->used_for_reference) && (!p->is_long_term)) {
		for (i = 0; i < p_Dpb->ref_frames_in_buffer; i++) {
			if (p_Dpb->fs_ref[i] == NULL)
				continue;
			if (p_Dpb->fs_ref[i]->frame_num == p->frame_num) {
				meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
					  PRINT_FLAG_DPB_DETAIL,
					  "duplicate frame_num in short-term reference picture buffer %d\n",
					   500);
				if (p_Dpb->fs_ref[i]->dpb_frame_count == p_H264_Dpb->dpb_frame_count) {
					meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
							  0, "duplicate frame, no insert to dpb\n");
					return -2;
				} else {
					meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
						  0, "duplicate frame_num release defore ref\n");
					meson_h264_dpb_unmark_for_reference(p_Dpb, p_Dpb->fs_ref[i]);
					meson_h264_dpb_update_ref_list(p_Dpb);
				}
			}
		}
	}

	meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
		  "%s p_Dpb->used_size %d\n", __func__, p_Dpb->used_size);
	if (p_Dpb->used_size >= p_Dpb->size) {
		meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
			PRINT_FLAG_ERROR,
			"%s Error: used_sizd %d is large than dpb size\r\n",
			__func__, p_Dpb->used_size);
		
		meson_h264_dpb_dump_dpb(p_Dpb, 0);
		return -1;
	}

	meson_h264_dpb_insert_picture_in_dpb(p_H264_Dpb, p_Dpb->fs[p_Dpb->used_size],
		p, data_flag);

	if (p->idr_flag)
		p_Vid->earlier_missing_poc = 0;

	if (p->structure != FRAME)
		p_Dpb->last_picture = p_Dpb->fs[p_Dpb->used_size];
	else
		p_Dpb->last_picture = NULL;

	p_Dpb->used_size++;
	meson_h264_dpb_update_ref_list(p_Dpb);
	meson_h264_dpb_update_ltref_list(p_Dpb);

	meson_h264_dpb_check_num_ref(p_Dpb);
	for (i = 0; i < p_Dpb->used_size; i++) {
		if (p_Dpb->fs[i]->pre_output)
			frame_outside_count++;
	}

	if (p_H264_Dpb->fast_output_enable == H264_OUTPUT_MODE_FAST)
		i = 1;
	else
		i = 0;

	if (i || (p_H264_Dpb->first_insert_frame < FirstInsertFrm_SKIPDONE)) {
		while (meson_h264_dpb_output_frames(p_H264_Dpb, i))
			;
	}

	meson_h264_dpb_dump_dpb(p_Dpb, 0);
	p_Dpb->first_pic_done = 1; 

	return 0;
}

static void meson_h264_dpb_bufmgr_post(struct h264_dpb_stru *p_H264_Dpb)
{
	
	struct VideoParameters *p_Vid = &p_H264_Dpb->mVideo;

	if (p_Vid->last_has_mmco_5) {
		struct StorablePicture *pic = p_Vid->dec_picture;

		p_Vid->pre_frame_num = 0;
		/* Preserve the post-MMCO5 POC, not the next slice's scratch POC. */
		p_Vid->PrevPicOrderCntMsb = 0;
		p_Vid->PrevPicOrderCntLsb = p_Vid->last_pic_bottom_field ?
			0 : pic->top_poc;
	}
}






static void meson_h264_dpb_qsort(void *base, size_t num, size_t width,
                  int (*compare)(const void *, const void *))
{
    sort(base, num, width, compare, NULL);
}

static inline int meson_h264_dpb_compare_pic_by_pic_num_desc(const void *arg1,
		const void *arg2)
{
	int pic_num1 = (*(struct StorablePicture **)arg1)->pic_num;
	int pic_num2 = (*(struct StorablePicture **)arg2)->pic_num;

	if (pic_num1 < pic_num2)
		return 1;
	if (pic_num1 > pic_num2)
		return -1;
	else
		return 0;
}

static inline int meson_h264_dpb_compare_pic_by_lt_pic_num_asc(const void *arg1,
		const void *arg2)
{
	int long_term_pic_num1 =
		(*(struct StorablePicture **)arg1)->long_term_pic_num;
	int long_term_pic_num2 =
		(*(struct StorablePicture **)arg2)->long_term_pic_num;

	if (long_term_pic_num1 < long_term_pic_num2)
		return -1;
	if (long_term_pic_num1 > long_term_pic_num2)
		return 1;
	else
		return 0;
}

static inline int meson_h264_dpb_compare_fs_by_frame_num_desc(const void *arg1,
		const void *arg2)
{
	int frame_num_wrap1 = (*(struct FrameStore **)arg1)->frame_num_wrap;
	int frame_num_wrap2 = (*(struct FrameStore **)arg2)->frame_num_wrap;

	if (frame_num_wrap1 < frame_num_wrap2)
		return 1;
	if (frame_num_wrap1 > frame_num_wrap2)
		return -1;
	else
		return 0;
}

static inline int meson_h264_dpb_compare_fs_by_lt_pic_idx_asc(const void *arg1,
		const void *arg2)
{
	int long_term_frame_idx1 =
		(*(struct FrameStore **)arg1)->long_term_frame_idx;
	int long_term_frame_idx2 =
		(*(struct FrameStore **)arg2)->long_term_frame_idx;

	if (long_term_frame_idx1 < long_term_frame_idx2)
		return -1;
	else if (long_term_frame_idx1 > long_term_frame_idx2)
		return 1;
	else
		return 0;
}

static inline int meson_h264_dpb_compare_pic_by_poc_asc(const void *arg1, const void *arg2)
{
	int poc1 = (*(struct StorablePicture **)arg1)->poc;
	int poc2 = (*(struct StorablePicture **)arg2)->poc;

	if (poc1 < poc2)
		return -1;
	else if (poc1 > poc2)
		return 1;
	else
		return 0;
}

static inline int meson_h264_dpb_compare_pic_by_poc_desc(const void *arg1, const void *arg2)
{
	int poc1 = (*(struct StorablePicture **)arg1)->poc;
	int poc2 = (*(struct StorablePicture **)arg2)->poc;

	if (poc1 < poc2)
		return 1;
	else if (poc1 > poc2)
		return -1;
	else
		return 0;
}

static inline int meson_h264_dpb_compare_fs_by_poc_asc(const void *arg1, const void *arg2)
{
	int poc1 = (*(struct FrameStore **)arg1)->poc;
	int poc2 = (*(struct FrameStore **)arg2)->poc;

	if (poc1 < poc2)
		return -1;
	else if (poc1 > poc2)
		return 1;
	else
		return 0;
}

static inline int meson_h264_dpb_compare_fs_by_poc_desc(const void *arg1, const void *arg2)
{
	int poc1 = (*(struct FrameStore **)arg1)->poc;
	int poc2 = (*(struct FrameStore **)arg2)->poc;

	if (poc1 < poc2)
		return 1;
	else if (poc1 > poc2)
		return -1;
	else
		return 0;
}

static inline int meson_h264_dpb_is_short_ref(struct StorablePicture *s)
{
	return (s &&
		(s->used_for_reference) && (!(s->is_long_term)));
}

static inline int meson_h264_dpb_is_long_ref(struct StorablePicture *s)
{
	return (s &&
		s->used_for_reference) && (s->is_long_term);
}

static void meson_h264_dpb_gen_pic_list_from_frame_list(enum PictureStructure currStructure,
		struct FrameStore **fs_list, int list_idx,
		struct StorablePicture **list,
		char *list_size, int long_term)
{
	int top_idx = 0;
	int bot_idx = 0;

	int (*is_ref)(struct StorablePicture *s) = (long_term) ? meson_h264_dpb_is_long_ref :
			meson_h264_dpb_is_short_ref;

	if (currStructure == TOP_FIELD) {
		while ((top_idx < list_idx) || (bot_idx < list_idx)) {
			for (; top_idx < list_idx; top_idx++) {
				if (fs_list[top_idx]->is_used & 1) {
					if (is_ref(fs_list[top_idx]->
						top_field)) {
						
						list[(short) *list_size] =
						fs_list[top_idx]->top_field;
						(*list_size)++;
						top_idx++;
						break;
					}
				}
			}
			for (; bot_idx < list_idx; bot_idx++) {
				if (fs_list[bot_idx]->is_used & 2) {
					if (is_ref(fs_list[bot_idx]->
						bottom_field)) {
						
						list[(short) *list_size] =
						fs_list[bot_idx]->bottom_field;
						(*list_size)++;
						bot_idx++;
						break;
					}
				}
			}
		}
	}
	if (currStructure == BOTTOM_FIELD) {
		while ((top_idx < list_idx) || (bot_idx < list_idx)) {
			for (; bot_idx < list_idx; bot_idx++) {
				if (fs_list[bot_idx]->is_used & 2) {
					if (is_ref(fs_list[bot_idx]->
						bottom_field)) {
						
						list[(short) *list_size] =
						fs_list[bot_idx]->bottom_field;
						(*list_size)++;
						bot_idx++;
						break;
					}
				}
			}
			for (; top_idx < list_idx; top_idx++) {
				if (fs_list[top_idx]->is_used & 1) {
					if (is_ref(fs_list[top_idx]->
						top_field)) {
						
						list[(short) *list_size] =
						fs_list[top_idx]->top_field;
						(*list_size)++;
						top_idx++;
						break;
					}
				}
			}
		}
	}
}

static void meson_h264_dpb_init_lists_p_slice(struct Slice *currSlice)
{
	struct VideoParameters *p_Vid = currSlice->p_Vid;
	struct DecodedPictureBuffer *p_Dpb = currSlice->p_Dpb;
	struct h264_dpb_stru *p_H264_Dpb = container_of(p_Dpb,
				struct h264_dpb_stru, mDPB);

	unsigned int i;

	int list0idx = 0;
	int listltidx = 0;

	struct FrameStore **fs_list0;
	struct FrameStore **fs_listlt;

	if (currSlice->structure == FRAME) {
		for (i = 0; i < p_Dpb->ref_frames_in_buffer; i++) {
			if (p_Dpb->fs_ref[i] == NULL ||
				p_Dpb->fs_ref[i]->frame == NULL) {
				p_H264_Dpb->dpb_error_flag = __LINE__;
				continue;
			}
			if (p_Dpb->fs_ref[i]->is_used == 3) {
				if ((p_Dpb->fs_ref[i]->frame->
					used_for_reference) &&
				    (!p_Dpb->fs_ref[i]->frame->
					is_long_term)) {
					currSlice->listX[0][list0idx++] =
					p_Dpb->fs_ref[i]->frame;
				}
			}
		}
		
		meson_h264_dpb_qsort((void *)currSlice->listX[0], list0idx,
			sizeof(struct StorablePicture *),
			meson_h264_dpb_compare_pic_by_pic_num_desc);
		currSlice->listXsize[0] = (char) list0idx;
		CHECK_VALID(currSlice->listXsize[0], 0);
		if (0U & PRINT_FLAG_DPB_DETAIL) {
			meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
				PRINT_FLAG_DPB_DETAIL,
				  "listX[0] (PicNum): ");
			for (i = 0; i < list0idx; i++) {
				meson_h264_dpb_dpb_print_cont(p_H264_Dpb->decoder_index,
					PRINT_FLAG_DPB_DETAIL, "%d  ",
					currSlice->listX[0][i]->pic_num);
			}
			meson_h264_dpb_dpb_print_cont(p_H264_Dpb->decoder_index,
				PRINT_FLAG_DPB_DETAIL, "\n");
		}
		
		for (i = 0; i < p_Dpb->ltref_frames_in_buffer; i++) {
			if (p_Dpb->fs_ltref[i]->is_used == 3) {
				if (p_Dpb->fs_ltref[i]->frame->is_long_term) {
					currSlice->listX[0][list0idx++] =
						p_Dpb->fs_ltref[i]->frame;
				}
			}
		}
		meson_h264_dpb_qsort((void *)&currSlice->listX[0][
			(short) currSlice->listXsize[0]],
			list0idx - currSlice->listXsize[0],
			sizeof(struct StorablePicture *),
			meson_h264_dpb_compare_pic_by_lt_pic_num_asc);
		currSlice->listXsize[0] = (char) list0idx;
		CHECK_VALID(currSlice->listXsize[0], 0);
	} else {
		fs_list0 = &(p_Dpb->fs_list0[0]);
		fs_listlt = &(p_Dpb->fs_listlt[0]);
		for (i = 0; i < p_Dpb->ref_frames_in_buffer; i++) {
			if (p_Dpb->fs_ref[i] == NULL) {
				p_H264_Dpb->dpb_error_flag = __LINE__;
				continue;
			}
			if (p_Dpb->fs_ref[i]->is_reference)
				fs_list0[list0idx++] = p_Dpb->fs_ref[i];
		}

		meson_h264_dpb_qsort((void *)fs_list0, list0idx, sizeof(struct FrameStore *),
		      meson_h264_dpb_compare_fs_by_frame_num_desc);

		meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
			  "fs_list0 (FrameNum): ");
		for (i = 0; i < list0idx; i++) {
			meson_h264_dpb_dpb_print_cont(p_H264_Dpb->decoder_index,
				  PRINT_FLAG_DPB_DETAIL, "%d  ",
				  fs_list0[i]->frame_num_wrap);
		}
		meson_h264_dpb_dpb_print_cont(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
			  "\n");

		currSlice->listXsize[0] = 0;
		meson_h264_dpb_gen_pic_list_from_frame_list(currSlice->structure, fs_list0,
						list0idx, currSlice->listX[0],
						&currSlice->listXsize[0], 0);

		meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
			  "listX[0] (PicNum): ");
		for (i = 0; i < currSlice->listXsize[0]; i++) {
			meson_h264_dpb_dpb_print_cont(p_H264_Dpb->decoder_index,
				PRINT_FLAG_DPB_DETAIL, "%d  ",
				currSlice->listX[0][i]->pic_num);
		}
		meson_h264_dpb_dpb_print_cont(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
			  "\n");

		for (i = 0; i < p_Dpb->ltref_frames_in_buffer; i++)
			fs_listlt[listltidx++] = p_Dpb->fs_ltref[i];

		meson_h264_dpb_qsort((void *)fs_listlt, listltidx, sizeof(struct FrameStore *),
		      meson_h264_dpb_compare_fs_by_lt_pic_idx_asc);

		meson_h264_dpb_gen_pic_list_from_frame_list(currSlice->structure, fs_listlt,
					listltidx, currSlice->listX[0],
					&currSlice->listXsize[0], 1);

	}
	currSlice->listXsize[1] = 0;

	currSlice->listXsize[0] = (char) imin(currSlice->listXsize[0],
			currSlice->num_ref_idx_active[LIST_0]);
	currSlice->listXsize[1] = (char) imin(currSlice->listXsize[1],
			currSlice->num_ref_idx_active[LIST_1]);
	CHECK_VALID(currSlice->listXsize[0], 0);
	CHECK_VALID(currSlice->listXsize[1], 1);

	for (i = currSlice->listXsize[0]; i < (MAX_LIST_SIZE); i++)
		currSlice->listX[0][i] = p_Vid->no_reference_picture;
	for (i = currSlice->listXsize[1]; i < (MAX_LIST_SIZE); i++)
		currSlice->listX[1][i] = p_Vid->no_reference_picture;

}

static void meson_h264_dpb_init_mbaff_lists(struct h264_dpb_stru *p_H264_Dpb,
			     struct Slice *currSlice)
{
	unsigned int j;
	int i;
	struct VideoParameters *p_Vid = &p_H264_Dpb->mVideo;
	for (i = 2; i < 6; i++) {
		for (j = 0; j < MAX_LIST_SIZE; j++)
			currSlice->listX[i][j] = p_Vid->no_reference_picture;
		currSlice->listXsize[i] = 0;
	}

	for (i = 0; i < currSlice->listXsize[0]; i++) {
		if (currSlice->listX[0][i] == NULL) {
			p_H264_Dpb->dpb_error_flag = __LINE__;
			pr_info(
			"error currSlice->listX[0][%d] is NULL\r\n", i);
			break;
		}
		currSlice->listX[2][2 * i] =
			currSlice->listX[0][i]->top_field;
		currSlice->listX[2][2 * i + 1] =
			currSlice->listX[0][i]->bottom_field;
		currSlice->listX[4][2 * i] =
			currSlice->listX[0][i]->bottom_field;
		currSlice->listX[4][2 * i + 1] =
			currSlice->listX[0][i]->top_field;
	}
	currSlice->listXsize[2] = currSlice->listXsize[4] =
		currSlice->listXsize[0] * 2;

	for (i = 0; i < currSlice->listXsize[1]; i++) {
		if (currSlice->listX[1][i] == NULL) {
			p_H264_Dpb->dpb_error_flag = __LINE__;
			pr_info(
			"error currSlice->listX[1][%d] is NULL\r\n", i);
			break;
		}
		currSlice->listX[3][2 * i] =
			currSlice->listX[1][i]->top_field;
		currSlice->listX[3][2 * i + 1] =
			currSlice->listX[1][i]->bottom_field;
		currSlice->listX[5][2 * i] =
			currSlice->listX[1][i]->bottom_field;
		currSlice->listX[5][2 * i + 1] =
			currSlice->listX[1][i]->top_field;
	}
	currSlice->listXsize[3] = currSlice->listXsize[5] =
		currSlice->listXsize[1] * 2;
}

static void meson_h264_dpb_init_lists_i_slice(struct Slice *currSlice)
{

	currSlice->listXsize[0] = 0;
	currSlice->listXsize[1] = 0;
}

static void meson_h264_dpb_init_lists_b_slice(struct Slice *currSlice)
{
	struct VideoParameters *p_Vid = currSlice->p_Vid;
	struct DecodedPictureBuffer *p_Dpb = currSlice->p_Dpb;
	struct h264_dpb_stru *p_H264_Dpb = container_of(p_Dpb,
		struct h264_dpb_stru, mDPB);

	unsigned int i;
	int j;

	int list0idx = 0;
	int list0idx_1 = 0;
	int listltidx = 0;

	struct FrameStore **fs_list0;
	struct FrameStore **fs_list1;
	struct FrameStore **fs_listlt;

	{
		
		if (currSlice->structure == FRAME) {
			for (i = 0; i < p_Dpb->ref_frames_in_buffer; i++) {
				if (p_Dpb->fs_ref[i] == NULL ||
					p_Dpb->fs_ref[i]->frame == NULL) {
					p_H264_Dpb->dpb_error_flag = __LINE__;
					continue;
				}
				if ((p_Dpb->fs_ref[i]->is_used == 3) &&
					((p_Dpb->fs_ref[i]->frame->
					used_for_reference) &&
					(!p_Dpb->fs_ref[i]->frame->
					is_long_term)) &&
					(currSlice->framepoc >=
					p_Dpb->fs_ref[i]->frame->poc)) {

					currSlice->listX[0][list0idx++] =
						p_Dpb->fs_ref[i]->frame;
				}
			}
			meson_h264_dpb_qsort((void *)currSlice->listX[0], list0idx,
				sizeof(struct StorablePicture *),
				meson_h264_dpb_compare_pic_by_poc_desc);

			list0idx_1 = list0idx;
			for (i = 0; i < p_Dpb->ref_frames_in_buffer; i++) {
				if (p_Dpb->fs_ref[i] == NULL ||
					p_Dpb->fs_ref[i]->frame == NULL) {
					p_H264_Dpb->dpb_error_flag = __LINE__;
					continue;
				}
				if ((p_Dpb->fs_ref[i]->is_used == 3) &&
					((p_Dpb->fs_ref[i]->frame->
					used_for_reference) &&
					(!p_Dpb->fs_ref[i]->frame->
					is_long_term)) &&
					(currSlice->framepoc <
					p_Dpb->fs_ref[i]->frame->poc)) {
					currSlice->
					listX[0][list0idx++] =
						p_Dpb->fs_ref[i]->frame;
				}
			}
			meson_h264_dpb_qsort((void *)&currSlice->listX[0][list0idx_1],
				list0idx - list0idx_1,
				sizeof(struct StorablePicture *),
				meson_h264_dpb_compare_pic_by_poc_asc);

			for (j = 0; j < list0idx_1; j++) {
				currSlice->
				listX[1][list0idx - list0idx_1 + j] =
					currSlice->listX[0][j];
			}
			for (j = list0idx_1; j < list0idx; j++) {
				currSlice->listX[1][j - list0idx_1] =
					currSlice->listX[0][j];
			}

			currSlice->listXsize[0] = currSlice->listXsize[1] =
				(char) list0idx;
			CHECK_VALID(currSlice->listXsize[0], 0);
			CHECK_VALID(currSlice->listXsize[1], 1);

			meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
				PRINT_FLAG_DPB_DETAIL,
				"listX[0] (PicNum): ");
			for (i = 0; i < currSlice->listXsize[0]; i++) {
				meson_h264_dpb_dpb_print_cont(p_H264_Dpb->decoder_index,
				PRINT_FLAG_DPB_DETAIL, "%d  ",
				currSlice->listX[0][i]->pic_num);
			}
			meson_h264_dpb_dpb_print_cont(p_H264_Dpb->decoder_index,
				PRINT_FLAG_DPB_DETAIL, "\n");
			meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
				PRINT_FLAG_DPB_DETAIL,
				"listX[1] (PicNum): ");
			for (i = 0; i < currSlice->listXsize[1]; i++) {
				meson_h264_dpb_dpb_print_cont(p_H264_Dpb->decoder_index,
					PRINT_FLAG_DPB_DETAIL, "%d  ",
					currSlice->listX[1][i]->pic_num);
			}
			meson_h264_dpb_dpb_print_cont(p_H264_Dpb->decoder_index,
				PRINT_FLAG_DPB_DETAIL, "\n");

			for (i = 0; i < p_Dpb->ltref_frames_in_buffer; i++) {
				if (p_Dpb->fs_ltref[i]->is_used == 3) {
					if (p_Dpb->fs_ltref[i]->frame->
						is_long_term) {
						currSlice->
						listX[0][list0idx] =
						p_Dpb->fs_ltref[i]->frame;
						currSlice->
						listX[1][list0idx++] =
						p_Dpb->fs_ltref[i]->frame;
					}
				}
			}
			meson_h264_dpb_qsort((void *)&currSlice->
				listX[0][(short) currSlice->listXsize[0]],
				list0idx - currSlice->listXsize[0],
				sizeof(struct StorablePicture *),
				meson_h264_dpb_compare_pic_by_lt_pic_num_asc);
			meson_h264_dpb_qsort((void *)&currSlice->
				listX[1][(short) currSlice->listXsize[0]],
				list0idx - currSlice->listXsize[0],
				sizeof(struct StorablePicture *),
				meson_h264_dpb_compare_pic_by_lt_pic_num_asc);
			currSlice->listXsize[0] = currSlice->listXsize[1] =
				(char) list0idx;
			CHECK_VALID(currSlice->listXsize[0], 0);
			CHECK_VALID(currSlice->listXsize[1], 1);
		} else {
			fs_list0 = &(p_Dpb->fs_list0[0]);
			fs_list1 = &(p_Dpb->fs_list1[0]);
			fs_listlt = &(p_Dpb->fs_listlt[0]);

			currSlice->listXsize[0] = 0;
			currSlice->listXsize[1] = 1;

			for (i = 0; i < p_Dpb->ref_frames_in_buffer; i++) {
				if (p_Dpb->fs_ref[i] == NULL) {
					p_H264_Dpb->dpb_error_flag = __LINE__;
					continue;
				}
				if (p_Dpb->fs_ref[i]->is_used) {
					if (currSlice->ThisPOC >=
						p_Dpb->fs_ref[i]->poc) {
						fs_list0[list0idx++] =
							p_Dpb->fs_ref[i];
					}
				}
			}
			meson_h264_dpb_qsort((void *)fs_list0, list0idx,
				sizeof(struct FrameStore *),
				meson_h264_dpb_compare_fs_by_poc_desc);
			list0idx_1 = list0idx;
			for (i = 0; i < p_Dpb->ref_frames_in_buffer; i++) {
				if (p_Dpb->fs_ref[i] == NULL) {
					p_H264_Dpb->dpb_error_flag = __LINE__;
					continue;
				}
				if (p_Dpb->fs_ref[i]->is_used) {
					if (currSlice->ThisPOC <
						p_Dpb->fs_ref[i]->poc) {
						fs_list0[list0idx++] =
							p_Dpb->fs_ref[i];
					}
				}
			}
			meson_h264_dpb_qsort((void *)&fs_list0[list0idx_1],
				list0idx - list0idx_1,
				sizeof(struct FrameStore *),
				meson_h264_dpb_compare_fs_by_poc_asc);

			for (j = 0; j < list0idx_1; j++) {
				fs_list1[list0idx - list0idx_1 + j] =
				fs_list0[j];
			}
			for (j = list0idx_1; j < list0idx; j++)
				fs_list1[j - list0idx_1] = fs_list0[j];

			meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
				PRINT_FLAG_DPB_DETAIL,
				"fs_list0 currPoc=%d (Poc): ",
				currSlice->ThisPOC);
			for (i = 0; i < list0idx; i++) {
				meson_h264_dpb_dpb_print_cont(p_H264_Dpb->decoder_index,
				PRINT_FLAG_DPB_DETAIL, "%d  ",
				fs_list0[i]->poc);
			}
			meson_h264_dpb_dpb_print_cont(p_H264_Dpb->decoder_index,
				PRINT_FLAG_DPB_DETAIL, "\n");
			meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
				PRINT_FLAG_DPB_DETAIL,
				"fs_list1 currPoc=%d (Poc): ",
				currSlice->ThisPOC);
			for (i = 0; i < list0idx; i++) {
				meson_h264_dpb_dpb_print_cont(p_H264_Dpb->decoder_index,
					PRINT_FLAG_DPB_DETAIL, "%d  ",
					fs_list1[i]->poc);
			}
			meson_h264_dpb_dpb_print_cont(p_H264_Dpb->decoder_index,
					PRINT_FLAG_DPB_DETAIL, "\n");

			currSlice->listXsize[0] = 0;
			currSlice->listXsize[1] = 0;
			meson_h264_dpb_gen_pic_list_from_frame_list(currSlice->structure,
						fs_list0, list0idx,
						currSlice->listX[0],
						&currSlice->listXsize[0], 0);
			meson_h264_dpb_gen_pic_list_from_frame_list(currSlice->structure,
						fs_list1, list0idx,
						currSlice->listX[1],
						&currSlice->listXsize[1], 0);

			for (i = 0; i < p_Dpb->ltref_frames_in_buffer; i++)
				fs_listlt[listltidx++] = p_Dpb->fs_ltref[i];

			meson_h264_dpb_qsort((void *)fs_listlt, listltidx,
				sizeof(struct FrameStore *),
				meson_h264_dpb_compare_fs_by_lt_pic_idx_asc);

			meson_h264_dpb_gen_pic_list_from_frame_list(currSlice->structure,
				fs_listlt, listltidx,
				currSlice->listX[0],
				&currSlice->listXsize[0], 1);
			meson_h264_dpb_gen_pic_list_from_frame_list(currSlice->structure,
				fs_listlt, listltidx,
				currSlice->listX[1],
				&currSlice->listXsize[1], 1);

		}
	}

	if ((currSlice->listXsize[0] == currSlice->listXsize[1]) &&
	    (currSlice->listXsize[0] > 1)) {

		int diff = 0;

		for (j = 0; j < currSlice->listXsize[0]; j++) {
			if (currSlice->listX[0][j] !=
				currSlice->listX[1][j]) {
				diff = 1;
				break;
			}
		}
		if (!diff) {
			struct StorablePicture *tmp_s =
				currSlice->listX[1][0];
			currSlice->listX[1][0] = currSlice->listX[1][1];
			currSlice->listX[1][1] = tmp_s;
		}
	}

	currSlice->listXsize[0] = (char) imin(currSlice->listXsize[0],
		currSlice->num_ref_idx_active[LIST_0]);
	currSlice->listXsize[1] = (char) imin(currSlice->listXsize[1],
		currSlice->num_ref_idx_active[LIST_1]);
	CHECK_VALID(currSlice->listXsize[0], 0);
	CHECK_VALID(currSlice->listXsize[1], 1);

	for (i = currSlice->listXsize[0]; i < (MAX_LIST_SIZE); i++)
		currSlice->listX[0][i] = p_Vid->no_reference_picture;
	for (i = currSlice->listXsize[1]; i < (MAX_LIST_SIZE); i++)
		currSlice->listX[1][i] = p_Vid->no_reference_picture;

}

static struct StorablePicture *meson_h264_dpb_get_short_term_pic(struct Slice *currSlice,
		struct DecodedPictureBuffer *p_Dpb, int picNum)
{
	unsigned int i;
	struct h264_dpb_stru *p_H264_Dpb = container_of(p_Dpb,
		struct h264_dpb_stru, mDPB);
	for (i = 0; i < p_Dpb->ref_frames_in_buffer; i++) {
		if (currSlice->structure == FRAME) {
			if (p_Dpb->fs_ref[i] == NULL) {
				p_H264_Dpb->dpb_error_flag = __LINE__;
				continue;
			}
			if (p_Dpb->fs_ref[i]->is_reference == 3) {
				if (p_Dpb->fs_ref[i]->frame == NULL) {
					p_H264_Dpb->dpb_error_flag = __LINE__;
					continue;
				}
				if ((!p_Dpb->fs_ref[i]->frame->
					is_long_term) &&
				    (p_Dpb->fs_ref[i]->frame->
					pic_num == picNum))
					return p_Dpb->fs_ref[i]->frame;
			}
		} else {
			if (p_Dpb->fs_ref[i] == NULL) {
				p_H264_Dpb->dpb_error_flag = __LINE__;
				continue;
			}
			if (p_Dpb->fs_ref[i]->is_reference & 1) {
				if (p_Dpb->fs_ref[i]->top_field == NULL) {
					p_H264_Dpb->dpb_error_flag = __LINE__;
					continue;
				}
				if ((!p_Dpb->fs_ref[i]->top_field->
					is_long_term) &&
				    (p_Dpb->fs_ref[i]->top_field->
					pic_num == picNum))
					return p_Dpb->fs_ref[i]->top_field;
			}
			if (p_Dpb->fs_ref[i]->is_reference & 2) {
				if (p_Dpb->fs_ref[i]->bottom_field == NULL) {
					p_H264_Dpb->dpb_error_flag = __LINE__;
					continue;
				}
				if ((!p_Dpb->fs_ref[i]->bottom_field->
					is_long_term) &&
				    (p_Dpb->fs_ref[i]->bottom_field->
					pic_num == picNum))
					return p_Dpb->fs_ref[i]->bottom_field;
			}
		}
	}

	return currSlice->p_Vid->no_reference_picture;
}

static void meson_h264_dpb_reorder_short_term(struct Slice *currSlice, int cur_list,
				int num_ref_idx_lX_active_minus1,
				int picNumLX, int *refIdxLX)
{
	struct h264_dpb_stru *p_H264_Dpb = container_of(currSlice->p_Vid,
					struct h264_dpb_stru, mVideo);

	struct StorablePicture **RefPicListX = currSlice->listX[cur_list];
	int cIdx, nIdx;

	struct StorablePicture *picLX;

	picLX = meson_h264_dpb_get_short_term_pic(currSlice, currSlice->p_Dpb, picNumLX);

	for (cIdx = num_ref_idx_lX_active_minus1 + 1; cIdx > *refIdxLX;
		cIdx--) {
		meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
			"%s: RefPicListX[ %d ] = RefPicListX[ %d ]\n",
			__func__, cIdx, cIdx - 1);
		RefPicListX[cIdx] = RefPicListX[cIdx - 1];
	}

	meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
		"%s: RefPicListX[ %d ] = pic %p pic_num(%d)\n", __func__,
		*refIdxLX, picLX, picNumLX);

	RefPicListX[(*refIdxLX)++] = picLX;

	nIdx = *refIdxLX;

	for (cIdx = *refIdxLX; cIdx <= num_ref_idx_lX_active_minus1 + 1;
		cIdx++) {
		if (RefPicListX[cIdx])
			if ((RefPicListX[cIdx]->is_long_term) ||
			    (RefPicListX[cIdx]->pic_num != picNumLX)) {
				meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
					PRINT_FLAG_DPB_DETAIL,
					"%s: RefPicListX[ %d ] = RefPicListX[ %d ]\n",
					__func__, nIdx, cIdx);
				RefPicListX[nIdx++] = RefPicListX[cIdx];
			}
	}
}

static struct StorablePicture *meson_h264_dpb_get_long_term_pic(struct Slice *currSlice,
		struct DecodedPictureBuffer *p_Dpb, int LongtermPicNum)
{
	unsigned int i;

	for (i = 0; i < p_Dpb->ltref_frames_in_buffer; i++) {
		if (currSlice->structure == FRAME) {
			if (p_Dpb->fs_ltref[i]->is_reference == 3)
				if ((p_Dpb->fs_ltref[i]->frame) &&
					(p_Dpb->fs_ltref[i]->frame->
					is_long_term) &&
				    (p_Dpb->fs_ltref[i]->frame->
					long_term_pic_num ==
					LongtermPicNum))
					return p_Dpb->fs_ltref[i]->frame;
		} else {
			if (p_Dpb->fs_ltref[i]->is_reference & 1)
				if ((p_Dpb->fs_ltref[i]->top_field) &&
					(p_Dpb->fs_ltref[i]->top_field->
					is_long_term) &&
				    (p_Dpb->fs_ltref[i]->top_field->
					long_term_pic_num == LongtermPicNum))
					return p_Dpb->fs_ltref[i]->top_field;

			if (p_Dpb->fs_ltref[i]->is_reference & 2)
				if ((p_Dpb->fs_ltref[i]->bottom_field) &&
					(p_Dpb->fs_ltref[i]->bottom_field->
					is_long_term) &&
				    (p_Dpb->fs_ltref[i]->bottom_field->
					long_term_pic_num ==
					LongtermPicNum))
					return p_Dpb->fs_ltref[i]->
						bottom_field;
		}
	}
	return NULL;
}

static void meson_h264_dpb_reorder_long_term(struct Slice *currSlice,
				struct StorablePicture **RefPicListX,
				int num_ref_idx_lX_active_minus1,
			      int LongTermPicNum, int *refIdxLX)
{
	int cIdx, nIdx;

	struct StorablePicture *picLX;

	picLX = meson_h264_dpb_get_long_term_pic(currSlice, currSlice->p_Dpb, LongTermPicNum);

	for (cIdx = num_ref_idx_lX_active_minus1 + 1; cIdx > *refIdxLX; cIdx--)
		RefPicListX[cIdx] = RefPicListX[cIdx - 1];

	RefPicListX[(*refIdxLX)++] = picLX;

	nIdx = *refIdxLX;

	for (cIdx = *refIdxLX; cIdx <= num_ref_idx_lX_active_minus1 + 1;
		cIdx++) {
		if (RefPicListX[cIdx]) {
			if ((!RefPicListX[cIdx]->is_long_term) ||
			    (RefPicListX[cIdx]->long_term_pic_num !=
				LongTermPicNum))
				RefPicListX[nIdx++] = RefPicListX[cIdx];
		}
	}
}

static void meson_h264_dpb_reorder_ref_pic_list(struct Slice *currSlice, int cur_list)
{
	int *modification_of_pic_nums_idc =
		currSlice->modification_of_pic_nums_idc[cur_list];
	int *abs_diff_pic_num_minus1 =
		currSlice->abs_diff_pic_num_minus1[cur_list];
	int *long_term_pic_idx = currSlice->long_term_pic_idx[cur_list];
	int num_ref_idx_lX_active_minus1 =
		currSlice->num_ref_idx_active[cur_list] - 1;

	struct VideoParameters *p_Vid = currSlice->p_Vid;
	int i;

	int maxPicNum, currPicNum, picNumLXNoWrap, picNumLXPred, picNumLX;
	int refIdxLX = 0;

	if (currSlice->structure == FRAME) {
		maxPicNum  = p_Vid->max_frame_num;
		currPicNum = currSlice->frame_num;
	} else {
		maxPicNum  = 2 * p_Vid->max_frame_num;
		currPicNum = 2 * currSlice->frame_num + 1;
	}

	picNumLXPred = currPicNum;

	for (i = 0;  i < REORDERING_COMMAND_MAX_SIZE &&
		modification_of_pic_nums_idc[i] != 3; i++) {
		if (modification_of_pic_nums_idc[i] > 3) {
			struct h264_dpb_stru *p_H264_Dpb =
			container_of(p_Vid, struct h264_dpb_stru, mVideo);
			meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
				PRINT_FLAG_ERROR,
				"error, Invalid modification_of_pic_nums_idc command\n");
			
			break;
		}
		if (modification_of_pic_nums_idc[i] < 2) {
			if (modification_of_pic_nums_idc[i] == 0) {
				if (picNumLXPred - (abs_diff_pic_num_minus1[i]
					+ 1) < 0)
					picNumLXNoWrap = picNumLXPred -
					(abs_diff_pic_num_minus1[i] + 1) +
					maxPicNum;
				else
					picNumLXNoWrap = picNumLXPred -
					(abs_diff_pic_num_minus1[i] + 1);
			} else { 
				if (picNumLXPred + (abs_diff_pic_num_minus1[i]
					+ 1)  >=  maxPicNum)
					picNumLXNoWrap = picNumLXPred +
					(abs_diff_pic_num_minus1[i] + 1) -
					maxPicNum;
				else
					picNumLXNoWrap = picNumLXPred +
					(abs_diff_pic_num_minus1[i] + 1);
			}
			picNumLXPred = picNumLXNoWrap;

			if (picNumLXNoWrap > currPicNum)
				picNumLX = picNumLXNoWrap - maxPicNum;
			else
				picNumLX = picNumLXNoWrap;

			meson_h264_dpb_reorder_short_term(currSlice, cur_list,
					num_ref_idx_lX_active_minus1, picNumLX,
					&refIdxLX);
		} else { 
			meson_h264_dpb_reorder_long_term(currSlice, currSlice->listX[cur_list],
					num_ref_idx_lX_active_minus1,
					long_term_pic_idx[i], &refIdxLX);
		}

	}
	
	currSlice->listXsize[cur_list] =
		(char)(num_ref_idx_lX_active_minus1 + 1);
}

static void meson_h264_dpb_reorder_lists(struct Slice *currSlice)
{
	struct VideoParameters *p_Vid = currSlice->p_Vid;
	struct h264_dpb_stru *p_H264_Dpb = container_of(p_Vid,
		struct h264_dpb_stru, mVideo);
	int i;

	if ((currSlice->slice_type != I_SLICE) &&
		(currSlice->slice_type != SI_SLICE)) {
		if (currSlice->ref_pic_list_reordering_flag[LIST_0])
			meson_h264_dpb_reorder_ref_pic_list(currSlice, LIST_0);
		if (p_Vid->no_reference_picture ==
		    currSlice->
			listX[0][currSlice->num_ref_idx_active[LIST_0] - 1]) {
			if (p_Vid->non_conforming_stream)
				meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
					PRINT_FLAG_DPB_DETAIL,
					"RefPicList0[ %d ] is equal to 'no reference picture'\n",
					currSlice->
					num_ref_idx_active[LIST_0] - 1);
			else
				meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
					PRINT_FLAG_DPB_DETAIL,
					"RefPicList0 [ num_ref_idx_l0_active_minus1 ] is equal to 'no reference picture', invalid bitstream %d\n",
					500);
		}
		
		currSlice->listXsize[0] =
			(char) imin(currSlice->listXsize[0],
			currSlice->num_ref_idx_active[LIST_0]);
		CHECK_VALID(currSlice->listXsize[0], 0);
		if (0U & PRINT_FLAG_DPB_DETAIL) {
			meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
				PRINT_FLAG_DPB_DETAIL,
				"listX[0] reorder (PicNum): ");
			for (i = 0; i < currSlice->listXsize[0]; i++) {
				if (currSlice->listX[0][i])
					meson_h264_dpb_dpb_print_cont(p_H264_Dpb->decoder_index,
						PRINT_FLAG_DPB_DETAIL, "%d  ",
						currSlice->listX[0][i]->pic_num);
			}
			meson_h264_dpb_dpb_print_cont(p_H264_Dpb->decoder_index,
				PRINT_FLAG_DPB_DETAIL, "\n");
		}
	}

	if (currSlice->slice_type == B_SLICE) {
		if (currSlice->ref_pic_list_reordering_flag[LIST_1])
			meson_h264_dpb_reorder_ref_pic_list(currSlice, LIST_1);
		if (p_Vid->no_reference_picture ==
		    currSlice->listX[1][currSlice->
			num_ref_idx_active[LIST_1] - 1]) {
			if (p_Vid->non_conforming_stream)
				meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
					PRINT_FLAG_DPB_DETAIL,
					"RefPicList1[ %d ] is equal to 'no reference picture'\n",
					currSlice->
					num_ref_idx_active[LIST_1] - 1);
			else
				meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
					PRINT_FLAG_DPB_DETAIL,
					"RefPicList1 [ num_ref_idx_l1_active_minus1 ] is equal to 'no reference picture', invalid bitstream %d\n",
					500);
		}
		
		currSlice->listXsize[1] =
			(char)currSlice->num_ref_idx_active[LIST_1];
		if (0U & PRINT_FLAG_DPB_DETAIL) {
			meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
			PRINT_FLAG_DPB_DETAIL,
				  "listX[1] reorder (PicNum): ");
			for (i = 0; i < currSlice->listXsize[1]; i++) {
				if (currSlice->listX[1][i])
					meson_h264_dpb_dpb_print_cont(p_H264_Dpb->decoder_index,
						PRINT_FLAG_DPB_DETAIL, "%d  ",
						currSlice->listX[1][i]->pic_num);
			}
			meson_h264_dpb_dpb_print_cont(p_H264_Dpb->decoder_index,
				PRINT_FLAG_DPB_DETAIL, "\n");
		}
	}

	if (currSlice->slice_type == P_SLICE) {
	} else if (currSlice->slice_type == B_SLICE) {
	}
}

static void meson_h264_dpb_init_colocate_buf(struct h264_dpb_stru *p_H264_Dpb, int count)
{
	p_H264_Dpb->colocated_buf_map = 0;
	p_H264_Dpb->colocated_buf_count = count;
}

static int meson_h264_dpb_allocate_colocate_buf(struct h264_dpb_stru *p_H264_Dpb)
{
	int i;

	for (i = 0; i < p_H264_Dpb->colocated_buf_count; i++) {
		if (((p_H264_Dpb->colocated_buf_map >> i) & 0x1) == 0) {
			p_H264_Dpb->colocated_buf_map |= (1 << i);
			break;
		}
	}
	if (i == p_H264_Dpb->colocated_buf_count) {
		i = -1;
		p_H264_Dpb->buf_alloc_fail = 1;
	}
	return i;
}

static int meson_h264_dpb_release_colocate_buf(struct h264_dpb_stru *p_H264_Dpb, int index)
{
	if (index >= 0) {
		if (index >= p_H264_Dpb->colocated_buf_count) {
			meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_ERROR,
				"%s error, index %d is bigger than buf count %d\n",
				__func__, index,
				p_H264_Dpb->colocated_buf_count);
		} else {
			if (((p_H264_Dpb->colocated_buf_map >>
				index) & 0x1) == 0x1) {
				p_H264_Dpb->colocated_buf_map &=
					(~(1 << index));
			} else {
				meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
					PRINT_FLAG_ERROR,
					"%s error, index %d is not allocated\n",
					__func__, index);
			}
		}
	}
	return 0;
}

static void meson_h264_dpb_set_frame_output_flag(struct h264_dpb_stru *p_H264_Dpb, int index)
{
	struct DecodedPictureBuffer *p_Dpb = &p_H264_Dpb->mDPB;

	p_H264_Dpb->mFrameStore[index].is_output = 1;
	p_H264_Dpb->mFrameStore[index].pre_output = 0;
	p_H264_Dpb->mFrameStore[index].show_frame = false;
	meson_h264_dpb_dump_dpb(p_Dpb, 0);
}



static int meson_h264_dpb_release_picture(struct h264_dpb_stru *p_H264_Dpb,
		   struct StorablePicture *pic)
{
	/* A complementary field borrows both resources from its FrameStore.
	 * An unrelated unmatched field does not own this picture's allocation.
	 */
	if (pic->buf_spec_is_alloced) {
		meson_h264_dpb_release_colocate_buf(p_H264_Dpb,
			pic->colocated_buf_index);
		meson_h264_dpb_release_buf_spec_num(p_H264_Dpb->vdec,
			pic->buf_spec_num);
	}

	meson_h264_dpb_free_picture(p_H264_Dpb, pic);
	return 0;
}

static void meson_h264_dpb_check_frame_store_same_pic_num(struct DecodedPictureBuffer *p_Dpb,
	struct StorablePicture *p, struct Slice *currSlice)
{
	if (p_Dpb->last_picture) {
		if ((int)p_Dpb->last_picture->frame_num == p->pic_num) {
			if (((p->structure == TOP_FIELD) &&
				(p_Dpb->last_picture->is_used == 2)) ||
			    ((p->structure == BOTTOM_FIELD) &&
				(p_Dpb->last_picture->is_used == 1))) {
				if ((p->used_for_reference &&
					(p_Dpb->last_picture->
					is_orig_reference != 0)) ||
				    (!p->used_for_reference &&
					(p_Dpb->last_picture->
					is_orig_reference == 0))) {
					p->buf_spec_num =
						p_Dpb->last_picture->
						buf_spec_num;
					p->buf_spec_is_alloced = 0;
					p->colocated_buf_index = p_Dpb->
						last_picture->
						colocated_buf_index;
					if (currSlice->structure ==
						TOP_FIELD) {
						p->bottom_poc =
							p_Dpb->last_picture->
							bottom_field->poc;
					} else {
						p->top_poc =
							p_Dpb->last_picture->
							top_field->poc;
					}
					p->frame_poc = imin(p->bottom_poc,
						p->top_poc);
				}
			}
		}
	}
}

static int meson_h264_dpb_h264_slice_header_process(struct h264_dpb_stru *p_H264_Dpb, int *frame_num_gap)
{

	int new_pic_flag = 0;
	struct Slice *currSlice = &p_H264_Dpb->mSlice;
	struct VideoParameters *p_Vid = &p_H264_Dpb->mVideo;
	struct DecodedPictureBuffer *p_Dpb =
				&p_H264_Dpb->mDPB;
	new_pic_flag = (p_H264_Dpb->mVideo.dec_picture == NULL);
	p_H264_Dpb->buf_alloc_fail = 0;
	p_H264_Dpb->dpb_error_flag = 0;
	meson_h264_dpb_slice_prepare(p_H264_Dpb, &p_H264_Dpb->mDPB, &p_H264_Dpb->mVideo,
		      &p_H264_Dpb->mSPS, &p_H264_Dpb->mSlice);

	if (p_Dpb->num_ref_frames != p_H264_Dpb->mSPS.num_ref_frames) {
		meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, 0,
		"num_ref_frames change from %d to %d\r\n",
			p_Dpb->num_ref_frames, p_H264_Dpb->mSPS.num_ref_frames);
		p_Dpb->num_ref_frames = p_H264_Dpb->mSPS.num_ref_frames;
	}
	
	if (p_H264_Dpb->mDPB.init_done == 0) {

		meson_h264_dpb_init_dpb(p_H264_Dpb, 0);
	}

	if (new_pic_flag) { 
		meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index, PRINT_FLAG_DPB_DETAIL,
		"check frame_num gap: cur frame_num %d pre_frame_num %d max_frmae_num %d\r\n",
		currSlice->frame_num,
		p_Vid->pre_frame_num,
		p_Vid->max_frame_num);
		if (p_Vid->recovery_point == 0 &&
			p_Vid->max_frame_num <= FRAME_NUM_MAX_SIZE &&
			currSlice->frame_num != p_Vid->pre_frame_num &&
			currSlice->frame_num !=
			(p_Vid->pre_frame_num + 1) % p_Vid->max_frame_num) {

			*frame_num_gap = 1;
		}

		if (currSlice->nal_reference_idc) {
			meson_h264_dpb_dpb_print(p_H264_Dpb->decoder_index,
				PRINT_FLAG_DPB_DETAIL,
			"nal_reference_idc not 0, set pre_frame_num(%d) to frame_num (%d)\n",
			p_Vid->pre_frame_num, currSlice->frame_num);
			p_Vid->pre_frame_num = currSlice->frame_num;
		}

		meson_h264_dpb_decode_poc(&p_H264_Dpb->mVideo, &p_H264_Dpb->mSlice);
		p_H264_Dpb->mVideo.dec_picture = meson_h264_dpb_get_new_pic(p_H264_Dpb,
						 p_H264_Dpb->mSlice.structure,

						 1);
		if (p_H264_Dpb->mVideo.dec_picture) {
			u32 offset_lo, offset_hi;
			struct DecodedPictureBuffer *p_Dpb =
				&p_H264_Dpb->mDPB;
			struct StorablePicture *p =
				p_H264_Dpb->mVideo.dec_picture;
			meson_h264_dpb_init_picture(p_H264_Dpb, &p_H264_Dpb->mSlice,
				p_H264_Dpb->mVideo.dec_picture);
			
			offset_lo  =
			p_H264_Dpb->dpb_param.l.data[OFFSET_DELIMITER_LO];
			offset_hi  =
			p_H264_Dpb->dpb_param.l.data[OFFSET_DELIMITER_HI];
			p_H264_Dpb->mVideo.dec_picture->offset_delimiter =
				(offset_lo	| offset_hi << 16);
			p_H264_Dpb->mVideo.dec_picture->buf_spec_num  = -1;
			p_H264_Dpb->mVideo.dec_picture->
				colocated_buf_index = -1;
			meson_h264_dpb_update_pic_num(p_H264_Dpb);

			if ((currSlice->structure == TOP_FIELD) ||
			    (currSlice->structure == BOTTOM_FIELD)) {

				meson_h264_dpb_check_frame_store_same_pic_num(p_Dpb, p,
					currSlice);
			}

			
		}
	}

	if (p_H264_Dpb->mSlice.slice_type == P_SLICE)
		meson_h264_dpb_init_lists_p_slice(&p_H264_Dpb->mSlice);
	else if (p_H264_Dpb->mSlice.slice_type == B_SLICE)
		meson_h264_dpb_init_lists_b_slice(&p_H264_Dpb->mSlice);
	else
		meson_h264_dpb_init_lists_i_slice(&p_H264_Dpb->mSlice);

	meson_h264_dpb_reorder_lists(&p_H264_Dpb->mSlice);

	if (p_H264_Dpb->mSlice.structure == FRAME)
		meson_h264_dpb_init_mbaff_lists(p_H264_Dpb, &p_H264_Dpb->mSlice);

	if (new_pic_flag)
		return 1;

	return 0;
}









/* SC2 H.264 MULTI mailbox and input-context ABI (vendor vmh264/vdec). */
#define H264_MULTI_WORKSPACE_SIZE SZ_2M
#define H264_MULTI_WORKSPACE_MARGIN SZ_64K
#define H264_MULTI_SWAP_ADDR (0xc55 * 4)
#define H264_MULTI_SWAP_CTL (0xc56 * 4)
#define H264_MULTI_BIT_COUNT (0xc1a * 4)
#define H264_MULTI_BUFFER_INDEX (0xc24 * 4)
#define H264_MULTI_BUFFER_DATA (0xc22 * 4)
#define H264_MULTI_POC_RESET (0xc30 * 4)
#define H264_MULTI_POC (0xc32 * 4)
#define H264_MULTI_MV_WRITE (0xc38 * 4)
#define H264_MULTI_MV_READ (0xc39 * 4)
#define H264_MULTI_MV_CONTROL (0xc3d * 4)
#define H264_MULTI_CUR_CANVAS (0x9b3 * 4)
#define H264_MULTI_REC_CANVAS (0x9b2 * 4)
#define H264_MULTI_DBKR_CANVAS (0x9b0 * 4)
#define H264_MULTI_DBKW_CANVAS (0x9b1 * 4)
#define H264_MULTI_IQIDCT (0xe0e * 4)
#define H264_MULTI_VCOP_CONTROL (0xe00 * 4)
#define H264_MULTI_VLD_CONTROL (0xc18 * 4)
#define H264_MULTI_BYTE64 (0xe1d * 4)
#define H264_MULTI_PIC_MUX (0x98d * 4)
#define H264_MULTI_EXTIF (0x9e5 * 4)
#define H264_MULTI_MBY_MBX (0xc07 * 4)

struct h264_multi_surface {
    bool decoded;
    bool displayed;
    u32 info[3];
};

struct h264_multi {
    struct mutex lock;
    struct h264_dpb_stru dpb;
    struct VideoParameters video_checkpoint;
    struct h264_multi_surface surface[24];
    void *lmem;
    dma_addr_t lmem_dma;
    void *swap;
    dma_addr_t swap_dma;
    u32 retained_offset;
    u32 committed_offset;
    u32 job_end;
    u32 empty_status;
    u32 swap_wrap;
    u32 sequence_info;
    u32 sequence_level;
    u32 iqidct_control;
    u32 vcop_control;
    u32 vld_control;
    int error;
    bool swap_valid;
    bool have_input;
    bool running;
    bool initialized;
    bool waiting_capture;
    bool iqidct_valid;
    bool waiting_surface;
    bool empty;
    bool incomplete;
    bool eos_sent;
};

static int h264_multi_stop(struct amvdec_session *sess);
static int h264_multi_run(struct amvdec_session *sess);
static int h264_multi_configure_slice(struct amvdec_session *sess);
static void h264_multi_resume(struct amvdec_session *sess);

static int meson_h264_dpb_prepare_display_buf(struct amvdec_session *sess,
                               struct FrameStore *frame)
{
    struct codec_h264 *h264 = sess->priv;
    struct h264_multi *m = h264->multi;
    struct StorablePicture *pic = frame->frame;
    int index = frame->buf_spec_num;
    u32 field = V4L2_FIELD_NONE;

    if (index < 0 || index >= sess->num_dst_bufs || !pic ||
        frame->is_used != 3 || pic->non_existing ||
        (frame->data_flag & (ERROR_FLAG | NULL_FLAG))) {
        m->error = -EILSEQ;
        return -EILSEQ;
    }
    if (frame->data_flag & NODISP_FLAG) {
        struct amvdec_timestamp *ts;
        unsigned long flags;
        bool matched = false;

        /* A completed leading picture precedes the random-access boundary.
         * Retire only its exact coded-packet token, never a missing picture.
         * The DPB retains any reference hold; no CAPTURE is returned as pixels.
         */
        spin_lock_irqsave(&sess->ts_spinlock, flags);
        list_for_each_entry(ts, &sess->timestamps, list) {
            if ((u32)(frame->offset_delimiter - ts->offset) >=
                (u32)(ts->end_offset - ts->offset))
                continue;
            if (!ts->remaining)
                break;
            matched = true;
            if (!--ts->remaining) {
                list_del(&ts->list);
                kfree(ts);
                atomic_dec(&sess->esparser_queued_bufs);
            }
            break;
        }
        spin_unlock_irqrestore(&sess->ts_spinlock, flags);
        if (!matched) {
            m->error = -ENOENT;
            return m->error;
        }
        frame->is_output = true;
        frame->show_frame = false;
        schedule_work(&sess->esparser_queue_work);
        return 0;
    }
    if (m->surface[index].displayed)
        return -EAGAIN;
    if (!pic->coded_frame || pic->mb_aff_frame_flag)
        field = pic->top_poc <= pic->bottom_poc ?
            V4L2_FIELD_INTERLACED_TB : V4L2_FIELD_INTERLACED_BT;
    m->surface[index].displayed = true;
    frame->show_frame = true;
    sess->field = field;
    amvdec_dst_buf_done_idx(sess, index, frame->offset_delimiter, field);
    return 0;
}

static int meson_h264_dpb_release_buf_spec_num(struct amvdec_session *sess, int index)
{
    struct codec_h264 *h264 = sess->priv;

    if (index < 0 || index >= ARRAY_SIZE(h264->multi->surface))
        return -EINVAL;
    h264->multi->surface[index].decoded = false;
    return 0;
}

static int meson_h264_dpb_get_free_buf_idx(struct amvdec_session *sess)
{
    struct codec_h264 *h264 = sess->priv;
    struct h264_multi *m = h264->multi;
    unsigned int i;

    for (i = 0; i < sess->num_dst_bufs; i++) {
        if (m->surface[i].decoded || m->surface[i].displayed)
            continue;
        m->surface[i].decoded = true;
        return i;
    }
    return -ENOBUFS;
}



static int h264_multi_pause(struct amvdec_session *sess)
{
    struct amvdec_core *core = sess->core;
    u32 value;
    int ret;

    amvdec_write_dos(core, MPSR, 0);
    amvdec_write_dos(core, CPSR, 0);
    sess->processor_started = false;
    ((struct codec_h264 *)sess->priv)->multi->running = false;
    /* Vendor amvdec_stop waits for both microprocessor DMA engines. */
    ret = read_poll_timeout(amvdec_read_dos, value, !(value & BIT(15)),
                            10, 100000, false, core, IMEM_DMA_CTRL);
    if (ret)
        return ret;
    ret = read_poll_timeout(amvdec_read_dos, value, !(value & BIT(15)),
                            10, 100000, false, core, LMEM_DMA_CTRL);
    if (ret)
        return ret;
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

static int h264_multi_swap(struct amvdec_session *sess, bool save, u32 *fifo_len)
{
    struct codec_h264 *h264 = sess->priv;
    struct h264_multi *m = h264->multi;
    struct amvdec_core *core = sess->core;
    u32 value;
    int ret;

    amvdec_write_dos(core, VLD_MEM_VIFIFO_CONTROL, save ? BIT(15) : 0);
    amvdec_write_dos(core, H264_MULTI_SWAP_ADDR, m->swap_dma);
    amvdec_write_dos(core, H264_MULTI_SWAP_CTL, save ? 3 : 1);
    ret = read_poll_timeout(amvdec_read_dos, value, !(value & BIT(7)),
                            10, 100000, false, core, H264_MULTI_SWAP_CTL);
    if (ret)
        return ret;
    amvdec_write_dos(core, H264_MULTI_SWAP_CTL, 0);
    if (save) {
        u32 read = amvdec_read_dos(core, VLD_MEM_VIFIFO_RP);

        if (read < sess->vififo_paddr ||
            read >= sess->vififo_paddr + sess->vififo_size)
            return -EIO;
        WRITE_ONCE(m->retained_offset, read - sess->vififo_paddr);
        m->swap_wrap = amvdec_read_dos(core, VLD_MEM_VIFIFO_WRAP_COUNT);
        m->swap_valid = true;
    } else {
        amvdec_write_dos(core, VLD_MEM_VIFIFO_WRAP_COUNT, m->swap_wrap);
        /* Snapshot the restored reservoir before CONTROL/WP replace LEVEL. */
        *fifo_len = amvdec_read_dos(core, VLD_MEM_VIFIFO_LEVEL);
    }
    return 0;
}

static void h264_multi_load_rpm(struct h264_multi *m)
{
    const u16 *words = m->lmem;
    unsigned int i, j;

    dma_rmb();
    for (i = 0; i < RPM_END; i += 4)
        for (j = 0; j < 4; j++)
            m->dpb.dpb_param.l.data[i + j] = words[i + 3 - j];
}

static int h264_multi_start(struct amvdec_session *sess)
{
    struct amvdec_core *core = sess->core;
    struct codec_h264 *h264 = sess->priv;
    struct h264_multi *m;

    if (!h264) {
        h264 = kzalloc(sizeof(*h264), GFP_KERNEL);
        if (!h264)
            return -ENOMEM;
        sess->priv = h264;
    }
    m = kvzalloc(sizeof(*m), GFP_KERNEL);
    if (!m)
        return -ENOMEM;
    h264->multi = m;
    mutex_init(&m->lock);
    meson_h264_dpb_dpb_init_global(&m->dpb, 0, 0, 0);
    m->dpb.vdec = sess;
    m->dpb.first_insert_frame = FirstInsertFrm_IDLE;
    h264->workspace_vaddr = dma_alloc_coherent(core->dev,
        H264_MULTI_WORKSPACE_SIZE, &h264->workspace_paddr, GFP_KERNEL);
    m->lmem = dma_alloc_coherent(core->dev, PAGE_SIZE, &m->lmem_dma, GFP_KERNEL);
    m->swap = dma_alloc_coherent(core->dev, PAGE_SIZE, &m->swap_dma, GFP_KERNEL);
    if (!h264->workspace_vaddr || !m->lmem || !m->swap) {
        h264_multi_stop(sess);
        return -ENOMEM;
    }
    amvdec_write_dos_bits(core, POWER_CTL_VLD, BIT(9) | BIT(6));
    amvdec_write_dos(core, PSCALE_CTRL, 0);
    amvdec_write_dos(core, AV_SCRATCH_0, 0);
    amvdec_write_dos(core, AV_SCRATCH_2, 0);
    amvdec_write_dos(core, AV_SCRATCH_3, 0);
    amvdec_write_dos(core, AV_SCRATCH_4, 2);
    amvdec_write_dos(core, AV_SCRATCH_5, 0);
    amvdec_write_dos(core, AV_SCRATCH_7, 0);
    amvdec_write_dos(core, AV_SCRATCH_8,
                     h264->workspace_paddr + H264_MULTI_WORKSPACE_MARGIN);
    amvdec_write_dos(core, AV_SCRATCH_9, BIT(2));
    amvdec_write_dos(core, AV_SCRATCH_I, 0);
    amvdec_write_dos(core, AV_SCRATCH_J, 0);
    amvdec_write_dos(core, AV_SCRATCH_L, m->lmem_dma);
    amvdec_write_dos(core, AV_SCRATCH_M, 0);
    amvdec_write_dos(core, AV_SCRATCH_N, 0);
    amvdec_write_dos(core, AV_SCRATCH_F,
        (amvdec_read_dos(core, AV_SCRATCH_F) & 0xffffff03) | BIT(4));
    amvdec_clear_dos_bits(core, H264_MULTI_PIC_MUX, BIT(31));
    amvdec_write_dos(core, H264_MULTI_EXTIF, 0);
    amvdec_write_dos(core, MDEC_PIC_DC_THRESH, 0x404038aa);
    return 0;
}

static int h264_multi_stop(struct amvdec_session *sess)
{
    struct codec_h264 *h264 = sess->priv;
    struct h264_multi *m = h264->multi;
    struct device *dev = sess->core->dev;

    if (!m)
        return 0;
    if (m->lmem)
        dma_free_coherent(dev, PAGE_SIZE, m->lmem, m->lmem_dma);
    if (m->swap)
        dma_free_coherent(dev, PAGE_SIZE, m->swap, m->swap_dma);
    if (h264->workspace_vaddr)
        dma_free_coherent(dev, H264_MULTI_WORKSPACE_SIZE,
                           h264->workspace_vaddr, h264->workspace_paddr);
    if (h264->ref_vaddr)
        dma_free_coherent(dev, h264->ref_size,
                           h264->ref_vaddr, h264->ref_paddr);
    h264->workspace_vaddr = NULL;
    h264->ref_vaddr = NULL;
    h264->multi = NULL;
    kvfree(m);
    return 0;
}

static int h264_multi_run(struct amvdec_session *sess)
{
    struct codec_h264 *h264 = sess->priv;
    struct h264_multi *m = h264->multi;
    struct amvdec_core *core = sess->core;
    u32 size = 0, rp;
    int ret;

    if (m->error)
        return m->error;
    if (m->running || m->waiting_capture || m->waiting_surface ||
        (m->empty && m->job_end == m->committed_offset))
        return 0;
    if (!m->have_input)
        return 0;
    if (sess->processor_started) {
        ret = h264_multi_pause(sess);
        if (ret)
            return ret;
    }
    /* Restore only a checkpoint made after a real PIC_DATA_DONE. */
    ret = amvdec_vdec1_reset(sess, 0);
    if (ret)
        return ret;
    amvdec_write_dos(core, POWER_CTL_VLD, BIT(4));
    amvdec_write_dos_bits(core, POWER_CTL_VLD, BIT(9) | BIT(6));
    if (m->swap_valid) {
        ret = h264_multi_swap(sess, false, &size);
        if (ret)
            return ret;
    } else {
        amvdec_write_dos(core, VLD_MEM_VIFIFO_START_PTR, sess->vififo_paddr);
        amvdec_write_dos(core, VLD_MEM_VIFIFO_END_PTR,
                         sess->vififo_paddr + sess->vififo_size - 8);
        amvdec_write_dos(core, VLD_MEM_VIFIFO_CURR_PTR, sess->vififo_paddr);
        amvdec_write_dos(core, VLD_MEM_VIFIFO_CONTROL, 1);
        amvdec_write_dos(core, VLD_MEM_VIFIFO_CONTROL, 0);
        amvdec_write_dos(core, VLD_MEM_VIFIFO_BUF_CNTL, 2);
        amvdec_write_dos(core, VLD_MEM_VIFIFO_RP, sess->vififo_paddr);
    }
    rp = amvdec_read_dos(core, VLD_MEM_VIFIFO_RP);
    if (rp < sess->vififo_paddr || rp >= sess->vififo_paddr + sess->vififo_size)
        return -EIO;
    size += (sess->stream_write + sess->vififo_size -
             (rp - sess->vififo_paddr)) % sess->vififo_size;
    /* RP and occupancy must belong to the same stopped reservoir. */
    amvdec_write_dos(core, VLD_MEM_VIFIFO_CONTROL,
                     (0x11 << 16) | BIT(10) | (7 << 3));
    amvdec_write_dos(core, VLD_MEM_VIFIFO_WP,
                     sess->vififo_paddr + sess->stream_write);
    if (!size)
        return 0;
    m->video_checkpoint = m->dpb.mVideo;
    m->job_end = m->committed_offset;
    m->empty = false;
    m->incomplete = false;
    amvdec_write_dos(core, AV_SCRATCH_8,
        h264->workspace_paddr + H264_MULTI_WORKSPACE_MARGIN);
    amvdec_write_dos(core, AV_SCRATCH_L, m->lmem_dma);
    amvdec_write_dos(core, AV_SCRATCH_F,
        (amvdec_read_dos(core, AV_SCRATCH_F) & 0xffffff03) | BIT(4));
    if (m->dpb.mDPB.size)
        amvdec_write_dos(core, AV_SCRATCH_7, (h264->max_refs << 24) |
            (m->dpb.mDPB.size << 16) | (m->dpb.mDPB.size << 8));
    amvdec_write_dos(core, AV_SCRATCH_2, m->initialized);
    amvdec_write_dos(core, AV_SCRATCH_3, 0);
    amvdec_write_dos(core, AV_SCRATCH_4, 2);
    amvdec_write_dos(core, AV_SCRATCH_5, m->sequence_info);
    amvdec_write_dos(core, AV_SCRATCH_J,
        m->initialized ? H264_ACTION_DECODE_START : 0);
    amvdec_write_dos(core, AV_SCRATCH_I, m->dpb.decode_pic_count);
    amvdec_write_dos(core, M4_CONTROL_REG, BIT(13));
    amvdec_write_dos(core, AV_SCRATCH_E, size);
    amvdec_write_dos(core, H264_MULTI_BIT_COUNT, size * 8);
    amvdec_write_dos_bits(core, VLD_MEM_VIFIFO_CONTROL, BIT(2) | BIT(1));
    amvdec_write_dos(core, AV_SCRATCH_9, BIT(2) |
        ((m->sequence_level & 0xff) << 7) |
        (m->dpb.bitstream_restriction_flag << 15));
    amvdec_write_dos_bits(core, MDEC_PIC_DC_CTRL, BIT(17));
    amvdec_clear_dos_bits(core, MDEC_PIC_DC_CTRL, BIT(16) | BIT(31));
    amvdec_write_dos(core, MDEC_PIC_DC_THRESH, 0x404038aa);
    /* The VLD/IQIDCT reset loses controls established by the parsed PPS. */
    if (m->iqidct_valid)
        amvdec_write_dos(core, H264_MULTI_IQIDCT, m->iqidct_control);
    else if (m->initialized)
        amvdec_write_dos(core, H264_MULTI_IQIDCT, 0x200);
    if (m->vcop_control)
        amvdec_write_dos(core, H264_MULTI_VCOP_CONTROL, m->vcop_control);
    if (m->vld_control)
        amvdec_write_dos(core, H264_MULTI_VLD_CONTROL, m->vld_control);
    amvdec_write_dos(core, MPSR, 1);
    amvdec_write_dos(core, AV_SCRATCH_J, H264_ACTION_SEARCH_HEAD);
    sess->processor_started = true;
    m->running = true;
    m->initialized = true;
    return 0;
}

static void codec_h264_input_committed(struct amvdec_session *sess,
                                        u32 offset, u32 bytes)
{
    struct codec_h264 *h264 = sess->priv;
    struct h264_multi *m;
    int ret;

    if (sess->core->platform->revision != VDEC_REVISION_SC2)
        return;
    m = h264->multi;
    mutex_lock(&m->lock);
    m->committed_offset = offset + bytes;
    m->have_input |= bytes != 0;
    ret = h264_multi_run(sess);
    if (ret < 0)
        m->error = ret;
    mutex_unlock(&m->lock);
    if (ret < 0)
        amvdec_abort(sess);
}

static u32 codec_h264_input_read_pointer(struct amvdec_session *sess)
{
    struct codec_h264 *h264 = sess->priv;

    if (sess->core->platform->revision != VDEC_REVISION_SC2 || !h264->multi)
        return amvdec_read_dos(sess->core, VLD_MEM_VIFIFO_RP);
    return sess->vififo_paddr + READ_ONCE(h264->multi->retained_offset);
}

/* Validate the fixed vendor RPM representation before any DPB mutation. */
static int h264_multi_validate_rpm(struct codec_h264 *h264)
{
    struct h264_multi *m = h264->multi;
    union param *rpm = &m->dpb.dpb_param;
    u32 log_frame = rpm->l.data[LOG2_MAX_FRAME_NUM];
    u32 log_poc = rpm->l.data[LOG2_MAX_PIC_ORDER_CNT_LSB];
    u32 poc_type = rpm->l.data[PIC_ORDER_CNT_TYPE];
    u32 structure = rpm->l.data[NEW_PICTURE_STRUCTURE];
    u32 type = rpm->l.data[SLICE_TYPE];
    u32 mbaff = (rpm->dpb.picture_structure_mmco & 3) == 3;
    u32 total = h264->mb_width * h264->mb_height;
    u32 max_pic_num;
    unsigned int list, i, j;

    if (!m->dpb.mDPB.size || log_frame < 4 || log_frame > 16 ||
        log_poc > 16 || poc_type > 2 || (poc_type == 0 && log_poc < 4) ||
        rpm->l.data[NUM_REF_FRAMES_IN_PIC_ORDER_CNT_CYCLE] >
            ARRAY_SIZE(rpm->mmco.offset_for_ref_frame_base) ||
        rpm->l.data[MAX_REFERENCE_FRAME_NUM] >= h264->max_refs ||
        structure > 3 ||
        (type != I_Slice && type != P_Slice && type != B_Slice) ||
        rpm->dpb.num_ref_idx_l0_active_minus1 >= 32 ||
        rpm->dpb.num_ref_idx_l1_active_minus1 >= 32)
        return -EILSEQ;
    if (structure == 3)
        structure = FRAME;
    if ((m->dpb.mSPS.frame_mbs_only_flag && (structure != FRAME || mbaff)) ||
        (structure != FRAME && mbaff))
        return -EILSEQ;
    if (structure != FRAME || mbaff)
        total /= 2;
    if (rpm->l.data[FIRST_MB_IN_SLICE] >= total ||
        rpm->dpb.frame_num >= (1U << log_frame) ||
        (poc_type == 0 && rpm->dpb.pic_order_cnt_lsb >= (1U << log_poc)))
        return -EILSEQ;
    if (m->dpb.mVideo.dec_picture) {
        struct StorablePicture *pic = m->dpb.mVideo.dec_picture;

        if (pic->frame_num != rpm->dpb.frame_num || pic->structure != structure ||
            pic->idr_flag != ((rpm->dpb.NAL_info_mmco & 0x1f) == 5) ||
            pic->used_for_reference != !!(rpm->dpb.NAL_info_mmco & 0x60))
            return -EILSEQ;
    }
    max_pic_num = (1U << log_frame) * (structure == FRAME ? 1 : 2);

    for (list = 0; list < (type == B_Slice ? 2 : type == P_Slice); list++) {
        const u16 *cmd = list ? rpm->mmco.l1_reorder_cmd :
            rpm->mmco.l0_reorder_cmd;
        u32 active = 1 + (list ? rpm->dpb.num_ref_idx_l1_active_minus1 :
                          rpm->dpb.num_ref_idx_l0_active_minus1);

        for (i = 0, j = 0; i < REORDERING_COMMAND_MAX_SIZE; i++) {
            u32 op;

            if (j >= ARRAY_SIZE(rpm->mmco.l0_reorder_cmd))
                return -EILSEQ;
            op = cmd[j++];
            if (op == 3)
                break;
            if (op > 2 || i >= active ||
                j >= ARRAY_SIZE(rpm->mmco.l0_reorder_cmd))
                return -EILSEQ;
            if (op < 2 && cmd[j] >= max_pic_num)
                return -EILSEQ;
            j++;
        }
        if (i == REORDERING_COMMAND_MAX_SIZE)
            return -EILSEQ;
    }
    if (!(rpm->dpb.NAL_info_mmco & 0x60) ||
        (rpm->dpb.NAL_info_mmco & 0x1f) == 5)
        return 0;
    for (i = 0; i < ARRAY_SIZE(rpm->mmco.mmco_cmd);) {
        u32 op = rpm->mmco.mmco_cmd[i++];
        u32 operands = op == 3 ? 2 : op == 5 ? 0 : 1;

        if (!op)
            return 0;
        if (op > 6 || operands > ARRAY_SIZE(rpm->mmco.mmco_cmd) - i)
            return -EILSEQ;
        if ((op == 1 || op == 3) && rpm->mmco.mmco_cmd[i] >= max_pic_num)
            return -EILSEQ;
        i += operands;
    }
    return -EILSEQ;
}


static int h264_multi_configure_slice(struct amvdec_session *sess)
{
    struct amvdec_core *core = sess->core;
    struct codec_h264 *h264 = sess->priv;
    struct h264_multi *m = h264->multi;
    struct h264_dpb_stru *dpb = &m->dpb;
    struct Slice *slice = &dpb->mSlice;
    struct StorablePicture *pic = dpb->mVideo.dec_picture;
    u32 canvas, value, offset, base, reduced, slot;
    unsigned int i, j, list, direct8;
    int ret;

    if (!pic)
        return -EILSEQ;
    if (pic->buf_spec_num < 0) {
        pic->buf_spec_num = meson_h264_dpb_get_free_buf_idx(sess);
        if (pic->buf_spec_num < 0) {
            m->waiting_surface = true;
            return 0;
        }
        pic->buf_spec_is_alloced = 1;
        if (pic->used_for_reference)
            pic->colocated_buf_index = meson_h264_dpb_allocate_colocate_buf(dpb);
    }
    if (pic->used_for_reference && pic->colocated_buf_index < 0)
        return -ENOBUFS;
    if (pic->buf_spec_num >= sess->num_dst_bufs ||
        pic->buf_spec_num >= ARRAY_SIZE(m->surface))
        return -EILSEQ;
    amvdec_write_dos(core, H264_MULTI_POC_RESET, 0);
    amvdec_write_dos(core, H264_MULTI_POC, pic->frame_poc);
    amvdec_write_dos(core, H264_MULTI_POC, pic->top_poc);
    amvdec_write_dos(core, H264_MULTI_POC, pic->bottom_poc);
    amvdec_write_dos(core, H264_MULTI_CUR_CANVAS, pic->buf_spec_num << 24);
    canvas = amvdec_read_dos(core, H264_MULTI_CUR_CANVAS) & 0xffffff;
    amvdec_write_dos(core, H264_MULTI_REC_CANVAS, canvas);
    amvdec_write_dos(core, H264_MULTI_DBKR_CANVAS, canvas);
    amvdec_write_dos(core, H264_MULTI_DBKW_CANVAS, canvas);
    value = pic->mb_aff_frame_flag ? 0xf4c0 :
        pic->structure == TOP_FIELD ? 0xf400 :
        pic->structure == BOTTOM_FIELD ? 0xf440 : 0xf480;
    if (pic->bottom_poc < pic->top_poc)
        value |= BIT(8);
    m->surface[pic->buf_spec_num].info[0] = value;
    m->surface[pic->buf_spec_num].info[1] = pic->top_poc;
    m->surface[pic->buf_spec_num].info[2] = pic->bottom_poc;
    amvdec_write_dos(core, H264_MULTI_BUFFER_INDEX, 16);
    for (i = 0; i < dpb->mDPB.size; i++) {
        int long_term = meson_h264_dpb_get_long_term_flag_by_buf_spec_num(dpb, i);

        value = m->surface[i].info[0] & ~(BIT(4) | BIT(5));
        if (long_term > 0)
            value |= (long_term & 3) << 4;
        m->surface[i].info[0] = value;
        if (i == pic->buf_spec_num)
            value |= 0xf;
        amvdec_write_dos(core, H264_MULTI_BUFFER_DATA, value);
        amvdec_write_dos(core, H264_MULTI_BUFFER_DATA, m->surface[i].info[1]);
        amvdec_write_dos(core, H264_MULTI_BUFFER_DATA, m->surface[i].info[2]);
    }
    if (h264->mb_width <= 4) {
        amvdec_write_dos_bits(core, H264_MULTI_IQIDCT, BIT(16));
        amvdec_write_dos(core, H264_MULTI_BYTE64,
            (amvdec_read_dos(core, H264_MULTI_BYTE64) & ~0xf) | 0xa);
    } else {
        amvdec_clear_dos_bits(core, H264_MULTI_IQIDCT, BIT(16));
    }
    for (list = 0; list < 2; list++) {
        u32 last = 0;
        unsigned int count = slice->listXsize[list];

        /* The DPB clips default lists to available references. PPS active
         * counts can be larger when the slice never uses those tail entries.
         * Retain the vendor's last-entry padding and validate every real ref.
         */
        if (count > 32 ||
            (slice->slice_type != I_SLICE && (!list || slice->slice_type == B_SLICE) &&
             (!count || count > slice->num_ref_idx_active[list]))) {
            dev_err(core->dev, "H264 reference list %u size=%u active=%d type=%d structure=%d\n",
                    list, count, slice->num_ref_idx_active[list],
                    slice->slice_type, pic->structure);
            return -EILSEQ;
        }
        amvdec_write_dos(core, H264_MULTI_BUFFER_INDEX, list * 8);
        for (i = 0; i < 32; i += 4) {
            value = 0;
            for (j = 0; j < 4; j++) {
                if (i + j < count) {
                    struct StorablePicture *ref = slice->listX[list][i + j];
                    u32 structure;

                    if (!ref || ref == dpb->mVideo.no_reference_picture ||
                        ref->non_existing || ref->buf_spec_num < 0 ||
                        ref->buf_spec_num >= sess->num_dst_bufs ||
                        !m->surface[ref->buf_spec_num].decoded) {
                        dev_err(core->dev, "H264 reference list %u entry=%u absent=%d nonexisting=%d surface=%d\n",
                                list, i + j, !ref || ref == dpb->mVideo.no_reference_picture,
                                ref ? ref->non_existing : -1,
                                ref ? ref->buf_spec_num : -1);
                        return -EILSEQ;
                    }
                    structure = ref->structure == TOP_FIELD ? 1 :
                        ref->structure == BOTTOM_FIELD ? 2 : 3;
                    last = ref->buf_spec_num | (structure << 5);
                }
                value = (value << 8) | last;
            }
            amvdec_write_dos(core, H264_MULTI_BUFFER_DATA, value);
        }
    }
    ret = read_poll_timeout(amvdec_read_dos, value, !(value & BIT(11)),
                            10, 100000, false, core, H264_MULTI_MV_CONTROL);
    if (ret)
        return ret;
    direct8 = (slice->mode_8x8_flags & 6) == 6;
    reduced = direct8 ? 2 : 0;
    slot = dpb->colocated_buf_size >> reduced;
    offset = (pic->structure == FRAME && !pic->mb_aff_frame_flag ? 96 : 192);
    offset = (offset >> reduced) * slice->first_mb_in_slice;
    if (pic->colocated_buf_index >= 0) {
        if (pic->colocated_buf_index >= dpb->colocated_buf_count)
            return -EILSEQ;
        base = dpb->colocated_mv_addr_start +
            ((dpb->colocated_buf_size * pic->colocated_buf_index) >> reduced);
        if (offset >= slot || base < dpb->colocated_mv_addr_start ||
            base > dpb->colocated_mv_addr_end ||
            slot > dpb->colocated_mv_addr_end - base)
            return -EILSEQ;
        amvdec_write_dos(core, H264_MULTI_MV_WRITE, base + offset);
    } else {
        amvdec_write_dos(core, H264_MULTI_MV_WRITE, U32_MAX);
    }
    if (slice->listXsize[1]) {
        struct StorablePicture *ref = slice->listX[1][0];
        unsigned int ref_structure = ref->mb_aff_frame_flag ? 3 :
            ref->coded_frame ? 2 : ref->structure == BOTTOM_FIELD;
        unsigned int structure = pic->mb_aff_frame_flag ? 3 :
            pic->coded_frame ? 2 : pic->structure == BOTTOM_FIELD;
        u32 x = slice->first_mb_in_slice % h264->mb_width;
        u32 y = slice->first_mb_in_slice / h264->mb_width;
        u32 parity;

        if (ref->colocated_buf_index < 0 ||
            ref->colocated_buf_index >= dpb->colocated_buf_count) {
            dev_err(core->dev, "H264 colocated reference slot=%d count=%u reference=%d\n",
                    ref->colocated_buf_index, dpb->colocated_buf_count,
                    ref->used_for_reference);
            return -EILSEQ;
        }
        if (structure < 2)
            offset = ref_structure != 2 ? slice->first_mb_in_slice * 2 :
                y * 2 * h264->mb_width + x;
        else if (ref_structure < 2)
            offset = ((y / 2) * h264->mb_width + x) * 2;
        else
            offset = slice->first_mb_in_slice * (structure == 2 ? 1 : 2);
        offset = (offset * 96) >> reduced;
        if (pic->structure == FRAME || pic->mb_aff_frame_flag)
            parity = abs((s64)pic->poc - ref->top_poc) <
                abs((s64)pic->poc - ref->bottom_poc) ? 0 : 1;
        else
            parity = ref->structure == BOTTOM_FIELD;
        base = dpb->colocated_mv_addr_start +
            ((dpb->colocated_buf_size * ref->colocated_buf_index) >> reduced);
        if (offset >= slot || base < dpb->colocated_mv_addr_start ||
            base > dpb->colocated_mv_addr_end ||
            slot > dpb->colocated_mv_addr_end - base)
            return -EILSEQ;
        value = ((base + offset) >> 3) | (ref_structure << 30) | (parity << 29);
        amvdec_write_dos(core, H264_MULTI_MV_READ, value);
    }
    m->waiting_surface = false;
    amvdec_write_dos(core, AV_SCRATCH_J,
        pic->proc_flag ? H264_ACTION_DECODE_SLICE : H264_ACTION_DECODE_NEWPIC);
    pic->proc_flag = 1;
    return 0;
}

static int h264_multi_parameters(struct amvdec_session *sess)
{
    struct codec_h264 *h264 = sess->priv;
    struct h264_multi *m = h264->multi;
    struct amvdec_core *core = sess->core;
    struct h264_dpb_stru *dpb = &m->dpb;
    u32 parsed = amvdec_read_dos(core, AV_SCRATCH_1);
    u32 seq = amvdec_read_dos(core, AV_SCRATCH_2);
    u32 level = amvdec_read_dos(core, AV_SCRATCH_B);
    u32 crop = amvdec_read_dos(core, AV_SCRATCH_6);
    u32 width = parsed & 0xff, total = (parsed >> 8) & 0xffff;
    u32 height, capacity, refs, chroma, frame_only;
    u32 crop_left, crop_right, crop_top, crop_bottom;
    u32 tick, scale;
    bool reuse;

    if (!width && total)
        width = 256;
    if (!width || !total || total % width)
        return -EILSEQ;
    height = total / width;
    refs = (level >> 8) & 0xff;
    frame_only = !!(seq & BIT(15));
    chroma = (seq >> 13) & 3;
    if (chroma > 1 || refs > 16 || width * 16 > sess->fmt_out->max_width ||
        height * 16 > ALIGN(sess->fmt_out->max_height, 16))
        return -EINVAL;
    capacity = max(codec_h264_max_dpb(level & 0xff, width, height), refs);
    h264_multi_load_rpm(m);
    m->iqidct_control = amvdec_read_dos(core, H264_MULTI_IQIDCT);
    m->iqidct_valid = true;
    dpb->bitstream_restriction_flag =
        !!(dpb->dpb_param.l.data[SPS_FLAGS2] & BIT(3));
    dpb->num_reorder_frames = dpb->dpb_param.l.data[NUM_REORDER_FRAMES];
    dpb->max_dec_frame_buffering = dpb->dpb_param.l.data[MAX_BUFFER_FRAME];
    if (dpb->bitstream_restriction_flag) {
        if (dpb->max_dec_frame_buffering < refs ||
            dpb->max_dec_frame_buffering > capacity ||
            dpb->num_reorder_frames > dpb->max_dec_frame_buffering)
            return -EILSEQ;
        capacity = max(1U, (u32)dpb->max_dec_frame_buffering);
        dpb->reorder_output = dpb->num_reorder_frames + 1;
    } else {
        dpb->reorder_output = capacity + 1;
    }
    if (capacity + 6 > sess->fmt_out->max_buffers)
        return -ENOBUFS;
    /* Repeated SPS/config mailboxes must not destroy a decoded first field. */
    reuse = dpb->mDPB.init_done && width == h264->mb_width &&
        height == h264->mb_height && refs + 1 == h264->max_refs &&
        capacity == h264->max_dpb &&
        frame_only == dpb->mSPS.frame_mbs_only_flag;
    if (!reuse && dpb->mVideo.dec_picture)
        return -EILSEQ;
    if (dpb->mDPB.init_done && !reuse) {
        unsigned int i;

        for (i = 0; i < dpb->mDPB.used_size; i++)
            if (dpb->mDPB.fs[i]->is_used != 3)
                return -EILSEQ;
        meson_h264_dpb_flush_dpb(dpb);
        if (m->error)
            return m->error;
    }
    if (!reuse) {
        meson_h264_dpb_dpb_init_global(dpb, 0, 0, refs + 1);
        dpb->vdec = sess;
        dpb->first_insert_frame = FirstInsertFrm_IDLE;
        m->video_checkpoint = dpb->mVideo;
    }
    h264->mb_width = width;
    h264->mb_height = height;
    h264->max_refs = refs + 1;
    h264->max_dpb = capacity;
    h264->seq_info = seq;
    h264->level = level & 0xff;
    m->sequence_info = parsed;
    m->sequence_level = level;
    dpb->mSPS.level_idc = level & 0xff;
    dpb->mSPS.frame_mbs_only_flag = frame_only;
    /* CONFIG publishes pixel crops, not four independent SPS RPM offsets. */
    crop_left = (crop >> 24) & 0xff;
    crop_right = (crop >> 16) & 0xff;
    crop_top = (crop >> 8) & 0xff;
    crop_bottom = crop & 0xff;
    if (crop_left + crop_right >= width * 16 ||
        crop_top + crop_bottom >= height * 16)
        return -EILSEQ;
    sess->crop_left = crop_left;
    sess->crop_top = crop_top;
    sess->visible_width = width * 16 - crop_left - crop_right;
    sess->visible_height = height * 16 - crop_top - crop_bottom;
    sess->field = frame_only ? V4L2_FIELD_NONE : V4L2_FIELD_INTERLACED;
    sess->keyframe_found = 1;
    tick = val(&dpb->dpb_param.l.data[NUM_UNITS_IN_TICK]);
    scale = val(&dpb->dpb_param.l.data[TIME_SCALE]);
    if (tick && tick <= U32_MAX / 2 && scale) {
        sess->timeperframe.numerator = 2 * tick;
        sess->timeperframe.denominator = scale;
    }
    if (dpb->dpb_param.l.data[ASPECT_RATIO_IDC] == AR_EXTEND) {
        sess->pixelaspect.numerator = dpb->dpb_param.l.data[ASPECT_RATIO_SAR_WIDTH];
        sess->pixelaspect.denominator = dpb->dpb_param.l.data[ASPECT_RATIO_SAR_HEIGHT];
    } else if (dpb->dpb_param.l.data[ASPECT_RATIO_IDC] < ARRAY_SIZE(par_table)) {
        sess->pixelaspect = par_table[dpb->dpb_param.l.data[ASPECT_RATIO_IDC]];
    }
    if (reuse) {
        amvdec_write_dos(core, AV_SCRATCH_J, H264_ACTION_CONFIG_DONE);
        amvdec_write_dos(core, AV_SCRATCH_0, (h264->max_refs << 24) |
            (dpb->mDPB.size << 16) | (dpb->mDPB.size << 8));
        return 0;
    }
    m->waiting_capture = true;
    return 1;
}

static void h264_multi_resume(struct amvdec_session *sess)
{
    struct codec_h264 *h264 = sess->priv;
    struct h264_multi *m = h264->multi;
    struct h264_dpb_stru *dpb = &m->dpb;
    struct amvdec_core *core = sess->core;
    u32 stride_mbs = ALIGN(h264->mb_width, 4);
    u32 height_mbs = ALIGN(h264->mb_height, 4);
    u32 bytes = stride_mbs * height_mbs * 96 * h264->max_refs;
    int ret;

    mutex_lock(&m->lock);
    if (!m->waiting_capture || !bytes ||
        sess->num_dst_bufs < h264->max_dpb + 6 ||
        sess->num_dst_bufs > ARRAY_SIZE(m->surface)) {
        ret = -EINVAL;
        goto error;
    }
    ret = amvdec_set_canvases(sess, (u32[]){ANC0_CANVAS_ADDR, 0},
                             (u32[]){24, 0});
    if (ret)
        goto error;
    if (h264->ref_vaddr)
        dma_free_coherent(core->dev, h264->ref_size,
                           h264->ref_vaddr, h264->ref_paddr);
    h264->ref_vaddr = NULL;
    h264->ref_size = bytes;
    h264->ref_vaddr = dma_alloc_coherent(core->dev, bytes,
                                        &h264->ref_paddr, GFP_KERNEL);
    if (!h264->ref_vaddr) {
        ret = -ENOMEM;
        goto error;
    }
    memset(m->surface, 0, sizeof(m->surface));
    dpb->mDPB.size = sess->num_dst_bufs;
    dpb->max_reference_size = h264->max_refs;
    dpb->colocated_buf_size = stride_mbs * height_mbs * 96;
    dpb->colocated_mv_addr_start = h264->ref_paddr;
    dpb->colocated_mv_addr_end = h264->ref_paddr + bytes;
    m->waiting_capture = false;
    amvdec_write_dos(core, AV_SCRATCH_J, H264_ACTION_CONFIG_DONE);
    /* CONFIG_PARAM waits for allocation in scratch 0; scratch 7 is run restore. */
    amvdec_write_dos(core, AV_SCRATCH_0, (h264->max_refs << 24) |
        (sess->num_dst_bufs << 16) | (sess->num_dst_bufs << 8));
    mutex_unlock(&m->lock);
    return;
error:
    m->error = ret;
    mutex_unlock(&m->lock);
    amvdec_abort(sess);
}


static irqreturn_t h264_multi_threaded_isr(struct amvdec_session *sess)
{
    struct codec_h264 *h264 = sess->priv;
    struct h264_multi *m = h264->multi;
    struct h264_dpb_stru *dpb = &m->dpb;
    struct amvdec_core *core = sess->core;
    u32 status = amvdec_read_dos(core, AV_SCRATCH_J);
    int ret = 0, gap = 0;
    bool source_change = false;

    mutex_lock(&m->lock);
    dev_dbg(core->dev, "H264 MULTI status=%02x bitcount=%u input=%08x/%08x dpb=%u\n",
             status, amvdec_read_dos(core, H264_MULTI_BIT_COUNT),
             m->job_end, m->committed_offset, dpb->mDPB.used_size);
    switch (status) {
    case H264_CONFIG_REQUEST:
        ret = h264_multi_parameters(sess);
        source_change = ret > 0;
        break;
    case H264_SLICE_HEAD_DONE:
        if (m->waiting_surface)
            break;
        h264_multi_load_rpm(m);
        m->iqidct_control = amvdec_read_dos(core, H264_MULTI_IQIDCT);
        m->iqidct_valid = true;
        m->vcop_control = amvdec_read_dos(core, H264_MULTI_VCOP_CONTROL);
        m->vld_control = amvdec_read_dos(core, H264_MULTI_VLD_CONTROL);
        ret = h264_multi_validate_rpm(h264);
        if (ret < 0) {
            dev_err(core->dev, "H264 MULTI invalid RPM: %d\n", ret);
            break;
        }
        ret = meson_h264_dpb_h264_slice_header_process(dpb, &gap);
        if (ret < 0 || dpb->dpb_error_flag ||
            (gap && !dpb->mSPS.frame_num_gap_allowed) ||
            dpb->mSlice.slice_type >= NUM_SLICE_TYPES) {
            dev_err(core->dev, "H264 MULTI DPB failed: ret=%d error=%d gap=%d allowed=%d type=%d\n",
                    ret, dpb->dpb_error_flag, gap,
                    dpb->mSPS.frame_num_gap_allowed, dpb->mSlice.slice_type);
            ret = -EILSEQ;
            break;
        }
        ret = h264_multi_configure_slice(sess);
        if (ret < 0)
            dev_err(core->dev, "H264 MULTI reference/MV configuration failed: %d\n", ret);
        break;
    case H264_PIC_DATA_DONE: {
        struct StorablePicture *pic = dpb->mVideo.dec_picture;
        unsigned char flags = 0;

        ret = h264_multi_pause(sess);
        if (ret)
            break;
        if (!pic || pic->buf_spec_num < 0 ||
            pic->buf_spec_num >= ARRAY_SIZE(m->surface)) {
            ret = -EILSEQ;
            break;
        }
        {
            u32 position = amvdec_read_dos(core, H264_MULTI_MBY_MBX);
            u32 expected = h264->mb_width * h264->mb_height;
            u32 decoded = (position & 0xff) * h264->mb_width +
                ((position >> 8) & 0xff) + 1;

            if (pic->structure != FRAME)
                expected /= 2;
            if (amvdec_read_dos(core, AV_SCRATCH_9) ||
                (position && decoded < expected)) {
                ret = -EILSEQ;
                break;
            }
        }
        if (dpb->mSlice.slice_type == I_SLICE)
            flags |= I_FLAG;
        if (pic->idr_flag)
            flags |= IDR_FLAG;
        ret = meson_h264_dpb_store_picture_in_dpb(dpb, pic, flags);
        if (ret < 0)
            break;
        /* MMCO5 and complementary-field combination can change both POCs. */
        m->surface[pic->buf_spec_num].info[0] &= ~BIT(8);
        if (pic->bottom_poc < pic->top_poc)
            m->surface[pic->buf_spec_num].info[0] |= BIT(8);
        m->surface[pic->buf_spec_num].info[1] = pic->top_poc;
        m->surface[pic->buf_spec_num].info[2] = pic->bottom_poc;
        meson_h264_dpb_bufmgr_post(dpb);
        dpb->mVideo.dec_picture = NULL;
        dpb->decode_pic_count++;
        ret = h264_multi_swap(sess, true, NULL);
        if (!ret)
            ret = h264_multi_run(sess);
        break;
    }
    case H264_SEARCH_BUFEMPTY:
    case H264_DECODE_BUFEMPTY: {
        struct StorablePicture *pic = dpb->mVideo.dec_picture;
        struct SPSParameters *sps = dpb->mVideo.active_sps;
        struct StorablePicture *dummy = dpb->mVideo.no_reference_picture;

        ret = h264_multi_pause(sess);
        if (ret)
            break;
        m->empty = true;
        m->empty_status = status;
        m->incomplete = pic != NULL;
        if (pic) {
            meson_h264_dpb_release_picture(dpb, pic);
            dpb->mVideo = m->video_checkpoint;
            dpb->mVideo.active_sps = sps;
            dpb->mVideo.no_reference_picture = dummy;
            dpb->mVideo.dec_picture = NULL;
        }
        /* Replay unfinished coded input from the last completed picture. */
        if (m->job_end != m->committed_offset)
            ret = h264_multi_run(sess);
        break;
    }
    case H264_SEI_DATA_READY:
        amvdec_write_dos(core, AV_SCRATCH_J, H264_SEI_DATA_DONE);
        break;
    case H264_AUX_DATA_READY:
        ret = h264_multi_pause(sess);
        if (ret)
            break;
        if (dpb->mVideo.dec_picture) {
            ret = -EILSEQ;
            break;
        }
        ret = h264_multi_swap(sess, true, NULL);
        if (!ret)
            ret = h264_multi_run(sess);
        break;
    case H264_DECODE_TIMEOUT:
        ret = -ETIMEDOUT;
        break;
    case H264_DECODE_OVER_SIZE:
        ret = -EOVERFLOW;
        break;
    case 0:
        break;
    default:
        dev_err(core->dev, "Unexpected H264 MULTI status %08x\n", status);
        ret = -EIO;
        break;
    }
    if (m->error)
        ret = m->error;
    if (ret < 0) {
        dev_err(core->dev, "H264 MULTI failed: %d status=%02x\n", ret, status);
        m->error = ret;
        h264_multi_pause(sess);
    }
    mutex_unlock(&m->lock);
    if (ret < 0)
        amvdec_abort(sess);
    else if (source_change)
        amvdec_src_change(sess, h264->mb_width * 16, h264->mb_height * 16,
                          h264->max_dpb + 6);
    return IRQ_HANDLED;
}

static void h264_multi_recycle(struct amvdec_core *core, u32 index)
{
    struct amvdec_session *sess = core->cur_sess;
    struct codec_h264 *h264 = sess->priv;
    struct h264_multi *m = h264->multi;
    struct h264_dpb_stru *dpb = &m->dpb;
    unsigned int i;
    int ret = 0;

    mutex_lock(&m->lock);
    if (index >= sess->num_dst_bufs || !m->surface[index].displayed) {
        ret = -EINVAL;
        goto error;
    }
    m->surface[index].displayed = false;
    for (i = 0; i < dpb->mDPB.used_size; i++) {
        struct FrameStore *frame = dpb->mDPB.fs[i];

        if (frame->buf_spec_num == index && frame->pre_output) {
            frame->pre_output = 0;
            frame->is_output = 1;
            frame->show_frame = false;
        }
    }
    meson_h264_dpb_bufmgr_h264_remove_unused_frame(dpb, 0);
    if (m->waiting_surface)
        ret = h264_multi_configure_slice(sess);
error:
    if (ret < 0)
        m->error = ret;
    mutex_unlock(&m->lock);
    if (ret < 0)
        amvdec_abort(sess);
}

static int codec_h264_drain_step(struct amvdec_session *sess)
{
    struct codec_h264 *h264 = sess->priv;
    struct h264_multi *m = h264->multi;
    struct h264_dpb_stru *dpb = &m->dpb;
    unsigned int i;
    int ret = 0;

    if (sess->core->platform->revision != VDEC_REVISION_SC2)
        return -EOPNOTSUPP;
    mutex_lock(&m->lock);
    if (m->error) {
        ret = m->error;
        goto unlock;
    }
    if (!m->eos_sent) {
        ret = esparser_queue_eos(sess->core, eos_sequence, sizeof(eos_sequence));
        if (ret == -EAGAIN) {
            ret = 0;
            goto unlock;
        }
        if (ret < 0)
            goto unlock;
        m->committed_offset += sizeof(eos_sequence);
        m->eos_sent = true;
        m->have_input = true;
        ret = h264_multi_run(sess);
        goto unlock;
    }
    if (m->running || m->waiting_capture || m->waiting_surface ||
        !m->empty || m->job_end != m->committed_offset)
        goto unlock;
    /* SEARCH_BUFEMPTY plus the completed-picture ledger is real drain.
     * No timeout, cached ANC surface, or missing field becomes a picture.
     */
    if (m->incomplete || m->empty_status != H264_SEARCH_BUFEMPTY) {
        ret = -EILSEQ;
        goto unlock;
    }
    for (i = 0; i < dpb->mDPB.used_size; i++) {
        if (dpb->mDPB.fs[i]->is_used != 3) {
            ret = -EILSEQ;
            goto unlock;
        }
    }
    meson_h264_dpb_flush_dpb(dpb);
    if (m->error) {
        ret = m->error;
        goto unlock;
    }
    for (i = 0; i < dpb->mDPB.used_size; i++)
        if (!dpb->mDPB.fs[i]->is_output && !dpb->mDPB.fs[i]->pre_output)
            goto unlock;
    if (atomic_read(&sess->esparser_queued_bufs)) {
        ret = -EILSEQ;
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

static int codec_h264_can_recycle(struct amvdec_core *core)
{
	if (core->platform->revision == VDEC_REVISION_SC2)
		return 1;
	return !amvdec_read_dos(core, AV_SCRATCH_7) ||
	       !amvdec_read_dos(core, AV_SCRATCH_8);
}

static void codec_h264_recycle(struct amvdec_core *core, u32 buf_idx)
{
	if (core->platform->revision == VDEC_REVISION_SC2) {
		h264_multi_recycle(core, buf_idx);
		return;
	}
	/*
	 * Tell the firmware it can recycle this buffer.
	 * AV_SCRATCH_8 serves the same purpose.
	 */
	if (!amvdec_read_dos(core, AV_SCRATCH_7))
		amvdec_write_dos(core, AV_SCRATCH_7, buf_idx + 1);
	else
		amvdec_write_dos(core, AV_SCRATCH_8, buf_idx + 1);
}

static int codec_h264_start(struct amvdec_session *sess)
{
	u32 workspace_offset;
	struct amvdec_core *core = sess->core;
	struct codec_h264 *h264 = sess->priv;

	if (core->platform->revision == VDEC_REVISION_SC2)
		return h264_multi_start(sess);
	if (!h264) {
		h264 = kzalloc(sizeof(*h264), GFP_KERNEL);
		if (!h264)
			return -ENOMEM;
		sess->priv = h264;
	}

	/* Allocate some memory for the H.264 decoder's state */
	h264->workspace_vaddr =
		dma_alloc_coherent(core->dev, SIZE_WORKSPACE,
				   &h264->workspace_paddr, GFP_KERNEL);
	if (!h264->workspace_vaddr)
		return -ENOMEM;

	/* Allocate some memory for the H.264 SEI dump */
	h264->sei_vaddr = dma_alloc_coherent(core->dev, SIZE_SEI,
					     &h264->sei_paddr, GFP_KERNEL);
	if (!h264->sei_vaddr)
		return -ENOMEM;

	amvdec_write_dos_bits(core, POWER_CTL_VLD, BIT(9) | BIT(6));

	workspace_offset = h264->workspace_paddr - WORKSPACE_BUF_OFFSET;
	amvdec_write_dos(core, AV_SCRATCH_1, workspace_offset);
	amvdec_write_dos(core, AV_SCRATCH_G, h264->ext_fw_paddr);
	amvdec_write_dos(core, AV_SCRATCH_I, h264->sei_paddr -
					     workspace_offset);
	amvdec_write_dos(core, AV_SCRATCH_0, 0);
	amvdec_write_dos(core, AV_SCRATCH_7, 0);
	amvdec_write_dos(core, AV_SCRATCH_8, 0);
	amvdec_write_dos(core, AV_SCRATCH_9, 0);
	amvdec_write_dos(core, AV_SCRATCH_N, 0);
	amvdec_write_dos(core, PSCALE_CTRL, 0);

	amvdec_write_dos(core, AV_SCRATCH_F,
		(amvdec_read_dos(core, AV_SCRATCH_F) & 0xffffffc3) |
		BIT(4) | BIT(7));

	amvdec_write_dos(core, MDEC_PIC_DC_THRESH, 0x404038aa);

	return 0;
}

static int codec_h264_stop(struct amvdec_session *sess)
{
	struct codec_h264 *h264 = sess->priv;
	struct amvdec_core *core = sess->core;

	if (core->platform->revision == VDEC_REVISION_SC2)
		return h264_multi_stop(sess);

	if (h264->ext_fw_vaddr)
		dma_free_coherent(core->dev, SIZE_EXT_FW,
				  h264->ext_fw_vaddr, h264->ext_fw_paddr);

	if (h264->workspace_vaddr)
		dma_free_coherent(core->dev, SIZE_WORKSPACE,
				  h264->workspace_vaddr, h264->workspace_paddr);

	if (h264->ref_vaddr)
		dma_free_coherent(core->dev, h264->ref_size,
				  h264->ref_vaddr, h264->ref_paddr);

	if (h264->sei_vaddr)
		dma_free_coherent(core->dev, SIZE_SEI,
				  h264->sei_vaddr, h264->sei_paddr);

	return 0;
}

static int codec_h264_load_extended_firmware(struct amvdec_session *sess,
					     const u8 *data, u32 len)
{
	struct codec_h264 *h264;
	struct amvdec_core *core = sess->core;

	if (len < SIZE_EXT_FW)
		return -EINVAL;

	h264 = kzalloc(sizeof(*h264), GFP_KERNEL);
	if (!h264)
		return -ENOMEM;

	h264->ext_fw_vaddr = dma_alloc_coherent(core->dev, SIZE_EXT_FW,
						&h264->ext_fw_paddr,
						GFP_KERNEL);
	if (!h264->ext_fw_vaddr) {
		kfree(h264);
		return -ENOMEM;
	}

	memcpy(h264->ext_fw_vaddr, data, SIZE_EXT_FW);
	sess->priv = h264;

	return 0;
}



static void codec_h264_set_par(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	u32 seq_info = amvdec_read_dos(core, AV_SCRATCH_2);
	u32 ar_idc = (seq_info >> AR_IDC_BIT) & AR_IDC_MASK;

	if (!(seq_info & AR_PRESENT_FLAG))
		return;

	if (ar_idc == AR_EXTEND) {
		u32 ar_info = amvdec_read_dos(core, AV_SCRATCH_3);

		sess->pixelaspect.numerator = ar_info & 0xffff;
		sess->pixelaspect.denominator = (ar_info >> 16) & 0xffff;
		return;
	}

	if (ar_idc >= ARRAY_SIZE(par_table))
		return;

	sess->pixelaspect = par_table[ar_idc];
}

static void codec_h264_resume(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_h264 *h264 = sess->priv;
	u32 mb_width, mb_height, mb_total;

	if (core->platform->revision == VDEC_REVISION_SC2) {
		h264_multi_resume(sess);
		return;
	}

	if (amvdec_set_canvases(sess,
				(u32[]){ ANC0_CANVAS_ADDR, 0 },
				(u32[]){ 24, 0 })) {
		amvdec_abort(sess);
		return;
	}

	dev_dbg(core->dev, "max_refs = %u; actual_dpb_size = %u\n",
		h264->max_refs, sess->num_dst_bufs);

	/* Align to a multiple of 4 macroblocks */
	mb_width = ALIGN(h264->mb_width, 4);
	mb_height = ALIGN(h264->mb_height, 4);
	mb_total = mb_width * mb_height;

	if (h264->ref_vaddr) {
		dma_free_coherent(core->dev, h264->ref_size,
				  h264->ref_vaddr, h264->ref_paddr);
		h264->ref_vaddr = NULL;
	}
	h264->ref_size = mb_total * h264->mv_bytes * h264->max_refs;
	h264->ref_vaddr = dma_alloc_coherent(core->dev, h264->ref_size,
					     &h264->ref_paddr, GFP_KERNEL);
	if (!h264->ref_vaddr) {
		amvdec_abort(sess);
		return;
	}

	/* Address to store the references' MVs */
	amvdec_write_dos(core, AV_SCRATCH_1, h264->ref_paddr);
	/* End of ref MV */
	amvdec_write_dos(core, AV_SCRATCH_4, h264->ref_paddr + h264->ref_size);

	amvdec_write_dos(core, AV_SCRATCH_0, (h264->max_refs << 24) |
					     (sess->num_dst_bufs << 16) |
					     (h264->max_dpb << 8));
}

/*
 * Configure the H.264 decoder when the parser detected a parameter set change
 */
static void codec_h264_src_change(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_h264 *h264 = sess->priv;
	u32 parsed_info, mb_total;
	u32 crop_infor, crop_bottom, crop_right;
	u32 frame_width, frame_height;

	parsed_info = amvdec_read_dos(core, AV_SCRATCH_1);
	sess->keyframe_found = 1;


	/* Total number of 16x16 macroblocks */
	mb_total = (parsed_info >> MB_TOTAL_BIT) & MB_TOTAL_MASK;
	/* Number of macroblocks per line */
	h264->mb_width = parsed_info & MB_WIDTH_MASK;
	if (!h264->mb_width || !mb_total || mb_total % h264->mb_width) {
		amvdec_abort(sess);
		return;
	}
	/* Number of macroblock lines */
	h264->mb_height = mb_total / h264->mb_width;

	h264->max_refs = ((parsed_info >> MAX_REF_BIT) & MAX_REF_MASK) + 1;
	h264->seq_info = amvdec_read_dos(core, AV_SCRATCH_2);
	h264->mv_bytes = MB_MV_SIZE;
	h264->max_dpb = h264->max_refs - 1;
	{
		u32 tick = amvdec_read_dos(core, AV_SCRATCH_4);
		u32 scale = amvdec_read_dos(core, AV_SCRATCH_5);

		/* H264 VUI clock is twice the FRAME cadence, including PAFF/MBAFF. */
		if (tick && tick <= U32_MAX / 2 && scale) {
			sess->timeperframe.numerator = 2 * tick;
			sess->timeperframe.denominator = scale;
		}
	}
	dev_dbg(core->dev,
		"H264 parameters info=%08x seq=%08x refs=%u dpb=%u level=%u mb=%ux%u\n",
		parsed_info, h264->seq_info, h264->max_refs, h264->max_dpb,
		h264->level, h264->mb_width, h264->mb_height);
	/* Baseline/Main SPSs omit chroma_format_idc; firmware reports zero. */
	if (((h264->seq_info >> 13) & 3) > 1 ||
	    h264->max_refs > 17 || h264->max_dpb + 6 > sess->fmt_out->max_buffers ||
	    h264->mb_width * 16 > sess->fmt_out->max_width ||
	    h264->mb_height * 16 > ALIGN(sess->fmt_out->max_height, 16)) {
		amvdec_abort(sess);
		return;
	}

	crop_infor = amvdec_read_dos(core, AV_SCRATCH_6);
	crop_bottom = (crop_infor & 0xff);
	crop_right = (crop_infor >> 16) & 0xff;
	sess->crop_left = (crop_infor >> 24) & 0xff;
	sess->crop_top = (crop_infor >> 8) & 0xff;

	frame_width = h264->mb_width * 16;
	frame_height = h264->mb_height * 16;
	if (crop_right + sess->crop_left >= frame_width ||
	    crop_bottom + sess->crop_top >= frame_height) {
		amvdec_abort(sess);
		return;
	}
	sess->visible_width = frame_width - crop_right - sess->crop_left;
	sess->visible_height = frame_height - crop_bottom - sess->crop_top;
	sess->field = h264->seq_info & BIT(15) ? V4L2_FIELD_NONE :
		V4L2_FIELD_INTERLACED;

	dev_dbg(core->dev, "frame: %ux%u; crop: %u %u\n",
		frame_width, frame_height, crop_right, crop_bottom);

	codec_h264_set_par(sess);
	amvdec_src_change(sess, frame_width, frame_height, h264->max_refs + 5);
}

/*
 * The bitstream offset is split in half in 2 different registers.
 * Fetch its MSB here, which location depends on the frame number.
 */
static u32 get_offset_msb(struct amvdec_core *core, int frame_num)
{
	int take_msb = frame_num % 2;
	int reg_offset = (frame_num / 2) * 4;
	u32 offset_msb = amvdec_read_dos(core, AV_SCRATCH_A + reg_offset);

	if (take_msb)
		return offset_msb & 0xffff0000;

	return (offset_msb & 0x0000ffff) << 16;
}

static void codec_h264_frames_ready(struct amvdec_session *sess, u32 status)
{
	struct amvdec_core *core = sess->core;
	int error_count;
	struct codec_h264 *h264 = sess->priv;
	int num_frames;
	int i;

	error_count = amvdec_read_dos(core, AV_SCRATCH_D);
	num_frames = (status >> 8) & 0xff;
	if (num_frames > 6) {
		amvdec_abort(sess);
		return;
	}
	if (error_count) {
		dev_warn(core->dev,
			 "decoder error(s) happened, count %d\n", error_count);
		amvdec_write_dos(core, AV_SCRATCH_D, 0);
	}

	for (i = 0; i < num_frames; i++) {
		u32 frame_status = amvdec_read_dos(core, AV_SCRATCH_1 + i * 4);
		u32 buffer_index = frame_status & BUF_IDX_MASK;
		u32 pic_struct = (frame_status >> PIC_STRUCT_BIT) &
				 PIC_STRUCT_MASK;
		u32 offset = (frame_status >> OFFSET_BIT) & OFFSET_MASK;
		u32 field = V4L2_FIELD_NONE;

		/*
		 * A buffer decode error means it was decoded,
		 * but part of the picture will have artifacts.
		 * Typical reason is a temporarily corrupted bitstream
		 */
		if (frame_status & SINGLE_ERROR_FLAG)
			dev_dbg(core->dev, "Buffer %d decode error\n",
				buffer_index);

		/* Single-firmware pic_struct 0..4 means progressive only when the
		 * SPS says picture timing is present. PAFF/MBAFF otherwise use
		 * prog_frame and POC order, even when pic_struct is zero.
		 */
		if (h264->seq_info & BIT(4)) {
			if (pic_struct == SINGLE_PIC_TOP_BOT)
				field = V4L2_FIELD_INTERLACED_TB;
			else if (pic_struct == SINGLE_PIC_BOT_TOP)
				field = V4L2_FIELD_INTERLACED_BT;
		} else if (!(h264->seq_info & BIT(15)) &&
			   !(frame_status & BIT(8))) {
			field = frame_status & BIT(9) ? V4L2_FIELD_INTERLACED_BT :
				V4L2_FIELD_INTERLACED_TB;
		}
		sess->field = field;

		offset |= get_offset_msb(core, i);

		amvdec_dst_buf_done_idx(sess, buffer_index, offset, field);
		if (sess->should_stop && (frame_status & BIT(15)))
			amvdec_eos(sess);
	}
}

static irqreturn_t codec_h264_threaded_isr(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	u32 status;
	u32 size;
	u8 cmd;

	if (core->platform->revision == VDEC_REVISION_SC2)
		return h264_multi_threaded_isr(sess);

	status = amvdec_read_dos(core, AV_SCRATCH_0);
	cmd = status & CMD_MASK;
	dev_dbg(core->dev, "H264 IRQ status=%08x info=%08x level=%08x\n",
		status, amvdec_read_dos(core, AV_SCRATCH_1),
		amvdec_read_dos(core, AV_SCRATCH_A));

	switch (cmd) {
	case CMD_SRC_CHANGE:
		codec_h264_src_change(sess);
		break;
	case CMD_FRAMES_READY:
		codec_h264_frames_ready(sess, status);
		break;
	case CMD_FATAL_ERROR:
		dev_err(core->dev, "H.264 decoder fatal error\n");
		goto abort;
	case CMD_BAD_WIDTH:
		size = (amvdec_read_dos(core, AV_SCRATCH_1) + 1) * 16;
		dev_err(core->dev, "Unsupported video width: %u\n", size);
		goto abort;
	case CMD_BAD_HEIGHT:
		size = (amvdec_read_dos(core, AV_SCRATCH_1) + 1) * 16;
		dev_err(core->dev, "Unsupported video height: %u\n", size);
		goto abort;
	case 0: /* Unused but not worth printing for */
	case 9:
		break;
	default:
		dev_info(core->dev, "Unexpected H264 ISR: %08X\n", cmd);
		break;
	}

	if (cmd && cmd != CMD_SRC_CHANGE)
		amvdec_write_dos(core, AV_SCRATCH_0, 0);

	/* Decoder has some SEI data for us ; ignore */
	if (amvdec_read_dos(core, AV_SCRATCH_J) & SEI_DATA_READY)
		amvdec_write_dos(core, AV_SCRATCH_J, 0);

	return IRQ_HANDLED;
abort:
	amvdec_abort(sess);
	return IRQ_HANDLED;
}

static irqreturn_t codec_h264_isr(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;

	amvdec_write_dos(core, ASSIST_MBOX1_CLR_REG, 1);

	return IRQ_WAKE_THREAD;
}

struct amvdec_codec_ops codec_h264_ops = {
	.start = codec_h264_start,
	.stop = codec_h264_stop,
	.load_extended_firmware = codec_h264_load_extended_firmware,
	.isr = codec_h264_isr,
	.threaded_isr = codec_h264_threaded_isr,
	.can_recycle = codec_h264_can_recycle,
	.recycle = codec_h264_recycle,
	.eos_sequence = codec_h264_eos_sequence,
	.resume = codec_h264_resume,
	.input_committed = codec_h264_input_committed,
	.input_read_pointer = codec_h264_input_read_pointer,
	.drain_step = codec_h264_drain_step,
};
