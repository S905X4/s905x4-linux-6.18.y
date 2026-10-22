/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * AVS2 picture management, derived from Amlogic's AVS2 decoder.
 * Copyright (C) 2017 Amlogic, Inc. All rights reserved.
 */
#ifndef MESON_SC2_AVS2_GLOBAL_H
#define MESON_SC2_AVS2_GLOBAL_H

#include <linux/types.h>

#define BASELINE_PICTURE_PROFILE 18
#define BASELINE_PROFILE 32
#define BASELINE10_PROFILE 34
#define SCENE_PROFILE 48
#define SCENE10_PROFILE 50
#define MAXREF 7
#define MAXGOP 32
#define REF_MAXBUFFER 23
#define REF_BUFFER 16
#define AVS2_MAX_BUFFER_NUM (REF_MAXBUFFER + 1)
#define MIN_CU_SIZE 8
#define MAX_CU_SIZE 64
#define ALF_MAX_NUM_COEF 9
#define NO_VAR_BINS 16
#define I_IMG 0
#define INTRA_IMG I_IMG
#define P_IMG 1
#define INTER_IMG P_IMG
#define B_IMG 2
#define BACKGROUND_IMG 3
#define F_IMG 4
#define BP_IMG 5
#define SOP 2
#define I_PICTURE_START_CODE 0xb3
#define PB_PICTURE_START_CODE 0xb6
#define SEQUENCE_HEADER_CODE 0xb0
#define SEQUENCE_END_CODE 0xb1
#define VIDEO_EDIT_CODE 0xb7
#define USER_DATA_START_CODE 0xb2
#define EXTENSION_START_CODE 0xb5
#define SLICE_START_CODE_MIN 0x00
#define SLICE_START_CODE_MAX 0x8f

#define RPM_BEGIN 0x100
#define ALF_BEGIN 0x180
#define RPM_END 0x280
#define BACKGROUND_PICTURE_DISABLE_BIT 11
#define B_MHPSKIP_ENABLED_BIT 10
#define DHP_ENABLED_BIT 9
#define WSM_ENABLED_BIT 8
#define INTER_AMP_ENABLE_BIT 7
#define USENSQT_BIT 6
#define USESDIP_BIT 5
#define B_SECT_ENABLED_BIT 4
#define SAO_ENABLE_BIT 3
#define ALF_ENABLE_BIT 2
#define B_PMVR_ENABLED_BIT 1
#define CROSSSLICELOOPFILTER_BIT 0

/* Firmware 27's little-endian 16-bit RPM words, including the ALF overlay. */
union param_u {
	struct { u16 data[RPM_END - RPM_BEGIN]; } l;
	struct {
		u16 profile_id, level_id, progressive_sequence, is_field_sequence;
		u16 horizontal_size, vertical_size, chroma_format;
		u16 sample_precision, encoding_precision;
		u16 aspect_ratio_information, frame_rate_code;
		u16 bit_rate_lower, bit_rate_upper, low_delay;
		u16 temporal_id_exist_flag, g_uiMaxSizeInBit, avs2_seq_flags;
		u16 num_of_RPS, picture_reorder_delay;
		u16 time_code_flag, time_code;
		u16 background_picture_flag, background_picture_output_flag;
		u16 coding_order, cur_layer, displaydelay, predict, RPS_idx;
		u16 referred_by_others_cur, num_of_ref_cur, ref_pic_cur[8];
		u16 num_to_remove_cur, remove_pic_cur[8];
		u16 progressive_frame, picture_structure, top_field_first;
		u16 repeat_first_field, is_top_field;
		u16 picture_coding_type, background_pred_flag;
		u16 background_reference_enable, random_access_decodable_flag;
		u16 lcu_size, alpha_c_offset, beta_offset;
		u16 chroma_quant_param_delta_cb, chroma_quant_param_delta_cr;
		u16 loop_filter_disable, video_signal_type, color_description;
		u16 display_primaries_x[3], display_primaries_y[3];
		u16 white_point_x, white_point_y;
		u16 max_display_mastering_luminance;
		u16 min_display_mastering_luminance;
		u16 max_content_light_level, max_picture_average_light_level;
	} p;
	struct {
		u16 padding[ALF_BEGIN - RPM_BEGIN];
		u16 picture_alf_enable_Y, picture_alf_enable_Cb;
		u16 picture_alf_enable_Cr, alf_filters_num_m_1;
		u16 region_distance[16];
		u16 alf_cb_coeffmulti[9], alf_cr_coeffmulti[9];
		u16 alf_y_coeffmulti[16][9];
	} alf;
};

enum ALFComponentID { ALF_Y, ALF_Cb, ALF_Cr, NUM_ALF_COMPONENT };

struct ALFParam_s {
	s32 alf_flag, num_coeff, filters_per_group, componentID;
	s32 filterPattern[NO_VAR_BINS];
	s32 coeffmulti[NO_VAR_BINS][ALF_MAX_NUM_COEF];
};

/* DPB identity and lifetime are independent of the backend's DMA allocations. */
struct avs2_frame_s {
	s32 imgcoi_ref, imgtr_fwRefDistance;
	s32 referred_by_others, is_output, to_prepare_disp, temporal_id;
	s32 ref_poc[MAXREF];
	s32 index, mmu_alloc_flag, lcu_size_log2;
	u32 mc_y_adr, mc_u_v_adr, mc_canvas_y, mc_canvas_u_v;
	u32 mpred_mv_wr_start_addr;
	u8 bg_flag;
	unsigned long header_adr, dw_header_adr;
	int buf_size, lcu_total, comp_body_size;
	u32 dw_y_adr, dw_u_v_adr;
	int y_canvas_index, uv_canvas_index, double_write_mode, bit_depth;
	unsigned long cma_alloc_addr;
	int BUF_index, pic_w, pic_h, stream_offset;
	u32 pts;
	u64 pts64;
	int vf_ref, decode_idx, slice_type;
	s32 imgtr_fwRefDistance_bak, error_mark, decoded_lcu;
	int mv_buf_index;
	int max_qp, avg_qp, min_qp, max_skip, avg_skip, min_skip;
	int max_mv, min_mv, avg_mv;
	u32 hw_decode_time, frame_size;
	char *cuva_data_buf;
	int cuva_data_size;
};

struct ImageParameters_s {
	s32 number, numIPFrames, type, typeb, typeb_before, qp;
	s32 current_mb_nr, current_slice_nr, tr;
	s32 width, width_cr, height, height_cr;
	s32 PicWidthInMbs, PicHeightInMbs, PicSizeInMbs, max_mb_nr;
	s32 num_of_references, auto_crop_right, auto_crop_bottom;
	s32 buf_cycle, picture_structure, imgtr_next_P, imgcoi_next_ref;
	s32 random_access_decodable_flag, seq_header_indicate, B_discard_flag;
	u32 pic_distance, coding_order, PrevPicDistanceLsb;
	s32 CurrPicDistanceMsb, types, new_sequence_flag, sequence_end_flag;
	s32 inter_amp_enable, progressive_frame;
	s32 pic_alf_on[NUM_ALF_COMPONENT];
	s32 is_field_sequence, is_top_field;
};

struct STDOUT_DATA_s {
	s32 type, typeb, framenum, tr, qp;
	s32 tmp_time, picture_structure, curr_frame_bits, emulate_bits;
	u32 DecMD5Value[4];
	s32 background_picture_output_flag, picture_reorder_delay;
	char str_reference_list[128];
};

struct outdata_s {
	struct STDOUT_DATA_s stdoutdata[REF_MAXBUFFER];
	s32 buffer_num;
};

struct Video_Com_data_s {
	s32 Bframe_ctr, tot_time, picture_distance, coding_order;
	struct avs2_frame_s *f_rec;
	s32 seq_header;
	struct avs2_frame_s *cur_pic;
	s32 total_frames;
	char str_list_reference[128];
};

struct reference_management {
	s32 poc, qp_offset, num_of_ref, referred_by_others;
	s32 ref_pic[MAXREF], predict, deltaRPS, num_to_remove;
	s32 remove_pic[MAXREF];
};

struct Video_Dec_data_s {
	s32 background_reference_enable, background_picture_flag;
	s32 background_picture_output_flag, background_picture_enable;
	s32 background_number;
	s32 aspect_ratio_information, frame_rate_code;
	s32 bit_rate_lower, bit_rate_upper;
	s32 video_format, color_description, color_primaries;
	s32 transfer_characteristics, matrix_coefficients;
	s32 progressive_sequence, is_field_sequence, low_delay;
	s32 horizontal_size, vertical_size, sample_precision, video_range;
	s32 b_pmvr_enabled, dhp_enabled, b_dmh_enabled;
	s32 b_mhpskip_enabled, wsm_enabled, b_secT_enabled;
	s32 tmp_time, FrameNum, eos, pre_img_type, pre_img_types;
	s32 dec_ref_num, vec_flag;
	s32 display_primaries_x0, display_primaries_y0;
	s32 display_primaries_x1, display_primaries_y1;
	s32 display_primaries_x2, display_primaries_y2;
	s32 white_point_x, white_point_y;
	s32 max_display_mastering_luminance, min_display_mastering_luminance;
	s32 maximum_content_light_level, maximum_frame_average_light_level;
	s32 top_field_first, repeat_first_field, progressive_frame, is_top_field;
	s32 picture_qp, fixed_picture_qp, time_code_flag, time_code;
	s32 loop_filter_disable, loop_filter_parameter_flag;
	s32 picture_coding_type, background_pred_flag;
	s32 displaydelay, picture_reorder_delay, temporal_id_exist_flag;
	s32 gop_size;
	struct reference_management decod_RPS[MAXGOP], curr_RPS;
	s32 last_output, trtmp, cur_layer;
	/* Picture weights are decoded and applied by firmware; defaults by HW. */
	s32 weight_quant_enable_flag, load_seq_weight_quant_data_flag;
	s32 pic_weight_quant_enable_flag, pic_weight_quant_data_index;
	s32 weighting_quant_param, weighting_quant_model;
	s16 quant_param_undetail[6], quant_param_detail[6];
	s32 WeightQuantEnable, mb_adapt_wq_disable, mb_wq_mode;
	s32 chroma_quant_param_disable, chroma_quant_param_delta_u;
	s32 chroma_quant_param_delta_v;
	s32 b_pre_dec_intra_img, pre_dec_img_type, CurrentSceneModel;
	s32 curr_IDRcoi, curr_IDRtr, next_IDRtr, next_IDRcoi, end_SeqTr;
};

struct inp_par {
	s32 buf_cycle, ref_pic_order, output_dec_pic;
	s32 profile_id, level_id, chroma_format, g_uiMaxSizeInBit;
	s32 alpha_c_offset, beta_offset, useNSQT, useDQP, useSDIP;
	s32 sao_enable, temporal_id_exist_flag, alf_enable;
	s32 crossSliceLoopFilter, sample_bit_depth, output_bit_depth;
	s32 MD5Enable, output_interlace_merged_picture;
};

struct avs2_decoder {
	u8 init_hw_flag;
	struct inp_par input;
	struct ImageParameters_s img;
	struct Video_Com_data_s hc;
	struct Video_Dec_data_s hd;
	union param_u param;
	struct avs2_frame_s frm_pool[AVS2_MAX_BUFFER_NUM];
	struct avs2_frame_s *fref[REF_MAXBUFFER], *m_bg, *f_bg;
	struct outdata_s outprint;
	u32 cm_header_start;
	struct ALFParam_s m_alfPictureParam[NUM_ALF_COMPONENT];
	s8 bk_img_is_top_field;
	s32 lcu_size, lcu_size_log2, lcu_x_num, lcu_y_num, lcu_total;
	s32 ref_maxbuffer, to_prepare_disp_count;
	s8 bufmgr_error_flag;
	/* prepare_header has a vendor void ABI; error is its negative errno. */
	int error;
	void *priv;
	int (*write_frame)(struct avs2_decoder *, struct avs2_frame_s *);
	int (*report_frame)(struct avs2_decoder *, struct outdata_s *, s32);
};

void init_frame_t(struct avs2_frame_s *frame);
void init_avs2_decoder(struct avs2_decoder *dec);
s32 avs2_init_global_buffers(struct avs2_decoder *dec);
void avs2_prepare_header(struct avs2_decoder *dec, s32 start_code);
s32 avs2_process_header(struct avs2_decoder *dec);
/* Vendor result: last reported queue index, or -1 if no output; see error. */
int avs2_post_process(struct avs2_decoder *dec);
/* STOP/START drain: emit pending pictures without retiring DPB references. */
int avs2_drain_output(struct avs2_decoder *dec);
int avs2_flush_dpb(struct avs2_decoder *dec);
int write_frame(struct avs2_decoder *dec, s32 poc);
int report_frame(struct avs2_decoder *dec, struct outdata_s *data, s32 pos);

#endif
