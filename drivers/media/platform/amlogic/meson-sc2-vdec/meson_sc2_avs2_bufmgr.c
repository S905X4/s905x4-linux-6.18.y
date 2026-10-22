/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * AVS2 DPB/RPS management, ported from Amlogic avs2_bufmgr.c (2017).
 * Copyright (C) 2017 Amlogic, Inc. All rights reserved.
 *
 * The firmware supplies expanded RPS and picture syntax in param_u. DMA,
 * firmware loading and presentation belong to the native hardware backend.
 */
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/string.h>

#include "meson_sc2_avs2_global.h"

static int avs2_error(struct avs2_decoder *dec, int error)
{
	dec->error = error;
	dec->bufmgr_error_flag = 1;
	return error;
}

void init_frame_t(struct avs2_frame_s *frame)
{
	memset(frame, 0, sizeof(*frame));
	frame->imgcoi_ref = -257;
	frame->imgtr_fwRefDistance = -256;
	frame->is_output = -1;
	frame->referred_by_others = -1;
	frame->temporal_id = -1;
	frame->mv_buf_index = -1;
}

s32 avs2_init_global_buffers(struct avs2_decoder *dec)
{
	int i;

	if (dec->ref_maxbuffer < 2 || dec->ref_maxbuffer > REF_MAXBUFFER)
		return avs2_error(dec, -EINVAL);
	dec->img.buf_cycle = (dec->input.buf_cycle + 1) * 2;
	for (i = 0; i < REF_MAXBUFFER; i++) {
		dec->fref[i] = &dec->frm_pool[i];
		init_frame_t(dec->fref[i]);
		dec->fref[i]->index = i;
	}
	dec->m_bg = &dec->frm_pool[REF_MAXBUFFER];
	init_frame_t(dec->m_bg);
	dec->m_bg->index = REF_MAXBUFFER;
	dec->f_bg = NULL;
	return 0;
}

void init_avs2_decoder(struct avs2_decoder *dec)
{
	int i;

	memset(dec, 0, sizeof(*dec));
	dec->ref_maxbuffer = REF_MAXBUFFER;
	dec->to_prepare_disp_count = 1;
	dec->img.type = I_IMG;
	dec->img.new_sequence_flag = 1;
	dec->hd.last_output = -1;
	dec->hd.end_SeqTr = -1;
	for (i = 0; i < NUM_ALF_COMPONENT; i++) {
		dec->m_alfPictureParam[i].num_coeff = ALF_MAX_NUM_COEF;
		dec->m_alfPictureParam[i].filters_per_group = 3;
		dec->m_alfPictureParam[i].componentID = i;
	}
}

static int validate_picture(struct avs2_decoder *dec, s32 code)
{
	const union param_u *p = &dec->param;
	int i;

	/* Bound every firmware-controlled index before parsing or shifting. */
	if (!p->p.horizontal_size || !p->p.vertical_size ||
	    p->p.horizontal_size > 8192 || p->p.vertical_size > 8192 ||
	    p->p.chroma_format != 1 || p->p.lcu_size < 4 ||
	    p->p.lcu_size > 6 || p->p.g_uiMaxSizeInBit < 4 ||
	    p->p.g_uiMaxSizeInBit > 6 || p->p.coding_order > 255 ||
	    p->p.num_of_ref_cur > MAXREF || p->p.num_to_remove_cur > MAXREF ||
	    p->p.num_of_RPS > MAXGOP ||
	    (p->p.predict && p->p.RPS_idx >= MAXGOP) ||
	    p->p.cur_layer > 7 || p->p.picture_reorder_delay > 255 ||
	    p->p.displaydelay > 255 || p->p.sample_precision < 1 ||
	    p->p.sample_precision > 2)
		return avs2_error(dec, -EINVAL);
	if (p->p.profile_id != BASELINE_PROFILE &&
	    p->p.profile_id != BASELINE_PICTURE_PROFILE &&
	    p->p.profile_id != BASELINE10_PROFILE &&
	    p->p.profile_id != SCENE_PROFILE && p->p.profile_id != SCENE10_PROFILE)
		return avs2_error(dec, -EOPNOTSUPP);
	if ((p->p.profile_id == BASELINE10_PROFILE ||
	     p->p.profile_id == SCENE10_PROFILE) &&
	    (p->p.encoding_precision < 1 || p->p.encoding_precision > 2))
		return avs2_error(dec, -EINVAL);
	if (code == PB_PICTURE_START_CODE &&
	    (p->p.picture_coding_type < 1 || p->p.picture_coding_type > 3))
		return avs2_error(dec, -EINVAL);
	for (i = 0; i < p->p.num_of_ref_cur; i++)
		if (!p->p.ref_pic_cur[i] || p->p.ref_pic_cur[i] > 255)
			return avs2_error(dec, -EINVAL);
	for (i = 0; i < p->p.num_to_remove_cur; i++)
		if (!p->p.remove_pic_cur[i] || p->p.remove_pic_cur[i] > 255)
			return avs2_error(dec, -EINVAL);
	return 0;
}

static int read_alf_coeff(struct avs2_decoder *dec, struct ALFParam_s *alf)
{
	const union param_u *p = &dec->param;
	int group, pos, symbol, previous = 0;

	memset(alf->filterPattern, 0, sizeof(alf->filterPattern));
	if (alf->componentID == ALF_Y) {
		if (p->alf.alf_filters_num_m_1 >= NO_VAR_BINS)
			return avs2_error(dec, -EINVAL);
		alf->filters_per_group = p->alf.alf_filters_num_m_1 + 1;
	} else {
		alf->filters_per_group = 1;
	}
	for (group = 0; group < alf->filters_per_group; group++) {
		if (group) {
			symbol = alf->filters_per_group == NO_VAR_BINS ? 1 :
				 p->alf.region_distance[group - 1];
			if (symbol < 1 || symbol >= NO_VAR_BINS - previous)
				return avs2_error(dec, -EINVAL);
			previous += symbol;
			alf->filterPattern[previous] = 1;
		}
		for (pos = 0; pos < ALF_MAX_NUM_COEF; pos++) {
			s16 coeff;

			switch (alf->componentID) {
			case ALF_Y:
				coeff = (s16)p->alf.alf_y_coeffmulti[group][pos];
				break;
			case ALF_Cb:
				coeff = (s16)p->alf.alf_cb_coeffmulti[pos];
				break;
			case ALF_Cr:
				coeff = (s16)p->alf.alf_cr_coeffmulti[pos];
				break;
			default:
				return avs2_error(dec, -EINVAL);
			}
			if ((pos < 8 && (coeff < -64 || coeff > 63)) ||
			    (pos == 8 && (coeff < -1088 || coeff > 1071)))
				return avs2_error(dec, -EINVAL);
			alf->coeffmulti[group][pos] = coeff;
		}
	}
	return 0;
}

static int read_alf(struct avs2_decoder *dec)
{
	const union param_u *p = &dec->param;
	int enabled[NUM_ALF_COMPONENT] = {
		p->alf.picture_alf_enable_Y,
		p->alf.picture_alf_enable_Cb,
		p->alf.picture_alf_enable_Cr,
	};
	int i;

	for (i = 0; i < NUM_ALF_COMPONENT; i++) {
		if (dec->input.alf_enable && enabled[i] > 1)
			return avs2_error(dec, -EINVAL);
		dec->img.pic_alf_on[i] = dec->input.alf_enable && enabled[i];
		dec->m_alfPictureParam[i].alf_flag = dec->img.pic_alf_on[i];
		if (dec->img.pic_alf_on[i] &&
		    read_alf_coeff(dec, &dec->m_alfPictureParam[i]))
			return dec->error;
	}
	return 0;
}

static void read_sequence(struct avs2_decoder *dec)
{
	const union param_u *p = &dec->param;
	struct inp_par *input = &dec->input;
	struct ImageParameters_s *img = &dec->img;
	struct Video_Dec_data_s *hd = &dec->hd;
	u16 flags = p->p.avs2_seq_flags;

	input->profile_id = p->p.profile_id;
	input->level_id = p->p.level_id;
	hd->progressive_sequence = p->p.progressive_sequence;
	hd->is_field_sequence = p->p.is_field_sequence;
	img->is_field_sequence = hd->is_field_sequence;
	hd->horizontal_size = p->p.horizontal_size;
	hd->vertical_size = p->p.vertical_size;
	input->chroma_format = p->p.chroma_format;
	input->output_bit_depth = 8;
	input->sample_bit_depth = 8;
	hd->sample_precision = p->p.sample_precision;
	if (input->profile_id == BASELINE10_PROFILE ||
	    input->profile_id == SCENE10_PROFILE) {
		input->output_bit_depth = 6 + p->p.sample_precision * 2;
		input->sample_bit_depth = 6 + p->p.encoding_precision * 2;
	}
	hd->aspect_ratio_information = p->p.aspect_ratio_information;
	hd->frame_rate_code = p->p.frame_rate_code;
	hd->bit_rate_lower = p->p.bit_rate_lower;
	hd->bit_rate_upper = p->p.bit_rate_upper;
	hd->low_delay = p->p.low_delay;
	hd->temporal_id_exist_flag = p->p.temporal_id_exist_flag;
	input->g_uiMaxSizeInBit = p->p.g_uiMaxSizeInBit;
	hd->background_picture_enable = !(flags & BIT(BACKGROUND_PICTURE_DISABLE_BIT));
	hd->b_dmh_enabled = 1;
	hd->b_mhpskip_enabled = !!(flags & BIT(B_MHPSKIP_ENABLED_BIT));
	hd->dhp_enabled = !!(flags & BIT(DHP_ENABLED_BIT));
	hd->wsm_enabled = !!(flags & BIT(WSM_ENABLED_BIT));
	img->inter_amp_enable = !!(flags & BIT(INTER_AMP_ENABLE_BIT));
	input->useNSQT = !!(flags & BIT(USENSQT_BIT));
	input->useSDIP = !!(flags & BIT(USESDIP_BIT));
	hd->b_secT_enabled = !!(flags & BIT(B_SECT_ENABLED_BIT));
	input->sao_enable = !!(flags & BIT(SAO_ENABLE_BIT));
	input->alf_enable = !!(flags & BIT(ALF_ENABLE_BIT));
	hd->b_pmvr_enabled = !!(flags & BIT(B_PMVR_ENABLED_BIT));
	hd->gop_size = p->p.num_of_RPS;
	hd->picture_reorder_delay = hd->low_delay ? 0 : p->p.picture_reorder_delay;
	input->crossSliceLoopFilter = !!(flags & BIT(CROSSSLICELOOPFILTER_BIT));
	input->alpha_c_offset = (s16)p->p.alpha_c_offset;
	input->beta_offset = (s16)p->p.beta_offset;
	hd->chroma_quant_param_delta_u = (s16)p->p.chroma_quant_param_delta_cb;
	hd->chroma_quant_param_delta_v = (s16)p->p.chroma_quant_param_delta_cr;
	hd->loop_filter_disable = p->p.loop_filter_disable;
	img->width = hd->horizontal_size;
	img->height = hd->vertical_size;
	img->width_cr = img->width >> 1;
	img->height_cr = img->height >> 1;
	img->PicWidthInMbs = img->width / MIN_CU_SIZE;
	img->PicHeightInMbs = img->height / MIN_CU_SIZE;
	img->PicSizeInMbs = img->PicWidthInMbs * img->PicHeightInMbs;
	img->max_mb_nr = img->PicSizeInMbs;
	img->buf_cycle = input->buf_cycle + 1;
	dec->lcu_size = 1 << p->p.lcu_size;
	dec->hc.seq_header++;
}

static void read_picture(struct avs2_decoder *dec, s32 code)
{
	const union param_u *p = &dec->param;
	struct ImageParameters_s *img = &dec->img;
	struct Video_Dec_data_s *hd = &dec->hd;
	struct reference_management *rps = &hd->curr_RPS;
	int i;

	img->typeb = 0;
	hd->background_picture_flag = 0;
	hd->background_picture_output_flag = 0;
	hd->background_reference_enable = 0;
	hd->background_pred_flag = 0;
	if (code == I_PICTURE_START_CODE) {
		img->type = I_IMG;
		hd->time_code_flag = p->p.time_code_flag;
		if (hd->time_code_flag)
			hd->time_code = p->p.time_code;
		if (hd->background_picture_enable) {
			hd->background_picture_flag = p->p.background_picture_flag;
			if (hd->background_picture_flag) {
				img->typeb = BACKGROUND_IMG;
				hd->background_picture_output_flag =
					p->p.background_picture_output_flag;
			}
		}
	} else {
		hd->picture_coding_type = p->p.picture_coding_type;
		img->type = hd->picture_coding_type == 1 ? P_IMG :
			    hd->picture_coding_type == 3 ? F_IMG : B_IMG;
		if (hd->background_picture_enable && img->type != B_IMG) {
			if (img->type == P_IMG)
				hd->background_pred_flag = p->p.background_pred_flag;
			hd->background_reference_enable = hd->background_pred_flag ?
				1 : p->p.background_reference_enable;
		}
		if (hd->background_pred_flag)
			img->typeb = BP_IMG;
		img->random_access_decodable_flag = p->p.random_access_decodable_flag;
	}
	img->coding_order = p->p.coding_order;
	hd->cur_layer = hd->temporal_id_exist_flag ? p->p.cur_layer : 0;
	hd->displaydelay = hd->low_delay ? 0 : p->p.displaydelay;
	/* Non-output GB pictures do not signal an output delay. */
	if (img->typeb == BACKGROUND_IMG && !hd->background_picture_output_flag)
		hd->displaydelay = 0;
	/* Firmware expands both explicit and sequence-predicted RPS into RPM. */
	memset(rps, 0, sizeof(*rps));
	rps->predict = p->p.predict;
	rps->referred_by_others = p->p.referred_by_others_cur;
	rps->num_of_ref = p->p.num_of_ref_cur;
	rps->num_to_remove = p->p.num_to_remove_cur;
	for (i = 0; i < rps->num_of_ref; i++)
		rps->ref_pic[i] = p->p.ref_pic_cur[i];
	for (i = 0; i < rps->num_to_remove; i++)
		rps->remove_pic[i] = p->p.remove_pic_cur[i];
	hd->progressive_frame = p->p.progressive_frame;
	img->progressive_frame = hd->progressive_frame;
	img->picture_structure = hd->progressive_frame ? 1 : p->p.picture_structure;
	hd->top_field_first = p->p.top_field_first;
	hd->repeat_first_field = p->p.repeat_first_field;
	hd->is_top_field = hd->is_field_sequence ? p->p.is_top_field : 0;
	img->is_top_field = hd->is_top_field;
	img->qp = hd->picture_qp;
}

static void calc_picture_distance(struct avs2_decoder *dec)
{
	struct ImageParameters_s *img = &dec->img;
	struct Video_Dec_data_s *hd = &dec->hd;
	int i, j;

	if (img->coding_order < img->PrevPicDistanceLsb) {
		dec->hc.total_frames++;
		for (i = 0; i < dec->ref_maxbuffer; i++) {
			struct avs2_frame_s *frame = dec->fref[i];

			if (frame->imgtr_fwRefDistance >= 0) {
				frame->imgtr_fwRefDistance -= 256;
				frame->imgcoi_ref -= 256;
			}
			for (j = 0; j < MAXREF; j++)
				frame->ref_poc[j] -= 256;
		}
		for (i = 0; i < dec->outprint.buffer_num; i++) {
			dec->outprint.stdoutdata[i].framenum -= 256;
			dec->outprint.stdoutdata[i].tr -= 256;
		}
		hd->last_output -= 256;
		hd->curr_IDRtr -= 256;
		hd->curr_IDRcoi -= 256;
		hd->next_IDRtr -= 256;
		hd->next_IDRcoi -= 256;
	}
	img->tr = img->coding_order;
	if (!hd->low_delay)
		img->tr += hd->displaydelay - hd->picture_reorder_delay;
	img->pic_distance = img->tr;
	dec->hc.picture_distance = img->pic_distance;
}

static void reset_unused_buffers(struct avs2_decoder *dec)
{
	int i;

	for (i = 0; i < REF_MAXBUFFER; i++) {
		struct avs2_frame_s *frame = dec->fref[i];

		/* The new sequence no longer references an old background, even
		 * when presentation still pins its allocation.
		 */
		frame->bg_flag = 0;
		if (frame->vf_ref || frame->to_prepare_disp)
			continue;
		frame->imgcoi_ref = -257;
		frame->imgtr_fwRefDistance = -256;
		frame->is_output = -1;
		frame->referred_by_others = -1;
		memset(frame->ref_poc, 0, sizeof(frame->ref_poc));
	}
	dec->f_bg = NULL;
	dec->m_bg->imgcoi_ref = -257;
	dec->m_bg->imgtr_fwRefDistance = -256;
	dec->m_bg->is_output = -1;
	dec->m_bg->referred_by_others = -1;
	dec->m_bg->bg_flag = 0;
	memset(dec->m_bg->ref_poc, 0, sizeof(dec->m_bg->ref_poc));
}

void avs2_prepare_header(struct avs2_decoder *dec, s32 start_code)
{
	struct ImageParameters_s *img = &dec->img;
	struct Video_Dec_data_s *hd = &dec->hd;

	dec->error = 0;
	dec->bufmgr_error_flag = 0;
	if (dec->ref_maxbuffer < 2 || dec->ref_maxbuffer > REF_MAXBUFFER ||
	    !dec->fref[0] || !dec->m_bg || dec->outprint.buffer_num < 0 ||
	    dec->outprint.buffer_num > REF_MAXBUFFER) {
		avs2_error(dec, -EINVAL);
		return;
	}
	switch (start_code) {
	case SEQUENCE_HEADER_CODE:
		img->new_sequence_flag = 1;
		if (hd->vec_flag || img->sequence_end_flag) {
			if (avs2_flush_dpb(dec))
				return;
			hd->vec_flag = 0;
			reset_unused_buffers(dec);
			img->number = 0;
			img->PrevPicDistanceLsb = 0;
			dec->init_hw_flag = 0;
		}
		img->seq_header_indicate = 1;
		break;
	case I_PICTURE_START_CODE:
	case PB_PICTURE_START_CODE:
		if (validate_picture(dec, start_code))
			return;
		read_sequence(dec);
		read_picture(dec, start_code);
		if (read_alf(dec))
			return;
		calc_picture_distance(dec);
		if (!img->seq_header_indicate) {
			img->B_discard_flag = 1;
			avs2_error(dec, -EILSEQ);
			return;
		}
		if (start_code == PB_PICTURE_START_CODE && img->type != B_IMG)
			img->B_discard_flag = 0;
		if (img->type == B_IMG && img->B_discard_flag &&
		    !img->random_access_decodable_flag)
			avs2_error(dec, -EILSEQ);
		break;
	case SEQUENCE_END_CODE:
		img->new_sequence_flag = 1;
		img->sequence_end_flag = 1;
		break;
	case VIDEO_EDIT_CODE:
		hd->vec_flag = 1;
		break;
	}
}

static int prepare_reference_info(struct avs2_decoder *dec)
{
	struct ImageParameters_s *img = &dec->img;
	struct Video_Dec_data_s *hd = &dec->hd;
	struct avs2_frame_s *frame;
	int i, j;

	if (img->tr > hd->next_IDRtr && hd->curr_IDRtr != hd->next_IDRtr) {
		hd->curr_IDRtr = hd->next_IDRtr;
		hd->curr_IDRcoi = hd->next_IDRcoi;
	}
	img->num_of_references = hd->curr_RPS.num_of_ref;
	if (img->num_of_references > MAXREF ||
	    img->num_of_references >= dec->ref_maxbuffer)
		return avs2_error(dec, -EINVAL);
	for (i = 0; i < img->num_of_references; i++) {
		for (j = i; j < dec->ref_maxbuffer; j++)
			if (dec->fref[j]->imgcoi_ref ==
			    (s32)img->coding_order - hd->curr_RPS.ref_pic[i])
				break;
		if (j == dec->ref_maxbuffer)
			return avs2_error(dec, -ENOENT);
		frame = dec->fref[i];
		dec->fref[i] = dec->fref[j];
		dec->fref[j] = frame;
	}
	if (img->type == B_IMG &&
	    (img->num_of_references != 2 ||
	     dec->fref[0]->imgtr_fwRefDistance <= img->tr ||
	     dec->fref[1]->imgtr_fwRefDistance >= img->tr))
		return avs2_error(dec, -EILSEQ);
	if (hd->background_reference_enable && !dec->f_bg)
		return avs2_error(dec, -ENOENT);
	/* A reference, background, queued or displayed surface is never reused. */
	for (i = img->num_of_references; i < dec->ref_maxbuffer; i++) {
		frame = dec->fref[i];
		if ((frame->imgcoi_ref < -256 ||
		     abs(frame->imgtr_fwRefDistance - img->tr) >= 128) &&
		    frame->is_output == -1 && !frame->bg_flag &&
		    !frame->vf_ref && !frame->to_prepare_disp)
			break;
	}
	if (i == dec->ref_maxbuffer)
		return avs2_error(dec, -ENOBUFS);
	frame = dec->fref[i];
	dec->hc.f_rec = frame;
	dec->hc.cur_pic = frame;
	frame->imgtr_fwRefDistance = img->tr;
	frame->imgcoi_ref = img->coding_order;
	frame->temporal_id = hd->cur_layer;
	frame->is_output = 1;
	frame->error_mark = 0;
	frame->decoded_lcu = 0;
	frame->slice_type = img->type;
	frame->referred_by_others = hd->curr_RPS.referred_by_others;
	memset(frame->ref_poc, 0, sizeof(frame->ref_poc));
	for (j = 0; j < img->num_of_references; j++)
		frame->ref_poc[j] = dec->fref[j]->imgtr_fwRefDistance;
	if (img->type == B_IMG) {
		frame->ref_poc[0] = dec->fref[1]->imgtr_fwRefDistance;
		frame->ref_poc[1] = dec->fref[0]->imgtr_fwRefDistance;
	} else if (img->type == I_IMG) {
		for (j = 0; j < 4; j++)
			frame->ref_poc[j] = img->tr;
	}
	if (img->type != I_IMG) {
		img->imgtr_next_P = img->type == B_IMG ?
			dec->fref[0]->imgtr_fwRefDistance : img->tr;
		if (img->type == B_IMG) {
			hd->trtmp = dec->fref[0]->imgtr_fwRefDistance;
			dec->fref[0]->imgtr_fwRefDistance =
				dec->fref[1]->imgtr_fwRefDistance;
		}
	}
	return 0;
}

s32 avs2_process_header(struct avs2_decoder *dec)
{
	struct ImageParameters_s *img = &dec->img;
	struct Video_Dec_data_s *hd = &dec->hd;

	if (dec->error)
		return dec->error;
	if (!dec->lcu_size || !hd->horizontal_size || !hd->vertical_size ||
	    dec->ref_maxbuffer < 2 || dec->ref_maxbuffer > REF_MAXBUFFER ||
	    !dec->m_bg || !dec->fref[0])
		return avs2_error(dec, -EINVAL);
	img->auto_crop_right = ALIGN(hd->horizontal_size, MIN_CU_SIZE) - hd->horizontal_size;
	img->auto_crop_bottom = ALIGN(hd->vertical_size, MIN_CU_SIZE) - hd->vertical_size;
	img->width = hd->horizontal_size + img->auto_crop_right;
	img->height = hd->vertical_size + img->auto_crop_bottom;
	img->width_cr = img->width >> 1;
	img->height_cr = img->height >> 1;
	img->PicWidthInMbs = img->width / MIN_CU_SIZE;
	img->PicHeightInMbs = img->height / MIN_CU_SIZE;
	img->PicSizeInMbs = img->PicWidthInMbs * img->PicHeightInMbs;
	img->max_mb_nr = img->PicSizeInMbs;
	if (img->new_sequence_flag && img->sequence_end_flag) {
		hd->end_SeqTr = img->tr;
		img->sequence_end_flag = 0;
	}
	if (img->new_sequence_flag) {
		hd->next_IDRtr = img->tr;
		hd->next_IDRcoi = img->coding_order;
		img->new_sequence_flag = 0;
	}
	img->current_mb_nr = 0;
	if (img->typeb == BACKGROUND_IMG && !hd->background_picture_output_flag) {
		img->num_of_references = 0;
		dec->hc.cur_pic = dec->m_bg;
		dec->m_bg->imgtr_fwRefDistance = img->tr;
		dec->m_bg->imgcoi_ref = img->coding_order;
		dec->m_bg->slice_type = img->type;
		dec->m_bg->error_mark = 0;
		dec->m_bg->decoded_lcu = 0;
	} else if (prepare_reference_info(dec)) {
		return dec->error;
	}
	if (img->typeb == BACKGROUND_IMG && img->is_field_sequence)
		dec->bk_img_is_top_field = img->is_top_field;
	img->types = img->type;
	if (img->type != B_IMG) {
		hd->pre_img_type = img->type;
		hd->pre_img_types = img->types;
	}
	dec->lcu_size_log2 = dec->param.p.lcu_size;
	dec->lcu_x_num = DIV_ROUND_UP(img->width, dec->lcu_size);
	dec->lcu_y_num = DIV_ROUND_UP(img->height, dec->lcu_size);
	dec->lcu_total = dec->lcu_x_num * dec->lcu_y_num;
	dec->hc.cur_pic->lcu_size_log2 = dec->lcu_size_log2;
	dec->hc.cur_pic->lcu_total = dec->lcu_total;
	dec->hc.cur_pic->pic_w = hd->horizontal_size;
	dec->hc.cur_pic->pic_h = hd->vertical_size;
	dec->hc.cur_pic->bit_depth = dec->input.sample_bit_depth;
	return SOP;
}

/* Preserve ordering without overflowing the vendor's signed display serial. */
static void compact_display_serials(struct avs2_decoder *dec)
{
	int i, previous = 0, rank = 1;

	for (;;) {
		struct avs2_frame_s *next = NULL;

		for (i = 0; i < REF_MAXBUFFER; i++) {
			struct avs2_frame_s *frame = dec->fref[i];

			if (frame->to_prepare_disp > previous &&
			    (!next || frame->to_prepare_disp < next->to_prepare_disp))
				next = frame;
		}
		if (!next)
			break;
		previous = next->to_prepare_disp;
		next->to_prepare_disp = rank++;
	}
	dec->to_prepare_disp_count = rank;
}

int write_frame(struct avs2_decoder *dec, s32 poc)
{
	struct avs2_frame_s *frame = NULL;
	int i, ret;

	for (i = 0; i < dec->ref_maxbuffer; i++)
		if (dec->fref[i]->imgtr_fwRefDistance == poc &&
		    dec->fref[i]->is_output == 1) {
			frame = dec->fref[i];
			break;
		}
	if (!frame)
		return avs2_error(dec, -ENOENT);
	frame->imgtr_fwRefDistance_bak = poc;
	if (dec->write_frame) {
		ret = dec->write_frame(dec, frame);
		if (ret)
			return avs2_error(dec, ret);
	}
	if (dec->to_prepare_disp_count == INT_MAX)
		compact_display_serials(dec);
	frame->is_output = -1;
	frame->to_prepare_disp = dec->to_prepare_disp_count++;
	if (!frame->referred_by_others || frame->imgcoi_ref == -257) {
		frame->imgtr_fwRefDistance = -256;
		frame->imgcoi_ref = -257;
		frame->temporal_id = -1;
	}
	return 0;
}

int report_frame(struct avs2_decoder *dec, struct outdata_s *data, s32 pos)
{
	struct STDOUT_DATA_s *entry;
	s64 absolute_poc;
	int ret;

	if (pos < 0 || pos >= data->buffer_num)
		return avs2_error(dec, -EINVAL);
	entry = &data->stdoutdata[pos];
	if (dec->report_frame) {
		ret = dec->report_frame(dec, data, pos);
		if (ret)
			return avs2_error(dec, ret);
	}
	absolute_poc = entry->tr + (s64)dec->hc.total_frames * 256;
	if (absolute_poc == dec->hd.end_SeqTr)
		dec->img.sequence_end_flag = 0;
	if (absolute_poc == dec->hd.next_IDRtr)
		dec->hd.vec_flag = 0;
	dec->hd.FrameNum++;
	return 0;
}

static int output_frame(struct avs2_decoder *dec, int pos)
{
	struct outdata_s *data = &dec->outprint;
	struct STDOUT_DATA_s *entry = &data->stdoutdata[pos];
	int ret;

	ret = report_frame(dec, data, pos);
	if (ret)
		return ret;
	if (entry->typeb != BACKGROUND_IMG || entry->background_picture_output_flag) {
		ret = write_frame(dec, entry->tr);
		if (ret)
			return ret;
	}
	dec->hd.last_output = entry->tr;
	memmove(entry, entry + 1, (data->buffer_num - pos - 1) * sizeof(*entry));
	data->buffer_num--;
	return 0;
}

int avs2_drain_output(struct avs2_decoder *dec)
{
	int i, pos;

	if (dec->outprint.buffer_num < 0 || dec->outprint.buffer_num > REF_MAXBUFFER ||
	    dec->ref_maxbuffer < 2 || dec->ref_maxbuffer > REF_MAXBUFFER ||
	    !dec->fref[0] || !dec->m_bg)
		return avs2_error(dec, -EINVAL);
	while (dec->outprint.buffer_num) {
		pos = 0;
		for (i = 1; i < dec->outprint.buffer_num; i++)
			if (dec->outprint.stdoutdata[i].tr < dec->outprint.stdoutdata[pos].tr)
				pos = i;
		if (output_frame(dec, pos))
			return dec->error;
	}
	return 0;
}

int avs2_flush_dpb(struct avs2_decoder *dec)
{
	int i, ret;

	ret = avs2_drain_output(dec);
	if (ret)
		return ret;
	for (i = 0; i < REF_MAXBUFFER; i++) {
		dec->fref[i]->imgtr_fwRefDistance = -256;
		dec->fref[i]->imgcoi_ref = -257;
		dec->fref[i]->temporal_id = -1;
		dec->fref[i]->referred_by_others = 0;
		dec->fref[i]->bg_flag = 0;
	}
	/* Flush ends codec ownership, not outstanding presentation ownership. */
	dec->m_bg->imgtr_fwRefDistance = -256;
	dec->m_bg->imgcoi_ref = -257;
	dec->m_bg->temporal_id = -1;
	dec->m_bg->referred_by_others = 0;
	dec->m_bg->bg_flag = 0;
	dec->f_bg = NULL;
	dec->hc.cur_pic = NULL;
	dec->hc.f_rec = NULL;
	return 0;
}

static void reference_list_info(struct avs2_decoder *dec)
{
	char *str = dec->hc.str_list_reference;
	struct avs2_frame_s *frame = dec->hc.cur_pic;
	int i, len = 0;

	str[0] = '\0';
	if (!dec->img.num_of_references)
		return;
	len += scnprintf(str + len, sizeof(dec->hc.str_list_reference) - len, "[");
	for (i = 0; i < dec->img.num_of_references; i++) {
		int index = dec->img.type == B_IMG ? dec->img.num_of_references - 1 - i : i;

		len += scnprintf(str + len, sizeof(dec->hc.str_list_reference) - len,
				 "%4d ", frame->ref_poc[index]);
	}
	scnprintf(str + len, sizeof(dec->hc.str_list_reference) - len, "]");
}

static int frame_postprocessing(struct avs2_decoder *dec)
{
	struct outdata_s *data = &dec->outprint;
	struct ImageParameters_s *img = &dec->img;
	struct Video_Dec_data_s *hd = &dec->hd;
	struct STDOUT_DATA_s *entry;
	bool output_current = false;
	int i, j, searches, minimum, pos, last = -1;

	if (data->buffer_num < 0 || data->buffer_num >= REF_MAXBUFFER)
		return avs2_error(dec, -ENOSPC);
	img->PrevPicDistanceLsb = img->coding_order % 256;
	entry = &data->stdoutdata[data->buffer_num++];
	memset(entry, 0, sizeof(*entry));
	entry->type = img->type;
	entry->typeb = img->typeb;
	entry->framenum = img->tr;
	entry->tr = img->tr;
	entry->tmp_time = hd->tmp_time;
	entry->picture_structure = img->picture_structure;
	entry->background_picture_output_flag = hd->background_picture_output_flag;
	entry->picture_reorder_delay = hd->picture_reorder_delay;
	strscpy(entry->str_reference_list, dec->hc.str_list_reference,
		sizeof(entry->str_reference_list));
	searches = data->buffer_num;
	for (j = 0; j < searches; j++) {
		pos = -1;
		minimum = INT_MAX;
		for (i = 0; i < data->buffer_num; i++) {
			entry = &data->stdoutdata[i];
			if (entry->tr < minimum &&
			    entry->tr + entry->picture_reorder_delay <= (s32)img->coding_order) {
				pos = i;
				minimum = entry->tr;
			}
		}
		/* The zero-delay current picture bypasses older delayed pictures. */
		if (!hd->displaydelay && !output_current && img->tr <= minimum) {
			for (i = 0; i < data->buffer_num; i++)
				if (data->stdoutdata[i].tr == img->tr &&
				    data->stdoutdata[i].typeb == img->typeb) {
					pos = i;
					output_current = true;
					break;
				}
		}
		if (pos < 0)
			break;
		if (output_frame(dec, pos))
			return dec->error;
		last = pos;
	}
	return last;
}

int avs2_post_process(struct avs2_decoder *dec)
{
	struct ImageParameters_s *img = &dec->img;
	struct Video_Dec_data_s *hd = &dec->hd;
	int i, j, ret;

	if (dec->error)
		return dec->error;
	if (!dec->hc.cur_pic || img->num_of_references < 0 ||
	    img->num_of_references > MAXREF || hd->curr_RPS.num_to_remove < 0 ||
	    hd->curr_RPS.num_to_remove > MAXREF)
		return avs2_error(dec, -EINVAL);
	if (img->typeb == BACKGROUND_IMG && hd->background_picture_enable) {
		for (i = 0; i < dec->ref_maxbuffer; i++)
			dec->fref[i]->bg_flag = 0;
		dec->m_bg->bg_flag = 0;
		dec->f_bg = dec->hc.cur_pic;
		dec->f_bg->bg_flag = 1;
	}
	if (img->typeb == BACKGROUND_IMG && !hd->background_picture_output_flag)
		hd->background_number++;
	if (img->type == B_IMG)
		dec->fref[0]->imgtr_fwRefDistance = hd->trtmp;
	reference_list_info(dec);
	ret = frame_postprocessing(dec);
	if (dec->error)
		return dec->error;
	/* RPS removal happens after output; displayed/queued surfaces stay pinned. */
	for (i = 0; i < hd->curr_RPS.num_to_remove; i++) {
		for (j = 0; j < dec->ref_maxbuffer; j++) {
			struct avs2_frame_s *frame = dec->fref[j];

			if (frame->imgcoi_ref < -256 || frame->imgcoi_ref !=
			    (s32)img->coding_order - hd->curr_RPS.remove_pic[i])
				continue;
			frame->referred_by_others = 0;
			if (frame->is_output == -1) {
				frame->imgtr_fwRefDistance = -256;
				frame->imgcoi_ref = -257;
				frame->temporal_id = -1;
			}
			break;
		}
	}
	if (img->type == I_IMG || img->type == P_IMG || img->type == F_IMG)
		img->number++;
	else
		dec->hc.Bframe_ctr++;
	return ret;
}
