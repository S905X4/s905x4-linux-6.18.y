/* SPDX-License-Identifier: GPL-2.0-only */
/* SC2 AV1 backend, included only by meson_sc2_hw.c: one shared engine lock.
 * Register algorithms from Amlogic vav1.c and the measured SC2 sequence path.
 * The Request serializer owns syntax. RPM is only its firmware acknowledgment.
 */
#define AV1_GMC_PARAM_BUFF_ADDR (0x316d * 4)
#define AV1_DECOMP_AXIURG_CTL (0x34c7 * 4)
#define AV1_MMU_DMA_CTRL (0x363e * 4)
#define A8_PHYSICAL_SLOTS 16
#define AV1_WORK_SIZE 0x230000
#define AV1_RPM 0x221000
#define AV1_LMEM 0x222000
#define AV1_AUX 0x223000

enum av1_workspace_id { IPP, SEG, DAALA, CDF, GMC, DBLK_PARA, DBLK_DATA,
                       CDEF, UPS, GRAIN, MMU_VBH, MPRED_ABOVE };
static const u32 av1_workspace_offsets[] = {
    [IPP] = 0, [SEG] = 0x4000, [DAALA] = 0xcf000, [CDF] = 0xd1000,
    [GMC] = 0x151000, [DBLK_PARA] = 0x152000, [DBLK_DATA] = 0x153000,
    [CDEF] = 0x19c000, [UPS] = 0x1bf000, [GRAIN] = 0x1f5000,
    [MMU_VBH] = 0x215000, [MPRED_ABOVE] = 0x21e000,
};
enum av1_slot_dma_id { A8_MV };
struct av1_retained_picture {
    struct meson_sc2_surface surface;
    struct v4l2_ctrl_av1_frame frame;
    u64 timestamp;
    bool valid, seg_enabled, seg_update_map, intra_only, prev_seg_enabled;
    u8 type, order_hint, ref_order_hint[7];
    u16 seg_y[8], seg_c[8], seg_mi_cols, seg_mi_rows;
    s32 ref_delta[8], mode_delta[2];
    u32 segment_feature[8];
};
struct sc2_av1_state {
    void *work;
    dma_addr_t work_dma;
    struct av1_retained_picture slot[A8_PHYSICAL_SLOTS];
    struct meson_av1_syntax_reference snapshots[8];
    struct v4l2_ctrl_av1_sequence sequence_control;
    int ref_map[8], refs[7], current_slot, primary;
    u32 sequence[128], width, height;
    unsigned int order_bits;
    bool order_enabled, initialized;
};

static u32 av1_workspace(struct sc2_av1_state *st, enum av1_workspace_id id)
{
    return lower_32_bits(st->work_dma) + av1_workspace_offsets[id];
}
static u32 av1_seq_dma(struct sc2_av1_state *st, unsigned int physical,
                       enum av1_slot_dma_id id)
{
    return lower_32_bits(st->slot[physical].surface.dma) +
           st->slot[physical].surface.mv_offset;
}
static void sc2_av1_reset(struct meson_sc2_hevc_context *ctx)
{
    unsigned int i;
    if (!ctx->av1)
        return;
    ctx->av1->initialized = false;
    for (i = 0; i < 8; i++)
        ctx->av1->ref_map[i] = -1;
    memset(ctx->av1->slot, 0, sizeof(ctx->av1->slot));
    memset(ctx->av1->work, 0, AV1_WORK_SIZE);
}
static void sc2_av1_release(struct meson_sc2_hevc_context *ctx)
{
    if (!ctx->av1)
        return;
    kfree(ctx->av1);
    ctx->av1 = NULL;
}
static int sc2_av1_alloc(struct meson_sc2_hevc_context *ctx)
{
    if (ctx->av1)
        return 0;
    ctx->av1 = kzalloc(sizeof(*ctx->av1), GFP_KERNEL);
    if (!ctx->av1)
        return -ENOMEM;
    /* The context's existing coherent allocation is disjoint from every
     * other context. W_MAP lies beyond this stock AV1 workspace. */
    BUILD_BUG_ON(AV1_WORK_SIZE > W_MAP);
    ctx->av1->work = ctx->work;
    ctx->av1->work_dma = ctx->work_dma;
    sc2_av1_reset(ctx);
    return 0;
}

#define A1_REG(word) ((word) * 4U)
#define A1_SKIP_MODE_INFO A1_REG(0x316c)
#define A1_REF_SCALE_PARSER A1_REG(0x316b)
#define A1_SEG_WRITE A1_REG(0x3165)
#define A1_SEG_READ A1_REG(0x3166)
#define A1_REF_SEG_INFO A1_REG(0x3171)
#define A1_PARSER_MEM_ADDR A1_REG(0x312d)
#define A1_PARSER_MEM_DATA A1_REG(0x312e)
#define A1_PIC_SIZE_FB A1_REG(0x300d)
#define A1_REFINFO_CONFIG A1_REG(0x3442)
#define A1_REFINFO_DATA A1_REG(0x3443)
#define A1_REF_SCALE_MPP A1_REG(0x3441)
#define A1_SAO_CTRL9 A1_REG(0x362d)
#define A1_MMU_STATUS A1_REG(0x3639)
#define A1_DBLK_CDEF0 A1_REG(0x3515)
#define A1_DBLK_CDEF1 A1_REG(0x3516)
#define A1_DBLK_UPS1 A1_REG(0x351c)
#define A1_DBLK_UPS2 A1_REG(0x351d)
#define A1_DBLK_UPS3 A1_REG(0x351e)
#define A1_DBLK_UPS4 A1_REG(0x351f)
#define A1_DBLK_UPS5 A1_REG(0x3520)
#define A1_UPSCALE_X0 A1_REG(0x316e)
#define A1_UPSCALE_STEP A1_REG(0x316f)
#define A1_DBLK_LEVELS A1_REG(0x3523)
#define A1_DBLK_DELTAS A1_REG(0x3524)
#define A1_DBLK_SEGMENT A1_REG(0x3525)
#define A1_FGS_IDX A1_REG(0x3660)
#define A1_FGS_DATA A1_REG(0x3661)
#define A1_FGS_TABLE A1_REG(0x3666)
#define A1_FGS_LENGTH A1_REG(0x3667)
#define A8_QUANT_FEATURE A1_REG(0x3146)
#define A8_MV_RPTR1 A1_REG(0x3263)
#define A8_MV_RPTR2 A1_REG(0x3264)
#define A8_FGS_CTRL A1_REG(0x3662)

static u16 av1_rec_threshold(unsigned int level, unsigned int sharpness)
{
    unsigned int limit = level >> ((sharpness > 0) + (sharpness > 4));

    if (sharpness)
        limit = min(limit, 9U - sharpness);
    limit = max(limit, 1U);
    return (limit << 8) | (2 * (level + 2) + limit);
}

static s32 av1_rec_signed7(unsigned int value)
{
    return sign_extend32(value & 0x7f, 6);
}


static bool av1_seq_intra(unsigned int type)
{
    return type == 0 || type == 2;
}
static int av1_seq_distance(struct sc2_av1_state *st, unsigned int a, unsigned int b)
{
    unsigned int mask, sign, distance;

    if (!st->order_enabled)
        return 0;
    sign = 1U << (st->order_bits - 1);
    mask = (sign << 1) - 1;
    distance = (a - b) & mask;
    return (int)(distance & (sign - 1)) - (int)(distance & sign);
}
static u32 av1_seq_skip_mode(struct sc2_av1_state *st, const unsigned int *p)
{
    int forward = -1, backward = -1, second = -1;
    unsigned int i;

    if (!st->order_enabled || av1_seq_intra(p[14]) || !p[28])
        return 0xff; /* invalid reference nibbles, skip disabled */
    for (i = 0; i < 7; i++) {
        unsigned int hint = st->slot[st->refs[i]].order_hint;
        int distance = av1_seq_distance(st, hint, p[21]);
        if (distance < 0 && (forward < 0 ||
            av1_seq_distance(st, hint, st->slot[st->refs[forward]].order_hint) > 0))
            forward = i;
        if (distance > 0 && (backward < 0 ||
            av1_seq_distance(st, hint, st->slot[st->refs[backward]].order_hint) < 0))
            backward = i;
    }
    if (forward >= 0 && backward >= 0)
        second = backward;
    else if (forward >= 0) {
        for (i = 0; i < 7; i++) {
            unsigned int hint = st->slot[st->refs[i]].order_hint;
            if (av1_seq_distance(st, hint, st->slot[st->refs[forward]].order_hint) < 0 &&
                (second < 0 || av1_seq_distance(st, hint,
                  st->slot[st->refs[second]].order_hint) > 0))
                second = i;
        }
    }
    if (forward < 0 || second < 0)
        return 0xff;
    return BIT(8) | min(forward, second) | (max(forward, second) << 4);
}

/* Vendor motion_field_projection contributes only reference offsets and
 * eligibility here; vector projection itself is real hardware MPRED work.
 */
static bool av1_seq_project(struct sc2_av1_state *st, unsigned int type, int selected[3],
                            s32 offsets[3][7], unsigned int *count)
{
    struct av1_retained_picture *ref = &st->slot[st->refs[type - 1]];
    unsigned int i;

    if (av1_seq_intra(ref->type))
        return false;
    if (*count >= 3)
        return false;
    selected[*count] = type;
    for (i = 0; i < 7; i++)
        offsets[*count][i] = av1_seq_distance(st, ref->order_hint, ref->ref_order_hint[i]);
    (*count)++;
    /* Fixed-size streams satisfy the vendor mi_rows/mi_cols eligibility test. */
    return true;
}

static unsigned int av1_seq_motion_fields(struct sc2_av1_state *st, int selected[3], s32 offsets[3][7])
{
    unsigned int count = 0;
    int stamp = 2;
    struct av1_retained_picture *last = &st->slot[st->refs[0]];
    unsigned int current_hint = st->slot[st->current_slot].order_hint;

    if (!st->order_enabled)
        return 0;
    if (last->ref_order_hint[6] != st->slot[st->refs[3]].order_hint)
        av1_seq_project(st, 1, selected, offsets, &count);
    stamp--;
    if (av1_seq_distance(st, st->slot[st->refs[4]].order_hint, current_hint) > 0 &&
        av1_seq_project(st, 5, selected, offsets, &count))
        stamp--;
    if (av1_seq_distance(st, st->slot[st->refs[5]].order_hint, current_hint) > 0 &&
        av1_seq_project(st, 6, selected, offsets, &count))
        stamp--;
    if (stamp >= 0 && av1_seq_distance(st, st->slot[st->refs[6]].order_hint,
                                     current_hint) > 0 &&
        av1_seq_project(st, 7, selected, offsets, &count))
        stamp--;
    if (stamp >= 0)
        av1_seq_project(st, 2, selected, offsets, &count);
    return count;
}

static void av1_seq_mc(struct sc2_av1_state *st, struct meson_sc2_hw *hw, bool intra)
{
    unsigned int bank, type;
    u32 packed;

    for (bank = 0; bank < 2; bank++) {
        sc2_write(hw, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, ((bank * 16) << 8) | 1);
        for (type = 0; type < 8; type++) {
            unsigned int physical = !type || intra ? st->current_slot : st->refs[type-1];
            struct av1_retained_picture *ref = &st->slot[physical];
            packed = ((u32)ref->order_hint << 24) | (physical << 16) |
                     (physical << 8) | physical;
            sc2_write(hw, HEVCD_MPP_ANC_CANVAS_DATA_ADDR, packed);
        }
    }
    sc2_write(hw, A1_REFINFO_CONFIG, BIT(2));
    for (type = 0; type < 8; type++) {
        sc2_write(hw, A1_REFINFO_DATA, st->width);
        sc2_write(hw, A1_REFINFO_DATA, st->height);
        sc2_write(hw, A1_REFINFO_DATA, 1 << 14);
        sc2_write(hw, A1_REFINFO_DATA, 1 << 14);
        sc2_write(hw, A1_REFINFO_DATA, 0);
    }
    sc2_write(hw, A1_REF_SCALE_MPP, 0);
    sc2_write(hw, A1_REF_SCALE_PARSER, 0); /* all retained dimensions match */
}

static void av1_seq_mpred(struct sc2_av1_state *st, struct meson_sc2_hw *hw, const unsigned int *p, bool intra, bool error_resilient)
{
    int selected[3] = { 0, 0, 0 };
    s32 offsets[3][7] = { { 0 } };
    u32 ref_words[3] = { 0 }, offset_words[6] = { 0 }, value;
    unsigned int count = 0, i, j, byte = 0;
    bool allow_mvs = !intra && !error_resilient && st->order_enabled &&
                      (p[57] & BIT(7)) && p[29];

    value = sc2_read(hw, HEVC_MPRED_CTRL4) & ~BIT(6);
    if (!intra) {
        value &= ~(0xffU << 12);
        for (i = 0; i < 7; i++)
            if (av1_seq_distance(st, st->slot[st->refs[i]].order_hint, p[21]) > 0)
                value |= BIT(13 + i); /* sign_bias[INTRA] is zero */
    }
    sc2_write(hw, HEVC_MPRED_CTRL4, value);
    sc2_write(hw, HEVC_MPRED_CTRL3, 0x13151315); /* SC2 precedes T5D */
    sc2_write(hw, HEVC_MPRED_ABV_START_ADDR, av1_workspace(st, MPRED_ABOVE));
    sc2_write(hw, HEVC_MPRED_L0_REF00_POC, (st->order_enabled << 27) |
      (st->sequence[9] << 24) | (p[21] << 16) | 0x1313);
    if (!intra) {
        /* Exact vendor packing: six5-bit physical indices, then LAST index
         * and seven8-bit order hints across the remaining two words.
         */
        for (i = 1; i < 7; i++)
            ref_words[0] |= (u32)st->refs[i] << ((i - 1) * 5);
        ref_words[1] = ((u32)st->refs[0] << 24) |
            ((u32)st->slot[st->refs[6]].order_hint << 16) |
            ((u32)st->slot[st->refs[5]].order_hint << 8) |
            st->slot[st->refs[4]].order_hint;
        ref_words[2] = ((u32)st->slot[st->refs[3]].order_hint << 24) |
            ((u32)st->slot[st->refs[2]].order_hint << 16) |
            ((u32)st->slot[st->refs[1]].order_hint << 8) |
            st->slot[st->refs[0]].order_hint;
        count = av1_seq_motion_fields(st, selected, offsets);
        for (i = 0; i < count; i++)
            for (j = 0; j < 7; j++, byte++)
                offset_words[byte / 4] |= (u32)(u8)offsets[i][j] <<
                                           ((3 - byte % 4) * 8);
        offset_words[5] |= count | (selected[0] << 2) |
                          (selected[1] << 5) | (selected[2] << 8);
    }
    for (i = 0; i < 3; i++)
        sc2_write(hw, HEVC_MPRED_L0_REF00_POC + (3 + i) * 4, ref_words[i]);
    for (i = 0; i < 6; i++)
        sc2_write(hw, HEVC_MPRED_L0_REF00_POC + (6 + i) * 4, offset_words[i]);
    sc2_write(hw, HEVC_MPRED_MV_WR_START_ADDR, av1_seq_dma(st, st->current_slot, A8_MV));
    sc2_write(hw, HEVC_MPRED_MV_WPTR, av1_seq_dma(st, st->current_slot, A8_MV));
    for (i = 0; i < count; i++) {
        u32 mv = av1_seq_dma(st, st->refs[selected[i] - 1], A8_MV);
        if (!i) {
            sc2_write(hw, HEVC_MPRED_MV_RD_START_ADDR, mv);
            sc2_write(hw, HEVC_MPRED_MV_RPTR, mv);
        } else if (i == 1) {
            sc2_write(hw, HEVC_MPRED_L0_REF00_POC + 4, mv);
            sc2_write(hw, A8_MV_RPTR1, mv);
        } else {
            sc2_write(hw, HEVC_MPRED_L0_REF00_POC + 8, mv);
            sc2_write(hw, A8_MV_RPTR2, mv);
        }
    }
    value = (sc2_read(hw, HEVC_MPRED_CTRL0) & ~(BIT(10) | BIT(11))) | BIT(10);
    if (allow_mvs)
        value |= BIT(11);
    sc2_write(hw, HEVC_MPRED_CTRL0, value);
}

static void av1_seq_mcrcc(struct sc2_av1_state *st, struct meson_sc2_hw *hw, bool intra)
{
    unsigned int i, nearest = 1, second = 1;
    unsigned int best = 1023, distance[7];
    u32 first_canvas, canvas;
    unsigned int current_hint = st->slot[st->current_slot].order_hint;

    sc2_write(hw, HEVCD_MCRCC_CTL1, 2);
    if (intra) {
        sc2_write(hw, HEVCD_MCRCC_CTL2, ~0U);
        sc2_write(hw, HEVCD_MCRCC_CTL3, ~0U);
        sc2_write(hw, HEVCD_MCRCC_CTL1, 0xff0);
        return;
    }
    for (i = 0; i < 7; i++) {
        distance[i] = abs(av1_seq_distance(st, st->slot[st->refs[i]].order_hint,
                                          current_hint));
        if (distance[i] < best) {
            nearest = i + 1;
            best = distance[i];
        }
    }
    sc2_write(hw, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, (nearest << 8) | BIT(1));
    first_canvas = sc2_read(hw, HEVCD_MPP_ANC_CANVAS_DATA_ADDR) & 0xffff;
    best = 1023;
    for (i = 0; i < 7; i++) {
        sc2_write(hw, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, ((i + 1) << 8) | BIT(1));
        canvas = sc2_read(hw, HEVCD_MPP_ANC_CANVAS_DATA_ADDR) & 0xffff;
        if (distance[i] < best && canvas != first_canvas) {
            second = i + 1;
            best = distance[i];
        }
    }
    sc2_write(hw, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, (nearest << 8) | BIT(1));
    canvas = sc2_read(hw, HEVCD_MPP_ANC_CANVAS_DATA_ADDR) & 0xffff;
    sc2_write(hw, HEVCD_MCRCC_CTL2, canvas | (canvas << 16));
    sc2_write(hw, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, (second << 8) | BIT(1));
    canvas = sc2_read(hw, HEVCD_MPP_ANC_CANVAS_DATA_ADDR) & 0xffff;
    sc2_write(hw, HEVCD_MCRCC_CTL3, canvas | (canvas << 16));
    sc2_write(hw, HEVCD_MCRCC_CTL1, 0xff0);
}

static void av1_seq_filters(struct sc2_av1_state *st, struct meson_sc2_hw *hw, const unsigned int *p)
{
    static const s32 defaults[8] = { 1, 0, 0, 0, -1, 0, -1, -1 };
    struct av1_retained_picture *cur = &st->slot[st->current_slot];
    struct av1_retained_picture *prev = st->primary < 0 ? NULL :
                                       &st->slot[st->primary];
    unsigned int i;
    u32 control = p[61];

    for (i = 0; i < 32; i++)
        sc2_write(hw, HEVC_DBLK_CFG9, av1_rec_threshold(i * 2, p[60]) |
          ((u32)av1_rec_threshold(i * 2 + 1, p[60]) << 16));
    for (i = 0; i < 8; i++) {
        unsigned int packed = p[62 + i / 2] >> ((i & 1) * 8);
        if ((control & 3) == 3)
            cur->ref_delta[i] = !prev || (packed & 0x80) ?
                av1_rec_signed7(packed) : prev->ref_delta[i];
        else
            cur->ref_delta[i] = !prev || (control & 4) ? defaults[i] : prev->ref_delta[i];
    }
    for (i = 0; i < 2; i++) {
        unsigned int packed = p[70] >> (i * 8);
        if ((control & 3) == 3)
            cur->mode_delta[i] = !prev || (packed & 0x80) ?
                av1_rec_signed7(packed) : prev->mode_delta[i];
        else
            cur->mode_delta[i] = !prev || (control & 4) ? 0 : prev->mode_delta[i];
    }
    cur->seg_enabled = p[76] & 1;
    cur->intra_only = (p[76] >> 2) & 1;
    cur->seg_update_map = (p[76] >> 3) & 1;
    cur->prev_seg_enabled = prev && prev->seg_enabled &&
                           (prev->seg_update_map || prev->prev_seg_enabled);
    cur->seg_mi_cols = DIV_ROUND_UP(st->width, 4);
    cur->seg_mi_rows = DIV_ROUND_UP(st->height, 4);
    for (i = 0; i < 8; i++) {
        cur->seg_y[i] = cur->seg_c[i] = 0;
        if ((p[76] & 3) == 3) {
            cur->seg_y[i] = p[77 + i];
            cur->seg_c[i] = p[85 + i];
        } else if (cur->seg_enabled && prev) {
            cur->seg_y[i] = prev->seg_y[i];
            cur->seg_c[i] = prev->seg_c[i];
        }
    }
    sc2_write(hw, HEVC_DBLK_CFGB, (0x57 << 8) | 4 | ((control & 1) ? BIT(28) : 0) |
      (cur->seg_enabled ? BIT(29) : 0) | (st->width >= 1280 ? BIT(4) : 0));
    sc2_write(hw, A1_DBLK_LEVELS, p[72] | (p[73] << 6) | (p[74] << 12) | (p[75] << 18));
    for (i = 0; i < 10; i++)
        sc2_write(hw, A1_DBLK_DELTAS, i < 2 ? cur->mode_delta[i] : cur->ref_delta[i - 2]);
    for (i = 0; i < 8; i++)
        sc2_write(hw, A1_DBLK_SEGMENT, (u32)cur->seg_y[i] | ((u32)cur->seg_c[i] << 16));
    sc2_write(hw, A1_DBLK_CDEF0, av1_workspace(st, CDEF));
    sc2_write(hw, A1_DBLK_CDEF1, (sc2_read(hw, A1_DBLK_CDEF1) & 0xffffff00) | 17);
    sc2_write(hw, A1_DBLK_UPS1, av1_workspace(st, UPS));
    sc2_write(hw, A1_DBLK_UPS2, 128);
    sc2_write(hw, A1_DBLK_UPS3, 128);
    sc2_write(hw, A1_DBLK_UPS4, 1 << 14);
    sc2_write(hw, A1_DBLK_UPS5, 1 << 14);
    sc2_write(hw, A1_UPSCALE_X0, (128 << 16) | 128);
    sc2_write(hw, A1_UPSCALE_STEP, (1U << 30) | (1 << 14));
    sc2_write(hw, HEVC_ASSIST_SCRATCH_8, av1_workspace(st, CDF) + st->current_slot * 0x8000);
    sc2_write(hw, A1_SEG_WRITE, av1_workspace(st, SEG) + st->current_slot * 0xca80);
    sc2_write(hw, HEVC_ASSIST_SCRATCH_9, av1_workspace(st, CDF) + (prev ? st->primary * 0x8000 : 0));
    sc2_write(hw, A1_SEG_READ, av1_workspace(st, SEG) + (prev ? st->primary * 0xca80 : 0));
    if (prev && (p[76] & 3) == 1)
        for (i = 0; i < 8; i++)
            sc2_write(hw, A8_QUANT_FEATURE, prev->segment_feature[i]);
    if (prev && (p[76] & 9) == 1) {
        cur->seg_mi_cols = prev->seg_mi_cols;
        cur->seg_mi_rows = prev->seg_mi_rows;
    }
    sc2_write(hw, A1_PARSER_MEM_ADDR, 0x11b0 + st->current_slot);
    sc2_write(hw, A1_PARSER_MEM_DATA, cur->seg_enabled ? sc2_read(hw, A1_REF_SEG_INFO) : 0);
    for (i = 0; i < 8; i++)
        cur->segment_feature[i] = cur->seg_enabled ? sc2_read(hw, A8_QUANT_FEATURE) :
                                  (BIT(31) | (i << 22));
}

static void sc2_av1_parser(struct meson_sc2_hevc_context *ctx,
                            const struct meson_sc2_av1_job *job, size_t length)
{
    struct sc2_av1_state *st = ctx->av1;
    struct meson_sc2_hw *hw = ctx->hw;
    unsigned int i;
    u32 value;

    /* aom_config_work_space_hw: FRONT | BACK, main MMU on, DW MMU off. */
    sc2_write(hw, HEVC_ASSIST_SCRATCH_1, lower_32_bits(st->work_dma + AV1_RPM));
    sc2_write(hw, HEVC_ASSIST_SCRATCH_I, lower_32_bits(st->work_dma + AV1_LMEM));
    sc2_write(hw, HEVC_ASSIST_SCRATCH_5, av1_workspace(st, DAALA));
    sc2_write(hw, AV1_GMC_PARAM_BUFF_ADDR, av1_workspace(st, GMC));
    sc2_write(hw, HEVCD_IPP_LINEBUFF_BASE, av1_workspace(st, IPP));
    sc2_write(hw, HEVC_DBLK_CFG4, av1_workspace(st, DBLK_PARA));
    sc2_write(hw, HEVC_DBLK_CFG5, av1_workspace(st, DBLK_DATA));
    sc2_write(hw, HEVC_DBLK_CFG3, 0x404010);
    sc2_write(hw, HEVCD_MPP_DECOMP_CTL1, BIT(4));
    sc2_write(hw, HEVCD_MPP_DECOMP_CTL2, 0);
    sc2_write(hw, HEVC_CM_BODY_LENGTH, job->output.destination->body_size);
    sc2_write(hw, HEVC_CM_HEADER_OFFSET, job->output.destination->body_size);
    sc2_write(hw, HEVC_CM_HEADER_LENGTH, job->output.destination->header_size);
    sc2_write(hw, HEVC_SAO_MMU_VH0_ADDR, av1_workspace(st, MMU_VBH));
    sc2_write(hw, HEVC_SAO_MMU_VH1_ADDR, av1_workspace(st, MMU_VBH) + 0x1800);
    value = (sc2_read(hw, HEVC_SAO_CTRL5) | BIT(10)) & ~BIT(15);
    sc2_write(hw, HEVC_SAO_CTRL5, value);
    sc2_write(hw, HEVC_ASSIST_SCRATCH_I, lower_32_bits(st->work_dma + AV1_LMEM));
    sc2_write(hw, AV1_MMU_DMA_CTRL, lower_32_bits(ctx->work_dma + W_MAP));
    sc2_write(hw, HEVC_MPRED_CTRL4, sc2_read(hw, HEVC_MPRED_CTRL4) | BIT(1));
    sc2_write(hw, HEVC_ASSIST_SCRATCH_L, lower_32_bits(st->work_dma + AV1_AUX));
    sc2_write(hw, HEVC_ASSIST_SCRATCH_7, (SZ_16K >> 4) << 16);
    sc2_write(hw, HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR, BIT(1) | BIT(2));
    for (i = 0; i < A8_PHYSICAL_SLOTS; i++) {
        struct meson_sc2_surface *s = st->slot[i].valid ?
            &st->slot[i].surface : job->output.destination;
        sc2_write(hw, HEVCD_MPP_ANC2AXI_TBL_DATA, s->dma >> 5);
    }
    sc2_write(hw, HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR, 1);
    sc2_write(hw, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, BIT(8) | 1);
    for (i = 0; i < 32; i++)
        sc2_write(hw, HEVCD_MPP_ANC_CANVAS_DATA_ADDR, 0);

    /* Native SC2 FIFO programming used by the measured AV1 SINGLE path. */
    sc2_write(hw, HEVC_STREAM_CONTROL, 0x0eb14070);
    sc2_write(hw, HEVC_STREAM_START_ADDR, lower_32_bits(ctx->input_dma));
    sc2_write(hw, HEVC_STREAM_END_ADDR, lower_32_bits(ctx->input_dma) + SC2_INPUT_SIZE);
    sc2_write(hw, HEVC_STREAM_RD_PTR, lower_32_bits(ctx->input_dma));
    sc2_write(hw, HEVC_STREAM_WR_PTR, lower_32_bits(ctx->input_dma) +
      round_down(length + 16384, 64));
    sc2_write(hw, SC2_DOS_SW_RESET3, BIT(14));
    sc2_write(hw, HEVC_CABAC_CONTROL, sc2_read(hw, HEVC_CABAC_CONTROL) & ~1);
    sc2_write(hw, HEVC_PARSER_CORE_CONTROL, sc2_read(hw, HEVC_PARSER_CORE_CONTROL) & ~1);
    /* AV1 differs from HEVC: FIFO ctl=3 and no emulation check. */
    sc2_write(hw, HEVC_PARSER_INT_CONTROL, (sc2_read(hw, HEVC_PARSER_INT_CONTROL) & 0x03ffffff) |
      (3U << 29) | (3 << 26) | BIT(24) | BIT(22) | BIT(7) | BIT(4) | 1);
    sc2_write(hw, HEVC_SHIFT_STATUS, (sc2_read(hw, HEVC_SHIFT_STATUS) & ~BIT(1)) | 1);
    sc2_write(hw, HEVC_SHIFT_CONTROL, BIT(10) | BIT(9) | (3 << 6) | (2 << 4) |
      (3 << 1) | 1);
    sc2_write(hw, HEVC_CABAC_CONTROL, 1);
    sc2_write(hw, HEVC_PARSER_CORE_CONTROL, 1);
    sc2_write(hw, HEVC_ASSIST_SCRATCH_0, 0);
    sc2_write(hw, HEVC_IQIT_SCALELUT_WR_ADDR, 0);
    for (i = 0; i < 1024; i++)
        sc2_write(hw, HEVC_IQIT_SCALELUT_DATA, 0);
    sc2_write(hw, HEVC_ASSIST_SCRATCH_J, 0x80000000); /* continuous SINGLE, mailbox0 */
    sc2_write(hw, HEVC_ASSIST_SCRATCH_N, 0);
    sc2_write(hw, HEVC_ASSIST_SCRATCH_M, 0);
    sc2_write(hw, HEVC_PARSER_CMD_WRITE, BIT(16));
    for (i = 0; i < ARRAY_SIZE(sc2_parser_commands); i++)
        sc2_write(hw, HEVC_PARSER_CMD_WRITE, sc2_parser_commands[i]);
    sc2_write(hw, HEVC_PARSER_CMD_SKIP_0, 0x0000090b);
    sc2_write(hw, HEVC_PARSER_CMD_SKIP_1, 0x1b14140f);
    sc2_write(hw, HEVC_PARSER_CMD_SKIP_2, 0x001b1910);
    sc2_write(hw, HEVC_PARSER_IF_CONTROL, BIT(5) | BIT(2) | 1);
    sc2_write(hw, HEVCD_IPP_TOP_CNTL, 1);
    sc2_write(hw, HEVCD_IPP_TOP_CNTL, (3 << 4) | BIT(1)); /* AV1 backend mode */
    sc2_write(hw, AV1_DECOMP_AXIURG_CTL, 1 | BIT(2) | BIT(3) | (24 << 4) |
      (32 << 11) | (24 << 18) | (32U << 25));
    sc2_write(hw, HEVC_STREAM_CONTROL, sc2_read(hw, HEVC_STREAM_CONTROL) | 1);
    sc2_write(hw, HEVC_STREAM_FIFO_CTL, sc2_read(hw, HEVC_STREAM_FIFO_CTL) | BIT(29));
    sc2_write(hw, HEVC_SHIFT_STARTCODE, 1); /* AV1 00 00 00 01 */
    sc2_write(hw, HEVC_SHIFT_EMULATECODE, 0x00000300);
    sc2_write(hw, HEVC_ASSIST_SCRATCH_E, 1);
    sc2_write(hw, SC2_MBOX_CLEAR, 1);
    sc2_write(hw, SC2_MBOX_MASK, 1);
    sc2_write(hw, HEVC_PSCALE_CTRL, 0);
    sc2_write(hw, HEVC_ASSIST_SCRATCH_G, 0);
    sc2_write(hw, HEVC_ASSIST_SCRATCH_H, 0);
    sc2_write(hw, HEVC_ASSIST_SCRATCH_I, 8); /* NAL_SEARCH_CTL aliases LMEM_DUMP_ADR */
    sc2_write(hw, HEVC_ASSIST_SCRATCH_K, 0);
    sc2_write(hw, HEVC_ASSIST_SCRATCH_C, 0);
    sc2_write(hw, HEVC_ASSIST_SCRATCH_0, 0xff); /* HEVC_ACTION_DONE */
    sc2_write(hw, HEVC_SHIFT_BYTE_COUNT, 0);
    sc2_write(hw, HEVC_ASSIST_SCRATCH_N, length);
    /* run_front() final binding overrides scratch-I search initialization. */
    sc2_write(hw, HEVC_ASSIST_SCRATCH_I, lower_32_bits(st->work_dma + AV1_LMEM));
}

/* Reconstruct the SC2 parser's logical-slot view on every fresh invocation.
 * Firmware registers/LMEM never carry authority across codecs or contexts. */
static void sc2_av1_ref_info(struct sc2_av1_state *st, struct meson_sc2_hw *hw)
{
    unsigned int i, j;
    sc2_write(hw, A1_PARSER_MEM_ADDR, 0x11a0);
    for (i = 0; i < 12; i++) {
        u32 info = 0;
        if (i < 2 || i >= 10) {
            for (j = 0; j < 4; j++) {
                int slot = st->ref_map[(i < 2 ? i : i - 10) * 4 + j];
                if (slot >= 0)
                    info |= (i < 2 ? st->slot[slot].order_hint :
                             (u32)slot) << (j * 8);
            }
        } else if (st->ref_map[i - 2] >= 0) {
            info = (st->width << 16) | st->height;
        }
        sc2_write(hw, A1_PARSER_MEM_DATA, info);
    }
}

/* Only our own canonical serializer's OBUs are read here, never source headers.
 * Section-5 AMLV envelope, vdec_av1_if.c:806-840. Source and destination do not
 * overlap: normalized bytes occupy the upper half of the input allocation. */
static int sc2_av1_frame_obus(struct meson_sc2_hevc_context *ctx,
                             size_t normalized, size_t *length)
{
    const u8 *src = ctx->input + SZ_8M;
    u8 *dst = ctx->input;
    size_t pos = 0, out = 0;
    while (pos < normalized) {
        size_t start = pos, payload = 0, size;
        unsigned int shift = 0, i;
        u8 byte;
        if (!(src[pos] & 2) || (src[pos] & 0x85))
            return -EPROTO;
        pos++;
        do {
            if (pos >= normalized || shift >= 28)
                return -EPROTO;
            byte = src[pos++];
            payload |= (size_t)(byte & 0x7f) << shift;
            shift += 7;
        } while (byte & 0x80);
        if (payload > normalized - pos)
            return -EPROTO;
        size = pos - start + payload;
        if (out + size + 20 > SZ_8M - SC2_INPUT_PADDING)
            return -ENOSPC;
        for (i = 0; i < 4; i++) {
            dst[out + i] = (size + 8) >> (24 - i * 8);
            dst[out + 4 + i] = dst[out + i] ^ 0xff;
        }
        memcpy(dst + out + 8, "\0\0\0\1AMLV", 8);
        for (i = 0; i < 4; i++)
            dst[out + 16 + i] = ((size + 4) >> (i * 7)) & 0x7f;
        dst[out + 16] |= 0x80;
        dst[out + 17] |= 0x80;
        dst[out + 18] |= 0x80;
        memcpy(dst + out + 20, src + start, size);
        pos += payload;
        out += size + 20;
    }
    memset(dst + out, 0, SC2_INPUT_PADDING);
    *length = out;
    return 0;
}

static int sc2_av1_validate(struct meson_sc2_hevc_context *ctx,
                           const struct meson_sc2_av1_job *job)
{
    struct sc2_av1_state *st = ctx->av1;
    const struct meson_sc2_hevc_job *out = &job->output;
    const struct v4l2_ctrl_av1_sequence *s = job->syntax->sequence;
    const struct v4l2_ctrl_av1_frame *f = job->syntax->frame;
    struct sc2_surface_owner *owner = sc2_owner(ctx->hw, out->destination);
    bool used[16] = {};
    int map[8];
    unsigned int i, j;
    u32 width = f->frame_width_minus_1 + 1;
    u32 height = f->frame_height_minus_1 + 1;
    bool intra = f->frame_type == V4L2_AV1_KEY_FRAME ||
                 f->frame_type == V4L2_AV1_INTRA_ONLY_FRAME;

    /* Main8 NV12 only. DW8 is not full-depth Main10. The stock workspace
     * capacities are the <=1920x1088 layout, not an invented 4K layout. */
    if (s->seq_profile || s->bit_depth != 8 || width < 64 || height < 64 ||
        width > 1920 || height > 1088 || (width & 1) || (height & 1) ||
        (s->flags & (V4L2_AV1_SEQUENCE_FLAG_MONO_CHROME |
                     V4L2_AV1_SEQUENCE_FLAG_FILM_GRAIN_PARAMS_PRESENT |
                     V4L2_AV1_SEQUENCE_FLAG_USE_128X128_SUPERBLOCK)) ||
        (s->flags & (V4L2_AV1_SEQUENCE_FLAG_SUBSAMPLING_X |
                     V4L2_AV1_SEQUENCE_FLAG_SUBSAMPLING_Y)) !=
                    (V4L2_AV1_SEQUENCE_FLAG_SUBSAMPLING_X |
                     V4L2_AV1_SEQUENCE_FLAG_SUBSAMPLING_Y) ||
        f->current_frame_id ||
        (f->flags & (V4L2_AV1_FRAME_FLAG_USE_SUPERRES |
                     V4L2_AV1_FRAME_FLAG_ALLOW_INTRABC |
                     V4L2_AV1_FRAME_FLAG_BUFFER_REMOVAL_TIME_PRESENT)) ||
        f->upscaled_width != width || f->tile_info.tile_cols > 64 ||
        f->tile_info.tile_rows > 64 || job->syntax->num_tiles > 256)
        return -EOPNOTSUPP;
    /* A fresh parser has no implicit previous global-motion predictor bank.
     * Reject nonidentity models instead of silently losing that authority. */
    for (i = 0; i < 8; i++)
        if (f->global_motion.type[i] != V4L2_AV1_WARP_MODEL_IDENTITY)
            return -EOPNOTSUPP;
    if (!owner || (owner->context && owner->context != ctx) ||
        owner->surface.width != width || owner->surface.height != height ||
        owner->surface.depth != 8 || owner->surface.mv_size < SZ_1M ||
        !out->output_buffer || out->output_stride != ALIGN(width, 64) ||
        out->output_uv_offset != out->output_stride * ALIGN(height, 64) ||
        out->output_size < out->output_uv_offset * 3 / 2 ||
        out->output_buffer->size < out->output_size)
        return -EINVAL;
    if (st->initialized &&
        (width != st->width || height != st->height ||
         memcmp(s, &st->sequence_control, sizeof(*s))))
        return -EOPNOTSUPP; /* STREAMOFF establishes a new sequence boundary. */
    if (!st->initialized && f->frame_type != V4L2_AV1_KEY_FRAME)
        return -ENODATA;
    memset(st->snapshots, 0, sizeof(st->snapshots));
    for (i = 0; i < 8; i++) {
        int physical = -1;
        map[i] = -1;
        /* A key frame reads no old references. Their allocations remain
         * owned by sc2_surface_owner, but a returned CAPTURE may now be the
         * key frame's destination, including after a userspace-only flush.
         */
        if (f->frame_type == V4L2_AV1_KEY_FRAME)
            continue;
        if (!job->references[i])
            continue;
        for (j = 0; j < 16; j++) {
            if (!st->slot[j].valid ||
                st->slot[j].surface.cpu != job->references[i]->cpu ||
                st->slot[j].timestamp != f->reference_frame_ts[i])
                continue;
            if (physical >= 0)
                return -EINVAL;
            physical = j;
        }
        if (physical < 0)
            return -ENOENT;
        owner = sc2_owner(ctx->hw, job->references[i]);
        if (!owner || owner->context != ctx || !owner->surface.valid ||
            !job->references[i]->valid || owner->surface.cpu == out->destination->cpu)
            return -ENOENT;
        map[i] = physical;
        used[physical] = true;
        st->snapshots[i] = (struct meson_av1_syntax_reference) {
            .frame = &st->slot[physical].frame,
            .timestamp = st->slot[physical].timestamp,
            .order_hint = st->slot[physical].order_hint, .valid = true,
        };
    }
    if (!intra) {
        for (i = 0; i < 7; i++) {
            int logical = f->ref_frame_idx[i];
            if (logical < 0 || logical >= 8 || map[logical] < 0 ||
                !st->snapshots[logical].valid)
                return -ENOENT;
            st->refs[i] = map[logical];
        }
    }
    for (j = 0; j < 16; j++) {
        if (used[j] && st->slot[j].surface.cpu == out->destination->cpu)
            return -EBUSY;
        if (used[j] && st->slot[j].timestamp == job->timestamp)
            return -EINVAL;
    }
    for (j = 0; j < 16 && used[j]; j++)
        ;
    if (j == 16)
        return -ENOSPC;
    /* Timestamp-resolved input slots also cover userspace show-existing key
     * reuse (which has no native coded request). A coded key frame instead
     * retires every old mapping before any of its backing can be overwritten.
     */
    memcpy(st->ref_map, map, sizeof(map));
    st->current_slot = j;
    /* Drop stale physical table entries before exposing any address. */
    for (i = 0; i < 16; i++)
        if (!used[i])
            st->slot[i].valid = false;
    st->slot[j] = (struct av1_retained_picture) {
        .surface = *out->destination, .frame = *f, .timestamp = job->timestamp,
        .type = f->frame_type, .order_hint = f->order_hint,
    };
    for (i = 0; i < 7; i++) {
        if (intra)
            st->refs[i] = j;
        else
            st->slot[j].ref_order_hint[i] = st->slot[st->refs[i]].order_hint;
    }
    st->primary = f->primary_ref_frame == 7 ? -1 :
        (f->primary_ref_frame < 7 && !intra ? st->refs[f->primary_ref_frame] : -1);
    st->width = width;
    st->height = height;
    st->order_enabled = s->flags & V4L2_AV1_SEQUENCE_FLAG_ENABLE_ORDER_HINT;
    st->order_bits = s->order_hint_bits;
    st->sequence[9] = st->order_bits ? st->order_bits - 1 : 0;
    job->syntax->references = st->snapshots;
    job->syntax->num_references = 8;
    /* Each invocation starts a fresh parser; external CDF/segment history is
     * explicitly rebound, never zeroed between dependent pictures. */
    job->syntax->emit_sequence_header = true;
    return 0;
}

static void sc2_av1_program(struct meson_sc2_hevc_context *ctx,
                            const struct meson_sc2_av1_job *job, const u32 *p)
{
    struct sc2_av1_state *st = ctx->av1;
    struct meson_sc2_hw *hw = ctx->hw;
    const struct meson_sc2_hevc_job *out = &job->output;
    struct meson_sc2_surface *s = out->destination;
    __le32 *map = ctx->work + W_MAP;
    bool intra = av1_seq_intra(p[14]);
    bool resilient = p[14] == 3 || (p[14] == 0 && p[15]) || p[16];
    u32 value, skip = av1_seq_skip_mode(st, p);
    unsigned int i, pages = DIV_ROUND_UP(s->body_size, SZ_4K);

    for (i = 0; i < SC2_MAP_ENTRIES; i++)
        map[i] = cpu_to_le32((s->dma + s->body_offset +
                              (i < pages ? i * SZ_4K : 0)) >> PAGE_SHIFT);
    if (st->primary >= 0) {
        struct av1_retained_picture *prev = &st->slot[st->primary];
        bool inherited = prev->seg_enabled &&
                         (prev->seg_update_map || prev->prev_seg_enabled);
        skip |= (inherited ? BIT(31) : 0) | (prev->intra_only ? BIT(30) : 0) |
                ((u32)st->primary << 24);
    } else {
        skip |= 0x1fU << 24;
    }
    sc2_write(hw, HEVC_ASSIST_SCRATCH_2, 0);
    sc2_write(hw, A1_SKIP_MODE_INFO, skip | (st->current_slot << 16));
    sc2_write(hw, HEVC_PARSER_PICTURE_SIZE, (st->height << 16) | st->width);
    sc2_write(hw, A1_PIC_SIZE_FB, (st->height << 16) | st->width);
    sc2_write(hw, HEVC_SAO_CTRL5,
              (sc2_read(hw, HEVC_SAO_CTRL5) | BIT(9) | BIT(10)) & ~BIT(15));
    sc2_write(hw, HEVCD_MPP_DECOMP_CTL1, BIT(4));
    sc2_write(hw, HEVCD_MPP_DECOMP_CTL2, s->body_size >> 5);
    sc2_write(hw, HEVC_CM_BODY_LENGTH, s->body_size);
    sc2_write(hw, HEVC_CM_HEADER_OFFSET, s->body_size);
    sc2_write(hw, HEVC_CM_HEADER_LENGTH, s->header_size);
    sc2_write(hw, AV1_MMU_DMA_CTRL, lower_32_bits(ctx->work_dma + W_MAP));
    av1_seq_mc(st, hw, intra);
    av1_seq_mpred(st, hw, p, intra, resilient);
    av1_seq_mcrcc(st, hw, intra);
    sc2_write(hw, A1_SAO_CTRL9, sc2_read(hw, A1_SAO_CTRL9) | BIT(1));
    sc2_write(hw, HEVC_SAO_CTRL5,
              (sc2_read(hw, HEVC_SAO_CTRL5) | BIT(14)) & ~(0xffU << 16));
    sc2_write(hw, HEVC_SAO_CTRL0, 6);
    sc2_write(hw, HEVC_CM_BODY_START_ADDR, lower_32_bits(s->dma + s->body_offset));
    sc2_write(hw, HEVC_CM_HEADER_START_ADDR, lower_32_bits(s->dma));
    sc2_write(hw, A1_FGS_TABLE, av1_workspace(st, GRAIN) + st->current_slot * 0x2000);
    sc2_write(hw, A1_FGS_LENGTH, 0x2000 * 8);
    sc2_write(hw, HEVC_SAO_Y_LENGTH, out->output_uv_offset);
    sc2_write(hw, HEVC_SAO_C_LENGTH, out->output_size - out->output_uv_offset);
    sc2_write(hw, HEVC_SAO_Y_START_ADDR, lower_32_bits(hw->output_dma));
    sc2_write(hw, HEVC_SAO_C_START_ADDR,
              lower_32_bits(hw->output_dma) + out->output_uv_offset);
    sc2_write(hw, HEVC_SAO_Y_WPTR, lower_32_bits(hw->output_dma));
    sc2_write(hw, HEVC_SAO_C_WPTR,
              lower_32_bits(hw->output_dma) + out->output_uv_offset);
    value = sc2_read(hw, HEVC_SAO_CTRL1) & ~(0x3000 | 0xff0 | 3 | (3 << 14));
    sc2_write(hw, HEVC_SAO_CTRL1, value | 0xff0 | BIT(8) | (2 << 14));
    value = sc2_read(hw, HEVCD_IPP_AXIIF_CONFIG) & ~(0x30 | 0xf | BIT(12) | (3 << 8));
    sc2_write(hw, HEVCD_IPP_AXIIF_CONFIG, value | 0xf | (2 << 8));
    av1_seq_filters(st, hw, p);
    dma_wmb();
    sc2_write(hw, HEVC_ASSIST_SCRATCH_0, 5); /* AV1: status only, NO MCPU IRQ. */
}

static int sc2_av1_run(struct meson_sc2_hevc_context *ctx,
                       const struct meson_sc2_av1_job *job, size_t length)
{
    struct meson_sc2_hw *hw = ctx->hw;
    struct sc2_av1_state *st = ctx->av1;
    const struct v4l2_ctrl_av1_frame *f = job->syntax->frame;
    unsigned long deadline = jiffies + SC2_JOB_TIMEOUT, remaining;
    unsigned int sequences = 0, headers = 0, grain = 0, i;
    u32 p[128], status, sequence;
    const __le16 *raw = st->work + AV1_RPM;

    sc2_av1_parser(ctx, job, length);
    dma_wmb();
    sc2_write(hw, SC2_DOS_SW_RESET3, BIT(12) | BIT(11));
    sc2_write(hw, SC2_DOS_SW_RESET3, 0);
    sc2_read(hw, SC2_DOS_SW_RESET3);
    usleep_range(10, 20);
    sc2_write(hw, HEVC_MPSR, 1);
    for (;;) {
        if (atomic_read(&ctx->aborted))
            return -ECANCELED;
        if (time_after_eq(jiffies, deadline))
            return -ETIMEDOUT;
        sequence = smp_load_acquire(&hw->irq_sequence);
        status = sc2_read(hw, HEVC_ASSIST_SCRATCH_0) & 0xff;
        if (status == 0xf2 || status == 0xf3 || status == 0xf4) {
            dma_rmb();
            for (i = 0; i < 128; i++)
                p[i] = le16_to_cpu(raw[(i & ~3U) + 3 - (i & 3)]);
            if (status == 0xf2) {
                if (++sequences != 1 || p[0] || p[56] != 8 || p[57] & BIT(15))
                    return -EPROTO;
                sc2_av1_ref_info(st, hw);
                dma_wmb();
                sc2_write(hw, HEVC_ASSIST_SCRATCH_0, 6);
            } else {
                if (!sequences || ++headers != 1 || p[13] || p[14] != f->frame_type ||
                    p[21] != f->order_hint || p[22] != f->primary_ref_frame ||
                    p[17] != f->refresh_frame_flags || p[25] != st->width ||
                    p[26] != st->width || p[27] != st->height || p[56] != 8)
                    return -EPROTO;
                for (i = 0; i < 7; i++)
                    if (!av1_seq_intra(p[14]) && p[34 + i] != f->ref_frame_idx[i])
                        return -EPROTO;
                sc2_av1_program(ctx, job, p);
            }
            continue;
        }
        if (status == 0xe0) {
            if (headers != 1 || sc2_read(hw, HEVC_ASSIST_SCRATCH_2) !=
                DIV_ROUND_UP(st->width, 64) * DIV_ROUND_UP(st->height, 64) ||
                sc2_read(hw, HEVC_SHIFT_BYTE_COUNT) > length + SC2_INPUT_PADDING ||
                (sc2_read(hw, A1_MMU_STATUS) >> 16) >
                    DIV_ROUND_UP(job->output.destination->body_size, SZ_4K))
                return -EIO;
            return 0;
        }
        if (status == 0xe1) {
            /* Firmware also exports the parameter bank for a no-grain frame.
             * No synthesized grain or inherited donor is accepted here. */
            if (!headers || ++grain > 4 ||
                (sc2_read(hw, HEVC_ASSIST_SCRATCH_B) & BIT(11)))
                return -EPROTO;
            sc2_write(hw, HEVC_ASSIST_SCRATCH_B, 8);
            sc2_write(hw, HEVC_ASSIST_SCRATCH_0, 8);
            sc2_write(hw, A1_FGS_IDX, 0);
            for (i = 0; i < 39; i++)
                sc2_read(hw, A1_FGS_DATA);
            continue;
        }
        if ((status >= 0x20 && status <= 0x25) || status == 0xe2 || status == 0xf5)
            return -EBADMSG;
        remaining = deadline - jiffies;
        if ((long)remaining <= 0)
            return -ETIMEDOUT;
        wait_event_timeout(hw->wait, atomic_read(&ctx->aborted) ||
            smp_load_acquire(&hw->irq_sequence) != sequence, remaining);
    }
}

int meson_sc2_av1_decode(struct meson_sc2_hevc_context *ctx,
                          const struct meson_sc2_av1_job *job)
{
    struct meson_sc2_hw *hw;
    struct sc2_av1_state *st;
    struct sc2_surface_owner *owner;
    struct meson_av1_syntax_error error;
    size_t normalized = 0, length = 0;
    s32 firmware_status;
    bool recovered, state_prepared = false;
    unsigned int i;
    int ret, drain;

    if (!ctx || !job || !job->syntax || !job->syntax->frame ||
        !job->syntax->sequence || !job->output.destination)
        return -EINVAL;
    hw = ctx->hw;
    mutex_lock(&hw->lock);
    if (hw->fatal || atomic_read(&ctx->aborted)) {
        ret = hw->fatal ? -EIO : -ECANCELED;
        goto unlock;
    }
    ret = sc2_av1_alloc(ctx);
    if (ret)
        goto unlock;
    st = ctx->av1;
    ret = sc2_av1_validate(ctx, job);
    if (ret)
        goto unlock;
    state_prepared = true;
    ret = meson_av1_request_syntax(job->syntax, ctx->input + SZ_8M,
             SZ_8M - SC2_INPUT_PADDING, &normalized, &error);
    if (ret) {
        dev_dbg(hw->dev, "AV1 syntax tile %zu: %s (%d)\n",
                error.tile, error.reason, ret);
        goto unlock;
    }
    ret = sc2_av1_frame_obus(ctx, normalized, &length);
    if (ret)
        goto unlock;
    owner = sc2_owner(hw, job->output.destination);
    owner->context = ctx;
    owner->surface.valid = false;
    job->output.destination->valid = false;
    ret = sc2_pin_output(hw, &job->output);
    if (ret)
        goto unlock;
    ret = sc2_power_on(hw);
    if (ret)
        goto unpin;
    ret = sc2_prepare(hw);
    if (!ret) {
        firmware_status = -EIO;
        ret = meson_sm_call(hw->firmware, SM_VIDEO_FW_LOAD, &firmware_status,
                            0x4156314d, 2, 0, 0, 0);
        if (!ret && firmware_status)
            ret = -EIO;
    }
    if (!ret && !atomic_read(&ctx->aborted))
        ret = sc2_av1_run(ctx, job, length);
    else if (!ret)
        ret = -ECANCELED;
    drain = sc2_drain(hw, &recovered);
    if (drain) {
        sc2_quarantine(hw, drain);
        ret = -EIO;
        goto unlock;
    }
    if (!ret && (recovered || atomic_read(&ctx->aborted)))
        ret = recovered ? -EIO : -ECANCELED;
    dma_rmb();
    if (!sc2_surface_guards(job->output.destination))
        ret = -EIO;
    if (!ret) {
        const struct v4l2_ctrl_av1_frame *f = job->syntax->frame;
        st->slot[st->current_slot].valid = true;
        for (i = 0; i < 8; i++)
            if (f->refresh_frame_flags & BIT(i))
                st->ref_map[i] = st->current_slot;
        st->sequence_control = *job->syntax->sequence;
        st->initialized = true;
        job->output.destination->valid = true;
        owner->surface.valid = true;
    }
    sc2_power_off(hw);
unpin:
    sc2_unpin_output(hw);
unlock:
    /* Validation has rebound logical slots even if a later preparation step
     * fails. Never retain that partial state, especially an attempted key
     * frame which retired its old map. Quarantine still owns live DMA memory.
     */
    if (ret && state_prepared && !hw->fatal)
        sc2_av1_reset(ctx);
    mutex_unlock(&hw->lock);
    return ret;
}
