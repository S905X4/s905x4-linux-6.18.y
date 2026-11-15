/* SPDX-License-Identifier: GPL-2.0-only */
/* SC2 VP9 host protocol, derived from Amlogic vvp9 and meson codec_vp9.
 * Firmware parses the compressed/uncompressed headers; host owns every PFN,
 * reference, probability bank, presentation and DMA lifetime. No HEVC actions.
 * Included by meson_sc2_hw.c so there is exactly one DOS resource owner.
 */
#include "meson_sc2_vp9_api.h"

#define V9_SLOTS 24
#define V9_WORK_SIZE 0x200000
/* Conservative 4K layout from codec_vp9; MV storage is per CAPTURE object. */
enum {
 V9_IPP = 0, V9_RPS = 0x64000, V9_PPS = 0x65800,
 V9_SWAP = 0x6a000, V9_SWAP2 = 0x6a800, V9_SCALE = 0x6b000,
 V9_DBLK_PARA = 0x73000, V9_DBLK_DATA = 0xf3000,
 V9_SEG = 0x173000, V9_PROB = 0x180800, V9_COUNT = 0x185800,
 V9_VBH = 0x188800, V9_ABOVE = 0x18d800,
 V9_RPM = 0x19d800, V9_LMEM = 0x19e000, V9_MAP = 0x1a0000,
};
union sc2_vp9_rpm {
 u16 words[0x400];
 struct {
  u16 profile, show_existing_frame, frame_to_show_idx, frame_type;
  u16 show_frame, error_resilient_mode, intra_only, display_size_present;
  u16 reset_frame_context, refresh_frame_flags, width, height;
  u16 display_width, display_height, ref_info, same_frame_size;
  u16 mode_ref_delta_enabled, ref_deltas[4], mode_deltas[2];
  u16 filter_level, sharpness_level, bit_depth, seg_quant_info[8];
  u16 seg_enabled, seg_abs_delta, seg_lf_info[8];
 } p;
};
struct sc2_vp9_picture {
 struct meson_sc2_surface *surface;
 u64 timestamp;
 unsigned int outstanding;
 bool key, intra, show;
};
struct sc2_vp9_state {
 void *work;
 dma_addr_t dma;
 union sc2_vp9_rpm rpm;
 struct sc2_vp9_picture slot[V9_SLOTS];
 s8 refs[8], previous;
 struct { u8 slot; u64 timestamp; } pending[32];
 u8 head, tail, pending_count;
 u32 count;
 bool need_key;
};
static void sc2_vp9_drop_slot(struct meson_sc2_hevc_context *ctx, unsigned int n)
{
 struct sc2_vp9_picture *p = &ctx->vp9->slot[n];
 struct sc2_surface_owner *owner;

 if (!p->surface)
  return;
 owner = container_of(p->surface, struct sc2_surface_owner, surface);
 memset(p, 0, sizeof(*p));
 sc2_surface_owner_put(ctx->hw, owner);
}

static void sc2_vp9_reset(struct meson_sc2_hevc_context *ctx)
{
 struct sc2_vp9_state *st = ctx->vp9;
 unsigned int i;
 if (!st)
  return;
 for (i = 0; i < V9_SLOTS; i++)
  sc2_vp9_drop_slot(ctx, i);
 memset(st->refs, -1, sizeof(st->refs));
 st->previous = -1;
 st->head = st->tail = st->pending_count = 0;
 st->count = 0;
 st->need_key = true;
 memset(st->work, 0, V9_WORK_SIZE);
}
static void sc2_vp9_release(struct meson_sc2_hevc_context *ctx)
{
 unsigned int i;

 if (!ctx->vp9)
  return;
 for (i = 0; i < V9_SLOTS; i++)
  sc2_vp9_drop_slot(ctx, i);
 kfree(ctx->vp9);
 ctx->vp9 = NULL;
}
static int sc2_vp9_alloc(struct meson_sc2_hevc_context *ctx)
{
 struct sc2_vp9_state *st;
 if (ctx->vp9)
  return 0;
 st = kzalloc(sizeof(*st), GFP_KERNEL);
 if (!st)
  return -ENOMEM;
 BUILD_BUG_ON(V9_WORK_SIZE > SC2_WORK_SIZE);
 st->work = ctx->work;
 st->dma = ctx->work_dma;
 ctx->vp9 = st;
 sc2_vp9_reset(ctx);
 return 0;
}
static bool sc2_vp9_slot_busy(struct sc2_vp9_state *st, unsigned int n)
{
 unsigned int i;
 if (st->slot[n].outstanding || st->previous == n)
  return true;
 for (i = 0; i < 8; i++)
  if (st->refs[i] == n)
   return true;
 return false;
}
static void sc2_vp9_reclaim(struct meson_sc2_hevc_context *ctx)
{
 unsigned int i;

 for (i = 0; i < V9_SLOTS; i++)
  if (!sc2_vp9_slot_busy(ctx->vp9, i))
   sc2_vp9_drop_slot(ctx, i);
}

int meson_sc2_vp9_capture_restart(struct meson_sc2_hevc_context *ctx)
{
 int ret = 0;

 mutex_lock(&ctx->hw->lock);
 if (ctx->hw->fatal)
  ret = -EIO;
 else
  atomic_set(&ctx->aborted, 0);
 mutex_unlock(&ctx->hw->lock);
 return ret;
}

bool meson_sc2_vp9_surface_busy(struct meson_sc2_hevc_context *ctx,
                               struct meson_sc2_surface *s)
{
 unsigned int i;
 bool busy = false;
 mutex_lock(&ctx->hw->lock);
 if (ctx->vp9)
  for (i = 0; i < V9_SLOTS; i++)
   if (ctx->vp9->slot[i].surface &&
       ctx->vp9->slot[i].surface->cpu == s->cpu)
    busy = sc2_vp9_slot_busy(ctx->vp9, i);
 mutex_unlock(&ctx->hw->lock);
 return busy;
}
int meson_sc2_vp9_put_output(struct meson_sc2_hevc_context *ctx,
                            struct meson_sc2_surface *s)
{
 unsigned int i;
 int ret = -ENOENT;
 mutex_lock(&ctx->hw->lock);
 if (ctx->vp9)
  for (i = 0; i < V9_SLOTS; i++)
   if (ctx->vp9->slot[i].surface &&
       ctx->vp9->slot[i].surface->cpu == s->cpu &&
       ctx->vp9->slot[i].outstanding) {
    ctx->vp9->slot[i].outstanding--;
    if (!sc2_vp9_slot_busy(ctx->vp9, i))
     sc2_vp9_drop_slot(ctx, i);
    ret = 0;
    break;
   }
 mutex_unlock(&ctx->hw->lock);
 return ret;
}
int meson_sc2_vp9_dequeue(struct meson_sc2_hevc_context *ctx, bool drain,
                         struct meson_sc2_vp9_result *result)
{
 struct sc2_vp9_state *st;
 struct sc2_vp9_picture *p;
 int ret = -EAGAIN;
 mutex_lock(&ctx->hw->lock);
 st = ctx->vp9;
 if (st && st->pending_count) {
  p = &st->slot[st->pending[st->head].slot];
  *result = (struct meson_sc2_vp9_result) {
   .surface = p->surface, .timestamp = st->pending[st->head].timestamp,
   .width = p->surface->width, .height = p->surface->height,
   .depth = p->surface->depth, .poc = p->surface->poc,
  };
  st->head = (st->head + 1) % ARRAY_SIZE(st->pending);
  st->pending_count--;
  ret = 0;
 }
 mutex_unlock(&ctx->hw->lock);
 return ret;
}
static void sc2_vp9_parser(struct meson_sc2_hevc_context *ctx, size_t size)
{
 struct meson_sc2_hw *hw = ctx->hw;
 u32 w = ctx->vp9->dma, in = ctx->input_dma;
 unsigned int i;
 sc2_write(hw, HEVCD_IPP_LINEBUFF_BASE, w + V9_IPP);
 sc2_write(hw, HEVC_ASSIST_SCRATCH_1, w + V9_RPM);
 sc2_write(hw, HEVC_ASSIST_SCRATCH_2, w + V9_RPS);
 sc2_write(hw, HEVC_ASSIST_SCRATCH_3, 0);
 sc2_write(hw, HEVC_ASSIST_SCRATCH_5, w + V9_PPS);
 sc2_write(hw, HEVC_ASSIST_SCRATCH_7, w + V9_SWAP);
 sc2_write(hw, HEVC_ASSIST_SCRATCH_8, w + V9_SWAP2);
 sc2_write(hw, HEVC_ASSIST_SCRATCH_9, w + V9_PROB);
 sc2_write(hw, HEVC_ASSIST_SCRATCH_A, w + V9_COUNT);
 sc2_write(hw, HEVC_ASSIST_SCRATCH_B, w + V9_SEG);
 sc2_write(hw, HEVC_ASSIST_SCRATCH_D, w + V9_SCALE);
 sc2_write(hw, HEVC_ASSIST_SCRATCH_F, w + V9_LMEM);
 sc2_write(hw, HEVC_ASSIST_MMU_MAP_ADDR, w + V9_MAP);
 sc2_write(hw, HEVC_SAO_MMU_VH0_ADDR, w + V9_VBH);
 sc2_write(hw, HEVC_SAO_MMU_VH1_ADDR, w + V9_VBH + 0x2800);
 sc2_write(hw, HEVC_DBLK_CFG3, 0x404010);
 sc2_write(hw, HEVC_DBLK_CFG4, w + V9_DBLK_PARA);
 sc2_write(hw, HEVC_DBLK_CFG5, w + V9_DBLK_DATA);
 sc2_write(hw, HEVC_DBLK_CFGE, w + V9_DBLK_PARA);
 sc2_write(hw, HEVC_STREAM_CONTROL, 0x0eb14070);
 sc2_write(hw, HEVC_STREAM_START_ADDR, in);
 sc2_write(hw, HEVC_STREAM_END_ADDR, in + SC2_INPUT_SIZE);
 sc2_write(hw, HEVC_STREAM_RD_PTR, in);
 sc2_write(hw, HEVC_STREAM_WR_PTR, in + ALIGN(size + SC2_INPUT_PADDING, 64));
 sc2_write(hw, HEVC_PARSER_INT_CONTROL,
  (3U << 29) | BIT(24) | BIT(22) | BIT(7) | BIT(4) | BIT(0));
 sc2_write(hw, HEVC_SHIFT_STARTCODE, 1);
 sc2_write(hw, HEVC_SHIFT_STATUS, 1);
 sc2_write(hw, HEVC_SHIFT_CONTROL,
  BIT(10) | BIT(9) | (3 << 6) | (2 << 4) | (3 << 1) | 1);
 sc2_write(hw, HEVC_CABAC_CONTROL, 1);
 sc2_write(hw, HEVC_PARSER_CORE_CONTROL, 1);
 sc2_write(hw, HEVC_ASSIST_SCRATCH_0, 0);
 sc2_write(hw, HEVC_IQIT_SCALELUT_WR_ADDR, 0);
 for (i = 0; i < 1024; i++)
  sc2_write(hw, HEVC_IQIT_SCALELUT_DATA, 0);
 sc2_write(hw, HEVC_PARSER_CMD_WRITE, BIT(16));
 for (i = 0; i < ARRAY_SIZE(sc2_parser_commands); i++)
  sc2_write(hw, HEVC_PARSER_CMD_WRITE, sc2_parser_commands[i]);
 sc2_write(hw, HEVC_PARSER_CMD_SKIP_0, 0x0000090b);
 sc2_write(hw, HEVC_PARSER_CMD_SKIP_1, 0x1b14140f);
 sc2_write(hw, HEVC_PARSER_CMD_SKIP_2, 0x001b1910);
 sc2_write(hw, HEVC_PARSER_IF_CONTROL, BIT(5) | BIT(2) | BIT(0));
 sc2_write(hw, HEVCD_IPP_TOP_CNTL, 1);
 sc2_write(hw, HEVCD_IPP_TOP_CNTL, 2);
 sc2_write(hw, HEVC_ASSIST_SCRATCH_E, 1);
 sc2_write(hw, SC2_MBOX_CLEAR, 1);
 sc2_write(hw, SC2_MBOX_MASK, 1);
 sc2_write(hw, HEVC_PSCALE_CTRL, 0);
 sc2_write(hw, HEVC_ASSIST_SCRATCH_G, 0);
 sc2_write(hw, HEVC_ASSIST_SCRATCH_H, 0);
 sc2_write(hw, HEVC_ASSIST_SCRATCH_I, 8);
 /* Vendor no-head frame mode consumes a raw VP9 frame, not AMLV packets. */
 sc2_write(hw, HEVC_ASSIST_SCRATCH_J, 0x80000004);
 sc2_write(hw, HEVC_ASSIST_SCRATCH_K, 0);
 sc2_write(hw, HEVC_ASSIST_SCRATCH_L, 0);
 sc2_write(hw, HEVC_ASSIST_SCRATCH_M, ctx->vp9->count);
 sc2_write(hw, HEVC_ASSIST_SCRATCH_N, size);
 sc2_write(hw, HEVC_SHIFT_BYTE_COUNT, 0);
 sc2_write(hw, HEVC_STREAM_CONTROL, sc2_read(hw, HEVC_STREAM_CONTROL) | 1);
 sc2_write(hw, HEVC_STREAM_FIFO_CTL, sc2_read(hw, HEVC_STREAM_FIFO_CTL) | BIT(29));
 sc2_write(hw, HEVC_ASSIST_SCRATCH_0, 0xff);
}
/* Vendor loop-filter table derivation, including segmentation and deltas. */
static void sc2_vp9_filter(struct meson_sc2_hw *hw, union sc2_vp9_rpm *r)
{
 unsigned int i, seg, ref, mode;
 u8 lim[64], mb[64], level[8][4][2] = {};
 int sharp = r->p.sharpness_level, base = r->p.filter_level;
 int scale = 1 << (base >> 5), data, v;
 for (i = 0; i < 64; i++) {
  v = i >> ((sharp > 0) + (sharp > 4));
  if (sharp)
   v = min(v, 9 - sharp);
  lim[i] = max(v, 1);
  mb[i] = 2 * (i + 2) + lim[i];
 }
 for (i = 0; i < 32; i++)
  sc2_write(hw, HEVC_DBLK_CFG9, ((lim[2*i+1] << 8 | mb[2*i+1]) << 16) |
   (lim[2*i] << 8 | mb[2*i]));
 sc2_write(hw, HEVC_DBLK_CFGB, (3 << 14) | (3 << 12) | (3 << 10) |
  BIT(8) | (r->p.width >= 1280 ? BIT(4) : 0) | 1);
 for (seg = 0; seg < 8; seg++) {
  v = base;
  if (r->p.seg_enabled && (r->p.seg_lf_info[seg] & BIT(15))) {
   data = r->p.seg_lf_info[seg] & 63;
   if (r->p.seg_lf_info[seg] & BIT(8))
    data = -data;
   v = clamp(r->p.seg_abs_delta ? data : base + data, 0, 63);
  }
  for (ref = 0; ref < 4; ref++)
   for (mode = 0; mode < 2; mode++)
    level[seg][ref][mode] = clamp(v + (r->p.mode_ref_delta_enabled ?
     (s8)r->p.ref_deltas[ref] * scale +
     (ref ? (s8)r->p.mode_deltas[mode] * scale : 0) : 0), 0, 63);
 }
 for (i = 0; i < 16; i++)
  sc2_write(hw, HEVC_DBLK_CFGA, !base ? 0 :
   level[i/2][0][i&1] | level[i/2][1][i&1] << 8 |
   level[i/2][2][i&1] << 16 | level[i/2][3][i&1] << 24);
}
static int sc2_vp9_adapt(struct meson_sc2_hevc_context *ctx, int slot_index)
{
 struct sc2_vp9_state *st = ctx->vp9;
 u32 status = sc2_read(ctx->hw, HEVC_ASSIST_SCRATCH_3), bank;
 unsigned int *prev, *prob, *count;
 if ((status & 0xff) != 0xfd)
  return 0;
 bank = status >> 8;
 if (bank > 3 || slot_index < 0)
  return -EPROTO;
 dma_rmb();
 prev = st->work + V9_PROB + bank * 0x1000;
 prob = st->work + V9_PROB + 0x4000;
 count = st->work + V9_COUNT;
 meson_sc2_vp9_adapt_prob(st->previous < 0 || st->slot[st->previous].key,
  st->slot[slot_index].key || st->slot[slot_index].intra, bank, prev, prob, count);
 memcpy(prev, prob, 0xf80);
 dma_wmb();
 sc2_write(ctx->hw, HEVC_ASSIST_SCRATCH_3, 0);
 return 0;
}
static int sc2_vp9_header(struct meson_sc2_hevc_context *ctx,
                          const struct meson_sc2_vp9_job *job,
                          struct sc2_surface_owner *owner,
                          struct meson_sc2_vp9_result *result, int *slot_index)
{
 struct sc2_vp9_state *st = ctx->vp9;
 struct meson_sc2_hw *hw = ctx->hw;
 struct meson_sc2_surface *s = owner ? &owner->surface : NULL, *ref;
 union sc2_vp9_rpm *r = &st->rpm;
 const __le16 *raw = st->work + V9_RPM;
 __le32 *map = st->work + V9_MAP;
 unsigned int i, j, pages, refs[3], scale = 0;
 int n = -1;
 u32 value, body;
 dma_rmb();
 for (i = 0; i < ARRAY_SIZE(r->words); i++)
  r->words[i] = le16_to_cpu(raw[(i & ~3) + 3 - (i & 3)]);
 /* Supported repeats are resolved before entering the firmware. */
 if (r->p.show_existing_frame)
  return -EBADMSG;
 if (r->p.frame_type && !r->p.intra_only) {
  for (i = 0; i < 3; i++) {
   if (!(r->p.same_frame_size & BIT(2 - i)))
    continue;
   j = (r->p.ref_info >> ((2 - i) * 4 + 1)) & 7;
   if (st->refs[j] < 0)
    return -EBADMSG;
   ref = st->slot[st->refs[j]].surface;
   if (!ref || !ref->valid)
    return -EBADMSG;
   r->p.width = ref->width;
   r->p.height = ref->height;
   break;
  }
 }
 result->width = r->p.width;
 result->height = r->p.height;
 result->depth = r->p.bit_depth;
 if (!r->p.width || !r->p.height || r->p.width > 3840 || r->p.height > 2160 ||
  (r->p.profile != 0 && r->p.profile != 2) ||
  r->p.bit_depth != (r->p.profile ? 10 : 8) ||
  r->p.filter_level > 63 || r->p.sharpness_level > 7 ||
  r->p.frame_type > 1 || r->p.refresh_frame_flags > 255)
  return -EOPNOTSUPP;
 if (!s || s->width != r->p.width || s->height != r->p.height || s->depth != r->p.bit_depth)
  return -EPIPE;
 if (st->need_key && r->p.frame_type && !r->p.intra_only)
  return -EBADMSG;
 for (i = 0; i < V9_SLOTS; i++) {
  if (st->slot[i].surface == s && sc2_vp9_slot_busy(st, i))
   return -EAGAIN;
  if (n < 0 && (!st->slot[i].surface || !sc2_vp9_slot_busy(st, i)))
   n = i;
 }
 if (n < 0)
  return -ENOSPC;
 sc2_vp9_drop_slot(ctx, n);
 owner->references++;
 st->slot[n] = (struct sc2_vp9_picture) {
  .surface = s, .timestamp = job->timestamp, .key = !r->p.frame_type,
  .intra = r->p.intra_only, .show = r->p.show_frame,
 };
 *slot_index = n;
 pages = DIV_ROUND_UP(s->body_size, SZ_4K);
 for (i = 0; i < SC2_MAP_ENTRIES; i++)
  map[i] = cpu_to_le32((s->dma + s->body_offset + (i < pages ? i * SZ_4K : 0)) >> 12);
 sc2_write(hw, HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR, BIT(1) | BIT(2));
 for (i = 0; i < V9_SLOTS; i++)
  sc2_write(hw, HEVCD_MPP_ANC2AXI_TBL_DATA,
   (st->slot[i].surface ? st->slot[i].surface : s)->dma >> 5);
 sc2_write(hw, HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR, 1);
 sc2_write(hw, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, 1);
 for (i = 0; i < 32; i++)
  sc2_write(hw, HEVCD_MPP_ANC_CANVAS_DATA_ADDR, 0);
 if (r->p.frame_type && !r->p.intra_only) {
  for (i = 0; i < 3; i++) {
   j = (r->p.ref_info >> ((2 - i) * 4 + 1)) & 7;
   if (st->refs[j] < 0 || !st->slot[st->refs[j]].surface->valid)
    return -EBADMSG;
   refs[i] = st->refs[j];
  }
  for (j = 0; j < 2; j++) {
   sc2_write(hw, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, (j * 16 << 8) | 1);
   for (i = 0; i < 3; i++)
    sc2_write(hw, HEVCD_MPP_ANC_CANVAS_DATA_ADDR, refs[i] * 0x010101);
  }
  sc2_write(hw, VP9D_MPP_REFINFO_TBL_ACCCONFIG, BIT(2));
  for (i = 0; i < 3; i++) {
   ref = st->slot[refs[i]].surface;
   if (ref->depth != s->depth)
    return -EBADMSG;
   if (ref->width != s->width || ref->height != s->height)
    scale |= BIT(i);
   sc2_write(hw, VP9D_MPP_REFINFO_DATA, ref->width);
   sc2_write(hw, VP9D_MPP_REFINFO_DATA, ref->height);
   sc2_write(hw, VP9D_MPP_REFINFO_DATA, (ref->width << 14) / s->width);
   sc2_write(hw, VP9D_MPP_REFINFO_DATA, (ref->height << 14) / s->height);
   /* SC2 paged references derive lengths from scatter headers. */
   sc2_write(hw, VP9D_MPP_REFINFO_DATA, 0);
  }
  sc2_write(hw, VP9D_MPP_REF_SCALE_ENBL, scale);
 }
 value = sc2_read(hw, HEVC_MPRED_CTRL4) & ~BIT(6);
 if (st->previous >= 0 && r->p.frame_type && !r->p.intra_only) {
  struct sc2_vp9_picture *p = &st->slot[st->previous];
  ref = p->surface;
  if (!r->p.error_resilient_mode && ref->width == s->width &&
   ref->height == s->height && p->show && !p->key && !p->intra)
   value |= BIT(6);
  sc2_write(hw, HEVC_MPRED_MV_RD_START_ADDR, ref->mv_dma);
  sc2_write(hw, HEVC_MPRED_MV_RPTR, ref->mv_dma);
  sc2_write(hw, HEVC_MPRED_MV_RD_END_ADDR, ref->mv_dma + ref->mv_size);
 }
 sc2_write(hw, HEVC_MPRED_CTRL3, 0x24122412);
 sc2_write(hw, HEVC_MPRED_CTRL4, value);
 sc2_write(hw, HEVC_MPRED_ABV_START_ADDR, st->dma + V9_ABOVE);
 sc2_write(hw, HEVC_MPRED_MV_WR_START_ADDR, s->mv_dma);
 sc2_write(hw, HEVC_MPRED_MV_WPTR, s->mv_dma);
 sc2_write(hw, HEVCD_MCRCC_CTL1, 2);
 if (r->p.frame_type && !r->p.intra_only) {
  sc2_write(hw, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, BIT(1));
  value = sc2_read(hw, HEVCD_MPP_ANC_CANVAS_DATA_ADDR) & 0xffff;
  sc2_write(hw, HEVCD_MCRCC_CTL2, value | value << 16);
  value = sc2_read(hw, HEVCD_MPP_ANC_CANVAS_DATA_ADDR) & 0xffff;
  sc2_write(hw, HEVCD_MCRCC_CTL3, value | value << 16);
  sc2_write(hw, HEVCD_MCRCC_CTL1, 0xff0);
 } else {
  sc2_write(hw, HEVCD_MCRCC_CTL1, 0);
 }
 sc2_write(hw, HEVC_PARSER_PICTURE_SIZE, s->height << 16 | s->width);
 sc2_write(hw, HEVCD_MPP_DECOMP_CTL1, BIT(4));
 body = DIV_ROUND_UP(s->width, 64) * DIV_ROUND_UP(s->height, 32) *
  (s->depth == 10 ? 4096 : 3200);
 sc2_write(hw, HEVC_CM_BODY_LENGTH, body);
 sc2_write(hw, HEVC_CM_HEADER_OFFSET, body);
 sc2_write(hw, HEVC_CM_HEADER_LENGTH, s->header_size);
 sc2_write(hw, HEVC_CM_HEADER_START_ADDR, s->dma);
 sc2_write(hw, SC2_SAO_CTRL9, sc2_read(hw, SC2_SAO_CTRL9) | 1);
 sc2_write(hw, HEVC_SAO_CTRL5, BIT(10) | (s->depth == 8 ? BIT(9) : 0));
 sc2_write(hw, HEVC_SAO_CTRL1, (sc2_read(hw, HEVC_SAO_CTRL1) & ~3) | 2);
 sc2_write(hw, HEVC_SAO_Y_START_ADDR, U32_MAX);
 sc2_write(hw, HEVC_SAO_C_START_ADDR, U32_MAX);
 sc2_write(hw, HEVC_SAO_Y_LENGTH, 0);
 sc2_write(hw, HEVC_SAO_C_LENGTH, 0);
 sc2_vp9_filter(hw, r);
 dma_wmb();
 sc2_write(hw, HEVC_ASSIST_SCRATCH_0, 5);
 return 0;
}
static int sc2_vp9_run(struct meson_sc2_hevc_context *ctx,
                       const struct meson_sc2_vp9_job *job,
                       struct sc2_surface_owner *owner,
                       struct meson_sc2_vp9_result *result, int *slot_index)
{
 struct meson_sc2_hw *hw = ctx->hw;
 unsigned long deadline = jiffies + SC2_JOB_TIMEOUT, remaining;
 u32 seq, status;
 bool header = false;
 int ret;
 sc2_vp9_parser(ctx, job->size);
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
  seq = smp_load_acquire(&hw->irq_sequence);
  status = sc2_read(hw, HEVC_ASSIST_SCRATCH_0);
  ret = sc2_vp9_adapt(ctx, *slot_index);
  if (ret)
   return ret;
  if (status == 0xf0) {
   if (header)
    return -EPROTO;
   ret = sc2_vp9_header(ctx, job, owner, result, slot_index);
   if (ret)
    return ret;
   header = true;
   continue;
  }
  if (header && status == 0x0a) {
   if ((sc2_read(hw, HEVC_PARSER_LCU_START) & 0xffffff) !=
       DIV_ROUND_UP(result->width, 64) * DIV_ROUND_UP(result->height, 64) - 1 ||
       (sc2_read(hw, SC2_SAO_MMU_STATUS) >> 16) >
       DIV_ROUND_UP(job->output.destination->body_size, SZ_4K))
    return -EIO;
   return 0;
  }
  if (status == 0x0b || (status >= 0x20 && status <= 0x24) || status == 0xf2)
   return -EBADMSG;
  remaining = deadline - jiffies;
  if ((long)remaining <= 0)
   return -ETIMEDOUT;
  wait_event_timeout(hw->wait, atomic_read(&ctx->aborted) ||
   smp_load_acquire(&hw->irq_sequence) != seq, remaining);
 }
}
/* An orphaned CAPTURE reference still owns its original FBC and MV storage.
 * A repeat needs a buffer in the current queue. Relocate only the compressed
 * display bytes; the original DPB/MV and previous-picture identity stay put.
 */
static int sc2_vp9_display_detached(struct meson_sc2_hevc_context *ctx,
                                  const struct meson_sc2_vp9_job *job,
                                  struct sc2_surface_owner *owner, int *slot_index)
{
 struct sc2_vp9_state *st = ctx->vp9;
 struct sc2_vp9_picture *picture = &st->slot[*slot_index];
 struct meson_sc2_surface *src = picture->surface, *dst = &owner->surface;
 struct sc2_surface_owner *source_owner;
 const __le32 *source_header = src->cpu;
 __le32 *target_header = dst->cpu;
 dma_addr_t source_body = src->dma + src->body_offset;
 dma_addr_t target_body = dst->dma + dst->body_offset;
 u32 columns = DIV_ROUND_UP(src->width, 64);
 u32 entries = columns * DIV_ROUND_UP(src->height, 64) * 32;
 u32 i, x, y, word, offset, used = 0;
 dma_addr_t page;
 int n;

 source_owner = container_of(src, struct sc2_surface_owner, surface);
 if (!source_owner->detached)
  return 0;
 if (src->dma == dst->dma || src->size != dst->size ||
     src->width != dst->width || src->height != dst->height ||
     src->depth != dst->depth || src->body_offset != dst->body_offset ||
     src->body_size != dst->body_size ||
     src->header_alloc_size != dst->header_alloc_size ||
     entries * sizeof(*source_header) > src->header_alloc_size)
  return -EINVAL;
 if (!sc2_surface_guards(src) ||
     memchr_inv(source_header + entries, 0,
                src->header_alloc_size - entries * sizeof(*source_header)))
  return -EIO;
 for (n = 0; n < V9_SLOTS; n++)
  if (!sc2_vp9_slot_busy(st, n))
   break;
 if (n == V9_SLOTS)
  return -ENOSPC;
 /* Rebase every populated tile entry, including edge padding, so no address
  * can retain the old allocation. Preserve mode, subpage offset and flags.
  */
 for (i = 0; i < entries; i++) {
  word = le32_to_cpu(source_header[i]);
  if (word) {
   page = (dma_addr_t)(word & GENMASK(29, 10)) << 2;
   if (page < source_body || page - source_body > src->body_size - SZ_4K)
    return -EBADMSG;
   offset = page - source_body;
   used = max(used, offset + (u32)SZ_4K);
   word = (word & ~GENMASK(29, 10)) | ((target_body + offset) >> 2);
  }
  target_header[i] = cpu_to_le32(word);
 }
 for (y = 0; y < DIV_ROUND_UP(src->height, 4); y++)
  for (x = 0; x < DIV_ROUND_UP(src->width, 32); x++) {
   i = ((y / 16) * columns + x / 2) * 32 + (y % 16) * 2 + x % 2;
   if (!source_header[i])
    return -EBADMSG;
  }
 memset(target_header + entries, 0,
        dst->header_alloc_size - entries * sizeof(*target_header));
 memcpy(dst->cpu + dst->body_offset, src->cpu + src->body_offset, used);
 dma_wmb();
 sc2_vp9_drop_slot(ctx, n);
 owner->references++;
 st->slot[n] = *picture;
 st->slot[n].surface = dst;
 st->slot[n].outstanding = 0;
 dst->valid = job->output.destination->valid = true;
 dst->poc = job->output.destination->poc = src->poc;
 *slot_index = n;
 return 0;
}

static void sc2_vp9_queue_output(struct sc2_vp9_state *st, int slot, u64 timestamp)
{
 st->slot[slot].outstanding++;
 st->pending[st->tail].slot = slot;
 st->pending[st->tail].timestamp = timestamp;
 st->tail = (st->tail + 1) % ARRAY_SIZE(st->pending);
 st->pending_count++;
}

/* show_existing has no coded pixels or new MV/probability state. SC2's
 * no-head firmware consumes it without a header interrupt; do not submit it
 * as a decode or manufacture PIC_DONE. Only a completed reference may show.
 */
static int sc2_vp9_repeat(struct meson_sc2_hevc_context *ctx,
                         const struct meson_sc2_vp9_job *job,
                         struct sc2_surface_owner *owner,
                         struct meson_sc2_vp9_result *result)
{
 struct sc2_vp9_state *st = ctx->vp9;
 const u8 prefix = *(const u8 *)job->data;
 struct meson_sc2_surface *ref, *dst = owner ? &owner->surface : NULL;
 int slot = st->refs[prefix & 7], ret;

 if (slot < 0)
  return -EBADMSG;
 ref = st->slot[slot].surface;
 if (!ref || !ref->valid || ref->depth != (prefix & BIT(4) ? 10 : 8))
  return -EBADMSG;
 result->width = ref->width;
 result->height = ref->height;
 result->depth = ref->depth;
 if (!dst || dst->width != ref->width || dst->height != ref->height ||
     dst->depth != ref->depth)
  return -EPIPE;
 ret = sc2_vp9_display_detached(ctx, job, owner, &slot);
 if (ret)
  return ret;
 if (!sc2_surface_guards(st->slot[slot].surface))
  return -EIO;
 sc2_vp9_queue_output(st, slot, job->timestamp);
 result->surface = st->slot[slot].surface;
 result->timestamp = job->timestamp;
 result->poc = result->surface->poc;
 result->hidden = false;
 return 0;
}

int meson_sc2_vp9_decode(struct meson_sc2_hevc_context *ctx,
                         const struct meson_sc2_vp9_job *job,
                         struct meson_sc2_vp9_result *result)
{
 struct meson_sc2_hw *hw;
 struct sc2_vp9_state *st;
 struct sc2_surface_owner *owner = NULL;
 int ret, drain, slot_index = -1;
 s32 status = -EIO;
 bool recovered;
 unsigned int i, refresh;
 if (!ctx || !job || !result || !job->data || !job->size ||
  job->size > SC2_INPUT_SIZE - SC2_INPUT_PADDING - 64 ||
  (!!job->output.destination != !!job->output.output_buffer))
  return -EINVAL;
 memset(result, 0, sizeof(*result));
 result->hidden = true;
 hw = ctx->hw;
 mutex_lock(&hw->lock);
 if (hw->fatal || atomic_read(&ctx->aborted)) {
  ret = hw->fatal ? -EIO : -ECANCELED;
  goto unlock;
 }
 ret = sc2_vp9_alloc(ctx);
 if (ret)
  goto unlock;
 st = ctx->vp9;
 if (st->pending_count == ARRAY_SIZE(st->pending)) {
  ret = -ENOSPC;
  goto unlock;
 }
 if (job->output.destination) {
  owner = sc2_owner(hw, job->output.destination);
  if (!owner || (owner->context && owner->context != ctx) ||
   !owner->capture_buffer || owner->capture_buffer != job->output.output_buffer ||
   job->output.output_size < owner->surface.size ||
   owner->surface.mv_size < ALIGN(
    DIV_ROUND_UP(owner->surface.width, 64) * DIV_ROUND_UP(owner->surface.height, 64) * 0x240 +
    DIV_ROUND_UP(owner->surface.width, 64) * 16 *
     ((DIV_ROUND_UP(owner->surface.height, 64) >> 3) +
      (DIV_ROUND_UP(owner->surface.height, 64) & 7)), SZ_64K)) {
   ret = -EINVAL;
   goto unlock;
  }
  owner->context = ctx;
 }
 /* Profiles 0/2 use the same eight-bit repeat header: marker 10,
  * profile-low 0, show_existing 1, followed by the three-bit reference.
  */
 if ((*(const u8 *)job->data & 0xe8) == 0x88) {
  ret = sc2_vp9_repeat(ctx, job, owner, result);
  if (ret && ret != -EPIPE) {
   st->need_key = true;
   memset(st->refs, -1, sizeof(st->refs));
   st->previous = -1;
  }
  sc2_vp9_reclaim(ctx);
  goto unlock;
 }
 memcpy(ctx->input, job->data, job->size);
 memset(ctx->input + job->size, 0, SC2_INPUT_PADDING + 64);
 ret = owner ? sc2_pin_output(hw, &job->output) : 0;
 if (ret)
  goto unlock;
 ret = sc2_power_on(hw);
 if (ret)
  goto unpin;
 ret = sc2_prepare(hw);
 if (!ret) {
  ret = meson_sm_call(hw->firmware, SM_VIDEO_FW_LOAD, &status, 19, 0, 0, 0, 0);
  if (!ret && status)
   ret = -EIO;
 }
 if (!ret)
  ret = sc2_vp9_run(ctx, job, owner, result, &slot_index);
 drain = sc2_drain(hw, &recovered);
 if (drain) {
  sc2_quarantine(hw, drain);
  ret = -EIO;
  goto unlock;
 }
 if (!ret && (recovered || atomic_read(&ctx->aborted)))
  ret = -EIO;
 dma_rmb();
 if (owner && (!sc2_surface_guards(job->output.destination) ||
  memchr_inv(owner->private_mv + owner->surface.mv_size,
   SC2_GUARD_BYTE, SC2_GUARD_SIZE)))
  ret = -EIO;
 if (!ret) {
  owner->surface.valid = job->output.destination->valid = true;
  owner->surface.poc = job->output.destination->poc = st->count++;
  result->surface = job->output.destination;
  result->timestamp = job->timestamp;
  result->poc = job->output.destination->poc;
  refresh = !st->rpm.p.frame_type ? 255 : st->rpm.p.refresh_frame_flags;
  for (i = 0; i < 8; i++)
   if (refresh & BIT(i))
    st->refs[i] = slot_index;
  st->previous = slot_index;
  st->need_key = false;
  if (st->rpm.p.show_frame) {
   sc2_vp9_queue_output(st, slot_index, job->timestamp);
   result->hidden = false;
  }
 } else if (ret != -EPIPE && ret != -EAGAIN) {
  /* No failed decode may become a reference; seek requires a key frame. */
  st->need_key = true;
  memset(st->refs, -1, sizeof(st->refs));
  st->previous = -1;
 }
 sc2_vp9_reclaim(ctx);
 sc2_power_off(hw);
unpin:
 sc2_unpin_output(hw);
unlock:
 mutex_unlock(&hw->lock);
 return ret;
}
