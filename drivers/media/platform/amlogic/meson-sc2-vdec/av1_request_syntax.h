/* SPDX-License-Identifier: GPL-2.0 */
#ifndef MESON_AV1_REQUEST_SYNTAX_H
#define MESON_AV1_REQUEST_SYNTAX_H

#include <linux/types.h>
#include <linux/v4l2-controls.h>

/* A snapshot of a firmware reference slot BEFORE this request. A valid slot
 * must describe the same retained decoded picture and parser/CDF/segment-map
 * history as firmware. frame is its resolved native control, not raw syntax.
 * Invalid slots have no frame; order_hint still records the parser slot value.
 */
struct meson_av1_syntax_reference {
	const struct v4l2_ctrl_av1_frame *frame;
	u64 timestamp;
	u32 order_hint;
	bool valid;
};

/* The native u8 context_update_tile_id loses bit 8 on 257..512-tile grids.
 * Preserve only this scalar from the parsed uncompressed header, not opaque
 * original headers. Required for those grids; low byte must match the control.
 * A native-only kernel caller has no source for this lost bit and must reject
 * such grids with -ENODATA until a genuine metadata transport is established.
 */
struct meson_av1_syntax_metadata {
	u16 context_update_tile_id;
};

struct meson_av1_syntax_request {
	const struct v4l2_ctrl_av1_sequence *sequence;
	const struct v4l2_ctrl_av1_frame *frame;
	/* Required when sequence film grain is present and frame show/showable. */
	const struct v4l2_ctrl_av1_film_grain *film_grain;
	/* Exactly tile_rows * tile_cols native entries, in raster order. */
	const struct v4l2_ctrl_av1_tile_group_entry *tiles;
	size_t num_tiles;
	/* Entries address ONLY coded tile bytes within this accessible OUTPUT span.
	 * Offsets may skip original headers, which are never read or trusted.
	 */
	const u8 *tile_data;
	size_t tile_data_size;
	/* Eight accessible immutable slot snapshots; NULL only for an intra frame
	 * which neither reads a primary reference nor signals ref_order_hint.
	 */
	const struct meson_av1_syntax_reference *references;
	size_t num_references;
	const struct meson_av1_syntax_metadata *metadata;
	/* First normalized request / actual sequence change only. Do not reset
	 * firmware or insert a changed sequence between dependent pictures.
	 */
	bool emit_sequence_header;
};

struct meson_av1_syntax_error {
	/* SIZE_MAX for sequence/frame/reference errors, otherwise tile index. */
	size_t tile;
	const char *reason;
};

/**
 * meson_av1_request_syntax() - controls-authoritative AV1 Main 8/10 4:2:0/mono
 *
 * Emits low-overhead, size-delimited optional SEQUENCE_HEADER, FRAME_HEADER,
 * and one TILE_GROUP containing every tile. No opaque header bytes are copied.
 * Supports hidden/inter/intra/switch frames, primary-reference CDF inheritance,
 * segmentation, uniform/nonuniform tiles, global motion, restoration, CDEF,
 * quantization, grain and superresolution. Retained CDFs, segment maps, motion
 * fields and reference backing are NOT reconstructible from header syntax:
 * caller must preserve/install the control-authoritative history in firmware.
 * Feed software comparison decoders normalized requests in decode order too.
 * Monochrome retains its sequence flag and emits only the applicable plane
 * syntax; absent chroma controls must be zero except native-derived restoration
 * sizes and qm_v inferred from the still-coded qm_u. Segmentation retains all
 * eight features per segment; lossless inference includes validated zero UV
 * deltas. Monochrome film-grain sequences/requests are unsupported. This syntax
 * support does not establish hardware monochrome output or neutral chroma.
 *
 * Native ABI omissions are canonicalized: full (not reduced-still) header;
 * no timing/decoder model/frame IDs (caller must validate source current,
 * reference and display IDs before resolving references via timestamps), one
 * unfiltered operating point, level 31, selectable screen tools/integer MV,
 * explicit seven reference indices and explicit frame size.
 * These choices preserve tile entropy and reference semantics, not original
 * packaging, timing, conformance level or missing-reference detection. Native
 * color range is retained; color description/chroma position are unspecified
 * and remain external display metadata. No source headers are required.
 * Original short-reference signaling and size-from-reference syntax are
 * replaced by resolved controls. Sequence/frame tool flags are not guessed.
 *
 * show_existing_frame has NO native control or tile request: FFmpeg av1dec.c
 * handles it before hwaccel start_frame, reusing its retained AVFrame. Caller
 * must do likewise (including key-frame slot refresh); do not submit it as a
 * coded frame or invent tile data. This API normalizes coded requests only.
 *
 * Explicit slot snapshots resolve the ABI's missing unmapped ref_order_hint
 * values and previous global-motion/segmentation/filter predictors. Grain
 * controls are resolved (FFmpeg fill_film_grain does this); inherited grain is
 * emitted as an explicit update, preserving seed and resolved parameters.
 * No missing mandatory authority is substituted by a default reference.
 * For >256 tiles, metadata must supply the full context-update tile ID: its
 * ninth bit is missing from the native u8 field, not inferable from tile data.
 * After successful decode, caller replaces every refresh_frame_flags-selected
 * slot with this picture's resolved frame/timestamp/order hint; updates occur
 * only after decode success, never after serialization alone. Slot snapshots
 * and firmware's retained CDF/segment maps must describe the same history.
 *
 * Inputs remain immutable/caller-owned and must not overlap destination.
 * No allocations, persistent globals, bitstream parsing, or firmware writes.
 * Success sets *length; failure sets it to zero, destination unspecified.
 * -ENODATA: absent authority; -EINVAL: inconsistent controls/spans; -ERANGE:
 * unencodable integer/overflow; -ENOSPC: destination; -EOPNOTSUPP: non-Main,
 * chroma other than 4:2:0/monochrome, monochrome grain or unknown tool flags.
 * Errors carry static reasons and tile index.
 *
 * Authority: include/uapi/linux/v4l2-controls.h AV1 controls; AV1 spec sections
 * 5.5, 5.9, 5.10; FFmpeg cbs_av1_syntax_template.c sequence_header_obu,
 * color_config, quantization_params, segmentation_params, loop_filter_params,
 * cdef_params, lr_params, uncompressed_header (coded_lossless/all_lossless),
 * tile_info, global_motion_params, film_grain_params;
 * av1dec.c read_global_param; v4l2_request_av1.c fill_sequence/fill_frame/
 * fill_film_grain/decode_slice. In particular native restoration enum values
 * differ from wire values, GM parameters are resolved fixed-point values (not
 * CBS subexp symbols), and order_hints[] indexes reference TYPES, not slots.
 * Hardware acceptance and software equivalence require caller validation.
 */
int meson_av1_request_syntax(const struct meson_av1_syntax_request *request,
			     u8 *destination, size_t capacity, size_t *length,
			     struct meson_av1_syntax_error *error);

#endif
