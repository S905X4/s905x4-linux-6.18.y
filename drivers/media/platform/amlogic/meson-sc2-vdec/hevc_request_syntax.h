/* SPDX-License-Identifier: GPL-2.0 */
#ifndef MESON_HEVC_REQUEST_SYNTAX_H
#define MESON_HEVC_REQUEST_SYNTAX_H

#include <linux/types.h>
#include <linux/v4l2-controls.h>

/**
 * struct meson_hevc_syntax_slice - one original, escaped slice NAL
 * @params: native Request slice control, with offsets relative to @data
 * @data: NAL header plus EBSP, optionally preceded by one Annex-B start code
 * @size: accessible bytes; params->bit_size identifies this slice's end
 * @entry_point_offsets: native control values (byte lengths, NOT minus one)
 * @num_entry_point_offsets: number of accessible entry-point control values
 *
 * Slice spans must exclude any preceding parameter sets and other NALs. If a
 * userspace producer uses offsets relative to a concatenated OUTPUT buffer,
 * the caller must rebase bit_size/data_byte_offset to each individual span.
 * data_byte_offset identifies the first CABAC EBSP byte, including any start
 * code and two-byte NAL header. No RBSP/unescape conversion is wanted here.
 */
struct meson_hevc_syntax_slice {
	const struct v4l2_ctrl_hevc_slice_params *params;
	const u8 *data;
	size_t size;
	const u32 *entry_point_offsets;
	size_t num_entry_point_offsets;
};

struct meson_hevc_syntax_request {
	const struct v4l2_ctrl_hevc_sps *sps;
	const struct v4l2_ctrl_hevc_pps *pps;
	/* Mandatory when SCALING_LIST_ENABLED is set, including default lists. */
	const struct v4l2_ctrl_hevc_scaling_matrix *scaling;
	const struct v4l2_ctrl_hevc_decode_params *decode;
	const struct meson_hevc_syntax_slice *slices;
	size_t num_slices;
};

/**
 * struct meson_hevc_syntax_error - evidence for a rejected input
 * @slice: offending slice index, or SIZE_MAX for picture/parameter controls
 * @reason: static string identifying the missing/inconsistent syntax authority
 *
 * -EINVAL means inconsistent/malformed controls or original slice prefix;
 * -ENODATA means a mandatory control/span is absent; -ENOSPC means destination
 * capacity; -ERANGE means an unencodable integer delta. -EOPNOTSUPP identifies
 * actual out-of-scope syntax, not interframes, multislices, tiles, WPP,
 * dependent segments, or nondefault scaling matrices.
 */
struct meson_hevc_syntax_error {
	size_t slice;
	const char *reason;
};

/**
 * meson_hevc_request_syntax() - serialize a controls-authoritative access unit
 *
 * Produces Annex-B VPS/SPS/PPS followed by reconstructed slice headers and
 * byte-for-byte original CABAC EBSP payloads, including entry-point boundaries.
 * SPS RPS tables are deliberately empty. Explicit RPS includes every supplied
 * DPB entry (unused-by-current entries are retained); list modifications map
 * resolved L0/L1 indices to the reconstructed RPS, independent of original
 * SPS tables. Parameter IDs/coding tools come from controls. Original headers
 * supply only pic_output_flag, absent from the native ABI. Their initial
 * geometry/type/ID fields are checked against controls, not used as overrides.
 * Reserved slice-header-extension bytes have no Main/Main10 decoding semantics
 * and are replaced by a zero-length extension. No VUI/HRD/crop/SEI is generated:
 * display crop/color/timing remain the caller's separate metadata contract.
 * The generated PTL declares progressive, non-packed single-layer Main/Main10,
 * high tier, level 6.2; this is a decoding envelope, not original-stream metadata.
 * Native controls do not expose PTL, RExt/SCC/multilayer extension parameter
 * sets, VUI field_seq_flag or VPS topology. The caller must admit only known
 * progressive single-layer Main/Main10 streams; these omitted tools cannot be
 * detected or reconstructed from the supplied controls and a slice NAL alone.
 * Explicitly supplied SCC/field/unknown tool flags are rejected, never ignored.
 *
 * IMPORTANT: a fresh firmware parser cannot infer full POC MSBs or install
 * timestamp-addressed reference backing from a bitstream. At header-done the
 * host MUST program control-authoritative full current/reference POCs, resolved
 * lists and reference/MV storage before firmware resume. Do not reset between
 * slices of a picture. For independent software decoding, feed normalized
 * access units in decode order with the same retained reference history.
 *
 * Source authority: native include/uapi/linux/v4l2-controls.h; ITU H.265 7.3.2,
 * 7.3.6, 7.3.7 and 8.3; local FFmpeg hevcdec.c (hls_slice_header,
 * decode_lt_rps, pred_weight_table), hevc_ps.c (scaling_list_data), and
 * v4l2_req_hevc_vx.c. The latter's old ABI DPB-index/POC mapping must NOT be
 * copied: native poc_* arrays contain DPB indices; chroma weights contain
 * resolved offsets, which this serializer converts back to delta syntax.
 *
 * All inputs stay caller-owned and immutable for the call. Destination must
 * not overlap any input. No allocations or retained state. On success, *length
 * is the complete byte count; on failure it is zero and destination contents
 * are unspecified. This source has not itself established hardware acceptance.
 */
int meson_hevc_request_syntax(const struct meson_hevc_syntax_request *request,
			      u8 *destination, size_t capacity, size_t *length,
			      struct meson_hevc_syntax_error *error);

#endif
