/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Fetch programming for 16-bit-float pixels -- pure arithmetic, so a host test can include this
 * header unchanged. A host build defines u32 and includes <stdbool.h> before including it.
 */
#ifndef _DPU_FP16_LAYOUT_H
#define _DPU_FP16_LAYOUT_H

#ifdef __KERNEL__
#include <linux/types.h>
#endif

/*
 * SRC_FORMAT bits 9-10 hold the pixel's byte count minus one; two bits wide, so an 8-byte pixel's
 * (bpp - 1) << 9 spills into bit 11, which is ROT90. This device's own driver encodes 16-bit float
 * as the 4-byte field plus bit 16 (sde_hw_sspp.c).
 */
#define DPU_SRC_FORMAT_FP16		(1u << 16)
#define DPU_SRC_FORMAT_BPP_SHIFT	9

static inline u32 dpu_src_format_bpp_bits(u32 bpp, int is_fp16)
{
	if (is_fp16)
		return DPU_SRC_FORMAT_FP16 | (3u << DPU_SRC_FORMAT_BPP_SHIFT);
	return (bpp - 1) << DPU_SRC_FORMAT_BPP_SHIFT;
}

/*
 * UBWC for 8-byte pixels as Mesa's fd6_layout.c lays it out -- the allocator of every such buffer
 * here: the tiled pitch aligned to 64 texels (512 bytes) and an 8x4 metadata block. At 4 bytes a
 * pixel both rules equal the 256-byte pitch and 16-pixel tile already used.
 */
#define DPU_FP16_UBWC_PITCH_ALIGN	512u
#define DPU_FP16_UBWC_META_TILE_W	8u

static inline u32 dpu_fp16_ubwc_pitch(u32 width)
{
	return (width * 8 + DPU_FP16_UBWC_PITCH_ALIGN - 1) / DPU_FP16_UBWC_PITCH_ALIGN * DPU_FP16_UBWC_PITCH_ALIGN;
}

/* this path never reads the pitch from the framebuffer, so a buffer laid out otherwise is refused */
static inline bool dpu_fp16_ubwc_pitch_ok(u32 width, u32 pitch)
{
	return pitch == dpu_fp16_ubwc_pitch(width);
}

#endif /* _DPU_FP16_LAYOUT_H */
