/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * The 16-bit-float chain's colour matrix takes IEEE binary16 values (this device's own driver writes them as
 * sixteen-bit halves of each register: sde_setup_fp16_cscv1). A DRM 3x4 colour matrix arrives as S31.32 in
 * SIGN-MAGNITUDE form. This header holds the conversion between the two, and the packing of a whole matrix into the
 * chain's registers, pure so a host test can include it unchanged. A host build defines u16, u32, u64 and fls64()
 * and includes <stdbool.h> before including it.
 */
#ifndef _DPU_FP16_CSC_H
#define _DPU_FP16_CSC_H

#ifdef __KERNEL__
#include <linux/bitops.h>
#include <linux/types.h>
#endif

/* the largest finite binary16: (2 - 2^-10) * 2^15 */
#define DPU_FP16_MAX_BITS	0x7bffu

/**
 * dpu_fp16_from_s31_32 - one DRM colour-matrix value as an IEEE binary16
 * @v:   the value, S31.32 in sign-magnitude form (bit 63 is the sign)
 * @out: the binary16 bit pattern
 *
 * Rounded to nearest, ties to even -- the rounding a compiler gives a constant, so a matrix a compositor computed
 * in double precision lands on the half a vendor table would hold. A magnitude below half the smallest subnormal
 * becomes a signed zero.
 *
 * Return: false when the magnitude rounds above the largest finite binary16 (65504): an infinity in a matrix is
 *         a wrong picture, so the caller refuses the pipeline.
 */
static inline bool dpu_fp16_from_s31_32(u64 v, u16 *out)
{
	u16 sign = (v >> 63) ? 0x8000 : 0;
	u64 mag = v & ~(1ULL << 63);
	u64 q, rem, half;
	int p, e, shift;

	if (!mag) {
		*out = sign;
		return true;
	}

	p = fls64(mag) - 1;		/* the top set bit: the value is in [2^(p-32), 2^(p-31)) */
	e = p - 32;

	if (e < -14) {
		/* subnormal: the unit is 2^-24, which is 2^8 in S31.32 */
		q = mag >> 8;
		rem = mag & 0xff;
		if (rem > 0x80 || (rem == 0x80 && (q & 1)))
			q++;
		*out = sign | (u16)q;		/* q == 0x400 is the smallest normal, by construction */
		return true;
	}

	/* normal: eleven significant bits, the top one implicit */
	shift = p - 10;
	if (shift > 0) {
		q = mag >> shift;
		rem = mag & ((1ULL << shift) - 1);
		half = 1ULL << (shift - 1);
		if (rem > half || (rem == half && (q & 1)))
			q++;
		if (q == 0x800) {		/* rounded up into the next binade */
			q = 0x400;
			e++;
		}
	} else {
		q = mag << -shift;
	}
	if (e > 15)
		return false;

	*out = sign | (u16)((e + 15) << 10) | (u16)(q & 0x3ff);
	return true;
}

/* the chain's registers from its base + 0x04: six of matrix, the three pre-clamps, the post-clamp */
#define DPU_FP16_CSC_WORDS	10

/**
 * dpu_fp16_csc_pack - a DRM 3x4 colour matrix as the 16-bit-float chain's ten registers
 * @v:     the twelve values, S31.32 sign-magnitude, row-major: each row's three coefficients, then its offset
 * @words: six matrix words -- value 2i in the low half, 2i + 1 in the high half -- then the pre-clamps of R, G
 *         and B and the post-clamp, each a (low, high) pair of halves
 *
 * The layout is the one the hardware was MEASURED to read: the twelve values in the order given, the offset
 * applied after the matrix (the stage-2 spike, every pixel check exact). The pre-clamps pass every finite half;
 * the post-clamp keeps the output in [0, 1].
 *
 * Return: false, with @words untouched, when a value has no finite binary16.
 */
static inline bool dpu_fp16_csc_pack(const u64 v[12], u32 words[DPU_FP16_CSC_WORDS])
{
	u16 h[12];
	int i;

	for (i = 0; i < 12; i++)
		if (!dpu_fp16_from_s31_32(v[i], &h[i]))
			return false;
	for (i = 0; i < 6; i++)
		words[i] = h[2 * i] | (u32)h[2 * i + 1] << 16;
	words[6] = 0x7bfffbff;		/* R: [-65504, 65504] */
	words[7] = 0x7bfffbff;		/* G */
	words[8] = 0x7bfffbff;		/* B */
	words[9] = 0x3c000000;		/* the output: [0, 1] */
	return true;
}

#endif /* _DPU_FP16_CSC_H */
