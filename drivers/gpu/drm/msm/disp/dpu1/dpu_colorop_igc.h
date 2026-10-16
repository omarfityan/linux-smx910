/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * The conversion of a DRM 1D LUT into a video pipe's inverse-gamma table -- pure arithmetic, no
 * hardware, so a host test can include this header unchanged and check it against the block's
 * measured behaviour.
 *
 * A host build defines u16, u32, u64, s64 and div64_u64() before including it.
 */
#ifndef _DPU_COLOROP_IGC_H
#define _DPU_COLOROP_IGC_H

#ifdef __KERNEL__
#include <linux/math64.h>
#include <linux/types.h>
#endif

/*
 * THE BLOCK'S GRID IS NOT DRM'S -- measured on this device (SM8550 VIG, inverse gamma v6):
 *
 *   slot i is read at 10-bit input 4i (slot 255 at 1020) and the endpoint register at input 1024;
 *   the block interpolates linearly between them; the layer mixer takes round(x / 4) of the
 *   twelve-bit result, so twelve-bit 4c is what arrives as ten-bit code c.
 *
 * A DRM_COLOROP_1D_LUT spreads its n entries uniformly over input [0, 1] instead. Copied slot for
 * slot, every curve is read 0.3 % compressed (proven with an identity table: 1024 of 1024 codes
 * matched that model, code - v running +0..+4), which near a PQ curve's knee is several codes. So
 * the LUT is RESAMPLED: slot i takes the DRM curve at input 4i / 1023, and the endpoint is set so
 * that input 1023 -- three quarters of the way from slot 255 to it -- lands on the LUT's last value.
 */
#define DPU_IGC_SLOTS		256
#define DPU_IGC_IN_MAX		1023	/* the pipe's ten-bit input full scale */
#define DPU_IGC_OUT_ONE		4092	/* twelve-bit value that reaches the mixer as code 1023 */
#define DPU_IGC_OUT_MAX		4095

/*
 * WHERE THE CURVE'S 1.0 GOES depends on what reads the block's output.  The layer mixer takes
 * round(x / 4), so 4092 is the value that arrives as 1023 (DPU_IGC_OUT_ONE).  The video pipe's
 * gamut (the 3D LUT) samples the twelve-bit output over 0..4095 instead, and at 4092 its top cell
 * is read 3 codes short: KWin's white 1023 came out as 1020 / 1021 (measured).  So while a 3D LUT
 * is in force the curve is scaled to DPU_IGC_OUT_MAX.
 */

/**
 * dpu_igc_convert_channel_scaled - one channel of a DRM 1D LUT into the block's slots and endpoint
 * @lut:      the channel's first value (a u32 per entry, full scale 0xffffffff)
 * @stride:   u32s between consecutive entries (4 for struct drm_color_lut32)
 * @n:        entries in the LUT (at least 2)
 * @one:      the twelve-bit value the LUT's 1.0 maps to: DPU_IGC_OUT_ONE, or DPU_IGC_OUT_MAX in
 *            front of a 3D LUT
 * @slots:    DPU_IGC_SLOTS twelve-bit values to fill
 * @endpoint: the value for the register past the last slot
 */
static inline void dpu_igc_convert_channel_scaled(const u32 *lut, unsigned int stride, unsigned int n,
						  u16 one, u16 *slots, u16 *endpoint)
{
	const u64 full = 0xffffffffull, den = DPU_IGC_IN_MAX;
	unsigned int i;
	u64 top, q;
	s64 e3;

	for (i = 0; i < DPU_IGC_SLOTS; i++) {
		/* input 4i is DRM position 4i * (n - 1) / 1023: entry j, remainder r (of den) */
		u64 num = (u64)4 * i * (n - 1);
		u64 j = div64_u64(num, den), r = num - j * den;
		s64 a = lut[j * stride], y_den;

		/* the curve times den; between two entries, so never below the smaller one */
		y_den = j + 1 < n ? a * (s64)den + ((s64)lut[(j + 1) * stride] - a) * (s64)r
				  : a * (s64)den;
		q = div64_u64((u64)y_den * one + den * full / 2, den * full);
		slots[i] = q > DPU_IGC_OUT_MAX ? DPU_IGC_OUT_MAX : q;
	}

	/* 3 * endpoint = 4 * (the last value) - slot 255; clamped to the register's range */
	top = div64_u64((u64)lut[(n - 1) * stride] * one + full / 2, full);
	e3 = 4 * (s64)top - (s64)slots[DPU_IGC_SLOTS - 1];
	q = e3 < 0 ? 0 : div64_u64((u64)e3 + 1, 3);
	*endpoint = q > DPU_IGC_OUT_MAX ? DPU_IGC_OUT_MAX : q;
}

/**
 * dpu_igc_convert_channel - dpu_igc_convert_channel_scaled() for the path without a 3D LUT
 */
static inline void dpu_igc_convert_channel(const u32 *lut, unsigned int stride, unsigned int n,
					   u16 *slots, u16 *endpoint)
{
	dpu_igc_convert_channel_scaled(lut, stride, n, DPU_IGC_OUT_ONE, slots, endpoint);
}

#endif /* _DPU_COLOROP_IGC_H */
