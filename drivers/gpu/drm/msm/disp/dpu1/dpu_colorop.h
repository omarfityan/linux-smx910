/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef _DPU_COLOROP_H
#define _DPU_COLOROP_H

#include <drm/drm_atomic.h>
#include <drm/drm_plane.h>

#include "dpu_hw_mdss.h"
#include "dpu_report_memory.h"
#include "dpu_fp16_csc.h"

struct dpu_sspp_sub_blks;
struct drm_property_blob;
struct drm_crtc;
struct dpu_hw_ctl;

/* the colour-pipeline refusals a plane's checks report: the keys of its refusal memory */
enum dpu_colorop_refusal {
	DPU_COLOROP_REFUSAL_NO_STATE = 1,
	DPU_COLOROP_REFUSAL_MATRIX,
	DPU_COLOROP_REFUSAL_TABLE_SIZE,
	DPU_COLOROP_REFUSAL_PIPE_WITHOUT_BLOCKS,
	DPU_COLOROP_REFUSAL_FP16_MATRIX_PIPE,
	DPU_COLOROP_REFUSAL_RGB_MATRIX_PIPE,
};

/**
 * struct dpu_colorop_reports - what a plane's colour-pipeline checks last reported (logging only)
 * @shape:   the pipeline shape, for the "at check" report
 * @refusal: the refusal last reported
 *
 * On the PLANE, never the plane state: a refused check's state is thrown away, and a memory kept there
 * would forget every refused offer (dpu_report_memory.h has the contract).
 */
struct dpu_colorop_reports {
	struct dpu_report_change shape;
	struct dpu_report_event refusal;
};

/* a refusal's line is printed once per run of that refusal on the plane */
static inline bool dpu_colorop_report_refusal(struct dpu_colorop_reports *reports,
					      enum dpu_colorop_refusal why)
{
	return dpu_report_event_note(&reports->refusal, why, jiffies,
				     msecs_to_jiffies(DPU_REPORT_WINDOW_MS));
}

int dpu_colorop_ctm_to_csc(const struct drm_property_blob *blob,
			   struct dpu_csc_cfg *cfg, bool src_yuv_8bit);
int dpu_colorop_ctm_to_fp16_csc(const struct drm_property_blob *blob,
				u32 words[DPU_FP16_CSC_WORDS]);
int dpu_colorop_lut_to_igc(const struct drm_property_blob *blob, u32 *out, u32 *last);

bool dpu_plane_colorops_enabled(void);
int dpu_plane_init_colorops(struct drm_plane *plane, bool has_blocks);
void dpu_colorop_crtc_flush(struct drm_crtc *crtc, struct dpu_hw_ctl *ctl);
bool dpu_plane_colorop_csc(struct drm_atomic_state *state,
			   struct drm_plane_state *plane_state,
			   struct dpu_csc_cfg *out, bool src_yuv_8bit);
int dpu_plane_colorop_check(struct drm_atomic_state *state,
			    struct drm_plane_state *plane_state,
			    struct dpu_colorop_reports *reports, bool fp16,
			    bool src_yuv_8bit);

#endif /* _DPU_COLOROP_H */
