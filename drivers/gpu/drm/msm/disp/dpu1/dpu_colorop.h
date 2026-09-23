/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef _DPU_COLOROP_H
#define _DPU_COLOROP_H

#include <drm/drm_atomic.h>
#include <drm/drm_plane.h>

#include "dpu_hw_mdss.h"

struct dpu_sspp_sub_blks;
struct drm_property_blob;
struct drm_crtc;
struct dpu_hw_ctl;

int dpu_colorop_ctm_to_csc(const struct drm_property_blob *blob,
			   struct dpu_csc_cfg *cfg);
int dpu_colorop_lut_to_igc(const struct drm_property_blob *blob, u32 *out);

bool dpu_plane_colorops_enabled(void);
int dpu_plane_init_colorops(struct drm_plane *plane, bool has_blocks);
void dpu_colorop_crtc_flush(struct drm_crtc *crtc, struct dpu_hw_ctl *ctl);
bool dpu_plane_colorop_csc(struct drm_atomic_state *state,
			   struct drm_plane_state *plane_state,
			   struct dpu_csc_cfg *out);
int dpu_plane_colorop_check(struct drm_atomic_state *state,
			    struct drm_plane_state *plane_state);

#endif /* _DPU_COLOROP_H */
