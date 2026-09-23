/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2018, The Linux Foundation. All rights reserved.
 * Copyright (C) 2013 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 */

#ifndef _DPU_PLANE_H_
#define _DPU_PLANE_H_

#include <drm/drm_crtc.h>

#include "dpu_kms.h"
#include "dpu_hw_mdss.h"
#include "dpu_hw_sspp.h"

/**
 * struct dpu_plane_state: Define dpu extension of drm plane state object
 * @base:	base drm plane state object
 * @pipe:	software pipe description array
 * @pipe_cfg:	software pipe configuration array
 * @stage:	assigned by crtc blender
 * @needs_qos_remap: qos remap settings need to be updated
 * @multirect_index: index of the rectangle of SSPP
 * @multirect_mode: parallel or time multiplex multirect mode
 * @pending:	whether the current update is still pending
 * @plane_fetch_bw: calculated BW per plane
 * @plane_clk: calculated clk per plane
 * @needs_dirtyfb: whether attached CRTC needs pixel data explicitly flushed
 * @layout:     framebuffer memory layout
 */
/* three channels of 256 twelve-bit entries, packed two to a dword */
#define DPU_PLANE_IGC_DWORDS	(3 * 128)

struct dpu_plane_state {
	struct drm_plane_state base;
	struct dpu_sw_pipe pipe[PIPES_PER_PLANE];
	struct dpu_sw_pipe_cfg pipe_cfg[PIPES_PER_PLANE];
	enum dpu_stage stage;
	bool needs_qos_remap;
	bool pending;

	u64 plane_fetch_bw;
	u64 plane_clk;

	bool needs_dirtyfb;

	/*
	 * The colour matrix the plane's DRM colour pipeline asks for, worked out
	 * at check time.  It has to be done there and kept: the flush path runs
	 * with only plane->state and cannot reach a colorop's atomic state, and
	 * a conversion that can fail must fail while the commit can still be
	 * refused -- not halfway through programming the hardware.
	 */
	bool has_pipeline_csc;
	struct dpu_csc_cfg pipeline_csc;

	/*
	 * And the same for the pipeline's inverse-gamma table: converted at
	 * check time and CARRIED, for the same reason.  It is held packed the
	 * way the block takes it -- three channels of 128 dwords, two twelve-bit
	 * entries to a dword -- so the flush path does no arithmetic either.
	 *
	 * @pipeline_igc_id is the DRM blob's object id, which changes whenever
	 * the compositor recomputes the curve.  It is what says whether the
	 * table on a pipe is still the table that pipe was asked for, so the
	 * commit path can skip re-sending 384 dwords for every frame of a clip
	 * whose colour has not changed.
	 */
	bool has_pipeline_igc;
	u32 pipeline_igc_id;
	u32 pipeline_igc[DPU_PLANE_IGC_DWORDS];

	/*
	 * What the compositor put in the pipeline, as booleans, purely so a
	 * change can be reported once instead of every frame.
	 */
	u32 pipeline_shape;

	struct dpu_hw_fmt_layout layout;
};

#define to_dpu_plane_state(x) \
	container_of(x, struct dpu_plane_state, base)

void dpu_plane_flush(struct drm_plane *plane);

void dpu_plane_set_error(struct drm_plane *plane, bool error);

struct drm_plane *dpu_plane_init(struct drm_device *dev,
		uint32_t pipe, enum drm_plane_type type,
		unsigned long possible_crtcs);

struct drm_plane *dpu_plane_init_virtual(struct drm_device *dev,
					 enum drm_plane_type type,
					 unsigned long possible_crtcs);

int dpu_plane_color_fill(struct drm_plane *plane,
		uint32_t color, uint32_t alpha);

#ifdef CONFIG_DEBUG_FS
void dpu_plane_danger_signal_ctrl(struct drm_plane *plane, bool enable);
#else
static inline void dpu_plane_danger_signal_ctrl(struct drm_plane *plane, bool enable) {}
#endif

int dpu_assign_plane_resources(struct dpu_global_state *global_state,
			       struct drm_atomic_state *state,
			       struct drm_crtc *crtc,
			       struct drm_plane_state **states,
			       unsigned int num_planes);

#endif /* _DPU_PLANE_H_ */
