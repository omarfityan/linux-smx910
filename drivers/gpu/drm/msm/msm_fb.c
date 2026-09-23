// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2013 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 */

#include <drm/drm_crtc.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_file.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_probe_helper.h>

#include "msm_drv.h"
#include "msm_kms.h"
#include "msm_gem.h"

struct msm_framebuffer {
	struct drm_framebuffer base;
	const struct msm_format *format;

	/* Count of # of attached planes which need dirtyfb: */
	refcount_t dirtyfb;

	/* Framebuffer per-plane address, if pinned, else zero: */
	uint64_t iova[DRM_FORMAT_MAX_PLANES];
	atomic_t prepare_count;
};
#define to_msm_framebuffer(x) container_of(x, struct msm_framebuffer, base)

static struct drm_framebuffer *msm_framebuffer_init(struct drm_device *dev,
		const struct drm_format_info *info,
		const struct drm_mode_fb_cmd2 *mode_cmd, struct drm_gem_object **bos);

static int msm_framebuffer_dirtyfb(struct drm_framebuffer *fb,
				   struct drm_file *file_priv, unsigned int flags,
				   unsigned int color, struct drm_clip_rect *clips,
				   unsigned int num_clips)
{
	struct msm_framebuffer *msm_fb = to_msm_framebuffer(fb);

	/* If this fb is not used on any display requiring pixel data to be
	 * flushed, then skip dirtyfb
	 */
	if (refcount_read(&msm_fb->dirtyfb) == 1)
		return 0;

	return drm_atomic_helper_dirtyfb(fb, file_priv, flags, color,
					 clips, num_clips);
}

static const struct drm_framebuffer_funcs msm_framebuffer_funcs = {
	.create_handle = drm_gem_fb_create_handle,
	.destroy = drm_gem_fb_destroy,
	.dirty = msm_framebuffer_dirtyfb,
};

#ifdef CONFIG_DEBUG_FS
void msm_framebuffer_describe(struct drm_framebuffer *fb, struct seq_file *m)
{
	struct msm_gem_stats stats = {};
	int i, n = fb->format->num_planes;

	seq_printf(m, "fb: %dx%d@%4.4s (%2d, ID:%d)\n",
			fb->width, fb->height, (char *)&fb->format->format,
			drm_framebuffer_read_refcount(fb), fb->base.id);

	for (i = 0; i < n; i++) {
		seq_printf(m, "   %d: offset=%d pitch=%d, obj: ",
				i, fb->offsets[i], fb->pitches[i]);
		msm_gem_describe(fb->obj[i], m, &stats);
	}
}
#endif

/* prepare/pin all the fb's bo's for scanout.
 */
int msm_framebuffer_prepare(struct drm_framebuffer *fb, bool needs_dirtyfb)
{
	struct msm_drm_private *priv = fb->dev->dev_private;
	struct drm_gpuvm *vm = priv->kms->vm;
	struct msm_framebuffer *msm_fb = to_msm_framebuffer(fb);
	int ret, i, n = fb->format->num_planes;

	if (needs_dirtyfb)
		refcount_inc(&msm_fb->dirtyfb);

	if (atomic_inc_return(&msm_fb->prepare_count) > 1)
		return 0;

	for (i = 0; i < n; i++) {
		msm_gem_vma_get(fb->obj[i]);
		ret = msm_gem_get_and_pin_iova(fb->obj[i], vm, &msm_fb->iova[i]);
		drm_dbg_state(fb->dev, "FB[%u]: iova[%d]: %08llx (%d)\n",
			      fb->base.id, i, msm_fb->iova[i], ret);
		if (ret)
			return ret;
	}

	return 0;
}

void msm_framebuffer_cleanup(struct drm_framebuffer *fb, bool needed_dirtyfb)
{
	struct msm_drm_private *priv = fb->dev->dev_private;
	struct drm_gpuvm *vm = priv->kms->vm;
	struct msm_framebuffer *msm_fb = to_msm_framebuffer(fb);
	int i, n = fb->format->num_planes;

	if (needed_dirtyfb)
		refcount_dec(&msm_fb->dirtyfb);

	if (atomic_dec_return(&msm_fb->prepare_count))
		return;

	memset(msm_fb->iova, 0, sizeof(msm_fb->iova));

	for (i = 0; i < n; i++) {
		msm_gem_unpin_iova(fb->obj[i], vm);
		msm_gem_vma_put(fb->obj[i]);
	}
}

uint32_t msm_framebuffer_iova(struct drm_framebuffer *fb, int plane)
{
	struct msm_framebuffer *msm_fb = to_msm_framebuffer(fb);
	return msm_fb->iova[plane] + fb->offsets[plane];
}

struct drm_gem_object *msm_framebuffer_bo(struct drm_framebuffer *fb, int plane)
{
	return drm_gem_fb_get_obj(fb, plane);
}

const struct msm_format *msm_framebuffer_format(struct drm_framebuffer *fb)
{
	struct msm_framebuffer *msm_fb = to_msm_framebuffer(fb);
	return msm_fb->format;
}

/*
 * 🔴 LOGGING ONLY -- NAME EVERY DISTINCT FRAMEBUFFER FORMAT USERSPACE EVER CREATES.
 *
 * msm_kms.c registers this file's msm_framebuffer_create() as .fb_create, so EVERY
 * drm_mode_addfb2 on this device passes through here.  That makes it the one place
 * that can answer "did the compositor ever import the video buffer at all?" --
 * a question three other instruments could not reach:
 *
 *   the uprobe kit          pinned to a libkwin sha that has moved twice
 *   the compositor's own    prepareAtomicPlane carries NO qCDebug on its refusal path,
 *     debug logging         so a refusal is indistinguishable from code that never ran
 *   the framebuffer debugfs a scanned-out client's framebuffer is byte-indistinguishable
 *     dump                  from the compositor's own -- same format, same size, and
 *                           `allocated by` always names whoever called ADDFB2
 *
 * REPORTED ONCE PER DISTINCT (FORMAT, WIDTH, HEIGHT), EVER -- not on change.  A
 * compositor cycling two formats would make an on-change report flap at the refresh
 * rate; a first-sighting report is monotonic and cannot.
 *
 * Keyed on the SIZE as well as the format, because that is what gives the measurement
 * a LIVE control.  Every format the compositor uses is already named during boot, so a
 * format-only key would leave an arm that prints nothing indistinguishable from a hook
 * that has stopped running.  With the size in the key, a player opened at a resolution
 * nothing has used yet prints -- proving the reporter is still alive at the moment the
 * real question is being asked.
 *
 * The racing case is benign: two simultaneous creates of a new format print it twice.
 * A lock here would cost more than the duplicate line it prevents.
 */
struct msm_fb_reported {
	u32 fourcc;
	u32 width;
	u32 height;
};
static struct msm_fb_reported reported_fbs[64];
static unsigned int reported_fb_count;
static bool reported_fb_table_full;

static void msm_fb_report_new_format(const struct drm_format_info *info,
				     const struct drm_mode_fb_cmd2 *mode_cmd)
{
	unsigned int i;

	for (i = 0; i < reported_fb_count; i++)
		if (reported_fbs[i].fourcc == mode_cmd->pixel_format &&
		    reported_fbs[i].width  == mode_cmd->width &&
		    reported_fbs[i].height == mode_cmd->height)
			return;

	/*
	 * 🔴 SAY SO WHEN THE TABLE FILLS, rather than going quiet.  A silent
	 * reporter that has stopped reporting is indistinguishable from one that
	 * has nothing to report, and that is the exact shape of instrument failure
	 * this driver's log has been bitten by before.
	 */
	if (reported_fb_count >= ARRAY_SIZE(reported_fbs)) {
		if (!reported_fb_table_full) {
			reported_fb_table_full = true;
			DRM_INFO("colorop: framebuffer report table FULL at %zu entries -- no further formats will be named\n",
				 ARRAY_SIZE(reported_fbs));
		}
		return;
	}
	reported_fbs[reported_fb_count].fourcc = mode_cmd->pixel_format;
	reported_fbs[reported_fb_count].width  = mode_cmd->width;
	reported_fbs[reported_fb_count].height = mode_cmd->height;
	reported_fb_count++;

	DRM_INFO("colorop: framebuffer CREATED, first of its format: %p4cc mod 0x%llx %ux%u %u-plane%s\n",
		 &mode_cmd->pixel_format, mode_cmd->modifier[0],
		 mode_cmd->width, mode_cmd->height, info->num_planes,
		 info->is_yuv ? "  <== YUV" : "");
}

struct drm_framebuffer *msm_framebuffer_create(struct drm_device *dev,
		struct drm_file *file, const struct drm_format_info *info,
		const struct drm_mode_fb_cmd2 *mode_cmd)
{
	struct drm_gem_object *bos[4] = {0};
	struct drm_framebuffer *fb;
	int ret, i, n = info->num_planes;

	msm_fb_report_new_format(info, mode_cmd);

	for (i = 0; i < n; i++) {
		bos[i] = drm_gem_object_lookup(file, mode_cmd->handles[i]);
		if (!bos[i]) {
			ret = -ENXIO;
			goto out_unref;
		}
	}

	fb = msm_framebuffer_init(dev, info, mode_cmd, bos);
	if (IS_ERR(fb)) {
		ret = PTR_ERR(fb);
		goto out_unref;
	}

	return fb;

out_unref:
	for (i = 0; i < n; i++)
		drm_gem_object_put(bos[i]);
	return ERR_PTR(ret);
}

static struct drm_framebuffer *msm_framebuffer_init(struct drm_device *dev,
		const struct drm_format_info *info,
		const struct drm_mode_fb_cmd2 *mode_cmd, struct drm_gem_object **bos)
{
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_kms *kms = priv->kms;
	struct msm_framebuffer *msm_fb = NULL;
	struct drm_framebuffer *fb;
	const struct msm_format *format;
	int ret, i, n;

	drm_dbg_state(dev, "create framebuffer: mode_cmd=%p (%dx%d@%p4cc)\n",
		      mode_cmd, mode_cmd->width, mode_cmd->height,
		      &mode_cmd->pixel_format);

	n = info->num_planes;
	format = mdp_get_format(kms, mode_cmd->pixel_format,
			mode_cmd->modifier[0]);
	if (!format) {
		DRM_DEV_ERROR(dev->dev, "unsupported pixel format: %p4cc\n",
			      &mode_cmd->pixel_format);
		ret = -EINVAL;
		goto fail;
	}

	msm_fb = kzalloc_obj(*msm_fb);
	if (!msm_fb) {
		ret = -ENOMEM;
		goto fail;
	}

	fb = &msm_fb->base;

	msm_fb->format = format;

	if (n > ARRAY_SIZE(fb->obj)) {
		ret = -EINVAL;
		goto fail;
	}

	for (i = 0; i < n; i++) {
		unsigned int width = mode_cmd->width / (i ? info->hsub : 1);
		unsigned int height = mode_cmd->height / (i ? info->vsub : 1);
		unsigned int min_size;

		min_size = (height - 1) * mode_cmd->pitches[i]
			 + width * info->cpp[i]
			 + mode_cmd->offsets[i];

		if (bos[i]->size < min_size) {
			ret = -EINVAL;
			goto fail;
		}

		msm_fb->base.obj[i] = bos[i];
	}

	drm_helper_mode_fill_fb_struct(dev, fb, info, mode_cmd);

	ret = drm_framebuffer_init(dev, fb, &msm_framebuffer_funcs);
	if (ret) {
		DRM_DEV_ERROR(dev->dev, "framebuffer init failed: %d\n", ret);
		goto fail;
	}

	refcount_set(&msm_fb->dirtyfb, 1);

	drm_dbg_state(dev, "create: FB ID: %d (%p)\n", fb->base.id, fb);

	return fb;

fail:
	kfree(msm_fb);

	return ERR_PTR(ret);
}

struct drm_framebuffer *
msm_alloc_stolen_fb(struct drm_device *dev, int w, int h, int p, uint32_t format)
{
	struct drm_mode_fb_cmd2 mode_cmd = {
		.pixel_format = format,
		.width = w,
		.height = h,
		.pitches = { p },
	};
	struct drm_gem_object *bo;
	struct drm_framebuffer *fb;
	int size;

	/* allocate backing bo */
	size = mode_cmd.pitches[0] * mode_cmd.height;
	DBG("allocating %d bytes for fb %d", size, dev->primary->index);
	bo = msm_gem_new(dev, size, MSM_BO_SCANOUT | MSM_BO_WC | MSM_BO_STOLEN);
	if (IS_ERR(bo)) {
		dev_warn(dev->dev, "could not allocate stolen bo\n");
		/* try regular bo: */
		bo = msm_gem_new(dev, size, MSM_BO_SCANOUT | MSM_BO_WC);
	}
	if (IS_ERR(bo)) {
		DRM_DEV_ERROR(dev->dev, "failed to allocate buffer object\n");
		return ERR_CAST(bo);
	}

	msm_gem_object_set_name(bo, "stolenfb");

	fb = msm_framebuffer_init(dev,
				  drm_get_format_info(dev, mode_cmd.pixel_format,
						      mode_cmd.modifier[0]),
				  &mode_cmd, &bo);
	if (IS_ERR(fb)) {
		DRM_DEV_ERROR(dev->dev, "failed to allocate fb\n");
		/* note: if fb creation failed, we can't rely on fb destroy
		 * to unref the bo:
		 */
		drm_gem_object_put(bo);
		return ERR_CAST(fb);
	}

	return fb;
}
