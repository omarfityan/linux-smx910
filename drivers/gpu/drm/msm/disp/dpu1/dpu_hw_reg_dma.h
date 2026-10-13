/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2026 The Linux Foundation. All rights reserved.
 *
 * Minimal LUTDMA (register-DMA) engine for the DPU.
 *
 * The DPU contains a small DMA engine that reads a command buffer out of
 * memory and replays it as register writes into the display blocks.  Several
 * colour blocks -- notably the source-pipe IGC and gamut LUTs -- have no MMIO
 * write path at all on this hardware revision and can only be programmed this
 * way, so a driver without this engine cannot expose them.
 *
 * This is the engine only: map the block, build a command buffer, submit it,
 * and report whether the hardware acknowledged the transfer.  Feature-level
 * users (colour operations, LUT uploads) are built on top of it.
 */

#ifndef _DPU_HW_REG_DMA_H
#define _DPU_HW_REG_DMA_H

#include <linux/types.h>
#include "dpu_hw_mdss.h"
#include "dpu_hw_util.h"

struct dpu_kms;
struct dpu_hw_ctl;
struct dpu_hw_sspp;

/*
 * Decode-select block identifiers.
 *
 * A command buffer opens with a decode-select operation naming the blocks the
 * writes that follow are addressed to; every subsequent offset is relative to
 * the selected block's base as the hardware decoder holds it.  Keeping every
 * write block-relative is what makes the command buffer independent of where
 * the CPU's register mapping happens to be anchored.
 */
#define DPU_REG_DMA_BLK_VIG0		BIT(0)
#define DPU_REG_DMA_BLK_VIG1		BIT(1)
#define DPU_REG_DMA_BLK_VIG2		BIT(2)
#define DPU_REG_DMA_BLK_VIG3		BIT(3)
#define DPU_REG_DMA_BLK_SSPP_IGC	BIT(4)
#define DPU_REG_DMA_BLK_DMA0		BIT(5)
#define DPU_REG_DMA_BLK_DMA1		BIT(6)
#define DPU_REG_DMA_BLK_DMA2		BIT(7)
#define DPU_REG_DMA_BLK_DMA3		BIT(8)
#define DPU_REG_DMA_BLK_DMA4		BIT(9)
#define DPU_REG_DMA_BLK_DMA5		BIT(10)
#define DPU_REG_DMA_BLK_DSPP0		BIT(17)
#define DPU_REG_DMA_BLK_DSPP1		BIT(18)
#define DPU_REG_DMA_BLK_DSPP2		BIT(19)
#define DPU_REG_DMA_BLK_DSPP3		BIT(20)
#define DPU_REG_DMA_BLK_DSPP_IGC	BIT(21)

/**
 * struct dpu_reg_dma_buf - a command buffer for the engine
 * @bo:    backing GEM object
 * @iova:  device address the engine fetches from
 * @vaddr: CPU mapping the command buffer is built through
 * @size:  usable size in bytes
 * @index: bytes of command buffer written so far
 */
struct dpu_reg_dma_buf {
	struct drm_gem_object *bo;
	u64 iova;
	u32 *vaddr;
	u32 size;
	u32 index;
};

/**
 * struct dpu_reg_dma - LUTDMA engine state
 * @base:  ioremapped LUTDMA register block
 * @db:    register map for the double-buffered submission block
 * @buf:   the command buffer
 * @ready: engine mapped and buffer allocated
 */
struct dpu_reg_dma {
	void __iomem *base;
	struct dpu_hw_blk_reg_map db;
	struct dpu_reg_dma_buf buf;
	bool ready;
};

int dpu_reg_dma_init(struct dpu_kms *dpu_kms, struct dpu_reg_dma *rdma,
		     u32 buf_size);
void dpu_reg_dma_fini(struct dpu_kms *dpu_kms, struct dpu_reg_dma *rdma);

void dpu_reg_dma_buf_reset(struct dpu_reg_dma *rdma);
int dpu_reg_dma_decode_sel(struct dpu_reg_dma *rdma, u32 blk_mask);
int dpu_reg_dma_single_write(struct dpu_reg_dma *rdma, u32 blk_off, u32 val);
int dpu_reg_dma_block_write(struct dpu_reg_dma *rdma, u32 blk_off,
			    const u32 *data, u32 count);
int dpu_reg_dma_indexed_write(struct dpu_reg_dma *rdma, u32 blk_off,
			      const u32 *data, u32 count);
int dpu_reg_dma_single_modify(struct dpu_reg_dma *rdma, u32 blk_off, u32 mask,
			      u32 val);

/*
 * LUT-bus targets.  A LUT-bus write fills a colour block's table memory rather
 * than its registers: the block is named here, the pipe by the preceding
 * decode-select.  The values are the device's own downstream driver's
 * (sde_reg_dma.h: enum sde_reg_dma_lutbus_block, LUTBUS_TABLE_SELECT_MAX and the
 * LUTBUS_*_TRANS_SIZE transfer granules, in dwords).
 */
#define DPU_REG_DMA_LUTBUS_IGC			0
#define DPU_REG_DMA_LUTBUS_GAMUT		1
#define DPU_REG_DMA_LUTBUS_SIXZONE		3
#define DPU_REG_DMA_LUTBUS_BLOCKS		4
#define DPU_REG_DMA_LUTBUS_TABLES		2
#define DPU_REG_DMA_LUTBUS_IGC_TRANS		3
#define DPU_REG_DMA_LUTBUS_GAMUT_TRANS		6
#define DPU_REG_DMA_LUTBUS_SIXZONE_TRANS	5

int dpu_reg_dma_lutbus_write(struct dpu_reg_dma *rdma, u32 block, u32 table,
			     u32 trans_size, const u32 *data, u32 count);

int dpu_reg_dma_kick_off(struct dpu_kms *dpu_kms, struct dpu_reg_dma *rdma,
			 struct dpu_hw_ctl *ctl, bool trigger);

/*
 * The video pipes' inverse-gamma table, which is what a DRM plane colour
 * pipeline's 1D lookup-table operation lands on.  The block has no CPU write
 * path, so these go through the engine above.
 */
u32 dpu_reg_dma_blk_for_sspp(enum dpu_sspp id);
bool dpu_reg_dma_sspp_igc_is_on(struct dpu_hw_sspp *sspp);
int dpu_reg_dma_program_sspp_igc(struct dpu_kms *dpu_kms, struct dpu_reg_dma *rdma,
				 struct dpu_hw_ctl *ctl, struct dpu_hw_sspp *sspp,
				 const u32 *lut, const u32 *last, bool report);
int dpu_reg_dma_disable_sspp_igc(struct dpu_kms *dpu_kms, struct dpu_reg_dma *rdma,
				 struct dpu_hw_ctl *ctl, struct dpu_hw_sspp *sspp);

struct dpu_hw_dspp;
void dpu_reg_dma_selftest_run(struct dpu_kms *dpu_kms,
			      struct dpu_hw_dspp *dspp, struct dpu_hw_ctl *ctl);
/*
 * Give the self-test a device to allocate against outside the commit path, and
 * take it away again so a driver rebind cannot leave the engine holding a
 * mapping and an address from a torn-down device.
 */
void dpu_reg_dma_selftest_bind(struct dpu_kms *dpu_kms);
void dpu_reg_dma_selftest_unbind(struct dpu_kms *dpu_kms);

#endif /* _DPU_HW_REG_DMA_H */
