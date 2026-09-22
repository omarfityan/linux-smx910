// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2026 The Linux Foundation. All rights reserved.
 *
 * Minimal LUTDMA (register-DMA) engine for the DPU.
 */

#define pr_fmt(fmt)	"[dpu-lutdma] " fmt

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/moduleparam.h>

#include "dpu_hw_reg_dma.h"
#include "dpu_hw_catalog.h"
#include "dpu_hw_ctl.h"
#include "dpu_hw_dspp.h"
#include "dpu_hw_top.h"
#include "dpu_kms.h"
#include "msm_gem.h"

/*
 * Where the engine lives.
 *
 * The register block is not covered by either range the DPU node maps: "mdp"
 * ends below it and "vbif" starts above it, so it is mapped here directly.
 * The address is transcribed from this device's own description of its display
 * subsystem, which lists four regions -- mdp, vbif, regdma and ipcc -- of which
 * the third is this one.
 */
#define DPU_REG_DMA_PHYS_BASE		0x0aeac000
#define DPU_REG_DMA_PHYS_LEN		0x800

/* the double-buffered submission block sits at the start of that region */
#define DPU_REG_DMA_DB_OFF		0x0

/*
 * Engine registers, relative to a submission block's base.
 *
 * Interrupt status and clear are banks of five 32-bit registers: the first
 * carries per-CTL transfer-complete, and the fifth carries error status.
 */
#define REG_DMA_OPMODE			0x4
#define REG_DMA_CTL0_QUEUE0_CMD0	0x14
#define REG_DMA_CTL_QUEUE_STRIDE	0x10
#define REG_DMA_INTR_STATUS		0x160
#define REG_DMA_INTR_ERR_STATUS		0x170
#define REG_DMA_INTR_CLEAR		0x1a0
#define REG_DMA_INTR_ERR_CLEAR		0x1b0

#define REG_DMA_OPMODE_EN		BIT(0)

/* every error bit this revision of the engine can raise */
#define REG_DMA_ERROR_MASK		(BIT(0) | BIT(1) | BIT(2) | BIT(16) | \
					 BIT(17) | BIT(18) | BIT(19))

/*
 * The decode-select operation is itself an absolutely-addressed single-register
 * write aimed at the engine's own block-select register.  Decoded, the constant
 * is a single-register-write opcode with the absolute-range flag set and an
 * offset of 0xac114 from the display subsystem's base -- which is this device's
 * LUTDMA block (at offset 0xac000) plus 0x114.
 */
#define REG_DMA_DECODE_SEL_OP		0x180ac114

#define REG_DMA_OP_SINGLE_WRITE		BIT(28)
#define REG_DMA_MAX_REL_OFF		(BIT(21) - 1)
#define REG_DMA_MAX_DWORDS		(BIT(14) - 1)

/* the size/flags word written beside the buffer address at submission */
#define REG_DMA_CMD1_LAST		BIT(24)
#define REG_DMA_CMD1_WRITE		BIT(22)

/*
 * A queued transfer is started from the display controller's own CTL block
 * rather than from the engine, which is what couples the register writes to a
 * frame's flush instead of to whenever the CPU got round to it.
 */
#define REG_DMA_CTL_TRIGGER_OFF		0xd4
#define REG_DMA_QUEUE0_TRIGGER		BIT(0)

/*
 * Transfer-complete bit in REG_DMA_INTR_STATUS, per control block, for queue 0.
 * The assignment is not a formula -- the last two do not continue the run -- and
 * control blocks past the sixth have no bit at all, so a zero here means the
 * caller named a block this engine cannot report on and must not be waited for.
 */
static const u32 reg_dma_trigger_done_mask[CTL_MAX - CTL_0] = {
	[CTL_0 - CTL_0] = BIT(16),
	[CTL_1 - CTL_0] = BIT(17),
	[CTL_2 - CTL_0] = BIT(18),
	[CTL_3 - CTL_0] = BIT(19),
	[CTL_4 - CTL_0] = BIT(25),
	[CTL_5 - CTL_0] = BIT(26),
};

static u32 reg_dma_queue_off(u32 ctl_index)
{
	return REG_DMA_CTL0_QUEUE0_CMD0 + ctl_index * REG_DMA_CTL_QUEUE_STRIDE;
}

/**
 * dpu_reg_dma_init - map the engine and allocate its command buffer
 *
 * The command buffer is fetched by the engine as a bus master behind the
 * display subsystem's address translation, so it needs a device address from
 * the same space framebuffers are mapped into -- not a physical address.
 */
int dpu_reg_dma_init(struct dpu_kms *dpu_kms, struct dpu_reg_dma *rdma,
		     u32 buf_size)
{
	void *vaddr;

	if (rdma->ready)
		return 0;

	if (!buf_size || (buf_size / sizeof(u32)) > REG_DMA_MAX_DWORDS) {
		DPU_ERROR("invalid command buffer size %u\n", buf_size);
		return -EINVAL;
	}

	if (!dpu_kms->base.vm) {
		DPU_ERROR("no address space; engine needs one to fetch commands\n");
		return -ENODEV;
	}

	rdma->base = ioremap(DPU_REG_DMA_PHYS_BASE, DPU_REG_DMA_PHYS_LEN);
	if (!rdma->base) {
		DPU_ERROR("failed to map the engine at %#x\n",
			  DPU_REG_DMA_PHYS_BASE);
		return -ENOMEM;
	}

	rdma->db.blk_addr = rdma->base + DPU_REG_DMA_DB_OFF;
	rdma->db.log_mask = DPU_DBG_MASK_TOP;

	vaddr = msm_gem_kernel_new(dpu_kms->dev, buf_size, MSM_BO_WC,
				   dpu_kms->base.vm, &rdma->buf.bo,
				   &rdma->buf.iova);
	if (IS_ERR_OR_NULL(vaddr)) {
		DPU_ERROR("failed to allocate a command buffer: %ld\n",
			  PTR_ERR(vaddr));
		iounmap(rdma->base);
		rdma->base = NULL;
		return vaddr ? PTR_ERR(vaddr) : -ENOMEM;
	}

	rdma->buf.vaddr = vaddr;
	rdma->buf.size = buf_size;
	rdma->buf.index = 0;
	rdma->ready = true;

	DRM_INFO("engine mapped at %#x, %u-byte command buffer at iova %#llx\n",
		 DPU_REG_DMA_PHYS_BASE, buf_size, rdma->buf.iova);

	return 0;
}

void dpu_reg_dma_fini(struct dpu_kms *dpu_kms, struct dpu_reg_dma *rdma)
{
	if (!rdma->ready)
		return;

	rdma->ready = false;

	if (rdma->buf.bo) {
		msm_gem_kernel_put(rdma->buf.bo, dpu_kms->base.vm);
		rdma->buf.bo = NULL;
		rdma->buf.vaddr = NULL;
	}

	if (rdma->base) {
		iounmap(rdma->base);
		rdma->base = NULL;
	}
}

void dpu_reg_dma_buf_reset(struct dpu_reg_dma *rdma)
{
	rdma->buf.index = 0;
}

static int dpu_reg_dma_room(struct dpu_reg_dma *rdma, u32 bytes)
{
	if (!rdma->ready)
		return -ENODEV;

	if (rdma->buf.index + bytes > rdma->buf.size) {
		DPU_ERROR("command buffer full at %u of %u bytes\n",
			  rdma->buf.index, rdma->buf.size);
		return -ENOSPC;
	}

	return 0;
}

/**
 * dpu_reg_dma_decode_sel - name the blocks the following writes address
 *
 * Every offset written afterwards is relative to the selected block's base.
 */
int dpu_reg_dma_decode_sel(struct dpu_reg_dma *rdma, u32 blk_mask)
{
	u32 *loc;
	int ret;

	ret = dpu_reg_dma_room(rdma, 2 * sizeof(u32));
	if (ret)
		return ret;

	loc = (u32 *)((u8 *)rdma->buf.vaddr + rdma->buf.index);
	loc[0] = REG_DMA_DECODE_SEL_OP;
	loc[1] = blk_mask;
	rdma->buf.index += 2 * sizeof(u32);

	return 0;
}

/**
 * dpu_reg_dma_single_write - append one block-relative register write
 * @blk_off: offset inside the block named by the preceding decode-select
 */
int dpu_reg_dma_single_write(struct dpu_reg_dma *rdma, u32 blk_off, u32 val)
{
	u32 *loc;
	int ret;

	if (blk_off > REG_DMA_MAX_REL_OFF) {
		DPU_ERROR("offset %#x exceeds the block-relative range\n",
			  blk_off);
		return -EINVAL;
	}

	ret = dpu_reg_dma_room(rdma, 2 * sizeof(u32));
	if (ret)
		return ret;

	loc = (u32 *)((u8 *)rdma->buf.vaddr + rdma->buf.index);
	loc[0] = REG_DMA_OP_SINGLE_WRITE | (blk_off & REG_DMA_MAX_REL_OFF);
	loc[1] = val;
	rdma->buf.index += 2 * sizeof(u32);

	return 0;
}

/**
 * dpu_reg_dma_kick_off - submit the command buffer and wait for the engine
 * @trigger: false submits the transfer but never starts it, so a caller can
 *           establish that the effect it is about to look for really does
 *           depend on the engine having run
 *
 * Returns 0 once the hardware reports the transfer complete, -ETIMEDOUT if it
 * never does, and -EIO if the engine raised an error.
 */
int dpu_reg_dma_kick_off(struct dpu_kms *dpu_kms, struct dpu_reg_dma *rdma,
			 struct dpu_hw_ctl *ctl, bool trigger)
{
	u32 ctl_index, done_mask, cmd1, dwords, err, status;
	int i, ret = 0;

	if (!rdma->ready)
		return -ENODEV;

	if (!ctl || ctl->idx < CTL_0 || ctl->idx >= CTL_MAX) {
		DPU_ERROR("no usable control block\n");
		return -EINVAL;
	}

	if (!rdma->buf.index) {
		DPU_ERROR("nothing to submit\n");
		return -EINVAL;
	}

	ctl_index = ctl->idx - CTL_0;
	done_mask = reg_dma_trigger_done_mask[ctl_index];
	if (!done_mask) {
		DPU_ERROR("ctl%u has no completion bit on this engine\n", ctl_index);
		return -EINVAL;
	}
	dwords = rdma->buf.index / sizeof(u32);

	cmd1 = REG_DMA_CMD1_LAST | REG_DMA_CMD1_WRITE |
	       (dwords & REG_DMA_MAX_DWORDS);

	/*
	 * The engine's clock is gated off while nothing is using it; force it
	 * on across the transfer.
	 */
	if (dpu_kms->hw_mdp && dpu_kms->hw_mdp->ops.setup_clk_force_ctrl)
		dpu_kms->hw_mdp->ops.setup_clk_force_ctrl(dpu_kms->hw_mdp,
					DPU_CLK_CTRL_REG_DMA, true);

	DPU_REG_WRITE(&rdma->db, REG_DMA_OPMODE, REG_DMA_OPMODE_EN);

	/* clear anything left over, so neither reading below is a stale bit */
	err = DPU_REG_READ(&rdma->db, REG_DMA_INTR_ERR_STATUS);
	if (err)
		DPU_REG_WRITE(&rdma->db, REG_DMA_INTR_ERR_CLEAR,
			      REG_DMA_ERROR_MASK);
	DPU_REG_WRITE(&rdma->db, REG_DMA_INTR_CLEAR, done_mask);

	/* the command buffer must be visible to the engine before it is named */
	wmb();

	DPU_REG_WRITE(&rdma->db, reg_dma_queue_off(ctl_index),
		      lower_32_bits(rdma->buf.iova));
	DPU_REG_WRITE(&rdma->db, reg_dma_queue_off(ctl_index) + 0x4, cmd1);

	/* and the queue entry must be complete before the transfer starts */
	wmb();

	if (!trigger) {
		DRM_INFO("queued %u dwords on ctl%u, NOT triggered\n",
			 dwords, ctl_index);
		goto out;
	}

	DPU_REG_WRITE(&ctl->hw, REG_DMA_CTL_TRIGGER_OFF, REG_DMA_QUEUE0_TRIGGER);

	/* the engine acknowledges in microseconds; bound the wait at 1 ms */
	ret = -ETIMEDOUT;
	for (i = 0; i < 100; i++) {
		status = DPU_REG_READ(&rdma->db, REG_DMA_INTR_STATUS);
		if (status & done_mask) {
			ret = 0;
			break;
		}
		udelay(10);
	}

	err = DPU_REG_READ(&rdma->db, REG_DMA_INTR_ERR_STATUS);

	DRM_INFO("ctl%u: %u dwords from iova %#llx -> %s (status %#x err %#x after %d us)\n",
		 ctl_index, dwords, rdma->buf.iova,
		 ret ? "NO COMPLETION" : "complete",
		 DPU_REG_READ(&rdma->db, REG_DMA_INTR_STATUS), err, i * 10);

	if (!ret && (err & REG_DMA_ERROR_MASK))
		ret = -EIO;

	DPU_REG_WRITE(&rdma->db, REG_DMA_INTR_CLEAR, done_mask);
	if (err)
		DPU_REG_WRITE(&rdma->db, REG_DMA_INTR_ERR_CLEAR,
			      REG_DMA_ERROR_MASK);

out:
	if (dpu_kms->hw_mdp && dpu_kms->hw_mdp->ops.setup_clk_force_ctrl)
		dpu_kms->hw_mdp->ops.setup_clk_force_ctrl(dpu_kms->hw_mdp,
					DPU_CLK_CTRL_REG_DMA, false);

	return ret;
}

/* ------------------------------------------------------------------------ *
 * Engine self-test
 *
 * Proves the engine moves a byte, before anything is built on top of it.  A
 * colour-correction coefficient is chosen as the target because it is both
 * readable back through the CPU's own mapping and visible on the panel, so the
 * transfer is confirmed twice over two independent channels.
 * ------------------------------------------------------------------------ */

/* colour-correction block, mirroring the offsets in dpu_hw_dspp.c */
#define SELFTEST_PCC_EN			BIT(0)
#define SELFTEST_PCC_RED_R		0x10
#define SELFTEST_PCC_GREEN_G		0x20
#define SELFTEST_PCC_BLUE_B		0x30

#define SELFTEST_PCC_UNITY		0x8000

/*
 * The value the engine is asked to deliver.  Nothing else in the driver or in
 * userspace has any reason to write it, so finding it in the register afterwards
 * can only mean the engine put it there -- and it halves green, so it is plainly
 * visible on the panel too.
 */
#define SELFTEST_PCC_SENTINEL		0x4321

#define SELFTEST_RUN			BIT(0)
#define SELFTEST_NO_TRIGGER		BIT(1)

static int lutdma_selftest;
static int lutdma_selftest_pending;
static struct dpu_reg_dma lutdma_selftest_engine;

static int lutdma_selftest_set(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_int(val, kp);

	if (!ret)
		lutdma_selftest_pending = 1;
	return ret;
}

static const struct kernel_param_ops lutdma_selftest_ops = {
	.set = lutdma_selftest_set,
	.get = param_get_int,
};
module_param_cb(lutdma_selftest, &lutdma_selftest_ops, &lutdma_selftest, 0644);
MODULE_PARM_DESC(lutdma_selftest,
	"register-DMA engine self-test (1=run, 3=run without starting the engine (negative control), 0=restore)");

static u32 selftest_decode_sel_for(enum dpu_dspp idx)
{
	switch (idx) {
	case DSPP_0: return DPU_REG_DMA_BLK_DSPP0;
	case DSPP_1: return DPU_REG_DMA_BLK_DSPP1;
	case DSPP_2: return DPU_REG_DMA_BLK_DSPP2;
	case DSPP_3: return DPU_REG_DMA_BLK_DSPP3;
	default:     return 0;
	}
}

/**
 * dpu_reg_dma_selftest_run - run a pending engine self-test
 *
 * Called from the atomic commit so the register is read back in the same
 * context that submitted the transfer, before a later frame's colour
 * programming can overwrite it.  Does nothing unless the parameter was written
 * since the last commit.
 */
void dpu_reg_dma_selftest_run(struct dpu_kms *dpu_kms,
			      struct dpu_hw_dspp *dspp, struct dpu_hw_ctl *ctl)
{
	struct dpu_reg_dma *rdma = &lutdma_selftest_engine;
	u32 pcc, blk, before, after;
	bool trigger;
	int ret;

	if (!lutdma_selftest_pending)
		return;
	lutdma_selftest_pending = 0;

	if (!dspp || !ctl || !dspp->cap || !dspp->cap->sblk ||
	    !dspp->cap->sblk->pcc.base) {
		DPU_ERROR("no colour block to test against\n");
		return;
	}

	pcc = dspp->cap->sblk->pcc.base;

	/* a write of 0 restores the block and leaves the engine alone */
	if (!(lutdma_selftest & SELFTEST_RUN)) {
		DPU_REG_WRITE(&dspp->hw, pcc, 0);
		if (ctl->ops.update_pending_flush_dspp)
			ctl->ops.update_pending_flush_dspp(ctl, dspp->idx,
							   DPU_DSPP_PCC);
		DRM_INFO("selftest: colour block restored\n");
		return;
	}

	blk = selftest_decode_sel_for(dspp->idx);
	if (!blk) {
		DPU_ERROR("no decode-select bit for this colour block\n");
		return;
	}

	ret = dpu_reg_dma_init(dpu_kms, rdma, PAGE_SIZE);
	if (ret)
		return;

	/*
	 * Establish a known starting value through the CPU's own mapping, and
	 * confirm it landed.  Without this the read at the end cannot tell a
	 * transfer that did not happen from a register that was never anything
	 * else.
	 */
	DPU_REG_WRITE(&dspp->hw, pcc + SELFTEST_PCC_RED_R, SELFTEST_PCC_UNITY);
	DPU_REG_WRITE(&dspp->hw, pcc + SELFTEST_PCC_GREEN_G, SELFTEST_PCC_UNITY);
	DPU_REG_WRITE(&dspp->hw, pcc + SELFTEST_PCC_BLUE_B, SELFTEST_PCC_UNITY);
	DPU_REG_WRITE(&dspp->hw, pcc, SELFTEST_PCC_EN);
	before = DPU_REG_READ(&dspp->hw, pcc + SELFTEST_PCC_GREEN_G);

	/* the whole program: name the block, write one register inside it */
	dpu_reg_dma_buf_reset(rdma);
	ret = dpu_reg_dma_decode_sel(rdma, blk);
	if (!ret)
		ret = dpu_reg_dma_single_write(rdma,
					       pcc + SELFTEST_PCC_GREEN_G,
					       SELFTEST_PCC_SENTINEL);
	if (ret) {
		DPU_ERROR("failed to build the command buffer: %d\n", ret);
		return;
	}

	trigger = !(lutdma_selftest & SELFTEST_NO_TRIGGER);

	ret = dpu_reg_dma_kick_off(dpu_kms, rdma, ctl, trigger);

	after = DPU_REG_READ(&dspp->hw, pcc + SELFTEST_PCC_GREEN_G);

	if (ctl->ops.update_pending_flush_dspp)
		ctl->ops.update_pending_flush_dspp(ctl, dspp->idx,
						   DPU_DSPP_PCC);

	DRM_INFO("selftest: dspp%d ctl%d %s | before %#x expected %#x read %#x | %s | kick_off %d\n",
		 dspp->idx - DSPP_0, ctl->idx - CTL_0,
		 trigger ? "engine started" : "ENGINE NOT STARTED (control)",
		 before,
		 trigger ? SELFTEST_PCC_SENTINEL : SELFTEST_PCC_UNITY,
		 after,
		 after == (trigger ? SELFTEST_PCC_SENTINEL : SELFTEST_PCC_UNITY)
			? "PASS" : "FAIL",
		 ret);
}
