// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2026 The Linux Foundation. All rights reserved.
 *
 * Minimal LUTDMA (register-DMA) engine for the DPU.
 */

#define pr_fmt(fmt)	"[dpu-lutdma] " fmt

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include <drm/drm_color_mgmt.h>

#include "dpu_hw_reg_dma.h"
#include "dpu_hw_catalog.h"
#include "dpu_hw_ctl.h"
#include "dpu_hw_dspp.h"
#include "dpu_hw_sspp.h"
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
#define DPU_REG_DMA_ADDR_ALIGN		256

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
#define REG_DMA_OP_SINGLE_MODIFY	BIT(29)
/* writes a run of registers, the engine advancing the address itself */
#define REG_DMA_OP_BLOCK_WRITE		BIT(30)
/* writes a run of values through one address, the block advancing its own index */
#define REG_DMA_OP_INDEXED_WRITE	(BIT(28) | BIT(29))
/*
 * streams a table into a colour block's own table memory -- the LUT bus --
 * rather than into its registers; the encoding is the device's own downstream
 * driver's (sde_hw_reg_dma_v1.c, write_block_lut_reg)
 */
#define REG_DMA_OP_LUTBUS_WRITE		(BIT(30) | BIT(29))
#define REG_DMA_LUTBUS_TABLE_SEL_MASK	0x10000
#define REG_DMA_LUTBUS_BLOCK_SEL_MASK	0xffff
#define REG_DMA_LUTBUS_TRANS_SZ_MASK	0xff0000
#define REG_DMA_LUTBUS_LUT_SIZE_MASK	0x3fff
#define REG_DMA_OP_NOP			0
/* a LUT-bus payload's residual data plus the next opcode must exceed four dwords */
#define REG_DMA_LUTBUS_NOPS		3
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
 * How long a submission is waited for, its completion read every 10 us.  The
 * inverse-gamma tables complete in microseconds and are given 1 ms.  The video
 * pipes' gamut lattice, some 18 times larger, is given as long as the device's
 * own driver waits for a completion -- 20 ms (sde_hw_reg_dma_v1.c, the last
 * command's REG_DMA_WAIT4_COMP).  A transfer that outlives its wait is not
 * stopped -- nothing here resets the queue -- so it may still land.
 */
#define REG_DMA_DONE_POLL_US		10
#define REG_DMA_WAIT_US			1000
#define REG_DMA_LUT_WAIT_US		20000

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

	/*
	 * The engine fetches from an aligned address. Page-granular allocation
	 * satisfies this today, which is a reason to check it rather than a
	 * reason to assume it.
	 */
	WARN_ON(rdma->buf.iova & (DPU_REG_DMA_ADDR_ALIGN - 1));

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

/*
 * The two run-writing operations share a header: an opcode carrying the
 * block-relative offset, a word giving the payload length, then the payload.
 * They differ only in what the engine does with the address between values.
 */
static int dpu_reg_dma_run(struct dpu_reg_dma *rdma, u32 opcode, u32 blk_off,
			   const u32 *data, u32 count)
{
	u32 payload = count * sizeof(u32);
	u32 *loc;
	int ret;

	if (blk_off > REG_DMA_MAX_REL_OFF) {
		DPU_ERROR("offset %#x exceeds the block-relative range\n", blk_off);
		return -EINVAL;
	}

	if (!count || count > REG_DMA_MAX_DWORDS) {
		DPU_ERROR("invalid run length %u\n", count);
		return -EINVAL;
	}

	ret = dpu_reg_dma_room(rdma, 2 * sizeof(u32) + payload);
	if (ret)
		return ret;

	loc = (u32 *)((u8 *)rdma->buf.vaddr + rdma->buf.index);
	loc[0] = opcode | (blk_off & REG_DMA_MAX_REL_OFF);
	loc[1] = count;
	rdma->buf.index += 2 * sizeof(u32);

	memcpy((u8 *)rdma->buf.vaddr + rdma->buf.index, data, payload);
	rdma->buf.index += payload;

	return 0;
}

/**
 * dpu_reg_dma_block_write - write consecutive registers from @blk_off onwards
 *
 * The engine advances the address itself, one register per value.
 */
int dpu_reg_dma_block_write(struct dpu_reg_dma *rdma, u32 blk_off,
			    const u32 *data, u32 count)
{
	return dpu_reg_dma_run(rdma, REG_DMA_OP_BLOCK_WRITE, blk_off, data, count);
}

/**
 * dpu_reg_dma_indexed_write - write a run of values through ONE register
 *
 * The address does not move; the block keeps its own index and advances it on
 * each write. This is how the source pipes' lookup tables are loaded, and it is
 * the reason those tables cannot sensibly be written by the CPU: the whole run
 * has to land as one unit, in order, alongside a frame.
 */
int dpu_reg_dma_indexed_write(struct dpu_reg_dma *rdma, u32 blk_off,
			      const u32 *data, u32 count)
{
	return dpu_reg_dma_run(rdma, REG_DMA_OP_INDEXED_WRITE, blk_off, data, count);
}

/**
 * dpu_reg_dma_single_modify - read-modify-write one register under @mask
 *
 * Three words rather than two: the mask sits between the opcode and the value.
 */
int dpu_reg_dma_single_modify(struct dpu_reg_dma *rdma, u32 blk_off, u32 mask,
			      u32 val)
{
	u32 *loc;
	int ret;

	if (blk_off > REG_DMA_MAX_REL_OFF) {
		DPU_ERROR("offset %#x exceeds the block-relative range\n", blk_off);
		return -EINVAL;
	}

	ret = dpu_reg_dma_room(rdma, 3 * sizeof(u32));
	if (ret)
		return ret;

	loc = (u32 *)((u8 *)rdma->buf.vaddr + rdma->buf.index);
	loc[0] = REG_DMA_OP_SINGLE_MODIFY | (blk_off & REG_DMA_MAX_REL_OFF);
	loc[1] = mask;
	loc[2] = val;
	rdma->buf.index += 3 * sizeof(u32);

	return 0;
}

/**
 * dpu_reg_dma_lutbus_write - stream a table into a colour block's LUT bus
 * @block:      the LUT-bus block, DPU_REG_DMA_LUTBUS_IGC / _GAMUT / _SIXZONE
 * @table:      which of the block's two tables receives the data
 * @trans_size: the block's transfer granule in dwords, DPU_REG_DMA_LUTBUS_*_TRANS
 * @data:       the table, @count dwords -- a whole number of transfers
 *
 * Unlike the other writes this one names no register: the payload goes to the
 * table memory of the block named in the header, in the pipe chosen by the
 * preceding decode-select.  Three no-op words follow it.
 */
int dpu_reg_dma_lutbus_write(struct dpu_reg_dma *rdma, u32 block, u32 table,
			     u32 trans_size, const u32 *data, u32 count)
{
	u32 payload = count * sizeof(u32);
	u32 nops = REG_DMA_LUTBUS_NOPS * sizeof(u32);
	u32 *loc;
	u32 i;
	int ret;

	if (table >= DPU_REG_DMA_LUTBUS_TABLES ||
	    block >= DPU_REG_DMA_LUTBUS_BLOCKS ||
	    (trans_size != DPU_REG_DMA_LUTBUS_IGC_TRANS &&
	     trans_size != DPU_REG_DMA_LUTBUS_GAMUT_TRANS &&
	     trans_size != DPU_REG_DMA_LUTBUS_SIXZONE_TRANS)) {
		DPU_ERROR("invalid LUT-bus write: table %u block %u transfer %u\n",
			  table, block, trans_size);
		return -EINVAL;
	}

	if (!data || !count || count > REG_DMA_MAX_DWORDS ||
	    count % trans_size) {
		DPU_ERROR("invalid LUT-bus payload of %u dwords\n", count);
		return -EINVAL;
	}

	ret = dpu_reg_dma_room(rdma, 2 * sizeof(u32) + payload + nops);
	if (ret)
		return ret;

	loc = (u32 *)((u8 *)rdma->buf.vaddr + rdma->buf.index);
	loc[0] = REG_DMA_OP_LUTBUS_WRITE |
		 ((table << 16) & REG_DMA_LUTBUS_TABLE_SEL_MASK) |
		 (block & REG_DMA_LUTBUS_BLOCK_SEL_MASK);
	loc[1] = ((trans_size << 16) & REG_DMA_LUTBUS_TRANS_SZ_MASK) |
		 ((count / trans_size) & REG_DMA_LUTBUS_LUT_SIZE_MASK);
	rdma->buf.index += 2 * sizeof(u32);

	memcpy((u8 *)rdma->buf.vaddr + rdma->buf.index, data, payload);
	rdma->buf.index += payload;

	loc = (u32 *)((u8 *)rdma->buf.vaddr + rdma->buf.index);
	for (i = 0; i < REG_DMA_LUTBUS_NOPS; i++)
		loc[i] = REG_DMA_OP_NOP;
	rdma->buf.index += nops;

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
static int dpu_reg_dma_kick_off_wait(struct dpu_kms *dpu_kms, struct dpu_reg_dma *rdma,
				     struct dpu_hw_ctl *ctl, bool trigger, u32 wait_us)
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
		/*
		 * Read the engine's own status even here. Without it the control
		 * can only say the register did not change, which is also what a
		 * transfer that ran and wrote somewhere else would look like.
		 */
		status = DPU_REG_READ(&rdma->db, REG_DMA_INTR_STATUS);
		err = DPU_REG_READ(&rdma->db, REG_DMA_INTR_ERR_STATUS);
		DRM_INFO("ctl%u: %u dwords queued at iova %#llx, NOT triggered (status %#x err %#x -- both must stay clear)\n",
			 ctl_index, dwords, rdma->buf.iova, status, err);
		goto out;
	}

	DPU_REG_WRITE(&ctl->hw, REG_DMA_CTL_TRIGGER_OFF, REG_DMA_QUEUE0_TRIGGER);

	/* the engine acknowledges in microseconds; wait as long as the caller says */
	ret = -ETIMEDOUT;
	for (i = 0; i < wait_us / REG_DMA_DONE_POLL_US; i++) {
		status = DPU_REG_READ(&rdma->db, REG_DMA_INTR_STATUS);
		if (status & done_mask) {
			ret = 0;
			break;
		}
		udelay(REG_DMA_DONE_POLL_US);
	}

	err = DPU_REG_READ(&rdma->db, REG_DMA_INTR_ERR_STATUS);

	DRM_INFO("ctl%u: %u dwords from iova %#llx -> %s (status %#x err %#x after %d us)\n",
		 ctl_index, dwords, rdma->buf.iova,
		 ret ? "NO COMPLETION" : "complete",
		 DPU_REG_READ(&rdma->db, REG_DMA_INTR_STATUS), err, i * REG_DMA_DONE_POLL_US);

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

int dpu_reg_dma_kick_off(struct dpu_kms *dpu_kms, struct dpu_reg_dma *rdma,
			 struct dpu_hw_ctl *ctl, bool trigger)
{
	return dpu_reg_dma_kick_off_wait(dpu_kms, rdma, ctl, trigger, REG_DMA_WAIT_US);
}

/* ------------------------------------------------------------------------ *
 * The video pipes' inverse-gamma table
 *
 * A DRM plane colour pipeline's second operation is a 1D lookup table, and on
 * this part that table lives in the source pipe's inverse-gamma block.  The
 * block has no CPU write path -- writing it is the reason this engine exists.
 *
 * 🔴 THE OFFSETS BELOW ARE DELIBERATELY A SECOND COPY of the ones the self-test
 * at the end of this file names SELFTEST_VIG_IGC_*.  Sharing them would let an
 * instrument and the feature it is supposed to prove fail together on the same
 * wrong constant, which is the one thing a control must not do.  The self-test
 * also hardcodes its block base while this path takes it from the catalogue, so
 * the two disagree loudly if the catalogue is wrong rather than agreeing
 * quietly.
 * ------------------------------------------------------------------------ */

/*
 * The video pipe class's anchor shift.  This device's device tree declares
 * qcom,sde-vig-top-off = <0xa00>, and the hardware's command decoder anchors a
 * source pipe's sub-blocks that much lower than the register mapping does.  A
 * command buffer built with the CPU's offset writes 0xa00 too high, and the
 * read-back then looks exactly like a transfer that never happened.
 *
 * 🔴 IT IS A PROPERTY OF THE PIPE CLASS, NOT A CONSTANT: the DMA pipes' shift is
 * 0x800.  This function is for VIG pipes and asserts as much.
 */
#define DPU_VIG_TOP_OFF			0xa00

/*
 * Inside the block: bit 8 enables the lookup table and bit 9 selects which of
 * two banks it reads, while bits 0-4 and 16 belong to gamut, share the
 * register, and have to survive -- which is what the mask is for.  The masked
 * modify keeps the bits named in the mask and takes the rest from the value.
 */
#define DPU_VIG_IGC_OPMODE		0x000
#define DPU_VIG_IGC_OP_MASK		0x1001f
#define DPU_VIG_IGC_LUT_ENABLE		BIT(8)
#define DPU_VIG_IGC_LUT_SEL		BIT(9)

/* the table's index and data ports, and the dither the vendor writes beside them */
#define DPU_VIG_IGC_INDEX		0x1b0
#define DPU_VIG_IGC_DATA		0x1b4
#define DPU_VIG_IGC_DITHER		0x1c0
/*
 * 🔴 DITHER OFF -- the vendor's value unless its client asks for dither
 * (sde_hw_reg_dma_v1_color_proc.c: strength | BIT(4) only on IGC_DITHER_ENABLE,
 * else 0), and a DRM colour pipeline has no way to ask.  This used to be 0x16
 * (enable + strength 6): the 482 register self-test's SENTINEL, carried into
 * every table load.  Measured in the pixels (the layer-mixer CRC decoded on
 * the grey dictionary): with 0x16 no plane grey below a PQ curve's clip was a
 * solid ten-bit value, black was lifted and white arrived late; with 0 every
 * grey was solid, black exact, and the clip where the curve puts it.  0x10
 * (enable, strength 0) is NOT off: it still moved most outputs.
 */
#define DPU_VIG_IGC_DITHER_OFF		0

/* the separately held endpoint, one register a channel */
#define DPU_VIG_IGC_LAST		0x1c4
#define DPU_VIG_IGC_LAST_N		3

/* 256 twelve-bit entries a channel, packed two to a dword, three channels */
#define DPU_VIG_IGC_DWORDS		128
#define DPU_VIG_IGC_CHANNELS		3
#define DPU_VIG_IGC_ENTRY_MASK		(BIT(12) - 1)

/* an entry chosen away from both ends, so a read-back names what it read */
#define DPU_VIG_IGC_PROBE_INDEX		64

/* how many table differences a load's report names before it says "..." */
#define DPU_VIG_IGC_REPORT_MAX		8

/*
 * 🔴 EVERY CHANNEL'S RUN IS CLOSED BY ITS OWN ENDPOINT WRITE -- or its last word is lost.
 *
 * The block's data port behaves as a one-deep write pipeline.  Measured, fresh
 * loads, both pipes, n=2 for each shape of the program: each data beat is
 * committed by the NEXT beat on that port, or by a write to that channel's
 * endpoint register; the next write to the shared index register DISCARDS the
 * beat still held; neither time (eight writes) nor a write to any other
 * register (the dither, an opmode write keeping every bit) commits it.
 *
 * The device's own program resets the index for the next channel straight
 * after each run and writes the three endpoints only at the very end, so the
 * last word of channels 0 and 1 -- entries 254 and 255, the highlights -- never
 * landed, on every load (382 of 384; reversed, the misses moved to channels 2
 * and 1, and the channel written last kept its word).  Writing each channel's
 * endpoint right after its run commits that word: all 384 land.  The final
 * endpoint block write is kept as the device issues it.
 *
 * A TEST SWITCH keeps the device's own program available as a control, off by
 * default: 1 runs it as it was, so the loss it causes can be shown in the same
 * boot as the fix.
 */
static int dpu_reg_dma_igc_test_program;
module_param_named(colorop_igc_test_program, dpu_reg_dma_igc_test_program, int, 0644);
MODULE_PARM_DESC(colorop_igc_test_program,
	"TEST ONLY, default 0: 1 loads the inverse-gamma table with the device's own program, which loses the last word of channels 0 and 1 -- a control for the fixed program");

/* one endpoint register a channel -- the loop below indexes them by channel */
static_assert(DPU_VIG_IGC_LAST_N == DPU_VIG_IGC_CHANNELS,
	      "one endpoint register per channel");

/*
 * The video pipes' gamut block: a 17-point 3D lookup table behind a
 * per-channel input map of 3 x 16 words.  The device's DT declares it at the
 * same base as inverse gamma (<0x1d00 0x60001> beside <0x1d00 0x60000>), so
 * the two share the op-mode word there: bits 0-4 are the gamut's -- enable,
 * map enable and the mode (17 = 0, 17b = 4), which also names the table on
 * display -- and 8-9 inverse gamma's.  The values are the device's own
 * downstream driver's (reg_dmav2_setup_vig_gamutv61, sde_hw_reg_dma_v1_color_proc.c,
 * and sde_hw_color_proc_common_v4.h).
 */
#define DPU_VIG_GAMUT_EN		BIT(0)
#define DPU_VIG_GAMUT_MAP_EN		BIT(1)
#define DPU_VIG_GAMUT_OP_BITS		0x1f
#define DPU_VIG_GAMUT_TABLE_A		0
#define DPU_VIG_GAMUT_TABLE_B		1
#define DPU_VIG_GAMUT_MODE_SHIFT	2	/* the op word's mode, bits 4:2 */
#define DPU_VIG_GAMUT_MODE_17B		4	/* gamut_mode_17b: table B shown; gamut_mode_17 = 0, A */
#define DPU_VIG_GAMUT_SCALEA		0x10
#define DPU_VIG_GAMUT_SCALE_STRIDE	0x40
#define DPU_VIG_GAMUT_MAP_WORDS		16
#define DPU_VIG_GAMUT_MAP_CHANNELS	3
#define DPU_VIG_GAMUT_TEST_MAX		11

/*
 * TEST switch: at the next colour flush, program bank A of every colour-
 * pipeline video pipe's gamut -- 1 a lattice of grey 512, 2 a grey lattice
 * graded along the fastest lattice digit, 3 = 2 with the input map enabled
 * and loaded raw from colorop_gamut_map, 4 / 5 / 6 = 2's grading in output
 * word c0 / c1 / c2 alone, 7 a grey lattice graded along the middle digit,
 * 8 = 7 with the map, 9 graded along the slowest digit, 10 = 9 with the map,
 * 11 the stock table held in debugfs (vig_gamut_test_blob) with its own map
 * -- and switch it off again on 0.  A measurement switch for the hardware
 * arms only: nothing else drives the block.
 */
static int dpu_reg_dma_gamut_test;
module_param_named(colorop_gamut_test, dpu_reg_dma_gamut_test, int, 0644);
MODULE_PARM_DESC(colorop_gamut_test,
		 "TEST: program the video planes' gamut, bank A: 0 off (default), 1 grey 512, 2 graded on the fastest digit, 3 = 2 + colorop_gamut_map, 4/5/6 = 2 in word c0/c1/c2 only, 7 graded on the middle digit, 8 = 7 + colorop_gamut_map, 9 graded on the slowest digit, 10 = 9 + colorop_gamut_map, 11 the stock table written to debugfs vig_gamut_test_blob");

static u32 dpu_reg_dma_gamut_map[DPU_VIG_GAMUT_MAP_CHANNELS * DPU_VIG_GAMUT_MAP_WORDS];
static int dpu_reg_dma_gamut_map_n;
module_param_array_named(colorop_gamut_map, dpu_reg_dma_gamut_map, uint,
			 &dpu_reg_dma_gamut_map_n, 0644);
MODULE_PARM_DESC(colorop_gamut_map, "TEST: the 48 input-map words colorop_gamut_test=3, 8 and 10 write raw");

int dpu_reg_dma_gamut_test_mode(void)
{
	return READ_ONCE(dpu_reg_dma_gamut_test);
}

const u32 *dpu_reg_dma_gamut_test_map(void)
{
	if (READ_ONCE(dpu_reg_dma_gamut_map_n) != ARRAY_SIZE(dpu_reg_dma_gamut_map))
		return NULL;
	return dpu_reg_dma_gamut_map;
}

/**
 * dpu_vig_gamut_test_lattice - a test lattice, packed as the LUT bus takes it
 * @mode: 1 every entry grey 512; 2 or 3 grey round(k x 1023 / 16), k = L mod 17
 *        of the linear index L = 4j + i -- graded along the fastest lattice digit;
 *        4, 5, 6 the same grading in ONE output word -- c0, c1, c2 -- the others 0;
 *        7 or 8 graded along the middle digit, k = (L / 17) mod 17; 9 or 10 along
 *        the slowest, k = (L / 289) mod 17 (the mod keeps the stream's three
 *        entries past 17^3 in range)
 * @out: DPU_VIG_GAMUT_DWORDS dwords
 *
 * The downstream setter's order: for each entry j, the four tables i, each as
 * three 16-bit words c0, c2, c1 carrying a 10-bit value MSB-aligned (<< 6).
 */
void dpu_vig_gamut_test_lattice(int mode, u32 *out)
{
	u16 *data = (u16 *)out;
	u32 step = mode >= 9 ? 289 : mode >= 7 ? 17 : 1;
	bool all = mode < 4 || mode > 6;
	u32 i, j, k, v;

	for (j = 0; j < DPU_VIG_GAMUT_TBL_SZ; j++) {
		for (i = 0; i < DPU_VIG_GAMUT_TBL_NUM; i++) {
			k = (4 * j + i) / step % 17;
			v = (mode == 1 ? 512 : DIV_ROUND_CLOSEST(k * 1023, 16)) << 6;
			*data++ = (u16)(all || mode == 4 ? v : 0);	/* c0 */
			*data++ = (u16)(all || mode == 6 ? v : 0);	/* c2 */
			*data++ = (u16)(all || mode == 5 ? v : 0);	/* c1 */
		}
	}
}

/*
 * A stock gamut table for colorop_gamut_test=11: the device's own downstream
 * struct drm_msm_3d_gamut (msm_drm_pp.h :190-:229, DZA1_v3.1.0), taken
 * verbatim through debugfs -- flags u64 (bit 0 the input map), mode u32 (1, the
 * 17-point lattice), 3 x 16 input-map words, then col[4][1229] of {c2_c1, c0},
 * all little-endian, 39,536 bytes with the tail pad.
 */
#define DPU_VIG_GAMUT_BLOB_FLAGS	0
#define DPU_VIG_GAMUT_BLOB_MODE		8
#define DPU_VIG_GAMUT_BLOB_MAP		12
#define DPU_VIG_GAMUT_BLOB_COL		204
#define DPU_VIG_GAMUT_BLOB_MODE_17	1
#define DPU_VIG_GAMUT_BLOB_MAP_EN	BIT(0)

static_assert(DPU_VIG_GAMUT_BLOB_MAP_WORDS == DPU_VIG_GAMUT_MAP_CHANNELS * DPU_VIG_GAMUT_MAP_WORDS);
static_assert(DPU_VIG_GAMUT_BLOB_COL == DPU_VIG_GAMUT_BLOB_MAP + 4 * DPU_VIG_GAMUT_BLOB_MAP_WORDS);
static_assert(DPU_VIG_GAMUT_BLOB_SIZE == ALIGN(DPU_VIG_GAMUT_BLOB_COL + 8 * DPU_VIG_GAMUT_BLOB_ENTRIES, 8));

/* entry n = i x 1229 + j of the table's col[4][1229] */
static void dpu_vig_gamut_blob_entry(const u8 *blob, u32 n, u32 *c0, u32 *c1, u32 *c2)
{
	const u8 *e = blob + DPU_VIG_GAMUT_BLOB_COL + 8 * n;
	u32 c2_c1 = get_unaligned_le32(e);

	*c0 = get_unaligned_le32(e + 4);
	*c1 = c2_c1 & 0xffff;
	*c2 = c2_c1 >> 16;
}

/**
 * dpu_vig_gamut_blob_check - is this a stock gamut table the LUT bus can take
 * @blob: the bytes written
 * @len: how many
 *
 * Return: 0, or -EINVAL for the wrong size, a mode other than the 17-point
 * lattice, a flag other than the input map's, or a component past 10 bits.
 */
int dpu_vig_gamut_blob_check(const u8 *blob, size_t len)
{
	u32 n, c0, c1, c2;

	if (len != DPU_VIG_GAMUT_BLOB_SIZE)
		return -EINVAL;
	if (get_unaligned_le32(blob + DPU_VIG_GAMUT_BLOB_MODE) != DPU_VIG_GAMUT_BLOB_MODE_17)
		return -EINVAL;
	if (get_unaligned_le64(blob + DPU_VIG_GAMUT_BLOB_FLAGS) & ~(u64)DPU_VIG_GAMUT_BLOB_MAP_EN)
		return -EINVAL;
	for (n = 0; n < DPU_VIG_GAMUT_BLOB_ENTRIES; n++) {
		dpu_vig_gamut_blob_entry(blob, n, &c0, &c1, &c2);
		if (c0 > 1023 || c1 > 1023 || c2 > 1023)
			return -EINVAL;
	}
	return 0;
}

/*
 * One lattice entry as the LUT bus takes it -- the downstream setter's order,
 * c0, c2, c1, each a 10-bit value MSB-aligned in its 16-bit word (DZA1
 * :4799-:4802).  A 3D-LUT colour operation would build its stream through it too.
 */
static u16 *dpu_vig_gamut_pack(u16 *data, u32 c0, u32 c1, u32 c2)
{
	*data++ = (u16)(c0 << 6);
	*data++ = (u16)(c2 << 6);
	*data++ = (u16)(c1 << 6);
	return data;
}

/**
 * dpu_vig_gamut_blob_lattice - a stock table's lattice, packed for the LUT bus
 * @blob: a table dpu_vig_gamut_blob_check() passed
 * @out: DPU_VIG_GAMUT_DWORDS dwords
 *
 * For each entry j, the four tables i -- stream position 4j + i -- as
 * dpu_vig_gamut_test_lattice() packs its own.
 */
void dpu_vig_gamut_blob_lattice(const u8 *blob, u32 *out)
{
	u16 *data = (u16 *)out;
	u32 i, j, c0, c1, c2;

	for (j = 0; j < DPU_VIG_GAMUT_TBL_SZ; j++) {
		for (i = 0; i < DPU_VIG_GAMUT_TBL_NUM; i++) {
			dpu_vig_gamut_blob_entry(blob, i * DPU_VIG_GAMUT_TBL_SZ + j, &c0, &c1, &c2);
			data = dpu_vig_gamut_pack(data, c0, c1, c2);
		}
	}
}

/**
 * dpu_vig_gamut_from_lut32 - a DRM 3D-LUT colour operation's data, packed for the LUT bus
 * @lut: DPU_VIG_GAMUT_POINTS entries, DRM's index R + 17 (G + 17 B) (drm_mode.h)
 * @out: DPU_VIG_GAMUT_DWORDS dwords
 *
 * The lattice's linear index is DRM's own -- the stream position 4j + i is the
 * index, R its fastest digit, G the middle, B the slowest (measured on the
 * hardware) -- so entry L lands at stream position L.  The words are c0 = R,
 * c1 = B, c2 = G (measured likewise), each rounded to the lattice's 10 bits.
 * The three positions past 17^3 that complete the last group of four are 0.
 */
void dpu_vig_gamut_from_lut32(const struct drm_color_lut32 *lut, u32 *out)
{
	u16 *data = (u16 *)out;
	u32 L;

	for (L = 0; L < DPU_VIG_GAMUT_TBL_NUM * DPU_VIG_GAMUT_TBL_SZ; L++) {
		if (L < DPU_VIG_GAMUT_POINTS) {
			const struct drm_color_lut32 *e = &lut[L];

			data = dpu_vig_gamut_pack(data, drm_color_lut32_extract(e->red, 10),
						  drm_color_lut32_extract(e->blue, 10),
						  drm_color_lut32_extract(e->green, 10));
		} else {
			data = dpu_vig_gamut_pack(data, 0, 0, 0);
		}
	}
}

/**
 * dpu_vig_gamut_blob_map - a stock table's input map
 * @blob: a table dpu_vig_gamut_blob_check() passed
 * @map: DPU_VIG_GAMUT_BLOB_MAP_WORDS words, the three channels' 16 in turn, as stored
 *
 * Return: true when the table enables the map (flags bit 0).
 */
bool dpu_vig_gamut_blob_map(const u8 *blob, u32 *map)
{
	u32 k;

	for (k = 0; k < DPU_VIG_GAMUT_BLOB_MAP_WORDS; k++)
		map[k] = get_unaligned_le32(blob + DPU_VIG_GAMUT_BLOB_MAP + 4 * k);
	return get_unaligned_le64(blob + DPU_VIG_GAMUT_BLOB_FLAGS) & DPU_VIG_GAMUT_BLOB_MAP_EN;
}

static u8 *dpu_vig_gamut_blob;		/* the held table, NULL until one is written */
static DEFINE_MUTEX(dpu_vig_gamut_blob_lock);

/**
 * dpu_reg_dma_gamut_test_blob - the held stock table, packed for colorop_gamut_test=11
 * @lattice: DPU_VIG_GAMUT_DWORDS dwords
 * @map: DPU_VIG_GAMUT_BLOB_MAP_WORDS words
 *
 * Return: 1 the table enables its input map, 0 it does not, -ENOENT none is held.
 */
int dpu_reg_dma_gamut_test_blob(u32 *lattice, u32 *map)
{
	int ret = -ENOENT;

	mutex_lock(&dpu_vig_gamut_blob_lock);
	if (dpu_vig_gamut_blob) {
		dpu_vig_gamut_blob_lattice(dpu_vig_gamut_blob, lattice);
		ret = dpu_vig_gamut_blob_map(dpu_vig_gamut_blob, map);
	}
	mutex_unlock(&dpu_vig_gamut_blob_lock);
	return ret;
}

/*
 * The debugfs node colorop_gamut_test=11 takes its table from: one write of
 * the whole struct at offset 0, refused while the switch is on -- the hook does
 * nothing for an unchanged mode with the block on, so a table written under a
 * held 11 would never load -- and read back whole.
 */
static ssize_t dpu_vig_gamut_blob_write(struct file *file, const char __user *ubuf,
					size_t len, loff_t *ppos)
{
	u8 *blob;
	int ret;

	if (*ppos || len != DPU_VIG_GAMUT_BLOB_SIZE)
		return -EINVAL;
	if (READ_ONCE(dpu_reg_dma_gamut_test))
		return -EBUSY;
	blob = vmemdup_user(ubuf, len);
	if (IS_ERR(blob))
		return PTR_ERR(blob);
	ret = dpu_vig_gamut_blob_check(blob, len);
	if (ret) {
		kvfree(blob);
		return ret;
	}
	mutex_lock(&dpu_vig_gamut_blob_lock);
	swap(blob, dpu_vig_gamut_blob);
	mutex_unlock(&dpu_vig_gamut_blob_lock);
	kvfree(blob);
	*ppos = len;
	return len;
}

static ssize_t dpu_vig_gamut_blob_read(struct file *file, char __user *ubuf,
				       size_t len, loff_t *ppos)
{
	ssize_t ret = 0;

	mutex_lock(&dpu_vig_gamut_blob_lock);
	if (dpu_vig_gamut_blob)
		ret = simple_read_from_buffer(ubuf, len, ppos, dpu_vig_gamut_blob,
					      DPU_VIG_GAMUT_BLOB_SIZE);
	mutex_unlock(&dpu_vig_gamut_blob_lock);
	return ret;
}

static const struct file_operations dpu_vig_gamut_blob_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = dpu_vig_gamut_blob_read,
	.write = dpu_vig_gamut_blob_write,
	.llseek = default_llseek,
};

/**
 * dpu_reg_dma_debugfs_init - add the node a stock gamut table is written to
 * @parent: the DPU's debugfs directory
 */
void dpu_reg_dma_debugfs_init(struct dentry *parent)
{
	debugfs_create_file("vig_gamut_test_blob", 0600, parent, NULL, &dpu_vig_gamut_blob_fops);
}

/**
 * dpu_reg_dma_gamut_bank - which bank a map-less gamut load writes: the device's own rule
 * @op:    a video pipe's op word as it reads now
 * @table: filled with the LUT-bus table to write -- the one @op does NOT show
 *
 * The v6.1 setter (sde_hw_reg_dma_v1_color_proc.c, DZA1_v3.1.0) reads the mode in bits 4:2 and
 * writes table A when it is gamut_mode_17b (table B shown), table B otherwise -- a cleared word
 * included, so after a switch-off or a power loss the first load goes to B -- and flips the mode
 * with the block on.  Unlike inverse gamma, whose engine picks the bank, the LUT bus names it.
 *
 * Return: the op word's bits 0-4 that show the bank written, the block on.
 */
u32 dpu_reg_dma_gamut_bank(u32 op, u32 *table)
{
	if (((op & DPU_VIG_GAMUT_OP_BITS) >> DPU_VIG_GAMUT_MODE_SHIFT) == DPU_VIG_GAMUT_MODE_17B) {
		*table = DPU_VIG_GAMUT_TABLE_A;
		return DPU_VIG_GAMUT_EN;
	}
	*table = DPU_VIG_GAMUT_TABLE_B;
	return DPU_VIG_GAMUT_EN | DPU_VIG_GAMUT_MODE_17B << DPU_VIG_GAMUT_MODE_SHIFT;
}

/**
 * dpu_reg_dma_sspp_gamut_op - a video pipe's gamut / inverse-gamma op word as it reads now
 */
u32 dpu_reg_dma_sspp_gamut_op(struct dpu_hw_sspp *sspp)
{
	return DPU_REG_READ(&sspp->hw, sspp->cap->sblk->igc_blk.base + DPU_VIG_IGC_OPMODE);
}

/**
 * dpu_reg_dma_program_sspp_gamut - load a video pipe's gamut and enable it
 * @lattice: DPU_VIG_GAMUT_DWORDS dwords, as dpu_vig_gamut_test_lattice() packs them
 * @map: the 3 x 16 input-map words, written raw with the map enabled; NULL for none
 *
 * Without a map -- the 3D-LUT colour operation, which can change every frame -- the
 * device's own ping-pong: the bank the live mode does not show is written and the
 * mode flipped to it in the same transfer, so the table on display is never
 * rewritten under the scan (dpu_reg_dma_gamut_bank).  With a map -- the test switch
 * only -- bank A (table 0, SCALEA, mode 17), the block switched off first: bank B's
 * map words would sit past the block's declared 0x1dff.  The op word is modified,
 * never written whole as the vendor writes it: inverse gamma's enable lives in it.
 * Kicked off as the inverse gamma's is -- queue 0, write-immediate, the class the
 * vendor gives both.
 *
 * Return: 0 when the engine acknowledged the transfer, negative otherwise.
 */
int dpu_reg_dma_program_sspp_gamut(struct dpu_kms *dpu_kms, struct dpu_reg_dma *rdma,
				   struct dpu_hw_ctl *ctl, struct dpu_hw_sspp *sspp,
				   const u32 *lattice, const u32 *map)
{
	u32 blk, op_off, op, table = DPU_VIG_GAMUT_TABLE_A;
	int ch, ret;

	if (!sspp->cap->sblk->igc_blk.len)
		return -ENODEV;
	blk = dpu_reg_dma_blk_for_sspp(sspp->idx);
	if (!blk)
		return -ENODEV;
	if (sspp->cap->sblk->igc_blk.base < DPU_VIG_TOP_OFF)
		return -EINVAL;
	op_off = sspp->cap->sblk->igc_blk.base - DPU_VIG_TOP_OFF + DPU_VIG_IGC_OPMODE;

	if (!map)
		op = dpu_reg_dma_gamut_bank(dpu_reg_dma_sspp_gamut_op(sspp), &table);
	else
		op = DPU_VIG_GAMUT_EN | DPU_VIG_GAMUT_MAP_EN;	/* mode 17 = 0: table A */

	dpu_reg_dma_buf_reset(rdma);
	ret = dpu_reg_dma_decode_sel(rdma, blk);
	if (!ret && map)
		ret = dpu_reg_dma_single_modify(rdma, op_off, ~DPU_VIG_GAMUT_OP_BITS, 0); /* off first */
	if (!ret)
		ret = dpu_reg_dma_lutbus_write(rdma, DPU_REG_DMA_LUTBUS_GAMUT, table,
					       DPU_REG_DMA_LUTBUS_GAMUT_TRANS, lattice,
					       DPU_VIG_GAMUT_DWORDS);
	for (ch = 0; map && ch < DPU_VIG_GAMUT_MAP_CHANNELS && !ret; ch++)
		ret = dpu_reg_dma_block_write(rdma, op_off + DPU_VIG_GAMUT_SCALEA +
					      ch * DPU_VIG_GAMUT_SCALE_STRIDE,
					      map + ch * DPU_VIG_GAMUT_MAP_WORDS,
					      DPU_VIG_GAMUT_MAP_WORDS);
	if (!ret)
		ret = dpu_reg_dma_single_modify(rdma, op_off, ~DPU_VIG_GAMUT_OP_BITS, op);
	if (!ret)
		ret = dpu_reg_dma_kick_off_wait(dpu_kms, rdma, ctl, true, REG_DMA_LUT_WAIT_US);

	return ret;
}

/**
 * dpu_reg_dma_disable_sspp_gamut - switch a video pipe's gamut off, inverse gamma's bits kept
 */
int dpu_reg_dma_disable_sspp_gamut(struct dpu_kms *dpu_kms, struct dpu_reg_dma *rdma,
				   struct dpu_hw_ctl *ctl, struct dpu_hw_sspp *sspp)
{
	u32 blk, op_off;
	int ret;

	if (!sspp->cap->sblk->igc_blk.len)
		return -ENODEV;
	blk = dpu_reg_dma_blk_for_sspp(sspp->idx);
	if (!blk)
		return -ENODEV;
	if (sspp->cap->sblk->igc_blk.base < DPU_VIG_TOP_OFF)
		return -EINVAL;
	op_off = sspp->cap->sblk->igc_blk.base - DPU_VIG_TOP_OFF + DPU_VIG_IGC_OPMODE;

	dpu_reg_dma_buf_reset(rdma);
	ret = dpu_reg_dma_decode_sel(rdma, blk);
	if (!ret)
		ret = dpu_reg_dma_single_modify(rdma, op_off, ~DPU_VIG_GAMUT_OP_BITS, 0); /* off */
	if (!ret)
		ret = dpu_reg_dma_kick_off_wait(dpu_kms, rdma, ctl, true, REG_DMA_LUT_WAIT_US);

	return ret;
}

/**
 * dpu_reg_dma_gamut_test_action - what the colour flush does to one pipe's gamut
 * @want: colorop_gamut_test now (out of range = off)
 * @applied: the value this pipe was last given
 * @on: the pipe's gamut enable as the register reads now
 *
 * Return: 1 program, -1 switch off, 0 nothing.  A table lost with the block's
 * power (the enable read clear) is programmed again.
 */
int dpu_reg_dma_gamut_test_action(int want, int applied, bool on)
{
	if (want < 0 || want > DPU_VIG_GAMUT_TEST_MAX)
		want = 0;
	if (!want)
		return (applied || on) ? -1 : 0;
	return (want != applied || !on) ? 1 : 0;
}

/**
 * dpu_reg_dma_gamut_colorop_action - what the colour flush does to one pipe's
 *                                    gamut for the 3D-LUT colour operation
 * @want: the data blob id the pipe's plane asks for, 0 for none
 * @applied: the blob id this pipe was last given, 0 for none
 * @failed: the blob id that last failed on this pipe, 0 for none
 * @on: the pipe's gamut enable as the register reads now
 * @test: the gamut TEST switch owns the block (set, or still applied)
 *
 * Return: 1 program, -1 switch off, 0 nothing.  A table lost with the block's
 * power (the enable read clear) is programmed again; a table that failed is
 * not retried until the plane asks for a different one.
 */
int dpu_reg_dma_gamut_colorop_action(u32 want, u32 applied, u32 failed, bool on, bool test)
{
	if (test)
		return 0;
	if (!want)
		return applied ? -1 : 0;
	if (want == applied && on)
		return 0;
	if (want == failed)
		return 0;
	return 1;
}

/**
 * dpu_reg_dma_sspp_gamut_is_on - the gamut enable as a video pipe's op word reads now
 */
bool dpu_reg_dma_sspp_gamut_is_on(struct dpu_hw_sspp *sspp)
{
	return DPU_REG_READ(&sspp->hw, sspp->cap->sblk->igc_blk.base + DPU_VIG_IGC_OPMODE) &
	       DPU_VIG_GAMUT_EN;
}

/**
 * dpu_reg_dma_blk_for_sspp - the decode-select bit naming a source pipe
 *
 * Return: the bit, or 0 for a pipe the engine cannot address.
 */
u32 dpu_reg_dma_blk_for_sspp(enum dpu_sspp id)
{
	switch (id) {
	case SSPP_VIG0: return DPU_REG_DMA_BLK_VIG0;
	case SSPP_VIG1: return DPU_REG_DMA_BLK_VIG1;
	case SSPP_VIG2: return DPU_REG_DMA_BLK_VIG2;
	case SSPP_VIG3: return DPU_REG_DMA_BLK_VIG3;
	case SSPP_DMA0: return DPU_REG_DMA_BLK_DMA0;
	case SSPP_DMA1: return DPU_REG_DMA_BLK_DMA1;
	case SSPP_DMA2: return DPU_REG_DMA_BLK_DMA2;
	case SSPP_DMA3: return DPU_REG_DMA_BLK_DMA3;
	case SSPP_DMA4: return DPU_REG_DMA_BLK_DMA4;
	case SSPP_DMA5: return DPU_REG_DMA_BLK_DMA5;
	default:        return 0;
	}
}

/* read one named entry of the table back through the block's own index port */
static u32 dpu_vig_igc_read_entry(struct dpu_hw_blk_reg_map *hw, u32 cpu_off,
				  u32 channel, u32 index)
{
	DPU_REG_WRITE(hw, cpu_off + DPU_VIG_IGC_INDEX, index);
	return DPU_REG_READ(hw, cpu_off + DPU_VIG_IGC_DATA + channel * sizeof(u32));
}

/**
 * dpu_vig_igc_port_is_fresh - has this block lost power since it was last enabled?
 * @hw:      the source pipe's register map
 * @cpu_off: the inverse-gamma block's offset inside it
 * @probe:   filled with the three channels' DPU_VIG_IGC_PROBE_INDEX entries, for the report
 *
 * The block's index/data port shows its own power-up contents until inverse gamma is first
 * enabled, and reads zero from then until the display next powers down -- in steady state, after a
 * switch-off and before a live reload alike (measured).  So a readable port is the one witness, in
 * the hardware itself, that the power was lost; the controller's runtime-PM state is not, because
 * idle power collapse suspends the controller while the display's power domain stays on.  Three
 * entries a channel are sampled, so power-up contents that happen to be zero at one cannot pass
 * for a dark port.  The engine's program resets the index before it writes, so this costs nothing.
 *
 * Return: true when any sampled entry reads non-zero.
 */
static bool dpu_vig_igc_port_is_fresh(struct dpu_hw_blk_reg_map *hw, u32 cpu_off, u32 *probe)
{
	static const u32 at[] = { 0, DPU_VIG_IGC_PROBE_INDEX, DPU_VIG_IGC_DWORDS - 1 };
	u32 any = 0;
	int ch, i;

	for (ch = 0; ch < DPU_VIG_IGC_CHANNELS; ch++) {
		for (i = 0; i < ARRAY_SIZE(at); i++) {
			u32 v = dpu_vig_igc_read_entry(hw, cpu_off, ch, at[i]);

			if (at[i] == DPU_VIG_IGC_PROBE_INDEX)
				probe[ch] = v;
			any |= v;
		}
	}

	return any != 0;
}

/**
 * dpu_reg_dma_sspp_igc_is_on - does this pipe's inverse-gamma block have a curve?
 *
 * 🔴 THE CACHED ANSWER IS NOT SAFE TO USE.  The display controller power-collapses
 * whenever the panel blanks, and the block loses both its table and its enable bit;
 * a driver that remembered "programmed" across that would skip re-sending the curve
 * and leave the compositor believing a chain had been applied that no longer exists.
 * The first clip would look right and the second would not, which reads as a
 * hardware flake.  So this reads the register, in the powered commit, every frame a
 * pipeline is in force -- one MMIO read, and it is this driver's own standing rule
 * that an acknowledgement is never a substitute for the effect.
 *
 * Return: true when inverse gamma is switched on.
 */
bool dpu_reg_dma_sspp_igc_is_on(struct dpu_hw_sspp *sspp)
{
	if (!sspp->cap->sblk->igc_blk.len)
		return false;

	return !!(DPU_REG_READ(&sspp->hw,
			       sspp->cap->sblk->igc_blk.base + DPU_VIG_IGC_OPMODE) &
		  DPU_VIG_IGC_LUT_ENABLE);
}

/**
 * dpu_reg_dma_program_sspp_igc - load and enable a video pipe's inverse-gamma table
 * @lut:    DPU_VIG_IGC_CHANNELS * DPU_VIG_IGC_DWORDS dwords, channel-major
 * @last:   DPU_VIG_IGC_LAST_N endpoint values, one a channel, from the same
 *          conversion as @lut (the block's grid is not DRM's)
 * @report: read the result back and log it -- true only when something changed,
 *          because this runs inside the commit and the table does not
 *
 * This is the device's own v6 sequence, transcribed from its driver and proven
 * on this hardware to load the table: name the pipe, then per channel reset the index and
 * push the run through the data port, then the dither, then the masked modify
 * that turns inverse gamma on, then the curve's endpoint.
 *
 * ⚠️ THE ENDPOINT WRITE HAS NEVER LANDED on this part -- measured alone, after an
 * enable, and inside the full sequence -- and those registers have been seen
 * holding THIRTEEN-BIT values, which a twelve-bit curve endpoint cannot be.  It
 * is submitted anyway because it is what the vendor submits and the transfer
 * completes cleanly with it; @report is what says whether that ever changes.
 *
 * Return: 0 when the engine acknowledged the transfer, negative otherwise.
 */
int dpu_reg_dma_program_sspp_igc(struct dpu_kms *dpu_kms, struct dpu_reg_dma *rdma,
				 struct dpu_hw_ctl *ctl, struct dpu_hw_sspp *sspp,
				 const u32 *lut, const u32 *last, bool report)
{
	struct dpu_hw_blk_reg_map *hw = &sspp->hw;
	u32 before[DPU_VIG_IGC_CHANNELS] = {};
	u32 blk, cpu_off, dma_off, orig_opmode, enable_val, bank, pipe_bit;
	bool fresh, known, shown1;
	const char *why;
	int ch, program, ret;

	if (!sspp->cap->sblk->igc_blk.len)
		return -ENODEV;

	blk = dpu_reg_dma_blk_for_sspp(sspp->idx);
	if (!blk)
		return -ENODEV;

	cpu_off = sspp->cap->sblk->igc_blk.base;
	if (cpu_off < DPU_VIG_TOP_OFF) {
		DPU_ERROR("%s: inverse gamma at %#x is below the pipe's anchor shift\n",
			  sspp->cap->name, cpu_off);
		return -EINVAL;
	}
	dma_off = cpu_off - DPU_VIG_TOP_OFF;

	orig_opmode = DPU_REG_READ(hw, cpu_off + DPU_VIG_IGC_OPMODE);

	/*
	 * Refuse rather than guess.  A bit set outside the mask and outside the
	 * two being written is a bit this modify would clear and nothing would
	 * put back -- gamut's, if anything ever programs it.
	 *
	 * 🔴 "AND OUTSIDE THE TWO BEING WRITTEN" IS HALF THE TEST.  The enable and
	 * bank-select bits are this function's own, and a block that is already
	 * on has the enable bit set.  Counting it as foreign refused EVERY reload
	 * of a live table, and the caller's failure path then switched inverse
	 * gamma off and latched the pipe as failed for that table -- measured: a
	 * change of curve during playback left the video with no inverse gamma at
	 * all, even across a power collapse.
	 */
	if (orig_opmode & ~(DPU_VIG_IGC_OP_MASK | DPU_VIG_IGC_LUT_ENABLE |
			    DPU_VIG_IGC_LUT_SEL)) {
		DPU_ERROR("%s: inverse-gamma opmode %#x has bits outside the mask %#x\n",
			  sspp->cap->name, orig_opmode, DPU_VIG_IGC_OP_MASK);
		return -EINVAL;
	}

	/*
	 * 🔴 WHICH BANK TO SHOW -- the choice this function exists to get right.
	 *
	 * The table has two banks, and the hardware, not this driver, decides
	 * which one the engine writes.  Measured in the pixels (the display
	 * controller's layer-mixer CRC on static stimuli, three tables):
	 *
	 *   the pipeline shows the bank the select bit (9) names;
	 *   an ENABLED write of the select bit sets the hardware's own pointer;
	 *   a switch-off leaves that pointer where it was, although it clears
	 *     the register's bit;
	 *   once enabled since power-up, the engine writes the bank opposite
	 *     the pointer; and after a power loss its first write is bank 0.
	 *
	 * The device's own driver chose from the register -- `(~lut_sel) &&
	 * BIT(0)`, a logical and that is always 1 for an enabled block, and the
	 * cleared bit after a switch-off -- so a second live reload, or a load
	 * after a switch-off from bank 0, left the PREVIOUS curve on screen
	 * until the panel next powered down, with nothing in the frame to say
	 * so.  The bank on display is therefore recorded here, per pipe, and the
	 * bank enabled is the one opposite it: the one the engine just wrote.
	 *
	 * The record is reset only by evidence of a real power loss: a readable
	 * port.  An unknown record with a dark port -- reachable only after an
	 * engine error -- can only be guessed; the enable that follows sets the
	 * pointer, so the NEXT load is right even when this one is not.
	 */
	pipe_bit = BIT(sspp->idx);
	fresh = dpu_vig_igc_port_is_fresh(hw, cpu_off, before);
	if (fresh) {
		known = true;
		shown1 = true;		/* as if the pointer were 1: the next write is bank 0 */
		why = "fresh, the port was readable";
	} else {
		known = dpu_kms->colorop_igc_bank_known & pipe_bit;
		shown1 = dpu_kms->colorop_igc_bank1 & pipe_bit;
		why = !known ? "UNKNOWN, this load may show the previous curve" :
		      shown1 ? "tracked, showing 1" : "tracked, showing 0";
	}
	bank = known ? !shown1 : 1;
	enable_val = DPU_VIG_IGC_LUT_ENABLE | (bank << 9);

	/*
	 * The endpoint (@last) is the 257th knot, at input 1024: it comes from the
	 * conversion that resampled the curve onto this block's grid, so input
	 * 1023 lands on the curve's own last value (dpu_colorop_igc.h).  It used
	 * to repeat the top slot -- right only while the slots were copied 1:1.
	 */
	dpu_reg_dma_buf_reset(rdma);
	ret = dpu_reg_dma_decode_sel(rdma, blk);

	program = READ_ONCE(dpu_reg_dma_igc_test_program);
	for (ch = 0; ch < DPU_VIG_IGC_CHANNELS && !ret; ch++) {
		ret = dpu_reg_dma_single_write(rdma, dma_off + DPU_VIG_IGC_INDEX, 0);
		if (ret)
			break;
		ret = dpu_reg_dma_indexed_write(rdma,
						dma_off + DPU_VIG_IGC_DATA +
						ch * sizeof(u32),
						lut + ch * DPU_VIG_IGC_DWORDS,
						DPU_VIG_IGC_DWORDS);
		/* commit the run's last word before the next index write discards it */
		if (!ret && program != 1)
			ret = dpu_reg_dma_single_write(rdma, dma_off + DPU_VIG_IGC_LAST +
						       ch * sizeof(u32), last[ch]);
	}
	if (!ret)
		ret = dpu_reg_dma_single_write(rdma, dma_off + DPU_VIG_IGC_DITHER,
					       DPU_VIG_IGC_DITHER_OFF);
	if (!ret)
		ret = dpu_reg_dma_single_modify(rdma, dma_off + DPU_VIG_IGC_OPMODE,
						DPU_VIG_IGC_OP_MASK, enable_val);
	if (!ret)
		ret = dpu_reg_dma_block_write(rdma, dma_off + DPU_VIG_IGC_LAST,
					      last, DPU_VIG_IGC_LAST_N);
	if (ret) {
		DPU_ERROR("%s: failed to build the inverse-gamma program: %d\n",
			  sspp->cap->name, ret);
		return ret;
	}

	ret = dpu_reg_dma_kick_off(dpu_kms, rdma, ctl, true);

	/*
	 * Record what the pipeline now shows.  A kick-off that failed may have
	 * stopped before or after the enable, so after one the bank is not
	 * known -- which is what the guess above is for.
	 */
	if (ret) {
		dpu_kms->colorop_igc_bank_known &= ~pipe_bit;
	} else {
		dpu_kms->colorop_igc_bank_known |= pipe_bit;
		if (bank)
			dpu_kms->colorop_igc_bank1 |= pipe_bit;
		else
			dpu_kms->colorop_igc_bank1 &= ~pipe_bit;
	}

	if (report) {
		u32 opmode_after, index_after, read_last[DPU_VIG_IGC_LAST_N];
		u32 got[DPU_VIG_IGC_CHANNELS], want[DPU_VIG_IGC_CHANNELS];
		bool table_ok = true;
		/*
		 * 🔴 A DARK PORT CANNOT TESTIFY.  Once this block has been enabled
		 * since the display last powered up, its index/data port reads zero
		 * and refuses writes -- in steady state, after a switch-off, and
		 * before a live reload alike (measured) -- until the next power
		 * collapse.  A read-back through it then says "did not land" for a
		 * load the pixels show landed.  So a load that began with the port
		 * already dark is reported UNVERIFIABLE, and only the pixels judge
		 * it; the port shows the block's own power-up contents before the
		 * first enable, which is what makes it a witness at all.
		 */
		bool port_dark = !fresh;
		int i;

		/* everything that does not move the index, first */
		opmode_after = DPU_REG_READ(hw, cpu_off + DPU_VIG_IGC_OPMODE);
		index_after  = DPU_REG_READ(hw, cpu_off + DPU_VIG_IGC_INDEX);
		for (i = 0; i < DPU_VIG_IGC_LAST_N; i++)
			read_last[i] = DPU_REG_READ(hw, cpu_off + DPU_VIG_IGC_LAST +
						    i * sizeof(u32));

		/*
		 * Then the table itself, which is the instrument with a
		 * demonstrated two-sided response: it showed the block's own
		 * garbage before the first load ever read back and the exact expected words
		 * after.  The endpoint registers cannot do that and are reported
		 * beside it as secondary evidence only.
		 */
		for (i = 0; i < DPU_VIG_IGC_CHANNELS; i++) {
			want[i] = lut[i * DPU_VIG_IGC_DWORDS + DPU_VIG_IGC_PROBE_INDEX];
			got[i] = dpu_vig_igc_read_entry(hw, cpu_off, i,
							DPU_VIG_IGC_PROBE_INDEX);
			if (got[i] != want[i])
				table_ok = false;
		}
		DPU_REG_WRITE(hw, cpu_off + DPU_VIG_IGC_INDEX, index_after);

		DRM_INFO("colorop: %s inverse gamma ctl%d | opmode %#x -> wrote %#x under keep-mask %#x -> read %#x (%s) | index %#x | dword %d of each channel: expected %#x %#x %#x read %#x %#x %#x -- %s | endpoint: wrote %#x %#x %#x read %#x %#x %#x -- %s | kick_off %d | bank: %s -> showing %u | program %d\n",
			 sspp->cap->name, ctl->idx - CTL_0,
			 orig_opmode, enable_val, DPU_VIG_IGC_OP_MASK, opmode_after,
			 (opmode_after & DPU_VIG_IGC_LUT_ENABLE) ? "INVERSE GAMMA ON"
								 : "STILL OFF",
			 index_after,
			 DPU_VIG_IGC_PROBE_INDEX,
			 want[0], want[1], want[2], got[0], got[1], got[2],
			 port_dark ? "TABLE UNVERIFIABLE" :
			 table_ok  ? "TABLE LANDED" : "TABLE DID NOT LAND",
			 last[0], last[1], last[2],
			 read_last[0], read_last[1], read_last[2],
			 (read_last[0] == last[0] && read_last[1] == last[1] &&
			  read_last[2] == last[2]) ? "endpoint landed"
						  : "endpoint did not land",
			 ret, why, bank, program);

		/*
		 * Then the WHOLE table against what was sent, on its own line so
		 * the line above keeps the exact form the measuring tools parse.
		 * One dword per channel cannot tell a complete load from one whose
		 * tail -- the top of the curve, the highlights -- did not land, and
		 * the pixels have shown power-up-dependent behaviour there.  EVERY
		 * difference is named, up to DPU_VIG_IGC_REPORT_MAX: the first alone
		 * located one of the two words a fresh load misses and hid the other.
		 * A dark port is not compared at all -- 384 zeros say nothing.
		 */
		if (port_dark) {
			DRM_INFO("colorop: %s inverse gamma table | before the load, dword %d read %#x %#x %#x | the port was dark before the load: this block has been enabled since the display last powered up and the read-back cannot see the table -- UNVERIFIABLE, judge it by the pixels\n",
				 sspp->cap->name, DPU_VIG_IGC_PROBE_INDEX,
				 before[0], before[1], before[2]);
		} else {
			u32 as_written = 0, top_ok = 0, n_bad = 0;
			char diffs[DPU_VIG_IGC_REPORT_MAX * 48];
			int c, k, len = 0;

			diffs[0] = '\0';
			for (c = 0; c < DPU_VIG_IGC_CHANNELS; c++) {
				for (k = 0; k < DPU_VIG_IGC_DWORDS; k++) {
					u32 w = lut[c * DPU_VIG_IGC_DWORDS + k];
					u32 g = dpu_vig_igc_read_entry(hw, cpu_off, c, k);

					if (g == w) {
						as_written++;
						if (k >= DPU_VIG_IGC_DWORDS - 8)
							top_ok++;
						continue;
					}
					if (n_bad++ < DPU_VIG_IGC_REPORT_MAX)
						len += scnprintf(diffs + len, sizeof(diffs) - len,
								 "%sch%d dword %d wrote %#x read %#x",
								 len ? "; " : "", c, k, w, g);
				}
			}
			DPU_REG_WRITE(hw, cpu_off + DPU_VIG_IGC_INDEX, index_after);

			DRM_INFO("colorop: %s inverse gamma table | before the load, dword %d read %#x %#x %#x | after: %u of %u dwords as written, top 8 of each channel %u of %u | %u difference(s)%s%s%s\n",
				 sspp->cap->name, DPU_VIG_IGC_PROBE_INDEX,
				 before[0], before[1], before[2],
				 as_written, DPU_VIG_IGC_CHANNELS * DPU_VIG_IGC_DWORDS,
				 top_ok, DPU_VIG_IGC_CHANNELS * 8, n_bad,
				 n_bad ? ": " : "", diffs,
				 n_bad > DPU_VIG_IGC_REPORT_MAX ? "; ..." : "");
		}
	}

	return ret;
}

/**
 * dpu_reg_dma_disable_sspp_igc - turn a video pipe's inverse gamma back off
 *
 * 🔴 TWO ROUTES, AND THE SECOND ONE IS NOT BELT AND BRACES.  The engine's masked
 * modify has been measured completing cleanly and doing nothing -- once in four
 * attempts when first measured -- and a pipe left with inverse gamma ON and a stale
 * table is a visibly wrong picture with no obvious cause, which outlives the
 * commit that caused it.  So the engine is asked first, because that is the
 * vendor's own off path, and the result is READ BACK and corrected by the CPU
 * when it did not take.
 *
 * Return: 0 if inverse gamma is off when this returns, -EIO if it is not.
 */
int dpu_reg_dma_disable_sspp_igc(struct dpu_kms *dpu_kms, struct dpu_reg_dma *rdma,
				 struct dpu_hw_ctl *ctl, struct dpu_hw_sspp *sspp)
{
	struct dpu_hw_blk_reg_map *hw = &sspp->hw;
	u32 blk, cpu_off, dma_off, by_engine, by_cpu;
	int ret;

	if (!sspp->cap->sblk->igc_blk.len)
		return -ENODEV;

	blk = dpu_reg_dma_blk_for_sspp(sspp->idx);
	if (!blk)
		return -ENODEV;

	cpu_off = sspp->cap->sblk->igc_blk.base;
	if (cpu_off < DPU_VIG_TOP_OFF)
		return -EINVAL;
	dma_off = cpu_off - DPU_VIG_TOP_OFF;

	/*
	 * Already off costs nothing.  The retire sweep this serves runs over
	 * every pipe the driver has ever enabled, so without this it would
	 * submit a transfer for pipes that have been off for minutes.
	 */
	if (!(DPU_REG_READ(hw, cpu_off + DPU_VIG_IGC_OPMODE) & DPU_VIG_IGC_LUT_ENABLE))
		return 0;

	dpu_reg_dma_buf_reset(rdma);
	ret = dpu_reg_dma_decode_sel(rdma, blk);
	if (!ret)
		ret = dpu_reg_dma_single_modify(rdma, dma_off + DPU_VIG_IGC_OPMODE,
						DPU_VIG_IGC_OP_MASK, 0);
	if (!ret)
		ret = dpu_reg_dma_kick_off(dpu_kms, rdma, ctl, true);

	by_engine = DPU_REG_READ(hw, cpu_off + DPU_VIG_IGC_OPMODE);
	if (!(by_engine & DPU_VIG_IGC_LUT_ENABLE))
		return 0;

	DPU_REG_WRITE(hw, cpu_off + DPU_VIG_IGC_OPMODE,
		      by_engine & ~(DPU_VIG_IGC_LUT_ENABLE | DPU_VIG_IGC_LUT_SEL));
	by_cpu = DPU_REG_READ(hw, cpu_off + DPU_VIG_IGC_OPMODE);

	DRM_INFO("colorop: %s inverse gamma off | the engine's own path left %#x, a cpu write left %#x -- %s | kick_off %d\n",
		 sspp->cap->name, by_engine, by_cpu,
		 (by_cpu & DPU_VIG_IGC_LUT_ENABLE) ? "STILL ON -- THE PICTURE IS WRONG"
						   : "off",
		 ret);

	return (by_cpu & DPU_VIG_IGC_LUT_ENABLE) ? -EIO : 0;
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
#define SELFTEST_SOURCE_PIPE		BIT(2)
/*
 * Two more source-pipe arms that differ from each other in ONE thing: whether
 * the pipe is carrying a plane.  Source-pipe registers are shadowed and latch on
 * the control block's flush for the frame that uses the pipe, and the transfer
 * is started from that same control block -- so a pipe in no active path is a
 * plausible reason for a write to go nowhere, and it was never held constant.
 */
#define SELFTEST_LIVE_DMA_PIPE		BIT(3)
#define SELFTEST_IDLE_DMA_PIPE		BIT(4)
/*
 * A video pipe, aimed at registers the CPU can actually verify.  The earlier
 * video-pipe arm targeted the only registers in that block the CPU CANNOT write,
 * so its read-back could never have told a transfer that landed from one that did
 * not -- measured from userspace: the index and dither registers beside them are
 * freely writable, and the last-entry registers are not.
 */
#define SELFTEST_VIG_PLAIN_REGS		BIT(5)
/*
 * The device's own v6 inverse-gamma sequence, in two shapes.
 *
 * The first submits the enable and the last-point write as separate transfers,
 * so that the second of them is byte for byte a program already known to
 * complete and write nothing, with a single thing changed ahead of it.  The
 * second submits the whole vendor sequence, lookup table included, in one
 * command buffer -- which is what a colour-operation implementation has to send
 * and is therefore worth knowing about, but changes too much at once to settle
 * anything on its own.
 */
#define SELFTEST_VIG_V6_SPLIT		BIT(6)
#define SELFTEST_VIG_V6_FULL		BIT(7)

/*
 * The source pipes' inverse-gamma block, which is the block the colour-operation
 * work needs and the reason this engine exists: on this hardware revision it has
 * no CPU write path at all.
 *
 * 🔴 IT HAS TWO DIFFERENT OFFSETS AND THEY ARE BOTH CORRECT.
 * The CPU reaches it at the block offset this device's description gives
 * (qcom,sde-vig-igc = <0x1d00 ...>). The engine reaches it at that offset MINUS
 * the pipe's top offset (qcom,sde-vig-top-off = <0xa00>), because the hardware
 * decoder anchors source-pipe sub-blocks differently from the register mapping.
 * Using the CPU's number in a command buffer writes 0xa00 too high, and the
 * read-back then looks exactly like a transfer that never happened.
 */
#define SELFTEST_VIG_IGC_CPU_OFF	0x1d00
#define SELFTEST_VIG_IGC_DMA_OFF	(0x1d00 - 0xa00)
/*
 * The last-entry registers at the end of that block are ordinary registers
 * written as a run, so unlike the lookup table itself they can be read straight
 * back through the register mapping -- which is what makes them a target that
 * can prove the transfer rather than merely not disprove it.
 */
#define SELFTEST_VIG_IGC_LAST		0x1c4
#define SELFTEST_VIG_IGC_LAST_N		3

/*
 * Two ordinary configuration registers in the same block, which the vendor writes
 * as plain single-register writes and which the CPU can read AND write -- measured
 * from userspace on this device, against the last-entry register above as a
 * control that correctly reported itself unwritable.
 *
 * The index register selects which entry of the lookup table the block's data
 * port will next touch; the dither register holds a four-bit strength plus an
 * enable bit.  Writing either changes no pixel while inverse gamma is disabled,
 * which it is on every video pipe here.
 *
 * 🔴 THE SENTINELS ARE NARROW ON PURPOSE.  The dither field is four bits wide plus
 * BIT(4), so a wider value would be truncated by the hardware and read back
 * changed -- scoring as a failure the transfer did not cause.  The index counts
 * dwords into a 128-entry table, so seven bits.  Both patterns fit their field.
 */
#define SELFTEST_VIG_IGC_INDEX		0x1b0
#define SELFTEST_VIG_IGC_DITHER		0x1c0
#define SELFTEST_VIG_INDEX_CPU		0x2a
#define SELFTEST_VIG_INDEX_ENGINE	0x55
#define SELFTEST_VIG_DITHER_CPU		0x0b
#define SELFTEST_VIG_DITHER_ENGINE	0x16

/* twelve-bit values, distinctive, that nothing else has any reason to write */
static const u32 selftest_vig_sentinel[SELFTEST_VIG_IGC_LAST_N] = {
	0xa51, 0xb62, 0xc73,
};

/*
 * The DMA source pipes' inverse-gamma block.
 *
 * This is a DIFFERENT SHAPE from the video pipes' block, and the difference is
 * exactly what makes it the right place to ask whether the engine reaches a
 * source pipe at all.  The vendor writes this lookup table as a run of ORDINARY
 * CONSECUTIVE REGISTERS, where the video pipes' table goes through a single
 * auto-advancing index port.  Ordinary registers can be read straight back
 * through the register mapping, so the read-back can prove the transfer rather
 * than merely fail to disprove it.
 *
 * Offsets, from this device's own description of itself:
 *   qcom,sde-dma-igc     = <0xa00 0x50000>    the block, as the CPU reaches it
 *   qcom,sde-dma-top-off = <0x800>            this pipe class's own anchor shift
 * and the engine's base is the block's base minus that shift.
 *
 * 🔴 THE SHIFT IS 0x800 HERE AND 0xa00 ON THE VIDEO PIPES.  It is a property of
 * the pipe class, not a constant, so it cannot be carried across from the other
 * arm.
 *
 * Nothing here enables inverse gamma: that is a separate masked modify of a
 * different register, which this deliberately does not do.  Lookup table
 * contents with the feature disabled change no pixel, so this arm has nothing to
 * see and nothing to put back beyond the values it found.
 */
#define SELFTEST_DMA_IGC_CPU_OFF	0xa00
#define SELFTEST_DMA_IGC_DMA_OFF	(0xa00 - 0x800)
#define SELFTEST_DMA_IGC_N		3

/*
 * Both patterns keep each sixteen-bit half below 0x1000, because these registers
 * hold two packed twelve-bit table entries and anything wider is masked away --
 * a sentinel the hardware truncates would read back changed and be scored as a
 * failure it did not cause.
 */
static const u32 selftest_dma_cpu_pattern[SELFTEST_DMA_IGC_N] = {
	0x01230456, 0x07890abc, 0x0def0111,
};
static const u32 selftest_dma_sentinel[SELFTEST_DMA_IGC_N] = {
	0x0a510b62, 0x0c730d84, 0x0e950fa6,
};

static int lutdma_selftest;
static int lutdma_selftest_pending;
static struct dpu_reg_dma lutdma_selftest_engine;
static struct dpu_kms *lutdma_selftest_kms;

void dpu_reg_dma_selftest_bind(struct dpu_kms *dpu_kms)
{
	lutdma_selftest_kms = dpu_kms;
}

void dpu_reg_dma_selftest_unbind(struct dpu_kms *dpu_kms)
{
	if (lutdma_selftest_kms != dpu_kms)
		return;

	dpu_reg_dma_fini(dpu_kms, &lutdma_selftest_engine);
	lutdma_selftest_kms = NULL;
	lutdma_selftest_pending = 0;
}

/*
 * Allocate here rather than in the commit path. A sysfs write is ordinary
 * process context; the atomic commit is not somewhere to be asking for memory.
 */
static int lutdma_selftest_set(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_int(val, kp);

	if (ret)
		return ret;

	if (lutdma_selftest & SELFTEST_RUN) {
		if (!lutdma_selftest_kms) {
			DPU_ERROR("no display device bound\n");
			return -ENODEV;
		}

		ret = dpu_reg_dma_init(lutdma_selftest_kms,
				       &lutdma_selftest_engine, PAGE_SIZE);
		if (ret)
			return ret;
	}

	lutdma_selftest_pending = 1;
	return 0;
}

static const struct kernel_param_ops lutdma_selftest_ops = {
	.set = lutdma_selftest_set,
	.get = param_get_int,
};
module_param_cb(lutdma_selftest, &lutdma_selftest_ops, &lutdma_selftest, 0644);
MODULE_PARM_DESC(lutdma_selftest,
	"register-DMA engine self-test: 1=post-blend colour block, 5=video source pipe last-entry registers, 9=DMA source pipe carrying the desktop, 17=DMA source pipe carrying nothing, 33=video source pipe index and dither registers, 65=enable inverse gamma then write the curve's last point, as two transfers, 129=the device's whole v6 inverse-gamma sequence in one transfer, +2 queues without starting the engine (control), 0=restore");

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

/*
 * A source pipe, located through the catalogue rather than by a constant, so the
 * offset used is the one this driver believes in.
 */
static const struct dpu_sspp_cfg *selftest_find_sspp(struct dpu_kms *dpu_kms,
						     enum dpu_sspp id)
{
	int i;

	for (i = 0; i < dpu_kms->catalog->sspp_count; i++)
		if (dpu_kms->catalog->sspp[i].id == id)
			return &dpu_kms->catalog->sspp[i];

	return NULL;
}

static u32 selftest_decode_sel_for_sspp(enum dpu_sspp id)
{
	switch (id) {
	case SSPP_VIG0: return DPU_REG_DMA_BLK_VIG0;
	case SSPP_VIG1: return DPU_REG_DMA_BLK_VIG1;
	case SSPP_VIG2: return DPU_REG_DMA_BLK_VIG2;
	case SSPP_VIG3: return DPU_REG_DMA_BLK_VIG3;
	case SSPP_DMA0: return DPU_REG_DMA_BLK_DMA0;
	case SSPP_DMA1: return DPU_REG_DMA_BLK_DMA1;
	case SSPP_DMA2: return DPU_REG_DMA_BLK_DMA2;
	case SSPP_DMA3: return DPU_REG_DMA_BLK_DMA3;
	case SSPP_DMA4: return DPU_REG_DMA_BLK_DMA4;
	case SSPP_DMA5: return DPU_REG_DMA_BLK_DMA5;
	default:        return 0;
	}
}

/*
 * Ask the engine to write three ordinary registers inside a video pipe's
 * inverse-gamma block, and read them back through the register mapping. This is
 * the block the colour-operation work has to reach, on the pipes that carry
 * video, through a different operation than the post-blend arm used -- so it
 * exercises the run-writing path and the source-pipe offset as well as the
 * engine.
 */
static void selftest_source_pipe(struct dpu_kms *dpu_kms, struct dpu_hw_ctl *ctl,
				 bool trigger)
{
	struct dpu_reg_dma *rdma = &lutdma_selftest_engine;
	const struct dpu_sspp_cfg *vig;
	struct dpu_hw_blk_reg_map hw;
	u32 before[SELFTEST_VIG_IGC_LAST_N], after[SELFTEST_VIG_IGC_LAST_N];
	bool pass = true;
	int i, ret;

	vig = selftest_find_sspp(dpu_kms, SSPP_VIG0);
	if (!vig) {
		DPU_ERROR("no video source pipe in the catalogue\n");
		return;
	}

	memset(&hw, 0, sizeof(hw));
	hw.blk_addr = dpu_kms->mmio + vig->base;
	hw.log_mask = DPU_DBG_MASK_SSPP;

	/* a known starting value, so the read at the end has something to differ from */
	for (i = 0; i < SELFTEST_VIG_IGC_LAST_N; i++)
		DPU_REG_WRITE(&hw, SELFTEST_VIG_IGC_CPU_OFF +
			      SELFTEST_VIG_IGC_LAST + i * sizeof(u32), 0);
	for (i = 0; i < SELFTEST_VIG_IGC_LAST_N; i++)
		before[i] = DPU_REG_READ(&hw, SELFTEST_VIG_IGC_CPU_OFF +
					 SELFTEST_VIG_IGC_LAST + i * sizeof(u32));

	dpu_reg_dma_buf_reset(rdma);
	ret = dpu_reg_dma_decode_sel(rdma, DPU_REG_DMA_BLK_VIG0);
	if (!ret)
		ret = dpu_reg_dma_block_write(rdma,
					      SELFTEST_VIG_IGC_DMA_OFF + SELFTEST_VIG_IGC_LAST,
					      selftest_vig_sentinel,
					      SELFTEST_VIG_IGC_LAST_N);
	if (ret) {
		DPU_ERROR("failed to build the command buffer: %d\n", ret);
		return;
	}

	ret = dpu_reg_dma_kick_off(dpu_kms, rdma, ctl, trigger);

	for (i = 0; i < SELFTEST_VIG_IGC_LAST_N; i++)
		after[i] = DPU_REG_READ(&hw, SELFTEST_VIG_IGC_CPU_OFF +
					SELFTEST_VIG_IGC_LAST + i * sizeof(u32));

	for (i = 0; i < SELFTEST_VIG_IGC_LAST_N; i++)
		if (after[i] != (trigger ? selftest_vig_sentinel[i] : 0))
			pass = false;

	DRM_INFO("selftest: source pipe (base %#x) ctl%d %s | before %#x %#x %#x | expected %#x %#x %#x | read %#x %#x %#x | %s | kick_off %d\n",
		 vig->base, ctl->idx - CTL_0,
		 trigger ? "engine started" : "ENGINE NOT STARTED (control)",
		 before[0], before[1], before[2],
		 trigger ? selftest_vig_sentinel[0] : 0,
		 trigger ? selftest_vig_sentinel[1] : 0,
		 trigger ? selftest_vig_sentinel[2] : 0,
		 after[0], after[1], after[2],
		 pass ? "PASS" : "FAIL", ret);
}

/*
 * Ask the engine to write a run of ordinary registers inside a DMA source pipe's
 * inverse-gamma block, and read them back through the register mapping.
 *
 * WHY THIS ARM EXISTS
 * A previous attempt wrote a video pipe and its read-back reported nothing
 * either way, and the two arms that produced a pass and a failure differed in a
 * variable nobody had held constant: the passing one targeted a post-blend block
 * that is in the display path every frame, the failing one a video pipe that was
 * carrying no plane at all.  This arm is run twice against the SAME block type
 * at the SAME offsets with the SAME operation -- once on the pipe carrying the
 * desktop, once on a pipe carrying nothing -- so that whether the pipe is live is
 * the only thing that differs.
 *
 * Nothing sets a flush bit by hand.  The live pipe's flush already happens every
 * frame because the desktop is on it, and the idle pipe's correctly does not;
 * forcing one would blur the very thing being separated.
 */
static void selftest_dma_pipe(struct dpu_kms *dpu_kms, struct dpu_hw_ctl *ctl,
			      bool trigger, enum dpu_sspp id, const char *liveness)
{
	struct dpu_reg_dma *rdma = &lutdma_selftest_engine;
	const struct dpu_sspp_cfg *sspp;
	struct dpu_hw_blk_reg_map hw;
	u32 orig[SELFTEST_DMA_IGC_N];
	u32 before[SELFTEST_DMA_IGC_N], after[SELFTEST_DMA_IGC_N];
	bool writable = true, pass = true;
	u32 blk;
	int i, ret;

	sspp = selftest_find_sspp(dpu_kms, id);
	if (!sspp) {
		DPU_ERROR("source pipe %d is not in the catalogue\n", id);
		return;
	}

	blk = selftest_decode_sel_for_sspp(id);
	if (!blk) {
		DPU_ERROR("no decode-select bit for source pipe %d\n", id);
		return;
	}

	memset(&hw, 0, sizeof(hw));
	hw.blk_addr = dpu_kms->mmio + sspp->base;
	hw.log_mask = DPU_DBG_MASK_SSPP;

	/*
	 * 🔴 THE CONTROL THE PREVIOUS ATTEMPT WAS MISSING.  Write a distinctive
	 * NON-ZERO value from the CPU and confirm it reads back, so the target is
	 * known to be a real, reachable, writable register before the engine is
	 * asked for anything.  Without it a wrong offset, a block that answers
	 * nothing and a register that cannot be written all produce the same zero,
	 * and all of them look exactly like the engine having failed.
	 */
	for (i = 0; i < SELFTEST_DMA_IGC_N; i++)
		orig[i] = DPU_REG_READ(&hw, SELFTEST_DMA_IGC_CPU_OFF + i * sizeof(u32));
	for (i = 0; i < SELFTEST_DMA_IGC_N; i++)
		DPU_REG_WRITE(&hw, SELFTEST_DMA_IGC_CPU_OFF + i * sizeof(u32),
			      selftest_dma_cpu_pattern[i]);
	for (i = 0; i < SELFTEST_DMA_IGC_N; i++) {
		before[i] = DPU_REG_READ(&hw, SELFTEST_DMA_IGC_CPU_OFF + i * sizeof(u32));
		if (before[i] != selftest_dma_cpu_pattern[i])
			writable = false;
	}

	dpu_reg_dma_buf_reset(rdma);
	ret = dpu_reg_dma_decode_sel(rdma, blk);
	if (!ret)
		ret = dpu_reg_dma_block_write(rdma, SELFTEST_DMA_IGC_DMA_OFF,
					      selftest_dma_sentinel,
					      SELFTEST_DMA_IGC_N);
	if (ret) {
		DPU_ERROR("failed to build the command buffer: %d\n", ret);
		return;
	}

	ret = dpu_reg_dma_kick_off(dpu_kms, rdma, ctl, trigger);

	for (i = 0; i < SELFTEST_DMA_IGC_N; i++)
		after[i] = DPU_REG_READ(&hw, SELFTEST_DMA_IGC_CPU_OFF + i * sizeof(u32));

	/*
	 * Started, the engine's values must be there.  Not started, the CPU's
	 * pattern must still be there -- which is a stronger control than reading
	 * zero, because a register that simply refuses every write would also read
	 * zero and would pass a weaker one.
	 */
	for (i = 0; i < SELFTEST_DMA_IGC_N; i++)
		if (after[i] != (trigger ? selftest_dma_sentinel[i]
					 : selftest_dma_cpu_pattern[i]))
			pass = false;

	DRM_INFO("selftest: %s pipe %s (base %#x) ctl%d %s | cpu-writable %s | before %#x %#x %#x | expected %#x %#x %#x | read %#x %#x %#x | %s | kick_off %d\n",
		 liveness, sspp->name, sspp->base, ctl->idx - CTL_0,
		 trigger ? "engine started" : "ENGINE NOT STARTED (control)",
		 writable ? "YES" : "NO -- the read-back below decides nothing",
		 before[0], before[1], before[2],
		 trigger ? selftest_dma_sentinel[0] : selftest_dma_cpu_pattern[0],
		 trigger ? selftest_dma_sentinel[1] : selftest_dma_cpu_pattern[1],
		 trigger ? selftest_dma_sentinel[2] : selftest_dma_cpu_pattern[2],
		 after[0], after[1], after[2],
		 (pass && writable) ? "PASS" : "FAIL", ret);

	/* put the block back exactly as it was found */
	for (i = 0; i < SELFTEST_DMA_IGC_N; i++)
		DPU_REG_WRITE(&hw, SELFTEST_DMA_IGC_CPU_OFF + i * sizeof(u32), orig[i]);
}

/*
 * Ask the engine to write two ordinary configuration registers inside a VIDEO
 * pipe's inverse-gamma block, and read them back through the register mapping.
 *
 * WHY THIS ARM EXISTS
 * The video pipes are the ones the colour-operation work needs, and the earlier
 * attempt to reach one could not report: it targeted the block's last-entry
 * registers, which are the only ones in it the CPU cannot write either, so a
 * transfer that landed and one that did not both read zero.  The index and dither
 * registers beside them are freely CPU-writable, which makes the read-back mean
 * something and lets the arm establish its own target before using it.
 *
 * Two independent registers rather than one, because each field is narrow and a
 * narrow sentinel is weak on its own.
 */
static void selftest_vig_plain_regs(struct dpu_kms *dpu_kms, struct dpu_hw_ctl *ctl,
				    bool trigger)
{
	struct dpu_reg_dma *rdma = &lutdma_selftest_engine;
	const struct dpu_sspp_cfg *vig;
	struct dpu_hw_blk_reg_map hw;
	u32 orig_index, orig_dither;
	u32 before_index, before_dither, after_index, after_dither;
	u32 last[SELFTEST_VIG_IGC_LAST_N];
	u32 want_index, want_dither;
	bool writable = true, pass = true;
	int i, ret;

	vig = selftest_find_sspp(dpu_kms, SSPP_VIG0);
	if (!vig) {
		DPU_ERROR("no video source pipe in the catalogue\n");
		return;
	}

	memset(&hw, 0, sizeof(hw));
	hw.blk_addr = dpu_kms->mmio + vig->base;
	hw.log_mask = DPU_DBG_MASK_SSPP;

	orig_index  = DPU_REG_READ(&hw, SELFTEST_VIG_IGC_CPU_OFF + SELFTEST_VIG_IGC_INDEX);
	orig_dither = DPU_REG_READ(&hw, SELFTEST_VIG_IGC_CPU_OFF + SELFTEST_VIG_IGC_DITHER);

	/* establish the target is real, reachable and writable before using it */
	DPU_REG_WRITE(&hw, SELFTEST_VIG_IGC_CPU_OFF + SELFTEST_VIG_IGC_INDEX,
		      SELFTEST_VIG_INDEX_CPU);
	DPU_REG_WRITE(&hw, SELFTEST_VIG_IGC_CPU_OFF + SELFTEST_VIG_IGC_DITHER,
		      SELFTEST_VIG_DITHER_CPU);
	before_index  = DPU_REG_READ(&hw, SELFTEST_VIG_IGC_CPU_OFF + SELFTEST_VIG_IGC_INDEX);
	before_dither = DPU_REG_READ(&hw, SELFTEST_VIG_IGC_CPU_OFF + SELFTEST_VIG_IGC_DITHER);
	if (before_index != SELFTEST_VIG_INDEX_CPU ||
	    before_dither != SELFTEST_VIG_DITHER_CPU)
		writable = false;

	dpu_reg_dma_buf_reset(rdma);
	ret = dpu_reg_dma_decode_sel(rdma, DPU_REG_DMA_BLK_VIG0);
	if (!ret)
		ret = dpu_reg_dma_single_write(rdma,
					       SELFTEST_VIG_IGC_DMA_OFF + SELFTEST_VIG_IGC_INDEX,
					       SELFTEST_VIG_INDEX_ENGINE);
	if (!ret)
		ret = dpu_reg_dma_single_write(rdma,
					       SELFTEST_VIG_IGC_DMA_OFF + SELFTEST_VIG_IGC_DITHER,
					       SELFTEST_VIG_DITHER_ENGINE);
	/*
	 * The last-entry registers the earlier arm used, written in the same
	 * program purely so the two can be compared in one run. They are reported
	 * separately and do NOT score this arm: the CPU cannot write them, so
	 * whatever they read proves nothing either way.
	 */
	if (!ret)
		ret = dpu_reg_dma_block_write(rdma,
					      SELFTEST_VIG_IGC_DMA_OFF + SELFTEST_VIG_IGC_LAST,
					      selftest_vig_sentinel,
					      SELFTEST_VIG_IGC_LAST_N);
	if (ret) {
		DPU_ERROR("failed to build the command buffer: %d\n", ret);
		return;
	}

	ret = dpu_reg_dma_kick_off(dpu_kms, rdma, ctl, trigger);

	after_index  = DPU_REG_READ(&hw, SELFTEST_VIG_IGC_CPU_OFF + SELFTEST_VIG_IGC_INDEX);
	after_dither = DPU_REG_READ(&hw, SELFTEST_VIG_IGC_CPU_OFF + SELFTEST_VIG_IGC_DITHER);
	for (i = 0; i < SELFTEST_VIG_IGC_LAST_N; i++)
		last[i] = DPU_REG_READ(&hw, SELFTEST_VIG_IGC_CPU_OFF +
				       SELFTEST_VIG_IGC_LAST + i * sizeof(u32));

	want_index  = trigger ? SELFTEST_VIG_INDEX_ENGINE  : SELFTEST_VIG_INDEX_CPU;
	want_dither = trigger ? SELFTEST_VIG_DITHER_ENGINE : SELFTEST_VIG_DITHER_CPU;
	if (after_index != want_index || after_dither != want_dither)
		pass = false;

	DRM_INFO("selftest: video pipe %s (base %#x) ctl%d %s | cpu-writable %s | index %#x -> expected %#x read %#x | dither %#x -> expected %#x read %#x | %s | last-entry regs (NOT SCORED, the cpu cannot write them) read %#x %#x %#x | kick_off %d\n",
		 vig->name, vig->base, ctl->idx - CTL_0,
		 trigger ? "engine started" : "ENGINE NOT STARTED (control)",
		 writable ? "YES" : "NO -- the read-back below decides nothing",
		 before_index, want_index, after_index,
		 before_dither, want_dither, after_dither,
		 (pass && writable) ? "PASS" : "FAIL",
		 last[0], last[1], last[2], ret);

	/* put the block back exactly as it was found */
	DPU_REG_WRITE(&hw, SELFTEST_VIG_IGC_CPU_OFF + SELFTEST_VIG_IGC_INDEX, orig_index);
	DPU_REG_WRITE(&hw, SELFTEST_VIG_IGC_CPU_OFF + SELFTEST_VIG_IGC_DITHER, orig_dither);
}

/* ------------------------------------------------------------------------ *
 * The vendor's v6 inverse-gamma sequence
 *
 * The lookup table in this block is what the colour-operation work has to
 * program, and the three registers holding the curve's last point are the one
 * part of it nothing has ever been able to write -- not the engine, not the CPU.
 * The standing explanation is that the device's own driver writes them only
 * AFTER a masked modify that turns inverse gamma on and picks a table bank, and
 * nothing here has ever turned anything on.
 *
 * Transcribed from this device's own writer -- reg_dmav1_setup_vig_igcv6() and
 * reg_dmav1_setup_vig_igc_common() -- rather than derived:
 *
 *   1  name the pipe
 *   2  index := 0                          single write        | per channel
 *   3  128 dwords through the data port    indexed run         | ch = 0, 1, 2
 *   4  dither                              single write
 *   5  opmode                              masked modify   <-- turns it ON
 *   6  the curve's last point, 3 registers consecutive run
 *   7  start the engine
 * ------------------------------------------------------------------------ */

/*
 * The opmode register at the block's base, and the mask the vendor modifies it
 * under.
 *
 * THE MASK NAMES THE BITS TO KEEP, NOT THE BITS TO WRITE, and this device's own
 * driver settles that without being asked.  The register is shared with the
 * gamut block -- this device declares both at the same offset -- and the two
 * features modify it under complementary masks: gamut keeps 0x300, which is
 * exactly inverse gamma's two bits, and inverse gamma keeps 0x1001f, which is
 * exactly the field gamut reads its own mode out of.  Under the other reading
 * each feature would be overwriting the other's.
 *
 * So bit 8 enables the lookup table, bit 9 chooses which of two banks it uses,
 * and bits 0-4 and 16 belong to gamut and have to survive.
 */
#define SELFTEST_VIG_IGC_OPMODE		0x000
#define SELFTEST_VIG_IGC_OP_MASK	0x1001f
#define SELFTEST_VIG_IGC_LUT_ENABLE	BIT(8)
#define SELFTEST_VIG_IGC_LUT_SEL	BIT(9)

/* the table's data port: one register per channel, each with its own index */
#define SELFTEST_VIG_IGC_DATA		0x1b4

/* 256 entries a channel, packed two to a register, plus a separately held last point */
#define SELFTEST_VIG_IGC_ENTRIES	256
#define SELFTEST_VIG_IGC_DWORDS		(SELFTEST_VIG_IGC_ENTRIES / 2)
#define SELFTEST_VIG_IGC_CHANNELS	3
#define SELFTEST_VIG_IGC_ENTRY_MASK	(BIT(12) - 1)

/* a four-bit strength beside an enable bit, encoded the way the vendor encodes it */
#define SELFTEST_VIG_DITHER_V6		0x16

/* the entry the read-back asks for by name, chosen away from both ends of the table */
#define SELFTEST_VIG_IGC_PROBE_INDEX	64

/*
 * The table this writes into the block.
 *
 * It is the identity curve for eight-bit input held in a twelve-bit table --
 * entry k is k * 16 -- displaced by the channel number, so a value read back
 * names both the entry it came from and the channel it came through while the
 * curve itself remains one that changes no picture.  That second property is
 * not decoration: step 5 turns inverse gamma ON, and if a plane ever did land
 * on this pipe with a table loaded, an identity curve is the difference between
 * an invisible test and a visibly wrong screen.
 */
static u32 selftest_igc_entry(u32 channel, u32 k)
{
	return (k * 16 + channel) & SELFTEST_VIG_IGC_ENTRY_MASK;
}

/* built once per channel and copied into the command buffer before it is reused */
static u32 selftest_igc_lut[SELFTEST_VIG_IGC_DWORDS];

static void selftest_igc_fill(u32 channel)
{
	u32 j;

	for (j = 0; j < SELFTEST_VIG_IGC_DWORDS; j++)
		selftest_igc_lut[j] = selftest_igc_entry(channel, 2 * j) |
				      selftest_igc_entry(channel, 2 * j + 1) << 16;
}

/*
 * Read one named entry of the table back.
 *
 * The data port is a single register and the index register beside it says
 * which entry that port is pointing at.  Setting the index and then reading
 * names the entry, which is a stronger read-back than reading the port
 * repeatedly and trusting it to advance -- and, unlike that, it does not rest
 * on the advancing being real.
 */
static u32 selftest_igc_read_entry(struct dpu_hw_blk_reg_map *hw, u32 channel,
				   u32 index)
{
	DPU_REG_WRITE(hw, SELFTEST_VIG_IGC_CPU_OFF + SELFTEST_VIG_IGC_INDEX, index);
	return DPU_REG_READ(hw, SELFTEST_VIG_IGC_CPU_OFF + SELFTEST_VIG_IGC_DATA +
			    channel * sizeof(u32));
}

/*
 * Put the block back the way it was found, and say which route did it.
 *
 * The vendor's own off path is the same masked modify carrying zero, so that is
 * tried first.  A CPU write of the original value follows as a second route,
 * because the engine turning the feature ON and then failing to turn it off
 * again is the one outcome of this arm that would outlive the commit.
 */
static void selftest_vig_igc_restore(struct dpu_kms *dpu_kms, struct dpu_hw_ctl *ctl,
				     struct dpu_hw_blk_reg_map *hw, u32 orig_opmode,
				     u32 orig_index, u32 orig_dither)
{
	struct dpu_reg_dma *rdma = &lutdma_selftest_engine;
	u32 by_engine, by_cpu;
	int ret;

	dpu_reg_dma_buf_reset(rdma);
	ret = dpu_reg_dma_decode_sel(rdma, DPU_REG_DMA_BLK_VIG0);
	if (!ret)
		ret = dpu_reg_dma_single_modify(rdma, SELFTEST_VIG_IGC_DMA_OFF +
						SELFTEST_VIG_IGC_OPMODE,
						SELFTEST_VIG_IGC_OP_MASK, 0);
	if (!ret)
		ret = dpu_reg_dma_kick_off(dpu_kms, rdma, ctl, true);

	by_engine = DPU_REG_READ(hw, SELFTEST_VIG_IGC_CPU_OFF + SELFTEST_VIG_IGC_OPMODE);

	DPU_REG_WRITE(hw, SELFTEST_VIG_IGC_CPU_OFF + SELFTEST_VIG_IGC_OPMODE, orig_opmode);
	by_cpu = DPU_REG_READ(hw, SELFTEST_VIG_IGC_CPU_OFF + SELFTEST_VIG_IGC_OPMODE);

	DPU_REG_WRITE(hw, SELFTEST_VIG_IGC_CPU_OFF + SELFTEST_VIG_IGC_INDEX, orig_index);
	DPU_REG_WRITE(hw, SELFTEST_VIG_IGC_CPU_OFF + SELFTEST_VIG_IGC_DITHER, orig_dither);

	DRM_INFO("selftest: v6 restore | wanted %#x | after the engine's off path %#x (%s) | after a cpu write %#x (%s) | kick_off %d\n",
		 orig_opmode, by_engine,
		 by_engine == orig_opmode ? "restored" : "NOT restored",
		 by_cpu,
		 by_cpu == orig_opmode ? "restored" : "STILL NOT RESTORED -- INVERSE GAMMA MAY BE LEFT ON",
		 ret);
}

/**
 * selftest_vig_igc_v6 - run the device's own v6 inverse-gamma sequence
 * @split: submit the enable and the last-point write as TWO transfers instead
 *         of one, and leave the lookup table out
 *
 * WHY THERE ARE TWO SHAPES OF THIS
 * The last-point registers refused a write in a program that was three
 * operations long and refused it again in a program that was one operation
 * long, so program length is not what distinguishes them.  What has never been
 * held constant is whether the feature was enabled first.
 *
 * @split submits exactly the program that already completed cleanly and wrote
 * nothing -- name the pipe, write the three registers -- with one thing changed
 * ahead of it: a transfer that turned the feature on.  One variable against a
 * result already in hand.
 *
 * Without @split this is the whole vendor sequence in a single command buffer,
 * lookup table included, which is what a colour-operation implementation would
 * actually have to submit.  It is the useful program; it is not the clean
 * experiment, because it changes several things at once.
 */
static void selftest_vig_igc_v6(struct dpu_kms *dpu_kms, struct dpu_hw_ctl *ctl,
				bool trigger, bool split)
{
	struct dpu_reg_dma *rdma = &lutdma_selftest_engine;
	const struct dpu_sspp_cfg *vig;
	struct dpu_hw_blk_reg_map hw;
	u32 orig_opmode, orig_index, orig_dither, probe;
	u32 opmode_after, index_after, dither_after;
	u32 last[SELFTEST_VIG_IGC_LAST_N];
	u32 lut_before, lut_at_0, lut_next, lut_at_probe, lut_c1, lut_c2, lut_when_off;
	u32 enable_val, lut_enable, lut_sel;
	bool opmode_cpu_writable;
	int ch, i, ret, ret_second = 0;

	vig = selftest_find_sspp(dpu_kms, SSPP_VIG0);
	if (!vig) {
		DPU_ERROR("no video source pipe in the catalogue\n");
		return;
	}

	memset(&hw, 0, sizeof(hw));
	hw.blk_addr = dpu_kms->mmio + vig->base;
	hw.log_mask = DPU_DBG_MASK_SSPP;

	orig_opmode = DPU_REG_READ(&hw, SELFTEST_VIG_IGC_CPU_OFF + SELFTEST_VIG_IGC_OPMODE);
	orig_index  = DPU_REG_READ(&hw, SELFTEST_VIG_IGC_CPU_OFF + SELFTEST_VIG_IGC_INDEX);
	orig_dither = DPU_REG_READ(&hw, SELFTEST_VIG_IGC_CPU_OFF + SELFTEST_VIG_IGC_DITHER);

	/*
	 * Refuse rather than guess.  The modify keeps only the bits in the mask
	 * and clears the rest, so any bit set outside the mask and outside the
	 * two bits being written is a bit this arm would destroy and the vendor's
	 * own off path would not put back.  Every video pipe read zero here when
	 * this was written; that is a reason to check it, not to assume it.
	 */
	if (orig_opmode & ~SELFTEST_VIG_IGC_OP_MASK) {
		DRM_INFO("selftest: v6 REFUSED -- %s opmode reads %#x, which has bits outside the mask %#x this modify would clear and the off path would not restore\n",
			 vig->name, orig_opmode, SELFTEST_VIG_IGC_OP_MASK);
		return;
	}

	/*
	 * Whether the CPU can write this register at all, established before it
	 * matters.  Bit 9 only chooses between two table banks and the table is
	 * disabled, so the probe cannot change a pixel; the point is to know
	 * whether there is a second way back before taking the first one.
	 */
	DPU_REG_WRITE(&hw, SELFTEST_VIG_IGC_CPU_OFF + SELFTEST_VIG_IGC_OPMODE,
		      orig_opmode | SELFTEST_VIG_IGC_LUT_SEL);
	probe = DPU_REG_READ(&hw, SELFTEST_VIG_IGC_CPU_OFF + SELFTEST_VIG_IGC_OPMODE);
	opmode_cpu_writable = probe == (orig_opmode | SELFTEST_VIG_IGC_LUT_SEL);
	DPU_REG_WRITE(&hw, SELFTEST_VIG_IGC_CPU_OFF + SELFTEST_VIG_IGC_OPMODE, orig_opmode);

	/* what the table's first entry reads as with the feature off, which is the
	 * state the only previous reading of this port was taken in */
	lut_before = selftest_igc_read_entry(&hw, 0, 0);

	/*
	 * The value the modify carries, worked out the way the device's own
	 * driver works it out.  Its second line is written `(~lut_sel) && BIT(0)`
	 * in the original, which is a logical and rather than a bitwise one and
	 * therefore always yields 1; it is transcribed as it stands, because a
	 * tidied version would be a different program from the one the hardware
	 * is known to accept.
	 */
	lut_enable = (orig_opmode >> 8) & BIT(0);
	lut_sel = (orig_opmode >> 9) & BIT(0);
	if (lut_enable)
		lut_sel = 1;
	enable_val = SELFTEST_VIG_IGC_LUT_ENABLE | (lut_sel << 9);

	dpu_reg_dma_buf_reset(rdma);
	ret = dpu_reg_dma_decode_sel(rdma, DPU_REG_DMA_BLK_VIG0);

	if (!split) {
		for (ch = 0; ch < SELFTEST_VIG_IGC_CHANNELS && !ret; ch++) {
			ret = dpu_reg_dma_single_write(rdma, SELFTEST_VIG_IGC_DMA_OFF +
						       SELFTEST_VIG_IGC_INDEX, 0);
			if (ret)
				break;
			selftest_igc_fill(ch);
			ret = dpu_reg_dma_indexed_write(rdma, SELFTEST_VIG_IGC_DMA_OFF +
							SELFTEST_VIG_IGC_DATA +
							ch * sizeof(u32),
							selftest_igc_lut,
							SELFTEST_VIG_IGC_DWORDS);
		}
		if (!ret)
			ret = dpu_reg_dma_single_write(rdma, SELFTEST_VIG_IGC_DMA_OFF +
						       SELFTEST_VIG_IGC_DITHER,
						       SELFTEST_VIG_DITHER_V6);
	}

	if (!ret)
		ret = dpu_reg_dma_single_modify(rdma, SELFTEST_VIG_IGC_DMA_OFF +
						SELFTEST_VIG_IGC_OPMODE,
						SELFTEST_VIG_IGC_OP_MASK, enable_val);

	/*
	 * Split, the last-point write goes in its own transfer so that what is
	 * submitted is byte for byte the program that has already run and
	 * completed, with the enable ahead of it as the only difference.
	 */
	if (!split && !ret)
		ret = dpu_reg_dma_block_write(rdma, SELFTEST_VIG_IGC_DMA_OFF +
					      SELFTEST_VIG_IGC_LAST,
					      selftest_vig_sentinel,
					      SELFTEST_VIG_IGC_LAST_N);
	if (ret) {
		DPU_ERROR("failed to build the command buffer: %d\n", ret);
		return;
	}

	ret = dpu_reg_dma_kick_off(dpu_kms, rdma, ctl, trigger);

	/* read the enable before submitting anything else, so the second transfer
	 * is known to have gone out with the feature in the state it reports */
	opmode_after = DPU_REG_READ(&hw, SELFTEST_VIG_IGC_CPU_OFF + SELFTEST_VIG_IGC_OPMODE);

	if (split) {
		dpu_reg_dma_buf_reset(rdma);
		ret_second = dpu_reg_dma_decode_sel(rdma, DPU_REG_DMA_BLK_VIG0);
		if (!ret_second)
			ret_second = dpu_reg_dma_block_write(rdma,
							     SELFTEST_VIG_IGC_DMA_OFF +
							     SELFTEST_VIG_IGC_LAST,
							     selftest_vig_sentinel,
							     SELFTEST_VIG_IGC_LAST_N);
		if (!ret_second)
			ret_second = dpu_reg_dma_kick_off(dpu_kms, rdma, ctl, trigger);
	}

	/*
	 * The index register before anything touches it.  The sequence resets it
	 * to zero and then pushes 128 values through the port, so a value near
	 * 128 here is the table load reporting itself, independently of reading a
	 * single entry back.
	 */
	index_after  = DPU_REG_READ(&hw, SELFTEST_VIG_IGC_CPU_OFF + SELFTEST_VIG_IGC_INDEX);
	dither_after = DPU_REG_READ(&hw, SELFTEST_VIG_IGC_CPU_OFF + SELFTEST_VIG_IGC_DITHER);
	for (i = 0; i < SELFTEST_VIG_IGC_LAST_N; i++)
		last[i] = DPU_REG_READ(&hw, SELFTEST_VIG_IGC_CPU_OFF +
				       SELFTEST_VIG_IGC_LAST + i * sizeof(u32));

	/* and only now the reads that move the index themselves */
	lut_at_0 = selftest_igc_read_entry(&hw, 0, 0);
	lut_next = DPU_REG_READ(&hw, SELFTEST_VIG_IGC_CPU_OFF + SELFTEST_VIG_IGC_DATA);
	lut_at_probe = selftest_igc_read_entry(&hw, 0, SELFTEST_VIG_IGC_PROBE_INDEX);
	lut_c1 = selftest_igc_read_entry(&hw, 1, 0);
	lut_c2 = selftest_igc_read_entry(&hw, 2, 0);

	DRM_INFO("selftest: v6 %s on %s (base %#x) ctl%d %s | opmode cpu-writable %s | opmode %#x -> wrote %#x under keep-mask %#x -> read %#x (%s) | index %#x -> %#x (%s) | dither %#x -> %#x | last point: expected %#x %#x %#x read %#x %#x %#x -- %s | kick_off %d/%d\n",
		 split ? "ENABLE THEN LAST POINT, two transfers"
		       : "FULL SEQUENCE, one transfer",
		 vig->name, vig->base, ctl->idx - CTL_0,
		 trigger ? "engine started" : "ENGINE NOT STARTED (control)",
		 opmode_cpu_writable ? "YES" : "no",
		 orig_opmode, enable_val, SELFTEST_VIG_IGC_OP_MASK, opmode_after,
		 (opmode_after & SELFTEST_VIG_IGC_LUT_ENABLE) ? "INVERSE GAMMA ON"
							      : "still off",
		 orig_index, index_after,
		 split ? "no table written in this shape"
		       : "expected near 0x80 if the table load ran",
		 orig_dither, dither_after,
		 selftest_vig_sentinel[0], selftest_vig_sentinel[1], selftest_vig_sentinel[2],
		 last[0], last[1], last[2],
		 (trigger && last[0] == selftest_vig_sentinel[0] &&
		  last[1] == selftest_vig_sentinel[1] &&
		  last[2] == selftest_vig_sentinel[2]) ? "LANDED" : "did not land",
		 ret, ret_second);

	DRM_INFO("selftest: v6 %s table read-back on %s (%s) | port with the feature off beforehand %#x | entry 0 %#x, would be %#x | next read %#x, differs from entry 0 if the port advances | entry %d %#x, would be %#x | channel 1 entry 0 %#x, would be %#x | channel 2 entry 0 %#x, would be %#x\n",
		 split ? "ENABLE THEN LAST POINT" : "FULL SEQUENCE", vig->name,
		 split ? "NO TABLE WRITTEN IN THIS SHAPE -- the would-be values are the other shape's"
		       : "table written",
		 lut_before,
		 lut_at_0,
		 selftest_igc_entry(0, 0) | selftest_igc_entry(0, 1) << 16,
		 lut_next,
		 SELFTEST_VIG_IGC_PROBE_INDEX, lut_at_probe,
		 selftest_igc_entry(0, 2 * SELFTEST_VIG_IGC_PROBE_INDEX) |
			selftest_igc_entry(0, 2 * SELFTEST_VIG_IGC_PROBE_INDEX + 1) << 16,
		 lut_c1, selftest_igc_entry(1, 0) | selftest_igc_entry(1, 1) << 16,
		 lut_c2, selftest_igc_entry(2, 0) | selftest_igc_entry(2, 1) << 16);

	/*
	 * Turn it back off before reading the port one last time, so what the
	 * table holds with the feature disabled is on the record beside what it
	 * held with the feature enabled.  A control arm never enabled anything
	 * and has nothing to undo beyond the two registers it probed.
	 */
	if (trigger)
		selftest_vig_igc_restore(dpu_kms, ctl, &hw, orig_opmode,
					 orig_index, orig_dither);
	else
		DPU_REG_WRITE(&hw, SELFTEST_VIG_IGC_CPU_OFF + SELFTEST_VIG_IGC_INDEX,
			      orig_index);

	lut_when_off = selftest_igc_read_entry(&hw, 0, 0);
	DPU_REG_WRITE(&hw, SELFTEST_VIG_IGC_CPU_OFF + SELFTEST_VIG_IGC_INDEX, orig_index);

	DRM_INFO("selftest: v6 %s table entry 0 with the feature off again %#x (it read %#x before the run and %#x while enabled)\n",
		 split ? "ENABLE THEN LAST POINT" : "FULL SEQUENCE",
		 lut_when_off, lut_before, lut_at_0);
}

/**
 * dpu_reg_dma_selftest_run - run a pending engine self-test
 *
 * Called from the atomic commit so the registers are read back in the same
 * context that submitted the transfer, before a later frame's programming could
 * touch them. Does nothing unless the parameter was written since the last
 * commit.
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

	if (!ctl) {
		DPU_ERROR("no control block to submit through\n");
		return;
	}

	trigger = !(lutdma_selftest & SELFTEST_NO_TRIGGER);

	/* a write of 0 restores the colour block and leaves the engine alone */
	if (!(lutdma_selftest & SELFTEST_RUN)) {
		if (dspp && dspp->cap && dspp->cap->sblk &&
		    dspp->cap->sblk->pcc.base) {
			DPU_REG_WRITE(&dspp->hw, dspp->cap->sblk->pcc.base, 0);
			if (ctl->ops.update_pending_flush_dspp)
				ctl->ops.update_pending_flush_dspp(ctl, dspp->idx,
								   DPU_DSPP_PCC);
		}
		DRM_INFO("selftest: colour block restored\n");
		return;
	}

	if (!rdma->ready) {
		DPU_ERROR("the engine was not set up\n");
		return;
	}

	if (lutdma_selftest & SELFTEST_VIG_V6_SPLIT) {
		selftest_vig_igc_v6(dpu_kms, ctl, trigger, true);
		return;
	}

	if (lutdma_selftest & SELFTEST_VIG_V6_FULL) {
		selftest_vig_igc_v6(dpu_kms, ctl, trigger, false);
		return;
	}

	if (lutdma_selftest & SELFTEST_VIG_PLAIN_REGS) {
		selftest_vig_plain_regs(dpu_kms, ctl, trigger);
		return;
	}

	if (lutdma_selftest & SELFTEST_LIVE_DMA_PIPE) {
		selftest_dma_pipe(dpu_kms, ctl, trigger, SSPP_DMA0,
				  "LIVE (carries the desktop)");
		return;
	}

	if (lutdma_selftest & SELFTEST_IDLE_DMA_PIPE) {
		selftest_dma_pipe(dpu_kms, ctl, trigger, SSPP_DMA2,
				  "IDLE (carries nothing)");
		return;
	}

	if (lutdma_selftest & SELFTEST_SOURCE_PIPE) {
		selftest_source_pipe(dpu_kms, ctl, trigger);
		return;
	}

	if (!dspp || !dspp->cap || !dspp->cap->sblk || !dspp->cap->sblk->pcc.base) {
		DPU_ERROR("no colour block to test against\n");
		return;
	}

	pcc = dspp->cap->sblk->pcc.base;
	blk = selftest_decode_sel_for(dspp->idx);
	if (!blk) {
		DPU_ERROR("no decode-select bit for this colour block\n");
		return;
	}

	/*
	 * Establish a known starting value through the register mapping, and
	 * confirm it landed. Without this the read at the end cannot tell a
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
