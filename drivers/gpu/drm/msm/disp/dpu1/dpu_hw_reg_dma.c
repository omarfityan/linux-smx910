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
	"register-DMA engine self-test: 1=post-blend colour block, 5=video source pipe inverse gamma, 9=DMA source pipe carrying the desktop, 17=DMA source pipe carrying nothing, +2 queues without starting the engine (control), 0=restore");

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
