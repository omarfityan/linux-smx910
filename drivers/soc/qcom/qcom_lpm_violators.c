// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm low-power-mode violators reporter
 *
 * Asks the always-on processor (AOP) to record which SoC subsystems are
 * preventing a system low-power mode -- CX power collapse in particular -- and
 * reports the answer.
 *
 * The AOP is told what to watch with a QMP message; it then writes timestamped
 * records into a message-RAM window, one byte per hardware DRV per sample. A
 * DRV whose byte is non-zero on every sample held the SoC up for the whole
 * window; a sample whose bytes are all zero marks entry to (or exit from) the
 * monitored low-power mode.
 *
 * Ported from the vendor "System PM Violators" driver (sys_pm_vx.c) in the
 * Qualcomm msm-kernel tree, onto mainline's exported qmp_send()/qmp_get()
 * interface. The MSG RAM layout, the QMP message grammar and the per-SoC DRV
 * name table are the vendor's and are reproduced faithfully.
 *
 * Usage:
 *      echo 1 > /sys/kernel/debug/qcom_lpm_violators/monitor   # arm the AOP
 *      ... let the system go idle / suspend ...
 *      cat   /sys/kernel/debug/qcom_lpm_violators/violators    # read the log
 *      echo 0 > /sys/kernel/debug/qcom_lpm_violators/monitor   # disarm
 */

#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/slab.h>

#include <linux/soc/qcom/qcom_aoss.h>

#define MODE_AOSS			0xaa
#define MODE_CXPC			0xcc
#define MODE_DDR			0xdd

#define VX_MODE_MASK_TYPE		0xff
#define VX_MODE_MASK_LOGSIZE		0xff
#define VX_MODE_SHIFT_LOGSIZE		8
#define VX_FLAG_MASK_DUR		0xffff
#define VX_FLAG_MASK_TS			0xff
#define VX_FLAG_SHIFT_TS		16
#define VX_FLAG_MASK_FLUSH_THRESH	0xff
#define VX_FLAG_SHIFT_FLUSH_THRESH	24

/*
 * DRV names are SoC-specific and positional: index i in this table is the DRV
 * whose byte sits at offset i of each record. Taken verbatim from the vendor
 * driver's drv_names_kalama[] -- SM8550 is "kalama".
 */
static const char * const drv_names_sm8550[] = {
	"TZ", "HYP", "HLOS", "L3", "SECPROC", "AUDIO", "AOP", "DEBUG",
	"GPU", "DISPLAY", "COMPUTE_DSP", "TME_SW", "TME_HW", "MDM SW",
	"MDM HW", "WLAN RF", "WLAN BB", "CAM_IFE0", "CAM_IFE1", "CAM_IFE2",
	"DDR AUX", "ARC CPRF",
};

struct vx_header {
	u8 type;
	u8 logsize;
	u16 dur_ms;
	u8 ts_shift;
	u8 flush_threshold;
};

struct vx_data {
	u32 ts;
	u8 *drv_vx;
};

struct vx_log {
	struct vx_header header;
	struct vx_data *data;
	unsigned int loglines;
};

struct vx_drvdata {
	void __iomem *base;
	struct qmp *qmp;
	struct dentry *dir;
	struct mutex lock;
	const char * const *drvs;
	size_t ndrv;
	bool monitoring;
};

static const char *mode_str(u8 type)
{
	switch (type) {
	case MODE_CXPC:	return "CXPC";
	case MODE_AOSS:	return "AOSS";
	case MODE_DDR:	return "DDR";
	default:	return "unknown";
	}
}

static u32 vx_read_word(void __iomem *base, unsigned int *itr)
{
	u32 v = le32_to_cpu(readl_relaxed(base + *itr));

	*itr += sizeof(u32);
	/* keep the message-RAM walk strictly ordered */
	smp_rmb();

	return v;
}

static void vx_free_log(struct vx_log *log)
{
	unsigned int i;

	if (!log->data)
		return;
	for (i = 0; i < log->loglines; i++)
		kfree(log->data[i].drv_vx);
	kfree(log->data);
	log->data = NULL;
}

static int vx_read_log(struct vx_drvdata *dd, struct vx_log *log)
{
	struct vx_header *hdr = &log->header;
	unsigned int itr = 0;
	unsigned int i, j, k;
	u32 val;

	memset(log, 0, sizeof(*log));
	hdr = &log->header;

	val = vx_read_word(dd->base, &itr);
	if (!val)
		return -ENOENT;
	hdr->type = val & VX_MODE_MASK_TYPE;
	hdr->logsize = (val >> VX_MODE_SHIFT_LOGSIZE) & VX_MODE_MASK_LOGSIZE;

	val = vx_read_word(dd->base, &itr);
	if (!val)
		return -ENOENT;
	hdr->dur_ms = val & VX_FLAG_MASK_DUR;
	hdr->ts_shift = (val >> VX_FLAG_SHIFT_TS) & VX_FLAG_MASK_TS;
	hdr->flush_threshold = (val >> VX_FLAG_SHIFT_FLUSH_THRESH) &
			       VX_FLAG_MASK_FLUSH_THRESH;

	if (!hdr->logsize)
		return -ENOENT;

	log->data = kcalloc(hdr->logsize, sizeof(*log->data), GFP_KERNEL);
	if (!log->data)
		return -ENOMEM;

	for (i = 0; i < hdr->logsize; i++) {
		log->data[i].ts = vx_read_word(dd->base, &itr);
		if (!log->data[i].ts)
			break;
		log->data[i].ts <<= hdr->ts_shift;

		log->data[i].drv_vx = kcalloc(ALIGN(dd->ndrv, 4),
					      sizeof(*log->data[i].drv_vx),
					      GFP_KERNEL);
		if (!log->data[i].drv_vx) {
			log->loglines = i;
			vx_free_log(log);
			return -ENOMEM;
		}

		/* one byte per DRV, packed four to a word */
		for (j = 0; j < dd->ndrv;) {
			val = vx_read_word(dd->base, &itr);
			for (k = 0; k < 4; k++)
				log->data[i].drv_vx[j++] = (val >> (8 * k)) & 0xff;
		}
	}
	log->loglines = i;

	return 0;
}

static int vx_violators_show(struct seq_file *s, void *unused)
{
	struct vx_drvdata *dd = s->private;
	struct vx_header *hdr;
	struct vx_log log;
	bool from_exit = false;
	unsigned int i, j;
	int ret;

	mutex_lock(&dd->lock);
	ret = vx_read_log(dd, &log);
	if (ret) {
		mutex_unlock(&dd->lock);
		if (ret == -ENOENT)
			seq_puts(s, "no log recorded -- is the monitor armed? (echo 1 > monitor)\n");
		return ret == -ENOENT ? 0 : ret;
	}
	hdr = &log.header;

	seq_printf(s, "Mode           : %s (%#x)\n", mode_str(hdr->type), hdr->type);
	seq_printf(s, "Duration (ms)  : %u\n", hdr->dur_ms);
	seq_printf(s, "Time Shift     : %u\n", hdr->ts_shift);
	seq_printf(s, "Flush Threshold: %u\n", hdr->flush_threshold);
	seq_printf(s, "Max Log Entries: %u\n", hdr->logsize);
	seq_printf(s, "Log Entries    : %u\n\n", log.loglines);

	seq_puts(s, "Timestamp|");
	for (i = 0; i < dd->ndrv; i++)
		seq_printf(s, "%*s|", 12, dd->drvs[i]);
	seq_puts(s, "\n");

	for (i = 0; i < log.loglines; i++) {
		u32 any = 0;

		seq_printf(s, "%9x|", log.data[i].ts);
		for (j = 0; j < dd->ndrv; j++)
			any |= log.data[i].drv_vx[j];

		/* an all-zero line marks entry to, then exit from, the mode */
		if (!any) {
			seq_printf(s, " %s %s\n", mode_str(hdr->type),
				   from_exit ? "Exit" : "Enter");
			from_exit = !from_exit;
			continue;
		}
		for (j = 0; j < dd->ndrv; j++)
			seq_printf(s, "%*u|", 12, log.data[i].drv_vx[j]);
		seq_puts(s, "\n");
	}

	/*
	 * The headline: a DRV non-zero on EVERY sample never let go for the
	 * whole window. That is the thing that blocked the low-power mode.
	 */
	seq_puts(s, "\nBLOCKED THE WHOLE WINDOW (non-zero on every sample):\n");
	if (!log.loglines) {
		seq_puts(s, "  (no samples)\n");
	} else {
		bool none = true;

		for (i = 0; i < dd->ndrv; i++) {
			bool always = true;

			for (j = 0; j < log.loglines; j++) {
				if (!log.data[j].drv_vx[i]) {
					always = false;
					break;
				}
			}
			if (always) {
				seq_printf(s, "  %s\n", dd->drvs[i]);
				none = false;
			}
		}
		if (none)
			seq_puts(s, "  (none -- no DRV held the mode off for the entire window)\n");
	}

	vx_free_log(&log);
	mutex_unlock(&dd->lock);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(vx_violators);

static int vx_monitor_get(void *data, u64 *val)
{
	struct vx_drvdata *dd = data;

	*val = dd->monitoring;

	return 0;
}

static int vx_monitor_set(void *data, u64 val)
{
	struct vx_drvdata *dd = data;
	int ret;

	mutex_lock(&dd->lock);
	if (val)
		ret = qmp_send(dd->qmp,
			       "{class: lpm_mon, type: cxpc, dur: 1000, flush: 10, ts_adj: 1}");
	else
		ret = qmp_send(dd->qmp,
			       "{class: lpm_mon, type: cxpc, dur: 1000, flush: 1, log_once: 1}");
	if (!ret)
		dd->monitoring = !!val;
	mutex_unlock(&dd->lock);

	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(vx_monitor_fops, vx_monitor_get, vx_monitor_set, "%llu\n");

static int vx_probe(struct platform_device *pdev)
{
	struct vx_drvdata *dd;

	dd = devm_kzalloc(&pdev->dev, sizeof(*dd), GFP_KERNEL);
	if (!dd)
		return -ENOMEM;

	dd->drvs = device_get_match_data(&pdev->dev);
	if (!dd->drvs)
		return -EINVAL;
	dd->ndrv = ARRAY_SIZE(drv_names_sm8550);

	dd->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dd->base))
		return PTR_ERR(dd->base);

	dd->qmp = qmp_get(&pdev->dev);
	if (IS_ERR(dd->qmp))
		return dev_err_probe(&pdev->dev, PTR_ERR(dd->qmp),
				     "failed to get the AOSS QMP handle\n");

	mutex_init(&dd->lock);
	platform_set_drvdata(pdev, dd);

	dd->dir = debugfs_create_dir("qcom_lpm_violators", NULL);
	debugfs_create_file("violators", 0400, dd->dir, dd, &vx_violators_fops);
	debugfs_create_file_unsafe("monitor", 0600, dd->dir, dd, &vx_monitor_fops);

	dev_info(&pdev->dev,
		 "AOP low-power-mode violators reporter ready (%zu DRVs); arm with debugfs 'monitor'\n",
		 dd->ndrv);

	return 0;
}

static void vx_remove(struct platform_device *pdev)
{
	struct vx_drvdata *dd = platform_get_drvdata(pdev);

	if (dd->monitoring)
		qmp_send(dd->qmp,
			 "{class: lpm_mon, type: cxpc, dur: 1000, flush: 1, log_once: 1}");
	debugfs_remove_recursive(dd->dir);
	qmp_put(dd->qmp);
}

static const struct of_device_id vx_of_match[] = {
	{ .compatible = "qcom,sm8550-lpm-violators", .data = drv_names_sm8550 },
	{ }
};
MODULE_DEVICE_TABLE(of, vx_of_match);

static struct platform_driver vx_driver = {
	.probe = vx_probe,
	.remove = vx_remove,
	.driver = {
		.name = "qcom-lpm-violators",
		.of_match_table = vx_of_match,
	},
};
module_platform_driver(vx_driver);

MODULE_DESCRIPTION("Qualcomm low-power-mode violators reporter");
MODULE_LICENSE("GPL");
