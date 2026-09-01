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
 * What the AOP is asked is configurable. The vendor driver and every previous
 * port hardcode "type: cxpc" -- and cxpc is ONE OF SEVEN types this AOP accepts.
 * The list is not guesswork: it is a contiguous NUL-separated string table in the
 * device's own AOP firmware (aop.mbn), immediately after the field names and
 * immediately before the class name, at 0x10ea0..0x10ede:
 *
 *      dur  flush  ts_adj  log_once  type            <- the message's field names
 *      rbsc  cxpc  rdsyst  rpmh_sys  arc_resc  bcm_resc_low  bcm_resc_high
 *      lpm_mon                                       <- the class name
 *
 * "aoss" has no standalone copy anywhere in the image -- and suffix merging is
 * demonstrably OFF in this pool (mx.lvl/lmx.lvl, cdsp/aop_cdsp, sync/freqsync are all
 * stored separately), so a parser literal would have needed its own. It is not a token.
 * "ddr" is a DIFFERENT case and must not be lumped in with it: a standalone "ddr" DOES
 * exist at 0x1084d, and exact-duplicate merging IS on (289 pool strings, 0 duplicates),
 * so a "type: ddr" literal would alias to it and be invisible in the run above. The
 * string evidence cannot exclude "ddr", and the seven below are a LOWER BOUND.
 *
 * The MODE_AOSS/MODE_DDR constants below are values the AOP may WRITE into the header
 * it produces; a decode table is not a request vocabulary.
 *
 * Asking for any of them leaves the window entirely zero -- but that observation says
 * NOTHING about the token, because a deliberately-invalid token ("zzzz") produces the
 * identical empty window, and so do "rpmh_sys" and "arc_resc", which are indisputably
 * IN the table above. An unactionable lpm_mon message is parsed as far as the class,
 * clears the window, and then does not start a monitor; "never parsed" and "parsed,
 * no monitor" are indistinguishable from outside. Only the presence of a header and
 * records carries information about a type.
 *
 * The AOP echoes the mode byte back in that header, so each token's byte is
 * discoverable rather than assumed -- including tokens whose byte mode_str() has no
 * name for, which print as "unknown" plus the raw value rather than being hidden.
 *
 * The AOP samples every 'dur' ms and writes one record per 'flush' samples, so
 * a record covers dur x flush milliseconds -- measured, not assumed: with
 * dur=1000/flush=10 the record timestamps are 192,000,000 ticks apart, which is
 * 10.000000 s at the 19.2 MHz XO, uniform to +/-2 ticks over 17 intervals.
 *
 * Usage:
 *      echo bcm_resc_high > /sys/kernel/debug/qcom_lpm_violators/type
 *      echo 1000 > /sys/kernel/debug/qcom_lpm_violators/dur_ms  # AOP sample period
 *      echo 10   > /sys/kernel/debug/qcom_lpm_violators/flush   # samples per record
 *      echo 1 > /sys/kernel/debug/qcom_lpm_violators/monitor   # arm the AOP
 *      ... let the system go idle / suspend ...
 *      cat   /sys/kernel/debug/qcom_lpm_violators/violators    # read the log
 *      echo 0 > /sys/kernel/debug/qcom_lpm_violators/monitor   # disarm
 *
 * type/dur_ms/flush/ts_adj take effect at the next arm; they are the AOP's settings,
 * not ours, and only an arm message conveys them.
 *
 * mainline's qmp_send() formats into a 64-byte buffer (QMP_MSG_LEN) where the vendor
 * driver uses 96 (MAX_QMP_MSG_SIZE). Four of the seven type tokens do not fit
 * alongside "ts_adj: 1", and qmp_send() returns -EINVAL rather than truncating. The
 * message shapes below are the ones that fit every token (worst case 60 bytes);
 * ts_adj is optional and an over-length request is refused rather than trimmed.
 */

#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>

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

/* The vendor's defaults, kept so an unconfigured arm behaves as before. */
#define VX_DEFAULT_DUR_MS		1000
#define VX_DEFAULT_FLUSH		10

/*
 * qmp_send() formats into a QMP_MSG_LEN (64) byte buffer and returns -EINVAL
 * rather than truncating, so the message is built and length-checked here to
 * fail cleanly instead of through a WARN_ON in the mailbox driver.
 */
#define VX_QMP_MSG_MAX			64

/*
 * "Blocked the whole window" over one or two records is not a finding, it is a
 * restatement of a single sample. Refuse the claim below this many records.
 */
#define VX_MIN_RECORDS_FOR_CLAIM	3

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
	struct device *dev;
	void __iomem *base;
	struct qmp *qmp;
	struct dentry *dir;
	struct mutex lock;
	const char * const *drvs;
	size_t ndrv;
	bool monitoring;
	const char *type;		/* one of vx_types[] */
	char last_raw[VX_QMP_MSG_MAX];	/* last message sent via the raw knob */
	int last_raw_rc;
	u16 dur_ms;			/* AOP sampling period */
	u8 flush;			/* samples accumulated per record */
	bool ts_adj;			/* ask for the halved timestamp encoding */
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

/*
 * Every "type:" this AOP accepts, read out of the firmware's own string table (see
 * the file comment). Only "cxpc" has ever been used, by anyone. Order is the
 * firmware's.
 */
static const char * const vx_types[] = {
	"rbsc", "cxpc", "rdsyst", "rpmh_sys", "arc_resc",
	"bcm_resc_low", "bcm_resc_high",
};

static const char *vx_type_match(const char *s)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(vx_types); i++)
		if (!strcmp(s, vx_types[i]))
			return vx_types[i];

	return NULL;
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
	seq_printf(s, "Log Entries    : %u\n", log.loglines);
	/* One record per flush samples of dur ms each -- see the file comment. */
	seq_printf(s, "Record Period  : %u ms (dur %u x flush %u)\n\n",
		   hdr->dur_ms * hdr->flush_threshold, hdr->dur_ms,
		   hdr->flush_threshold);

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
	seq_printf(s,
		   "\nBLOCKED THE WHOLE WINDOW (non-zero on all %u record(s)):\n",
		   log.loglines);
	if (log.loglines < VX_MIN_RECORDS_FOR_CLAIM) {
		seq_printf(s,
			   "  (only %u record(s) -- too few to support the claim; arm for at least %u ms)\n",
			   log.loglines,
			   VX_MIN_RECORDS_FOR_CLAIM * hdr->dur_ms *
			   hdr->flush_threshold);
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
	char buf[VX_QMP_MSG_MAX];
	const char *name;
	int len, ret;

	mutex_lock(&dd->lock);
	name = dd->type;

	if (val && dd->ts_adj)
		len = snprintf(buf, sizeof(buf),
			       "{class: lpm_mon, type: %s, dur: %u, flush: %u, ts_adj: 1}",
			       name, dd->dur_ms, dd->flush);
	else if (val)
		len = snprintf(buf, sizeof(buf),
			       "{class: lpm_mon, type: %s, dur: %u, flush: %u}",
			       name, dd->dur_ms, dd->flush);
	else
		len = snprintf(buf, sizeof(buf),
			       "{class: lpm_mon, type: %s, flush: 1, log_once: 1}",
			       name);
	if (len >= (int)sizeof(buf)) {
		dev_err(dd->dev,
			"lpm_mon message is %d bytes, over the %zu-byte QMP limit -- clear ts_adj or use a shorter type\n",
			len, sizeof(buf));
		mutex_unlock(&dd->lock);
		return -EINVAL;
	}

	ret = qmp_send(dd->qmp, "%s", buf);
	if (!ret)
		dd->monitoring = !!val;
	mutex_unlock(&dd->lock);

	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(vx_monitor_fops, vx_monitor_get, vx_monitor_set, "%llu\n");

static ssize_t vx_type_read(struct file *f, char __user *ubuf, size_t len,
			    loff_t *ppos)
{
	struct vx_drvdata *dd = f->private_data;
	char tmp[24];
	int n;

	n = scnprintf(tmp, sizeof(tmp), "%s\n", dd->type);

	return simple_read_from_buffer(ubuf, len, ppos, tmp, n);
}

static ssize_t vx_type_write(struct file *f, const char __user *ubuf, size_t len,
			     loff_t *ppos)
{
	struct vx_drvdata *dd = f->private_data;
	const char *match;
	char tmp[24], *p;

	if (len >= sizeof(tmp))
		return -EINVAL;
	if (copy_from_user(tmp, ubuf, len))
		return -EFAULT;
	tmp[len] = '\0';

	p = strim(tmp);
	match = vx_type_match(p);
	if (!match)
		return -EINVAL;

	mutex_lock(&dd->lock);
	dd->type = match;
	mutex_unlock(&dd->lock);

	return len;
}

static const struct file_operations vx_type_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = vx_type_read,
	.write = vx_type_write,
	.llseek = default_llseek,
};

/*
 * The raw escape hatch. 'type' is deliberately validated against the firmware's own
 * table, which makes it impossible to send a deliberately-INVALID token -- and without
 * that negative control an empty window cannot be read at all: a token the AOP never
 * parsed and a token it parsed but does not implement produce identical output. Both
 * are observed (aoss is absent from the firmware's table; rpmh_sys is in it), so the
 * two cases must be separable by experiment rather than by assumption.
 *
 * It also reaches the sibling CLASSES that sit beside lpm_mon in the same string pool
 * and in the same underscore form -- aoss_slp, ddr_mol, cx_mol, lpm_stress, adb_vote --
 * none of which appears in any known kernel, vendor's included.
 *
 * Root-only debugfs on a debug driver. The AOP's QMP channel is a text protocol that
 * demonstrably ignores what it cannot parse, and a reboot resets it.
 */
static ssize_t vx_raw_read(struct file *f, char __user *ubuf, size_t len, loff_t *ppos)
{
	struct vx_drvdata *dd = f->private_data;
	char tmp[VX_QMP_MSG_MAX + 32];
	int n;

	mutex_lock(&dd->lock);
	n = scnprintf(tmp, sizeof(tmp), "%s rc=%d\n",
		      dd->last_raw[0] ? dd->last_raw : "(none)", dd->last_raw_rc);
	mutex_unlock(&dd->lock);

	return simple_read_from_buffer(ubuf, len, ppos, tmp, n);
}

static ssize_t vx_raw_write(struct file *f, const char __user *ubuf, size_t len,
			    loff_t *ppos)
{
	struct vx_drvdata *dd = f->private_data;
	char msg[VX_QMP_MSG_MAX], *p;
	int ret;

	if (!len || len >= sizeof(msg))
		return -EINVAL;
	if (copy_from_user(msg, ubuf, len))
		return -EFAULT;
	msg[len] = '\0';

	p = strim(msg);
	if (!*p)
		return -EINVAL;

	ret = qmp_send(dd->qmp, "%s", p);

	mutex_lock(&dd->lock);
	strscpy(dd->last_raw, p, sizeof(dd->last_raw));
	dd->last_raw_rc = ret;
	mutex_unlock(&dd->lock);

	dev_info(dd->dev, "raw lpm_mon message %s -> rc %d\n", p, ret);

	return ret ? ret : len;
}

static const struct file_operations vx_raw_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = vx_raw_read,
	.write = vx_raw_write,
	.llseek = default_llseek,
};

static int vx_ts_adj_get(void *data, u64 *val)
{
	struct vx_drvdata *dd = data;

	*val = dd->ts_adj;

	return 0;
}

static int vx_ts_adj_set(void *data, u64 val)
{
	struct vx_drvdata *dd = data;

	if (val > 1)
		return -EINVAL;

	mutex_lock(&dd->lock);
	dd->ts_adj = val;
	mutex_unlock(&dd->lock);

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(vx_ts_adj_fops, vx_ts_adj_get, vx_ts_adj_set, "%llu\n");

static int vx_dur_get(void *data, u64 *val)
{
	struct vx_drvdata *dd = data;

	*val = dd->dur_ms;

	return 0;
}

static int vx_dur_set(void *data, u64 val)
{
	struct vx_drvdata *dd = data;

	if (!val || val > VX_FLAG_MASK_DUR)
		return -EINVAL;

	mutex_lock(&dd->lock);
	dd->dur_ms = val;
	mutex_unlock(&dd->lock);

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(vx_dur_fops, vx_dur_get, vx_dur_set, "%llu\n");

static int vx_flush_get(void *data, u64 *val)
{
	struct vx_drvdata *dd = data;

	*val = dd->flush;

	return 0;
}

static int vx_flush_set(void *data, u64 val)
{
	struct vx_drvdata *dd = data;

	if (!val || val > VX_FLAG_MASK_FLUSH_THRESH)
		return -EINVAL;

	mutex_lock(&dd->lock);
	dd->flush = val;
	mutex_unlock(&dd->lock);

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(vx_flush_fops, vx_flush_get, vx_flush_set, "%llu\n");

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
	dd->dev = &pdev->dev;
	dd->type = vx_type_match("cxpc");
	dd->dur_ms = VX_DEFAULT_DUR_MS;
	dd->flush = VX_DEFAULT_FLUSH;
	platform_set_drvdata(pdev, dd);

	dd->dir = debugfs_create_dir("qcom_lpm_violators", NULL);
	debugfs_create_file("violators", 0400, dd->dir, dd, &vx_violators_fops);
	debugfs_create_file_unsafe("monitor", 0600, dd->dir, dd, &vx_monitor_fops);
	debugfs_create_file("type", 0600, dd->dir, dd, &vx_type_fops);
	debugfs_create_file_unsafe("ts_adj", 0600, dd->dir, dd, &vx_ts_adj_fops);
	debugfs_create_file("raw", 0600, dd->dir, dd, &vx_raw_fops);
	debugfs_create_file_unsafe("dur_ms", 0600, dd->dir, dd, &vx_dur_fops);
	debugfs_create_file_unsafe("flush", 0600, dd->dir, dd, &vx_flush_fops);

	dev_info(&pdev->dev,
		 "AOP low-power-mode violators reporter ready (%zu DRVs); arm with debugfs 'monitor'\n",
		 dd->ndrv);
	dev_info(&pdev->dev,
		 "selectable lpm_mon types: rbsc cxpc rdsyst rpmh_sys arc_resc bcm_resc_low bcm_resc_high\n");

	return 0;
}

static void vx_remove(struct platform_device *pdev)
{
	struct vx_drvdata *dd = platform_get_drvdata(pdev);

	if (dd->monitoring)
		qmp_send(dd->qmp, "{class: lpm_mon, type: %s, flush: 1, log_once: 1}",
			 dd->type);
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
