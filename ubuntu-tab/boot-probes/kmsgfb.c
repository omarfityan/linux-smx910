/*
 * kmsgfb -- kernel-log to framebuffer renderer (freestanding, no libc, aarch64).
 *
 * Runs as PID 1 (rdinit=/kmsgfb), compiled INTO the kernel as a built-in
 * initramfs (CONFIG_INITRAMFS_SOURCE), so it is guaranteed to run independent of
 * the bootloader's ramdisk handling. It renders the live kernel log to the
 * screen so we can SEE which initcalls / driver probes succeeded -- a
 * zero-driver, display-driver-independent observability path (the raw bootloader
 * framebuffer scanout is writable from EL0; no DRM driver binds it).
 *
 * Device mechanism (needs CONFIG_STRICT_DEVMEM=n):
 *   1. openat /dev/mem  (char 1:1,  baked into the initramfs)
 *   2. openat /dev/kmsg (char 1:11, baked into the initramfs)
 *   3. mmap the bootloader splash FB at PA 0xb8000000, span 0x2b00000 (45 MiB)
 *   4. read() kmsg one record at a time; the first reads drain the boot
 *      backlog, then read() blocks for new records. Parse "<lvl>,<seq>,<ts>,
 *      <flags>;<message>" -> keep <message> up to the first '\n'.
 *   5. keep the last N lines in a ring; on each new line re-blit the whole text
 *      area white-on-black with a scaled 8x16 glyph font.
 *   6. dc civac each 64-byte line of the FB + dsb ish: the display controller
 *      DMAs from RAM and is NOT cache-coherent with the CPU, so a plain cached
 *      write is invisible. EL0 dc civac is permitted (SCTLR_EL1.UCI on arm64).
 *
 * Geometry: 2960x1848, ARGB8888, stride = 2960*4 = 11840 B/row. The stride is a
 * strong inference (panel width x 4), not empirically locked to +-1px/row -- and
 * 2D glyph text is the first thing that depends on it. So we ALSO draw a
 * few-pixel-wide vertical calibration bar at the left edge: if it is plumb the
 * stride is correct and the text is readable; if it slants, the slant gives the
 * exact stride error to correct.
 *
 * Two compile modes:
 *   cc -DHOSTSIM kmsgfb.c -o sim ; ./sim sample_kmsg.txt out.ppm
 *       native libc; renders a kmsg snippet to a PPM -- the readability gate.
 *   aarch64-linux-gnu-gcc -ffreestanding -nostdlib -static -O2 ...
 *       freestanding device binary; PID 1.
 */

#include "font8x16.h"

/* ---- geometry + style (tune SCALE from the host-sim PPM) ---- */
#define FB_W        2960
#define FB_H        1848
#define FB_STRIDE   2960            /* pixels per row (bytes = *4) */
#define FB_PA       0xb8000000UL
#define FB_LEN      0x2b00000UL

#define SCALE       3               /* each font pixel -> SCALE*SCALE block */
#define GW          (8 * SCALE)     /* glyph width  px */
#define GH          (16 * SCALE)    /* glyph height px */
#define MARGIN      8

#define COL_FG      0xffffffffU      /* white  -- byte-order-immune */
#define COL_BG      0x00000000U      /* black  -- byte-order-immune */

#define NCOLS       ((FB_W - 2 * MARGIN) / GW)
#define NLINES      ((FB_H - 2 * MARGIN) / GH)
#define MAXCOL      256

/* ---- line ring ---- */
static char  ring[NLINES][MAXCOL];
static int   ring_len[NLINES];
static int   ring_start;            /* index of oldest line */
static int   ring_count;            /* lines currently held */

/* ---- pinned live-FDT-dump region (rendered above the kmsg ring) ----
 * MAXFDT > NLINES on purpose: render() caps drawing at NLINES, but a larger
 * collection budget guards against the assembled (base+overlay) tree carrying
 * more matched nodes than the base-only host fixture -- so no rsc node is
 * silently dropped by an early walker stop. */
#define MAXFDT   48
static char fdt_text[MAXFDT][MAXCOL];
static int  fdt_textlen[MAXFDT];
static int  fdt_count;
static unsigned char fdtbuf[2 * 1024 * 1024]; /* base(~450K)+overlay assembled DT */

/* ---- framebuffer base + a put-pixel the two backends share ---- */
static volatile unsigned int *FB;

static inline void put_px(int x, int y, unsigned int c)
{
	if (x < 0 || y < 0 || x >= FB_W || y >= FB_H)
		return;
	FB[(unsigned long)y * FB_STRIDE + x] = c;
}

/* blit one glyph scaled SCALE x, top-left at (px,py) */
static void blit_glyph(int px, int py, unsigned char ch)
{
	const unsigned char *g = &font8x16[(unsigned)ch * 16];
	for (int row = 0; row < 16; row++) {
		unsigned char bits = g[row];
		for (int bit = 0; bit < 8; bit++) {
			unsigned int c = (bits & (0x80 >> bit)) ? COL_FG : COL_BG;
			int bx = px + bit * SCALE;
			int by = py + row * SCALE;
			for (int dy = 0; dy < SCALE; dy++)
				for (int dx = 0; dx < SCALE; dx++)
					put_px(bx + dx, by + dy, c);
		}
	}
}

/* clear the whole FB to black */
static void clear_fb(void)
{
	unsigned long npx = FB_LEN / 4;
	for (unsigned long i = 0; i < npx; i++)
		FB[i] = COL_BG;
}

/* draw the 4px vertical stride-calibration bar at the left edge.
 * Drawn intending "vertical" via offset y*FB_STRIDE; if the true hardware
 * stride differs from FB_STRIDE it appears slanted, and the slant gives the
 * exact stride error. 4px wide so a phone photo can resolve it. */
#define CALIB_W 4
static void calib_line(void)
{
	for (int y = 0; y < FB_H; y++)
		for (int x = 0; x < CALIB_W; x++)
			put_px(x, y, COL_FG);
}

/* re-render the whole text area from the ring */
static void render(void)
{
	/* clear just the text band (not the whole 45 MiB each frame) */
	for (int y = 0; y < NLINES * GH + MARGIN; y++)
		for (int x = CALIB_W; x < FB_W; x++)   /* keep the calib bar */
			put_px(x, y, COL_BG);

	/* draw the ring (holds the last NLINES lines: FDT dump and/or kmsg) */
	for (int i = 0; i < ring_count; i++) {
		int idx = (ring_start + i) % NLINES;
		int py  = MARGIN + i * GH;
		int n   = ring_len[idx];
		if (n > NCOLS) n = NCOLS;
		for (int c = 0; c < n; c++)
			blit_glyph(MARGIN + c * GW, py, (unsigned char)ring[idx][c]);
	}
	calib_line();
}

/* append one log line (already stripped of the kmsg prefix) to the ring */
static void push_line(const char *s, int len)
{
	int idx;
	if (len > MAXCOL - 1) len = MAXCOL - 1;
	if (ring_count < NLINES) {
		idx = (ring_start + ring_count) % NLINES;
		ring_count++;
	} else {
		idx = ring_start;
		ring_start = (ring_start + 1) % NLINES;
	}
	for (int i = 0; i < len; i++) ring[idx][i] = s[i];
	ring_len[idx] = len;
}

/*
 * Parse a kmsg record into the displayed text. Format:
 *   "<level>,<seq>,<ts_us>,<flags>;<message>\n[ optional \n cont lines ]"
 * We keep <message> up to the first '\n'. Prefix the decoded ts (seconds) so
 * the screen reads like dmesg. Returns nothing; pushes one line.
 */
static void handle_record(const char *rec, int reclen)
{
	int i = 0;
	/* skip to ';' (end of metadata) */
	while (i < reclen && rec[i] != ';') i++;
	if (i >= reclen) return;        /* malformed */
	i++;                            /* past ';' */
	const char *msg = rec + i;
	int mlen = 0;
	while (i + mlen < reclen && msg[mlen] != '\n') mlen++;
	push_line(msg, mlen);
}

/* ============================ live-FDT dump ==============================
 * Walk the raw flattened device tree the bootloader handed the kernel
 * (/sys/firmware/fdt on-device, or a .dtb file under HOSTSIM) and render the
 * power/clock backbone nodes the mainline drivers probe. We FILTER BY
 * compatible string (not node name) because the downstream tree may name the
 * nodes differently than mainline -- the compatible is the stable handle.
 *
 * The dumped lines are PINNED at the top of the screen (fdt_text[]); the live
 * kmsg ring renders BELOW them, so one photo shows both the tree the kernel
 * received AND the live probe log (incl. the -22 reject lines).
 *
 * FDT is big-endian; ARM64 reads native little-endian, so every 32-bit field
 * (token / len / nameoff / property cell) goes through be32().
 */
#define FDT_MAGIC       0xd00dfeedU
#define FDT_BEGIN_NODE  0x1U
#define FDT_END_NODE    0x2U
#define FDT_PROP        0x3U
#define FDT_NOP         0x4U
#define FDT_END         0x9U

static unsigned int be32(const unsigned char *p)
{
	return ((unsigned int)p[0] << 24) | ((unsigned int)p[1] << 16) |
	       ((unsigned int)p[2] << 8)  |  (unsigned int)p[3];
}

static int fstreq(const char *a, const char *b)
{
	while (*a && *a == *b) { a++; b++; }
	return *a == *b;
}

/* tiny string builder into a fixed line buffer */
struct sb { char buf[MAXCOL]; int len; };
static void sb_init(struct sb *b)            { b->len = 0; }
static void sb_putc(struct sb *b, char c)    { if (b->len < MAXCOL - 1) b->buf[b->len++] = c; }
static void sb_puts(struct sb *b, const char *s) { while (*s) sb_putc(b, *s++); }
static void sb_puthex(struct sb *b, unsigned int v)
{
	const char *h = "0123456789abcdef";
	sb_puts(b, "0x");
	int started = 0;
	for (int i = 28; i >= 0; i -= 4) {
		int d = (v >> i) & 0xf;
		if (d || started || i == 0) { sb_putc(b, h[d]); started = 1; }
	}
}
/* Route FDT lines through the PROVEN kmsg-ring path (push_line/render), not the
 * separate pinned region -- the ring is known-good (it renders the live log). */
static void push_line(const char *s, int len);
static void fdt_push_sb(struct sb *b)
{
	push_line(b->buf, b->len);
	fdt_count++;        /* kept for the host-sim summary count */
}

/* is the property value a printable string / string-list? */
static int val_is_string(const unsigned char *v, int len)
{
	if (len < 1 || v[len - 1] != 0) return 0;
	for (int i = 0; i < len; i++)
		if (v[i] != 0 && (v[i] < 0x20 || v[i] > 0x7e)) return 0;
	return 1;
}

/* render one property "  name = value" into the pinned region */
static void emit_prop(const char *name, const unsigned char *val, int len)
{
	struct sb b; sb_init(&b);
	sb_puts(&b, "  "); sb_puts(&b, name); sb_puts(&b, " = ");
	if (len == 0) {
		sb_puts(&b, "<empty>");
	} else if (val_is_string(val, len)) {
		for (int i = 0; i < len; i++) {
			if (val[i] == 0) { if (i < len - 1) sb_puts(&b, " | "); }
			else sb_putc(&b, (char)val[i]);
		}
	} else {
		int cells = len / 4;
		for (int c = 0; c < cells && c < 10; c++) {
			if (c) sb_putc(&b, ' ');
			sb_puthex(&b, be32(val + c * 4));
		}
		if (cells > 10) sb_puts(&b, " ...");
	}
	fdt_push_sb(&b);
}

/* the whitelist of properties we render for a matched node */
static int prop_whitelisted(const char *n)
{
	return fstreq(n, "model") || fstreq(n, "compatible") ||
	       fstreq(n, "label")  || fstreq(n, "reg") ||
	       fstreq(n, "reg-names") || fstreq(n, "qcom,drv-id") ||
	       fstreq(n, "qcom,tcs-offset") || fstreq(n, "qcom,tcs-config");
}

/* one collected property of the currently-open node */
struct fprop { const char *name; const unsigned char *val; int len; };

static int substr(const char *hay, const char *needle);

/* Walk the FDT. Emits the root (model/compatible) and every node whose
 * compatible contains "rpmh-rsc" or "cmd-db", plus pinctrl nodes. */
static void fdt_dump(const unsigned char *fdt, unsigned int len)
{
	if (len < 40 || be32(fdt) != FDT_MAGIC) {
		struct sb b; sb_init(&b);
		sb_puts(&b, "FDT: bad magic / short read len="); sb_puthex(&b, len);
		fdt_push_sb(&b);
		return;
	}
	unsigned int totalsize   = be32(fdt + 4);
	unsigned int off_struct  = be32(fdt + 8);
	unsigned int off_strings = be32(fdt + 12);
	/* Defensive: a short/truncated read makes off_strings point outside the
	 * buffer; reading property NAMES from there would segfault PID 1. Bail
	 * loudly instead. (The 2 MB buffer should hold the whole assembled tree.) */
	if (totalsize > len || off_struct >= len || off_strings >= len) {
		struct sb b; sb_init(&b);
		sb_puts(&b, "FDT: truncated read total="); sb_puthex(&b, totalsize);
		sb_puts(&b, " got=");       sb_puthex(&b, len);
		sb_puts(&b, " stroff=");    sb_puthex(&b, off_strings);
		fdt_push_sb(&b);
		return;
	}
	const unsigned char *strs = fdt + off_strings;
	const unsigned char *p    = fdt + off_struct;
	const unsigned char *end  = fdt + (totalsize < len ? totalsize : len);

	/* current open node state (FDT guarantees props precede subnodes) */
	char cur_name[64]; int cur_namelen = 0;
	int  depth = 0, pending = 0, flushed = 0;
	static struct fprop props[48]; int nprops = 0;

	/* close out the currently-open node: if it's a target, render it */
	/* (manual inline because we call it from two token sites) */
	#define MAYBE_FLUSH() do {                                              \
		if (pending && !flushed) {                                      \
			int is_rsc = 0, is_cmddb = 0;                           \
			for (int i = 0; i < nprops; i++)                        \
				if (fstreq(props[i].name, "compatible")) {      \
					const unsigned char *v = props[i].val;  \
					int l = props[i].len, s = 0;            \
					for (int k = 0; k < l; k++) {           \
						if (substr((const char*)v+s, "rpmh-rsc")) is_rsc = 1; \
						if (substr((const char*)v+s, "cmd-db"))   is_cmddb = 1; \
						while (k < l && v[k]) k++;       \
						s = k + 1;                       \
					}                                       \
				}                                               \
			int is_root = (depth == 1);                             \
			if (is_root || is_rsc || is_cmddb) {                    \
				struct sb h; sb_init(&h);                       \
				sb_putc(&h, is_root ? '/' : '@');               \
				for (int i = 0; i < cur_namelen; i++) sb_putc(&h, cur_name[i]); \
				fdt_push_sb(&h);                                \
				for (int i = 0; i < nprops; i++)                \
					if (prop_whitelisted(props[i].name))    \
						emit_prop(props[i].name, props[i].val, props[i].len); \
				if (is_rsc) {                                   \
					static const char *need[3] = {"qcom,drv-id","qcom,tcs-offset","qcom,tcs-config"}; \
					for (int q = 0; q < 3; q++) {           \
						int have = 0;                   \
						for (int i = 0; i < nprops; i++) if (fstreq(props[i].name, need[q])) have = 1; \
						if (!have) {                    \
							struct sb a; sb_init(&a); \
							sb_puts(&a, "  "); sb_puts(&a, need[q]); sb_puts(&a, " = <ABSENT>"); \
							fdt_push_sb(&a);        \
						}                               \
					}                                       \
				}                                               \
			}                                                       \
			flushed = 1;                                            \
		}                                                               \
	} while (0)

	while (p + 4 <= end && fdt_count < MAXFDT) {
		unsigned int tok = be32(p); p += 4;
		if (tok == FDT_BEGIN_NODE) {
			MAYBE_FLUSH();                  /* close the parent */
			const char *nm = (const char *)p;
			int l = 0; while (p + l < end && nm[l]) l++;
			/* advance past name + NUL, align to 4 */
			p += l + 1;
			p = fdt + (((p - fdt) + 3) & ~3UL);
			depth++;
			cur_namelen = l < 63 ? l : 63;
			for (int i = 0; i < cur_namelen; i++) cur_name[i] = nm[i];
			nprops = 0; pending = 1; flushed = 0;
		} else if (tok == FDT_PROP) {
			if (p + 8 > end) break;
			unsigned int plen = be32(p);
			unsigned int noff = be32(p + 4);
			p += 8;
			if (pending && nprops < 48) {
				props[nprops].name = (const char *)(strs + noff);
				props[nprops].val  = p;
				props[nprops].len  = (int)plen;
				nprops++;
			}
			p += plen;
			p = fdt + (((p - fdt) + 3) & ~3UL);
		} else if (tok == FDT_END_NODE) {
			MAYBE_FLUSH();
			depth--;
			pending = 0;                    /* parent has no further own props */
		} else if (tok == FDT_NOP) {
			/* skip */
		} else if (tok == FDT_END) {
			break;
		} else {
			break;                          /* malformed */
		}
	}
	#undef MAYBE_FLUSH
}

/* substring test used by the compatible matcher */
static int substr(const char *hay, const char *needle)
{
	for (; *hay; hay++) {
		const char *h = hay, *n = needle;
		while (*h && *n && *h == *n) { h++; n++; }
		if (!*n) return 1;
	}
	return 0;
}

/* ============================ HOST-SIM backend ============================ */
#ifdef HOSTSIM
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv)
{
	if (argc < 3) { fprintf(stderr, "usage: %s kmsg.txt out.ppm [fdt.dtb]\n", argv[0]); return 2; }
	FILE *f = fopen(argv[1], "rb");
	if (!f) { perror("open kmsg"); return 1; }

	/* allocate a host framebuffer at the real geometry */
	unsigned int *buf = calloc((size_t)FB_W * FB_H, 4);
	if (!buf) { perror("calloc"); return 1; }
	FB = buf;

	/* optional: parse a real .dtb to verify the live-FDT walker render */
	if (argc >= 4) {
		FILE *d = fopen(argv[3], "rb");
		if (!d) { perror("open fdt"); return 1; }
		size_t got = fread(fdtbuf, 1, sizeof(fdtbuf), d);
		fclose(d);
		fdt_dump(fdtbuf, (unsigned int)got);
		fprintf(stderr, "FDT: parsed %zu bytes -> %d ring lines (cap %d)\n", got, ring_count, NLINES);
		for (int i = 0; i < ring_count; i++) {
			int idx = (ring_start + i) % NLINES;
			fprintf(stderr, "  | %.*s\n", ring_len[idx], ring[idx]);
		}
	}

	/* feed lines: each input line is one kmsg record (\n-terminated) */
	char line[8192];
	while (fgets(line, sizeof line, f)) {
		int len = (int)strlen(line);
		while (len && (line[len-1] == '\n' || line[len-1] == '\r')) len--;
		handle_record(line, len);
	}
	fclose(f);

	render();

	fprintf(stderr, "geometry: %dx%d stride=%dpx  SCALE=%d  glyph=%dx%d  grid=%d cols x %d lines\n",
		FB_W, FB_H, FB_STRIDE, SCALE, GW, GH, NCOLS, NLINES);

	FILE *o = fopen(argv[2], "wb");
	if (!o) { perror("open ppm"); return 1; }
	fprintf(o, "P6\n%d %d\n255\n", FB_W, FB_H);
	for (long p = 0; p < (long)FB_W * FB_H; p++) {
		unsigned int v = buf[p];
		unsigned char rgb[3] = { (v >> 16) & 0xff, (v >> 8) & 0xff, v & 0xff };
		fwrite(rgb, 1, 3, o);
	}
	fclose(o);
	return 0;
}

/* ============================ DEVICE backend ============================= */
#else

#define __NR_openat 56
#define __NR_read   63
#define __NR_mmap   222
#define __NR_ppoll  73
#define __NR_mount  40
#define AT_FDCWD    (-100)
#define O_RDONLY    0
#define O_RDWR      2
#define O_NONBLOCK  0x800
#define PROT_RW     3
#define MAP_SHARED  1
#define POLLIN      1
#define EAGAIN      11

struct pollfd { int fd; short events; short revents; };

static const char dev_mem[]   = "/dev/mem";
static const char dev_kmsg[]  = "/dev/kmsg";
static const char sysfs_src[] = "sysfs";
static const char sys_dir[]   = "/sys";
static const char fdt_path[]  = "/sys/firmware/fdt";

static inline long sys_openat(int dirfd, const char *path, int flags)
{
	register long x8 asm("x8") = __NR_openat;
	register long x0 asm("x0") = dirfd;
	register long x1 asm("x1") = (long)path;
	register long x2 asm("x2") = flags;
	register long x3 asm("x3") = 0;
	asm volatile("svc #0" : "+r"(x0)
		     : "r"(x8), "r"(x1), "r"(x2), "r"(x3) : "memory", "cc");
	return x0;
}

static inline long sys_read(long fd, void *buf, unsigned long n)
{
	register long x8 asm("x8") = __NR_read;
	register long x0 asm("x0") = fd;
	register long x1 asm("x1") = (long)buf;
	register long x2 asm("x2") = (long)n;
	asm volatile("svc #0" : "+r"(x0)
		     : "r"(x8), "r"(x1), "r"(x2) : "memory", "cc");
	return x0;
}

static inline long sys_mmap(unsigned long addr, unsigned long len, long prot,
			    long flags, long fd, unsigned long off)
{
	register long x8 asm("x8") = __NR_mmap;
	register long x0 asm("x0") = (long)addr;
	register long x1 asm("x1") = (long)len;
	register long x2 asm("x2") = prot;
	register long x3 asm("x3") = flags;
	register long x4 asm("x4") = fd;
	register long x5 asm("x5") = (long)off;
	asm volatile("svc #0" : "+r"(x0)
		     : "r"(x8), "r"(x1), "r"(x2), "r"(x3), "r"(x4), "r"(x5)
		     : "memory", "cc");
	return x0;
}

static inline long sys_mount(const char *src, const char *tgt, const char *fst,
			     unsigned long flags, const void *data)
{
	register long x8 asm("x8") = __NR_mount;
	register long x0 asm("x0") = (long)src;
	register long x1 asm("x1") = (long)tgt;
	register long x2 asm("x2") = (long)fst;
	register long x3 asm("x3") = (long)flags;
	register long x4 asm("x4") = (long)data;
	asm volatile("svc #0" : "+r"(x0)
		     : "r"(x8), "r"(x1), "r"(x2), "r"(x3), "r"(x4) : "memory", "cc");
	return x0;
}

/* mount sysfs, read /sys/firmware/fdt, render the backbone nodes into the
 * pinned region. On any failure push one observable error line and return --
 * the kmsg loop still runs, so the boot stays observable. */
static void dump_live_fdt(void)
{
	struct sb b;
	long m = sys_mount(sysfs_src, sys_dir, sysfs_src, 0, 0);
	long fd = sys_openat(AT_FDCWD, fdt_path, O_RDONLY);
	/* TRACE: always emit one line so we can see mount/open results on screen */
	sb_init(&b);
	sb_puts(&b, "FDT-DBG mount="); sb_puthex(&b, (unsigned int)(long)m);
	sb_puts(&b, " fd=");          sb_puthex(&b, (unsigned int)(long)fd);
	fdt_push_sb(&b);
	if (fd < 0)
		return;
	unsigned long total = 0;
	for (;;) {
		long n = sys_read(fd, fdtbuf + total, sizeof(fdtbuf) - total);
		if (n <= 0) break;
		total += (unsigned long)n;
		if (total >= sizeof(fdtbuf)) break;
	}
	sb_init(&b);
	sb_puts(&b, "FDT-DBG total="); sb_puthex(&b, (unsigned int)total);
	sb_puts(&b, " magic=");        sb_puthex(&b, total >= 4 ? be32(fdtbuf) : 0);
	fdt_push_sb(&b);
	fdt_dump(fdtbuf, (unsigned int)total);
}

/* block until the kmsg fd has a new record (POLLIN), timeout=NULL */
static inline long sys_ppoll(struct pollfd *fds, unsigned long n)
{
	register long x8 asm("x8") = __NR_ppoll;
	register long x0 asm("x0") = (long)fds;
	register long x1 asm("x1") = (long)n;
	register long x2 asm("x2") = 0;   /* timeout NULL = wait forever */
	register long x3 asm("x3") = 0;   /* sigmask NULL */
	register long x4 asm("x4") = 0;
	asm volatile("svc #0" : "+r"(x0)
		     : "r"(x8), "r"(x1), "r"(x2), "r"(x3), "r"(x4) : "memory", "cc");
	return x0;
}

/* flush the active panel (2960x1848x4 ~ 22 MiB) to PoC for the display DMA */
static void fb_flush(unsigned long base)
{
	unsigned char *p   = (unsigned char *)base;
	unsigned char *end = p + (unsigned long)FB_W * FB_H * 4;
	for (; p < end; p += 64)
		asm volatile("dc civac, %0" :: "r"(p) : "memory");
	asm volatile("dsb ish" ::: "memory");
}

void _start(void)
{
	long memfd = sys_openat(AT_FDCWD, dev_mem, O_RDWR);
	long kfd   = sys_openat(AT_FDCWD, dev_kmsg, O_RDONLY | O_NONBLOCK);

	if (memfd >= 0) {
		long ret = sys_mmap(0, FB_LEN, PROT_RW, MAP_SHARED, memfd, FB_PA);
		if (ret > 0)
			FB = (volatile unsigned int *)ret;
	}

	(void)kfd;
	/* banner (proves THIS binary is running) + the live FDT dump, pushed into
	 * the ring -- pure FDT screen so apps_rsc stays on-screen (no kmsg backlog
	 * competing for the 38 ring slots) */
	static const char banner[] = "==== LIVE FDT DUMP v3 (2MB buf) ====";
	push_line(banner, sizeof(banner) - 1);
	/* paint the banner IMMEDIATELY -- a canary: if this shows on the next boot,
	 * our binary is running (vs an A/B rollback to the old boot) */
	if (FB) { clear_fb(); render(); fb_flush((unsigned long)FB); }

	dump_live_fdt();

	if (FB) {
		clear_fb();
		render();
		fb_flush((unsigned long)FB);
	}

	/* freeze on the FDT dump for a stable photo (we already have the -22 kmsg
	 * evidence from sess-102 + the earlier boots) */
	struct pollfd dummy = { 0, 0, 0 };
	for (;;) sys_ppoll(&dummy, 0);
}
#endif
