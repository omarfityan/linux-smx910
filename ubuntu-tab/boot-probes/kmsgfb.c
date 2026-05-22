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

/* ============================ HOST-SIM backend ============================ */
#ifdef HOSTSIM
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv)
{
	if (argc < 3) { fprintf(stderr, "usage: %s kmsg.txt out.ppm\n", argv[0]); return 2; }
	FILE *f = fopen(argv[1], "rb");
	if (!f) { perror("open kmsg"); return 1; }

	/* allocate a host framebuffer at the real geometry */
	unsigned int *buf = calloc((size_t)FB_W * FB_H, 4);
	if (!buf) { perror("calloc"); return 1; }
	FB = buf;

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
#define AT_FDCWD    (-100)
#define O_RDONLY    0
#define O_RDWR      2
#define O_NONBLOCK  0x800
#define PROT_RW     3
#define MAP_SHARED  1
#define POLLIN      1
#define EAGAIN      11

struct pollfd { int fd; short events; short revents; };

static const char dev_mem[]  = "/dev/mem";
static const char dev_kmsg[] = "/dev/kmsg";

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

	if (FB) {
		clear_fb();
		/* first paint: prove userspace + the calib bar, even before kmsg */
		render();
		fb_flush((unsigned long)FB);
	}

	if (kfd < 0 || !FB) {
		/* no kmsg or no FB: block forever so PID 1 never exits */
		struct pollfd dummy = { 0, 0, 0 };
		for (;;) sys_ppoll(&dummy, 0);
	}

	static char rec[8192];
	struct pollfd pfd = { (int)kfd, POLLIN, 0 };
	for (;;) {
		/* drain every record currently available into the ring */
		for (;;) {
			long n = sys_read(kfd, rec, sizeof rec);
			if (n > 0) {
				handle_record(rec, (int)n);
				continue;
			}
			if (n == -EAGAIN)
				break;          /* backlog drained */
			if (n == 0)
				break;
			/* -EPIPE etc (records overran the ring): retry read */
		}
		/* one paint per burst, then wait for the next record */
		render();
		fb_flush((unsigned long)FB);
		pfd.revents = 0;
		sys_ppoll(&pfd, 1);
	}
}
#endif
