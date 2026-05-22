/*
 * fbpaint -- first-userspace framebuffer paint probe (freestanding, no libc, aarch64).
 *
 * Produces an UNAMBIGUOUS positive signal that the mainline kernel crossed into
 * our userspace: as PID 1 it floods the bootloader splash framebuffer a vivid
 * solid colour the kernel instrument never uses, so "our pixels appeared" ==
 * "our userspace code executed". (A passive do-nothing init only "freezes",
 * which is hard to distinguish from a hang.)
 *
 * Mechanism (needs CONFIG_STRICT_DEVMEM=n in the boot.img kernel):
 *   1. openat /dev/mem (char 1:1, baked into the initramfs -- no runtime mknod).
 *   2. mmap the bootloader splash FB at PA 0xb8000000, span 0x2b00000 (~45 MiB,
 *      the same region+size the kernel fbdbg_stripe instrument paints).
 *   3. Flood every 32-bit pixel with PURPLE 0xff800080 (NOT in the instrument
 *      palette: cyan/white/magenta/yellow/green/red/orange). Single-colour flood
 *      => pixel byte-order (BGRA vs ARGB) is irrelevant.
 *   4. Cache-maintain: `dc civac` each 64-byte line to the point of coherency,
 *      then `dsb ish`. The display controller DMAs from RAM and is NOT
 *      cache-coherent with the CPU; a plain cached write would be invisible.
 *      arm64 Linux sets SCTLR_EL1.UCI so EL0 `dc civac` is always permitted.
 *   5. ppoll(NULL,0,NULL,...) forever. PID 1 never exits => no panic.
 *
 * Read:
 *   screen goes PURPLE and stays      -> FIRST USERSPACE, clean (no watchdog)
 *   PURPLE then ~7-10s reboot, repeat -> FIRST USERSPACE + a watchdog identified
 *   no purple, same ~7-10s loop       -> /init did not run; escalate
 *
 * Freestanding (no libc): if it fails, the fault is the kernel exec/mmap path,
 * not glibc startup.
 */

#define __NR_openat 56
#define __NR_mmap   222
#define __NR_ppoll  73

#define AT_FDCWD     (-100)
#define O_RDWR       2
#define PROT_RW      3        /* PROT_READ|PROT_WRITE */
#define MAP_SHARED   1

#define FB_PA        0xb8000000UL
#define FB_LEN       0x2b00000UL   /* ~45 MiB, matches fbdbg_stripe region */
#define FB_COLOR     0xff800080U   /* opaque purple; not in the instrument palette */

static const char dev_mem[] = "/dev/mem";

static inline long sys_openat(int dirfd, const char *path, int flags, int mode)
{
	register long x8 asm("x8") = __NR_openat;
	register long x0 asm("x0") = dirfd;
	register long x1 asm("x1") = (long)path;
	register long x2 asm("x2") = flags;
	register long x3 asm("x3") = mode;
	asm volatile("svc #0" : "+r"(x0)
		     : "r"(x8), "r"(x1), "r"(x2), "r"(x3) : "memory", "cc");
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

static inline long sys_ppoll_block(void)
{
	register long x8 asm("x8") = __NR_ppoll;
	register long x0 asm("x0") = 0;
	register long x1 asm("x1") = 0;
	register long x2 asm("x2") = 0;
	register long x3 asm("x3") = 0;
	register long x4 asm("x4") = 0;
	asm volatile("svc #0" : "+r"(x0)
		     : "r"(x8), "r"(x1), "r"(x2), "r"(x3), "r"(x4) : "memory", "cc");
	return x0;
}

void _start(void)
{
	long fd = sys_openat(AT_FDCWD, dev_mem, O_RDWR, 0);
	if (fd >= 0) {
		long ret = sys_mmap(0, FB_LEN, PROT_RW, MAP_SHARED, fd, FB_PA);
		if (ret > 0) {
			volatile unsigned int *fb = (volatile unsigned int *)ret;
			unsigned long words = FB_LEN / 4;
			for (unsigned long i = 0; i < words; i++)
				fb[i] = FB_COLOR;

			/* Clean each cache line to the point of coherency so the
			 * (non-snooping) display controller observes the flood. */
			unsigned char *p   = (unsigned char *)ret;
			unsigned char *end = p + FB_LEN;
			for (; p < end; p += 64)
				asm volatile("dc civac, %0" :: "r"(p) : "memory");
			asm volatile("dsb ish" ::: "memory");
		}
	}

	/* Whether or not the paint succeeded, never exit. */
	for (;;)
		sys_ppoll_block();
}
