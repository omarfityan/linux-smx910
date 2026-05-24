/*
 * kmsgmark -- PID-1-reached marker (freestanding, no libc, aarch64).
 *
 * Runs as PID 1 (rdinit=/kmsgmark), compiled INTO the kernel as a built-in
 * initramfs (CONFIG_INITRAMFS_SOURCE), so it runs independent of the
 * bootloader's ramdisk handling. Its sole job: prove the mainline kernel
 * reaches userspace (PID 1) on this device, in a form that survives the heavy
 * ECC corruption of the warm-reset-preserved ramoops console region.
 *
 * Mechanism:
 *   1. openat /dev/kmsg (char 1:11, baked into the initramfs) write-only.
 *   2. write the marker line "<1>MAINLINE_PID1_REACHED NN" 50 times, each with
 *      a unique 2-digit counter NN (00..49). Each write() to /dev/kmsg is ONE
 *      record -> one printk ring entry -> one PSTORE_CONSOLE record in the
 *      ramoops console region. 50 copies because that region is largely
 *      ECC-unrecoverable across the warm reset: even a small intact fraction
 *      preserves >=1 whole copy. The unique counter defeats any "repeated
 *      message" coalescing that could collapse identical lines.
 *      The "<1>" (KERN_ALERT) prefix guarantees the line passes any console
 *      loglevel filter, so it ALSO renders on fbcon if the panel stays alive.
 *   3. ppoll(NULL,0,NULL) forever: PID 1 must never exit (else the kernel
 *      panics "Attempted to kill init"). The marker is written; we just hold.
 *
 * Read back via the TWRP root-adb workshop:
 *   adb pull /sys/fs/pstore/console-ramoops ; grep MAINLINE_PID1_REACHED
 * Present  => the mainline kernel reached PID 1.
 * Absent + the initcall_debug trail still topping mid-device_initcall
 *          => it hung before userspace.
 *
 * Build (freestanding device binary; PID 1):
 *   aarch64-linux-gnu-gcc -ffreestanding -nostdlib -static -O2 -Wall \
 *       -Wl,-e,_start kmsgmark.c -o kmsgmark
 */

#define __NR_openat 56
#define __NR_write  64
#define __NR_ppoll  73
#define AT_FDCWD    (-100)
#define O_WRONLY    1

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

static inline long sys_write(long fd, const void *buf, unsigned long n)
{
	register long x8 asm("x8") = __NR_write;
	register long x0 asm("x0") = fd;
	register long x1 asm("x1") = (long)buf;
	register long x2 asm("x2") = (long)n;
	asm volatile("svc #0" : "+r"(x0)
		     : "r"(x8), "r"(x1), "r"(x2) : "memory", "cc");
	return x0;
}

/* ppoll(NULL, 0, NULL, NULL, 0): block forever, no fds. */
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

static const char dev_kmsg[] = "/dev/kmsg";

/*
 * File-scope const + __attribute__((used)) => the marker bytes are emitted
 * CONTIGUOUSLY in .rodata and never dead-stripped, so the authoritative embed
 * check `strings Image | grep MAINLINE_PID1_REACHED` is guaranteed to find the
 * token (a local stack array under -O2 could be materialised as scattered
 * immediates and leave no contiguous run). The "##" is the counter placeholder.
 */
static const char __attribute__((used)) marker[] = "<1>MAINLINE_PID1_REACHED ##\n";
#define MARKER_LEN (sizeof(marker) - 1)   /* bytes to write, excluding the NUL */

void _start(void)
{
	long fd = sys_openat(AT_FDCWD, dev_kmsg, O_WRONLY);

	if (fd >= 0) {
		char line[sizeof(marker)];
		for (unsigned i = 0; i < sizeof(marker); i++)
			line[i] = marker[i];

		/* "##" sits at the two bytes just before "\n" (index MARKER_LEN-1). */
		const int tens = (int)MARKER_LEN - 3;
		for (int i = 0; i < 50; i++) {
			line[tens]     = (char)('0' + (i / 10));
			line[tens + 1] = (char)('0' + (i % 10));
			sys_write(fd, line, MARKER_LEN);
		}
	}

	/* PID 1 must never exit. */
	for (;;)
		sys_ppoll_block();
}
