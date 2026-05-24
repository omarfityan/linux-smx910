/*
 * kmsgmark -- PID-1-reached marker (freestanding, no libc, aarch64).
 *
 * Runs as PID 1 (rdinit=/kmsgmark), compiled INTO the kernel as a built-in
 * initramfs (CONFIG_INITRAMFS_SOURCE), so it runs independent of the
 * bootloader's ramdisk handling. Its sole job: prove the mainline kernel
 * reaches userspace (PID 1) on this device, in a form that survives BOTH the
 * heavy ECC corruption of the warm-reset-preserved ramoops region AND the
 * recovery kernel clobbering the ramoops *console* ring on the next boot.
 *
 * Mechanism:
 *   1. openat /dev/kmsg (char 1:11, baked into the initramfs) write-only.
 *   2. write the marker line "<1>MAINLINE_PID1_REACHED NN" 50 times, each with
 *      a unique 2-digit counter NN (00..49). 50 copies because the ramoops
 *      region is largely ECC-unrecoverable across the warm reset: even a small
 *      intact fraction preserves >=1 whole copy. The unique counter defeats any
 *      "repeated message" coalescing. The "<1>" (KERN_ALERT) prefix passes any
 *      console loglevel, so it ALSO renders on fbcon if the panel stays alive.
 *   3. exit_group(0). PID 1 exiting makes the kernel panic ("Attempted to kill
 *      init!", kernel/exit.c), whose panic() path calls kmsg_dump(KMSG_DUMP_PANIC,
 *      kernel/panic.c) -> a dmesg-ramoops CRASH-DUMP record capturing the recent
 *      printk ring, including all 50 markers. With PSTORE_COMPRESS=n that record
 *      is PLAIN TEXT (corruption-survivable; a compressed dump is not), and with
 *      panic=0 the kernel then freezes (no silent reboot loop). The crash-dump
 *      zone is written only on a crash, so the recovery kernel -- which does not
 *      crash -- does NOT overwrite it: the marker survives into the recovery
 *      boot, immune to the console-clobber and to the dark-screen warm-restart
 *      timing race that the console read suffers.
 *
 * Read back via the recovery root-adb workshop:
 *   adb pull /sys/fs/pstore/dmesg-ramoops-0 ; grep MAINLINE_PID1_REACHED
 * Present  => the mainline kernel reached PID 1.
 *
 * Build (freestanding device binary; PID 1):
 *   aarch64-linux-gnu-gcc -ffreestanding -nostdlib -static -O2 -Wall \
 *       -Wl,-e,_start kmsgmark.c -o kmsgmark
 */

#define __NR_openat     56
#define __NR_write      64
#define __NR_exit_group 94
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

/* exit_group(code): terminate PID 1 -> kernel panic -> kmsg_dump crash record. */
static inline void sys_exit_group(int code)
{
	register long x8 asm("x8") = __NR_exit_group;
	register long x0 asm("x0") = code;
	asm volatile("svc #0" : "+r"(x0) : "r"(x8) : "memory", "cc");
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

	/*
	 * PID 1 exiting -> panic("Attempted to kill init!") -> kmsg_dump writes a
	 * plain-text dmesg-ramoops crash record (with all 50 markers) that survives
	 * the recovery boot. Loop the syscall so control never falls off _start.
	 */
	for (;;)
		sys_exit_group(0);
}
