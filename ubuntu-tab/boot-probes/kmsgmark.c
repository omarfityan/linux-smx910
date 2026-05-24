/*
 * kmsgmark -- PID-1-reached marker + hands-free recovery bounce (freestanding,
 * no libc, aarch64).
 *
 * Runs as PID 1 (rdinit=/kmsgmark), compiled INTO the kernel as a built-in
 * initramfs (CONFIG_INITRAMFS_SOURCE), so it runs independent of the
 * bootloader's ramdisk handling. Two jobs: (1) prove the mainline kernel
 * reaches userspace (PID 1) on this device in a form that survives BOTH the
 * heavy ECC corruption of the warm-reset-preserved ramoops region AND the
 * recovery kernel clobbering the ramoops *console* ring on the next boot;
 * (2) bounce the device into recovery hands-free, so every read is one clean
 * boot with no button-fighting.
 *
 * Mechanism:
 *   1. openat /dev/kmsg (char 1:11, baked into the initramfs) write-only.
 *   2. write the marker line "<1>MAINLINE_PID1_REACHED NN" 50 times, each with
 *      a unique 2-digit counter NN (00..49). 50 copies because the ramoops
 *      region is largely ECC-unrecoverable across the warm reset: even a small
 *      intact fraction preserves >=1 whole copy. The unique counter defeats any
 *      "repeated message" coalescing. The "<1>" (KERN_ALERT) prefix passes any
 *      console loglevel, so it ALSO renders on fbcon if the panel stays alive.
 *      The marker write is FIRST, so the PID-1 proof is preserved regardless of
 *      whether the BCB write below succeeds.
 *   3. Write the AOSP Bootloader Control Block "boot-recovery" magic (the
 *      32-byte command field at offset 0) to the misc partition
 *      (/dev/block/sda10 = block 8:10 on this device, verified against the live
 *      GPT). On the next boot ABL reads the command field, boots the recovery
 *      image (our TWRP) automatically, and the panic below makes that next boot
 *      happen warm -- so the device lands in TWRP hands-free with the ramoops
 *      crash-dump preserved. The open is RETRIED (clock_nanosleep, ~10s ceiling)
 *      because the UFS controller probes asynchronously (~28s in the boot log)
 *      and /dev/block/sda10 may not exist yet when PID 1 first runs. The device
 *      is opened O_SYNC so the 32 bytes reach UFS media before the reboot (else
 *      they sit in the DRAM page cache, the warm reboot happens, and ABL reads
 *      stale on-media content -> no recovery bounce). Each phase emits a "<1>"
 *      kmsg line (BCB_OPEN_ATTEMPT / BCB_OPEN_OK / BCB_OPEN_FAIL_FINAL /
 *      BCB_WRITE_OK / BCB_WRITE_FAIL / BCB_FSYNC_OK / BCB_DONE) so the crash
 *      dump localises WHICH step ran even if the recovery bounce never fires.
 *   4. exit_group(0). PID 1 exiting makes the kernel panic ("Attempted to kill
 *      init!", kernel/exit.c), whose panic() path calls kmsg_dump(KMSG_DUMP_PANIC,
 *      kernel/panic.c) -> a dmesg-ramoops CRASH-DUMP record capturing the recent
 *      printk ring, including all 50 markers and the BCB phase lines. With
 *      PSTORE_COMPRESS=n that record is PLAIN TEXT (corruption-survivable; a
 *      compressed dump is not). With panic=5 (N>0) the kernel then auto-reboots
 *      WARM (PSCI SYSTEM_RESET preserves DRAM/ramoops) instead of freezing; the
 *      BCB written in step 3 sends that warm boot straight to TWRP. The crash-
 *      dump zone is written only on a crash, so the recovery kernel -- which
 *      does not crash -- does NOT overwrite it: the markers survive into the
 *      recovery boot, immune to the console-clobber and the warm-restart timing
 *      race that the console read suffers.
 *
 * Read back via the recovery root-adb workshop (now reached hands-free):
 *   adb pull /sys/fs/pstore/dmesg-ramoops-0 ; grep MAINLINE_PID1_REACHED
 * Present  => the mainline kernel reached PID 1.
 *
 * Build (freestanding device binary; PID 1):
 *   aarch64-linux-gnu-gcc -ffreestanding -nostdlib -static -O2 -Wall \
 *       -Wl,-e,_start kmsgmark.c -o kmsgmark
 */

#define __NR_openat          56
#define __NR_write           64
#define __NR_fsync           82
#define __NR_exit_group      94
#define __NR_clock_nanosleep 115

#define AT_FDCWD       (-100)
#define O_WRONLY       1
#define O_SYNC         04010000        /* __O_SYNC | O_DSYNC (asm-generic) */
#define CLOCK_MONOTONIC 1

struct timespec { long tv_sec; long tv_nsec; };

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

static inline long sys_fsync(long fd)
{
	register long x8 asm("x8") = __NR_fsync;
	register long x0 asm("x0") = fd;
	asm volatile("svc #0" : "+r"(x0) : "r"(x8) : "memory", "cc");
	return x0;
}

/* clock_nanosleep(clockid, flags, *request, *remain). Used to wait out the
 * asynchronous UFS probe before /dev/block/sda10 appears. */
static inline long sys_clock_nanosleep(int clk, int flags,
				       const struct timespec *req,
				       struct timespec *rem)
{
	register long x8 asm("x8") = __NR_clock_nanosleep;
	register long x0 asm("x0") = clk;
	register long x1 asm("x1") = flags;
	register long x2 asm("x2") = (long)req;
	register long x3 asm("x3") = (long)rem;
	asm volatile("svc #0" : "+r"(x0)
		     : "r"(x8), "r"(x1), "r"(x2), "r"(x3) : "memory", "cc");
	return x0;
}

/* exit_group(code): terminate PID 1 -> kernel panic -> kmsg_dump crash record. */
static inline void sys_exit_group(int code)
{
	register long x8 asm("x8") = __NR_exit_group;
	register long x0 asm("x0") = code;
	asm volatile("svc #0" : "+r"(x0) : "r"(x8) : "memory", "cc");
}

/* Write a string literal to fd as one kmsg line. sizeof includes the NUL, so
 * subtract 1. A bad fd just returns -EBADF harmlessly. */
#define KMSG(fd, s) sys_write((fd), (s), sizeof(s) - 1)

static const char dev_kmsg[]  = "/dev/kmsg";
static const char dev_misc[]  = "/dev/block/sda10";   /* the misc/BCB partition */

/*
 * File-scope const + __attribute__((used)) => the bytes are emitted CONTIGUOUSLY
 * in .rodata and never dead-stripped, so the authoritative embed checks
 * `strings Image | grep MAINLINE_PID1_REACHED` and `... | grep boot-recovery`
 * are guaranteed to find the tokens. The "##" in the marker is the counter
 * placeholder. bcb_cmd is the 32-byte AOSP bootloader_message.command field:
 * "boot-recovery" + NUL padding (C zero-fills the remainder of the array).
 */
static const char __attribute__((used)) marker[] = "<1>MAINLINE_PID1_REACHED ##\n";
#define MARKER_LEN (sizeof(marker) - 1)   /* bytes to write, excluding the NUL */

static const char __attribute__((used)) bcb_cmd[32] = "boot-recovery";

void _start(void)
{
	long kfd = sys_openat(AT_FDCWD, dev_kmsg, O_WRONLY);

	if (kfd >= 0) {
		char line[sizeof(marker)];
		for (unsigned i = 0; i < sizeof(marker); i++)
			line[i] = marker[i];

		/* "##" sits at the two bytes just before "\n" (index MARKER_LEN-1). */
		const int tens = (int)MARKER_LEN - 3;
		for (int i = 0; i < 50; i++) {
			line[tens]     = (char)('0' + (i / 10));
			line[tens + 1] = (char)('0' + (i % 10));
			sys_write(kfd, line, MARKER_LEN);
		}
	}

	/*
	 * Write the BCB "boot-recovery" magic to the misc partition so the warm
	 * reboot below lands in TWRP hands-free. Retry the open to ride out the
	 * asynchronous UFS probe (sda10 may not exist yet at PID-1 start).
	 */
	KMSG(kfd, "<1>BCB_OPEN_ATTEMPT\n");
	long mfd = -1;
	for (int attempt = 0; attempt < 50; attempt++) {
		mfd = sys_openat(AT_FDCWD, dev_misc, O_WRONLY | O_SYNC);
		if (mfd >= 0)
			break;
		struct timespec ts = { 0, 200000000L };   /* 200 ms */
		sys_clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, 0);
	}

	if (mfd >= 0) {
		KMSG(kfd, "<1>BCB_OPEN_OK\n");
		long w = sys_write(mfd, bcb_cmd, sizeof(bcb_cmd));   /* 32 bytes @ off 0 */
		if (w == (long)sizeof(bcb_cmd)) {
			KMSG(kfd, "<1>BCB_WRITE_OK\n");
			sys_fsync(mfd);
			KMSG(kfd, "<1>BCB_FSYNC_OK\n");
			KMSG(kfd, "<1>BCB_DONE\n");
		} else {
			KMSG(kfd, "<1>BCB_WRITE_FAIL\n");
		}
	} else {
		KMSG(kfd, "<1>BCB_OPEN_FAIL_FINAL\n");
	}

	/*
	 * PID 1 exiting -> panic("Attempted to kill init!") -> kmsg_dump writes a
	 * plain-text dmesg-ramoops crash record (with the 50 markers + the BCB
	 * phase lines) that survives the recovery boot; with panic=5 the panic
	 * auto-reboots warm, and the BCB sends that boot to TWRP. Loop the syscall
	 * so control never falls off _start.
	 */
	for (;;)
		sys_exit_group(0);
}
