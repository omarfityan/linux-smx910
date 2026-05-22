/*
 * idle-init -- minimal first-userspace probe /init (freestanding, no libc, aarch64).
 *
 * Proves the mainline kernel crosses from kernel space into
 * userspace on the Galaxy Tab S9 Ultra (SM-X910). No libc, no files, no
 * framebuffer (STRICT_DEVMEM blocks /dev/mem on the RAM-backed FB), no
 * driver dependency whatsoever.
 *
 * Behaviour: issue exactly one syscall (ppoll with no fds and an infinite
 * timeout) in an endless loop. PID 1 therefore never exits, so the kernel
 * never panics ("Attempted to kill init") and the device STOPS its
 * panic-reboot-loop and freezes on the last on-panel state.
 *
 *   reboot-loop continues  -> /init never exec'd (or died at startup)
 *   device freezes (>=60s) -> /init exec'd and survived = FIRST USERSPACE
 *
 * Built freestanding so that if it fails to run, the fault is the kernel's
 * exec path, not glibc startup (TLS / brk / rseq / set_tid_address ...).
 */

#define __NR_ppoll 73   /* asm-generic syscall number, used by arm64 */

static inline long sys_ppoll_block(void)
{
	register long x8 asm("x8") = __NR_ppoll;
	register long x0 asm("x0") = 0;	/* struct pollfd *fds = NULL */
	register long x1 asm("x1") = 0;	/* nfds_t nfds = 0           */
	register long x2 asm("x2") = 0;	/* timespec *tmo_p = NULL (block forever) */
	register long x3 asm("x3") = 0;	/* sigset_t *sigmask = NULL  */
	register long x4 asm("x4") = 0;	/* size_t sigsetsize = 0     */

	asm volatile("svc #0"
		     : "+r"(x0)
		     : "r"(x8), "r"(x1), "r"(x2), "r"(x3), "r"(x4)
		     : "memory", "cc");
	return x0;
}

void _start(void)
{
	/* Block forever. ppoll(NULL, 0, NULL, ...) sleeps indefinitely with no
	 * fds; if it ever returns (e.g. -EINTR), just loop again. PID 1 must
	 * never return / exit. */
	for (;;)
		sys_ppoll_block();
}
