// SPDX-License-Identifier: MIT
/*
 * rootsh -- a setuid-root re-exec shim for the bring-up initramfs.
 *
 * The SELinux-free Linux adbd (Debian android-tools-adbd 5.1.1) drops the
 * interactive `adb shell` to AID_SHELL (uid 2000) even in insecure mode, and
 * there is no Android property service to flip service.adb.root. The serial
 * (ttyGS0) shell is uid 0 only because /init (PID 1, uid 0) execs it directly
 * without dropping. Installed setuid-root (initramfs.list mode 4755), this
 * shim restores uid 0 and execs an interactive busybox shell, giving a root
 * shell over adb (insmod / devmem / dd) without needing the serial line.
 *
 * Usage on device:  adb shell   ->   rootsh
 *
 * Purely additive: nothing in /init or the served shells references it, so it
 * cannot affect boot or the existing serial-root fallback.
 */
#include <unistd.h>

int main(void)
{
	/* setuid-root exec: euid is already 0 (the 4755 bit), so both calls
	 * succeed and set the real/effective/saved ids to 0 = full root. */
	setgid(0);
	setuid(0);
	execl("/bin/busybox", "sh", (char *)0);
	return 127;	/* exec failed */
}
