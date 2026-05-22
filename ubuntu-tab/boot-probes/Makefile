# Boot-probe userspace: tiny freestanding PID-1 programs baked into the kernel
# via CONFIG_INITRAMFS_SOURCE=ubuntu-tab/boot-probes/initramfs.list.
#
#   kmsgfb    -- renders /dev/kmsg to the framebuffer (the observability renderer)
#   fbpaint   -- floods the framebuffer one colour (first-userspace proof)
#   idle-init -- ppoll-forever stub (minimal first-userspace proof)
#
# The committed `kmsgfb` binary is what the kernel build embeds; rebuild it here
# when kmsgfb.c changes, then re-run the kernel build.

CROSS  ?= aarch64-linux-gnu-
CFLAGS  = -ffreestanding -nostdlib -static -O2 -Wall -Wl,-e,_start

PROBES = kmsgfb fbpaint idle-init

all: $(PROBES)

kmsgfb: kmsgfb.c font8x16.h
	$(CROSS)gcc $(CFLAGS) kmsgfb.c -o $@

fbpaint: fbpaint.c
	$(CROSS)gcc $(CFLAGS) fbpaint.c -o $@

idle-init: idle-init.c
	$(CROSS)gcc $(CFLAGS) idle-init.c -o $@

# host readability simulator for kmsgfb: renders a kmsg snippet to a PPM
#   ./sim sample_kmsg.txt out.ppm
sim: kmsgfb.c font8x16.h
	cc -DHOSTSIM -O2 -Wall kmsgfb.c -o sim

clean:
	rm -f $(PROBES) sim

.PHONY: all clean
