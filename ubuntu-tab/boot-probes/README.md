# boot-probes — freestanding first-userspace / observability probes

Tiny no-libc aarch64 programs that run as PID 1 (`rdinit=`), compiled into the
kernel as a built-in initramfs
(`CONFIG_INITRAMFS_SOURCE=ubuntu-tab/boot-probes/initramfs.list`). They need no
drivers and no rootfs — they exist to prove the mainline kernel reaches
userspace on the Galaxy Tab S9 Ultra (SM8550) and to get the kernel log out
before any real driver binds.

| Program | What it does |
|---|---|
| `kmsgfb`    | Reads `/dev/kmsg` and renders the kernel log to the bootloader framebuffer (white-on-black, scaled 8x16 font). The observability renderer. |
| `fbpaint`   | Floods the framebuffer one solid colour — unambiguous "our userspace ran" proof. |
| `idle-init` | `ppoll`-forever stub — minimal "kernel reached userspace and survived" proof. |

## Requirements
- `CONFIG_STRICT_DEVMEM=n` — so userspace can `mmap` the bootloader framebuffer at PA `0xb8000000`.
- `CONFIG_INITRAMFS_SOURCE="ubuntu-tab/boot-probes/initramfs.list"` + `rdinit=/kmsgfb` on the kernel cmdline.

## Build
```
make            # cross-compile kmsgfb, fbpaint, idle-init  (override CROSS=… if needed)
make sim        # host kmsgfb readability simulator:  ./sim sample_kmsg.txt out.ppm
```
The committed `kmsgfb` binary is what the kernel build embeds via
`initramfs.list`; rebuild it with `make` when `kmsgfb.c` changes, then rebuild
the kernel.

## Framebuffer geometry
2960×1848, ARGB8888, stride 11840 B/row. `kmsgfb` draws a vertical calibration
bar at the left edge: plumb = stride correct + text readable; slanted = read the
stride error from the slant.
