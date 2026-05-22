// SPDX-License-Identifier: GPL-2.0-only
/*
 * Based on arch/arm/kernel/setup.c
 *
 * Copyright (C) 1995-2001 Russell King
 * Copyright (C) 2012 ARM Ltd.
 */

#include <linux/acpi.h>
#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/stddef.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/initrd.h>
#include <linux/console.h>
#include <linux/cache.h>
#include <linux/screen_info.h>
#include <linux/init.h>
#include <linux/kexec.h>
#include <linux/root_dev.h>
#include <linux/cpu.h>
#include <linux/interrupt.h>
#include <linux/smp.h>
#include <linux/fs.h>
#include <linux/panic_notifier.h>
#include <linux/proc_fs.h>
#include <linux/memblock.h>
#include <linux/of_fdt.h>
#include <linux/efi.h>
#include <linux/psci.h>
#include <linux/sched/task.h>
#include <linux/scs.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/fbdbg.h>

#include <asm/acpi.h>
#include <asm/fixmap.h>
#include <asm/cpu.h>
#include <asm/cputype.h>
#include <asm/daifflags.h>
#include <asm/elf.h>
#include <asm/cpufeature.h>
#include <asm/cpu_ops.h>
#include <asm/kasan.h>
#include <asm/numa.h>
#include <asm/rsi.h>
#include <asm/scs.h>
#include <asm/sections.h>
#include <asm/setup.h>
#include <asm/smp_plat.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>
#include <asm/traps.h>
#include <asm/efi.h>
#include <asm/xen/hypervisor.h>
#include <asm/mmu_context.h>

static int num_standard_resources;
static struct resource *standard_resources;

phys_addr_t __fdt_pointer __initdata;
u64 mmu_enabled_at_boot __initdata;

/*
 * Standard memory resources
 */
static struct resource mem_res[] = {
	{
		.name = "Kernel code",
		.start = 0,
		.end = 0,
		.flags = IORESOURCE_SYSTEM_RAM
	},
	{
		.name = "Kernel data",
		.start = 0,
		.end = 0,
		.flags = IORESOURCE_SYSTEM_RAM
	}
};

#define kernel_code mem_res[0]
#define kernel_data mem_res[1]

/*
 * The recorded values of x0 .. x3 upon kernel entry.
 */
u64 __cacheline_aligned boot_args[4];

void __init smp_setup_processor_id(void)
{
	u64 mpidr = read_cpuid_mpidr() & MPIDR_HWID_BITMASK;
	set_cpu_logical_map(0, mpidr);

	pr_info("Booting Linux on physical CPU 0x%010lx [0x%08x]\n",
		(unsigned long)mpidr, read_cpuid_id());
}

bool arch_match_cpu_phys_id(int cpu, u64 phys_id)
{
	return phys_id == cpu_logical_map(cpu);
}

struct mpidr_hash mpidr_hash;
/**
 * smp_build_mpidr_hash - Pre-compute shifts required at each affinity
 *			  level in order to build a linear index from an
 *			  MPIDR value. Resulting algorithm is a collision
 *			  free hash carried out through shifting and ORing
 */
static void __init smp_build_mpidr_hash(void)
{
	u32 i, affinity, fs[4], bits[4], ls;
	u64 mask = 0;
	/*
	 * Pre-scan the list of MPIDRS and filter out bits that do
	 * not contribute to affinity levels, ie they never toggle.
	 */
	for_each_possible_cpu(i)
		mask |= (cpu_logical_map(i) ^ cpu_logical_map(0));
	pr_debug("mask of set bits %#llx\n", mask);
	/*
	 * Find and stash the last and first bit set at all affinity levels to
	 * check how many bits are required to represent them.
	 */
	for (i = 0; i < 4; i++) {
		affinity = MPIDR_AFFINITY_LEVEL(mask, i);
		/*
		 * Find the MSB bit and LSB bits position
		 * to determine how many bits are required
		 * to express the affinity level.
		 */
		ls = fls(affinity);
		fs[i] = affinity ? ffs(affinity) - 1 : 0;
		bits[i] = ls - fs[i];
	}
	/*
	 * An index can be created from the MPIDR_EL1 by isolating the
	 * significant bits at each affinity level and by shifting
	 * them in order to compress the 32 bits values space to a
	 * compressed set of values. This is equivalent to hashing
	 * the MPIDR_EL1 through shifting and ORing. It is a collision free
	 * hash though not minimal since some levels might contain a number
	 * of CPUs that is not an exact power of 2 and their bit
	 * representation might contain holes, eg MPIDR_EL1[7:0] = {0x2, 0x80}.
	 */
	mpidr_hash.shift_aff[0] = MPIDR_LEVEL_SHIFT(0) + fs[0];
	mpidr_hash.shift_aff[1] = MPIDR_LEVEL_SHIFT(1) + fs[1] - bits[0];
	mpidr_hash.shift_aff[2] = MPIDR_LEVEL_SHIFT(2) + fs[2] -
						(bits[1] + bits[0]);
	mpidr_hash.shift_aff[3] = MPIDR_LEVEL_SHIFT(3) +
				  fs[3] - (bits[2] + bits[1] + bits[0]);
	mpidr_hash.mask = mask;
	mpidr_hash.bits = bits[3] + bits[2] + bits[1] + bits[0];
	pr_debug("MPIDR hash: aff0[%u] aff1[%u] aff2[%u] aff3[%u] mask[%#llx] bits[%u]\n",
		mpidr_hash.shift_aff[0],
		mpidr_hash.shift_aff[1],
		mpidr_hash.shift_aff[2],
		mpidr_hash.shift_aff[3],
		mpidr_hash.mask,
		mpidr_hash.bits);
	/*
	 * 4x is an arbitrary value used to warn on a hash table much bigger
	 * than expected on most systems.
	 */
	if (mpidr_hash_size() > 4 * num_possible_cpus())
		pr_warn("Large number of MPIDR hash buckets detected\n");
}

static void __init setup_machine_fdt(phys_addr_t dt_phys)
{
	int size = 0;
	void *dt_virt = fixmap_remap_fdt(dt_phys, &size, PAGE_KERNEL);
	const char *name;

	if (dt_virt)
		memblock_reserve(dt_phys, size);

	/*
	 * dt_virt is a fixmap address, hence __pa(dt_virt) can't be used.
	 * Pass dt_phys directly.
	 */
	if (!early_init_dt_scan(dt_virt, dt_phys)) {
		pr_crit("\n"
			"Error: invalid device tree blob: PA=%pa, VA=%px, size=%d bytes\n"
			"The dtb must be 8-byte aligned and must not exceed 2 MB in size.\n"
			"\nPlease check your bootloader.\n",
			&dt_phys, dt_virt, size);

		/*
		 * Note that in this _really_ early stage we cannot even BUG()
		 * or oops, so the least terrible thing to do is cpu_relax(),
		 * or else we could end-up printing non-initialized data, etc.
		 */
		while (true)
			cpu_relax();
	}

	/* Early fixups are done, map the FDT as read-only now */
	fixmap_remap_fdt(dt_phys, &size, PAGE_KERNEL_RO);

	name = of_flat_dt_get_machine_name();
	if (!name)
		return;

	pr_info("Machine model: %s\n", name);
	dump_stack_set_arch_desc("%s (DT)", name);
}

static void __init request_standard_resources(void)
{
	struct memblock_region *region;
	struct resource *res;
	unsigned long i = 0;
	size_t res_size;

	kernel_code.start   = __pa_symbol(_text);
	kernel_code.end     = __pa_symbol(__init_begin - 1);
	kernel_data.start   = __pa_symbol(_sdata);
	kernel_data.end     = __pa_symbol(_end - 1);
	insert_resource(&iomem_resource, &kernel_code);
	insert_resource(&iomem_resource, &kernel_data);

	num_standard_resources = memblock.memory.cnt;
	res_size = num_standard_resources * sizeof(*standard_resources);
	standard_resources = memblock_alloc_or_panic(res_size, SMP_CACHE_BYTES);

	for_each_mem_region(region) {
		res = &standard_resources[i++];
		if (memblock_is_nomap(region)) {
			res->name  = "reserved";
			res->flags = IORESOURCE_MEM;
			res->start = __pfn_to_phys(memblock_region_reserved_base_pfn(region));
			res->end = __pfn_to_phys(memblock_region_reserved_end_pfn(region)) - 1;
		} else {
			res->name  = "System RAM";
			res->flags = IORESOURCE_SYSTEM_RAM | IORESOURCE_BUSY;
			res->start = __pfn_to_phys(memblock_region_memory_base_pfn(region));
			res->end = __pfn_to_phys(memblock_region_memory_end_pfn(region)) - 1;
		}

		insert_resource(&iomem_resource, res);
	}
}

static int __init reserve_memblock_reserved_regions(void)
{
	u64 i, j;

	for (i = 0; i < num_standard_resources; ++i) {
		struct resource *mem = &standard_resources[i];
		phys_addr_t r_start, r_end, mem_size = resource_size(mem);

		if (!memblock_is_region_reserved(mem->start, mem_size))
			continue;

		for_each_reserved_mem_range(j, &r_start, &r_end) {
			resource_size_t start, end;

			start = max(PFN_PHYS(PFN_DOWN(r_start)), mem->start);
			end = min(PFN_PHYS(PFN_UP(r_end)) - 1, mem->end);

			if (start > mem->end || end < mem->start)
				continue;

			reserve_region_with_split(mem, start, end, "reserved");
		}
	}

	return 0;
}
arch_initcall(reserve_memblock_reserved_regions);

u64 __cpu_logical_map[NR_CPUS] = { [0 ... NR_CPUS-1] = INVALID_HWID };

u64 cpu_logical_map(unsigned int cpu)
{
	return __cpu_logical_map[cpu];
}

/*
 * dense-ladder raw-FB breadcrumb. Same mechanism
 * as init/main.c's FBDBG_STRIPE_RAW: a raw str to the splash FB via the still-
 * live init idmap (TTBR0). Valid only up to cpu_uninstall_idmap() below (line
 * ~320). Inline asm bypasses instrumentation; clobbers x9/x10/x11.
 */
#define FBDBG_STRIPE_RAW(off, color)					\
	asm volatile(							\
		"mov	x9, %0\n"					\
		"mov	w10, %w1\n"					\
		"mov	w11, #16384\n"					\
		"1:	str	w10, [x9], #4\n"			\
		"	subs	w11, w11, #1\n"				\
		"	b.ne	1b\n"					\
		"	dsb	sy\n"					\
		:: "r"(0xb8000000UL + (off)), "r"((u32)(color))		\
		: "x9", "x10", "x11", "memory")

/*
 * framebuffer breadcrumb for setup_arch's BACK HALF
 * Past cpu_uninstall_idmap() (~line 350) the TTBR0 idmap is gone, so the
 * FBDBG_STRIPE_RAW raw str no longer reaches the splash FB. This maps the FB
 * through the TTBR1 fixmap via early_memremap_prot() with PROT_NORMAL_NC -- the
 * SAME memory attribute the proven M0-M13 idmap writes used (arm64 ioremap_wc
 * == PROT_NORMAL_NC). Valid throughout setup_arch (early_ioremap_init() ran;
 * early_ioremap_reset() is later, after our last checkpoint). Full ioremap_wc
 * is NOT usable here -- it needs vmalloc_init(), which runs in mm_core_init()
 * AFTER setup_arch returns. Paints a distinct-offset 64 KB stripe so the
 * deepest lit stripe localizes the death.
 */
/* Non-static + __ref so the fault/panic hooks (fault.c/traps.c/panic.c) can
 * call it; they run during boot, before init memory is freed, so calling the
 * __init early_memremap_prot from non-__init context is intentional. */
void __ref fbdbg_stripe(unsigned long off, u32 color)
{
	u32 *fb = early_memremap_prot(0xb8000000UL + off, 0x10000,
				      PROT_NORMAL_NC);
	unsigned int i;

	if (!fb)
		return;
	for (i = 0; i < 0x10000 / sizeof(u32); i++)
		fb[i] = color;
	dsb(sy);
	early_memunmap(fb, 0x10000);
}

static atomic_t fbdbg_in_emit = ATOMIC_INIT(0);  /* recursion guard */
static int fbdbg_emitted;                         /* first-emitter latch */

/* octal codec: 8 M/E-proven colors -> octal digits 0..7 (white is
 * reserved for the leader). Wide pitch 0x40000 (the codec's 0x20000
 * bloom-merged under the camera; 0x40000 decoded cleanly all session). */
static const u32 fbdbg_oct[8] = {
	0xffff00ff, /*0 magenta*/ 0xff00ffff, /*1 cyan  */ 0xffffff00, /*2 yellow*/
	0xff00ff00, /*3 green  */ 0xffff0000, /*4 red   */ 0xffff8000, /*5 orange*/
	0xff8000ff, /*6 violet */ 0xff0000ff, /*7 blue  */
};

/* paint `n` octal digits of `val`, MSB-first, at base + k*0x40000. */
void __ref fbdbg_emit_octal(unsigned long base, u64 val, int n)
{
	int k;

	for (k = 0; k < n; k++)
		fbdbg_stripe(base + (unsigned long)k * 0x40000UL,
				 fbdbg_oct[(val >> (3 * (n - 1 - k))) & 0x7]);
}

/* fault: white leader @0x040000 + 9 octal digits of the faulting PC's LINK
 * address (elr - kaslr_offset(), low 27 bits; text caps ~36 MiB above _text)
 * @0x080000. Latched (first emitter wins); recursion-guarded. Ordering
 * invariant (do NOT reorder): set the `emitted` latch only AFTER the full
 * paint; a fault re-entering mid-paint sees emitted==0 but in_emit==1 and
 * skips via the guard. Set latch THEN clear guard. */
void __ref fbdbg_emit_fault(unsigned long elr)
{
	if (READ_ONCE(fbdbg_emitted))
		return;
	if (atomic_xchg(&fbdbg_in_emit, 1))
		return;
	fbdbg_stripe(0x040000, 0xffffffff);                  /* white = fault */
	fbdbg_emit_octal(0x080000,
			  (elr - kaslr_offset()) & 0x7ffffffUL, 9);
	WRITE_ONCE(fbdbg_emitted, 1);
	atomic_set(&fbdbg_in_emit, 0);
}

/* panic / NULL-regs oops: red leader @0x040000 (coarse retaddr, NOT a precise
 * PC) + the same 9-digit field. Same slots; the latch picks one. */
void __ref fbdbg_emit_panic(unsigned long retaddr)
{
	if (READ_ONCE(fbdbg_emitted))
		return;
	if (atomic_xchg(&fbdbg_in_emit, 1))
		return;
	fbdbg_stripe(0x040000, 0xffff0000);                  /* red = panic */
	fbdbg_emit_octal(0x080000,
			  (retaddr - kaslr_offset()) & 0x7ffffffUL, 9);
	WRITE_ONCE(fbdbg_emitted, 1);
	atomic_set(&fbdbg_in_emit, 0);
}

/* WARN site. Yellow leader @0x040000 + 9 octal digits of the
 * WARN caller's LINK address. Shares the latch (first emitter wins) so the
 * FIRST WARN's site is captured; with panic_on_warn dropped the kernel then
 * continues and the deeper bracket marks show how far it gets. */
void __ref fbdbg_emit_warn(unsigned long caller)
{
	if (READ_ONCE(fbdbg_emitted))
		return;
	if (atomic_xchg(&fbdbg_in_emit, 1))
		return;
	fbdbg_stripe(0x040000, 0xffffff00);                  /* yellow = WARN */
	fbdbg_emit_octal(0x080000,
			  (caller - kaslr_offset()) & 0x7ffffffUL, 9);
	WRITE_ONCE(fbdbg_emitted, 1);
	atomic_set(&fbdbg_in_emit, 0);
}

void __init __no_sanitize_address setup_arch(char **cmdline_p)
{
	/* M11 green @ FB+0x2c0000: entered setup_arch. */
	FBDBG_STRIPE_RAW(0x2c0000, 0xff00ff00);

	setup_initial_init_mm(_text, _etext, _edata, _end);

	*cmdline_p = boot_command_line;

	kaslr_init();

	early_fixmap_init();
	early_ioremap_init();

	setup_machine_fdt(__fdt_pointer);

	/* M12 magenta @ FB+0x300000: device tree parsed (setup_machine_fdt
	 * returned) -- the prime-suspect checkpoint for a board-data death. */
	FBDBG_STRIPE_RAW(0x300000, 0xffff00ff);

	/*
	 * Initialise the static keys early as they may be enabled by the
	 * cpufeature code and early parameters.
	 */
	jump_label_init();
	parse_early_param();

	dynamic_scs_init();

	/*
	 * The primary CPU enters the kernel with all DAIF exceptions masked.
	 *
	 * We must unmask Debug and SError before preemption or scheduling is
	 * possible to ensure that these are consistently unmasked across
	 * threads, and we want to unmask SError as soon as possible after
	 * initializing earlycon so that we can report any SErrors immediately.
	 *
	 * IRQ and FIQ will be unmasked after the root irqchip has been
	 * detected and initialized.
	 */
	local_daif_restore(DAIF_PROCCTX_NOIRQ);

	/* M13 violet @ FB+0x340000: setup_arch reached idmap-teardown --
	 * the LAST raw-FB-reachable point (cpu_uninstall_idmap below drops the idmap;
	 * past here only ioremap_wc reaches the FB, e.g. paint_splash blue@1065). */
	FBDBG_STRIPE_RAW(0x340000, 0xff8000ff);

	/*
	 * TTBR0 is only used for the identity mapping at this stage. Make it
	 * point to zero page to avoid speculatively fetching new entries.
	 */
	cpu_uninstall_idmap();

	/* E0 white @ FB+0x380000: idmap teardown survived. FIRST
	 * early_memremap stripe -- also the mechanism check: if M13 (raw idmap)
	 * lit but E0 dark, suspect the fixmap-FB write mechanism, NOT a device
	 * death (3rd Lesson #53 caveat: Normal-NC FB write via the TTBR1 fixmap
	 * is unobserved on this device; same PA + same attribute as M0-M13). */
	fbdbg_stripe(0x380000, 0xffffffff);

	xen_early_init();
	efi_init();

	if (!efi_enabled(EFI_BOOT)) {
		if ((u64)_text % MIN_KIMG_ALIGN)
			pr_warn(FW_BUG "Kernel image misaligned at boot, please fix your bootloader!");
		WARN_TAINT(mmu_enabled_at_boot, TAINT_FIRMWARE_WORKAROUND,
			   FW_BUG "Booted with MMU enabled!");
	}

	/* E1 orange @ FB+0x3c0000: xen_early_init + efi_init done. */
	fbdbg_stripe(0x3c0000, 0xffff8000);

	arm64_memblock_init();

	/* E2 red @ FB+0x400000: arm64_memblock_init done. */
	fbdbg_stripe(0x400000, 0xffff0000);

	paging_init();

	/* E3 yellow @ FB+0x440000: paging_init done [PRIME suspect]. */
	fbdbg_stripe(0x440000, 0xffffff00);

	acpi_table_upgrade();

	/* Parse the ACPI tables for possible boot-time configuration */
	acpi_boot_table_init();

	if (acpi_disabled)
		unflatten_device_tree();

	/* E4 magenta @ FB+0x480000: acpi_*_table + (if acpi_disabled)
	 * unflatten_device_tree done [PRIME suspect]. CONDITIONAL: E4 lit does
	 * NOT prove unflatten ran unless acpi_disabled (Qualcomm DT boot ⇒
	 * acpi_disabled likely true; verify cmdline if E3 lit + E4 dark). */
	fbdbg_stripe(0x480000, 0xffff00ff);

	bootmem_init();

	/* E5 cyan @ FB+0x4c0000: bootmem_init done. */
	fbdbg_stripe(0x4c0000, 0xff00ffff);

	kasan_init();

	/* E6 green @ FB+0x500000: kasan_init done. */
	fbdbg_stripe(0x500000, 0xff00ff00);

	request_standard_resources();

	/* E7 violet @ FB+0x540000: request_standard_resources done --
	 * the deepest stripe before early_ioremap_reset() drops early_ioremap. */
	fbdbg_stripe(0x540000, 0xff8000ff);

	early_ioremap_reset();

	if (acpi_disabled)
		psci_dt_init();
	else
		psci_acpi_init();

	arm64_rsi_init();

	init_bootcpu_ops();
	smp_init_cpus();
	smp_build_mpidr_hash();

#ifdef CONFIG_ARM64_SW_TTBR0_PAN
	/*
	 * Make sure init_thread_info.ttbr0 always generates translation
	 * faults in case uaccess_enable() is inadvertently called by the init
	 * thread.
	 */
	init_task.thread_info.ttbr0 = phys_to_ttbr(__pa_symbol(reserved_pg_dir));
#endif

	if (boot_args[1] || boot_args[2] || boot_args[3]) {
		pr_err("WARNING: x1-x3 nonzero in violation of boot protocol:\n"
			"\tx1: %016llx\n\tx2: %016llx\n\tx3: %016llx\n"
			"This indicates a broken bootloader or old kernel\n",
			boot_args[1], boot_args[2], boot_args[3]);
	}
}

static inline bool cpu_can_disable(unsigned int cpu)
{
#ifdef CONFIG_HOTPLUG_CPU
	const struct cpu_operations *ops = get_cpu_ops(cpu);

	if (ops && ops->cpu_can_disable)
		return ops->cpu_can_disable(cpu);
#endif
	return false;
}

bool arch_cpu_is_hotpluggable(int num)
{
	return cpu_can_disable(num);
}

static void dump_kernel_offset(void)
{
	const unsigned long offset = kaslr_offset();

	if (IS_ENABLED(CONFIG_RANDOMIZE_BASE) && offset > 0) {
		pr_emerg("Kernel Offset: 0x%lx from 0x%lx\n",
			 offset, KIMAGE_VADDR);
		pr_emerg("PHYS_OFFSET: 0x%llx\n", PHYS_OFFSET);
	} else {
		pr_emerg("Kernel Offset: disabled\n");
	}
}

static int arm64_panic_block_dump(struct notifier_block *self,
				  unsigned long v, void *p)
{
	dump_kernel_offset();
	dump_cpu_features();
	dump_mem_limit();
	return 0;
}

static struct notifier_block arm64_panic_block = {
	.notifier_call = arm64_panic_block_dump
};

static int __init register_arm64_panic_block(void)
{
	atomic_notifier_chain_register(&panic_notifier_list,
				       &arm64_panic_block);
	return 0;
}
device_initcall(register_arm64_panic_block);

static int __init check_mmu_enabled_at_boot(void)
{
	if (!efi_enabled(EFI_BOOT) && mmu_enabled_at_boot)
		panic("Non-EFI boot detected with MMU and caches enabled");
	return 0;
}
device_initcall_sync(check_mmu_enabled_at_boot);
