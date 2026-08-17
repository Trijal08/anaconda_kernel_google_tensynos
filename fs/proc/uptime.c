// SPDX-License-Identifier: GPL-2.0
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/time.h>
#include <linux/time_namespace.h>
#include <linux/kernel_stat.h>
#include "internal.h"

#ifdef CONFIG_KSU_SUSFS_SUS_KSTAT
extern long susfs_uptime_offset_for_current(void);
#endif

static int uptime_proc_show(struct seq_file *m, void *v)
{
	struct timespec64 uptime;
	struct timespec64 idle;
	u64 idle_nsec;
	u32 rem;
	int i;

	idle_nsec = 0;
	for_each_possible_cpu(i) {
		struct kernel_cpustat kcs;

		kcpustat_cpu_fetch(&kcs, i);
		idle_nsec += get_idle_time(&kcs, i);
	}

	ktime_get_boottime_ts64(&uptime);
	timens_add_boottime(&uptime);

#ifdef CONFIG_KSU_SUSFS_SUS_KSTAT
	/* per-uid /proc/uptime spoof: shift the uptime a target app sees. Kernel-side,
	 * so it catches libc fopen, raw openat()+read() and `cat` alike. */
	{
		long __uoff = susfs_uptime_offset_for_current();
		if (__uoff) {
			uptime.tv_sec += __uoff;
			if (uptime.tv_sec < 0) {
				uptime.tv_sec = 0;
				uptime.tv_nsec = 0;
			}
		}
	}
#endif

	idle.tv_sec = div_u64_rem(idle_nsec, NSEC_PER_SEC, &rem);
	idle.tv_nsec = rem;
	seq_printf(m, "%lu.%02lu %lu.%02lu\n",
			(unsigned long) uptime.tv_sec,
			(uptime.tv_nsec / (NSEC_PER_SEC / 100)),
			(unsigned long) idle.tv_sec,
			(idle.tv_nsec / (NSEC_PER_SEC / 100)));
	return 0;
}

static int __init proc_uptime_init(void)
{
	struct proc_dir_entry *pde;

	pde = proc_create_single("uptime", 0, NULL, uptime_proc_show);
	pde_make_permanent(pde);
	return 0;
}
fs_initcall(proc_uptime_init);
