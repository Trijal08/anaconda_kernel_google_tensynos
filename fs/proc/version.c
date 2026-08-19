// SPDX-License-Identifier: GPL-2.0
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/utsname.h>
#include "internal.h"

#ifdef CONFIG_KSU_SUSFS_SPOOF_UNAME
#include <linux/jump_label.h>
#include <linux/string.h>
/*
 * Mist OS: /proc/version reads utsname() directly, so it leaks the REAL kernel
 * release/version even when uname(2) is spoofed (the newuname syscall in
 * kernel/sys.c only edits a per-call copy, never utsname()). Reuse the SAME
 * per-uid uname spoof here so one set_uname covers uname(2) AND /proc/version.
 * susfs_spoof_uname()/the static key are built into vmlinux (fs/susfs.c), so a
 * plain extern links (mirrors kernel/sys.c).
 */
extern struct static_key_false susfs_is_uname_spoof_buffer_set;
extern void susfs_spoof_uname(struct new_utsname *tmp);
/*
 * The build metadata (LINUX_COMPILE_BY/HOST/COMPILER) isn't part of the uname
 * struct, so it would still leak the custom build host and "-Anaconda" toolchain.
 * Substitute a Pixel-plausible Google build identity for a spoofed caller.
 */
#define SUSFS_PROC_VERSION_BUILDER  "kleaf@build-host"
#define SUSFS_PROC_VERSION_COMPILER \
	"Android (12833971, based on r536225) clang version 18.0.1 " \
	"(https://android.googlesource.com/toolchain/llvm-project " \
	"d8003a456d14a3deb8054cdaa529ffbf02d9b262), LLD 18.0.1"
#endif

static int version_proc_show(struct seq_file *m, void *v)
{
#ifdef CONFIG_KSU_SUSFS_SPOOF_UNAME
	if (static_branch_likely(&susfs_is_uname_spoof_buffer_set)) {
		struct new_utsname tmp;

		memcpy(&tmp, utsname(), sizeof(tmp));
		susfs_spoof_uname(&tmp);
		/*
		 * susfs_spoof_uname() leaves tmp == real when the caller's uid has
		 * no per-uid entry and no global override, so only rewrite the
		 * banner for a caller that is actually being spoofed.
		 */
		if (strcmp(tmp.release, utsname()->release) ||
		    strcmp(tmp.version, utsname()->version)) {
			seq_printf(m, "%s version %s (%s) (%s) %s\n",
				tmp.sysname, tmp.release,
				SUSFS_PROC_VERSION_BUILDER,
				SUSFS_PROC_VERSION_COMPILER,
				tmp.version);
			return 0;
		}
	}
#endif
	seq_printf(m, linux_proc_banner,
		utsname()->sysname,
		utsname()->release,
		utsname()->version);
	return 0;
}

static int __init proc_version_init(void)
{
	struct proc_dir_entry *pde;

	pde = proc_create_single("version", 0, NULL, version_proc_show);
	pde_make_permanent(pde);
	return 0;
}
fs_initcall(proc_version_init);
