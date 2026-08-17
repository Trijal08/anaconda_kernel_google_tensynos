#ifndef KSU_SUSFS_H
#define KSU_SUSFS_H

#include <linux/version.h>
#include <linux/types.h>
#include <linux/utsname.h>
#include <linux/hashtable.h>
#include <linux/path.h>
#include <linux/susfs_def.h>
#include <linux/statfs.h>

#define SUSFS_VERSION "v2.2.0"
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,0,0)
#define SUSFS_VARIANT "NON-GKI"
#else
#define SUSFS_VARIANT "GKI"
#endif

/*********/
/* MACRO */
/*********/
#define getname_safe(name) (name == NULL ? ERR_PTR(-EINVAL) : getname(name))
#define putname_safe(name) (IS_ERR(name) ? NULL : putname(name))

/********/
/* ENUM */
/********/
enum UID_SCHEME {
	UID_NON_APP_PROC = 0,
	UID_ROOT_PROC_EXCEPT_SU_PROC,
	UID_NON_SU_PROC,
	UID_UMOUNTED_APP_PROC,
	UID_UMOUNTED_PROC,
};

/**********/
/* STRUCT */
/**********/
/* generic per-uid companion entry, keyed by inode (ino+dev). Shared by the
 * inode-flag features (sus_path, sus_map) to add optional per-app targeting on
 * top of the global inode flag: a rule means "hide only for target_uid". */
struct st_susfs_ino_uid_hlist {
	unsigned long                           target_ino;
	unsigned long                           target_dev;
	int                                     target_uid;
	struct hlist_node                       node;
};

/* sus_path */
#ifdef CONFIG_KSU_SUSFS_SUS_PATH
struct st_susfs_sus_path {
	char                                    target_pathname[SUSFS_MAX_LEN_PATHNAME];
	int                                     err;
	/* per-app targeting, APPENDED AFTER err to keep the legacy ABI byte-stable.
	 * 0 = global (legacy: hide for every umounted app); >0 = hide only for
	 * current_uid().val == it. Populated only via the *_UID commands. */
	int                                     target_uid;
};

struct st_susfs_sus_path_list {
	struct list_head                        list;
	struct st_susfs_sus_path                info;
	char                                    target_pathname[SUSFS_MAX_LEN_PATHNAME];
};
#endif

/* sus_mount */
#ifdef CONFIG_KSU_SUSFS_SUS_MOUNT
struct st_susfs_hide_sus_mnts_for_non_su_procs {
	bool                                    enabled;
	int                                     err;
	/* per-app targeting, APPENDED AFTER err to keep the legacy ABI byte-stable.
	 * The *_UID command adds target_uid to a per-uid hide-set; the legacy 0/1
	 * command toggles the global state as before. */
	int                                     target_uid;
};

/* generic uid-only per-app entry (hide-set membership) */
struct st_susfs_uid_hlist {
	int                                     target_uid;
	struct hlist_node                       node;
};
#endif // #ifdef CONFIG_KSU_SUSFS_SUS_MOUNT

/* sus_kstat */
#ifdef CONFIG_KSU_SUSFS_SUS_KSTAT
#define KSTAT_SPOOF_INO (1 << 0)
#define KSTAT_SPOOF_DEV (1 << 1)
#define KSTAT_SPOOF_NLINK (1 << 2)
#define KSTAT_SPOOF_SIZE (1 << 3)
#define KSTAT_SPOOF_ATIME_TV_SEC (1 << 4)
#define KSTAT_SPOOF_ATIME_TV_NSEC (1 << 5)
#define KSTAT_SPOOF_MTIME_TV_SEC (1 << 6)
#define KSTAT_SPOOF_MTIME_TV_NSEC (1 << 7)
#define KSTAT_SPOOF_CTIME_TV_SEC (1 << 8)
#define KSTAT_SPOOF_CTIME_TV_NSEC (1 << 9)
#define KSTAT_SPOOF_BLOCKS (1 << 10)
#define KSTAT_SPOOF_BLKSIZE (1 << 11)

struct st_susfs_sus_kstat {
	int                                     is_statically;
	unsigned long                           target_ino;
	char                                    target_pathname[SUSFS_MAX_LEN_PATHNAME];
	unsigned long                           spoofed_ino;
	unsigned long                           spoofed_dev;
	unsigned int                            spoofed_nlink;
	long long                               spoofed_size;
	long                                    spoofed_atime_tv_sec;
	unsigned long                           spoofed_atime_tv_nsec;
	long                                    spoofed_mtime_tv_sec;
	unsigned long                           spoofed_mtime_tv_nsec;
	long                                    spoofed_ctime_tv_sec;
	unsigned long                           spoofed_ctime_tv_nsec;
	long long                               spoofed_blocks;
	long                                    spoofed_blksize;
	int                                     flags;
	int                                     err;
	/* per-app targeting, APPENDED AFTER err to keep the legacy ABI byte-stable.
	 * 0 = any uid (global, legacy); >0 = spoof only when current_uid().val == it.
	 * Populated only via CMD_SUSFS_ADD_SUS_KSTAT_STATICALLY_UID (full-size copy);
	 * legacy commands copy up to offsetof(target_uid), so old callers are unaffected. */
	int                                     target_uid;
};

struct st_susfs_sus_kstat_hlist {
	unsigned long                           target_ino;
	unsigned long                           target_dev;
	bool                                    is_fuse;
	struct st_susfs_sus_kstat               info;
	struct hlist_node                       node;
};

/* per-uid file-time offset: shift atime/mtime/ctime of every file the app OWNS
 * (inode owner uid == caller uid) by a fixed signed per-uid delta, so its whole
 * visible internal storage looks created at a different time — and unlike the
 * per-inode kstat spoof, it also covers files the app creates after boot. */
struct st_susfs_file_time_offset {
	int                                     target_uid;
	long                                    offset_sec;   /* signed: +future / -past */
	int                                     err;
};
struct st_susfs_file_time_offset_hlist {
	int                                     target_uid;
	long                                    offset_sec;
	struct hlist_node                       node;
};

/* per-uid /proc/uptime offset: shift the uptime a target app sees by a signed
 * per-uid delta (seconds), applied when /proc/uptime is generated. Kernel-side,
 * so it also catches raw openat()+read() (unlike the bionic uptime offset). */
struct st_susfs_uptime_offset {
	int                                     target_uid;
	long                                    offset_sec;
	int                                     err;
};
struct st_susfs_uptime_offset_hlist {
	int                                     target_uid;
	long                                    offset_sec;
	struct hlist_node                       node;
};
#endif

/* try_umount */
#ifdef CONFIG_KSU_SUSFS_TRY_UMOUNT
struct st_susfs_try_umount {
	char                                    target_pathname[SUSFS_MAX_LEN_PATHNAME];
	int                                     mnt_mode;
	int                                     err;
};

struct st_susfs_try_umount_list {
	struct list_head                        list;
	struct st_susfs_try_umount              info;
};
#endif

/* spoof_uname */
#ifdef CONFIG_KSU_SUSFS_SPOOF_UNAME
struct st_susfs_uname {
	char                                    release[__NEW_UTS_LEN+1];
	char                                    version[__NEW_UTS_LEN+1];
	int                                     err;
	/* per-app targeting, APPENDED AFTER err to keep the legacy ABI byte-stable.
	 * 0 = global (legacy: applies to every process); >0 = only current_uid().val == it.
	 * Populated only via CMD_SUSFS_SET_UNAME_UID (full-size copy); the legacy command
	 * copies up to offsetof(target_uid) and sets the global fallback as before. */
	int                                     target_uid;
};

#ifdef CONFIG_KSU_SUSFS_SPOOF_UNAME
/* per-uid uname override, keyed by target_uid */
struct st_susfs_uname_uid_hlist {
	int                                     target_uid;
	char                                    release[__NEW_UTS_LEN+1];
	char                                    version[__NEW_UTS_LEN+1];
	struct hlist_node                       node;
};
#endif
#endif

/* enable_log */
#ifdef CONFIG_KSU_SUSFS_ENABLE_LOG
struct st_susfs_log {
	bool                                    enabled;
	int                                     err;
};
#endif

/* spoof_cmdline_or_bootconfig */
#ifdef CONFIG_KSU_SUSFS_SPOOF_CMDLINE_OR_BOOTCONFIG
struct st_susfs_spoof_cmdline_or_bootconfig {
	char                                    fake_cmdline_or_bootconfig[SUSFS_FAKE_CMDLINE_OR_BOOTCONFIG_SIZE];
	int                                     err;
	/* per-app targeting, APPENDED AFTER err to keep the legacy ABI byte-stable.
	 * 0 = global (legacy); >0 = only current_uid().val == it. Populated only via
	 * CMD_SUSFS_SET_CMDLINE_OR_BOOTCONFIG_UID (full-size copy). */
	int                                     target_uid;
};

/* per-uid cmdline/bootconfig override, keyed by target_uid */
struct st_susfs_cmdline_uid_hlist {
	int                                     target_uid;
	char                                    *fake;
	struct hlist_node                       node;
};
#endif

/* open_redirect */
#ifdef CONFIG_KSU_SUSFS_OPEN_REDIRECT
struct st_susfs_open_redirect {
	char                                    target_pathname[SUSFS_MAX_LEN_PATHNAME];
	char                                    redirected_pathname[SUSFS_MAX_LEN_PATHNAME];
	int                                     uid_scheme;
	int                                     err;
	/* per-app targeting, APPENDED AFTER err to keep the legacy ABI byte-stable.
	 * 0 = any uid (global, legacy); >0 = redirect only when current_uid().val == it
	 * (evaluated in addition to uid_scheme). Populated only via
	 * CMD_SUSFS_ADD_OPEN_REDIRECT_UID; legacy command copies up to offsetof(target_uid). */
	int                                     target_uid;
};

struct st_susfs_open_redirect_hlist {
	unsigned long                           target_ino;
	unsigned long                           target_dev;
	unsigned long                           redirected_ino;
	unsigned long                           redirected_dev;
	int                                     spoofed_mnt_id;
	struct kstatfs                          spoofed_kstatfs;
	struct st_susfs_open_redirect           info;
	bool                                    reversed_lookup_only;
	struct hlist_node                       node;
};
#endif

/* sus_map */
#ifdef CONFIG_KSU_SUSFS_SUS_MAP
struct st_susfs_sus_map {
	char                                    target_pathname[SUSFS_MAX_LEN_PATHNAME];
	int                                     err;
	/* per-app targeting, APPENDED AFTER err to keep the legacy ABI byte-stable.
	 * 0 = global (legacy); >0 = hide only for current_uid().val == it. */
	int                                     target_uid;
};
#endif

/* avc log spoofing */
struct st_susfs_avc_log_spoofing {
	bool                                    enabled;
	int                                     err;
};

/* get enabled features */
struct st_susfs_enabled_features {
	char                                    enabled_features[SUSFS_ENABLED_FEATURES_SIZE];
	int                                     err;
};

/* show variant */
struct st_susfs_variant {
	char                                    susfs_variant[16];
	int                                     err;
};

/* show version */
struct st_susfs_version {
	char                                    susfs_version[16];
	int                                     err;
};

/***********************/
/* FORWARD DECLARATION */
/***********************/
/* sus_path */
#ifdef CONFIG_KSU_SUSFS_SUS_PATH
void susfs_add_sus_path(void __user **user_info, bool with_uid);
void susfs_add_sus_path_loop(void __user **user_info, bool with_uid);
#endif

/* sus_mount */
#ifdef CONFIG_KSU_SUSFS_SUS_MOUNT
void susfs_set_hide_sus_mnts_for_non_su_procs(void __user **user_info, bool with_uid);
bool susfs_sus_mount_hidden_for_current(void);
#endif // #ifdef CONFIG_KSU_SUSFS_SUS_MOUNT

/* sus_kstat */
#ifdef CONFIG_KSU_SUSFS_SUS_KSTAT
void susfs_add_sus_kstat(void __user **user_info, bool with_uid);
void susfs_update_sus_kstat(void __user **user_info);
void susfs_set_file_time_offset(void __user **user_info);
void susfs_set_uptime_offset(void __user **user_info);
long susfs_uptime_offset_for_current(void);
#endif

/* try_umount */
#ifdef CONFIG_KSU_SUSFS_TRY_UMOUNT
void susfs_add_try_umount(void __user **user_info);
void susfs_try_umount(uid_t uid);
#endif // #ifdef CONFIG_KSU_SUSFS_TRY_UMOUNT

/* spoof_uname */
#ifdef CONFIG_KSU_SUSFS_SPOOF_UNAME
void susfs_set_uname(void __user **user_info, bool with_uid);
void susfs_spoof_uname(struct new_utsname* tmp);
#endif

/* enable_log */
#ifdef CONFIG_KSU_SUSFS_ENABLE_LOG
void susfs_enable_log(void __user **user_info);
#endif

/* spoof_cmdline_or_bootconfig */
#ifdef CONFIG_KSU_SUSFS_SPOOF_CMDLINE_OR_BOOTCONFIG
void susfs_set_cmdline_or_bootconfig(void __user **user_info, bool with_uid);
#endif

/* open_redirect */
#ifdef CONFIG_KSU_SUSFS_OPEN_REDIRECT
void susfs_add_open_redirect(void __user **user_info, bool with_uid);
#endif

/* sus_map */
#ifdef CONFIG_KSU_SUSFS_SUS_MAP
void susfs_add_sus_map(void __user **user_info, bool with_uid);
#endif

void susfs_set_avc_log_spoofing(void __user **user_info);

void susfs_get_enabled_features(void __user **user_info);
void susfs_show_variant(void __user **user_info);
void susfs_show_version(void __user **user_info);

void susfs_start_sdcard_monitor_fn(void);

/* susfs_init */
void susfs_init(void);

#endif
