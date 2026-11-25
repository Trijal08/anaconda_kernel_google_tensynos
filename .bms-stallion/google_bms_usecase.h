/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2024 Google, LLC
 *
 */

#ifndef GOOGLE_BMS_USECASE_H_
#define GOOGLE_BMS_USECASE_H_

#include <linux/device.h>
#include <linux/klist.h>
#include <linux/mutex.h>
#include <linux/semaphore.h>
#include <linux/workqueue.h>

#include "misc/gvotable.h"

#define BMS_USECASE_MAX_ENTRIES 20
#define BMS_USECASE_INTERMEDIATE_STR "Intermediate"

struct gsu_usecase_config_t {
	int usecase;
	int val;
	const char *name;
	/* Add new fields after this */
	bool is_wireless;
	bool is_wired;
	bool is_otg;
	bool is_charging;
	bool is_cp;
	int chg_index; /* this is just used to determine if a charger has changed */

	/* do not add after */
	struct hlist_node hnode;
};

enum bms_usecase_charger_index {
	BMS_USECASE_CHARGER_INDEX_INVALID,
	BMS_USECASE_CHARGER_INDEX_MAIN,
	BMS_USECASE_CHARGER_INDEX_CP,
	BMS_USECASE_CHARGER_INDEX_HYBRID,
};

/*
 * Usecase Allocations
 *  -2: BMS_USECASE_NO_HOPS (reserved for usecase hop function)
 *  -1: GSU_RAW_MODE
 *   0: GSU_MODE_STANDBY
 *   1-199: gs101/gs201 Standard Usecases
 * 200-299: USB Charge Extended Usecases
 * 300-399: WLC Extended Usecases
 */
#define FOREACH_GSU_USECASE(S)	\
	S(BMS_USECASE_NO_HOPS, -2, "NO_HOPS", .is_wireless = false, .is_wired = false,		\
	 .is_otg = false, .is_charging = false, .is_cp = false,					\
	 .chg_index = BMS_USECASE_CHARGER_INDEX_INVALID),					\
	/* raw mode, default, */								\
	S(GSU_RAW_MODE, -1, "RAW", .is_wireless = false, .is_wired = false, .is_otg = false,	\
	 .is_charging = false, .is_cp = false, .chg_index = BMS_USECASE_CHARGER_INDEX_INVALID),	\
												\
	S(GSU_MODE_STANDBY, 0, "Standby", .is_wireless = false, .is_wired = false,		\
	 .is_otg = false, .is_charging = false, .is_cp = false,					\
	 .chg_index = BMS_USECASE_CHARGER_INDEX_INVALID),					\
	/* 1-1 wired mode 0x4 */								\
	S(GSU_MODE_USB_CHG, 1, "USB", .is_wireless = false, .is_wired = true, .is_otg = false,	\
	 .is_charging = false, .is_cp = false, .chg_index = BMS_USECASE_CHARGER_INDEX_MAIN),	\
	/* 1-1 wired mode 0x5 */								\
	S(GSU_MODE_USB_CHG_CHARGE_ENABLED, 201, "USB_CHG", .is_wireless = false,		\
	 .is_wired = true, .is_otg = false, .is_charging = true, .is_cp = false,		\
	 .chg_index = BMS_USECASE_CHARGER_INDEX_MAIN),						\
	/* 1-2 wired mode 0x0/0x1 */								\
	S(GSU_MODE_USB_DC, 2, "USB_DC", .is_wireless = false, .is_wired = true, .is_otg = false,\
	 .is_charging = true, .is_cp = true, .chg_index = BMS_USECASE_CHARGER_INDEX_CP),	\
												\
	S(GSU_MODE_USB_CHG_HYBRID, 221, "USB_CHG_HYBRID", .is_wireless = false,			\
	 .is_wired = true, .is_otg = false, .is_charging = true, .is_cp = false,		\
	 .chg_index = BMS_USECASE_CHARGER_INDEX_HYBRID),					\
	/* 2-1, 1041, */									\
	S(GSU_MODE_USB_CHG_WLC_TX, 3, "USB_CHG_RTX", .is_wireless = false, .is_wired = true,	\
	 .is_otg = false, .is_charging = true, .is_cp = false,					\
	 .chg_index = BMS_USECASE_CHARGER_INDEX_MAIN),						\
												\
	/* 3-1, mode 0x4 */									\
	S(GSU_MODE_WLC_RX, 5, "WLC_RX", .is_wireless = true, .is_wired = false, .is_otg = false,\
	 .is_charging = false, .is_cp = false, .chg_index = BMS_USECASE_CHARGER_INDEX_MAIN),	\
	/* 3-1, mode 0x5 */									\
	S(GSU_MODE_WLC_RX_CHARGE_ENABLED, 305, "WLC_RX_CHG", .is_wireless = true,		\
	 .is_wired = false, .is_otg = false, .is_charging = true, .is_cp = false,		\
	 .chg_index = BMS_USECASE_CHARGER_INDEX_MAIN),						\
	/* WLC spoofed */									\
	S(GSU_MODE_WLC_RX_SPOOFED, 300,	"WLC_RX_SPOOF", .is_wireless = true, .is_wired = false,	\
	 .is_otg = false, .is_charging = false, .is_cp = false,					\
	 .chg_index = BMS_USECASE_CHARGER_INDEX_INVALID),					\
												\
	S(GSU_MODE_WLC_RX_HYBRID, 325, "WLC_RX_HYBRID", .is_wireless = true, .is_wired = false,	\
	 .is_otg = false, .is_charging = true, .is_cp = false,					\
	 .chg_index = BMS_USECASE_CHARGER_INDEX_HYBRID),					\
	/* 3-2, mode 0x0 */									\
	S(GSU_MODE_WLC_DC, 6, "WLC_DC", .is_wireless = true, .is_wired = false, .is_otg = false,\
	 .is_charging = true, .is_cp = true, .chg_index = BMS_USECASE_CHARGER_INDEX_CP),	\
	S(GSU_MODE_USB_OTG_WLC_DC, 306, "OTG_WLC_DC", .is_wireless = true, .is_wired = false,	\
	 .is_otg = true, .is_charging = true, .is_cp = true,					\
	 .chg_index = BMS_USECASE_CHARGER_INDEX_CP),						\
												\
	/* 7, 524, */										\
	S(GSU_MODE_USB_OTG_WLC_RX, 7, "OTG_WLC_RX", .is_wireless = true, .is_wired = false,	\
	 .is_otg = true, .is_charging = false, .is_cp = false,					\
	 .chg_index = BMS_USECASE_CHARGER_INDEX_INVALID),					\
	S(GSU_MODE_USB_OTG_WLC_RX_CHARGE_ENABLED, 307, "OTG_WLC_RX_CHG",			\
	  .is_wireless = true, .is_wired = false, .is_otg = true, .is_charging = true,		\
	  .is_cp = false, .chg_index = BMS_USECASE_CHARGER_INDEX_MAIN),				\
												\
	S(GSU_MODE_USB_OTG_WLC_RX_HYBRID, 329, "OTG_WLC_HYBRID", .is_wireless = true,		\
	 .is_wired = false, .is_otg = true, .is_charging = true, .is_cp = false,		\
	 .chg_index = BMS_USECASE_CHARGER_INDEX_HYBRID),					\
	/* 5-1, 516,*/										\
	S(GSU_MODE_USB_OTG, 9, "OTG", .is_wireless = false, .is_wired = false, .is_otg = true,	\
	 .is_charging = false, .is_cp = false, .chg_index = BMS_USECASE_CHARGER_INDEX_INVALID),	\
	S(GSU_MODE_USB_OTG_FRS, 10, "OTG_FRS", .is_wireless = false, .is_wired = false,		\
	 .is_otg = true, .is_charging = false, .is_cp = false,					\
	 .chg_index = BMS_USECASE_CHARGER_INDEX_INVALID),					\
	/* 6-2, 1056, */									\
	S(GSU_MODE_WLC_TX, 11, "RTX", .is_wireless = false, .is_wired = false, .is_otg = false,	\
	 .is_charging =  false, .is_cp = false, .chg_index = BMS_USECASE_CHARGER_INDEX_INVALID),\
	S(GSU_MODE_USB_OTG_WLC_TX, 12, "OTG_RTX", .is_wireless = false, .is_wired = false,	\
	 .is_otg = true, .is_charging = false, .is_cp = false,					\
	 .chg_index = BMS_USECASE_CHARGER_INDEX_INVALID),					\
	S(GSU_MODE_USB_WLC_RX, 13, "USB_WLC_RX_CHG", .is_wireless = true, .is_wired = false,	\
	 .is_otg = false, .is_charging = true, .is_cp = false,					\
	 .chg_index = BMS_USECASE_CHARGER_INDEX_MAIN),						\
												\
	S(GSU_MODE_DOCK, 14, "DOCK", .is_wireless = false, .is_wired = false, .is_otg = false,	\
	 .is_charging = false, .is_cp = false, .chg_index = BMS_USECASE_CHARGER_INDEX_INVALID),	\
	S(GSU_MODE_DOCK_CHARGE_ENABLED, 314, "DOCK_CHG", .is_wireless = false,			\
	 .is_wired = false, .is_otg = false, .is_charging = true, .is_cp = false,		\
	 .chg_index = BMS_USECASE_CHARGER_INDEX_MAIN),						\
												\
	/* check max77779_wcin_is_valid if modifying pogo vout */				\
	S(GSU_MODE_POGO_VOUT, 15, "POGO_VOUT", .is_wireless = false, .is_wired = false,		\
	 .is_otg = false, .is_charging = false, .is_cp = false,					\
	 .chg_index = BMS_USECASE_CHARGER_INDEX_INVALID),					\
	S(GSU_MODE_USB_CHG_POGO_VOUT, 16, "USB_CHG_POGO_VOUT", .is_wireless = false,		\
	 .is_wired = false, .is_otg = false, .is_charging = false, .is_cp = false,		\
	 .chg_index = BMS_USECASE_CHARGER_INDEX_INVALID),					\
	S(GSU_MODE_USB_CHG_POGO_VOUT_CHARGE_ENABLED, 216, "USB_CHG_POGO_VOUT",			\
	 .is_wireless = false, .is_wired = false, .is_otg = false, .is_charging = true,		\
	 .is_cp = false, .chg_index = BMS_USECASE_CHARGER_INDEX_MAIN),				\
	S(GSU_MODE_USB_OTG_POGO_VOUT, 17, "USB_OTG_POGO_VOUT", .is_wireless = false,		\
	 .is_wired = false, .is_otg = true, .is_charging = false, .is_cp = false,		\
	 .chg_index = BMS_USECASE_CHARGER_INDEX_INVALID),					\
												\
	/* ifpmic firmware update */								\
	S(GSU_MODE_FWUPDATE, 18, "IFPMIC_FWUPDATE", .is_wireless = false, .is_wired = false,	\
	 .is_otg = false, .is_charging = false, .is_cp = false,					\
	 .chg_index = BMS_USECASE_CHARGER_INDEX_INVALID),					\
	/* WLC fwupdate */									\
	S(GSU_MODE_WLC_FWUPDATE, 19, "WLC_FWUPDATE", .is_wireless = false, .is_wired = false,	\
	 .is_otg = false, .is_charging = false, .is_cp = false,					\
	 .chg_index = BMS_USECASE_CHARGER_INDEX_INVALID),					\
												\
	/* indicates input_suspend and plugged in */						\
	S(GSU_MODE_STANDBY_BUCK_ON, 100, "STANDBY_BUCK_ON", .is_wireless = false,		\
	 .is_wired = true, .is_otg = false, .is_charging = false, .is_cp = false,		\
	 .chg_index = BMS_USECASE_CHARGER_INDEX_INVALID),					\

#define GSU_USECASE_CONFIG(...) { __VA_ARGS__ }							\

#define GSU_USECASE_ENUM_CONFIG(usecase, val, ...)						\
	usecase = val										\

enum gsu_usecases {
	FOREACH_GSU_USECASE(GSU_USECASE_ENUM_CONFIG)
};

/*
 * BMS usecase mode callback value parsing
 * based on uint32_t size but can be extended to use uint64_t if needed
 */
#define BMS_USECASE_BFF(name, h, l) \
static inline uint32_t _ ## name ## _set(uint32_t r, uint8_t v) \
{ \
	return ((r & ~GENMASK(h, l)) | v << l); \
} \
\
static inline uint32_t _ ## name ## _get(uint32_t r) \
{ \
	return ((r & GENMASK(h, l)) >> l); \
}

#define BMS_USECASE_MODE_SHIFT	0
#define BMS_USECASE_MODE_MASK	(0xffff << 0)
#define BMS_USECASE_MODE_CLEAR	(~(0xffff << 0))
#define BMS_USECASE_META_SHIFT	16
/* All metadata */
#define BMS_USECASE_META_MASK	(0xffff << 16)
#define BMS_USECASE_META_CLEAR	(~(0xffff << 16))

#define BMS_USECASE_META_ASYNC_SHIFT	16
#define BMS_USECASE_META_ASYNC_MASK	(0x1 << 16)
#define BMS_USECASE_META_ASYNC_CLEAR	(~(0x1 << 16))

BMS_USECASE_BFF(bms_usecase_mode, 15, 0)
BMS_USECASE_BFF(bms_usecase_meta, 31, 16)
BMS_USECASE_BFF(bms_usecase_meta_async, 16, 16)

#define GSU_MODE_FWUPDATE_MASK (0x1 << 0)
#define GSU_MODE_WLC_FWUPDATE_MASK (0x1 << 1)

enum bms_usecase_status {
	BMS_USECASE_STATUS_UNINITIALIZED = 0,
	BMS_USECASE_STATUS_NEW,
	BMS_USECASE_STATUS_ORPHAN, /* New status for ownership transfer */
};

enum bms_usecase_state {
	BMS_USECASE_UNINITIALIZED = 0,
	BMS_USECASE_INITIAL,
	BMS_USECASE_INTERMEDIATE,
};

/*
 * mode_callback: async entries free all
 * mode_callback: nope cb free entry only
 * mode_callback: blocking entries remove from list only
 * post_election_work: blocking entries free entry only
 */
enum bms_usecase_free_state {
	BMS_USECASE_FREE_ALL = 0,
	BMS_USECASE_FREE_FROM_LIST,
	BMS_USECASE_FREE_ENTRY,
};

struct bms_usecase_chg_data {
	struct device *dev;
	void *uc_data;
	int (*hop_func)(void *uc_data, int from_uc, int to_uc);
	void (*populate_cb_data)(void *prev_cb_data, void *new_cb_data,
				 void *uc_data,
				 int to_uc,
				 const char *reason);
	ssize_t cb_data_size;
};

struct bms_usecase_entry {
	int id;
	int usecase;

	char *reason;
	long value;

	enum bms_usecase_status status;

	struct semaphore sem;
	atomic_t sem_count;

	enum bms_usecase_state state;
	void *cb_data;
	struct device *dev;

	bool processed_hop;

	int mode_cb_ret;
	int uc_work_ret;

	struct klist_node list_node;
};

struct bms_usecase_data {
	int cur_usecase;
	int reg;

	struct klist queue;
	struct mutex queue_lock;
	struct mutex usecase_work_lock;

	struct bms_usecase_entry pool[BMS_USECASE_MAX_ENTRIES];
	struct mutex pool_lock;

#if defined(CONFIG_DEBUG_FS)
	struct dentry *de;
	struct delayed_work work;
#endif
	struct list_head subscribers;

	struct bms_usecase_chg_data *chg_data;
};

typedef void (*bms_usecase_cb)(void *data, enum gsu_usecases from_usecase,
			      enum gsu_usecases to_usecase);

struct bms_usecase_notify_data {
	void *data;
	bms_usecase_cb uc_changed_cb;
	bms_usecase_cb uc_setup_cb;
	char identifier[GVOTABLE_MAX_REASON_LEN];
	struct list_head list;
};

int bms_usecase_init(struct bms_usecase_data *bms_uc_data,
		     struct bms_usecase_chg_data *data);
void bms_usecase_remove(struct bms_usecase_data *bms_uc_data);
void bms_usecase_add_tail(struct bms_usecase_data *bms_uc_data, struct bms_usecase_entry *entry);
void bms_usecase_clear_queue(struct bms_usecase_data *bms_uc_data, int err);
void bms_usecase_entry_set_usecase(struct bms_usecase_entry *entry, int usecase);
int bms_usecase_get_usecase(struct bms_usecase_data *bms_uc_data);
int bms_usecase_get_reg(struct bms_usecase_data *bms_uc_data);
void bms_usecase_set(struct bms_usecase_data *bms_uc_data, int usecase, u8 reg);
struct bms_usecase_entry *bms_usecase_get_new_node(struct bms_usecase_data *bms_uc_data,
						   const char *reason, long value);
struct bms_usecase_entry *bms_usecase_get_entry(struct bms_usecase_data *bms_uc_data, int entry_id);
int bms_usecase_add_hops(struct bms_usecase_data *bms_uc_data, struct bms_usecase_entry *entry);
void bms_usecase_free_node(struct bms_usecase_data *bms_uc_data, struct bms_usecase_entry *entry,
			   enum bms_usecase_free_state free_state);
void bms_usecase_up(struct bms_usecase_entry *entry);
int bms_usecase_down(struct bms_usecase_entry *entry);
void bms_usecase_work_lock(struct bms_usecase_data *bms_uc_data);
void bms_usecase_work_unlock(struct bms_usecase_data *bms_uc_data);
struct klist_node *bms_usecase_queue_next(struct bms_usecase_data *bms_uc_data,
					  struct klist_iter *iter);
int bms_usecase_register_notifiers(void *data, bms_usecase_cb uc_setup_cb,
				   bms_usecase_cb uc_changed_cb, const char *identifier);
void bms_usecase_uc_changed_notify(struct bms_usecase_data *bms_uc_data, enum gsu_usecases from_uc,
				   enum gsu_usecases to_uc);
void bms_usecase_uc_setup_notify(struct bms_usecase_data *bms_uc_data, enum gsu_usecases from_uc,
				 enum gsu_usecases to_uc);

const char *bms_usecase_to_str(enum gsu_usecases usecase);
bool bms_usecase_is_uc_wireless(enum gsu_usecases usecase);
bool bms_usecase_is_uc_wired(enum gsu_usecases usecase);
bool bms_usecase_is_uc_charging_enabled(enum gsu_usecases usecase);
bool bms_usecase_is_chg_changed(enum gsu_usecases from_uc, enum gsu_usecases to_uc);
bool bms_usecase_is_uc_cp(enum gsu_usecases usecase);
bool bms_usecase_is_uc_otg(enum gsu_usecases usecase);
#endif
