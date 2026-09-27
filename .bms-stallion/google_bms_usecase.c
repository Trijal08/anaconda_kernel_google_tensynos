// SPDX-License-Identifier: GPL-2.0
/*
 * Google BMS Common Usecase Driver
 *
 * Copyright 2024 Google LLC
 *
 */

#include <linux/debugfs.h>
#include <linux/hashtable.h>

#include "google_bms.h"
#include "google_bms_usecase.h"

static struct bms_usecase_data *singleton_bms_uc_data;

static struct gsu_usecase_config_t _gsu_usecase_config[] = {
	FOREACH_GSU_USECASE(GSU_USECASE_CONFIG)
};

#define BMS_USECASE_NUM_USECASES (sizeof(_gsu_usecase_config) / sizeof(struct gsu_usecase_config_t))

/* 1 << 5 = 32 entries */
#define BMS_USECASE_HASHTABLE_SIZE	5
DECLARE_HASHTABLE(gsu_usecase_table, BMS_USECASE_HASHTABLE_SIZE);

static void bms_usecase_init_uc_hash_table(void)
{
	int i;

	for (i = 0; i < BMS_USECASE_NUM_USECASES; i++) {
		hash_add(gsu_usecase_table, &_gsu_usecase_config[i].hnode,
			 _gsu_usecase_config[i].usecase);
	}
}

const char *bms_usecase_to_str(enum gsu_usecases usecase)
{
	struct gsu_usecase_config_t *config;

	hash_for_each_possible(gsu_usecase_table, config, hnode, usecase) {
		if (config->usecase == usecase)
			return config->name;
	}

	return "unknown";
}
EXPORT_SYMBOL_GPL(bms_usecase_to_str);

bool bms_usecase_is_uc_wireless(enum gsu_usecases usecase)
{
	struct gsu_usecase_config_t *config;

	hash_for_each_possible(gsu_usecase_table, config, hnode, usecase) {
		if (config->usecase == usecase)
			return config->is_wireless;
	}

	dev_err(singleton_bms_uc_data->chg_data->dev, "%s: Error could not find usecase %d\n",
		__func__, usecase);

	return false;
}
EXPORT_SYMBOL_GPL(bms_usecase_is_uc_wireless);

bool bms_usecase_is_uc_wired(enum gsu_usecases usecase)
{
	struct gsu_usecase_config_t *config;

	hash_for_each_possible(gsu_usecase_table, config, hnode, usecase) {
		if (config->usecase == usecase)
			return config->is_wired;
	}

	dev_err(singleton_bms_uc_data->chg_data->dev, "%s: Error could not find usecase %d\n",
		__func__, usecase);

	return false;
}
EXPORT_SYMBOL_GPL(bms_usecase_is_uc_wired);

bool bms_usecase_is_uc_charging_enabled(enum gsu_usecases usecase)
{
	struct gsu_usecase_config_t *config;

	hash_for_each_possible(gsu_usecase_table, config, hnode, usecase) {
		if (config->usecase == usecase)
			return config->is_charging;
	}

	dev_err(singleton_bms_uc_data->chg_data->dev, "%s: Error could not find usecase %d\n",
		__func__, usecase);

	return false;
}
EXPORT_SYMBOL_GPL(bms_usecase_is_uc_charging_enabled);

bool bms_usecase_is_chg_changed(enum gsu_usecases from_uc, enum gsu_usecases to_uc)
{
	struct gsu_usecase_config_t *config;
	int from_chg = -1, to_chg = -1;

	hash_for_each_possible(gsu_usecase_table, config, hnode, from_uc) {
		if (config->usecase == from_uc) {
			from_chg = config->chg_index;
			break;
		}
	}

	hash_for_each_possible(gsu_usecase_table, config, hnode, to_uc) {
		if (config->usecase == to_uc) {
			to_chg = config->chg_index;
			break;
		}
	}

	if (from_chg == -1 || to_chg == -1) {
		dev_err(singleton_bms_uc_data->chg_data->dev,
		"%s: Error could not find usecase from_uc:%d chg:%d to_uc:%d chg:%d\n",
		__func__, from_uc, from_chg, to_uc, to_chg);
		return 0;
	}

	return from_chg != to_chg;
}
EXPORT_SYMBOL_GPL(bms_usecase_is_chg_changed);

bool bms_usecase_is_uc_cp(enum gsu_usecases usecase)
{
	struct gsu_usecase_config_t *config;

	hash_for_each_possible(gsu_usecase_table, config, hnode, usecase) {
		if (config->usecase == usecase)
			return config->is_cp;
	}

	dev_err(singleton_bms_uc_data->chg_data->dev, "%s: Error could not find usecase %d\n",
		__func__, usecase);

	return false;
}
EXPORT_SYMBOL_GPL(bms_usecase_is_uc_cp);

bool bms_usecase_is_uc_otg(enum gsu_usecases usecase)
{
	struct gsu_usecase_config_t *config;

	hash_for_each_possible(gsu_usecase_table, config, hnode, usecase) {
		if (config->usecase == usecase)
			return config->is_otg;
	}

	dev_err(singleton_bms_uc_data->chg_data->dev, "%s: Error could not find usecase %d\n",
		__func__, usecase);

	return false;
}
EXPORT_SYMBOL_GPL(bms_usecase_is_uc_otg);

static struct bms_usecase_entry *bms_usecase_alloc_node(struct bms_usecase_data *bms_uc_data,
							enum bms_usecase_state state,
							const char *reason,
							long value)
{
	int i;
	struct bms_usecase_entry *entry = NULL;
	struct device *dev;

	if (!singleton_bms_uc_data)
		return NULL;

	dev = bms_uc_data->chg_data->dev;

	mutex_lock(&bms_uc_data->pool_lock);
	for (i = 0; i < BMS_USECASE_MAX_ENTRIES; i++) {
		struct bms_usecase_entry *tmp = &bms_uc_data->pool[i];

		if (tmp->state != BMS_USECASE_UNINITIALIZED)
			continue;

		entry = tmp;
		entry->state = state;
		entry->status = BMS_USECASE_STATUS_NEW;
		strscpy(entry->reason, reason, GVOTABLE_MAX_REASON_LEN);
		entry->value = value;

		dev_dbg(dev, "Allocating node:%d state:%d reason:%s value:0x%lx\n",
			entry->id, state, entry->reason, entry->value);
		break;
	}
	mutex_unlock(&bms_uc_data->pool_lock);

	return entry;
}

struct bms_usecase_entry *bms_usecase_get_new_node(struct bms_usecase_data *bms_uc_data,
						   const char *reason,
						   long value)
{
	return bms_usecase_alloc_node(bms_uc_data, BMS_USECASE_INITIAL,
				      reason, value);
}
EXPORT_SYMBOL_GPL(bms_usecase_get_new_node);

static struct bms_usecase_entry*
bms_usecase_get_intermediate_node(struct bms_usecase_data *bms_uc_data)
{
	return bms_usecase_alloc_node(bms_uc_data, BMS_USECASE_INTERMEDIATE,
				      BMS_USECASE_INTERMEDIATE_STR,
				      _bms_usecase_meta_async_set(0, 1));
}

struct bms_usecase_entry *bms_usecase_get_entry(struct bms_usecase_data *bms_uc_data,
							int entry_id)
{
	if (entry_id >= BMS_USECASE_MAX_ENTRIES || entry_id < 0)
		return NULL;

	return &bms_uc_data->pool[entry_id];
}
EXPORT_SYMBOL_GPL(bms_usecase_get_entry);

static int bms_usecase_add_hop(struct bms_usecase_data *bms_uc_data,
			       const int from_uc,
			       const int to_uc,
			       struct bms_usecase_entry *orig_entry,
			       int *hops)
{
	int temp_uc, ret;
	struct bms_usecase_entry *new_entry;
	void *uc_data = bms_uc_data->chg_data->uc_data;
	struct device *dev = bms_uc_data->chg_data->dev;

	dev_dbg(dev, "%s: from_uc:%d to_uc:%d\n", __func__, from_uc, to_uc);

	if (!bms_uc_data->chg_data->hop_func || (from_uc == to_uc))
		return 0;

	temp_uc = bms_uc_data->chg_data->hop_func(uc_data, from_uc, to_uc);
	if (temp_uc == BMS_USECASE_NO_HOPS)
		return 0;
	if (temp_uc < 0) {
		dev_err(dev, "error in hop func:%d\n", temp_uc);
		return temp_uc;
	}

	ret = bms_usecase_add_hop(bms_uc_data, temp_uc, to_uc, orig_entry, hops);
	if (ret == -ENOMEM)
		return ret;

	new_entry = bms_usecase_get_intermediate_node(bms_uc_data);
	if (!new_entry) {
		dev_err(dev, "No mem in pool\n");
		return -ENOMEM;
	}

	bms_usecase_entry_set_usecase(new_entry, temp_uc);
	new_entry->processed_hop = true;
	/* entry->value and cb-data->value are unused in this case */

	if (bms_uc_data->chg_data->populate_cb_data)
		bms_uc_data->chg_data->populate_cb_data(orig_entry->cb_data,
							new_entry->cb_data,
							uc_data,
							new_entry->usecase,
							BMS_USECASE_INTERMEDIATE_STR);

	dev_dbg(dev, "Adding hop node:%d usecase:%d\n", new_entry->id,
		new_entry->usecase);

	/* queue lock is already held */
	klist_add_head(&new_entry->list_node, &bms_uc_data->queue);

	*hops += 1;

	return 0;
}

int bms_usecase_add_hops(struct bms_usecase_data *bms_uc_data, struct bms_usecase_entry *entry)
{
	int ret;
	int hops = 0;

	if (entry->processed_hop)
		return 0;

	mutex_lock(&bms_uc_data->queue_lock);

	ret = bms_usecase_add_hop(bms_uc_data, bms_uc_data->cur_usecase,
				  entry->usecase, entry, &hops);
	if (ret == 0)
		entry->processed_hop = true;

	mutex_unlock(&bms_uc_data->queue_lock);

	return ret ? ret : hops;
}
EXPORT_SYMBOL_GPL(bms_usecase_add_hops);

void bms_usecase_entry_set_usecase(struct bms_usecase_entry *entry, int usecase)
{
	entry->usecase = usecase;
}
EXPORT_SYMBOL_GPL(bms_usecase_entry_set_usecase);

struct klist_node *bms_usecase_queue_next(struct bms_usecase_data *bms_uc_data,
					  struct klist_iter *iter)
{
	struct klist_node *node;

	mutex_lock(&bms_uc_data->queue_lock);
	node = klist_next(iter);
	mutex_unlock(&bms_uc_data->queue_lock);

	return node;
}
EXPORT_SYMBOL_GPL(bms_usecase_queue_next);

static void bms_usecase_add_tail_locked(struct bms_usecase_data *bms_uc_data,
					struct bms_usecase_entry *entry)
{
	struct device *dev = bms_uc_data->chg_data->dev;

	dev_dbg(dev, "Adding node:%d blocking:%d\n", entry->id,
		!_bms_usecase_meta_async_get(entry->value));

	klist_add_tail(&entry->list_node, &bms_uc_data->queue);
}

void bms_usecase_add_tail(struct bms_usecase_data *bms_uc_data, struct bms_usecase_entry *entry)
{
	mutex_lock(&bms_uc_data->queue_lock);
	bms_usecase_add_tail_locked(bms_uc_data, entry);
	mutex_unlock(&bms_uc_data->queue_lock);
}
EXPORT_SYMBOL_GPL(bms_usecase_add_tail);

static void bms_usecase_free_node_locked(struct bms_usecase_data *bms_uc_data, struct bms_usecase_entry *entry,
			   enum bms_usecase_free_state free_state)
{
	struct device *dev = bms_uc_data->chg_data->dev;

	dev_dbg(dev, "Freeing node:%d free_state:%d\n", entry->id, free_state);

	if (free_state == BMS_USECASE_FREE_FROM_LIST || free_state == BMS_USECASE_FREE_ALL) {
		klist_remove(&entry->list_node);

		if (!_bms_usecase_meta_async_get(entry->value))
			bms_usecase_up(entry);
	}

	if (free_state == BMS_USECASE_FREE_ENTRY || free_state == BMS_USECASE_FREE_ALL) {
		mutex_lock(&bms_uc_data->pool_lock);
		entry->usecase = 0;
		entry->processed_hop = false;
		entry->state = BMS_USECASE_UNINITIALIZED;
		entry->status = BMS_USECASE_STATUS_UNINITIALIZED;
		entry->value = 0;
		entry->mode_cb_ret = 0;
		entry->uc_work_ret = 0;

		/* sem_count and sem state are not reset */
		memset(entry->cb_data, 0, bms_uc_data->chg_data->cb_data_size);
		memset(entry->reason, 0, GVOTABLE_MAX_REASON_LEN);
		mutex_unlock(&bms_uc_data->pool_lock);
	}
}

/*
 * When free_state is BMS_USECASE_FREE_FROM_LIST or BMS_USECASE_FREE_ALL, requires
 * &bms_uc_data->queue_lock to be held
 */
void bms_usecase_free_node(struct bms_usecase_data *bms_uc_data, struct bms_usecase_entry *entry,
			   enum bms_usecase_free_state free_state)
{
	mutex_lock(&bms_uc_data->queue_lock);
	bms_usecase_free_node_locked(bms_uc_data, entry, free_state);
	mutex_unlock(&bms_uc_data->queue_lock);
}
EXPORT_SYMBOL_GPL(bms_usecase_free_node);

void bms_usecase_clear_queue(struct bms_usecase_data *bms_uc_data, int err)
{
	struct klist_iter iter;
	struct klist_node *node;
	struct bms_usecase_entry *entry;
	struct device *dev = bms_uc_data->chg_data->dev;

	dev_warn(dev, "Clearing queue\n");
	mutex_lock(&bms_uc_data->queue_lock);

	klist_iter_init(&bms_uc_data->queue, &iter);

	/* klist_next requires &bms_uc_data->queue_lock */
	node = klist_next(&iter);
	while (node) {
		entry = container_of(node, struct bms_usecase_entry, list_node);

		node = klist_next(&iter);
		entry->uc_work_ret = err;
		bms_usecase_free_node_locked(bms_uc_data, entry,
				      !_bms_usecase_meta_async_get(entry->value));
	}

	klist_iter_exit(&iter);
	mutex_unlock(&bms_uc_data->queue_lock);
}
EXPORT_SYMBOL_GPL(bms_usecase_clear_queue);

void bms_usecase_up(struct bms_usecase_entry *entry)
{
	dev_dbg(entry->dev, "%s: node:%d\n", __func__, entry->id);

	atomic_inc(&entry->sem_count);
	up(&entry->sem);
}
EXPORT_SYMBOL_GPL(bms_usecase_up);

static void bms_usecase_reset_sem(struct bms_usecase_entry *entry)
{
	sema_init(&entry->sem, 0);
	atomic_set(&entry->sem_count, 0);
}

int bms_usecase_down(struct bms_usecase_entry *entry)
{
	int ret, sem_count;

	dev_dbg(entry->dev, "%s: node:%d\n", __func__, entry->id);

	ret = down_interruptible(&entry->sem);
	if (ret < 0) {
		dev_err(entry->dev, "Error! Can not down sem on node:%d ret:%d\n",
			entry->id, ret);
		return ret;
	}
	sem_count = atomic_read(&entry->sem_count);

	ret = atomic_dec_and_test(&entry->sem_count);
	if (!ret) {
		dev_err(entry->dev, "Error! Atomic sem count mismatch %d->%d"
			" ... resetting state on node:%d\n",
			sem_count, atomic_read(&entry->sem_count), entry->id);
		bms_usecase_reset_sem(entry);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(bms_usecase_down);

int bms_usecase_get_usecase(struct bms_usecase_data *bms_uc_data)
{
	return bms_uc_data->cur_usecase;
}
EXPORT_SYMBOL_GPL(bms_usecase_get_usecase);

int bms_usecase_get_reg(struct bms_usecase_data *bms_uc_data)
{
	return bms_uc_data->reg;
}
EXPORT_SYMBOL_GPL(bms_usecase_get_reg);

void bms_usecase_set(struct bms_usecase_data *bms_uc_data, int usecase, u8 reg)
{
	bms_uc_data->cur_usecase = usecase;
	bms_uc_data->reg = reg;
}
EXPORT_SYMBOL_GPL(bms_usecase_set);

/*
 * This is the top level lock. If should always be acquired first before other
 * locks if used.
 */
void bms_usecase_work_lock(struct bms_usecase_data *bms_uc_data)
{
	mutex_lock(&bms_uc_data->usecase_work_lock);
}
EXPORT_SYMBOL_GPL(bms_usecase_work_lock);

void bms_usecase_work_unlock(struct bms_usecase_data *bms_uc_data)
{
	mutex_unlock(&bms_uc_data->usecase_work_lock);
}
EXPORT_SYMBOL_GPL(bms_usecase_work_unlock);

#if defined(CONFIG_DEBUG_FS)
static const char *bms_usecase_state_to_str(enum bms_usecase_state state)
{
	switch (state) {
	case BMS_USECASE_UNINITIALIZED:
		return "Uninitialized";
	case BMS_USECASE_INITIAL:
		return "Initial";
	case BMS_USECASE_INTERMEDIATE:
		return BMS_USECASE_INTERMEDIATE_STR;
	default:
		return "Unknown";
	}
}

static void bms_usecase_process_work(struct work_struct *work)
{
	struct bms_usecase_data *bms_uc_data =
		container_of(work, struct bms_usecase_data, work.work);
	struct klist_iter iter;
	struct klist_node *node;
	struct device *dev = bms_uc_data->chg_data->dev;
	bool reschedule = false, err = false;
	int hops;

	bms_usecase_work_lock(bms_uc_data);

	klist_iter_init(&bms_uc_data->queue, &iter);

	node = bms_usecase_queue_next(bms_uc_data, &iter);
	while (node) {
		struct bms_usecase_entry *entry =
			container_of(node, struct bms_usecase_entry, list_node);

		hops = bms_usecase_add_hops(bms_uc_data, entry);
		if (hops < 0) {
			dev_err(dev, "Error adding hops (%d)\n", hops);
			err = true;
			break;
		}
		if (hops) {
			reschedule = true;
			break;
		}

		dev_info(dev, "Usecase:%s(%d)->%s(%d) state:%s Node:%d\n",
			 bms_usecase_to_str(bms_uc_data->cur_usecase), bms_uc_data->cur_usecase,
			 bms_usecase_to_str(entry->usecase), entry->usecase,
			 bms_usecase_state_to_str(entry->state),
			 entry->id);

		/* set usecase */
		bms_usecase_set(bms_uc_data, entry->usecase, 0);

		node = bms_usecase_queue_next(bms_uc_data, &iter);
		/*
		 * Check for ownership transfer via the orphan status.
		 * Must hold queue_lock to ensure atomic check-and-free relative to
		 * the caller setting the flag.
		 */
		mutex_lock(&bms_uc_data->queue_lock);
		if (!_bms_usecase_meta_async_get(entry->value) &&
		    entry->status == BMS_USECASE_STATUS_ORPHAN) {
			bms_usecase_free_node_locked(bms_uc_data, entry, BMS_USECASE_FREE_ALL);
		} else {
			bms_usecase_free_node_locked(bms_uc_data, entry,
						!_bms_usecase_meta_async_get(entry->value));
		}
		mutex_unlock(&bms_uc_data->queue_lock);

	}

	klist_iter_exit(&iter);

	if (err)
		bms_usecase_clear_queue(bms_uc_data, err);

	if (reschedule) {
		dev_info(dev, "Rescheduling\n");

		schedule_delayed_work(&bms_uc_data->work, 0);
	}

	bms_usecase_work_unlock(bms_uc_data);
}

static ssize_t debug_bms_usecase_get_queue(struct file *filp, char __user *buf,
					   size_t count, loff_t *ppos)
{
	struct bms_usecase_data *bms_uc_data = (struct bms_usecase_data *)filp->private_data;
	char *tmp;
	int len = 0;
	struct klist_iter iter;
	struct klist_node *node;

	tmp = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	mutex_lock(&bms_uc_data->queue_lock);

	if (list_empty(&bms_uc_data->queue.k_list)) {
		len += sysfs_emit_at(tmp, len, "list empty\n");
		goto free;
	}

	klist_iter_init(&bms_uc_data->queue, &iter);
	for (node = klist_next(&iter); node; node = klist_next(&iter)) {
		struct bms_usecase_entry *entry =
			container_of(node, struct bms_usecase_entry, list_node);

		len += sysfs_emit_at(tmp, len,
				     "node:%d usecase:%d state:%s status:%d reason:%s value:0x%x value_meta:0x%x\n",
				     entry->id, entry->usecase,
				     bms_usecase_state_to_str(entry->state),
				     entry->status,
				     entry->reason,
				     _bms_usecase_mode_get(entry->value),
				     _bms_usecase_meta_get(entry->value));
	}
	klist_iter_exit(&iter);

free:
	len = simple_read_from_buffer(buf, count, ppos, tmp, strlen(tmp));

	mutex_unlock(&bms_uc_data->queue_lock);

	kfree(tmp);

	return len;
}

static ssize_t debug_bms_usecase_add_queue(struct file *filp,
					   const char __user *user_buf,
					   size_t count, loff_t *ppos)
{
	struct bms_usecase_data *bms_uc_data = (struct bms_usecase_data *)filp->private_data;
	struct device *dev = bms_uc_data->chg_data->dev;
	struct bms_usecase_entry *entry;
	const int mem_size = count + 1;
	char *tmp, *cur, *saved_ptr;
	int ret, val, i;

	tmp = kzalloc(mem_size, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	ret = simple_write_to_buffer(tmp, mem_size, ppos, user_buf, count);
	if (ret < 0) {
		dev_err(dev, "%s couldn't write to buffer ret:%d\n", __func__, ret);
		return ret;
	}

	mutex_lock(&bms_uc_data->queue_lock);
	for (saved_ptr = tmp, i = 0; i < BMS_USECASE_MAX_ENTRIES; i++) {
		cur = strsep(&saved_ptr, " ");
		if (!cur)
			break;

		ret = kstrtoint(cur, 0, &val);
		if (ret < 0) {
			break;
		}

		entry = bms_usecase_get_new_node(bms_uc_data, "DEBUGFS",
						 _bms_usecase_meta_async_set(val, 1));
		if (!entry) {
			dev_err(dev, "%s couldn't allocate entry\n", __func__);
			ret = -ENOMEM;
			break;
		}
		bms_usecase_entry_set_usecase(entry, val);
		bms_usecase_add_tail_locked(bms_uc_data, entry);
	}
	mutex_unlock(&bms_uc_data->queue_lock);

	kfree(tmp);
	return ret < 0 ? ret : count;
}

BATTERY_DEBUG_ATTRIBUTE(debug_bms_usecase_queue_fops, debug_bms_usecase_get_queue,
			debug_bms_usecase_add_queue);

static ssize_t debug_bms_usecase_get_pool(struct file *filp, char __user *buf,
					  size_t count, loff_t *ppos)
{
	struct bms_usecase_data *bms_uc_data = (struct bms_usecase_data *)filp->private_data;
	char *tmp;
	int i = 0, len = 0;

	tmp = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	mutex_lock(&bms_uc_data->pool_lock);

	for (i = 0; i < BMS_USECASE_MAX_ENTRIES; i++) {
		struct bms_usecase_entry *entry = &bms_uc_data->pool[i];

		len += sysfs_emit_at(tmp, len,
				     "%2d: node:%d usecase:%d state:%s status:%d reason:%s value:0x%x value_meta:0x%x\n",
				     i, entry->id, entry->usecase,
				     bms_usecase_state_to_str(entry->state),
				     entry->status,
				     entry->reason,
				     _bms_usecase_mode_get(entry->value),
				     _bms_usecase_meta_get(entry->value));
	}

	len = simple_read_from_buffer(buf, count, ppos, tmp, strlen(tmp));

	mutex_unlock(&bms_uc_data->pool_lock);

	kfree(tmp);

	return len;
}

BATTERY_DEBUG_ATTRIBUTE(debug_bms_usecase_pool_fops, debug_bms_usecase_get_pool, NULL);

static int debug_bms_usecase_process_queue(void *d, u64 val)
{
	struct bms_usecase_data *bms_uc_data = (struct bms_usecase_data *)d;

	if (val)
		schedule_delayed_work(&bms_uc_data->work, 0);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_bms_usecase_process_queue_fops,
				NULL,
				debug_bms_usecase_process_queue, "%llu\n");

static void bms_usecase_debugfs_init(struct bms_usecase_data *bms_uc_data)
{
	bms_uc_data->de = debugfs_create_dir("google_bms_usecase", 0);
	if (!bms_uc_data->de)
		return;

	debugfs_create_file("process_queue", 0400, bms_uc_data->de, bms_uc_data,
			    &debug_bms_usecase_process_queue_fops);
	debugfs_create_file("queue", 0400, bms_uc_data->de, bms_uc_data,
			    &debug_bms_usecase_queue_fops);
	debugfs_create_file("pool", 0400, bms_uc_data->de, bms_uc_data,
			    &debug_bms_usecase_pool_fops);
}
#endif

int bms_usecase_register_notifiers(void *data, bms_usecase_cb uc_setup_cb,
				   bms_usecase_cb uc_changed_cb, const char *identifier)
{
	struct bms_usecase_notify_data *notify_data;
	int ret = 0;

	if (!singleton_bms_uc_data)
		return -EAGAIN;

	bms_usecase_work_lock(singleton_bms_uc_data);

	notify_data = kzalloc(sizeof(*notify_data), GFP_KERNEL);
	if (!notify_data) {
		ret = -ENOMEM;
		goto unlock;
	}

	notify_data->data = data;
	notify_data->uc_setup_cb = uc_setup_cb;
	notify_data->uc_changed_cb = uc_changed_cb;
	strscpy(notify_data->identifier, identifier, GVOTABLE_MAX_REASON_LEN);

	list_add(&notify_data->list, &singleton_bms_uc_data->subscribers);

	uc_changed_cb(data, 0, bms_usecase_get_usecase(singleton_bms_uc_data));

unlock:
	bms_usecase_work_unlock(singleton_bms_uc_data);
	return ret;
}
EXPORT_SYMBOL_GPL(bms_usecase_register_notifiers);

/* requires bms_usecase_work_lock */
static void bms_usecase_unregister_notifiers(struct bms_usecase_data *bms_uc_data)
{
	struct bms_usecase_notify_data *sub, *tmp;

	list_for_each_entry_safe(sub, tmp, &bms_uc_data->subscribers, list) {
		list_del(&sub->list);
		kfree(sub);
	}
}

/* requires bms_usecase_work_lock */
void bms_usecase_uc_changed_notify(struct bms_usecase_data *bms_uc_data, enum gsu_usecases from_uc,
				   enum gsu_usecases to_uc)
{
	struct bms_usecase_notify_data *sub;
	struct device *dev = bms_uc_data->chg_data->dev;

	list_for_each_entry(sub, &bms_uc_data->subscribers, list) {
		if (sub->uc_changed_cb) {
			dev_dbg(dev, "%s: ====== %s START =======\n", __func__, sub->identifier);
			sub->uc_changed_cb(sub->data, from_uc, to_uc);
			dev_dbg(dev, "%s: ====== %s END =======\n", __func__, sub->identifier);
		}
	}
}
EXPORT_SYMBOL_GPL(bms_usecase_uc_changed_notify);

/* requires bms_usecase_work_lock */
void bms_usecase_uc_setup_notify(struct bms_usecase_data *bms_uc_data, enum gsu_usecases from_uc,
				 enum gsu_usecases to_uc)
{
	struct bms_usecase_notify_data *sub;
	struct device *dev = bms_uc_data->chg_data->dev;

	list_for_each_entry(sub, &bms_uc_data->subscribers, list) {
		if (sub->uc_setup_cb) {
			dev_dbg(dev, "%s: ====== %s START =======\n", __func__, sub->identifier);
			sub->uc_setup_cb(sub->data, from_uc, to_uc);
			dev_dbg(dev, "%s: ====== %s END =======\n", __func__, sub->identifier);
		}
	}
}
EXPORT_SYMBOL_GPL(bms_usecase_uc_setup_notify);

void bms_usecase_remove(struct bms_usecase_data *bms_uc_data)
{
	bms_usecase_unregister_notifiers(bms_uc_data);

	debugfs_remove(bms_uc_data->de);

	mutex_lock(&bms_uc_data->pool_lock);
	singleton_bms_uc_data = NULL;
	mutex_unlock(&bms_uc_data->pool_lock);

	bms_usecase_clear_queue(bms_uc_data, 0);

	mutex_destroy(&bms_uc_data->pool_lock);
	mutex_destroy(&bms_uc_data->queue_lock);
	mutex_destroy(&bms_uc_data->usecase_work_lock);
}
EXPORT_SYMBOL_GPL(bms_usecase_remove);

int bms_usecase_init(struct bms_usecase_data *bms_uc_data, struct bms_usecase_chg_data *data)
{
	int i;
	struct device *dev = data->dev;
	ssize_t cb_data_size = data->cb_data_size;

	bms_uc_data->chg_data = data;

	mutex_init(&bms_uc_data->usecase_work_lock);
	mutex_init(&bms_uc_data->queue_lock);
	mutex_init(&bms_uc_data->pool_lock);

	INIT_LIST_HEAD(&bms_uc_data->subscribers);

	klist_init(&bms_uc_data->queue, NULL, NULL);
	bms_usecase_init_uc_hash_table();

	for (i = 0; i < BMS_USECASE_MAX_ENTRIES; i++) {
		struct bms_usecase_entry *entry = &bms_uc_data->pool[i];

		entry->id = i;
		entry->dev = dev;
		entry->status = BMS_USECASE_STATUS_UNINITIALIZED;
		sema_init(&entry->sem, 0);
		atomic_set(&entry->sem_count, 0);

		entry->reason = devm_kzalloc(dev, GVOTABLE_MAX_REASON_LEN, GFP_KERNEL);
		if (!entry->reason)
			return -ENOMEM;
		entry->cb_data = devm_kzalloc(dev, cb_data_size, GFP_KERNEL);
		if (!entry->cb_data)
			return -ENOMEM;
	}

#if defined(CONFIG_DEBUG_FS)
	INIT_DELAYED_WORK(&bms_uc_data->work, bms_usecase_process_work);

	bms_usecase_debugfs_init(bms_uc_data);
#endif

	singleton_bms_uc_data = bms_uc_data;

	dev_info(dev, "%s complete\n", __func__);

	return 0;
}
EXPORT_SYMBOL_GPL(bms_usecase_init);
