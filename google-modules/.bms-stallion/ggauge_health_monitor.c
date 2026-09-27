// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2026 Google LLC
 */

#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/timekeeping.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include "google_bms.h"
#include "max77779.h"
#include "max77779_vimon.h"

enum ghm_signal_reason {
	GHM_REASON_START_CHARGING = 1,
	GHM_REASON_STOP_CHARGING = 2,
};

#define GHM_VIMON_MASK (VIMON_IMMEDIATE_TRIGGER)
#define GHM_VIMON_COUNT (VIMON_CLIENT_ALWAYS_RUN)

#define GHM_DT_PHANDLE_NAME "google,vimon"
#define GHM_LOGBUFFER_NAME "ggauge_health_monitor"

/* Defaults */
#define GHM_DEFAULT_DURATION_MS 10000
#define GHM_DEFAULT_BIN_WIDTH_MS 250
#define GHM_DEFAULT_EWMA_SHIFT 2

/* Limits */
#define GHM_MAX_BINS 4096
#define GHM_MAX_RAW_SAMPLES 128000
#define GHM_LIMIT_DUR_MIN_MS 1000
#define GHM_LIMIT_DUR_MAX_MS 3600000
#define GHM_LIMIT_WIDTH_MIN_MS 100
#define GHM_LIMIT_WIDTH_MAX_MS 60000
#define GHM_LIMIT_EWMA_SHIFT_MIN 0
#define GHM_LIMIT_EWMA_SHIFT_MAX 10

/* Constants */
#define GHM_LOG_FORMAT_VERSION 1
#define GHM_EWMA_FIXED_SCALE 1024
#define GHM_DVDT_BIN_NO_DATA 9999999
#define GHM_DVDT_LOG_TYPE 0x4456	   /* 'DV' in hex for dV/dt Curve */
#define GHM_RAW_LOG_TYPE 0x4444		   /* 'DD' in hex for Debug Data */
#define GHM_SESSION_HEADER_LOG_TYPE 0x4844 /* 'HD' in hex for Header */

/* Conversion Helpers */
#define NANO_PER_MICRO 1000
#define NANO_PER_MILLI 1000000
#define NANO_PER_UNIT 1000000000

/* Fixed Sampling Configuration: 60Hz */
/* Period = 1,000,000,000 ns / 60 = 16,666,666 ns */
#define GHM_SAMPLING_FREQ_HZ 60
#define GHM_SAMPLE_PERIOD_NS (NANO_PER_UNIT / GHM_SAMPLING_FREQ_HZ)

struct max77779_sample_data {
	u16 v_val;
	s16 i_val;
} __packed;

struct ggauge_health_monitor {
	struct device *dev;
	struct device *vimon_dev;
	struct power_supply *chg_psy;
	struct power_supply *bat_psy;
	struct notifier_block psy_nb;
	struct logbuffer *ghm_logbuf;
	struct dentry *debugfs_dir;

	/* Configuration */
	struct mutex config_lock; /* Protects buffers and config params */
	bool session_active;
	bool verbose_logging_enabled;

	/* Logic State */
	int session_id;
	enum ghm_signal_reason session_type;
	int last_cc_max;

	/* Configurable Parameters */
	u32 monitor_duration_ms;
	u32 bin_width_ms;
	u32 ewma_shift;
	u32 ewma_scale;
	u32 bin_count;

	/* Workqueues */
	struct delayed_work monitor_stop_work;
	struct work_struct session_work;

	/* VIMON Interface */
	struct vimon_client_callbacks callbacks;

	/* EWMA Filter State */
	s64 v_ewma_state;
	bool v_filter_primed;

	/* Data Buffers */
	s32 *dvdt_curve;
	s64 *bin_current_sum;
	u64 *bin_first_v_uv;
	u64 *bin_last_v_uv;
	u32 *bin_sample_count;

	/* Raw Debug Buffer */
	struct max77779_sample_data *raw_samples;
	u32 raw_sample_cursor;
	u32 raw_sample_max;

	/* Session Data */
	u64 bin_duration_ns;
	u64 processed_sample_count;
	int temp_start_dc;
	int temp_end_dc;
};

static inline s64 raw_to_uv(u16 raw_v)
{
	return div_s64((s64)raw_v * MAX77779_VIMON_NV_PER_LSB, NANO_PER_MICRO);
}

static s64 ghm_filter_input(
	struct ggauge_health_monitor *ghm, s64 *state, bool *primed, s64 raw_val)
{
	s64 scaled_raw = raw_val * ghm->ewma_scale;

	if (*primed == false) {
		*state = scaled_raw;
		*primed = true;
		return raw_val;
	}

	*state = *state - (*state >> ghm->ewma_shift) + (scaled_raw >> ghm->ewma_shift);

	return div_s64(*state, ghm->ewma_scale);
}

/* Requires holding config_lock or session_active flag false. */
static void ghm_free_buffers(struct ggauge_health_monitor *ghm)
{
	kvfree(ghm->dvdt_curve);
	kvfree(ghm->bin_current_sum);
	kvfree(ghm->bin_first_v_uv);
	kvfree(ghm->bin_last_v_uv);
	kvfree(ghm->bin_sample_count);

	ghm->dvdt_curve = NULL;
	ghm->bin_current_sum = NULL;
	ghm->bin_first_v_uv = NULL;
	ghm->bin_last_v_uv = NULL;
	ghm->bin_sample_count = NULL;
}

/* Requires holding config_lock or session_active flag false. */
static int ghm_alloc_buffers(struct ggauge_health_monitor *ghm, u32 bins)
{
	if (bins > GHM_MAX_BINS || bins == 0)
		return -EINVAL;

	ghm->dvdt_curve = kvmalloc_array(bins, sizeof(s32), GFP_KERNEL);
	ghm->bin_current_sum = kvmalloc_array(bins, sizeof(s64), GFP_KERNEL);
	ghm->bin_first_v_uv = kvmalloc_array(bins, sizeof(u64), GFP_KERNEL);
	ghm->bin_last_v_uv = kvmalloc_array(bins, sizeof(u64), GFP_KERNEL);
	ghm->bin_sample_count = kvmalloc_array(bins, sizeof(u32), GFP_KERNEL);

	if (!ghm->dvdt_curve || !ghm->bin_current_sum || !ghm->bin_first_v_uv ||
		!ghm->bin_last_v_uv || !ghm->bin_sample_count) {
		ghm_free_buffers(ghm);
		return -ENOMEM;
	}

	return 0;
}

/* Producer context - requires session_active flag true. */
static void ghm_vimon_sample_ready_cb(void *private_data, const enum vimon_trigger_source reason,
	const u16 *data, const size_t len)
{
	struct ggauge_health_monitor *ghm = private_data;
	const size_t num_samples = len / sizeof(struct max77779_sample_data);
	const struct max77779_sample_data *samples = (const struct max77779_sample_data *)data;
	int i;

	if (!ghm || num_samples == 0)
		return;

	if (ghm->verbose_logging_enabled && ghm->raw_samples) {
		u32 space_left = ghm->raw_sample_max - ghm->raw_sample_cursor;
		u32 copy_count = min_t(u32, (u32)num_samples, space_left);

		if (copy_count > 0) {
			memcpy(&ghm->raw_samples[ghm->raw_sample_cursor], samples,
				copy_count * sizeof(struct max77779_sample_data));
			ghm->raw_sample_cursor += copy_count;
		}
	}

	for (i = 0; i < num_samples; i++) {
		s64 v_uv_raw = raw_to_uv(samples[i].v_val);
		s64 v_uv_smooth =
			ghm_filter_input(ghm, &ghm->v_ewma_state, &ghm->v_filter_primed, v_uv_raw);
		u64 sample_relative_ts = ghm->processed_sample_count * GHM_SAMPLE_PERIOD_NS;
		int bin_index = div64_u64(sample_relative_ts, ghm->bin_duration_ns);

		if (bin_index >= ghm->bin_count)
			break;

		if (ghm->bin_sample_count[bin_index] == 0)
			ghm->bin_first_v_uv[bin_index] = v_uv_smooth;

		ghm->bin_last_v_uv[bin_index] = v_uv_smooth;
		ghm->bin_current_sum[bin_index] += samples[i].i_val;
		ghm->bin_sample_count[bin_index]++;
		ghm->processed_sample_count++;
	}
}

static void ghm_vimon_removed_cb(void *private_data)
{
	struct ggauge_health_monitor *ghm = private_data;

	if (!ghm)
		return;

	dev_warn(ghm->dev, "VIMON device removed\n");
	cancel_delayed_work(&ghm->monitor_stop_work);
}

/* Stop work context - requires session_active flag true. */
static void ghm_dump_raw_buffer(struct ggauge_health_monitor *ghm)
{
	char log_line[230];
	char temp[64];
	int pos = 0;
	int header_len = 0;
	int len = 0;
	int i;

	dev_dbg(ghm->dev, "Dumping raw samples, total: %u\n", ghm->raw_sample_cursor);

	header_len = scnprintf(log_line, sizeof(log_line), "%X", GHM_RAW_LOG_TYPE);
	pos = header_len;

	for (i = 0; i < ghm->raw_sample_cursor; i++) {
		struct max77779_sample_data *s = &ghm->raw_samples[i];

		len = scnprintf(temp, sizeof(temp), " %X %X", s->v_val, s->i_val);

		if (pos + len >= sizeof(log_line)) {
			gbms_logbuffer_prlog(
				ghm->ghm_logbuf, LOGLEVEL_INFO, 0, LOGLEVEL_INFO, "%s", log_line);

			pos = scnprintf(log_line, sizeof(log_line), "%X", GHM_RAW_LOG_TYPE);
		}

		pos += scnprintf(log_line + pos, sizeof(log_line) - pos, "%s", temp);
	}

	if (pos > header_len)
		gbms_logbuffer_prlog(
			ghm->ghm_logbuf, LOGLEVEL_INFO, 0, LOGLEVEL_INFO, "%s", log_line);
}

/* Stop work context - requires session_active flag true. */
static void ghm_dump_processed_data(struct ggauge_health_monitor *ghm)
{
	char log_line[230];
	char temp[64];
	int pos = 0;
	int header_len = 0;
	int len = 0;
	int i;

	header_len = scnprintf(log_line, sizeof(log_line), "%X", GHM_DVDT_LOG_TYPE);
	pos = header_len;

	for (i = 0; i < ghm->bin_count; i++) {
		s64 avg_current = 0;

		if (ghm->bin_sample_count[i] > 0)
			avg_current = div_s64(ghm->bin_current_sum[i], ghm->bin_sample_count[i]);

		len = scnprintf(temp, sizeof(temp), " %X %X", ghm->dvdt_curve[i], (s16)avg_current);

		if (pos + len >= sizeof(log_line)) {
			gbms_logbuffer_prlog(
				ghm->ghm_logbuf, LOGLEVEL_INFO, 0, LOGLEVEL_INFO, "%s", log_line);
			pos = scnprintf(log_line, sizeof(log_line), "%X", GHM_DVDT_LOG_TYPE);
		}

		pos += scnprintf(log_line + pos, sizeof(log_line) - pos, "%s", temp);
	}

	if (pos > header_len)
		gbms_logbuffer_prlog(
			ghm->ghm_logbuf, LOGLEVEL_INFO, 0, LOGLEVEL_INFO, "%s", log_line);
}

/* Stop work context - requires session_active flag true. */
static void ghm_calculate_and_log(struct ggauge_health_monitor *ghm)
{
	int i;

	gbms_logbuffer_prlog(ghm->ghm_logbuf, LOGLEVEL_INFO, 0, LOGLEVEL_INFO,
		"%X Ver:%d ID:%d Rsn:%d Dur:%d Width:%d Cnt:%d Filt:%d/%d Tstart:%d Tend:%d",
		GHM_SESSION_HEADER_LOG_TYPE, GHM_LOG_FORMAT_VERSION, ghm->session_id,
		ghm->session_type, ghm->monitor_duration_ms, ghm->bin_width_ms, ghm->bin_count,
		ghm->ewma_shift, ghm->ewma_scale, ghm->temp_start_dc, ghm->temp_end_dc);

	for (i = 0; i < ghm->bin_count; i++) {
		s64 dv_uv = ghm->bin_last_v_uv[i] - ghm->bin_first_v_uv[i];
		s64 dvdt_uv_s;

		if (ghm->bin_sample_count[i] <= 1) {
			ghm->dvdt_curve[i] = GHM_DVDT_BIN_NO_DATA;
			continue;
		}

		dvdt_uv_s = div64_s64(dv_uv * (s64)NANO_PER_UNIT, ghm->bin_duration_ns);
		ghm->dvdt_curve[i] = (s32)clamp(dvdt_uv_s, (s64)S32_MIN, (s64)S32_MAX);
	}

	ghm_dump_processed_data(ghm);

	if (ghm->verbose_logging_enabled && ghm->raw_samples)
		ghm_dump_raw_buffer(ghm);
}

static void ghm_monitor_stop_work_func(struct work_struct *work)
{
	struct ggauge_health_monitor *ghm =
		container_of(work, struct ggauge_health_monitor, monitor_stop_work.work);
	union power_supply_propval val;
	int ret;

	if (ghm->vimon_dev)
		vimon_unregister_callback(ghm->vimon_dev, &ghm->callbacks);

	dev_info(ghm->dev, "session %d stopping\n", ghm->session_id);

	ret = power_supply_get_property(ghm->bat_psy, POWER_SUPPLY_PROP_TEMP, &val);
	ghm->temp_end_dc = (ret == 0) ? val.intval : ghm->temp_start_dc;

	ghm_calculate_and_log(ghm);

	mutex_lock(&ghm->config_lock);
	ghm->session_active = false;
	mutex_unlock(&ghm->config_lock);
}

static void ghm_start_session(struct ggauge_health_monitor *ghm, enum ghm_signal_reason reason)
{
	int ret;
	union power_supply_propval val;
	const char *reason_str =
		(reason == GHM_REASON_START_CHARGING) ? "charge start" : "charge stop";

	if (IS_ERR_OR_NULL(ghm))
		return;

	cancel_delayed_work_sync(&ghm->monitor_stop_work);
	vimon_unregister_callback(ghm->vimon_dev, &ghm->callbacks);

	mutex_lock(&ghm->config_lock);
	ghm->session_active = true;
	mutex_unlock(&ghm->config_lock);

	ghm->session_id++;
	ghm->session_type = reason;

	dev_info(ghm->dev, "session %d start (reason: %s)\n", ghm->session_id, reason_str);

	memset(ghm->dvdt_curve, 0, ghm->bin_count * sizeof(s32));
	memset(ghm->bin_current_sum, 0, ghm->bin_count * sizeof(s64));
	memset(ghm->bin_first_v_uv, 0, ghm->bin_count * sizeof(u64));
	memset(ghm->bin_last_v_uv, 0, ghm->bin_count * sizeof(u64));
	memset(ghm->bin_sample_count, 0, ghm->bin_count * sizeof(u32));

	ghm->v_ewma_state = 0;
	ghm->v_filter_primed = false;
	ghm->raw_sample_cursor = 0;
	ghm->processed_sample_count = 0;

	ret = power_supply_get_property(ghm->bat_psy, POWER_SUPPLY_PROP_TEMP, &val);
	ghm->temp_start_dc = (ret == 0) ? val.intval : -1;

	ghm->bin_duration_ns = (u64)ghm->bin_width_ms * NANO_PER_MILLI;

	ret = vimon_register_callback(
		ghm->vimon_dev, GHM_VIMON_MASK, GHM_VIMON_COUNT, ghm, &ghm->callbacks);
	if (ret) {
		dev_err(ghm->dev, "failed to register VIMON callback (%d)\n", ret);
		mutex_lock(&ghm->config_lock);
		ghm->session_active = false;
		mutex_unlock(&ghm->config_lock);
		return;
	}

	schedule_delayed_work(&ghm->monitor_stop_work, msecs_to_jiffies(ghm->monitor_duration_ms));
}

static void ghm_session_work_func(struct work_struct *work)
{
	struct ggauge_health_monitor *ghm =
		container_of(work, struct ggauge_health_monitor, session_work);
	enum ghm_signal_reason reason;

	if (ghm->last_cc_max > 0)
		reason = GHM_REASON_START_CHARGING;
	else
		reason = GHM_REASON_STOP_CHARGING;

	ghm_start_session(ghm, reason);
}

static ssize_t monitor_config_read(
	struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	struct ggauge_health_monitor *ghm = file->private_data;
	char buf[64];
	int len;

	mutex_lock(&ghm->config_lock);
	len = scnprintf(buf, sizeof(buf), "%u %u %u\n", ghm->monitor_duration_ms, ghm->bin_width_ms,
		ghm->ewma_shift);
	mutex_unlock(&ghm->config_lock);

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static ssize_t monitor_config_write(
	struct file *file, const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct ggauge_health_monitor *ghm = file->private_data;
	char buf[64];
	u32 dur, width, shift;
	u32 bin_cnt;
	struct ggauge_health_monitor temp_ghm = { 0 };
	int ret;
	ssize_t read_bytes;

	read_bytes = simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf, count);
	if (read_bytes < 0)
		return read_bytes;
	buf[read_bytes] = '\0';

	if (sscanf(buf, "%u %u %u", &dur, &width, &shift) != 3)
		return -EINVAL;

	if (dur < GHM_LIMIT_DUR_MIN_MS || dur > GHM_LIMIT_DUR_MAX_MS)
		return -EINVAL;
	if (width < GHM_LIMIT_WIDTH_MIN_MS || width > GHM_LIMIT_WIDTH_MAX_MS)
		return -EINVAL;
	if (shift > GHM_LIMIT_EWMA_SHIFT_MAX)
		return -EINVAL;

	bin_cnt = dur / width;

	mutex_lock(&ghm->config_lock);

	if (ghm->session_active) {
		mutex_unlock(&ghm->config_lock);
		return -EBUSY;
	}

	ret = ghm_alloc_buffers(&temp_ghm, bin_cnt);
	if (ret) {
		mutex_unlock(&ghm->config_lock);
		return ret;
	}

	ghm_free_buffers(ghm);

	ghm->dvdt_curve = temp_ghm.dvdt_curve;
	ghm->bin_current_sum = temp_ghm.bin_current_sum;
	ghm->bin_first_v_uv = temp_ghm.bin_first_v_uv;
	ghm->bin_last_v_uv = temp_ghm.bin_last_v_uv;
	ghm->bin_sample_count = temp_ghm.bin_sample_count;

	ghm->monitor_duration_ms = dur;
	ghm->bin_width_ms = width;
	ghm->ewma_shift = shift;
	ghm->bin_count = bin_cnt;

	mutex_unlock(&ghm->config_lock);

	dev_info(ghm->dev, "Config updated: Dur=%ums Width=%ums Bins=%u\n",
		ghm->monitor_duration_ms, ghm->bin_width_ms, ghm->bin_count);

	return count;
}

static const struct file_operations monitor_config_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = monitor_config_read,
	.write = monitor_config_write,
	.llseek = default_llseek,
};

static int verbose_logging_set(void *data, u64 val)
{
	struct ggauge_health_monitor *ghm = data;
	bool enable = !!val;

	mutex_lock(&ghm->config_lock);

	if (ghm->session_active) {
		mutex_unlock(&ghm->config_lock);
		return -EBUSY;
	}

	if (enable && !ghm->verbose_logging_enabled) {
		u32 alloc_size = GHM_MAX_RAW_SAMPLES;

		ghm->raw_samples =
			kvmalloc_array(alloc_size, sizeof(struct max77779_sample_data), GFP_KERNEL);
		if (!ghm->raw_samples) {
			mutex_unlock(&ghm->config_lock);
			return -ENOMEM;
		}
		ghm->raw_sample_max = alloc_size;
		ghm->verbose_logging_enabled = true;
		dev_info(ghm->dev, "Verbose logging enabled (buf size: %u)\n", alloc_size);

	} else if (!enable && ghm->verbose_logging_enabled) {
		kvfree(ghm->raw_samples);
		ghm->raw_samples = NULL;
		ghm->raw_sample_max = 0;
		ghm->verbose_logging_enabled = false;
		dev_info(ghm->dev, "Verbose logging disabled\n");
	}

	mutex_unlock(&ghm->config_lock);
	return 0;
}

static int verbose_logging_get(void *data, u64 *val)
{
	struct ggauge_health_monitor *ghm = data;

	mutex_lock(&ghm->config_lock);
	*val = ghm->verbose_logging_enabled;
	mutex_unlock(&ghm->config_lock);

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(verbose_logging_fops, verbose_logging_get, verbose_logging_set, "%llu\n");

static int vimon_power_supply_callback(struct notifier_block *nb, unsigned long event, void *data)
{
	struct power_supply *psy = data;
	struct ggauge_health_monitor *ghm = container_of(nb, struct ggauge_health_monitor, psy_nb);
	union power_supply_propval val;
	int ret;
	int current_cc_max;

	if (!ghm->chg_psy || psy != ghm->chg_psy)
		return NOTIFY_DONE;
	if (event != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_DONE;

	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &val);
	if (ret < 0)
		return NOTIFY_DONE;

	current_cc_max = val.intval;

	if ((ghm->last_cc_max > 0) != (current_cc_max > 0)) {
		ghm->last_cc_max = current_cc_max;
		schedule_work(&ghm->session_work);
	} else {
		ghm->last_cc_max = current_cc_max;
	}

	return NOTIFY_OK;
}

static int ggauge_health_monitor_probe(struct platform_device *pdev)
{
	struct ggauge_health_monitor *ghm;
	struct device_node *np;
	const char *chg_psy_name;
	const char *bat_psy_name;
	union power_supply_propval val;
	int ret;

	np = of_parse_phandle(pdev->dev.of_node, GHM_DT_PHANDLE_NAME, 0);
	if (!np)
		return -ENODEV;
	of_node_put(np);

	if (of_property_read_string(pdev->dev.of_node, "google,chg-power-supply", &chg_psy_name) <
			0 ||
		of_property_read_string(
			pdev->dev.of_node, "google,bat-power-supply", &bat_psy_name) < 0)
		return -EINVAL;

	ghm = devm_kzalloc(&pdev->dev, sizeof(*ghm), GFP_KERNEL);
	if (!ghm)
		return -ENOMEM;

	ghm->dev = &pdev->dev;
	platform_set_drvdata(pdev, ghm);
	mutex_init(&ghm->config_lock);

	ghm->monitor_duration_ms = GHM_DEFAULT_DURATION_MS;
	ghm->bin_width_ms = GHM_DEFAULT_BIN_WIDTH_MS;
	ghm->ewma_shift = GHM_DEFAULT_EWMA_SHIFT;
	ghm->ewma_scale = GHM_EWMA_FIXED_SCALE;
	ghm->bin_count = GHM_DEFAULT_DURATION_MS / GHM_DEFAULT_BIN_WIDTH_MS;

	ret = ghm_alloc_buffers(ghm, ghm->bin_count);
	if (ret)
		return ret;

	ghm->vimon_dev = max77779_get_dev(ghm->dev, GHM_DT_PHANDLE_NAME);
	if (!ghm->vimon_dev) {
		ret = -EPROBE_DEFER;
		goto err_free_bufs;
	}

	ghm->chg_psy = power_supply_get_by_name(chg_psy_name);
	if (!ghm->chg_psy) {
		ret = -EPROBE_DEFER;
		goto err_free_bufs;
	}

	ghm->bat_psy = power_supply_get_by_name(bat_psy_name);
	if (!ghm->bat_psy) {
		ret = -EPROBE_DEFER;
		goto err_put_chg;
	}

	ghm->ghm_logbuf = logbuffer_register(GHM_LOGBUFFER_NAME);
	if (!ghm->ghm_logbuf) {
		ret = -ENOMEM;
		goto err_put_bat;
	}

	ghm->callbacks.on_sample_ready = ghm_vimon_sample_ready_cb;
	ghm->callbacks.on_removed = ghm_vimon_removed_cb;
	INIT_DELAYED_WORK(&ghm->monitor_stop_work, ghm_monitor_stop_work_func);
	INIT_WORK(&ghm->session_work, ghm_session_work_func);

	ghm->debugfs_dir = debugfs_create_dir("ggauge_health_monitor", NULL);

	debugfs_create_file("monitor_config", 0600, ghm->debugfs_dir, ghm, &monitor_config_fops);
	debugfs_create_file("verbose_logging", 0600, ghm->debugfs_dir, ghm, &verbose_logging_fops);

	ret = power_supply_get_property(
		ghm->chg_psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &val);
	ghm->last_cc_max = (ret == 0) ? val.intval : 0;

	ghm->psy_nb.notifier_call = vimon_power_supply_callback;
	ret = power_supply_reg_notifier(&ghm->psy_nb);
	if (ret < 0)
		goto err_debugfs;

	dev_info(&pdev->dev, "VIMON health monitor probed\n");
	return 0;

err_debugfs:
	debugfs_remove_recursive(ghm->debugfs_dir);
	logbuffer_unregister(ghm->ghm_logbuf);
err_put_bat:
	power_supply_put(ghm->bat_psy);
err_put_chg:
	power_supply_put(ghm->chg_psy);
err_free_bufs:
	ghm_free_buffers(ghm);
	return ret;
}

static int ggauge_health_monitor_remove(struct platform_device *pdev)
{
	struct ggauge_health_monitor *ghm = platform_get_drvdata(pdev);

	power_supply_unreg_notifier(&ghm->psy_nb);

	cancel_delayed_work_sync(&ghm->monitor_stop_work);
	cancel_work_sync(&ghm->session_work);

	if (ghm->vimon_dev)
		vimon_unregister_callback(ghm->vimon_dev, &ghm->callbacks);

	debugfs_remove_recursive(ghm->debugfs_dir);

	if (ghm->ghm_logbuf)
		logbuffer_unregister(ghm->ghm_logbuf);

	if (ghm->bat_psy)
		power_supply_put(ghm->bat_psy);
	if (ghm->chg_psy)
		power_supply_put(ghm->chg_psy);

	if (ghm->verbose_logging_enabled && ghm->raw_samples)
		kvfree(ghm->raw_samples);

	ghm_free_buffers(ghm);

	return 0;
}

static const struct of_device_id ggauge_health_monitor_match[] = {
	{ .compatible = "google,ggauge_health_monitor" },
	{},
};
MODULE_DEVICE_TABLE(of, ggauge_health_monitor_match);

static struct platform_driver ggauge_health_monitor_driver = {
	.driver = {
		.name = "ggauge_health_monitor",
		.of_match_table = ggauge_health_monitor_match,
	},
	.probe = ggauge_health_monitor_probe,
	.remove = ggauge_health_monitor_remove,
};

module_platform_driver(ggauge_health_monitor_driver);

MODULE_DESCRIPTION("VIMON Health Monitor Driver");
MODULE_AUTHOR("Mikolaj Wronski <mwronski@google.com>");
MODULE_LICENSE("GPL");
