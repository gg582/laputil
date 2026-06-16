// SPDX-License-Identifier: GPL-2.0-only
/*
 * drivers/cpufreq/cpufreq_laputil.c
 * laputil: laptop-oriented conservative governor (with powersave_bias)
 *
 * Copyright (C) 2025 Lee Yunjin <gzblues61@daum.net>
 *
 * Conservative-style governor:
 * - periodically sample CPU idle vs wall time per CPU in a policy
 * - compute average load across policy CPUs
 * - if load > up_threshold (effective) -> increase freq by freq_step (% of max)
 * - if load < down_threshold (effective) -> after sampling_down_factor cycles, decrease freq
 * - powersave_bias shifts effective thresholds to bias toward power or perf
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kernel_stat.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/cpufreq.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/cpumask.h>
#include <linux/cpu.h>
#include <linux/tick.h>
#include <linux/mutex.h>
#include <linux/sched/cpufreq.h>
#include <linux/errno.h>
#include <linux/string.h>
#include "include/ac_names_gen.h"

struct lap_cpu_dbs {
	u64 prev_cpu_idle;
	u64 prev_cpu_nice;
	u64 prev_update_time;
};

DEFINE_PER_CPU(struct lap_cpu_dbs, lap_cpu_dbs);

struct lap_tuners {
	unsigned int down_threshold;	/* percent (0..100) */
	unsigned int freq_step;		/* percent (of policy->max) */
	unsigned int up_threshold;	/* percent (0..100) */
	unsigned int sampling_down_factor;	/* how many sampling intervals to wait before down */
	unsigned int ignore_nice_load;	/* boolean (preserved but unused here) */
	unsigned int sampling_rate;	/* seconds between samples */
	int powersave_bias;		/* 0..10 base bias, plus dynamic adjustments */
	_Bool last_on_ac;
	int last_battery_capacity;
	/* newly exposed tunables */
	unsigned int ema_alpha_scaling_factor;
	unsigned int system_idle_threshold;
	int powersave_bias_ac;
	int powersave_bias_low_battery;
	int powersave_bias_medium_battery;
	unsigned int min_freq_step_percent;
	unsigned int max_freq_step_percent;
};

struct lap_policy_info {
	unsigned int requested_freq;
	unsigned int prev_load;
	unsigned int idle_periods;
	unsigned int smoothed_load;
	struct lap_tuners tuners;
	struct delayed_work work;
	struct mutex lock;
	struct cpumask eff_mask;	/* Efficiency cores mask */
	struct cpumask perf_mask;	/* Performance cores mask */
	struct cpufreq_policy *policy;
	bool clusters_initialized;
};

#define LAP_DEF_UP_THRESHOLD		70
#define LAP_DEF_DOWN_THRESHOLD		10
#define LAP_DEF_FREQ_STEP		5
#define LAP_DEF_SAMPLING_DOWN_FAC	2
#define LAP_MAX_SAMPLING_DOWN_FAC	5
#define LAP_DEF_SAMPLING_RATE		1
#define LAP_POWERSAVE_BIAS_MIN		0
#define LAP_POWERSAVE_BIAS_MAX		10
#define LAP_DEF_POWERSAVE_BIAS_DEFAULT	1
#define LAP_DEF_EMA_ALPHA_SCALING_FACTOR	3
#define LAP_DEF_SYSTEM_IDLE_THRESHOLD		5
#define LAP_DEF_MIN_FREQ_STEP_PERCENT		5
#define LAP_DEF_MAX_FREQ_STEP_PERCENT		25

/* Detect efficiency and performance cores based on max frequency */
static void detect_clusters(struct cpufreq_policy *policy, struct cpumask *eff_mask,
			    struct cpumask *perf_mask)
{
	unsigned int cpu;
	unsigned int min_freq = UINT_MAX, max_freq = 0;
	unsigned int current_max_freq;

	/* Pass 1: Find the absolute min and max frequencies in the policy */
	for_each_cpu(cpu, policy->cpus) {
		current_max_freq = cpufreq_quick_get_max(cpu);
		if (current_max_freq < min_freq)
			min_freq = current_max_freq;
		if (current_max_freq > max_freq)
			max_freq = current_max_freq;
	}

	cpumask_clear(eff_mask);
	cpumask_clear(perf_mask);

	/* Handle the edge case where all cores have the same frequency */
	if (min_freq == max_freq) {
		cpumask_copy(perf_mask, policy->cpus);
		pr_info("Detected single cluster (%u cores, max_freq: %u kHz)\n",
			cpumask_weight(perf_mask), max_freq);
		return;
	}

	/* Pass 2: Populate masks based on the found min and max frequencies */
	for_each_cpu(cpu, policy->cpus) {
		current_max_freq = cpufreq_quick_get_max(cpu);
		if (current_max_freq == min_freq)
			cpumask_set_cpu(cpu, eff_mask);
		else if (current_max_freq == max_freq)
			cpumask_set_cpu(cpu, perf_mask);
	}

	pr_info("Detected %u efficiency cores (max_freq: %u kHz), %u performance cores (max_freq: %u kHz)\n",
		cpumask_weight(eff_mask), min_freq, cpumask_weight(perf_mask), max_freq);
}

/* Compute freq step in kHz from percent of policy->max */
static inline unsigned int lap_get_freq_step_khz(struct lap_tuners *tuners,
						 struct cpufreq_policy *policy)
{
	unsigned int step_khz;

	step_khz = (tuners->freq_step * policy->max) / 100;

	if (unlikely(step_khz == 0))
		step_khz = (tuners->min_freq_step_percent * policy->max) / 100;

	if (unlikely(step_khz == 0))
		step_khz = 1;

	return step_khz;
}

static void lap_snapshot_tuners(struct lap_policy_info *lp, struct lap_tuners *tuners)
{
	tuners->down_threshold = READ_ONCE(lp->tuners.down_threshold);
	tuners->freq_step = READ_ONCE(lp->tuners.freq_step);
	tuners->up_threshold = READ_ONCE(lp->tuners.up_threshold);
	tuners->sampling_down_factor = READ_ONCE(lp->tuners.sampling_down_factor);
	tuners->ignore_nice_load = READ_ONCE(lp->tuners.ignore_nice_load);
	tuners->sampling_rate = READ_ONCE(lp->tuners.sampling_rate);
	tuners->powersave_bias = READ_ONCE(lp->tuners.powersave_bias);
	tuners->last_on_ac = READ_ONCE(lp->tuners.last_on_ac);
	tuners->last_battery_capacity = READ_ONCE(lp->tuners.last_battery_capacity);
	tuners->ema_alpha_scaling_factor = READ_ONCE(lp->tuners.ema_alpha_scaling_factor);
	tuners->system_idle_threshold = READ_ONCE(lp->tuners.system_idle_threshold);
	tuners->powersave_bias_ac = READ_ONCE(lp->tuners.powersave_bias_ac);
	tuners->powersave_bias_low_battery = READ_ONCE(lp->tuners.powersave_bias_low_battery);
	tuners->powersave_bias_medium_battery = READ_ONCE(lp->tuners.powersave_bias_medium_battery);
	tuners->min_freq_step_percent = READ_ONCE(lp->tuners.min_freq_step_percent);
	tuners->max_freq_step_percent = READ_ONCE(lp->tuners.max_freq_step_percent);
}

/* lap_is_on_ac - Return 1 if system is on AC power, 0 otherwise. */
static inline _Bool lap_is_on_ac(int *battery_capacity)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int i;

	*battery_capacity = 50; /* Default to 50% capacity if status unknown */

	/* Optimization: Check AC status first.
	 * Returns immediately if any AC source is online.
	 */
	for (i = 0; i < ARRAY_SIZE(ac_names) && ac_names[i] != NULL; i++) {
		psy = power_supply_get_by_name(ac_names[i]);
		if (!psy)
			continue;
		if (!power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE, &val) && val.intval) {
			power_supply_put(psy);
			return 1;
		}
		power_supply_put(psy);
	}

	/* Check battery capacity only if AC is offline */
	for (i = 0; i < ARRAY_SIZE(battery_names) && battery_names[i] != NULL; i++) {
		psy = power_supply_get_by_name(battery_names[i]);
		if (!psy)
			continue;
		if (!power_supply_get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &val))
			*battery_capacity = val.intval;
		power_supply_put(psy);
		return 0; /* Assume battery mode if battery device exists */
	}

	return 0;
}

/* lap_dbs_update - compute average load (0..100) across all CPUs in policy */
static unsigned int lap_dbs_update(struct cpufreq_policy *policy, bool ignore_nice,
				   bool on_ac, int battery_capacity)
{
	struct lap_policy_info *lp = policy->governor_data;
	unsigned int load_sum = 0, eff_load_sum = 0, perf_load_sum = 0;
	unsigned int cpu;
	unsigned int eff_cpus = 0, perf_cpus = 0;
	u64 cur_time;
	unsigned int time_elapsed;
	unsigned int cur_load;
	u64 cur_idle, cur_nice;
	u64 idle_delta, nice_delta;

	if (!lp || !cpumask_weight(policy->cpus))
		return 0;

	/* Initialize core masks if not already done */
	if (!READ_ONCE(lp->clusters_initialized)) {
		detect_clusters(policy, &lp->eff_mask, &lp->perf_mask);
		WRITE_ONCE(lp->clusters_initialized, true);
		eff_cpus = cpumask_weight(&lp->eff_mask);
		perf_cpus = cpumask_weight(&lp->perf_mask);
	} else {
		eff_cpus = cpumask_weight(&lp->eff_mask);
		perf_cpus = cpumask_weight(&lp->perf_mask);
	}

	/* Compute load for efficiency and performance cores separately */
	for_each_cpu(cpu, policy->cpus) {
		struct lap_cpu_dbs *cdbs = per_cpu_ptr(&lap_cpu_dbs, cpu);

		cur_idle = get_cpu_idle_time_us(cpu, &cur_time);
		if (cur_idle == (u64)-1) {
			/* idle time accounting not supported on this arch/cpu */
			cdbs->prev_update_time = cur_time;
			cdbs->prev_cpu_idle = 0;
			cdbs->prev_cpu_nice = 0;
			continue;
		}
		cur_nice = jiffies_to_usecs(kcpustat_cpu(cpu).cpustat[CPUTIME_NICE]);

		time_elapsed = (unsigned int)(cur_time - cdbs->prev_update_time);
		idle_delta = (unsigned int)(cur_idle - cdbs->prev_cpu_idle);
		nice_delta = (unsigned int)(cur_nice - cdbs->prev_cpu_nice);

		if (unlikely(time_elapsed == 0)) {
			cur_load = 100;
		} else {
			int busy_time = time_elapsed - idle_delta;

			if (ignore_nice)
				busy_time -= nice_delta;
			if (busy_time < 0)
				busy_time = 0;
			cur_load = 100 * busy_time / time_elapsed;
		}

		cdbs->prev_cpu_idle = cur_idle;
		cdbs->prev_cpu_nice = cur_nice;
		cdbs->prev_update_time = cur_time;

		if (cpumask_test_cpu(cpu, &lp->eff_mask))
			eff_load_sum += cur_load;
		else if (cpumask_test_cpu(cpu, &lp->perf_mask))
			perf_load_sum += cur_load;
		load_sum += cur_load;
	}

	/* Calculate average load based on battery capacity */
	if (eff_cpus && perf_cpus) {
		unsigned int eff_load = eff_load_sum / eff_cpus;
		unsigned int perf_load = perf_load_sum / perf_cpus;

		if (!on_ac && battery_capacity <= 20)
			return eff_load;
		else
			return (eff_load * 6 + perf_load * 4) / 10;
	}

	/* Fallback to average load across all CPUs */
	return load_sum / cpumask_weight(policy->cpus);
}

/* cs_dbs_update - apply Laputil decision logic for one policy */
static unsigned long cs_dbs_update(struct cpufreq_policy *policy)
{
	struct lap_policy_info *lp = policy->governor_data;
	struct lap_tuners tuners;
	unsigned int requested_freq;
	unsigned int load;
	unsigned int step_khz;
	unsigned long next_delay;
	int bias;
	int eff_up, eff_down;
	int battery_capacity;
	bool on_ac;
	bool refresh_power;
	bool should_increase = false;
	bool should_decrease = false;
	int load_delta;
	u8 ema_alpha;

	if (!lp)
		return HZ;

	lap_snapshot_tuners(lp, &tuners);

	mutex_lock(&lp->lock);
	requested_freq = lp->requested_freq;
	refresh_power = (lp->idle_periods % 5 == 0);
	mutex_unlock(&lp->lock);

	/*
	 * Avoid holding lp->lock across power-supply lookup, CPU load sampling,
	 * or cpufreq transitions. Otherwise sysfs show/store handlers can block
	 * indefinitely behind the governor worker.
	 */
	if (refresh_power)
		on_ac = lap_is_on_ac(&battery_capacity);
	else {
		on_ac = READ_ONCE(lp->tuners.last_on_ac);
        battery_capacity = READ_ONCE(lp->tuners.last_battery_capacity);
    }

	load = lap_dbs_update(policy, tuners.ignore_nice_load, on_ac, battery_capacity);

	if (load < tuners.system_idle_threshold &&
	    lp->smoothed_load < tuners.system_idle_threshold)
		next_delay = (unsigned long)tuners.sampling_rate * HZ * 2;
	else
		next_delay = (unsigned long)tuners.sampling_rate * HZ;

	load_delta = (int)load - (int)lp->prev_load;
	ema_alpha = (load_delta + 100) / tuners.ema_alpha_scaling_factor;
	if (ema_alpha < 30)
		ema_alpha = 30;

	if (requested_freq > policy->max || requested_freq < policy->min)
		requested_freq = policy->cur;

	step_khz = lap_get_freq_step_khz(&tuners, policy);

	bias = tuners.powersave_bias;
	if (on_ac) {
		bias += tuners.powersave_bias_ac;
	} else {
		if (battery_capacity <= 20)
			bias += tuners.powersave_bias_low_battery;
		else if (battery_capacity <= 50)
			bias += tuners.powersave_bias_medium_battery;
	}

	eff_up = (int)tuners.up_threshold + bias;
	eff_down = (int)tuners.down_threshold + bias;

	if (eff_up > 100)
		eff_up = 100;
	if (eff_up < 1)
		eff_up = 1;
	if (eff_down < 0)
		eff_down = 0;
	if (eff_down >= eff_up) {
		eff_down = eff_up - 1;
		if (eff_down < 0)
			eff_down = 0;
	}

	mutex_lock(&lp->lock);
	{
		unsigned int idle_periods = lp->idle_periods;
		unsigned int smoothed_load = lp->smoothed_load;

		smoothed_load = (ema_alpha * load + (100 - ema_alpha) * smoothed_load) / 100;

		if (smoothed_load > (unsigned int)eff_up) {
			if (requested_freq != policy->max) {
				requested_freq += step_khz;
				if (requested_freq > policy->max)
					requested_freq = policy->max;
				should_increase = true;
			}
			idle_periods = 0;
		} else if (load < (unsigned int)eff_down) {
			idle_periods++;
			if (idle_periods >= tuners.sampling_down_factor) {
				if (requested_freq != policy->min) {
					if (requested_freq > step_khz)
						requested_freq -= step_khz;
					else
						requested_freq = policy->min;
					should_decrease = true;
				}
				idle_periods = 0;
			}
		} else {
			idle_periods = 0;
		}

		lp->prev_load = load;
		lp->requested_freq = requested_freq;
		lp->idle_periods = idle_periods;
		lp->smoothed_load = smoothed_load;

		if (refresh_power) {
			lp->tuners.last_on_ac = on_ac;
			lp->tuners.last_battery_capacity = battery_capacity;
		}
	}
	mutex_unlock(&lp->lock);

	if (should_increase)
		cpufreq_driver_target(policy, requested_freq, CPUFREQ_RELATION_H);
	else if (should_decrease)
		cpufreq_driver_target(policy, requested_freq, CPUFREQ_RELATION_L);

	return next_delay;
}

/* Delayed work handler per policy */
static void lap_work_handler(struct work_struct *work)
{
	struct lap_policy_info *lp = container_of(work, struct lap_policy_info, work.work);
	struct cpufreq_policy *policy = lp->policy;
	unsigned long delay_jiffies;

	delay_jiffies = cs_dbs_update(policy);
	schedule_delayed_work_on(policy->cpu, &lp->work, delay_jiffies);
}

/* sysfs interface */
static struct lap_policy_info *lap_policy_from_kobj(struct kobject *kobj)
{
	struct cpufreq_policy *policy;

	policy = container_of(kobj, struct cpufreq_policy, kobj);
	return policy->governor_data;
}

static ssize_t lap_emit_uint(char *buf, unsigned int value)
{
	return sysfs_emit(buf, "%u\n", value);
}

static ssize_t lap_emit_int(char *buf, int value)
{
	return sysfs_emit(buf, "%d\n", value);
}

#define lap_gov_attr(_name) \
static struct kobj_attribute _name##_attr = { \
	.attr = { .name = #_name, .mode = 0644 }, \
	.show = show_##_name, \
	.store = store_##_name, \
}

/* --- sampling_rate --- */
static ssize_t show_sampling_rate(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lap_policy_info *lp = lap_policy_from_kobj(kobj);

	if (!lp)
		return lap_emit_uint(buf, 0);

	return lap_emit_uint(buf, READ_ONCE(lp->tuners.sampling_rate));
}

static ssize_t store_sampling_rate(struct kobject *kobj, struct kobj_attribute *attr,
				   const char *buf, size_t count)
{
	unsigned int val;
	int ret;
	struct lap_policy_info *lp = lap_policy_from_kobj(kobj);

	if (!lp)
		return -EINVAL;

	ret = kstrtouint(buf, 10, &val);
	if (ret || val == 0)
		return -EINVAL;

	mutex_lock(&lp->lock);
	lp->tuners.sampling_rate = val;
	mutex_unlock(&lp->lock);
	return count;
}

/* --- sampling_down_factor --- */
static ssize_t show_sampling_down_factor(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lap_policy_info *lp = lap_policy_from_kobj(kobj);

	if (!lp)
		return lap_emit_uint(buf, 0);

	return lap_emit_uint(buf, READ_ONCE(lp->tuners.sampling_down_factor));
}

static ssize_t store_sampling_down_factor(struct kobject *kobj, struct kobj_attribute *attr,
					  const char *buf, size_t count)
{
	unsigned int val;
	int ret;
	struct lap_policy_info *lp = lap_policy_from_kobj(kobj);

	if (!lp)
		return -EINVAL;

	ret = kstrtouint(buf, 10, &val);
	if (ret || val < 1 || val > LAP_MAX_SAMPLING_DOWN_FAC)
		return -EINVAL;

	mutex_lock(&lp->lock);
	lp->tuners.sampling_down_factor = val;
	mutex_unlock(&lp->lock);
	return count;
}

/* --- up_threshold --- */
static ssize_t show_up_threshold(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lap_policy_info *lp = lap_policy_from_kobj(kobj);

	if (!lp)
		return lap_emit_uint(buf, 0);

	return lap_emit_uint(buf, READ_ONCE(lp->tuners.up_threshold));
}

static ssize_t store_up_threshold(struct kobject *kobj, struct kobj_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned int val;
	int ret;
	struct lap_policy_info *lp = lap_policy_from_kobj(kobj);

	if (!lp)
		return -EINVAL;

	ret = kstrtouint(buf, 10, &val);
	if (ret || val > 100)
		return -EINVAL;

	mutex_lock(&lp->lock);
	if (val <= lp->tuners.down_threshold) {
		mutex_unlock(&lp->lock);
		return -EINVAL;
	}
	lp->tuners.up_threshold = val;
	mutex_unlock(&lp->lock);
	return count;
}

/* --- down_threshold --- */
static ssize_t show_down_threshold(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lap_policy_info *lp = lap_policy_from_kobj(kobj);

	if (!lp)
		return lap_emit_uint(buf, 0);

	return lap_emit_uint(buf, READ_ONCE(lp->tuners.down_threshold));
}

static ssize_t store_down_threshold(struct kobject *kobj, struct kobj_attribute *attr,
				    const char *buf, size_t count)
{
	unsigned int val;
	int ret;
	struct lap_policy_info *lp = lap_policy_from_kobj(kobj);

	if (!lp)
		return -EINVAL;

	ret = kstrtouint(buf, 10, &val);
	if (ret || val > 100)
		return -EINVAL;

	mutex_lock(&lp->lock);
	if (val >= lp->tuners.up_threshold) {
		mutex_unlock(&lp->lock);
		return -EINVAL;
	}
	lp->tuners.down_threshold = val;
	mutex_unlock(&lp->lock);
	return count;
}

/* --- ignore_nice_load --- */
static ssize_t show_ignore_nice_load(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lap_policy_info *lp = lap_policy_from_kobj(kobj);

	if (!lp)
		return lap_emit_uint(buf, 0);

	return lap_emit_uint(buf, READ_ONCE(lp->tuners.ignore_nice_load));
}

static ssize_t store_ignore_nice_load(struct kobject *kobj, struct kobj_attribute *attr,
				      const char *buf, size_t count)
{
	unsigned int val;
	int ret;
	struct lap_policy_info *lp = lap_policy_from_kobj(kobj);

	if (!lp)
		return -EINVAL;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return -EINVAL;

	mutex_lock(&lp->lock);
	lp->tuners.ignore_nice_load = !!val;
	mutex_unlock(&lp->lock);
	return count;
}

/* --- freq_step --- */
static ssize_t show_freq_step(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lap_policy_info *lp = lap_policy_from_kobj(kobj);

	if (!lp)
		return lap_emit_uint(buf, 0);

	return lap_emit_uint(buf, READ_ONCE(lp->tuners.freq_step));
}

static ssize_t store_freq_step(struct kobject *kobj, struct kobj_attribute *attr,
			       const char *buf, size_t count)
{
	unsigned int val;
	int ret;
	struct lap_policy_info *lp = lap_policy_from_kobj(kobj);

	if (!lp)
		return -EINVAL;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return -EINVAL;

	mutex_lock(&lp->lock);
	if (val > lp->tuners.max_freq_step_percent || val < lp->tuners.min_freq_step_percent) {
		mutex_unlock(&lp->lock);
		return -EINVAL;
	}
	lp->tuners.freq_step = val;
	mutex_unlock(&lp->lock);
	return count;
}

/* --- powersave_bias --- */
static ssize_t show_powersave_bias(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lap_policy_info *lp = lap_policy_from_kobj(kobj);

	if (!lp)
		return lap_emit_int(buf, 0);

	return lap_emit_int(buf, READ_ONCE(lp->tuners.powersave_bias));
}

static ssize_t store_powersave_bias(struct kobject *kobj, struct kobj_attribute *attr,
				    const char *buf, size_t count)
{
	int val;
	int ret;
	struct lap_policy_info *lp = lap_policy_from_kobj(kobj);

	if (!lp)
		return -EINVAL;

	ret = kstrtoint(buf, 10, &val);
	if (ret)
		return -EINVAL;

	if (val < LAP_POWERSAVE_BIAS_MIN)
		val = LAP_POWERSAVE_BIAS_MIN;
	if (val > LAP_POWERSAVE_BIAS_MAX)
		val = LAP_POWERSAVE_BIAS_MAX;

	mutex_lock(&lp->lock);
	lp->tuners.powersave_bias = val;
	mutex_unlock(&lp->lock);
	return count;
}

/* --- ema_alpha_scaling_factor --- */
static ssize_t show_ema_alpha_scaling_factor(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lap_policy_info *lp = lap_policy_from_kobj(kobj);

	if (!lp)
		return lap_emit_uint(buf, 0);

	return lap_emit_uint(buf, READ_ONCE(lp->tuners.ema_alpha_scaling_factor));
}

static ssize_t store_ema_alpha_scaling_factor(struct kobject *kobj, struct kobj_attribute *attr,
					      const char *buf, size_t count)
{
	unsigned int val;
	int ret;
	struct lap_policy_info *lp = lap_policy_from_kobj(kobj);

	if (!lp)
		return -EINVAL;

	ret = kstrtouint(buf, 10, &val);
	if (ret || val < 1 || val > 100)
		return -EINVAL;

	mutex_lock(&lp->lock);
	lp->tuners.ema_alpha_scaling_factor = val;
	mutex_unlock(&lp->lock);
	return count;
}

/* --- system_idle_threshold --- */
static ssize_t show_system_idle_threshold(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lap_policy_info *lp = lap_policy_from_kobj(kobj);

	if (!lp)
		return lap_emit_uint(buf, 0);

	return lap_emit_uint(buf, READ_ONCE(lp->tuners.system_idle_threshold));
}

static ssize_t store_system_idle_threshold(struct kobject *kobj, struct kobj_attribute *attr,
					   const char *buf, size_t count)
{
	unsigned int val;
	int ret;
	struct lap_policy_info *lp = lap_policy_from_kobj(kobj);

	if (!lp)
		return -EINVAL;

	ret = kstrtouint(buf, 10, &val);
	if (ret || val > 100)
		return -EINVAL;

	mutex_lock(&lp->lock);
	lp->tuners.system_idle_threshold = val;
	mutex_unlock(&lp->lock);
	return count;
}

/* --- powersave_bias_ac --- */
static ssize_t show_powersave_bias_ac(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lap_policy_info *lp = lap_policy_from_kobj(kobj);

	if (!lp)
		return lap_emit_int(buf, 0);

	return lap_emit_int(buf, READ_ONCE(lp->tuners.powersave_bias_ac));
}

static ssize_t store_powersave_bias_ac(struct kobject *kobj, struct kobj_attribute *attr,
				       const char *buf, size_t count)
{
	int val;
	int ret;
	struct lap_policy_info *lp = lap_policy_from_kobj(kobj);

	if (!lp)
		return -EINVAL;

	ret = kstrtoint(buf, 10, &val);
	if (ret)
		return -EINVAL;
	if (val < LAP_POWERSAVE_BIAS_MIN)
		val = LAP_POWERSAVE_BIAS_MIN;
	if (val > LAP_POWERSAVE_BIAS_MAX)
		val = LAP_POWERSAVE_BIAS_MAX;

	mutex_lock(&lp->lock);
	lp->tuners.powersave_bias_ac = val;
	mutex_unlock(&lp->lock);
	return count;
}

/* --- powersave_bias_low_battery --- */
static ssize_t show_powersave_bias_low_battery(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lap_policy_info *lp = lap_policy_from_kobj(kobj);

	if (!lp)
		return lap_emit_int(buf, 0);

	return lap_emit_int(buf, READ_ONCE(lp->tuners.powersave_bias_low_battery));
}

static ssize_t store_powersave_bias_low_battery(struct kobject *kobj, struct kobj_attribute *attr,
						const char *buf, size_t count)
{
	int val;
	int ret;
	struct lap_policy_info *lp = lap_policy_from_kobj(kobj);

	if (!lp)
		return -EINVAL;

	ret = kstrtoint(buf, 10, &val);
	if (ret)
		return -EINVAL;
	if (val < LAP_POWERSAVE_BIAS_MIN)
		val = LAP_POWERSAVE_BIAS_MIN;
	if (val > LAP_POWERSAVE_BIAS_MAX)
		val = LAP_POWERSAVE_BIAS_MAX;

	mutex_lock(&lp->lock);
	lp->tuners.powersave_bias_low_battery = val;
	mutex_unlock(&lp->lock);
	return count;
}

/* --- powersave_bias_medium_battery --- */
static ssize_t show_powersave_bias_medium_battery(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lap_policy_info *lp = lap_policy_from_kobj(kobj);

	if (!lp)
		return lap_emit_int(buf, 0);

	return lap_emit_int(buf, READ_ONCE(lp->tuners.powersave_bias_medium_battery));
}

static ssize_t store_powersave_bias_medium_battery(struct kobject *kobj, struct kobj_attribute *attr,
						   const char *buf, size_t count)
{
	int val;
	int ret;
	struct lap_policy_info *lp = lap_policy_from_kobj(kobj);

	if (!lp)
		return -EINVAL;

	ret = kstrtoint(buf, 10, &val);
	if (ret)
		return -EINVAL;
	if (val < LAP_POWERSAVE_BIAS_MIN)
		val = LAP_POWERSAVE_BIAS_MIN;
	if (val > LAP_POWERSAVE_BIAS_MAX)
		val = LAP_POWERSAVE_BIAS_MAX;

	mutex_lock(&lp->lock);
	lp->tuners.powersave_bias_medium_battery = val;
	mutex_unlock(&lp->lock);
	return count;
}

/* --- min_freq_step_percent --- */
static ssize_t show_min_freq_step_percent(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lap_policy_info *lp = lap_policy_from_kobj(kobj);

	if (!lp)
		return lap_emit_uint(buf, 0);

	return lap_emit_uint(buf, READ_ONCE(lp->tuners.min_freq_step_percent));
}

static ssize_t store_min_freq_step_percent(struct kobject *kobj, struct kobj_attribute *attr,
					   const char *buf, size_t count)
{
	unsigned int val;
	int ret;
	struct lap_policy_info *lp = lap_policy_from_kobj(kobj);

	if (!lp)
		return -EINVAL;

	ret = kstrtouint(buf, 10, &val);
	if (ret || val < 1 || val > 100)
		return -EINVAL;

	mutex_lock(&lp->lock);
	if (val >= lp->tuners.max_freq_step_percent) {
		mutex_unlock(&lp->lock);
		return -EINVAL;
	}
	lp->tuners.min_freq_step_percent = val;
	mutex_unlock(&lp->lock);
	return count;
}

/* --- max_freq_step_percent --- */
static ssize_t show_max_freq_step_percent(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lap_policy_info *lp = lap_policy_from_kobj(kobj);

	if (!lp)
		return lap_emit_uint(buf, 0);

	return lap_emit_uint(buf, READ_ONCE(lp->tuners.max_freq_step_percent));
}

static ssize_t store_max_freq_step_percent(struct kobject *kobj, struct kobj_attribute *attr,
					   const char *buf, size_t count)
{
	unsigned int val;
	int ret;
	struct lap_policy_info *lp = lap_policy_from_kobj(kobj);

	if (!lp)
		return -EINVAL;

	ret = kstrtouint(buf, 10, &val);
	if (ret || val < 1 || val > 100)
		return -EINVAL;

	mutex_lock(&lp->lock);
	if (val <= lp->tuners.min_freq_step_percent) {
		mutex_unlock(&lp->lock);
		return -EINVAL;
	}
	lp->tuners.max_freq_step_percent = val;
	mutex_unlock(&lp->lock);
	return count;
}

/* declare attributes */
lap_gov_attr(sampling_rate);
lap_gov_attr(sampling_down_factor);
lap_gov_attr(up_threshold);
lap_gov_attr(down_threshold);
lap_gov_attr(ignore_nice_load);
lap_gov_attr(freq_step);
lap_gov_attr(powersave_bias);
lap_gov_attr(ema_alpha_scaling_factor);
lap_gov_attr(system_idle_threshold);
lap_gov_attr(powersave_bias_ac);
lap_gov_attr(powersave_bias_low_battery);
lap_gov_attr(powersave_bias_medium_battery);
lap_gov_attr(min_freq_step_percent);
lap_gov_attr(max_freq_step_percent);

static struct attribute *lap_attrs[] = {
	&sampling_rate_attr.attr,
	&sampling_down_factor_attr.attr,
	&up_threshold_attr.attr,
	&down_threshold_attr.attr,
	&ignore_nice_load_attr.attr,
	&freq_step_attr.attr,
	&powersave_bias_attr.attr,
	&ema_alpha_scaling_factor_attr.attr,
	&system_idle_threshold_attr.attr,
	&powersave_bias_ac_attr.attr,
	&powersave_bias_low_battery_attr.attr,
	&powersave_bias_medium_battery_attr.attr,
	&min_freq_step_percent_attr.attr,
	&max_freq_step_percent_attr.attr,
	NULL
};

static struct attribute_group lap_attr_group = {
	.attrs = lap_attrs,
};

static int lap_start(struct cpufreq_policy *policy)
{
	struct lap_policy_info *lp = policy->governor_data;
	unsigned long delay;
	unsigned int cpu;

	if (!lp)
		return -EINVAL;

	for_each_cpu(cpu, policy->cpus) {
		struct lap_cpu_dbs *cdbs = per_cpu_ptr(&lap_cpu_dbs, cpu);
		u64 cur_time;
		u64 cur_idle;
		u64 cur_nice;

		cur_idle = get_cpu_idle_time_us(cpu, &cur_time);
		cur_nice = jiffies_to_usecs(kcpustat_cpu(cpu).cpustat[CPUTIME_NICE]);

		cdbs->prev_cpu_idle = cur_idle;
		cdbs->prev_cpu_nice = cur_nice;
		cdbs->prev_update_time = cur_time;
	}

	mutex_lock(&lp->lock);
	lp->requested_freq = policy->cur;
	lp->prev_load = 0;
	lp->idle_periods = 0;
	lp->smoothed_load = 0;
	mutex_unlock(&lp->lock);

	delay = (unsigned long)READ_ONCE(lp->tuners.sampling_rate) * HZ;
	schedule_delayed_work_on(policy->cpu, &lp->work, delay);
	return 0;
}

static void lap_stop(struct cpufreq_policy *policy)
{
	struct lap_policy_info *lp = policy->governor_data;

	if (!lp)
		return;
	cancel_delayed_work_sync(&lp->work);
}

static void lap_limits(struct cpufreq_policy *policy)
{
	struct lap_policy_info *lp = policy->governor_data;

	if (!lp)
		return;

	mutex_lock(&lp->lock);
	if (lp->requested_freq > policy->max)
		lp->requested_freq = policy->max;
	if (lp->requested_freq < policy->min)
		lp->requested_freq = policy->min;
	mutex_unlock(&lp->lock);
}

static int lap_init(struct cpufreq_policy *policy)
{
	struct lap_policy_info *lp;

	lp = kzalloc(sizeof(*lp), GFP_KERNEL);
	if (!lp)
		return -ENOMEM;

	lp->tuners.up_threshold = LAP_DEF_UP_THRESHOLD;
	lp->tuners.down_threshold = LAP_DEF_DOWN_THRESHOLD;
	lp->tuners.freq_step = LAP_DEF_FREQ_STEP;
	lp->tuners.sampling_down_factor = LAP_DEF_SAMPLING_DOWN_FAC;
	lp->tuners.ignore_nice_load = 1;
	lp->tuners.sampling_rate = LAP_DEF_SAMPLING_RATE;
	lp->tuners.powersave_bias = 0;
	lp->tuners.ema_alpha_scaling_factor = LAP_DEF_EMA_ALPHA_SCALING_FACTOR;
	lp->tuners.system_idle_threshold = LAP_DEF_SYSTEM_IDLE_THRESHOLD;
	lp->tuners.powersave_bias_ac = LAP_DEF_POWERSAVE_BIAS_DEFAULT;
	lp->tuners.powersave_bias_low_battery = 10;
	lp->tuners.powersave_bias_medium_battery = 5;
	lp->tuners.min_freq_step_percent = LAP_DEF_MIN_FREQ_STEP_PERCENT;
	lp->tuners.max_freq_step_percent = LAP_DEF_MAX_FREQ_STEP_PERCENT;

	lp->policy = policy;
	mutex_init(&lp->lock);
	lp->clusters_initialized = false;
	cpumask_clear(&lp->eff_mask);
	cpumask_clear(&lp->perf_mask);
	INIT_DEFERRABLE_WORK(&lp->work, lap_work_handler);

	policy->governor_data = lp;

	if (sysfs_create_group(&policy->kobj, &lap_attr_group)) {
		kfree(lp);
		policy->governor_data = NULL;
		return -EINVAL;
	}
	return 0;
}

static void lap_exit(struct cpufreq_policy *policy)
{
	struct lap_policy_info *lp = policy->governor_data;

	if (!lp)
		return;

	cancel_delayed_work_sync(&lp->work);
	sysfs_remove_group(&policy->kobj, &lap_attr_group);
	kfree(lp);
	policy->governor_data = NULL;
}

static struct cpufreq_governor cpufreq_gov_laputil = {
	.name = "laputil",
	.owner = THIS_MODULE,
	.flags = CPUFREQ_GOV_DYNAMIC_SWITCHING,
	.start = lap_start,
	.stop = lap_stop,
	.init = lap_init,
	.exit = lap_exit,
	.limits = lap_limits,
};

static int __init laputil_module_init(void)
{
	return cpufreq_register_governor(&cpufreq_gov_laputil);
}

static void __exit laputil_module_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_laputil);
}

MODULE_AUTHOR("Lee Yunjin <gzblues61@daum.net>");
MODULE_DESCRIPTION("'cpufreq_laputil' - Conservative-style governor for laptops (with powersave_bias)");
MODULE_LICENSE("GPL");

module_init(laputil_module_init);
module_exit(laputil_module_exit);
