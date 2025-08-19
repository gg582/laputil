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
/*
 * Per-CPU previous state for idle accounting.
 */

struct lap_cpu_dbs {
    u64 prev_cpu_idle;
    u64 prev_cpu_nice;
    u64 prev_update_time;
};

DEFINE_PER_CPU(struct lap_cpu_dbs, lap_cpu_dbs);

/* Tunables */
struct lap_tuners {
    unsigned int down_threshold;       /* percent (0..100) */
    unsigned int freq_step;            /* percent (of policy->max) */
    unsigned int up_threshold;         /* percent (0..100) */
    unsigned int sampling_down_factor; /* how many sampling intervals to wait before down */
    unsigned int ignore_nice_load;     /* boolean (preserved but unused here) */
    unsigned int sampling_rate;        /* seconds between samples */
    int powersave_bias;                /* signed bias (-100 .. 100) */
};

/* Per-policy info */
struct lap_policy_info {
    struct cpufreq_policy *policy;

    /* runtime state */
    unsigned int requested_freq;
    unsigned int prev_load;
    unsigned int idle_periods; // Now correctly used to count consecutive low-load cycles
    unsigned int smoothed_load;

    /* tuners embedded */
    struct lap_tuners tuners;

    /* periodic work */
    struct delayed_work work;

    /* protect access */
    struct mutex lock;
};

/* Conservative-like defaults */
#define LAP_DEF_UP_THRESHOLD          75
#define LAP_DEF_DOWN_THRESHOLD        15
#define LAP_DEF_FREQ_STEP             5  
#define LAP_DEF_SAMPLING_DOWN_FAC     2
#define LAP_MAX_SAMPLING_DOWN_FAC     10
#define LAP_DEF_SAMPLING_RATE         1   /* seconds */
#define LAP_POWERSAVE_BIAS_MIN       -100
#define LAP_POWERSAVE_BIAS_MAX        100
/* powersave bias tweak on lap_is_on_ac() */
#define LAP_DEF_POWERSAVE_BIAS_DEFAULT             2
#define LAP_MAX_FREQ_STEP_PERCENT     35
#define LAP_MIN_FREQ_STEP_PERCENT     10
/* Should be within 1-200. 1-4 will be safe */
#define LAP_DEF_EMA_ALPHA_SCALING_FACTOR 3 

/* Compute freq step in kHz from percent of policy->max */
static inline unsigned int lap_get_freq_step_khz(struct lap_tuners *tuners,
                                                 struct cpufreq_policy *policy)
{
    unsigned int step_khz;

    step_khz = (tuners->freq_step * policy->max) / 100;

    // Fallback logic for when the calculated step is 0
    if (unlikely(step_khz == 0)) {
        // Use a fixed minimal percentage to ensure some step, then calculate
        step_khz = (LAP_MIN_FREQ_STEP_PERCENT * policy->max) / 100;
    }

    // Final check: if the minimal percentage still results in 0, use 1kHz as a last resort
    if (unlikely(step_khz == 0))
        step_khz = 1;

    return step_khz;
}

/*
 * lap_dbs_update - compute average load (0..100) across all CPUs in policy
 */
/*
 * lap_dbs_update - compute average load (0..100) across all CPUs in policy
 */
static unsigned int lap_dbs_update(struct cpufreq_policy *policy, bool ignore_nice)
{
    unsigned int load_sum = 0;
    unsigned int cpu;
    unsigned int cpus = cpumask_weight(policy->cpus);
    u64 cur_time;
    unsigned int time_elapsed;
    unsigned int cur_load;

    u64 cur_idle;
    u64 cur_nice;
    u64 idle_delta, nice_delta;

    if (!cpus)
        return 0;

    for_each_cpu(cpu, policy->cpus) {
        struct lap_cpu_dbs *cdbs = per_cpu_ptr(&lap_cpu_dbs, cpu);

        cur_idle = get_cpu_idle_time_us(cpu, &cur_time);
        cur_nice = jiffies_to_usecs(kcpustat_cpu(cpu).cpustat[CPUTIME_NICE]);

        time_elapsed = (unsigned int)(cur_time - cdbs->prev_update_time);
        idle_delta = (unsigned int)(cur_idle - cdbs->prev_cpu_idle);
        nice_delta = (unsigned int)(cur_nice - cdbs->prev_cpu_nice); 

        if (unlikely(time_elapsed == 0)) {
            cur_load = 100;
        } else {
            unsigned int busy_time = time_elapsed - idle_delta;
            if (ignore_nice)
                busy_time -= nice_delta; // ignore_nice == true -> exclude nice time

            cur_load = 100 * busy_time / time_elapsed;
        }

        cdbs->prev_cpu_idle = cur_idle;
        cdbs->prev_cpu_nice = cur_nice; // update prev_cpu_nice field
        cdbs->prev_update_time = cur_time;

        load_sum += cur_load;
    }

    return load_sum / cpus;
}

/*
 * lap_is_on_ac - Return 1 if system is on AC power, 0 otherwise.
 */
static inline _Bool lap_is_on_ac(void)
{
    struct power_supply *psy;
    union power_supply_propval val;
    int i;

    for (i = 0; i < ARRAY_SIZE(ac_names); i++) {
        psy = power_supply_get_by_name(ac_names[i]);
        if (!psy)
            continue;

        if (!power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE, &val)) {
            if (val.intval)
                return 1;
        }
    }

    return 0;
}

/*
 * cs_dbs_update - apply Laputil decision logic for one policy,
 * taking powersave_bias into account.
 */
static unsigned long cs_dbs_update(struct cpufreq_policy *policy)
{
    struct lap_policy_info *lp = policy->governor_data;
    struct lap_tuners *tuners;
    unsigned int requested_freq;
    unsigned int load;
    unsigned int step_khz;
    int bias;
    int eff_up, eff_down;

    if (!lp)
        return HZ; /* fallback 1s */

    tuners = &lp->tuners;

    // Lock for safe access to tuners and state
    mutex_lock(&lp->lock);

    /* compute current average load */
    load = lap_dbs_update(policy, lp->tuners.ignore_nice_load);
    int load_delta = load - lp->prev_load;
    u8 ema_alpha = (load_delta + 100) / LAP_DEF_EMA_ALPHA_SCALING_FACTOR;
    lp->smoothed_load = (ema_alpha * load + (100 - ema_alpha) * lp->smoothed_load) / 100;

    requested_freq = lp->requested_freq;

    /* sanitize requested_freq */
    if (requested_freq > policy->max || requested_freq < policy->min) {
        requested_freq = policy->cur;
        lp->requested_freq = requested_freq;
    }

    step_khz = lap_get_freq_step_khz(tuners, policy);


    /* derive effective thresholds using powersave_bias */
    bias = tuners->powersave_bias;
    if(lap_is_on_ac()) {
        bias += LAP_DEF_POWERSAVE_BIAS_DEFAULT;
    } else {
        bias -= LAP_DEF_POWERSAVE_BIAS_DEFAULT;
    }

    eff_up = (int)tuners->up_threshold + bias;
    eff_down = (int)tuners->down_threshold + bias;

    /* clamp eff_up/eff_down into sane ranges */
    if (eff_up > 100) eff_up = 100;
    if (eff_up < 1) eff_up = 1;
    if (eff_down < 0) eff_down = 0;
    if (eff_down >= eff_up) {
        eff_down = eff_up - 1;
        if (eff_down < 0)
            eff_down = 0;
    }

    /* decision: up (use effective up threshold) */
    if (lp->smoothed_load > (unsigned int)eff_up) {
        if (requested_freq != policy->max) {
            requested_freq += step_khz;
            if (requested_freq > policy->max)
                requested_freq = policy->max;

            cpufreq_driver_target(policy, requested_freq, CPUFREQ_RELATION_H);
            lp->requested_freq = requested_freq;
        }
        lp->idle_periods = 0; // Reset idle counter on upward change
    } else if (load < (unsigned int)eff_down) {
        // Increment idle counter if load is low
        lp->idle_periods++;
        // Check if low load has persisted for long enough
        if (lp->idle_periods >= tuners->sampling_down_factor) {
            if (requested_freq != policy->min) {
                if (requested_freq > step_khz)
                    requested_freq -= step_khz;
                else
                    requested_freq = policy->min;

                cpufreq_driver_target(policy, requested_freq, CPUFREQ_RELATION_L);
                lp->requested_freq = requested_freq;
            }
            lp->idle_periods = 0; // Reset idle counter after down-scaling
        }
    } else {
        // Load is between thresholds, reset idle counter
        lp->idle_periods = 0;
    }

    lp->prev_load = load;
    lp->requested_freq = requested_freq;
    mutex_unlock(&lp->lock);
    return (unsigned long)tuners->sampling_rate * HZ;
}

/* Delayed work handler per policy */
static void lap_work_handler(struct work_struct *work)
{
    struct lap_policy_info *lp = container_of(work, struct lap_policy_info, work.work);
    struct cpufreq_policy *policy = lp->policy;
    unsigned long delay_jiffies;

    /* run decision and get next delay */
    delay_jiffies = cs_dbs_update(policy);

    /* reschedule on the same CPU/policy */
    schedule_delayed_work_on(policy->cpu, &lp->work, delay_jiffies);
}

/************************** sysfs interface ************************/

#define lap_gov_attr(_name) \
static struct kobj_attribute _name##_attr = { \
    .attr = { .name = #_name, .mode = 0644 }, \
    .show = show_##_name, \
    .store = store_##_name, \
}

/* All store functions must now acquire the mutex to prevent race conditions. */

/* sampling_rate */
static ssize_t show_sampling_rate(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;
    ssize_t ret;

    if (!lp)
        return sprintf(buf, "0\n");

    mutex_lock(&lp->lock);
    ret = sprintf(buf, "%u\n", lp->tuners.sampling_rate);
    mutex_unlock(&lp->lock);

    return ret;
}

static ssize_t store_sampling_rate(struct kobject *kobj, struct kobj_attribute *attr,
                                   const char *buf, size_t count)
{
    unsigned int val;
    int ret;
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;

    if (!lp)
        return -EINVAL;

    ret = kstrtouint(buf, 10, &val);
    if (ret)
        return -EINVAL;

    if (val == 0) // sampling rate must be non-zero
        return -EINVAL;

    mutex_lock(&lp->lock);
    lp->tuners.sampling_rate = val;
    mutex_unlock(&lp->lock);
    
    return count;
}

/* sampling_down_factor */
static ssize_t show_sampling_down_factor(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;
    ssize_t ret;

    if (!lp)
        return sprintf(buf, "0\n");

    mutex_lock(&lp->lock);
    ret = sprintf(buf, "%u\n", lp->tuners.sampling_down_factor);
    mutex_unlock(&lp->lock);
    
    return ret;
}

static ssize_t store_sampling_down_factor(struct kobject *kobj, struct kobj_attribute *attr,
                                         const char *buf, size_t count)
{
    unsigned int val;
    int ret;
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;

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

/* up_threshold */
static ssize_t show_up_threshold(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;
    ssize_t ret;

    if (!lp)
        return sprintf(buf, "0\n");

    mutex_lock(&lp->lock);
    ret = sprintf(buf, "%u\n", lp->tuners.up_threshold);
    mutex_unlock(&lp->lock);
    
    return ret;
}

static ssize_t store_up_threshold(struct kobject *kobj, struct kobj_attribute *attr,
                                 const char *buf, size_t count)
{
    unsigned int val;
    int ret;
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;

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

/* down_threshold */
static ssize_t show_down_threshold(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;
    ssize_t ret;

    if (!lp)
        return sprintf(buf, "0\n");

    mutex_lock(&lp->lock);
    ret = sprintf(buf, "%u\n", lp->tuners.down_threshold);
    mutex_unlock(&lp->lock);
    
    return ret;
}

static ssize_t store_down_threshold(struct kobject *kobj, struct kobj_attribute *attr,
                                   const char *buf, size_t count)
{
    unsigned int val;
    int ret;
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;

    if (!lp)
        return -EINVAL;

    ret = kstrtouint(buf, 10, &val);
    if (ret || val < 0)
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

/* ignore_nice_load  */
static ssize_t show_ignore_nice_load(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;
    ssize_t ret;

    if (!lp)
        return sprintf(buf, "0\n");

    mutex_lock(&lp->lock);
    ret = sprintf(buf, "%u\n", lp->tuners.ignore_nice_load);
    mutex_unlock(&lp->lock);
    
    return ret;
}

static ssize_t store_ignore_nice_load(struct kobject *kobj, struct kobj_attribute *attr,
                                     const char *buf, size_t count)
{
    unsigned int val;
    int ret;
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;

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

/* freq_step (percent) */
static ssize_t show_freq_step(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;
    ssize_t ret;

    if (!lp)
        return sprintf(buf, "0\n");

    mutex_lock(&lp->lock);
    ret = sprintf(buf, "%u\n", lp->tuners.freq_step);
    mutex_unlock(&lp->lock);
    
    return ret;
}

static ssize_t store_freq_step(struct kobject *kobj, struct kobj_attribute *attr,
                              const char *buf, size_t count)
{
    unsigned int val;
    int ret;
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;

    if (!lp)
        return -EINVAL;

    ret = kstrtouint(buf, 10, &val);
    if (ret || val > LAP_MAX_FREQ_STEP_PERCENT || val < LAP_MIN_FREQ_STEP_PERCENT)
        return -EINVAL;

    mutex_lock(&lp->lock);
    lp->tuners.freq_step = val;
    mutex_unlock(&lp->lock);
    
    return count;
}

/* powersave_bias (signed -100..100) */
static ssize_t show_powersave_bias(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;
    ssize_t ret;

    if (!lp)
        return sprintf(buf, "0\n");

    mutex_lock(&lp->lock);
    ret = sprintf(buf, "%d\n", lp->tuners.powersave_bias);
    mutex_unlock(&lp->lock);
    
    return ret;
}

static ssize_t store_powersave_bias(struct kobject *kobj, struct kobj_attribute *attr,
                                    const char *buf, size_t count)
{
    int val;
    int ret;
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;

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

/* declare attributes */
lap_gov_attr(sampling_rate);
lap_gov_attr(sampling_down_factor);
lap_gov_attr(up_threshold);
lap_gov_attr(down_threshold);
lap_gov_attr(ignore_nice_load);
lap_gov_attr(freq_step);
lap_gov_attr(powersave_bias);

static struct attribute *lap_attrs[] = {
    &sampling_rate_attr.attr,
    &sampling_down_factor_attr.attr,
    &up_threshold_attr.attr,
    &down_threshold_attr.attr,
    &ignore_nice_load_attr.attr,
    &freq_step_attr.attr,
    &powersave_bias_attr.attr,
    NULL
};

static struct attribute_group lap_attr_group = {
    .attrs = lap_attrs,
    .name = "laputil",
};

/************************** Governor operations ************************/

static int lap_start(struct cpufreq_policy *policy)
{
    struct lap_policy_info *lp = policy->governor_data;
    unsigned long delay;

    if (!lp)
        return -EINVAL;

    mutex_lock(&lp->lock);
    lp->requested_freq = policy->cur;
    lp->prev_load = 0;
    lp->idle_periods = 0; // Correctly initialize the counter
    mutex_unlock(&lp->lock);
    
    delay = (unsigned long)lp->tuners.sampling_rate * HZ;
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

static int lap_init(struct cpufreq_policy *policy)
{
    struct lap_policy_info *lp;

    lp = kzalloc(sizeof(*lp), GFP_KERNEL);
    if (!lp)
        return -ENOMEM;

    /* defaults */
    lp->tuners.up_threshold = LAP_DEF_UP_THRESHOLD;
    lp->tuners.down_threshold = LAP_DEF_DOWN_THRESHOLD;
    lp->tuners.freq_step = LAP_DEF_FREQ_STEP;
    lp->tuners.sampling_down_factor = LAP_DEF_SAMPLING_DOWN_FAC;
    lp->tuners.ignore_nice_load = 1;
    lp->tuners.sampling_rate = LAP_DEF_SAMPLING_RATE;
    lp->tuners.powersave_bias = 0;

    lp->policy = policy;
    mutex_init(&lp->lock);

    INIT_DELAYED_WORK(&lp->work, lap_work_handler);

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
    .start = lap_start,
    .stop = lap_stop,
    .init = lap_init,
    .exit = lap_exit,
    .limits = NULL,
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

