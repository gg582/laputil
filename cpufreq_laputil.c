// SPDX-License-Identifier: GPL-2.0-only
/*
 * drivers/cpufreq/cpufreq_laputil.c
 * laputil: laptop-oriented conservative governor (with powersave_bias)
 *
 * Copyright (C) 2025 Lee Yunjin <gzblues61@daum.net>
 *
 * Conservative-style governor:
 *  - periodically sample CPU idle vs wall time per CPU in a policy
 *  - compute average load across policy CPUs
 *  - if load > up_threshold (effective) -> increase freq by freq_step (% of max)
 *  - if load < down_threshold (effective) -> after sampling_down_factor cycles, decrease freq
 *  - powersave_bias shifts effective thresholds to bias toward power or perf
 */

#include <linux/module.h>
#include <linux/kernel.h>
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

/*
 * Per-CPU previous state for idle accounting.
 */
struct lap_cpu_dbs {
    u64 prev_cpu_idle;
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
    unsigned int down_skip;
    unsigned int requested_freq;
    unsigned int prev_load;
    unsigned int idle_periods;

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
#define LAP_DEF_FREQ_STEP             3
#define LAP_DEF_SAMPLING_DOWN_FAC     2
#define LAP_MAX_SAMPLING_DOWN_FAC     10
#define LAP_DEF_SAMPLING_RATE         1   /* seconds */
#define LAP_POWERSAVE_BIAS_MIN       -100
#define LAP_POWERSAVE_BIAS_MAX        100

/* Compute freq step in kHz from percent of policy->max */
static inline unsigned int lap_get_freq_step_khz(struct lap_tuners *tuners,
                                                 struct cpufreq_policy *policy)
{
    unsigned int step_khz;

    step_khz = (tuners->freq_step * policy->max) / 100;

    if (unlikely(step_khz == 0))
        step_khz = LAP_DEF_FREQ_STEP; /* fallback small step (kHz) */

    return step_khz;
}

/*
 * lap_dbs_update - compute average load (0..100) across all CPUs in policy
 */
static unsigned int lap_dbs_update(struct cpufreq_policy *policy)
{
    unsigned int load_sum = 0;
    unsigned int cpu;
    unsigned int cpus = cpumask_weight(policy->cpus);

    if (!cpus)
        return 0;

    for_each_cpu(cpu, policy->cpus) {
        struct lap_cpu_dbs *cdbs = per_cpu_ptr(&lap_cpu_dbs, cpu);
        u64 cur_idle = 0;
        u64 cur_time = 0;
        unsigned int time_elapsed = 0;
        unsigned int idle_delta = 0;
        unsigned int cur_load;

        /* get cumulative idle and time (API variant used in many kernels) */
        cur_idle = get_cpu_idle_time(cpu, &cur_time, 0);

        time_elapsed = (unsigned int)(cur_time - cdbs->prev_update_time);
        idle_delta = (unsigned int)(cur_idle - cdbs->prev_cpu_idle);

        if (unlikely(time_elapsed == 0 || idle_delta > time_elapsed)) {
            cur_load = 100;
        } else {
            cur_load = 100 * (time_elapsed - idle_delta) / time_elapsed;
        }

        cdbs->prev_cpu_idle = cur_idle;
        cdbs->prev_update_time = cur_time;

        load_sum += cur_load;
    }

    return load_sum / cpus;
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

    /* compute current average load */
    load = lap_dbs_update(policy);

    requested_freq = lp->requested_freq;

    /* sanitize requested_freq */
    if (requested_freq > policy->max || requested_freq < policy->min) {
        requested_freq = policy->cur;
        lp->requested_freq = requested_freq;
    }

    step_khz = lap_get_freq_step_khz(tuners, policy);

    /* derive effective thresholds using powersave_bias */
    bias = tuners->powersave_bias; /* signed -100..100 */

    eff_up = (int)tuners->up_threshold + bias;
    eff_down = (int)tuners->down_threshold + bias;

    /* clamp eff_up/eff_down into sane ranges */
    if (eff_up > 100) eff_up = 100;
    if (eff_up < 1) eff_up = 1;
    if (eff_down < 0) eff_down = 0;
    if (eff_down >= eff_up) {
        /* ensure down < up */
        eff_down = eff_up - 1;
        if (eff_down < 0)
            eff_down = 0;
    }

    /* idle_periods placeholder handling for compatibility */
    if (lp->idle_periods < UINT_MAX) {
        unsigned int freq_steps = lp->idle_periods * step_khz;

        if (requested_freq > policy->min + freq_steps)
            requested_freq -= freq_steps;
        else
            requested_freq = policy->min;

        lp->idle_periods = UINT_MAX;
    }

    /* decision: up (use effective up threshold) */
    if (load > (unsigned int)eff_up) {
        if (lp->prev_load > (unsigned int)eff_up) {
            lp->down_skip = 0;

            if (requested_freq != policy->max) {
                requested_freq += step_khz;
                if (requested_freq > policy->max)
                    requested_freq = policy->max;

                cpufreq_driver_target(policy, requested_freq, CPUFREQ_RELATION_H);
                lp->requested_freq = requested_freq;
            }
        }

        lp->prev_load = load;
        return (unsigned long)tuners->sampling_rate * HZ;
    }

    /* sampling_down_factor: delay lowering until persisted */
    if (++lp->down_skip < tuners->sampling_down_factor)
        goto out_no_change;
    lp->down_skip = 0;

    /* decision: down (use effective down threshold) */
    if (load < (unsigned int)eff_down) {
        if (requested_freq != policy->min) {
            if (requested_freq > step_khz)
                requested_freq -= step_khz;
            else
                requested_freq = policy->min;

            cpufreq_driver_target(policy, requested_freq, CPUFREQ_RELATION_L);
            lp->requested_freq = requested_freq;
        }
    }

out_no_change:
    lp->prev_load = load;
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

/* sampling_rate */
static ssize_t show_sampling_rate(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;

    if (!lp)
        return sprintf(buf, "0\n");

    return sprintf(buf, "%u\n", lp->tuners.sampling_rate);
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

    if (val == 0)
        val = 1;

    lp->tuners.sampling_rate = val;
    return count;
}

/* sampling_down_factor */
static ssize_t show_sampling_down_factor(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;

    if (!lp)
        return sprintf(buf, "0\n");

    return sprintf(buf, "%u\n", lp->tuners.sampling_down_factor);
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

    lp->tuners.sampling_down_factor = val;
    return count;
}

/* up_threshold */
static ssize_t show_up_threshold(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;

    if (!lp)
        return sprintf(buf, "0\n");

    return sprintf(buf, "%u\n", lp->tuners.up_threshold);
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
    if (ret || val > 100 || val <= lp->tuners.down_threshold)
        return -EINVAL;

    lp->tuners.up_threshold = val;
    return count;
}

/* down_threshold */
static ssize_t show_down_threshold(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;

    if (!lp)
        return sprintf(buf, "0\n");

    return sprintf(buf, "%u\n", lp->tuners.down_threshold);
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
    if (ret || val < 0 || val >= lp->tuners.up_threshold)
        return -EINVAL;

    lp->tuners.down_threshold = val;
    return count;
}

/* ignore_nice_load (kept but not used) */
static ssize_t show_ignore_nice_load(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;

    if (!lp)
        return sprintf(buf, "0\n");

    return sprintf(buf, "%u\n", lp->tuners.ignore_nice_load);
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

    lp->tuners.ignore_nice_load = !!val;
    return count;
}

/* freq_step (percent) */
static ssize_t show_freq_step(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;

    if (!lp)
        return sprintf(buf, "0\n");

    return sprintf(buf, "%u\n", lp->tuners.freq_step);
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
    if (ret || val > 100)
        return -EINVAL;

    lp->tuners.freq_step = val;
    return count;
}

/* powersave_bias (signed -100..100) */
static ssize_t show_powersave_bias(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;

    if (!lp)
        return sprintf(buf, "0\n");

    return sprintf(buf, "%d\n", lp->tuners.powersave_bias);
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

    lp->tuners.powersave_bias = val;
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
    lp->down_skip = 0;
    lp->requested_freq = policy->cur;
    lp->prev_load = 0;
    lp->idle_periods = 0;

    delay = (unsigned long)lp->tuners.sampling_rate * HZ;
    schedule_delayed_work_on(policy->cpu, &lp->work, delay);
    mutex_unlock(&lp->lock);

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

