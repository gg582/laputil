// SPDX-License-Identifier: GPL-2.0-only
/*
 * drivers/cpufreq/cpufreq_laputil.c
 * laputil: laptop-oriented conservative governor (clean reimplementation)
 *
 * Copyright (C) 2025 Lee Yunjin <gzblues61@daum.net>
 *
 * This governor implements a conservative-style algorithm:
 *  - periodically sample CPU idle vs wall time per CPU in a policy
 *  - compute average load across policy CPUs
 *  - if load > up_threshold -> increase freq by freq_step (% of max)
 *  - if load < down_threshold -> after sampling_down_factor cycles, decrease freq
 *  - changes are applied step-by-step via cpufreq_driver_target()
 *
 * The implementation keeps tunables in sysfs under:
 *   /sys/devices/system/cpu/cpufreq/laputil/<policy>/laputil/...
 *
 * Comment style mirrors conventional kernel style (C-style comments).
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
#include <linux/tick.h>       /* get_cpu_idle_time_us / wrappers used by ondemand/conservative */
#include <linux/mutex.h>
#include <linux/sched/cpufreq.h> /* for cpufreq helpers */

/*
 * Per-CPU previous state for idle accounting.
 * prev_cpu_idle: previous absolute idle time (microseconds or cputime depending on API)
 * prev_update_time: previous wall/update timestamp (type preserved as u64)
 */
struct lap_cpu_dbs {
    u64 prev_cpu_idle;
    u64 prev_update_time;
};

DEFINE_PER_CPU(struct lap_cpu_dbs, lap_cpu_dbs);

/* Tunables */
struct lap_tuners {
    unsigned int down_threshold;       /* percent */
    unsigned int freq_step;            /* percent (of policy->max) */
    unsigned int up_threshold;         /* percent */
    unsigned int sampling_down_factor; /* how many sampling intervals to wait before down */
    unsigned int ignore_nice_load;     /* boolean */
    unsigned int sampling_rate;        /* seconds between samples (we return sampling_rate * HZ) */
};

struct lap_policy_info {
    struct cpufreq_policy *policy;

    /* runtime state */
    unsigned int down_skip;       /* counter for down detection */
    unsigned int requested_freq;  /* last requested frequency (kHz) */
    unsigned int prev_load;       /* previous averaged load percentage */
    unsigned int idle_periods;    /* unused but present for compatibility */

    /* tuners (embedded to avoid separate alloc & bad-cast/ leak) */
    struct lap_tuners tuners;

    /* periodic work for this policy */
    struct delayed_work work;

    /* protect access to policy info */
    struct mutex lock;
};

/* Conservative-like defaults */
#define LAP_DEF_UP_THRESHOLD        75
#define LAP_DEF_DOWN_THRESHOLD      15
#define LAP_DEF_FREQ_STEP           3
#define LAP_DEF_SAMPLING_DOWN_FAC   2
#define LAP_MAX_SAMPLING_DOWN_FAC   10
#define LAP_DEF_SAMPLING_RATE       1   /* seconds; default 1s responsive */

/* helper: compute freq step in kHz from percent of policy->max */
static inline unsigned int lap_get_freq_step_khz(struct lap_tuners *tuners,
                                                 struct cpufreq_policy *policy)
{
    unsigned int step_khz;

    /* freq_step is percent of policy->max (max is in kHz) */
    step_khz = (tuners->freq_step * policy->max) / 100;

    if (unlikely(step_khz == 0))
        step_khz = LAP_DEF_FREQ_STEP; /* fallback absolute kHz (small) */

    return step_khz;
}

/*
 * dbs_update - compute average load (0..100) across all CPUs in policy
 *
 * Implementation mirrors conservative/ondemand approach:
 * - use get_cpu_idle_time* API (returns cumulative idle time)
 * - subtract previous cumulative idle to get idle_delta
 * - compute time_elapsed via update_time difference
 * - current load = 100 * (busy_time / total_time)
 *
 * Note: kernel API variants exist; many kernels provide get_cpu_idle_time_us()
 * and related helpers used in ondemand/conservative governors. This code
 * uses the generic get_cpu_idle_time(cpu, &update_time, 0) pattern where
 * available in many trees; if your kernel has different prototype, adapt
 * that call accordingly (I can patch it per-kernel on request).
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

        /* get_cpu_idle_time variant used in many governors
         * - return cumulative idle as u64 (microsecs or cputime)
         * - also returns update_time/wall if API supports it
         *
         * Some kernels provide get_cpu_idle_time_us(cpu, &wall), others
         * provide get_cpu_idle_time(cpu, &update_time, 0). If your kernel
         * uses a different symbol, adjust this call.
         */
        cur_idle = get_cpu_idle_time(cpu, &cur_time, 0);

        /* elapsed wall time since last sample (units match cur_time) */
        time_elapsed = (unsigned int)(cur_time - cdbs->prev_update_time);

        /* change in idle time (units match cur_idle) */
        idle_delta = (unsigned int)(cur_idle - cdbs->prev_cpu_idle);

        /* sanity checks */
        if (unlikely(time_elapsed == 0 || idle_delta > time_elapsed)) {
            /* if impossible data, consider CPU busy (100% load) */
            cur_load = 100;
        } else {
            /* busy = time_elapsed - idle_delta */
            cur_load = 100 * (time_elapsed - idle_delta) / time_elapsed;
        }

        /* save for next round */
        cdbs->prev_cpu_idle = cur_idle;
        cdbs->prev_update_time = cur_time;

        load_sum += cur_load;
    }

    /* average */
    return load_sum / cpus;
}

/*
 * cs_dbs_update - apply Laputil decision logic for one policy
 *
 * returns: delay in jiffies until next invocation (sampling_rate * HZ)
 */
static unsigned long cs_dbs_update(struct cpufreq_policy *policy)
{
    struct lap_policy_info *lp = policy->governor_data;
    struct lap_tuners *tuners;
    unsigned int requested_freq;
    unsigned int load;
    unsigned int step_khz;

    if (!lp)
        return HZ; /* safe 1s default */

    tuners = &lp->tuners;

    /* compute current average load */
    load = lap_dbs_update(policy);

    requested_freq = lp->requested_freq;

    /* sanitize stored requested_freq */
    if (requested_freq > policy->max || requested_freq < policy->min) {
        requested_freq = policy->cur;
        lp->requested_freq = requested_freq;
    }

    step_khz = lap_get_freq_step_khz(tuners, policy);

    /* handle idle_periods (compat placeholder) */
    if (lp->idle_periods < UINT_MAX) {
        unsigned int freq_steps = lp->idle_periods * step_khz;

        if (requested_freq > policy->min + freq_steps)
            requested_freq -= freq_steps;
        else
            requested_freq = policy->min;

        lp->idle_periods = UINT_MAX;
    }

    /* decision: up */
    if (load > tuners->up_threshold) {
        if (lp->prev_load > tuners->up_threshold) {
            lp->down_skip = 0;

            if (requested_freq != policy->max) {
                requested_freq += step_khz;
                if (requested_freq > policy->max)
                    requested_freq = policy->max;

                /* request a frequency increase */
                cpufreq_driver_target(policy, requested_freq, CPUFREQ_RELATION_H);
                lp->requested_freq = requested_freq;
            }
        }

        /* record and return */
        lp->prev_load = load;
        return (unsigned long)tuners->sampling_rate * HZ;
    }

    /* sampling_down_factor: delay lowering until persisted */
    if (++lp->down_skip < tuners->sampling_down_factor)
        goto out_no_change;
    lp->down_skip = 0;

    /* decision: down */
    if (load < tuners->down_threshold) {
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

/* delayed work handler for policy */
static void lap_work_handler(struct work_struct *work)
{
    struct lap_policy_info *lp = container_of(work, struct lap_policy_info, work.work);
    struct cpufreq_policy *policy = lp->policy;
    unsigned long delay_jiffies;

    /* run the decision/update and get next delay */
    delay_jiffies = cs_dbs_update(policy);

    /* requeue */
    schedule_delayed_work_on(policy->cpu, &lp->work, delay_jiffies);
}

/************************** sysfs interface ************************/

/* helper macro to define attributes easily */
#define lap_gov_attr(_name) \
static struct kobj_attribute _name##_attr = { \
    .attr = { .name = #_name, .mode = 0644 }, \
    .show = show_##_name, \
    .store = store_##_name, \
}

/* show/store functions: retrieve policy from kobject owner */
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
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;
    int ret;

    if (!lp)
        return -EINVAL;

    ret = kstrtouint(buf, 10, &val);
    if (ret)
        return -EINVAL;

    if (val == 0)
        val = 1; /* avoid 0-second sampling */

    lp->tuners.sampling_rate = val;
    return count;
}

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
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;
    int ret;

    if (!lp)
        return -EINVAL;

    ret = kstrtouint(buf, 10, &val);
    if (ret || val < 1 || val > LAP_MAX_SAMPLING_DOWN_FAC)
        return -EINVAL;

    lp->tuners.sampling_down_factor = val;
    return count;
}

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
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;
    int ret;

    if (!lp)
        return -EINVAL;

    ret = kstrtouint(buf, 10, &val);
    if (ret || val > 100 || val <= lp->tuners.down_threshold)
        return -EINVAL;

    lp->tuners.up_threshold = val;
    return count;
}

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
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;
    int ret;

    if (!lp)
        return -EINVAL;

    ret = kstrtouint(buf, 10, &val);
    if (ret || val < 1 || val >= lp->tuners.up_threshold)
        return -EINVAL;

    lp->tuners.down_threshold = val;
    return count;
}

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
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;
    int ret;

    if (!lp)
        return -EINVAL;

    ret = kstrtouint(buf, 10, &val);
    if (ret)
        return -EINVAL;

    lp->tuners.ignore_nice_load = !!val;
    return count;
}

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
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;
    int ret;

    if (!lp)
        return -EINVAL;

    ret = kstrtouint(buf, 10, &val);
    if (ret || val > 100)
        return -EINVAL;

    lp->tuners.freq_step = val;
    return count;
}

/* declare attributes */
lap_gov_attr(sampling_rate);
lap_gov_attr(sampling_down_factor);
lap_gov_attr(up_threshold);
lap_gov_attr(down_threshold);
lap_gov_attr(ignore_nice_load);
lap_gov_attr(freq_step);

static struct attribute *lap_attrs[] = {
    &sampling_rate_attr.attr,
    &sampling_down_factor_attr.attr,
    &up_threshold_attr.attr,
    &down_threshold_attr.attr,
    &ignore_nice_load_attr.attr,
    &freq_step_attr.attr,
    NULL
};

static struct attribute_group lap_attr_group = {
    .attrs = lap_attrs,
    .name = "laputil",
};

/************************** Governor operations ************************/

/* start: called when policy switches to this governor */
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

    /* schedule first run after sampling_rate seconds */
    delay = (unsigned long)lp->tuners.sampling_rate * HZ;
    schedule_delayed_work_on(policy->cpu, &lp->work, delay);
    mutex_unlock(&lp->lock);

    return 0;
}

/* stop: cancel periodic work */
static void lap_stop(struct cpufreq_policy *policy)
{
    struct lap_policy_info *lp = policy->governor_data;

    if (!lp)
        return;

    cancel_delayed_work_sync(&lp->work);
}

/* init: allocate per-policy state and create sysfs group */
static int lap_init(struct cpufreq_policy *policy)
{
    struct lap_policy_info *lp;

    lp = kzalloc(sizeof(*lp), GFP_KERNEL);
    if (!lp)
        return -ENOMEM;

    /* set defaults for tuners */
    lp->tuners.up_threshold = LAP_DEF_UP_THRESHOLD;
    lp->tuners.down_threshold = LAP_DEF_DOWN_THRESHOLD;
    lp->tuners.freq_step = LAP_DEF_FREQ_STEP;
    lp->tuners.sampling_down_factor = LAP_DEF_SAMPLING_DOWN_FAC;
    lp->tuners.ignore_nice_load = 1;
    lp->tuners.sampling_rate = LAP_DEF_SAMPLING_RATE;

    lp->policy = policy;
    mutex_init(&lp->lock);

    /* init delayed work */
    INIT_DELAYED_WORK(&lp->work, lap_work_handler);

    policy->governor_data = lp;

    if (sysfs_create_group(&policy->kobj, &lap_attr_group)) {
        kfree(lp);
        policy->governor_data = NULL;
        return -EINVAL;
    }

    return 0;
}

/* exit: free memory and remove sysfs */
static void lap_exit(struct cpufreq_policy *policy)
{
    struct lap_policy_info *lp = policy->governor_data;

    if (!lp)
        return;

    /* ensure pending work is gone */
    cancel_delayed_work_sync(&lp->work);

    sysfs_remove_group(&policy->kobj, &lap_attr_group);

    kfree(lp);
    policy->governor_data = NULL;
}

/* governor registration */
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
MODULE_DESCRIPTION("'cpufreq_laputil' - Conservative-style governor for laptops");
MODULE_LICENSE("GPL");

module_init(laputil_module_init);
module_exit(laputil_module_exit);

