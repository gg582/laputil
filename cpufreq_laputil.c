// SPDX-License-Identifier: GPL-2.0-only
/*
 * drivers/cpufreq/cpufreq_laputil.c
 * laptop-oriented conservative governor
 *
 * Copyright (C) 2001 Russell King
 * (C) 2003 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>.
 * Jun Nakajima <jun.nakajima@intel.com>
 * (C) 2009 Alexander Clouter <alex@digriz.org.uk>
 * (C) 2025 Lee Yunjin <gzblues61@daum.net>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/cpufreq.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>

/*
 * This structure holds the previous state for each CPU core.
 */
struct cpu_dbs_info {
    u64 prev_cpu_idle;
    u64 prev_update_time;
};

/*
 * Use a macro to declare a per-CPU variable of type 'struct cpu_dbs_info'.
 */
DEFINE_PER_CPU(struct cpu_dbs_info, cpu_dbs);

/* Structures for the governor */
struct cs_policy_dbs_info {
    struct cpufreq_policy *policy;
    unsigned int down_skip;
    unsigned int requested_freq;
    unsigned int prev_load;
    unsigned int idle_periods;
};

struct cs_dbs_tuners {
    unsigned int down_threshold;
    unsigned int freq_step;
    unsigned int up_threshold;
    unsigned int sampling_down_factor;
    unsigned int ignore_nice_load;
    unsigned int sampling_rate;
};

/* Conservative governor macros */
#define DEF_FREQUENCY_UP_THRESHOLD      (90)
#define DEF_FREQUENCY_DOWN_THRESHOLD    (10)
#define DEF_FREQUENCY_STEP              (3)
#define DEF_SAMPLING_DOWN_FACTOR        (2)
#define MAX_SAMPLING_DOWN_FACTOR        (10)
#define CPU_IDLE_RT_DOWN_THRESHOLD      (750)

static inline unsigned int get_freq_step(struct cs_dbs_tuners *cs_tuners,
                                        struct cpufreq_policy *policy)
{
    unsigned int freq_step = (cs_tuners->freq_step * policy->max) / 100;

    if (unlikely(freq_step == 0))
        freq_step = DEF_FREQUENCY_STEP;

    return freq_step;
}

static unsigned int dbs_update(struct cpufreq_policy *policy)
{
    unsigned int load = 0;
    unsigned int cpu;

    for_each_cpu(cpu, policy->cpus) {
        struct cpu_dbs_info *cdbs = per_cpu_ptr(&cpu_dbs, cpu);
        u64 cur_idle_time;
        time64_t update_time;
        unsigned int time_elapsed, idle_time;
        unsigned int cur_load_per_cpu;

        // Get the current idle time and update time from the kernel
        cur_idle_time = get_cpu_idle_time(cpu, &update_time, 0);

        // Calculate the elapsed time since the last update
        time_elapsed = (unsigned int)(update_time - cdbs->prev_update_time);

        // Calculate the change in idle time during that period
        idle_time = (unsigned int)(cur_idle_time - cdbs->prev_cpu_idle);

        // Handle the case where no time has elapsed or idle time is invalid
        if (unlikely(!time_elapsed || idle_time > time_elapsed)) {
            cur_load_per_cpu = 100;
        } else {
            // Calculate the load percentage: 100 * (busy_time / total_time)
            cur_load_per_cpu = 100 * (time_elapsed - idle_time) / time_elapsed;
        }

        // Store the current values for the next calculation
        cdbs->prev_cpu_idle = cur_idle_time;
        cdbs->prev_update_time = update_time;

        load += cur_load_per_cpu;
    }

    // Return the average load for all CPUs in the policy
    if (cpumask_weight(policy->cpus) > 0)
        return load / cpumask_weight(policy->cpus);

    return 0;
}

static unsigned int cs_dbs_update(struct cpufreq_policy *policy)
{
    struct cs_policy_dbs_info *dbs_info = policy->governor_data;
    struct cs_dbs_tuners *cs_tuners = policy->governor_data;
    unsigned int requested_freq = dbs_info->requested_freq;
    unsigned int load = dbs_update(policy);
    unsigned int freq_step;

    if (cs_tuners->freq_step == 0)
        goto out;

    if (requested_freq > policy->max || requested_freq < policy->min) {
        requested_freq = policy->cur;
        dbs_info->requested_freq = requested_freq;
    }

    freq_step = get_freq_step(cs_tuners, policy);

    if (dbs_info->idle_periods < UINT_MAX) {
        unsigned int freq_steps = dbs_info->idle_periods * freq_step;

        if (requested_freq > policy->min + freq_steps)
            requested_freq -= freq_steps;
        else
            requested_freq = policy->min;

        dbs_info->idle_periods = UINT_MAX;
    }

    if (load > cs_tuners->up_threshold) {
        if (dbs_info->prev_load > cs_tuners->up_threshold) {
            dbs_info->down_skip = 0;

            if (requested_freq == policy->max)
                goto out;

            requested_freq += freq_step;
            if (requested_freq > policy->max)
                requested_freq = policy->max;

            cpufreq_driver_target(policy, requested_freq, CPUFREQ_RELATION_H);
            dbs_info->requested_freq = requested_freq;
        }
        goto out;
    }

    if (++dbs_info->down_skip < cs_tuners->sampling_down_factor)
        goto out;
    dbs_info->down_skip = 0;

    if (load < cs_tuners->down_threshold) {
        if (requested_freq == policy->min)
            goto out;

        if (requested_freq > freq_step)
            requested_freq -= freq_step;
        else
            requested_freq = policy->min;

        cpufreq_driver_target(policy, requested_freq, CPUFREQ_RELATION_L);
        dbs_info->requested_freq = requested_freq;
    }

out:
    dbs_info->prev_load = load;
    return cs_tuners->sampling_rate * HZ;
}

/************************** sysfs interface ************************/

#define gov_attr(_name) \
static struct kobj_attribute _name##_attr = { \
    .attr = { .name = #_name, .mode = 0644 }, \
    .show = show_##_name, \
    .store = store_##_name, \
}

static ssize_t show_sampling_rate(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct cs_dbs_tuners *cs_tuners = policy->governor_data;
    return sprintf(buf, "%u\n", cs_tuners->sampling_rate);
}

static ssize_t store_sampling_rate(struct kobject *kobj, struct kobj_attribute *attr,
                                   const char *buf, size_t count)
{
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct cs_dbs_tuners *cs_tuners = policy->governor_data;
    unsigned int input;
    int ret;

    ret = kstrtouint(buf, 10, &input);
    if (ret)
        return -EINVAL;

    cs_tuners->sampling_rate = input;
    return count;
}

static ssize_t show_sampling_down_factor(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct cs_dbs_tuners *cs_tuners = policy->governor_data;
    return sprintf(buf, "%u\n", cs_tuners->sampling_down_factor);
}

static ssize_t store_sampling_down_factor(struct kobject *kobj, struct kobj_attribute *attr,
                                         const char *buf, size_t count)
{
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct cs_dbs_tuners *cs_tuners = policy->governor_data;
    unsigned int input;
    int ret;

    ret = kstrtouint(buf, 10, &input);
    if (ret || input > MAX_SAMPLING_DOWN_FACTOR || input < 1)
        return -EINVAL;

    cs_tuners->sampling_down_factor = input;
    return count;
}

static ssize_t show_up_threshold(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct cs_dbs_tuners *cs_tuners = policy->governor_data;
    return sprintf(buf, "%u\n", cs_tuners->up_threshold);
}

static ssize_t store_up_threshold(struct kobject *kobj, struct kobj_attribute *attr,
                                 const char *buf, size_t count)
{
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct cs_dbs_tuners *cs_tuners = policy->governor_data;
    unsigned int input;
    int ret;

    ret = kstrtouint(buf, 10, &input);
    if (ret || input > 100 || input <= cs_tuners->down_threshold)
        return -EINVAL;

    cs_tuners->up_threshold = input;
    return count;
}

static ssize_t show_down_threshold(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct cs_dbs_tuners *cs_tuners = policy->governor_data;
    return sprintf(buf, "%u\n", cs_tuners->down_threshold);
}

static ssize_t store_down_threshold(struct kobject *kobj, struct kobj_attribute *attr,
                                   const char *buf, size_t count)
{
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct cs_dbs_tuners *cs_tuners = policy->governor_data;
    unsigned int input;
    int ret;

    ret = kstrtouint(buf, 10, &input);
    if (ret || input < 1 || input >= cs_tuners->up_threshold)
        return -EINVAL;

    cs_tuners->down_threshold = input;
    return count;
}

static ssize_t show_ignore_nice_load(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct cs_dbs_tuners *cs_tuners = policy->governor_data;
    return sprintf(buf, "%u\n", cs_tuners->ignore_nice_load);
}

static ssize_t store_ignore_nice_load(struct kobject *kobj, struct kobj_attribute *attr,
                                     const char *buf, size_t count)
{
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct cs_dbs_tuners *cs_tuners = policy->governor_data;
    unsigned int input;
    int ret;

    ret = kstrtouint(buf, 10, &input);
    if (ret)
        return -EINVAL;

    if (input > 1)
        input = 1;

    cs_tuners->ignore_nice_load = input;
    return count;
}

static ssize_t show_freq_step(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct cs_dbs_tuners *cs_tuners = policy->governor_data;
    return sprintf(buf, "%u\n", cs_tuners->freq_step);
}

static ssize_t store_freq_step(struct kobject *kobj, struct kobj_attribute *attr,
                              const char *buf, size_t count)
{
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct cs_dbs_tuners *cs_tuners = policy->governor_data;
    unsigned int input;
    int ret;

    ret = kstrtouint(buf, 10, &input);
    if (ret)
        return -EINVAL;

    if (input > 100)
        input = 100;

    cs_tuners->freq_step = input;
    return count;
}

gov_attr(sampling_rate);
gov_attr(sampling_down_factor);
gov_attr(up_threshold);
gov_attr(down_threshold);
gov_attr(ignore_nice_load);
gov_attr(freq_step);

static struct attribute *cs_attrs[] = {
    &sampling_rate_attr.attr,
    &sampling_down_factor_attr.attr,
    &up_threshold_attr.attr,
    &down_threshold_attr.attr,
    &ignore_nice_load_attr.attr,
    &freq_step_attr.attr,
    NULL
};

static struct attribute_group cs_attr_group = {
    .attrs = cs_attrs,
    .name = "laputil",
};

/************************** Governor operations ************************/

static int cs_start(struct cpufreq_policy *policy)
{
    struct cs_policy_dbs_info *dbs_info = policy->governor_data;

    dbs_info->down_skip = 0;
    dbs_info->requested_freq = policy->cur;
    dbs_info->prev_load = 0;
    dbs_info->idle_periods = 0;

    return 0;
}

static void cs_exit(struct cpufreq_policy *policy)
{
    kfree(policy->governor_data);
}

static int cs_init(struct cpufreq_policy *policy)
{
    struct cs_policy_dbs_info *dbs_info;
    struct cs_dbs_tuners *tuners;

    dbs_info = kzalloc(sizeof(*dbs_info), GFP_KERNEL);
    if (!dbs_info)
        return -ENOMEM;

    tuners = kzalloc(sizeof(*tuners), GFP_KERNEL);
    if (!tuners) {
        kfree(dbs_info);
        return -ENOMEM;
    }

    tuners->down_threshold = DEF_FREQUENCY_DOWN_THRESHOLD;
    tuners->freq_step = DEF_FREQUENCY_STEP;
    tuners->up_threshold = DEF_FREQUENCY_UP_THRESHOLD;
    tuners->sampling_down_factor = DEF_SAMPLING_DOWN_FACTOR;
    tuners->ignore_nice_load = 0;
    tuners->sampling_rate = 10; /* Default sampling rate in jiffies */

    dbs_info->policy = policy;
    policy->governor_data = dbs_info;

    return sysfs_create_group(&policy->kobj, &cs_attr_group);
}

static void cs_stop(struct cpufreq_policy *policy)
{
    sysfs_remove_group(&policy->kobj, &cs_attr_group);
}

static struct cpufreq_governor cpufreq_gov_laputil = {
    .name = "laputil",
    .owner = THIS_MODULE,
    .start = cs_start,
    .exit = cs_exit,
    .init = cs_init,
    .stop = cs_stop,
    .limits = NULL, /* Optional: implement if needed */
};

static int __init cpufreq_gov_laputil_init(void)
{
    return cpufreq_register_governor(&cpufreq_gov_laputil);
}

static void __exit cpufreq_gov_laputil_exit(void)
{
    cpufreq_unregister_governor(&cpufreq_gov_laputil);
}

MODULE_AUTHOR("Alexander Clouter <alex@digriz.org.uk>");
MODULE_DESCRIPTION("'cpufreq_laputil' - A dynamic cpufreq governor for "
                   "Low Latency Frequency Transition capable processors "
                   "optimized for use in a battery environment");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_LAPUTIL
struct cpufreq_governor *cpufreq_default_governor(void)
{
    return &cpufreq_gov_laputil;
}
#endif

module_init(cpufreq_gov_laputil_init);
module_exit(cpufreq_gov_laputil_exit);
