/*
 * drivers/cpufreq/cpufreq_laputil.c
 * laputil: laptop-oriented conservative governor
 *
 * Copyright (C) 2025 Lee Yunjin <gzblues61@daum.net>
 *
 * Conservative-style governor:
 * - periodically sample CPU idle vs wall time per CPU in a policy
 * - compute average load across policy CPUs
 * - optimize cpu frequency steps via adam inspired optimizer
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
#include <linux/types.h>
#include "include/ac_names_gen.h"
#include "include/battery.h"

#define FP_SHIFT 16
#define FP_SCALE (1 << FP_SHIFT)

#define BETA1_FP ((s64)(58982))
#define BETA2_FP ((s64)(65470))

#define ONE_MINUS_BETA1_FP (FP_SCALE - BETA1_FP)
#define ONE_MINUS_BETA2_FP (FP_SCALE - BETA2_FP)

#define AFS_HIGH_THRESHOLD 150
#define AFS_LOW_THRESHOLD  50

static s64 m_fp = 0;
static s64 v_fp = 0;

struct lap_cpu_dbs {
    u64 prev_cpu_idle;
    u64 prev_cpu_nice;
    u64 prev_update_time;
};

DEFINE_PER_CPU(struct lap_cpu_dbs, lap_cpu_dbs);

struct lap_tuners {
    unsigned int down_threshold;
    unsigned int freq_step;
    unsigned int up_threshold;
    unsigned int sampling_down_factor;
    unsigned int ignore_nice_load;
    unsigned int sampling_rate;
};

struct lap_policy_info {
    struct cpufreq_policy *policy;
    unsigned int requested_freq;
    unsigned int prev_load;
    unsigned int prev_remaining;
    unsigned int idle_periods;
    unsigned int smoothed_load;
    struct lap_tuners tuners;
    struct delayed_work work;
    struct mutex lock;
    struct cpumask eff_mask;
    struct cpumask perf_mask;
};

#define LAP_DEF_UP_THRESHOLD             75
#define LAP_DEF_DOWN_THRESHOLD           10
#define LAP_DEF_FREQ_STEP                 5
#define LAP_DEF_SAMPLING_DOWN_FAC         2
#define LAP_MAX_SAMPLING_DOWN_FAC         5
#define LAP_DEF_SAMPLING_RATE             1
#define LAP_MAX_FREQ_STEP_PERCENT        25
#define LAP_MIN_FREQ_STEP_PERCENT         5
#define LAP_DEF_EMA_ALPHA_SCALING_FACTOR  3

/* update_power_momentum_fixed - update momentum values from power draw */
static void update_power_momentum_fixed(s64 current_power)
{
    s64 power_fp = current_power << FP_SHIFT;
    s64 power_sq_fp = (current_power * current_power) << FP_SHIFT;

    m_fp = (((BETA1_FP * m_fp) >> FP_SHIFT) +
            ((ONE_MINUS_BETA1_FP * power_fp) >> FP_SHIFT));
    v_fp = (((BETA2_FP * v_fp) >> FP_SHIFT) +
            ((ONE_MINUS_BETA2_FP * power_sq_fp) >> FP_SHIFT));
}

/* detect_clusters - Detect efficiency and performance cores */
static void detect_clusters(struct cpufreq_policy *policy, struct cpumask *eff_mask, struct cpumask *perf_mask)
{
    unsigned int cpu;
    unsigned int eff_max_freq = UINT_MAX, perf_max_freq = 0;

    cpumask_clear(eff_mask);
    cpumask_clear(perf_mask);

    for_each_cpu(cpu, policy->cpus) {
        unsigned int max_freq = cpufreq_quick_get_max(cpu);
        if (max_freq < eff_max_freq) {
            eff_max_freq = max_freq;
            cpumask_set_cpu(cpu, eff_mask);
        }
        if (max_freq > perf_max_freq) {
            perf_max_freq = max_freq;
            cpumask_set_cpu(cpu, perf_mask);
        }
    }
}

/* lap_get_freq_step_khz - Compute freq step in kHz from percent */
static inline unsigned int lap_get_freq_step_khz(struct lap_tuners *tuners,
                                                 struct cpufreq_policy *policy)
{
    unsigned int step_khz;

    step_khz = (tuners->freq_step * policy->max) / 100;

    if (unlikely(step_khz == 0)) {
        step_khz = (LAP_MIN_FREQ_STEP_PERCENT * policy->max) / 100;
    }

    if (unlikely(step_khz == 0))
        step_khz = 1;

    return step_khz;
}

/* lap_is_on_ac - Retrieves Battery/AC Status and Average Power */
static inline void lap_is_on_ac(battery_t *battery_status)
{
    struct power_supply *psy;
    union power_supply_propval val;
    int i;
    bool found_ac = false;

    /* Default to on-battery, 100% capacity, 0 power draw */
    battery_status->active = 1;
    battery_status->remaining = 100;
    battery_status->power_avg = 0;

    /* First, check all possible AC adapter names */
    for (i = 0; i < ARRAY_SIZE(ac_names) && ac_names[i] != NULL; i++) {
        psy = power_supply_get_by_name(ac_names[i]);
        if (!psy)
            continue;

        if (power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE, &val) == 0 && val.intval) {
            found_ac = true;
            power_supply_put(psy);
            break; /* If any AC adapter is online, we are on AC power */
        }
        power_supply_put(psy);
    }

    if (found_ac) {
        battery_status->active = 0; /* On AC power */
        return;
    }

    /* If not on AC, find the first available battery to get its status */
    for (i = 0; i < ARRAY_SIZE(battery_names) && battery_names[i] != NULL; i++) {
        psy = power_supply_get_by_name(battery_names[i]);
        if (!psy)
            continue;

        /* Try to get capacity and average power from this battery */
        if (power_supply_get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &val) == 0) {
            battery_status->remaining = val.intval;

            /* **BUG FIX**: Correctly get and store power_avg */
            if (power_supply_get_property(psy, POWER_SUPPLY_PROP_POWER_AVG, &val) == 0) {
                /* Store the value in the provided battery_status struct */
                battery_status->power_avg = val.intval / 1000; /* uW to mW */
            }

            power_supply_put(psy);
            return; /* We found our battery, no need to check others */
        }
        power_supply_put(psy);
    }
}
/* lap_dbs_update - compute average load (0..100) across all CPUs in policy */
static unsigned int lap_dbs_update(struct cpufreq_policy *policy, bool ignore_nice)
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
    battery_t battery_status;

    if (!lp || !cpumask_weight(policy->cpus))
        return 0;

    /* Initialize core masks if not already done */
    if (cpumask_empty(&lp->eff_mask) || cpumask_empty(&lp->perf_mask)) {
        detect_clusters(policy, &lp->eff_mask, &lp->perf_mask);
    }
    eff_cpus = cpumask_weight(&lp->eff_mask);
    perf_cpus = cpumask_weight(&lp->perf_mask);

    /* Compute load for efficiency and performance cores separately */
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
                busy_time -= nice_delta;
            cur_load = 100 * busy_time / time_elapsed;
        }

        cdbs->prev_cpu_idle = cur_idle;
        cdbs->prev_cpu_nice = cur_nice;
        cdbs->prev_update_time = cur_time;

        if (cpumask_test_cpu(cpu, &lp->eff_mask)) {
            eff_load_sum += cur_load;
        } else if (cpumask_test_cpu(cpu, &lp->perf_mask)) {
            perf_load_sum += cur_load;
        }
        load_sum += cur_load;
    }

    /* Calculate average load based on battery capacity */
    if (eff_cpus > 0 && perf_cpus > 0) {
        unsigned int eff_load = eff_load_sum / eff_cpus;
        unsigned int perf_load = perf_load_sum / perf_cpus;

        lap_is_on_ac(&battery_status);
        if (battery_status.active && battery_status.remaining <= 20) {
            return eff_load;
        } else {
            return (eff_load * 7 + perf_load * 3) / 10;
        }
    }

    return load_sum / cpumask_weight(policy->cpus);
}

/**
 * isqrt - integer square root
 * @n: the number to calculate the square root of
 *
 * A fast, purely integer-based square root function suitable for the kernel.
 * It uses a bitwise binary search method.
 */
static u64 isqrt(u64 n)
{
    u64 root = 0;
    u64 bit;
    u64 trial;

    bit = (n > 0xFFFFFFFFUL) ? (1UL << 62) : (1UL << 30);

    while (bit > n)
        bit >>= 2;

    for (; bit != 0; bit >>= 2) {
        trial = root + bit;
        root >>= 1;
        if (n >= trial) {
            n -= trial;
            root += bit;
        }
    }
    return root;
}

/* adaptive_frequency_step_update - determine next freq step via optimizer */
static void adaptive_frequency_step_update(struct cpufreq_policy *policy, s64 m, s64 v, unsigned int current_load)
{
    struct lap_policy_info *lp = policy->governor_data;
    s32 trend_mw = m >> FP_SHIFT;
    s64 volatility_mw2 = v >> FP_SHIFT;
    s64 update_vector;
    s32 freq_step = 0;
    unsigned int requested_freq, step_khz;

    if (volatility_mw2 > 0) {
        u64 sqrt_v = isqrt((u64)volatility_mw2);
        if (sqrt_v > 0) {
            update_vector = div64_s64((s64)trend_mw << 8, sqrt_v);
        } else {
            update_vector = 0;
        }
    } else {
        update_vector = 0;
    }

    /* **TUNING**: Adjust step sizes for better responsiveness */
    if (update_vector > AFS_HIGH_THRESHOLD) {
        freq_step = +2;
    } else if (update_vector > AFS_LOW_THRESHOLD) {
        freq_step = +1;
    } else if (update_vector < -AFS_HIGH_THRESHOLD) {
        freq_step = -4; /* More aggressive step-down for clear idle trend */
    } else if (update_vector < -AFS_LOW_THRESHOLD) {
        freq_step = -2; /* Stronger step-down for moderate idle trend */
    }

    /* **TUNING**: Strengthen the override condition */
    if (current_load > lp->tuners.up_threshold && freq_step <= 0) {
        /*
         * Override only if the power trend is not clearly decreasing.
         * A negative trend indicates the load is transient and about to drop.
         */
        if (trend_mw > -1000) { /* Threshold increased to -1000mW */
            freq_step = +1;
        }
    }

    /* Also, if load is very low, force a step-down regardless of optimizer */
    if (current_load < lp->tuners.down_threshold && freq_step >= 0) {
        freq_step = -1;
    }

    if (freq_step == 0) {
        return;
    }

    requested_freq = lp->requested_freq;
    step_khz = lap_get_freq_step_khz(&lp->tuners, policy);

    if (freq_step > 0) {
        requested_freq += (freq_step * step_khz);
        if (requested_freq > policy->max)
            requested_freq = policy->max;
        cpufreq_driver_target(policy, requested_freq, CPUFREQ_RELATION_H);
    } else {
        if (requested_freq > ((-freq_step) * step_khz))
            requested_freq -= ((-freq_step) * step_khz);
        else
            requested_freq = policy->min;

        if (requested_freq < policy->min)
             requested_freq = policy->min;
        cpufreq_driver_target(policy, requested_freq, CPUFREQ_RELATION_L);
    }
    lp->requested_freq = requested_freq;
}

/* cs_dbs_update - apply Laputil decision logic for one policy */
static unsigned long cs_dbs_update(struct cpufreq_policy *policy)
{
    struct lap_policy_info *lp = policy->governor_data;
    struct lap_tuners *tuners;
    unsigned int load;
    battery_t battery_status;
    s64 current_power = 0;

    if (!lp)
        return HZ;

    tuners = &lp->tuners;
    mutex_lock(&lp->lock);

    lap_is_on_ac(&battery_status);

    if (battery_status.active) {
        /* Sanity check: ignore absurdly high power values during boot */
        if (battery_status.power_avg < 150000) { /* 150W limit */
            current_power = battery_status.power_avg;
        }
    }

    update_power_momentum_fixed(current_power);

    load = lap_dbs_update(policy, lp->tuners.ignore_nice_load);

    int load_delta = load - lp->prev_load;
    u8 ema_alpha = (load_delta + 100) / LAP_DEF_EMA_ALPHA_SCALING_FACTOR;
    ema_alpha = ema_alpha < 30 ? 30 : ema_alpha;
    lp->smoothed_load = (ema_alpha * load + (100 - ema_alpha) * lp->smoothed_load) / 100;

    adaptive_frequency_step_update(policy, m_fp, v_fp, lp->smoothed_load);

    lp->prev_load = load;
    mutex_unlock(&lp->lock);
    return (unsigned long)tuners->sampling_rate * HZ;
}

/* lap_work_handler - Delayed work handler per policy */
static void lap_work_handler(struct work_struct *work)
{
    struct lap_policy_info *lp = container_of(work, struct lap_policy_info, work.work);
    struct cpufreq_policy *policy = lp->policy;
    unsigned long delay_jiffies;

    delay_jiffies = cs_dbs_update(policy);
    schedule_delayed_work_on(policy->cpu, &lp->work, delay_jiffies);
}

/*
 * sysfs interface for governor tunables
 */

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
    ssize_t ret;

    if (!lp)
        return snprintf(buf, 2, "0\n");

    mutex_lock(&lp->lock);
    ret = snprintf(buf, 12, "%u\n", lp->tuners.sampling_rate);
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
    if (ret || val == 0)
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
        return snprintf(buf, 2, "0\n");

    mutex_lock(&lp->lock);
    ret = snprintf(buf, 12, "%u\n", lp->tuners.sampling_down_factor);
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
        return snprintf(buf, 2, "0\n");

    mutex_lock(&lp->lock);
    ret = snprintf(buf, 12, "%u\n", lp->tuners.up_threshold);
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
        return snprintf(buf, 2, "0\n");

    mutex_lock(&lp->lock);
    ret = snprintf(buf, 12, "%u\n", lp->tuners.down_threshold);
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
    if (ret)
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

/* ignore_nice_load */
static ssize_t show_ignore_nice_load(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;
    ssize_t ret;

    if (!lp)
        return snprintf(buf, 3, "0\n");

    mutex_lock(&lp->lock);
    ret = snprintf(buf, 3, "%u\n", lp->tuners.ignore_nice_load);
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

/* freq_step */
static ssize_t show_freq_step(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;
    ssize_t ret;

    if (!lp)
        return snprintf(buf, 3, "0\n");

    mutex_lock(&lp->lock);
    ret = snprintf(buf, 12, "%u\n", lp->tuners.freq_step);
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

/*
 * Governor lifecycle functions (init, exit, start, stop)
 */

/* lap_start - Called when the governor is activated for a policy */
static int lap_start(struct cpufreq_policy *policy)
{
    struct lap_policy_info *lp = policy->governor_data;
    unsigned long delay;
    battery_t battery_status;

    if (!lp)
        return -EINVAL;

    mutex_lock(&lp->lock);
    lp->requested_freq = policy->cur;
    lp->prev_load = 0;
    lp->idle_periods = 0;
    
    lap_is_on_ac(&battery_status);
    lp->prev_remaining = battery_status.remaining;

    /* Reset momentum values to start fresh */
    m_fp = 0;
    v_fp = 0;

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

    lp->tuners.up_threshold = LAP_DEF_UP_THRESHOLD;
    lp->tuners.down_threshold = LAP_DEF_DOWN_THRESHOLD;
    lp->tuners.freq_step = LAP_DEF_FREQ_STEP;
    lp->tuners.sampling_down_factor = LAP_DEF_SAMPLING_DOWN_FAC;
    lp->tuners.ignore_nice_load = 1;
    lp->tuners.sampling_rate = LAP_DEF_SAMPLING_RATE;

    lp->policy = policy;
    mutex_init(&lp->lock);
    cpumask_clear(&lp->eff_mask);
    cpumask_clear(&lp->perf_mask);
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

/*
 * Governor registration
 */

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
MODULE_DESCRIPTION("'cpufreq_laputil' - Conservative-style governor for laptops (with adam-inspired optimizer)");
MODULE_LICENSE("GPL");

module_init(laputil_module_init);
module_exit(laputil_module_exit);
