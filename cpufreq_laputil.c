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
    unsigned int io_load_pc;
    unsigned int eff_base_power_mw;
    unsigned int eff_scale_factor;
    unsigned int perf_base_power_mw;
    unsigned int perf_scale_factor;
};

struct lap_policy_info {
    struct cpufreq_policy *policy;
    unsigned int requested_eff_freq;
    unsigned int requested_perf_freq;
    unsigned int eff_prev_load;
    unsigned int perf_prev_load;
    unsigned int prev_remaining;
    unsigned int idle_periods;
    unsigned int perf_smoothed_load;
    unsigned int eff_smoothed_load;
    s64          eff_v_fp;
    s64          eff_m_fp;
    s64          eff_power_ema_fp;
    s64          perf_v_fp;
    s64          perf_m_fp;
    s64          perf_power_ema_fp;
    struct lap_tuners tuners;
    struct delayed_work work;
    struct mutex lock;
    struct cpumask eff_mask;
    struct cpumask perf_mask;
    unsigned int eff_cpus;
    unsigned int perf_cpus;
    unsigned int eff_load;
    unsigned int perf_load;
};

#define LAP_DEF_UP_THRESHOLD             75
#define LAP_DEF_DOWN_THRESHOLD           10
#define LAP_DEF_FREQ_STEP                 5
#define LAP_DEF_SAMPLING_DOWN_FAC         2
#define LAP_MAX_SAMPLING_DOWN_FAC         5
#define LAP_DEF_SAMPLING_RATE             1
#define LAP_MAX_FREQ_STEP_PERCENT        25
#define LAP_MIN_FREQ_STEP_PERCENT         5
#define LAP_DEF_LOAD_EMA_ALPHA_SCALING_FACTOR  3
#define LAP_DEF_POWER_EMA_ALPHA_FP (FP_SCALE / 10)

/* Function Prototypes */
static void update_mse_momentum_fixed(s64 gradient, s64 *m, s64 *v)
static void detect_clusters(struct cpufreq_policy *policy, struct cpumask *eff_mask, struct cpumask *perf_mask);
static inline unsigned int lap_get_freq_step_khz(struct lap_tuners *tuners, struct cpufreq_policy *policy);
static inline void lap_is_on_ac(battery_t *battery_status);
static u64 isqrt(u64 n);
static unsigned long cs_dbs_update(struct cpufreq_policy *policy);
static unsigned int lap_dbs_get_group_load(struct cpufreq_policy *policy, const struct cpumask *mask, bool ignore_nice);;;;
static unsigned int adaptive_frequency_step_update_group(struct cpufreq_policy *policy, unsigned int current_freq, s64 m, s64 v, unsigned int current_load);
static void lap_work_handler(struct work_struct *work);
static ssize_t show_sampling_rate(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t store_sampling_rate(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count);
static ssize_t show_sampling_down_factor(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t store_sampling_down_factor(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count);
static ssize_t show_up_threshold(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t store_up_threshold(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count);
static ssize_t show_down_threshold(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t store_down_threshold(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count);
static ssize_t show_ignore_nice_load(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t store_ignore_nice_load(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count);
static ssize_t show_freq_step(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t store_freq_step(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count);
static ssize_t show_io_load_pc(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t store_io_load_pc(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count);
static int lap_start(struct cpufreq_policy *policy);
static void lap_stop(struct cpufreq_policy *policy);
static int lap_init(struct cpufreq_policy *policy
static void lap_exit(struct cpufreq_policy *policy);
static int laputil_module_init(void);
static void laputil_module_exit(void);

/* update_mse_momentum_fixed - update momentum values from mse gradient */
static void update_mse_momentum_fixed(s64 gradient, s64 *m, s64 *v)
{
    s64 gradient_sq = div64_s64(gradient * gradient, FP_SCALE);
    *m = (((BETA1_FP * (*m)) >> FP_SHIFT) + ((ONE_MINUS_BETA1_FP * gradient) >> FP_SHIFT));
    *v = (((BETA2_FP * (*v)) >> FP_SHIFT) + ((ONE_MINUS_BETA2_FP * gradient_sq) >> FP_SHIFT));
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
static inline unsigned int lap_get_freq_step_khz(struct lap_tuners *tuners, struct cpufreq_policy *policy)
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
    bool found_ac = 0;
    battery_status->active = 1;
    battery_status->remaining = 100;
    battery_status->power_avg = 0;
    for (i = 0; i < ARRAY_SIZE(ac_names) && ac_names[i] != NULL; i++) {
        psy = power_supply_get_by_name(ac_names[i]);
        if (!psy)
            continue;
        if (power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE, &val) == 0 && val.intval) {
            found_ac = 1;
            power_supply_put(psy);
            break;
        }
        power_supply_put(psy);
    }
    if (found_ac) {
        battery_status->active = 0;
        return;
    }
    for (i = 0; i < ARRAY_SIZE(battery_names) && battery_names[i] != NULL; i++) {
        psy = power_supply_get_by_name(battery_names[i]);
        if (!psy)
            continue;
        if (power_supply_get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &val) == 0) {
            battery_status->remaining = val.intval;
            if (power_supply_get_property(psy, POWER_SUPPLY_PROP_POWER_AVG, &val) == 0) {
                battery_status->power_avg = val.intval / 1000;
            }
            power_supply_put(psy);
            return;
        }
        power_supply_put(psy);
    }
}

/* lap_dbs_get_group_load - Compute average load (0..100) for a specific CPU group. */

static unsigned int lap_dbs_get_group_load(struct cpufreq_policy *policy, const struct cpumask *mask, bool ignore_nice)
{
    unsigned int load_sum = 0;
    unsigned int cpu;
    unsigned int num_cpus = cpumask_weight(mask);
    u64 cur_time;
    unsigned int time_elapsed;
    unsigned int cur_load;
    u64 cur_idle, cur_nice;
    u64 idle_delta, nice_delta;

    if (!num_cpus)
        return 0;

    for_each_cpu(cpu, mask) {
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

        load_sum += cur_load;
    }
    return load_sum / num_cpus;
}

// isqrt - integer square root
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

/* adaptive_frequency_step_update_group - Determine next freq step for a group. */
static unsigned int adaptive_frequency_step_update_group(struct cpufreq_policy *policy, unsigned int current_freq, s64 m, s64 v, unsigned int current_load)
{
    struct lap_policy_info *lp = policy->governor_data;
    unsigned int requested_freq = current_freq;
    unsigned int step_khz = lap_get_freq_step_khz(&lp->tuners, policy);
    s64 update_vector = 0;
    s32 freq_change_scalar;
    s32 scaled_update;

    if (v > 0) {
        u64 sqrt_v = isqrt((u64)v >> FP_SHIFT);
        if (sqrt_v > 0) {
            update_vector = div64_s64(m, sqrt_v);
        }
    }

    if (abs(update_vector) > AFS_HIGH_THRESHOLD) {
        freq_change_scalar = 3;
    } else if (abs(update_vector) > AFS_LOW_THRESHOLD) {
        freq_change_scalar = 2;
    } else {
        freq_change_scalar = 1;
    }

    if (update_vector > 0) {
        scaled_update = freq_change_scalar;
    } else if (update_vector < 0) {
        scaled_update = -freq_change_scalar;
    } else {
        scaled_update = 0;
    }

    // Override Adam's decision based on explicit load thresholds
    _Bool override = 1;
    if (current_load > lp->tuners.up_threshold) {
        scaled_update = max_t(int, scaled_update, 1);
    } else if (current_load < lp->tuners.down_threshold) {
        scaled_update = min_t(int, scaled_update, -1);
    } else {
        override = 0;
    }

    if (scaled_update == 0 && !override) {
        return requested_freq;
    }

    if (scaled_update > 0) {
        requested_freq += (scaled_update * step_khz);
        if (requested_freq > policy->max)
            requested_freq = policy->max;
    } else {
        requested_freq -= ((-scaled_update) * step_khz);
        if (requested_freq < policy->min)
            requested_freq = policy->min;
    }

    return requested_freq;
}

/* get_io_load_pc - Get a percentage metric for I/O activity. */
static unsigned int get_io_load_pc(void)
{
    static unsigned long long prev_io_ops;
    unsigned long long cur_io_ops = kstat_read(NULL, KSTAT_IO_OPS_READ) +
                                   kstat_read(NULL, KSTAT_IO_OPS_WRITE);
    unsigned long long io_delta = cur_io_ops - prev_io_ops;
    prev_io_ops = cur_io_ops;

    // Scale the I/O delta to a percentage. This value is a heuristic.
    // Adjust the divisor (e.g., 1000) based on typical I/O rates.
    // A smaller divisor makes the governor more sensitive to I/O.
    return min((unsigned int)(io_delta / 1000), 1000U);
}

/* cs_dbs_update - apply Laputil decision logic for one policy */
static unsigned long cs_dbs_update(struct cpufreq_policy *policy)
{
    struct lap_policy_info *lp = policy->governor_data;
    struct lap_tuners *tuners;
    s64 current_power = 0;
    s64 power_gradient;
    battery_t battery_status;
    unsigned int eff_load, perf_load;
    unsigned int new_eff_freq, new_perf_freq;

    if (!lp)
        return HZ;

    tuners = &lp->tuners;
    mutex_lock(&lp->lock);

    // dynamic power target
    lp->io_load_ac = get_io_load_pc();
    lap_is_on_ac(&battery_status);
    if (battery_status.active) {
        if (battery_status.power_avg < 150000) {
            current_power = battery_status.power_avg;
        }
    }
    
    // Power consumption is a policy-wide metric.
    if (lp->eff_power_ema_fp == 0) {
        lp->eff_power_ema_fp = current_power << FP_SHIFT;
        lp->perf_power_ema_fp = current_power << FP_SHIFT;
    } else {
        s64 alpha_term = div64_s64(LAP_DEF_POWER_EMA_ALPHA_FP * (current_power << FP_SHIFT), FP_SCALE);
        s64 eff_one_minus_alpha_term = div64_s64((FP_SCALE - LAP_DEF_POWER_EMA_ALPHA_FP) * lp->eff_power_ema_fp, FP_SCALE);
        s64 perf_one_minus_alpha_term = div64_s64((FP_SCALE - LAP_DEF_POWER_EMA_ALPHA_FP) * lp->perf_power_ema_fp, FP_SCALE);
        lp->eff_power_ema_fp = alpha_term + eff_one_minus_alpha_term;
        lp->perf_power_ema_fp = alpha_term + perf_one_minus_alpha_term;
    }
    
    s64 power_error = (s64)lp->tuners.io_load_pc << FP_SHIFT;
    power_gradient = (power_error - lp->eff_power_ema_fp) >> FP_SHIFT;

    update_mse_momentum_fixed(power_gradient, lp->eff_m_fp, lp->eff_v_fp);
    update_mse_momentum_fixed(power_gradient, lp->perf_m_fp, lp->perf_v_fp);

    eff_load = lap_dbs_get_group_load(policy, &lp->eff_mask, lp->tuners.ignore_nice_load);
    perf_load = lap_dbs_get_group_load(policy, &lp->perf_mask, lp->tuners.ignore_nice_load);
    
    int eff_load_delta = eff_load - lp->eff_prev_load;
    u8 eff_ema_alpha = (eff_load_delta + 100) / LAP_DEF_LOAD_EMA_ALPHA_SCALING_FACTOR;
    eff_ema_alpha = eff_ema_alpha < 30 ? 30 : eff_ema_alpha;
    lp->eff_smoothed_load = (eff_ema_alpha * eff_load + (100 - eff_ema_alpha) * lp->eff_smoothed_load) / 100;
    
    int perf_load_delta = perf_load - lp->perf_prev_load;
    u8 perf_ema_alpha = (perf_load_delta + 100) / LAP_DEF_LOAD_EMA_ALPHA_SCALING_FACTOR;
    perf_ema_alpha = perf_ema_alpha < 30 ? 30 : perf_ema_alpha;
    lp->perf_smoothed_load = (perf_ema_alpha * perf_load + (100 - perf_ema_alpha) * lp->perf_smoothed_load) / 100;

    new_eff_freq = adaptive_frequency_step_update_group(policy, lp->requested_eff_freq, lp->eff_m_fp, lp->eff_v_fp, lp->eff_smoothed_load);
    new_perf_freq = adaptive_frequency_step_update_group(policy, lp->requested_perf_freq, lp->perf_m_fp, lp->perf_v_fp, lp->perf_smoothed_load);
    
    if (lp->perf_smoothed_load > lp->tuners.up_threshold) {
        cpufreq_driver_target(policy, new_perf_freq, CPUFREQ_RELATION_H);
    } else if (lp->eff_smoothed_load < lp->tuners.down_threshold) {
        cpufreq_driver_target(policy, new_eff_freq, CPUFREQ_RELATION_L);
    } else {
        unsigned int blended_freq = (new_eff_freq + new_perf_freq) / 2;
        cpufreq_driver_target(policy, blended_freq, CPUFREQ_RELATION_L);
    }

    if (new_perf_freq > policy->cur && lp->perf_smoothed_load > lp->tuners.up_threshold) {
        cpufreq_driver_target(policy, new_perf_freq, CPUFREQ_RELATION_H);
    } else if (new_eff_freq < policy->cur && lp->eff_smoothed_load < lp->tuners.down_threshold) {
        cpufreq_driver_target(policy, new_eff_freq, CPUFREQ_RELATION_L);
    } else {
        // Fallback to a blended or no-op decision
        if (new_eff_freq != policy->cur && new_perf_freq != policy->cur) {
             cpufreq_driver_target(policy, policy->cur, CPUFREQ_RELATION_L);
        }
    }
    
    lp->requested_eff_freq = new_eff_freq;
    lp->requested_perf_freq = new_perf_freq;
    lp->eff_prev_load = eff_load;
    lp->perf_prev_load = perf_load;
    
    mutex_unlock(&lp->lock);
    return (unsigned long)tuners->sampling_rate * HZ;
}

// lap_work_handler - Delayed work handler per policy
static void lap_work_handler(struct work_struct *work)
{
    struct lap_policy_info *lp = container_of(work, struct lap_policy_info, work.work);
    struct cpufreq_policy *policy = lp->policy;
    unsigned long delay_jiffies;
    delay_jiffies = cs_dbs_update(policy);
    schedule_delayed_work_on(policy->cpu, &lp->work, delay_jiffies);
}

// sysfs interface for governor tunables
#define lap_gov_attr(_name) \
static struct kobj_attribute _name##_attr = { \
    .attr = { .name = #_name, .mode = 0644 }, \
    .show = show_##_name, \
    .store = store_##_name, \
}

static ssize_t show_io_load_pc(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;
    ssize_t ret;
    if (!lp)
        return snprintf(buf, 2, "0\n");
    mutex_lock(&lp->lock);
    ret = snprintf(buf, 12, "%u\n", lp->tuners.io_load_pc);
    mutex_unlock(&lp->lock);
    return ret;
}

static ssize_t store_io_load_pc(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    unsigned int val;
    int ret;
    const struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;
    if (!lp)
        return -EINVAL;
    ret = kstrtouint(buf, 10, &val);
    if (ret)
        return -EINVAL;
    mutex_lock(&lp->lock);
    lp->tuners.io_load_pc = val;
    mutex_unlock(&lp->lock);
    return count;
}

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

static ssize_t store_sampling_rate(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
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

static ssize_t store_sampling_down_factor(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
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

static ssize_t store_up_threshold(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
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

static ssize_t store_down_threshold(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
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

static ssize_t store_ignore_nice_load(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    unsigned int val;
    int ret;
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;
    if (!lp)
        return -EINVAL;
    ret = kstrtouint(buf, 10, &val);
    if (ret || val > 1)
        return -EINVAL;
    mutex_lock(&lp->lock);
    lp->tuners.ignore_nice_load = val;
    mutex_unlock(&lp->lock);
    return count;
}

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

static ssize_t store_freq_step(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    unsigned int val;
    int ret;
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;
    if (!lp)
        return -EINVAL;
    ret = kstrtouint(buf, 10, &val);
    if (ret || val < LAP_MIN_FREQ_STEP_PERCENT || val > LAP_MAX_FREQ_STEP_PERCENT)
        return -EINVAL;
    mutex_lock(&lp->lock);
    lp->tuners.freq_step = val;
    mutex_unlock(&lp->lock);
    return count;
}

static struct kobj_attribute io_load_pc_attr = {
    .attr = {.name = "io_load_pc", .mode = 0644},
    .show = show_io_load_pc,
    .store = store_io_load_pc,
};

static struct kobj_attribute sampling_rate_attr = {
    .attr = {.name = "sampling_rate", .mode = 0644},
    .show = show_sampling_rate,
    .store = store_sampling_rate,
};

static struct kobj_attribute sampling_down_factor_attr = {
    .attr = {.name = "sampling_down_factor", .mode = 0644},
    .show = show_sampling_down_factor,
    .store = store_sampling_down_factor,
};

static struct kobj_attribute up_threshold_attr = {
    .attr = {.name = "up_threshold", .mode = 0644},
    .show = show_up_threshold,
    .store = store_up_threshold,
};

static struct kobj_attribute down_threshold_attr = {
    .attr = {.name = "down_threshold", .mode = 0644},
    .show = show_down_threshold,
    .store = store_down_threshold,
};

static struct kobj_attribute ignore_nice_load_attr = {
    .attr = {.name = "ignore_nice_load", .mode = 0644},
    .show = show_ignore_nice_load,
    .store = store_ignore_nice_load,
};

static struct kobj_attribute freq_step_attr = {
    .attr = {.name = "freq_step", .mode = 0644},
    .show = show_freq_step,
    .store = store_freq_step,
};

static struct attribute *lap_attrs[] = {
    &freq_step_attr.attr,
    &ignore_nice_load_attr.attr,
    &sampling_down_factor_attr.attr,
    &sampling_rate_attr.attr,
    &up_threshold_attr.attr,
    &down_threshold_attr.attr,
    &io_load_pc_attr.attr,
    NULL
};

static struct attribute_group lap_attr_group = {
    .attrs = lap_attrs,
    .name = "laputil"
};

/**
 * @brief Starts the governor for a policy.
 * @param policy CPU frequency policy.
 * @return 0 on success, negative errno on failure.
 */
static int lap_start(struct cpufreq_policy *policy)
{
    struct lap_policy_info *lp = policy->governor_data;
    if (!lp)
        return -EINVAL;
    mutex_lock(&lp->lock);
    lp->requested_eff_freq = policy->cur;
    lp->requested_perf_freq = policy->cur;
    lp->perf_prev_load = 0;
    lp->eff_prev_load = 0;
    lp->idle_periods = 0;
    lp->perf_smoothed_load = 0;
    lp->eff_smoothed_load = 0;
    lp->eff_m_fp = 0;
    lp->eff_v_fp = 0;
    lp->eff_power_ema_fp = 0;
    lp->perf_m_fp = 0;
    lp->perf_v_fp = 0;
    lp->perf_power_ema_fp = 0;
    if (cpumask_empty(&lp->eff_mask) || cpumask_empty(&lp->perf_mask))
        detect_clusters(policy, &lp->eff_mask, &lp->perf_mask);
    schedule_delayed_work_on(policy->cpu, &lp->work, 0);
    mutex_unlock(&lp->lock);
    return 0;
}

/**
 * @brief Stops the governor for a policy.
 * @param policy CPU frequency policy.
 */
static void lap_stop(struct cpufreq_policy *policy)
{
    struct lap_policy_info *lp = policy->governor_data;
    if (!lp)
        return;
    cancel_delayed_work_sync(&lp->work);
}

/* get_dynamic_power_target - Calculates a dynamic power target for a group based on its load. */
static unsigned int get_dynamic_power_target(struct lap_policy_info *lp, unsigned int group_load, bool is_eff_core)
{
    unsigned int base_power, scale_factor;

    if (is_eff_core) {
        base_power = lp->tuners.eff_base_power_mw;
        scale_factor = lp->tuners.eff_scale_factor;
    } else {
        base_power = lp->tuners.perf_base_power_mw;
        scale_factor = lp->tuners.perf_scale_factor;
    }

    return base_power + (group_load * scale_factor);
}

/* @brief Initializes the governor for a policy. */
static int lap_init(struct cpufreq_policy *policy)
{
    struct lap_policy_info *lp;
    int ret;
    lp = kzalloc(sizeof(*lp), GFP_KERNEL);
    if (!lp)
        return -ENOMEM;
    INIT_DELAYED_WORK(&lp->work, lap_work_handler);
    lp->policy = policy;
    mutex_init(&lp->lock);

    detect_clusters(policy, &lp->eff_mask, &lp->perf_mask);

    for_each_cpu(cpu, &lp->eff_mask)
        eff_max_freq = max(eff_max_freq, cpufreq_quick_get_max(cpu));
    for_each_cpu(cpu, &lp->perf_mask)
        perf_max_freq = max(perf_max_freq, cpufreq_quick_get_max(cpu));

    lp->tuners.eff_base_power_mw = eff_max_freq / 1000;
    lp->tuners.eff_scale_factor = (eff_max_freq / 10000);
    lp->tuners.perf_base_power_mw = perf_max_freq / 1000;
    lp->tuners.perf_scale_factor = (perf_max_freq / 10000);

    lp->tuners.up_threshold = LAP_DEF_UP_THRESHOLD;
    lp->tuners.down_threshold = LAP_DEF_DOWN_THRESHOLD;
    lp->tuners.freq_step = LAP_DEF_FREQ_STEP;
    lp->tuners.sampling_down_factor = LAP_DEF_SAMPLING_DOWN_FAC;
    lp->tuners.ignore_nice_load = 1;
    lp->tuners.sampling_rate = LAP_DEF_SAMPLING_RATE;
    cpumask_clear(&lp->eff_mask);
    cpumask_clear(&lp->perf_mask);
    policy->governor_data = lp;
    ret = sysfs_create_group(&policy->kobj, &lap_attr_group);
    if (ret) {
        kfree(lp);
        policy->governor_data = NULL;
        return ret;
    }
    return 0;
}

/**
 * @brief Exits the governor for a policy.
 * @param policy CPU frequency policy.
 */
static void lap_exit(struct cpufreq_policy *policy)
{
    struct lap_policy_info *lp = policy->governor_data;
    if (!lp)
        return;
    sysfs_remove_group(&policy->kobj, &lap_attr_group);
    kfree(lp);
}

static struct cpufreq_governor lap_governor = {
    .name = "laputil",
    .flags = 0,
    .init = lap_init,
    .exit = lap_exit,
    .start = lap_start,
    .stop = lap_stop,
};

static int __init laputil_module_init(void)
{
    return cpufreq_register_governor(&lap_governor);
}

static void __exit laputil_module_exit(void)
{
    cpufreq_unregister_governor(&lap_governor);
}

MODULE_AUTHOR("Lee Yunjin <gzblues61@daum.net>");
MODULE_DESCRIPTION("'cpufreq_laputil' - Conservative-style governor for laptops (with adam-inspired optimizer)");
MODULE_LICENSE("GPL");

module_init(laputil_module_init);
module_exit(laputil_module_exit);
