/*
 * drivers/cpufreq/cpufreq_laputil.c
 * laputil: laptop-oriented conservative governor
 *
 * Copyright (C) 2025 Lee Yunjin <gzblues61@daum.net>
 *
 * Conservative-style governor with Adam-inspired optimizer for power efficiency.
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

#define AFS_HIGH_THRESHOLD 200
#define AFS_LOW_THRESHOLD  50

// Adam learning rate
#define LAP_DEF_LEARNING_RATE_FP (FP_SCALE / 100)
#define LAP_MAX_LEARNING_RATE_FP (FP_SCALE)

// New: Target load for the optimizer
#define LAP_DEF_TARGET_LOAD 50

// Dynamic tuning values for AC vs. Battery
#define LAP_AC_LEARNING_RATE_FP (FP_SCALE / 50)  // 0.02
#define LAP_BATTERY_LEARNING_RATE_FP (FP_SCALE / 200) // 0.005
#define LAP_AC_FREQ_STEP 10
#define LAP_BATTERY_FREQ_STEP 3
#define LAP_AC_TARGET_LOAD 70
#define LAP_BATTERY_TARGET_LOAD 30

struct lap_cpu_dbs {
    u64 prev_cpu_idle;
    u64 prev_cpu_nice;
    u64 prev_update_time;
};

DEFINE_PER_CPU(struct lap_cpu_dbs, lap_cpu_dbs);

// Global data for system-wide load calculation
static DEFINE_MUTEX(lap_global_lock);
static unsigned int lap_global_load;
static bool lap_on_ac_power;

struct lap_tuners {
    unsigned int freq_step;
    unsigned int sampling_down_factor;
    unsigned int ignore_nice_load;
    unsigned int sampling_rate;
    s64 learning_rate_fp;
    unsigned int target_load;
};

struct lap_policy_info {
    struct cpufreq_policy *policy;
    unsigned int requested_freq;
    unsigned int prev_load;
    s64          v_fp;
    s64          m_fp;
    struct lap_tuners tuners;
    struct delayed_work work;
    struct mutex lock;
};

#define LAP_DEF_FREQ_STEP          5
#define LAP_MAX_FREQ_STEP_PERCENT  25
#define LAP_MIN_FREQ_STEP_PERCENT  5
#define LAP_DEF_SAMPLING_DOWN_FAC  2
#define LAP_MAX_SAMPLING_DOWN_FAC  5
#define LAP_DEF_SAMPLING_RATE      1
#define LAP_DEF_LOAD_EMA_ALPHA_SCALING_FACTOR 3

/* Function Prototypes */
static void update_adam_momentum(s64 gradient, struct lap_policy_info *lp);
static inline unsigned int lap_get_freq_step_khz(struct lap_tuners *tuners, struct cpufreq_policy *policy);
static unsigned int lap_dbs_get_load(bool ignore_nice);
static u64 isqrt(u64 n);
static void lap_apply_adam_policy(struct cpufreq_policy *policy, struct lap_policy_info *lp);
static unsigned long cs_dbs_update(struct cpufreq_policy *policy);
static void lap_work_handler(struct work_struct *work);

// Sysfs functions
static ssize_t show_sampling_rate(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t store_sampling_rate(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count);
static ssize_t show_sampling_down_factor(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t store_sampling_down_factor(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count);
static ssize_t show_ignore_nice_load(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t store_ignore_nice_load(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count);
static ssize_t show_freq_step(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t store_freq_step(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count);
static ssize_t show_learning_rate(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t store_learning_rate(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count);
static ssize_t show_target_load(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t store_target_load(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count);
static int lap_start(struct cpufreq_policy *policy);
static void lap_stop(struct cpufreq_policy *policy);
static int lap_init(struct cpufreq_policy *policy);
static void lap_exit(struct cpufreq_policy *policy);
static int laputil_module_init(void);
static void laputil_module_exit(void);

/* update_adam_momentum - Update Adam momentum values from load gradient */
static void update_adam_momentum(s64 gradient, struct lap_policy_info *lp)
{
    s64 gradient_sq = div64_s64(gradient * gradient, FP_SCALE);
    lp->m_fp = (((BETA1_FP * lp->m_fp) >> FP_SHIFT) +
            ((ONE_MINUS_BETA1_FP * gradient) >> FP_SHIFT));
    lp->v_fp = (((BETA2_FP * lp->v_fp) >> FP_SHIFT) +
            ((ONE_MINUS_BETA2_FP * gradient_sq) >> FP_SHIFT));
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

/* lap_dbs_get_load - compute average load (0..100) across all online CPUs */
static unsigned int lap_dbs_get_load(bool ignore_nice)
{
    unsigned int load_sum = 0;
    unsigned int cpu;
    u64 cur_time;
    unsigned int time_elapsed;
    unsigned int cur_load;
    u64 cur_idle, cur_nice;
    u64 idle_delta, nice_delta;

    for_each_online_cpu(cpu) {
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

    if (unlikely(num_online_cpus() == 0))
        return 0;

    return load_sum / num_online_cpus();
}

/* lap_is_on_ac - Retrieves AC status and updates governor state */
static void lap_is_on_ac(void)
{
    struct power_supply *psy;
    union power_supply_propval val;
    int i;
    
    lap_on_ac_power = false;

    for (i = 0; i < ARRAY_SIZE(ac_names) && ac_names[i] != NULL; i++) {
        psy = power_supply_get_by_name(ac_names[i]);
        if (!psy)
            continue;
        
        if (power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE, &val) == 0 && val.intval) {
            lap_on_ac_power = true;
        }
        power_supply_put(psy);

        if (lap_on_ac_power) {
            return;
        }
    }
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

/* lap_apply_adam_policy - determine next freq step via optimizer */
static void lap_apply_adam_policy(struct cpufreq_policy *policy, struct lap_policy_info *lp)
{
    unsigned int requested_freq = lp->requested_freq;
    unsigned int step_khz = lap_get_freq_step_khz(&lp->tuners, policy);
    s64 update_vector = 0;
    s64 trend_fp = lp->m_fp;
    s64 volatility_fp = lp->v_fp;
    s64 scaled_update_fp;
    
    if (volatility_fp > 0) {
        u64 sqrt_v = isqrt((u64)volatility_fp >> FP_SHIFT);
        if (sqrt_v > 0) {
            s64 scaled_trend = (lp->tuners.learning_rate_fp * trend_fp) >> FP_SHIFT;
            update_vector = div64_s64(scaled_trend, sqrt_v);
        }
    }
    
    scaled_update_fp = clamp_t(s64, update_vector, -AFS_HIGH_THRESHOLD, AFS_HIGH_THRESHOLD);
    
    requested_freq += (scaled_update_fp * step_khz) >> FP_SHIFT;
    
    requested_freq = clamp_val(requested_freq, policy->min, policy->max);

    if (requested_freq != lp->requested_freq) {
        cpufreq_driver_target(policy, requested_freq, CPUFREQ_RELATION_L);
        lp->requested_freq = requested_freq;
    }
}

/* cs_dbs_update - apply Laputil decision logic for one policy */
static unsigned long cs_dbs_update(struct cpufreq_policy *policy)
{
    struct lap_policy_info *lp = policy->governor_data;
    struct lap_tuners *tuners;
    unsigned int load;
    s64 load_error;
    
    if (!lp)
        return HZ;
    tuners = &lp->tuners;
    mutex_lock(&lp->lock);
    
    // Step 1: Update power source status (only on CPU0 to prevent race conditions)
    if (policy->cpu == 0) {
        lap_is_on_ac();
    }
    
    // Step 2: Dynamically adjust tuners based on power source
    if (lap_on_ac_power) {
        lp->tuners.learning_rate_fp = LAP_AC_LEARNING_RATE_FP;
        lp->tuners.freq_step = LAP_AC_FREQ_STEP;
        lp->tuners.target_load = LAP_AC_TARGET_LOAD;
    } else {
        lp->tuners.learning_rate_fp = LAP_BATTERY_LEARNING_RATE_FP;
        lp->tuners.freq_step = LAP_BATTERY_FREQ_STEP;
        lp->tuners.target_load = LAP_BATTERY_TARGET_LOAD;
    }

    // Step 3: Compute system-wide CPU load (Only one policy updates)
    mutex_lock(&lap_global_lock);
    lap_global_load = lap_dbs_get_load(lp->tuners.ignore_nice_load);
    mutex_unlock(&lap_global_lock);

    load = lap_global_load;
    
    int load_delta = load - lp->prev_load;
    u8 ema_alpha = (load_delta + 100) / LAP_DEF_LOAD_EMA_ALPHA_SCALING_FACTOR;
    ema_alpha = ema_alpha < 30 ? 30 : ema_alpha;
    
    load_error = load - lp->tuners.target_load;
    
    update_adam_momentum(load_error, lp);

    lap_apply_adam_policy(policy, lp);

    lp->prev_load = load;
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

// Show target load.
static ssize_t show_target_load(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;
    ssize_t ret;
    if (!lp)
        return snprintf(buf, 2, "0\n");
    mutex_lock(&lp->lock);
    ret = snprintf(buf, 12, "%u\n", lp->tuners.target_load);
    mutex_unlock(&lp->lock);
    return ret;
}

// Store target load.
static ssize_t store_target_load(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
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
    lp->tuners.target_load = val;
    mutex_unlock(&lp->lock);
    return count;
}

// Show learning rate.
static ssize_t show_learning_rate(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;
    ssize_t ret;
    if (!lp)
        return snprintf(buf, 2, "0\n");
    mutex_lock(&lp->lock);
    ret = snprintf(buf, 12, "%llu\n", (unsigned long long)lp->tuners.learning_rate_fp * 1000 / FP_SCALE);
    mutex_unlock(&lp->lock);
    return ret;
}

// Store learning rate.
static ssize_t store_learning_rate(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    unsigned int val;
    int ret;
    struct cpufreq_policy *policy = container_of(kobj, struct cpufreq_policy, kobj);
    struct lap_policy_info *lp = policy->governor_data;
    if (!lp)
        return -EINVAL;
    ret = kstrtouint(buf, 10, &val);
    if (ret || val == 0 || val > 1000)
        return -EINVAL;
    mutex_lock(&lp->lock);
    lp->tuners.learning_rate_fp = (s64)val * FP_SCALE / 1000;
    mutex_unlock(&lp->lock);
    return count;
}

// Show sampling rate.
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

// Store sampling rate.
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

// Show sampling down factor.
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

// Store sampling down factor.
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

// Show ignore nice load.
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

// Store ignore nice load.
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

// Show frequency step.
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

// Store frequency step.
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

static struct kobj_attribute learning_rate_attr = {
    .attr = {.name = "learning_rate", .mode = 0644},
    .show = show_learning_rate,
    .store = store_learning_rate,
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

static struct kobj_attribute target_load_attr = {
    .attr = {.name = "target_load", .mode = 0644},
    .show = show_target_load,
    .store = store_target_load,
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
    &target_load_attr.attr,
    &learning_rate_attr.attr,
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
    struct lap_policy_info *lp;
    lp = kzalloc(sizeof(*lp), GFP_KERNEL);
    if (!lp)
        return -ENOMEM;
    INIT_DELAYED_WORK(&lp->work, lap_work_handler);
    lp->policy = policy;
    mutex_init(&lp->lock);
    lp->tuners.freq_step = LAP_DEF_FREQ_STEP;
    lp->tuners.sampling_down_factor = LAP_DEF_SAMPLING_DOWN_FAC;
    lp->tuners.ignore_nice_load = 1;
    lp->tuners.sampling_rate = LAP_DEF_SAMPLING_RATE;
    lp->tuners.learning_rate_fp = LAP_DEF_LEARNING_RATE_FP;
    lp->tuners.target_load = LAP_DEF_TARGET_LOAD;
    policy->governor_data = lp;
    if (sysfs_create_group(&policy->kobj, &lap_attr_group)) {
        kfree(lp);
        policy->governor_data = NULL;
        return -EINVAL;
    }
    schedule_delayed_work_on(policy->cpu, &lp->work, 0);
    return 0;
}

/**
 * @brief Stops the governor for a policy.
 * @param policy CPU frequency policy.
 */
static void lap_stop(struct cpufreq_policy *policy)
{
    struct lap_policy_info *lp = policy->governor_data;
    if (lp) {
        cancel_delayed_work_sync(&lp->work);
        sysfs_remove_group(&policy->kobj, &lap_attr_group);
        kfree(lp);
        policy->governor_data = NULL;
    }
}

/**
 * @brief Initializes the governor for a policy.
 * @param policy CPU frequency policy.
 * @return 0 on success, negative errno on failure.
 */
static int lap_init(struct cpufreq_policy *policy)
{
    return 0;
}

/**
 * @brief Exits the governor for a policy.
 * @param policy CPU frequency policy.
 */
static void lap_exit(struct cpufreq_policy *policy)
{
    return;
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
