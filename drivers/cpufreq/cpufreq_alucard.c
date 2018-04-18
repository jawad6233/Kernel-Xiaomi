/*
 *  drivers/cpufreq/cpufreq_alucard.c
 *
 *  Copyright (C)  2011 Samsung Electronics co. ltd
 *    ByungChang Cha <bc.cha@samsung.com>
 *
 *  Based on ondemand governor
 *  Copyright (C)  2001 Russell King
 *            (C)  2003 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>.
 *                      Jun Nakajima <jun.nakajima@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Created by Alucard_24@xda
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/ktime.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/display_state.h>
#include <asm/cputime.h>

struct cpufreq_alucard_policyinfo {
	struct timer_list policy_timer;
	struct timer_list policy_slack_timer;
	spinlock_t load_lock; /* protects load tracking stat */
	u64 last_evaluated_jiffy;
	struct cpufreq_policy *policy;
	struct cpufreq_frequency_table *freq_table;
	spinlock_t target_freq_lock; /*protects target freq */
	unsigned int target_freq;
	unsigned int min_freq;
	struct rw_semaphore enable_sem;
	bool reject_notification;
	int governor_enabled;
	struct cpufreq_alucard_tunables *cached_tunables;
	unsigned long *cpu_busy_times;
	unsigned int up_rate;
	unsigned int down_rate;
};

/* Tuning Interface */
#ifdef CONFIG_MACH_LGE
#define FREQ_RESPONSIVENESS		2265600
#else
#define FREQ_RESPONSIVENESS		998400
#endif

#define CPUS_DOWN_RATE			3
#define CPUS_UP_RATE			1

#define DEC_CPU_LOAD			90
#define DEC_CPU_LOAD_AT_MIN_FREQ	60

#define INC_CPU_LOAD			90
#define INC_CPU_LOAD_AT_MIN_FREQ	60


#define FREQ_RESPONSIVENESS		 768000
#define FREQ_RESPONSIVENESS_MAX		1094400
#define FREQ_RESPONSIVENESS_MAX_BIGC	1401000

#define CPUS_DOWN_RATE				1
#define CPUS_UP_RATE				1

static void do_alucard_timer(struct work_struct *work);

struct cpufreq_alucard_tunables {
	int usage_count;
	/*
	 * The sample rate of the timer used to increase frequency
	 */
	unsigned long timer_rate;
	unsigned long timer_rate_prev;
	/*
	 * Max additional time to wait in idle, beyond timer_rate, at speeds
	 * above minimum before wakeup to reduce speed, or -1 if unnecessary.
	 */
#define DEFAULT_TIMER_SLACK (4 * DEFAULT_TIMER_RATE)
	int timer_slack_val;
	bool io_is_busy;
	/*
	 * Whether to align timer windows across all CPUs.
	 */
	bool align_windows;
	/*
	 * CPUs frequency scaling
	 */
	int freq_responsiveness;
	int freq_responsiveness_max;
	unsigned int cpus_up_rate_at_max_freq;
	unsigned int cpus_up_rate;
	unsigned int cpus_down_rate_at_max_freq;
	unsigned int cpus_down_rate;
	int pump_inc_step;
	int pump_inc_step_at_min_freq;
	int pump_dec_step;
	int pump_dec_step_at_min_freq;
};

/* For cases where we have single governor instance for system */
static struct cpufreq_alucard_tunables *common_tunables;
static struct cpufreq_alucard_tunables *cached_common_tunables;

static struct attribute_group *get_sysfs_attr(void);

/* Round to starting jiffy of next evaluation window */
static u64 round_to_nw_start(u64 jif,
			     struct cpufreq_alucard_tunables *tunables)
{
	unsigned long step = usecs_to_jiffies(tunables->timer_rate);
	u64 ret;

	if (tunables->align_windows) {
		do_div(jif, step);
		ret = (jif + 1) * step;
	} else {
		ret = jiffies + usecs_to_jiffies(tunables->timer_rate);
	}

	return ret;
}

static void cpufreq_alucard_timer_resched(unsigned long cpu,
					      bool slack_only)
{
	struct cpufreq_alucard_policyinfo *ppol = per_cpu(polinfo, cpu);
	struct cpufreq_alucard_cpuinfo *pcpu;
	struct cpufreq_alucard_tunables *tunables =
		ppol->policy->governor_data;
	u64 expires;
	unsigned long flags;
	int i;

	spin_lock_irqsave(&ppol->load_lock, flags);
	expires = round_to_nw_start(ppol->last_evaluated_jiffy, tunables);
	if (!slack_only) {
		for_each_cpu(i, ppol->policy->cpus) {
			pcpu = &per_cpu(cpuinfo, i);
			pcpu->time_in_idle = get_cpu_idle_time(i,
						&pcpu->time_in_idle_timestamp,
						tunables->io_is_busy);
		}
		del_timer(&ppol->policy_timer);
		ppol->policy_timer.expires = expires;
		add_timer(&ppol->policy_timer);
	}

	if (tunables->timer_slack_val >= 0 &&
	    ppol->target_freq > ppol->policy->min) {
		expires += usecs_to_jiffies(tunables->timer_slack_val);
		del_timer(&ppol->policy_slack_timer);
		ppol->policy_slack_timer.expires = expires;
		add_timer(&ppol->policy_slack_timer);
	}

	spin_unlock_irqrestore(&ppol->load_lock, flags);
}

/* The caller shall take enable_sem write semaphore to avoid any timer race.
 * The policy_timer and policy_slack_timer must be deactivated when calling
 * this function.
 */
static void cpufreq_alucard_timer_start(
	struct cpufreq_alucard_tunables *tunables, int cpu)
{
	struct cpufreq_alucard_policyinfo *ppol = per_cpu(polinfo, cpu);
	struct cpufreq_alucard_cpuinfo *pcpu;
	u64 expires = round_to_nw_start(ppol->last_evaluated_jiffy, tunables);
	unsigned long flags;
	int i;

	spin_lock_irqsave(&ppol->load_lock, flags);
	ppol->policy_timer.expires = expires;
	add_timer(&ppol->policy_timer);
	if (tunables->timer_slack_val >= 0 &&
	    ppol->target_freq > ppol->policy->min) {
		expires += usecs_to_jiffies(tunables->timer_slack_val);
		ppol->policy_slack_timer.expires = expires;
		add_timer(&ppol->policy_slack_timer);
	}

	for_each_cpu(i, ppol->policy->cpus) {
		pcpu = &per_cpu(cpuinfo, i);
		pcpu->time_in_idle =
			get_cpu_idle_time(i, &pcpu->time_in_idle_timestamp,
					  tunables->io_is_busy);
	}
	spin_unlock_irqrestore(&ppol->load_lock, flags);
}

static unsigned int choose_target_freq(struct cpufreq_alucard_policyinfo *pcpu,
					unsigned int step, bool isup)
{
	struct cpufreq_policy *policy = pcpu->policy;
	struct cpufreq_frequency_table *table = pcpu->freq_table;
	struct cpufreq_frequency_table *pos;
	unsigned int target_freq = 0, freq;
	int i = 0, t = 0;

	if (!policy || !table || !step)
		return 0;

	cpufreq_for_each_valid_entry(pos, table) {
		freq = pos->frequency;
		i = pos - table;
		if (isup) {
			if (freq > policy->cur) {
				target_freq = freq;
				step--;
				if (step == 0) {
					break;
				}
			}
		} else {
			if (freq == policy->cur) {
				for (t = (i - 1); t >= 0; t--) {
					if (table[t].frequency != CPUFREQ_ENTRY_INVALID) {
						target_freq = table[t].frequency;
						step--;
						if (step == 0) {
							break;
						}
					}
				}
				break;
			}
		}
	}
	
	return target_freq;
}

static bool update_load(int cpu)
{
	struct cpufreq_alucard_policyinfo *ppol = per_cpu(polinfo, cpu);
	struct cpufreq_alucard_cpuinfo *pcpu = &per_cpu(cpuinfo, cpu);
	struct cpufreq_alucard_tunables *tunables =
		ppol->policy->governor_data;
	u64 now;
	u64 now_idle;
	unsigned int delta_idle;
	unsigned int delta_time;
	bool ignore = false;

	now_idle = get_cpu_idle_time(cpu, &now, tunables->io_is_busy);
	delta_idle = (unsigned int)(now_idle - pcpu->time_in_idle);
	delta_time = (unsigned int)(now - pcpu->time_in_idle_timestamp);

	WARN_ON_ONCE(!delta_time);

	if (delta_time < delta_idle) {
		pcpu->load = 0;
		ignore = true;
	} else {
		pcpu->load = 100 * (delta_time - delta_idle);
		do_div(pcpu->load, delta_time);
	}
	pcpu->time_in_idle = now_idle;
	pcpu->time_in_idle_timestamp = now;

	return ignore;
}

static void cpufreq_alucard_timer(unsigned long data)
{
	struct cpufreq_alucard_policyinfo *ppol = per_cpu(polinfo, data);
	struct cpufreq_alucard_tunables *tunables =
		ppol->policy->governor_data;
	struct cpufreq_alucard_cpuinfo *pcpu;
	struct cpufreq_govinfo govinfo;
	unsigned int freq_responsiveness = tunables->freq_responsiveness;
	unsigned int freq_responsiveness_max = tunables->freq_responsiveness_max;
	int target_cpu_load;
	int pump_inc_step = tunables->pump_inc_step;
	int pump_dec_step = tunables->pump_dec_step;
	unsigned int cpus_up_rate = tunables->cpus_up_rate;
	unsigned int cpus_down_rate = tunables->cpus_down_rate;
	unsigned int new_freq = 0;
	unsigned int max_load = 0;
	unsigned long flags;
	unsigned long max_cpu;
	int i, fcpu;

	if (!down_read_trylock(&ppol->enable_sem))
		return;
	if (!ppol->governor_enabled)
		goto exit;

	fcpu = cpumask_first(ppol->policy->related_cpus);
	spin_lock_irqsave(&ppol->load_lock, flags);
	ppol->last_evaluated_jiffy = get_jiffies_64();

	if (is_display_on() &&
		tunables->timer_rate != tunables->timer_rate_prev)
		tunables->timer_rate = tunables->timer_rate_prev;
	else if (!is_display_on() &&
		tunables->timer_rate != DEFAULT_TIMER_RATE_SUSP) {
		tunables->timer_rate_prev = tunables->timer_rate;
		tunables->timer_rate
			= max(tunables->timer_rate,
				DEFAULT_TIMER_RATE_SUSP);
	}

	/* CPUs Online Scale Frequency*/
	target_cpu_load = (ppol->policy->cur * 100) / ppol->policy->max;
	if (ppol->policy->cur < freq_responsiveness) {
		pump_inc_step = tunables->pump_inc_step_at_min_freq;
		pump_dec_step = tunables->pump_dec_step_at_min_freq;
	} else if (ppol->policy->cur > freq_responsiveness_max) {
		cpus_up_rate = tunables->cpus_up_rate_at_max_freq;
		cpus_down_rate = tunables->cpus_down_rate_at_max_freq;
	}

	max_cpu = cpumask_first(ppol->policy->cpus);
	for_each_cpu(i, ppol->policy->cpus) {
		pcpu = &per_cpu(cpuinfo, i);
		if (update_load(i))
			continue;

		if (pcpu->load > max_load) {
			max_load = pcpu->load;
			max_cpu = i;
		}
	}
	spin_unlock_irqrestore(&ppol->load_lock, flags);


	/*
	 * mutex that serializes governor limit change with
	 * do_alucard_timer invocation. We do not want do_alucard_timer to run
	 * when user is changing the governor or limits.
	 */
	struct mutex timer_mutex;
};

static DEFINE_PER_CPU(struct cpufreq_alucard_cpuinfo, od_alucard_cpuinfo);

static unsigned int alucard_enable;	/* number of CPUs using this policy */
/*
 * alucard_mutex protects alucard_enable in governor start/stop.
 */
static DEFINE_MUTEX(alucard_mutex);

static struct workqueue_struct *alucard_wq;

/* alucard tuners */
static struct alucard_tuners {
	unsigned int sampling_rate;
	int inc_cpu_load_at_min_freq;
	int inc_cpu_load;
	int dec_cpu_load_at_min_freq;
	int dec_cpu_load;
	int freq_responsiveness;
	unsigned int cpus_up_rate;
	unsigned int cpus_down_rate;
} alucard_tuners_ins = {
	.sampling_rate = SAMPLING_RATE,
	.inc_cpu_load_at_min_freq = INC_CPU_LOAD_AT_MIN_FREQ,
	.inc_cpu_load = INC_CPU_LOAD,
	.dec_cpu_load_at_min_freq = DEC_CPU_LOAD_AT_MIN_FREQ,
	.dec_cpu_load = DEC_CPU_LOAD,
	.freq_responsiveness = FREQ_RESPONSIVENESS,
	.cpus_up_rate = CPUS_UP_RATE,
	.cpus_down_rate = CPUS_DOWN_RATE,
};

/************************** sysfs interface ************************/

/* cpufreq_alucard Governor Tunables */
#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct kobject *kobj, struct attribute *attr, char *buf)		\
{									\
	return sprintf(buf, "%d\n", alucard_tuners_ins.object);		\
}
show_one(sampling_rate, sampling_rate);
show_one(inc_cpu_load_at_min_freq, inc_cpu_load_at_min_freq);
show_one(inc_cpu_load, inc_cpu_load);
show_one(dec_cpu_load_at_min_freq, dec_cpu_load_at_min_freq);
show_one(dec_cpu_load, dec_cpu_load);
show_one(freq_responsiveness, freq_responsiveness);
show_one(cpus_up_rate, cpus_up_rate);
show_one(cpus_down_rate, cpus_down_rate);

#define show_pcpu_param(file_name, num_core)		\
static ssize_t show_##file_name##_##num_core		\
(struct kobject *kobj, struct attribute *attr, char *buf)		\
{									\
	struct cpufreq_alucard_cpuinfo *this_alucard_cpuinfo = &per_cpu(od_alucard_cpuinfo, num_core - 1); \
	return sprintf(buf, "%d\n", \
			this_alucard_cpuinfo->file_name);		\
}

show_pcpu_param(pump_inc_step_at_min_freq, 1);
show_pcpu_param(pump_inc_step_at_min_freq, 2);
show_pcpu_param(pump_inc_step_at_min_freq, 3);
show_pcpu_param(pump_inc_step_at_min_freq, 4);
show_pcpu_param(pump_inc_step, 1);
show_pcpu_param(pump_inc_step, 2);
show_pcpu_param(pump_inc_step, 3);
show_pcpu_param(pump_inc_step, 4);
show_pcpu_param(pump_dec_step_at_min_freq, 1);
show_pcpu_param(pump_dec_step_at_min_freq, 2);
show_pcpu_param(pump_dec_step_at_min_freq, 3);
show_pcpu_param(pump_dec_step_at_min_freq, 4);
show_pcpu_param(pump_dec_step, 1);
show_pcpu_param(pump_dec_step, 2);
show_pcpu_param(pump_dec_step, 3);
show_pcpu_param(pump_dec_step, 4);

#define store_pcpu_param(file_name, num_core)		\
static ssize_t store_##file_name##_##num_core		\
(struct kobject *kobj, struct attribute *attr,				\
	const char *buf, size_t count)					\
{									\
	int input;						\
	struct cpufreq_alucard_cpuinfo *this_alucard_cpuinfo; \
	int ret;							\
														\
	ret = sscanf(buf, "%d", &input);					\
	if (ret != 1)											\
		return -EINVAL;										\
														\
	this_alucard_cpuinfo = &per_cpu(od_alucard_cpuinfo, num_core - 1); \
														\
	if (input == this_alucard_cpuinfo->file_name) {		\
		return count;						\
	}								\
										\
	this_alucard_cpuinfo->file_name = input;			\
	return count;							\
}



static ssize_t store_timer_rate(struct cpufreq_alucard_tunables *tunables,
		const char *buf, size_t count)
{
	int ret;
	unsigned long val, val_round;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	val_round = jiffies_to_usecs(usecs_to_jiffies(val));
	if (val != val_round)
		pr_warn("timer_rate not aligned to jiffy. Rounded up to %lu\n",
			val_round);
	tunables->timer_rate = val_round;
	tunables->timer_rate_prev = val_round;

	return count;
}

static ssize_t show_timer_slack(struct cpufreq_alucard_tunables *tunables,
		char *buf)
{
	return sprintf(buf, "%d\n", tunables->timer_slack_val);
}

store_pcpu_pump_param(pump_inc_step_at_min_freq, 1);
store_pcpu_pump_param(pump_inc_step_at_min_freq, 2);
store_pcpu_pump_param(pump_inc_step_at_min_freq, 3);
store_pcpu_pump_param(pump_inc_step_at_min_freq, 4);
store_pcpu_pump_param(pump_inc_step, 1);
store_pcpu_pump_param(pump_inc_step, 2);
store_pcpu_pump_param(pump_inc_step, 3);
store_pcpu_pump_param(pump_inc_step, 4);
store_pcpu_pump_param(pump_dec_step_at_min_freq, 1);
store_pcpu_pump_param(pump_dec_step_at_min_freq, 2);
store_pcpu_pump_param(pump_dec_step_at_min_freq, 3);
store_pcpu_pump_param(pump_dec_step_at_min_freq, 4);
store_pcpu_pump_param(pump_dec_step, 1);
store_pcpu_pump_param(pump_dec_step, 2);
store_pcpu_pump_param(pump_dec_step, 3);
store_pcpu_pump_param(pump_dec_step, 4);

define_one_global_rw(pump_inc_step_at_min_freq_1);
define_one_global_rw(pump_inc_step_at_min_freq_2);
define_one_global_rw(pump_inc_step_at_min_freq_3);
define_one_global_rw(pump_inc_step_at_min_freq_4);
define_one_global_rw(pump_inc_step_1);
define_one_global_rw(pump_inc_step_2);
define_one_global_rw(pump_inc_step_3);
define_one_global_rw(pump_inc_step_4);
define_one_global_rw(pump_dec_step_at_min_freq_1);
define_one_global_rw(pump_dec_step_at_min_freq_2);
define_one_global_rw(pump_dec_step_at_min_freq_3);
define_one_global_rw(pump_dec_step_at_min_freq_4);
define_one_global_rw(pump_dec_step_1);
define_one_global_rw(pump_dec_step_2);
define_one_global_rw(pump_dec_step_3);
define_one_global_rw(pump_dec_step_4);

/* sampling_rate */
static ssize_t store_sampling_rate(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	int input;
	int ret = 0;
	int mpd = strcmp(current->comm, "mpdecision");

	if (mpd == 0)
		return ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	input = max(input, 10000);

	if (input == alucard_tuners_ins.sampling_rate)
		return count;

	alucard_tuners_ins.sampling_rate = input;

	return count;
}

/* inc_cpu_load_at_min_freq */
static ssize_t store_inc_cpu_load_at_min_freq(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1) {
		return -EINVAL;
	}

	input = min(input, alucard_tuners_ins.inc_cpu_load);

	if (input == alucard_tuners_ins.inc_cpu_load_at_min_freq)
		return count;

	alucard_tuners_ins.inc_cpu_load_at_min_freq = input;

	return count;
}

/* inc_cpu_load */
static ssize_t store_inc_cpu_load(struct kobject *a, struct attribute *b,
					const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	input = max(min(input, 100),0);

	if (input == alucard_tuners_ins.inc_cpu_load)
		return count;

	alucard_tuners_ins.inc_cpu_load = input;

	return count;
}

/* dec_cpu_load_at_min_freq */
static ssize_t store_dec_cpu_load_at_min_freq(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1) {
		return -EINVAL;
	}

	input = min(input, alucard_tuners_ins.dec_cpu_load);

	if (input == alucard_tuners_ins.dec_cpu_load_at_min_freq)
		return count;

	alucard_tuners_ins.dec_cpu_load_at_min_freq = input;

	return count;
}

/* dec_cpu_load */
static ssize_t store_dec_cpu_load(struct kobject *a, struct attribute *b,
					const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	input = max(min(input, 95),5);

	if (input == alucard_tuners_ins.dec_cpu_load)
		return count;

	alucard_tuners_ins.dec_cpu_load = input;

	return count;
}

/* freq_responsiveness */
static ssize_t store_freq_responsiveness(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	if (input == alucard_tuners_ins.freq_responsiveness)
		return count;

	alucard_tuners_ins.freq_responsiveness = input;

	return count;
}

/* cpus_up_rate */
static ssize_t store_cpus_up_rate(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input == alucard_tuners_ins.cpus_up_rate)
		return count;

	alucard_tuners_ins.cpus_up_rate = input;

	return count;
}

/* cpus_down_rate */
static ssize_t store_cpus_down_rate(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input == alucard_tuners_ins.cpus_down_rate)
		return count;

	alucard_tuners_ins.cpus_down_rate = input;

	return count;
}

define_one_global_rw(sampling_rate);
define_one_global_rw(inc_cpu_load_at_min_freq);
define_one_global_rw(inc_cpu_load);
define_one_global_rw(dec_cpu_load_at_min_freq);
define_one_global_rw(dec_cpu_load);
define_one_global_rw(freq_responsiveness);
define_one_global_rw(cpus_up_rate);
define_one_global_rw(cpus_down_rate);

static struct attribute *alucard_attributes[] = {
	&sampling_rate.attr,
	&inc_cpu_load_at_min_freq.attr,
	&inc_cpu_load.attr,
	&dec_cpu_load_at_min_freq.attr,
	&dec_cpu_load.attr,
	&freq_responsiveness.attr,
	&pump_inc_step_at_min_freq_1.attr,
	&pump_inc_step_at_min_freq_2.attr,
	&pump_inc_step_at_min_freq_3.attr,
	&pump_inc_step_at_min_freq_4.attr,
	&pump_inc_step_1.attr,
	&pump_inc_step_2.attr,
	&pump_inc_step_3.attr,
	&pump_inc_step_4.attr,
	&pump_dec_step_at_min_freq_1.attr,
	&pump_dec_step_at_min_freq_2.attr,
	&pump_dec_step_at_min_freq_3.attr,
	&pump_dec_step_at_min_freq_4.attr,
	&pump_dec_step_1.attr,
	&pump_dec_step_2.attr,
	&pump_dec_step_3.attr,
	&pump_dec_step_4.attr,
	&cpus_up_rate.attr,
	&cpus_down_rate.attr,
	NULL
};

static struct attribute_group alucard_attr_group = {
	.attrs = alucard_attributes,
	.name = "alucard",
};

/************************** sysfs end ************************/

static void cpufreq_frequency_table_policy_minmax_limits(struct cpufreq_policy *policy,
					struct cpufreq_alucard_cpuinfo *this_alucard_cpuinfo)
{
	struct cpufreq_frequency_table *table = this_alucard_cpuinfo->freq_table;
	unsigned int i = 0;

	for (i = 0; (table[i].frequency != CPUFREQ_TABLE_END); i++) {
		unsigned int freq = table[i].frequency;
		if (freq == CPUFREQ_ENTRY_INVALID) {
			continue;
		}
		if (freq == policy->min)
			this_alucard_cpuinfo->min_index = i;
		if (freq == policy->max)
			this_alucard_cpuinfo->max_index = i;

		if (freq >= policy->min &&
			freq >= policy->max)
			break;
	}
}

static void cpufreq_frequency_table_policy_cur_limit(struct cpufreq_policy *policy,
					struct cpufreq_frequency_table *table,
					unsigned int *index)
{
	unsigned int i = 0;

	for (i = 0; (table[i].frequency != CPUFREQ_TABLE_END); i++) {
		unsigned int freq = table[i].frequency;
		if (freq == CPUFREQ_ENTRY_INVALID) {
			continue;
		}
		if (freq == policy->cur) {
			*index = i;
			break;
		}
	}
}

static void alucard_check_cpu(struct cpufreq_alucard_cpuinfo *this_alucard_cpuinfo)
{
	struct cpufreq_policy *policy;
	unsigned int freq_responsiveness = alucard_tuners_ins.freq_responsiveness;
	int dec_cpu_load = alucard_tuners_ins.dec_cpu_load;
	int inc_cpu_load = alucard_tuners_ins.inc_cpu_load;
	int pump_inc_step = this_alucard_cpuinfo->pump_inc_step;
	int pump_dec_step = this_alucard_cpuinfo->pump_dec_step;
	unsigned int max_load = 0;
	unsigned int cpus_up_rate = alucard_tuners_ins.cpus_up_rate;
	unsigned int cpus_down_rate = alucard_tuners_ins.cpus_down_rate;
	unsigned int index = 0;
	unsigned int j;

	policy = this_alucard_cpuinfo->cur_policy;
	if (!policy)
		return;

	/* Get min, current, max indexes from current cpu policy */
	cpufreq_frequency_table_policy_cur_limit(policy,
				this_alucard_cpuinfo->freq_table,
				&index);

	for_each_cpu(j, policy->cpus) {
		struct cpufreq_alucard_cpuinfo *j_alucard_cpuinfo = &per_cpu(od_alucard_cpuinfo, j);
		u64 cur_wall_time, cur_idle_time;
		unsigned int idle_time, wall_time;
		unsigned int load;
		
		cur_idle_time = get_cpu_idle_time(j, &cur_wall_time, 0);

		wall_time = (unsigned int)
			(cur_wall_time - j_alucard_cpuinfo->prev_cpu_wall);
		j_alucard_cpuinfo->prev_cpu_wall = cur_wall_time;

		idle_time = (unsigned int)
			(cur_idle_time - j_alucard_cpuinfo->prev_cpu_idle);
		j_alucard_cpuinfo->prev_cpu_idle = cur_idle_time;

		if (unlikely(!wall_time || wall_time < idle_time))
			continue;

		load = 100 * (wall_time - idle_time) / wall_time;

		if (load > max_load)
			max_load = load;
	}

	/* CPUs Online Scale Frequency*/
	if (policy->cur < freq_responsiveness
		 && policy->cur > 0) {
		inc_cpu_load = alucard_tuners_ins.inc_cpu_load_at_min_freq;
		dec_cpu_load = alucard_tuners_ins.dec_cpu_load_at_min_freq;
		pump_inc_step = this_alucard_cpuinfo->pump_inc_step_at_min_freq;
		pump_dec_step = this_alucard_cpuinfo->pump_dec_step_at_min_freq;
	}
	/* Check for frequency increase or for frequency decrease */
	if (max_load >= inc_cpu_load && index < this_alucard_cpuinfo->max_index) {
		if (this_alucard_cpuinfo->up_rate % cpus_up_rate == 0) {
			if ((index + pump_inc_step) <= this_alucard_cpuinfo->max_index)
				index += pump_inc_step;
			else
				index = this_alucard_cpuinfo->max_index;

			this_alucard_cpuinfo->up_rate = 1;
			this_alucard_cpuinfo->down_rate = 1;

			if (this_alucard_cpuinfo->freq_table[index].frequency != CPUFREQ_ENTRY_INVALID)
				__cpufreq_driver_target(policy,
										this_alucard_cpuinfo->freq_table[index].frequency,
										CPUFREQ_RELATION_L);
		} else {
			if (this_alucard_cpuinfo->up_rate < cpus_up_rate)
				++this_alucard_cpuinfo->up_rate;
			else
				this_alucard_cpuinfo->up_rate = 1;
		}
	} else if (max_load < dec_cpu_load && index > this_alucard_cpuinfo->min_index) {
		if (this_alucard_cpuinfo->down_rate % cpus_down_rate == 0) {
			if ((index - this_alucard_cpuinfo->min_index) >= pump_dec_step)
				index -= pump_dec_step;
			else
				index = this_alucard_cpuinfo->min_index;

			this_alucard_cpuinfo->up_rate = 1;
			this_alucard_cpuinfo->down_rate = 1;

			if (this_alucard_cpuinfo->freq_table[index].frequency != CPUFREQ_ENTRY_INVALID)
				__cpufreq_driver_target(policy,
										this_alucard_cpuinfo->freq_table[index].frequency,
										CPUFREQ_RELATION_L);
		} else {
			if (this_alucard_cpuinfo->down_rate < cpus_down_rate)
				++this_alucard_cpuinfo->down_rate;
			else
				this_alucard_cpuinfo->down_rate = 1;
		}
	}
}

static void do_alucard_timer(struct work_struct *work)
{
	struct cpufreq_alucard_cpuinfo *this_alucard_cpuinfo = 
		container_of(work, struct cpufreq_alucard_cpuinfo, work.work);
	int delay;

	mutex_lock(&this_alucard_cpuinfo->timer_mutex);

	alucard_check_cpu(this_alucard_cpuinfo);

	tunables->timer_rate = DEFAULT_TIMER_RATE;
	tunables->timer_rate_prev = DEFAULT_TIMER_RATE;
	tunables->timer_slack_val = DEFAULT_TIMER_SLACK;
	tunables->freq_responsiveness = FREQ_RESPONSIVENESS;
	if (policy->cpu < 2)
		tunables->freq_responsiveness_max = FREQ_RESPONSIVENESS_MAX;
	else
		tunables->freq_responsiveness_max = FREQ_RESPONSIVENESS_MAX_BIGC;
	tunables->cpus_up_rate_at_max_freq = CPUS_UP_RATE;
	tunables->cpus_up_rate = CPUS_UP_RATE;
	tunables->cpus_down_rate_at_max_freq = CPUS_DOWN_RATE;
	tunables->cpus_down_rate = CPUS_DOWN_RATE;
	tunables->pump_inc_step_at_min_freq = PUMP_INC_STEP_AT_MIN_FREQ;
	tunables->pump_inc_step = PUMP_INC_STEP;
	tunables->pump_dec_step = PUMP_DEC_STEP;
	tunables->pump_dec_step_at_min_freq = PUMP_DEC_STEP_AT_MIN_FREQ;

	return tunables;
}

	/* We want all CPUs to do sampling nearly on same jiffy */
	if (num_online_cpus() > 1) {
		delay -= jiffies % delay;
	}

	queue_delayed_work_on(this_alucard_cpuinfo->cpu, alucard_wq,
			&this_alucard_cpuinfo->work, delay);
	mutex_unlock(&this_alucard_cpuinfo->timer_mutex);
}

static int cpufreq_governor_alucard(struct cpufreq_policy *policy,
				unsigned int event)
{
	struct cpufreq_alucard_cpuinfo *this_alucard_cpuinfo;
	unsigned int cpu = policy->cpu, j;
	int rc, delay;

	this_alucard_cpuinfo = &per_cpu(od_alucard_cpuinfo, cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		if (!policy)
			return -EINVAL;

		mutex_lock(&alucard_mutex);
		this_alucard_cpuinfo->freq_table = cpufreq_frequency_get_table(cpu);
		if (!this_alucard_cpuinfo->freq_table) {
			mutex_unlock(&alucard_mutex);
			return -EINVAL;
		}
		cpufreq_frequency_table_policy_minmax_limits(policy,
				this_alucard_cpuinfo);	

		for_each_cpu(j, policy->cpus) {
			struct cpufreq_alucard_cpuinfo *j_alucard_cpuinfo = &per_cpu(od_alucard_cpuinfo, j);

			j_alucard_cpuinfo->prev_cpu_idle = get_cpu_idle_time(j,
				&j_alucard_cpuinfo->prev_cpu_wall, 0);
		}

		alucard_enable++;
		/*
		 * Start the timerschedule work, when this governor
		 * is used for first time
		 */
		if (alucard_enable == 1) {
			rc = sysfs_create_group(cpufreq_global_kobject,
						&alucard_attr_group);
			if (rc) {
				alucard_enable--;
				mutex_unlock(&alucard_mutex);
				return rc;
			}
		}
		this_alucard_cpuinfo->cpu = cpu;
		this_alucard_cpuinfo->cur_policy = policy;
		this_alucard_cpuinfo->up_rate = 1;
		this_alucard_cpuinfo->down_rate = 1;
		this_alucard_cpuinfo->governor_enabled = true;
		mutex_unlock(&alucard_mutex);

		mutex_init(&this_alucard_cpuinfo->timer_mutex);

		delay = usecs_to_jiffies(alucard_tuners_ins.sampling_rate);
		/* We want all CPUs to do sampling nearly on same jiffy */
		if (num_online_cpus() > 1) {
			delay -= jiffies % delay;
		}

		INIT_DEFERRABLE_WORK(&this_alucard_cpuinfo->work, do_alucard_timer);
		queue_delayed_work_on(cpu,
			alucard_wq, &this_alucard_cpuinfo->work, delay);

		break;
	case CPUFREQ_GOV_STOP:
		cancel_delayed_work_sync(&this_alucard_cpuinfo->work);

		mutex_lock(&alucard_mutex);
		mutex_destroy(&this_alucard_cpuinfo->timer_mutex);

		this_alucard_cpuinfo->governor_enabled = false;

		this_alucard_cpuinfo->cur_policy = NULL;

		alucard_enable--;
		if (!alucard_enable) {
			sysfs_remove_group(cpufreq_global_kobject,
					   &alucard_attr_group);
		}

		mutex_unlock(&alucard_mutex);

		break;
	case CPUFREQ_GOV_LIMITS:
		if (!this_alucard_cpuinfo->cur_policy
			 || !policy) {
			pr_debug("Unable to limit cpu freq due to cur_policy == NULL\n");
			return -EPERM;
		}
		mutex_lock(&this_alucard_cpuinfo->timer_mutex);
		__cpufreq_driver_target(this_alucard_cpuinfo->cur_policy,
				policy->cur, CPUFREQ_RELATION_L);

		cpufreq_frequency_table_policy_minmax_limits(policy,
				this_alucard_cpuinfo);
		mutex_unlock(&this_alucard_cpuinfo->timer_mutex);

		break;
	}
	return 0;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_ALUCARD
static
#endif
struct cpufreq_governor cpufreq_gov_alucard = {
	.name                   = "alucard",
	.governor               = cpufreq_governor_alucard,
	.owner                  = THIS_MODULE,
};


static int __init cpufreq_gov_alucard_init(void)
{
	unsigned int cpu;

	for_each_possible_cpu(cpu) {
		struct cpufreq_alucard_cpuinfo *this_alucard_cpuinfo = &per_cpu(od_alucard_cpuinfo, cpu);

		this_alucard_cpuinfo->pump_inc_step_at_min_freq = PUMP_INC_STEP_AT_MIN_FREQ;
		this_alucard_cpuinfo->pump_inc_step = PUMP_INC_STEP;
		this_alucard_cpuinfo->pump_dec_step = PUMP_DEC_STEP;
		this_alucard_cpuinfo->pump_dec_step_at_min_freq = PUMP_DEC_STEP_AT_MIN_FREQ;
	}

	alucard_wq = alloc_workqueue("alucard_wq", WQ_HIGHPRI, 0);
	if (!alucard_wq) {
		printk(KERN_ERR "Failed to create alucard_wq workqueue\n");
		return -EFAULT;
	}

	return cpufreq_register_governor(&cpufreq_gov_alucard);
}

static void __exit cpufreq_gov_alucard_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_alucard);
}

MODULE_AUTHOR("Alucard24@XDA");
MODULE_DESCRIPTION("'cpufreq_alucard' - A dynamic cpufreq governor v3.0 (SnapDragon)");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_ALUCARD
fs_initcall(cpufreq_gov_alucard_init);
#else
module_init(cpufreq_gov_alucard_init);
#endif
module_exit(cpufreq_gov_alucard_exit);

