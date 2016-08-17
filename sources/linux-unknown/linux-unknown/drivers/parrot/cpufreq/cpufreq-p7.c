#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>

#define NUM_CPUS 2

static struct clk* p7_cpu_clk;

static struct cpufreq_frequency_table freq_table[] = {
	{ 0, 780000 },
	{ 1, 390000 },
	{ 2, 195000 },
	{ 3, 97500 },
	{ 4, CPUFREQ_TABLE_END },
};

static int p7_verify_speed(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy, freq_table);
}

static unsigned int p7_getspeed(unsigned int cpu)
{
	unsigned long rate;

	if (cpu >= NR_CPUS)
		return 0;

	rate = clk_get_rate(p7_cpu_clk) / 1000;

	return rate;
}

static int p7_target(struct cpufreq_policy *policy,
		       unsigned int target_freq,
		       unsigned int relation)
{
	int ret;
	unsigned int i;
	unsigned int freq;
	struct cpufreq_freqs freqs;

	cpufreq_frequency_table_target(policy, freq_table, target_freq,
		relation, &i);

	freqs.new = freq_table[i].frequency;
	freqs.old = p7_getspeed(policy->cpu);
	freqs.cpu = policy->cpu;

	if (freqs.old == freqs.new && policy->cur == freqs.new)
		return 0;

	for_each_cpu(i, policy->cpus) {
		freqs.cpu = i;
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
	}

	freq = freqs.new * 1000;

	pr_debug("%s: %u MHz --> %u MHz\n", __func__,
		freqs.old / 1000, freqs.new / 1000);

	ret = clk_set_rate(p7_cpu_clk, freq);

	freqs.new = freq / 1000;

	for_each_cpu(i, policy->cpus) {
		freqs.cpu = i;
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	}

	return ret;
}

static int p7_cpu_init(struct cpufreq_policy *policy)
{
	int i;
	struct clk *p7_sys_clk;
	unsigned long rate;

	p7_sys_clk = clk_get_sys("sys_clk", NULL);
	if (IS_ERR(p7_sys_clk)) {
		pr_err("cpufreq-p7: failed to get sys_clk\n");
		return -EINVAL;
	}

	rate = clk_get_rate(p7_sys_clk);
	rate /= 1000;
	clk_put(p7_sys_clk);

	/*
	 * (From P7 UserManual)
	 * cpu_clk / sys_clk
	 * This clocks are generated from fast_clk, and can be dynamically
	 * divided. So the only thing needed to do is to configure the division
	 * ratio. For now, the recommended ratio are sys_clk=fast_clk/2 and
	 * cpu_clk=fast_clk or cpu_clk=fast_clk/2. Other ratio requires extra
	 * care. This is done in SYSTEM_CLKGEN_CPU_DIV and SYSTEM_CLKGEN_SYS_DIV
	 *
	 * Note : sys_clk frequency must not be greater than cpu_clk frequency.
	 */
	for (i = 0; freq_table[i].frequency != CPUFREQ_TABLE_END; i++)
	{
		if (freq_table[i].frequency < rate)
		{
			freq_table[i].frequency = CPUFREQ_TABLE_END;
			break;
		}
	}

	cpufreq_frequency_table_cpuinfo(policy, freq_table);
	cpufreq_frequency_table_get_attr(freq_table, policy->cpu);

	policy->min = policy->cpuinfo.min_freq;
	policy->max = policy->cpuinfo.max_freq;
	policy->cur = p7_getspeed(policy->cpu);

	/*
	 * Both processors share the voltage and clock. So both CPUs
	 * needs to be scaled together and hence needs software
	 * co-ordination. Use cpufreq affected_cpus interface to
	 * handle this scenario.
	 */
	policy->shared_type = CPUFREQ_SHARED_TYPE_ANY;
	cpumask_setall(policy->cpus);

	p7_cpu_clk = clk_get_sys("cpu_clk", NULL);
	if (IS_ERR(p7_cpu_clk)) {
		pr_err("cpufreq-p7: failed to get cpu_clk\n");
		return -EINVAL;
	}

	return 0;
}

static int p7_cpu_exit(struct cpufreq_policy *policy)
{
	clk_put(p7_cpu_clk);

	return 0;
}

static struct freq_attr *p7_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver p7_cpufreq_driver = {
	.flags		= CPUFREQ_STICKY,
	.verify		= p7_verify_speed,
	.target		= p7_target,
	.get		= p7_getspeed,
	.init		= p7_cpu_init,
	.exit		= p7_cpu_exit,
	.name		= "p7",
	.attr		= p7_cpufreq_attr,
};

static int __init p7_cpufreq_init(void)
{
	return cpufreq_register_driver(&p7_cpufreq_driver);
}

static void __exit p7_cpufreq_exit(void)
{
	cpufreq_unregister_driver(&p7_cpufreq_driver);
}

MODULE_AUTHOR("Aurelien Lefebvre <aurelien.lefebvre@parrot.com>");
MODULE_DESCRIPTION("cpufreq driver for p7");
MODULE_LICENSE("GPL");
module_init(p7_cpufreq_init);
module_exit(p7_cpufreq_exit);
