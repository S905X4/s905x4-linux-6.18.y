// SPDX-License-Identifier: GPL-2.0-only
/* Amlogic SC2 single-cluster CPU/DSU sequencing, using the generic OPP core. */
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/cpu.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#include "cpufreq-dt.h"

enum sc2_clock {
	SC2_CORE,
	SC2_LOW,
	SC2_HIGH,
	SC2_DSU,
	SC2_DSU_PRE,
	SC2_VDDCPU_PWM,
};

/* SC2 has one CPU cluster and one shared CPU supply. */
static struct clk_bulk_data sc2_clocks[] = {
	[SC2_CORE] = { .id = "core" },
	[SC2_LOW] = { .id = "low" },
	[SC2_HIGH] = { .id = "high" },
	[SC2_DSU] = { .id = "dsu" },
	[SC2_DSU_PRE] = { .id = "dsu-pre" },
	[SC2_VDDCPU_PWM] = { .id = "vddcpu-pwm" },
};

static struct platform_device *sc2_pdev;
static bool sc2_clock_failed;
static bool sc2_regulator_enabled;

struct sc2_cpufreq {
	struct platform_device *cpufreq_dt;
	int opp_token;
};

static int sc2_set_rate_checked(struct clk *clk, unsigned long rate)
{
	int ret;

	ret = clk_set_rate(clk, rate);
	if (ret)
		return ret;

	/* The secure firmware controls PLL output dividers as well as the DCO. */
	return clk_get_rate(clk) == rate ? 0 : -EIO;
}

static int sc2_cpufreq_config_clks(struct device *dev, struct opp_table *table,
				 struct dev_pm_opp *opp, void *data,
				 bool scaling_down)
{
	struct clk *core = sc2_clocks[SC2_CORE].clk;
	struct clk *low = sc2_clocks[SC2_LOW].clk;
	struct clk *high = sc2_clocks[SC2_HIGH].clk;
	struct clk *dsu = sc2_clocks[SC2_DSU].clk;
	struct clk *dsu_pre = sc2_clocks[SC2_DSU_PRE].clk;
	unsigned long rate, old_rate, park_rate;
	int ret;

	if (!opp)
		return -EINVAL;
	if (sc2_clock_failed)
		return -EIO;

	rate = data ? *(unsigned long *)data : dev_pm_opp_get_freq(opp);
	old_rate = clk_get_rate(core);
	park_rate = min3(old_rate, rate, 1000000000UL);

	/*
	 * OPP calls us after raising voltage, and before lowering it. The DSU
	 * follows the CPU up to 1.2 GHz, then uses its independent GP1 path.
	 * These thresholds and rates are the SC2 BSP's single-cluster policy.
	 */
	if (rate > 1200000000UL) {
		ret = sc2_set_rate_checked(dsu_pre, rate > 1800000000UL ?
					  1500000000UL : 1200000000UL);
		if (ret)
			goto failed;
		ret = clk_set_parent(dsu, dsu_pre);
		if (ret)
			goto failed;
	}

	/* Never reprogram the SYS PLL while the CPU is running from it. */
	ret = clk_set_rate(low, park_rate);
	if (ret)
		goto failed;
	ret = clk_set_parent(core, low);
	if (ret)
		goto failed;

	if (rate > 1000000000UL) {
		ret = sc2_set_rate_checked(high, rate);
		if (ret)
			goto failed;
		ret = clk_set_parent(core, high);
	} else {
		ret = clk_set_rate(low, rate);
	}
	if (ret)
		goto failed;

	/* The fixed 2 GHz / 3 path rounds the stock 667 MHz OPP. */
	if (abs((long)clk_get_rate(core) - (long)rate) > 10000000) {
		ret = -EIO;
		goto failed;
	}

	if (rate <= 1200000000UL) {
		ret = clk_set_parent(dsu, core);
		if (ret)
			goto failed;
	}

	return 0;

failed:
	/*
	 * Leave the raised/old voltage intact. A later request must not lower
	 * it while the actual CPU/DSU state after a firmware failure is unknown.
	 */
	sc2_clock_failed = true;
	dev_err(dev, "CPU/DSU transition failed (%d); retaining voltage\n", ret);
	return ret;
}

static const char * const sc2_opp_clk_names[] = { "core", NULL };

static int sc2_cpufreq_config_regulators(struct device *dev,
		struct dev_pm_opp *old_opp, struct dev_pm_opp *new_opp,
		struct regulator **regulators, unsigned int count)
{
	struct dev_pm_opp_supply supply;
	int ret;

	if (sc2_clock_failed)
		return -EIO;
	if (count != 1)
		return -EINVAL;

	ret = dev_pm_opp_get_supplies(new_opp, &supply);
	if (ret)
		return ret;
	ret = regulator_set_voltage_triplet(regulators[0], supply.u_volt_min,
					    supply.u_volt, supply.u_volt_max);
	if (ret)
		return ret;

	if (!sc2_regulator_enabled) {
		ret = regulator_enable(regulators[0]);
		if (ret)
			return ret;
		sc2_regulator_enabled = true;
	}
	return 0;
}

static int sc2_cpufreq_check_pwm_source(struct device *dev)
{
	struct clk *clk = sc2_clocks[SC2_VDDCPU_PWM].clk;
	struct clk_hw *hw;

	if (!clk_get_rate(clk))
		return dev_err_probe(dev, -EINVAL, "CPU voltage PWM has no clock\n");

	/* Never couple the CPU rail to a PLL changed by display mode setting. */
	for (hw = __clk_get_hw(clk); hw; hw = clk_hw_get_parent(hw)) {
		const char *name = clk_hw_get_name(hw);

		if (!strcmp(name, "vid_pll") || !strncmp(name, "hdmi_pll", 8))
			return dev_err_probe(dev, -EINVAL,
					     "CPU voltage PWM inherits mutable %s\n", name);
	}

	return 0;
}

static int sc2_cpufreq_probe(struct platform_device *pdev)
{
	struct dev_pm_opp_config config = {
		.clk_names = sc2_opp_clk_names,
		.config_clks = sc2_cpufreq_config_clks,
		.config_regulators = sc2_cpufreq_config_regulators,
	};
	struct device *cpu_dev = get_cpu_device(0);
	struct sc2_cpufreq *priv;
	int ret;

	if (!cpu_dev)
		return -EPROBE_DEFER;
	if (!of_property_present(cpu_dev->of_node, "cpu-supply"))
		return dev_err_probe(&pdev->dev, -EINVAL, "CPU supply is required\n");

	sc2_clock_failed = false;
	sc2_regulator_enabled = false;
	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ret = clk_bulk_get(cpu_dev, ARRAY_SIZE(sc2_clocks), sc2_clocks);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed to get CPU/DSU clocks\n");

	ret = sc2_cpufreq_check_pwm_source(&pdev->dev);
	if (ret)
		goto put_clocks;

	ret = clk_prepare_enable(sc2_clocks[SC2_CORE].clk);
	if (ret)
		goto put_clocks;
	ret = clk_prepare_enable(sc2_clocks[SC2_DSU_PRE].clk);
	if (ret)
		goto disable_core;

	priv->opp_token = dev_pm_opp_set_config(cpu_dev, &config);
	if (priv->opp_token < 0) {
		ret = priv->opp_token;
		goto disable_dsu;
	}

	priv->cpufreq_dt = cpufreq_dt_pdev_register(&pdev->dev);
	if (IS_ERR(priv->cpufreq_dt)) {
		ret = PTR_ERR(priv->cpufreq_dt);
		goto clear_config;
	}
	platform_set_drvdata(pdev, priv);
	return 0;

clear_config:
	dev_pm_opp_clear_config(priv->opp_token);
disable_dsu:
	clk_disable_unprepare(sc2_clocks[SC2_DSU_PRE].clk);
disable_core:
	clk_disable_unprepare(sc2_clocks[SC2_CORE].clk);
put_clocks:
	clk_bulk_put(ARRAY_SIZE(sc2_clocks), sc2_clocks);
	return ret;
}

static void sc2_cpufreq_remove(struct platform_device *pdev)
{
	struct sc2_cpufreq *priv = platform_get_drvdata(pdev);

	platform_device_unregister(priv->cpufreq_dt);
	dev_pm_opp_clear_config(priv->opp_token);
	clk_disable_unprepare(sc2_clocks[SC2_DSU_PRE].clk);
	clk_disable_unprepare(sc2_clocks[SC2_CORE].clk);
	clk_bulk_put(ARRAY_SIZE(sc2_clocks), sc2_clocks);
}

static struct platform_driver sc2_cpufreq_driver = {
	.probe = sc2_cpufreq_probe,
	.remove = sc2_cpufreq_remove,
	.driver.name = "sc2-cpufreq",
};

static int __init sc2_cpufreq_init(void)
{
	int ret;

	if (!of_machine_is_compatible("amlogic,sc2"))
		return -ENODEV;

	ret = platform_driver_register(&sc2_cpufreq_driver);
	if (ret)
		return ret;

	sc2_pdev = platform_device_register_simple("sc2-cpufreq", -1, NULL, 0);
	ret = PTR_ERR_OR_ZERO(sc2_pdev);
	if (ret)
		platform_driver_unregister(&sc2_cpufreq_driver);
	return ret;
}
module_init(sc2_cpufreq_init);

static void __exit sc2_cpufreq_exit(void)
{
	platform_device_unregister(sc2_pdev);
	platform_driver_unregister(&sc2_cpufreq_driver);
}
module_exit(sc2_cpufreq_exit);

MODULE_DESCRIPTION("Amlogic SC2 CPU/DSU frequency sequencing");
MODULE_LICENSE("GPL");
