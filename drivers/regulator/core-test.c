// SPDX-License-Identifier: GPL-2.0-only
/* Consumer-visible voltage stepping and failure semantics. */
#include <kunit/device.h>
#include <kunit/test.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

struct voltage_step_test {
	struct regulator *consumer;
	unsigned int selector;
	unsigned int calls;
	unsigned int fail_at;
	int lower_bound;
	int upper_bound;
	bool invalid_step;
};

static int voltage_step_get_selector(struct regulator_dev *rdev)
{
	struct voltage_step_test *ctx = rdev_get_drvdata(rdev);

	return ctx->selector;
}

static int voltage_step_set_selector(struct regulator_dev *rdev,
				     unsigned int selector)
{
	struct voltage_step_test *ctx = rdev_get_drvdata(rdev);
	int old_uV = 689000 + ctx->selector * 10000;
	int new_uV = 689000 + selector * 10000;

	ctx->calls++;
	if (ctx->fail_at && ctx->calls >= ctx->fail_at)
		return -EIO;

	if (abs(new_uV - old_uV) > 30000 ||
	    new_uV < ctx->lower_bound || new_uV > ctx->upper_bound)
		ctx->invalid_step = true;
	ctx->selector = selector;
	return 0;
}

static int voltage_step_is_enabled(struct regulator_dev *rdev)
{
	return 1;
}

static const struct regulator_ops voltage_step_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.get_voltage_sel = voltage_step_get_selector,
	.set_voltage_sel = voltage_step_set_selector,
	.is_enabled = voltage_step_is_enabled,
};

static const struct regulator_desc voltage_step_desc = {
	.name = "voltage-step-test",
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.ops = &voltage_step_ops,
	.n_voltages = 37,
	.min_uV = 689000,
	.uV_step = 10000,
};

static int voltage_step_test_init(struct kunit *test)
{
	struct voltage_step_test *ctx;
	struct regulator_init_data init_data = {
		.constraints = {
			.min_uV = 689000,
			.max_uV = 1049000,
			.max_uV_step = 30000,
			.settling_time = 200,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.always_on = true,
		},
	};
	struct regulator_consumer_supply supply = { .supply = "cpu" };
	struct regulator_config config = { .init_data = &init_data };
	struct regulator_dev *rdev;
	struct device *dev;

	ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	test->priv = ctx;
	ctx->selector = 25; /* Inherited 939 mV, no consumer request yet. */
	ctx->lower_bound = 689000;
	ctx->upper_bound = 1049000;
	dev = kunit_device_register(test, "voltage-step-test");
	if (IS_ERR(dev))
		return PTR_ERR(dev);

	supply.dev_name = dev_name(dev);
	init_data.consumer_supplies = &supply;
	init_data.num_consumer_supplies = 1;
	config.dev = dev;
	config.driver_data = ctx;
	rdev = devm_regulator_register(dev, &voltage_step_desc, &config);
	if (IS_ERR(rdev))
		return PTR_ERR(rdev);

	ctx->consumer = devm_regulator_get(dev, "cpu");
	return PTR_ERR_OR_ZERO(ctx->consumer);
}

static void voltage_step_set_bounds(struct voltage_step_test *ctx, int target)
{
	int current_uV = regulator_get_voltage(ctx->consumer);

	ctx->lower_bound = min(current_uV, target);
	ctx->upper_bound = max(current_uV, target);
	ctx->calls = 0;
	ctx->invalid_step = false;
}

static void voltage_step_initial_down_test(struct kunit *test)
{
	struct voltage_step_test *ctx = test->priv;
	int ret;

	voltage_step_set_bounds(ctx, 779000);
	ret = regulator_set_voltage_triplet(ctx->consumer, 779000, 779000, 779000);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, regulator_get_voltage(ctx->consumer), 779000);
	KUNIT_EXPECT_FALSE(test, ctx->invalid_step);
}

static void voltage_step_short_final_step_test(struct kunit *test)
{
	struct voltage_step_test *ctx = test->priv;
	int ret;

	voltage_step_set_bounds(ctx, 779000);
	KUNIT_ASSERT_EQ(test, regulator_set_voltage(ctx->consumer, 779000, 779000), 0);
	voltage_step_set_bounds(ctx, 819000);
	ret = regulator_set_voltage(ctx->consumer, 819000, 819000);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, regulator_get_voltage(ctx->consumer), 819000);
	KUNIT_EXPECT_FALSE(test, ctx->invalid_step);
}

static void voltage_step_failure_retry_test(struct kunit *test)
{
	struct voltage_step_test *ctx = test->priv;
	unsigned int fail_at;
	int ret, stopped_uV;

	for (fail_at = 1; fail_at <= 6; fail_at++) {
		ctx->fail_at = 0;
		voltage_step_set_bounds(ctx, 939000);
		KUNIT_ASSERT_EQ(test, regulator_set_voltage(ctx->consumer, 939000, 939000), 0);
		voltage_step_set_bounds(ctx, 779000);
		ctx->fail_at = fail_at;
		ret = regulator_set_voltage(ctx->consumer, 779000, 779000);
		KUNIT_EXPECT_EQ(test, ret, -EIO);
		stopped_uV = regulator_get_voltage(ctx->consumer);
		KUNIT_EXPECT_EQ(test, stopped_uV, 939000 - (fail_at - 1) * 30000);
		KUNIT_EXPECT_FALSE(test, ctx->invalid_step);

		/* The helper's same-range retry must not report false success. */
		ret = regulator_set_voltage_triplet(ctx->consumer, 779000, 779000, 779000);
		KUNIT_EXPECT_EQ(test, ret, -EIO);
		KUNIT_EXPECT_EQ(test, regulator_get_voltage(ctx->consumer), stopped_uV);
	}
}

static void voltage_step_opp_pairs_test(struct kunit *test)
{
	static const int voltages[] = {
		779000, 799000, 809000, 839000, 849000,
		879000, 919000, 959000, 989000, 1019000,
	};
	struct voltage_step_test *ctx = test->priv;
	unsigned int i, j;
	int ret;

	for (i = 0; i < ARRAY_SIZE(voltages); i++) {
		for (j = 0; j < ARRAY_SIZE(voltages); j++) {
			voltage_step_set_bounds(ctx, voltages[i]);
			ret = regulator_set_voltage(ctx->consumer, voltages[i], voltages[i]);
			KUNIT_ASSERT_EQ(test, ret, 0);
			voltage_step_set_bounds(ctx, voltages[j]);
			ret = regulator_set_voltage(ctx->consumer, voltages[j], voltages[j]);
			KUNIT_ASSERT_EQ(test, ret, 0);
			KUNIT_EXPECT_EQ(test, regulator_get_voltage(ctx->consumer), voltages[j]);
			KUNIT_EXPECT_FALSE(test, ctx->invalid_step);
		}
	}
}

static struct kunit_case voltage_step_test_cases[] = {
	KUNIT_CASE(voltage_step_initial_down_test),
	KUNIT_CASE(voltage_step_short_final_step_test),
	KUNIT_CASE(voltage_step_failure_retry_test),
	KUNIT_CASE(voltage_step_opp_pairs_test),
	{}
};

static struct kunit_suite voltage_step_test_suite = {
	.name = "regulator-voltage-step",
	.init = voltage_step_test_init,
	.test_cases = voltage_step_test_cases,
};

kunit_test_suite(voltage_step_test_suite);

MODULE_DESCRIPTION("Regulator voltage step consumer tests");
MODULE_LICENSE("GPL");
