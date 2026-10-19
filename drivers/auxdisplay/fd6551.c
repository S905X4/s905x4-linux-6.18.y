// SPDX-License-Identifier: GPL-2.0-only
/*
 * FD6551 five-grid, seven-segment LED controller.
 *
 * Protocol: FD6551 device manual, revision A3, 2021-08-13, section 10.
 * The HK1 RBOX X4 wiring and digit/icon layout are corroborated by Arthur
 * Liberman's vfd-configurations/hk1-rbox-x4-vfd.conf; OpenVFD identifies
 * its controller number 6 as FD6551. No OpenVFD implementation is copied.
 */
#include <linux/bitops.h>
#include <linux/cleanup.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include "line-display.h"

#define FD6551_GRIDS		5
#define FD6551_DIGITS		4
#define FD6551_SEGMENTS		7
#define FD6551_CONTROL_ADDR	0x24
#define FD6551_GRID_ADDR		0x33
#define FD6551_MAX_BRIGHTNESS	8

struct fd6551 {
	struct i2c_client *client;
	struct i2c_client *grids[FD6551_GRIDS];
	struct linedisp linedisp;
	struct work_struct work;
	struct mutex lock;
	spinlock_t text_lock;
	struct led_classdev display;
	u32 digits[FD6551_DIGITS];
	u8 text[FD6551_DIGITS];
	u8 icons[FD6551_GRIDS];
};

struct fd6551_led {
	struct led_classdev cdev;
	struct fd6551 *priv;
	u8 grid;
	u8 mask;
};

/* Byte 0x48 selects display control; intensity zero is the brightest. */
static int fd6551_brightness_set(struct led_classdev *cdev,
			       enum led_brightness brightness)
{
	struct fd6551 *priv = container_of(cdev, struct fd6551, display);
	u8 control = brightness ? ((FD6551_MAX_BRIGHTNESS - brightness) << 1) | 1 : 0;
	int ret;

	mutex_lock(&priv->lock);
	ret = i2c_smbus_write_byte(priv->client, control);
	mutex_unlock(&priv->lock);
	return ret;
}

static int fd6551_icon_set(struct led_classdev *cdev,
			  enum led_brightness brightness)
{
	struct fd6551_led *led = container_of(cdev, struct fd6551_led, cdev);
	struct fd6551 *priv = led->priv;
	u8 value;
	int ret;

	mutex_lock(&priv->lock);
	value = priv->icons[led->grid];
	if (brightness)
		value |= led->mask;
	else
		value &= ~led->mask;
	ret = i2c_smbus_write_byte(priv->grids[led->grid], value);
	if (!ret)
		priv->icons[led->grid] = value;
	mutex_unlock(&priv->lock);
	return ret;
}

static void fd6551_text_work(struct work_struct *work)
{
	struct fd6551 *priv = container_of(work, struct fd6551, work);
	u8 text[FD6551_DIGITS];
	unsigned long flags;
	int i, ret = 0;

	spin_lock_irqsave(&priv->text_lock, flags);
	memcpy(text, priv->text, sizeof(text));
	spin_unlock_irqrestore(&priv->text_lock, flags);

	mutex_lock(&priv->lock);
	for (i = 0; i < FD6551_DIGITS; i++) {
		ret = i2c_smbus_write_byte(priv->grids[priv->digits[i]], text[i]);
		if (ret)
			break;
	}
	mutex_unlock(&priv->lock);
	if (ret)
		dev_err_ratelimited(&priv->client->dev, "display write failed: %d\n", ret);
}

static int fd6551_map_type(struct linedisp *linedisp)
{
	return LINEDISP_MAP_SEG7;
}

/* The line-display core can call this from its scrolling timer. */
static void fd6551_text_update(struct linedisp *linedisp)
{
	struct fd6551 *priv = container_of(linedisp, struct fd6551, linedisp);
	unsigned long flags;
	int i, value;

	spin_lock_irqsave(&priv->text_lock, flags);
	for (i = 0; i < FD6551_DIGITS; i++) {
		value = map_to_seg7(&linedisp->map->map.seg7, linedisp->buf[i]);
		priv->text[i] = value < 0 ? 0 : value & GENMASK(6, 0);
	}
	schedule_work(&priv->work);
	spin_unlock_irqrestore(&priv->text_lock, flags);
}

static const struct linedisp_ops fd6551_linedisp_ops = {
	.get_map_type = fd6551_map_type,
	.update = fd6551_text_update,
};

static void fd6551_unregister_display(void *data)
{
	struct fd6551 *priv = data;

	/* Stop the timer and sysfs producers before draining their work. */
	linedisp_unregister(&priv->linedisp);
	cancel_work_sync(&priv->work);
}

static void fd6551_power_off(void *data)
{
	struct fd6551 *priv = data;

	fd6551_brightness_set(&priv->display, LED_OFF);
}

static int fd6551_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct led_init_data init = { };
	struct fwnode_handle *display __free(fwnode_handle) = NULL;
	struct fd6551 *priv;
	u64 used = 0;
	u32 brightness;
	int i, ret;

	if (client->addr != FD6551_CONTROL_ADDR)
		return -EINVAL;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WRITE_BYTE))
		return -EOPNOTSUPP;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->client = client;
	mutex_init(&priv->lock);
	spin_lock_init(&priv->text_lock);
	INIT_WORK(&priv->work, fd6551_text_work);

	ret = device_property_read_u32_array(dev, "fdhisi,digits", priv->digits,
					     FD6551_DIGITS);
	if (ret)
		return ret;
	for (i = 0; i < FD6551_DIGITS; i++) {
		u32 grid = priv->digits[i];
		u64 mask;

		if (grid >= FD6551_GRIDS)
			return -EINVAL;
		mask = GENMASK_ULL(grid * FD6551_SEGMENTS + 6,
				   grid * FD6551_SEGMENTS);
		if (used & mask)
			return -EINVAL;
		used |= mask;
	}

	ret = device_property_read_u32(dev, "default-brightness-level", &brightness);
	if (ret)
		return ret;
	if (brightness > FD6551_MAX_BRIGHTNESS)
		return -EINVAL;

	for (i = 0; i < FD6551_GRIDS; i++) {
		priv->grids[i] = devm_i2c_new_dummy_device(dev, client->adapter,
							FD6551_GRID_ADDR + i);
		if (IS_ERR(priv->grids[i]))
			return PTR_ERR(priv->grids[i]);
	}

	/* RAM contents are undefined at power-on: blank before enabling. */
	ret = i2c_smbus_write_byte(client, 0);
	if (ret)
		return ret;
	for (i = 0; i < FD6551_GRIDS; i++) {
		ret = i2c_smbus_write_byte(priv->grids[i], 0);
		if (ret)
			return ret;
	}
	ret = devm_add_action_or_reset(dev, fd6551_power_off, priv);
	if (ret)
		return ret;

	display = device_get_named_child_node(dev, "display");
	if (!display)
		return -EINVAL;
	init.fwnode = display;
	priv->display.max_brightness = FD6551_MAX_BRIGHTNESS;
	priv->display.brightness = brightness;
	priv->display.brightness_set_blocking = fd6551_brightness_set;
	priv->display.flags = LED_CORE_SUSPENDRESUME;
	ret = fd6551_brightness_set(&priv->display, brightness);
	if (ret)
		return ret;
	ret = devm_led_classdev_register_ext(dev, &priv->display, &init);
	if (ret)
		return ret;

	device_for_each_child_node_scoped(dev, child) {
		struct fd6551_led *led;
		u32 index;

		if (!fwnode_property_present(child, "reg"))
			continue;
		ret = fwnode_property_read_u32(child, "reg", &index);
		if (ret)
			return ret;
		if (index >= FD6551_GRIDS * FD6551_SEGMENTS ||
		    (used & BIT_ULL(index)))
			return -EINVAL;
		used |= BIT_ULL(index);
		led = devm_kzalloc(dev, sizeof(*led), GFP_KERNEL);
		if (!led)
			return -ENOMEM;
		led->priv = priv;
		led->grid = index / FD6551_SEGMENTS;
		led->mask = BIT(index % FD6551_SEGMENTS);
		led->cdev.max_brightness = 1;
		led->cdev.brightness_set_blocking = fd6551_icon_set;
		init.fwnode = child;
		ret = devm_led_classdev_register_ext(dev, &led->cdev, &init);
		if (ret)
			return ret;
	}

	ret = linedisp_register(&priv->linedisp, dev, FD6551_DIGITS,
				&fd6551_linedisp_ops);
	if (ret) {
		cancel_work_sync(&priv->work);
		return ret;
	}
	return devm_add_action_or_reset(dev, fd6551_unregister_display, priv);
}

static const struct of_device_id fd6551_of_match[] = {
	{ .compatible = "fdhisi,fd6551" },
	{ }
};
MODULE_DEVICE_TABLE(of, fd6551_of_match);

static struct i2c_driver fd6551_driver = {
	.driver = {
		.name = "fd6551",
		.of_match_table = fd6551_of_match,
	},
	.probe = fd6551_probe,
};
module_i2c_driver(fd6551_driver);

MODULE_DESCRIPTION("FD6551 seven-segment LED display controller");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("LINEDISP");
