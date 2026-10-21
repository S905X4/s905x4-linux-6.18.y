// SPDX-License-Identifier: GPL-2.0-only
/*
 * Amlogic Secure Monitor driver
 *
 * Copyright (C) 2016 Endless Mobile, Inc.
 * Author: Carlo Caione <carlo@endlessm.com>
 */

#define pr_fmt(fmt) "meson-sm: " fmt

#include <linux/arm-smccc.h>
#include <linux/bug.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/property.h>
#include <linux/types.h>
#include <linux/sizes.h>
 #include <linux/slab.h>

#include <linux/firmware/meson/meson_sm.h>

struct meson_sm_cmd {
	unsigned int index;
	u32 smc_id;
};
#define CMD(d, s) { .index = (d), .smc_id = (s), }

struct meson_sm_chip {
	unsigned int shmem_size;
	u32 cmd_shmem_in_base;
	u32 cmd_shmem_out_base;
	struct meson_sm_cmd cmd[];
};

static const struct meson_sm_chip gxbb_chip = {
	.shmem_size		= SZ_4K,
	.cmd_shmem_in_base	= 0x82000020,
	.cmd_shmem_out_base	= 0x82000021,
	.cmd = {
		CMD(SM_EFUSE_READ,	0x82000030),
		CMD(SM_EFUSE_WRITE,	0x82000031),
		CMD(SM_EFUSE_USER_MAX,	0x82000033),
		CMD(SM_GET_CHIP_ID,	0x82000044),
		CMD(SM_A1_PWRC_SET,	0x82000093),
		CMD(SM_A1_PWRC_GET,	0x82000095),
		{ /* sentinel */ },
	},
};

/* Function 15 is the SC2 video-index ABI, not OP-TEE GET_THREAD_COUNT. */
static const struct meson_sm_chip sc2_chip = {
	.shmem_size		= SZ_4K,
	.cmd_shmem_in_base	= 0x82000020,
	.cmd_shmem_out_base	= 0x82000021,
	.cmd = {
		CMD(SM_EFUSE_READ,	0x82000030),
		CMD(SM_EFUSE_WRITE,	0x82000031),
		CMD(SM_EFUSE_USER_MAX,	0x82000033),
		CMD(SM_GET_CHIP_ID,	0x82000044),
		CMD(SM_A1_PWRC_SET,	0x82000093),
		CMD(SM_A1_PWRC_GET,	0x82000095),
		CMD(SM_VIDEO_FW_LOAD,	0xb200000f),
		{ /* sentinel */ },
	},
};

/* Global, not per-probe: reprobe must not reset a consumed notification. */
static DEFINE_MUTEX(sc2_boot_complete_lock);
static bool sc2_boot_complete_attempted;
static struct arm_smccc_res sc2_boot_complete_result;

struct meson_sm_firmware {
	const struct meson_sm_chip *chip;
	void __iomem *sm_shmem_in_base;
	void __iomem *sm_shmem_out_base;
};

static u32 meson_sm_get_cmd(const struct meson_sm_chip *chip,
			    unsigned int cmd_index)
{
	const struct meson_sm_cmd *cmd = chip->cmd;

	while (cmd->smc_id && cmd->index != cmd_index)
		cmd++;

	return cmd->smc_id;
}

static s32 __meson_sm_call(u32 cmd, u32 arg0, u32 arg1, u32 arg2,
			   u32 arg3, u32 arg4)
{
	struct arm_smccc_res res;

	arm_smccc_smc(cmd, arg0, arg1, arg2, arg3, arg4, 0, 0, &res);
	return res.a0;
}

static void __iomem *meson_sm_map_shmem(u32 cmd_shmem, unsigned int size)
{
	u32 sm_phy_base;

	sm_phy_base = __meson_sm_call(cmd_shmem, 0, 0, 0, 0, 0);
	if (!sm_phy_base)
		return NULL;

	return ioremap_cache(sm_phy_base, size);
}

/**
 * meson_sm_call - generic SMC32 call to the secure-monitor
 *
 * @fw:		Pointer to secure-monitor firmware
 * @cmd_index:	Index of the SMC32 function ID
 * @ret:	Returned value
 * @arg0:	SMC32 Argument 0
 * @arg1:	SMC32 Argument 1
 * @arg2:	SMC32 Argument 2
 * @arg3:	SMC32 Argument 3
 * @arg4:	SMC32 Argument 4
 *
 * Return:	0 on success, a negative value on error
 */
int meson_sm_call(struct meson_sm_firmware *fw, unsigned int cmd_index,
		  s32 *ret, u32 arg0, u32 arg1, u32 arg2, u32 arg3, u32 arg4)
{
	u32 cmd;
	s32 lret;

	if (!fw->chip)
		return -ENOENT;

	cmd = meson_sm_get_cmd(fw->chip, cmd_index);
	if (!cmd)
		return -EINVAL;

	lret = __meson_sm_call(cmd, arg0, arg1, arg2, arg3, arg4);

	if (ret)
		*ret = lret;

	return 0;
}
EXPORT_SYMBOL(meson_sm_call);

/**
 * meson_sm_call_read - retrieve data from secure-monitor
 *
 * @fw:		Pointer to secure-monitor firmware
 * @buffer:	Buffer to store the retrieved data
 * @bsize:	Size of the buffer
 * @cmd_index:	Index of the SMC32 function ID
 * @arg0:	SMC32 Argument 0
 * @arg1:	SMC32 Argument 1
 * @arg2:	SMC32 Argument 2
 * @arg3:	SMC32 Argument 3
 * @arg4:	SMC32 Argument 4
 *
 * Return:	size of read data on success, a negative value on error
 *		When 0 is returned there is no guarantee about the amount of
 *		data read and bsize bytes are copied in buffer.
 */
int meson_sm_call_read(struct meson_sm_firmware *fw, void *buffer,
		       unsigned int bsize, unsigned int cmd_index, u32 arg0,
		       u32 arg1, u32 arg2, u32 arg3, u32 arg4)
{
	s32 size;
	int ret;

	if (!fw->chip)
		return -ENOENT;

	if (!fw->chip->cmd_shmem_out_base)
		return -EINVAL;

	if (bsize > fw->chip->shmem_size)
		return -EINVAL;

	if (meson_sm_call(fw, cmd_index, &size, arg0, arg1, arg2, arg3, arg4) < 0)
		return -EINVAL;

	if (size < 0 || size > bsize)
		return -EINVAL;

	ret = size;

	/* In some cases (for example GET_CHIP_ID command),
	 * SMC doesn't return the number of bytes read, even
	 * though the bytes were actually read into sm_shmem_out.
	 * So this check is needed.
	 */
	if (!size)
		size = bsize;

	if (buffer)
		memcpy(buffer, fw->sm_shmem_out_base, size);

	return ret;
}
EXPORT_SYMBOL(meson_sm_call_read);

/**
 * meson_sm_call_write - send data to secure-monitor
 *
 * @fw:		Pointer to secure-monitor firmware
 * @buffer:	Buffer containing data to send
 * @size:	Size of the data to send
 * @cmd_index:	Index of the SMC32 function ID
 * @arg0:	SMC32 Argument 0
 * @arg1:	SMC32 Argument 1
 * @arg2:	SMC32 Argument 2
 * @arg3:	SMC32 Argument 3
 * @arg4:	SMC32 Argument 4
 *
 * Return:	size of sent data on success, a negative value on error
 */
int meson_sm_call_write(struct meson_sm_firmware *fw, void *buffer,
			unsigned int size, unsigned int cmd_index, u32 arg0,
			u32 arg1, u32 arg2, u32 arg3, u32 arg4)
{
	s32 written;

	if (!fw->chip)
		return -ENOENT;

	if (size > fw->chip->shmem_size)
		return -EINVAL;

	if (!fw->chip->cmd_shmem_in_base)
		return -EINVAL;

	memcpy(fw->sm_shmem_in_base, buffer, size);

	if (meson_sm_call(fw, cmd_index, &written, arg0, arg1, arg2, arg3, arg4) < 0)
		return -EINVAL;

	if (written <= 0 || written > size)
		return -EINVAL;

	return written;
}
EXPORT_SYMBOL(meson_sm_call_write);

/**
 * meson_sm_get - get pointer to meson_sm_firmware structure.
 *
 * @sm_node:		Pointer to the secure-monitor Device Tree node.
 *
 * Return:		NULL is the secure-monitor device is not ready.
 */
struct meson_sm_firmware *meson_sm_get(struct device_node *sm_node)
{
	struct platform_device *pdev = of_find_device_by_node(sm_node);
	struct meson_sm_firmware *fw;

	if (!pdev)
		return NULL;

	fw = platform_get_drvdata(pdev);

	put_device(&pdev->dev);

	return fw;
}
EXPORT_SYMBOL_GPL(meson_sm_get);

#define SM_CHIP_ID_LENGTH	119
#define SM_CHIP_ID_OFFSET	4
#define SM_CHIP_ID_SIZE		12

static ssize_t serial_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct meson_sm_firmware *fw;
	uint8_t *id_buf;
	int ret;

	fw = platform_get_drvdata(pdev);

	id_buf = kmalloc(SM_CHIP_ID_LENGTH, GFP_KERNEL);
	if (!id_buf)
		return -ENOMEM;

	ret = meson_sm_call_read(fw, id_buf, SM_CHIP_ID_LENGTH, SM_GET_CHIP_ID,
				 0, 0, 0, 0, 0);
	if (ret < 0) {
		kfree(id_buf);
		return ret;
	}

	ret = sprintf(buf, "%12phN\n", &id_buf[SM_CHIP_ID_OFFSET]);

	kfree(id_buf);

	return ret;
}

static DEVICE_ATTR_RO(serial);

static ssize_t boot_complete_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	const struct arm_smccc_res *res = &sc2_boot_complete_result;
	ssize_t ret;

	mutex_lock(&sc2_boot_complete_lock);
	if (!sc2_boot_complete_attempted)
		ret = sysfs_emit(buf, "not-attempted\n");
	else
		ret = sysfs_emit(buf, "attempted a0=%#lx a1=%#lx a2=%#lx a3=%#lx\n",
				 res->a0, res->a1, res->a2, res->a3);
	mutex_unlock(&sc2_boot_complete_lock);
	return ret;
}

static ssize_t boot_complete_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int ret;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;
	ret = mutex_lock_interruptible(&sc2_boot_complete_lock);
	if (ret)
		return ret;
	if (!sc2_boot_complete_attempted) {
		/* Pin the module so unload/reload cannot reset the per-boot guard.
		 * Consume the attempt before entering firmware: uncertain delivery
		 * and nonzero status must never cause an automatic second call.
		 */
		if (!try_module_get(THIS_MODULE)) {
			ret = -ENODEV;
			goto unlock;
		}
		sc2_boot_complete_attempted = true;
		arm_smccc_smc(0xb200e060, 0, 0, 0, 0, 0, 0, 0,
			      &sc2_boot_complete_result);
		dev_notice(dev, "boot-complete a0=%#lx a1=%#lx a2=%#lx a3=%#lx\n",
			   sc2_boot_complete_result.a0, sc2_boot_complete_result.a1,
			   sc2_boot_complete_result.a2, sc2_boot_complete_result.a3);
	}
	ret = sc2_boot_complete_result.a0 ? -EIO : 0;
unlock:
	mutex_unlock(&sc2_boot_complete_lock);
	return ret ? ret : count;
}
static DEVICE_ATTR(boot_complete, 0600, boot_complete_show, boot_complete_store);

static umode_t meson_sm_sysfs_is_visible(struct kobject *kobj,
				       struct attribute *attr, int index)
{
	struct meson_sm_firmware *fw = dev_get_drvdata(kobj_to_dev(kobj));

	if (attr == &dev_attr_boot_complete.attr && fw->chip != &sc2_chip)
		return 0;
	return attr->mode;
}

static struct attribute *meson_sm_sysfs_attrs[] = {
	&dev_attr_serial.attr,
	&dev_attr_boot_complete.attr,
	NULL,
};
static const struct attribute_group meson_sm_sysfs_group = {
	.attrs = meson_sm_sysfs_attrs,
	.is_visible = meson_sm_sysfs_is_visible,
};
__ATTRIBUTE_GROUPS(meson_sm_sysfs);

static const struct of_device_id meson_sm_ids[] = {
	{ .compatible = "amlogic,meson-sc2-sm", .data = &sc2_chip },
	{ .compatible = "amlogic,meson-gxbb-sm", .data = &gxbb_chip },
	{ /* sentinel */ },
};

static int __init meson_sm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct meson_sm_chip *chip;
	struct meson_sm_firmware *fw;

	fw = devm_kzalloc(dev, sizeof(*fw), GFP_KERNEL);
	if (!fw)
		return -ENOMEM;

	chip = device_get_match_data(dev);
	if (!chip)
		return -EINVAL;

	if (chip->cmd_shmem_in_base) {
		fw->sm_shmem_in_base = meson_sm_map_shmem(chip->cmd_shmem_in_base,
							  chip->shmem_size);
		if (WARN_ON(!fw->sm_shmem_in_base))
			goto out;
	}

	if (chip->cmd_shmem_out_base) {
		fw->sm_shmem_out_base = meson_sm_map_shmem(chip->cmd_shmem_out_base,
							   chip->shmem_size);
		if (WARN_ON(!fw->sm_shmem_out_base))
			goto unmap_in_base;
	}

	fw->chip = chip;

	platform_set_drvdata(pdev, fw);

	if (devm_of_platform_populate(dev))
		goto unmap_out_base;

	pr_info("secure-monitor enabled\n");

	return 0;

unmap_out_base:
	iounmap(fw->sm_shmem_out_base);
unmap_in_base:
	iounmap(fw->sm_shmem_in_base);
out:
	return -EINVAL;
}

static struct platform_driver meson_sm_driver = {
	.driver = {
		.name = "meson-sm",
		.of_match_table = of_match_ptr(meson_sm_ids),
		.dev_groups = meson_sm_sysfs_groups,
	},
};
module_platform_driver_probe(meson_sm_driver, meson_sm_probe);
MODULE_DESCRIPTION("Amlogic Secure Monitor driver");
MODULE_LICENSE("GPL v2");
