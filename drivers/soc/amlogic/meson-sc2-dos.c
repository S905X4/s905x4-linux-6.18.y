// SPDX-License-Identifier: GPL-2.0-only
/* SC2 engines share DOS configuration and secure-loader state. */
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/soc/amlogic/meson-sc2-dos.h>

static DEFINE_MUTEX(sc2_dos_lock);
static struct device *sc2_dos_owner;
static enum meson_sc2_dos_engine sc2_dos_engine;
static unsigned int sc2_dos_users;

int meson_sc2_dos_acquire(struct device *dev, enum meson_sc2_dos_engine engine)
{
	int ret = 0;

	if (!dev || engine < MESON_SC2_DOS_VDEC || engine > MESON_SC2_DOS_HEVC)
		return -EINVAL;
	mutex_lock(&sc2_dos_lock);
	if (sc2_dos_users && (sc2_dos_owner != dev || sc2_dos_engine != engine)) {
		ret = -EBUSY;
	} else {
		if (!sc2_dos_users)
			sc2_dos_owner = get_device(dev);
		sc2_dos_engine = engine;
		sc2_dos_users++;
	}
	mutex_unlock(&sc2_dos_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(meson_sc2_dos_acquire);

void meson_sc2_dos_release(struct device *dev, enum meson_sc2_dos_engine engine)
{
	mutex_lock(&sc2_dos_lock);
	if (WARN_ON(!sc2_dos_users || sc2_dos_owner != dev || sc2_dos_engine != engine))
		goto out;
	if (!--sc2_dos_users) {
		put_device(sc2_dos_owner);
		sc2_dos_owner = NULL;
	}
out:
	mutex_unlock(&sc2_dos_lock);
}
EXPORT_SYMBOL_GPL(meson_sc2_dos_release);

MODULE_DESCRIPTION("Amlogic SC2 shared DOS engine ownership");
MODULE_LICENSE("GPL");
