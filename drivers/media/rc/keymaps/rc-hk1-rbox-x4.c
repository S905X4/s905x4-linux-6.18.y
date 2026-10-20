// SPDX-License-Identifier: GPL-2.0+
/*
 * HK1 RBOX X4 NEC map_0 (sc2_ah212_4g, customcode 0xfb04).
 * Apply the active Vendor_0001_Product_0001.kl semantics to the vendor
 * keycodes; omit entries that this Android layout leaves unmapped.
 * NEC address 0x04 and inverse 0xfb become rc-core scancodes 0x04xx.
 */

#include <linux/module.h>
#include <media/rc-map.h>

static struct rc_map_table hk1_rbox_x4[] = {
	{ 0x0447, KEY_0 },
	{ 0x0413, KEY_1 },
	{ 0x0410, KEY_2 },
	{ 0x0411, KEY_3 },
	{ 0x040f, KEY_4 },
	{ 0x040c, KEY_5 },
	{ 0x040d, KEY_6 },
	{ 0x040b, KEY_7 },
	{ 0x0408, KEY_8 },
	{ 0x0409, KEY_9 },
	{ 0x0450, KEY_EPG },
	{ 0x044d, KEY_F6 },
	{ 0x0443, KEY_F7 },
	{ 0x0417, KEY_F8 },
	{ 0x0400, KEY_F9 },
	{ 0x0401, KEY_F10 },
	{ 0x0416, KEY_RED },
	{ 0x0449, KEY_BACKSPACE },
	{ 0x0406, KEY_ZOOMIN },
	{ 0x0414, KEY_ZOOMOUT },
	{ 0x0444, KEY_UP },
	{ 0x041d, KEY_DOWN },
	{ 0x041c, KEY_LEFT },
	{ 0x0448, KEY_RIGHT },
	{ 0x0453, KEY_MENU },
	{ 0x0445, KEY_VOLUMEUP },
	{ 0x0419, KEY_VOLUMEDOWN },
	{ 0x0452, KEY_PLAYPAUSE },
	{ 0x0405, KEY_PREVIOUSSONG },
	{ 0x0459, KEY_NEXTSONG },
	{ 0x041b, KEY_FASTFORWARD },
	{ 0x0404, KEY_REWIND },
	{ 0x041a, KEY_POWER },
	{ 0x040a, KEY_BACK },
	{ 0x040e, KEY_MUTE },
	{ 0x041f, KEY_HOMEPAGE },
	{ 0x0407, KEY_INFO },
	{ 0x0454, KEY_SUBTITLE },
	{ 0x0402, KEY_AUDIO },
	{ 0x0415, KEY_SEARCH },
	{ 0x0455, KEY_F10 },
};

static struct rc_map_list hk1_rbox_x4_map = {
	.map = {
		.scan = hk1_rbox_x4,
		.size = ARRAY_SIZE(hk1_rbox_x4),
		.rc_proto = RC_PROTO_NEC,
		.name = RC_MAP_HK1_RBOX_X4,
	}
};

static int __init init_rc_map_hk1_rbox_x4(void)
{
	return rc_map_register(&hk1_rbox_x4_map);
}

static void __exit exit_rc_map_hk1_rbox_x4(void)
{
	rc_map_unregister(&hk1_rbox_x4_map);
}

module_init(init_rc_map_hk1_rbox_x4)
module_exit(exit_rc_map_hk1_rbox_x4)

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("HK1 RBOX X4 stock remote controller keytable");
