// SPDX-License-Identifier: GPL-2.0+
/*
 * HK1 RBOX X4 stock NEC keytable (sc2_ah212_4g, customcode 0xfb04).
 * Vendor words store command in bits 23:16 and Linux keycode in bits 15:0.
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
	{ 0x045c, KEY_RIGHTCTRL },
	{ 0x0451, KEY_F3 },
	{ 0x0450, KEY_F4 },
	{ 0x0440, KEY_F5 },
	{ 0x044d, KEY_F6 },
	{ 0x0443, KEY_F7 },
	{ 0x0417, KEY_F8 },
	{ 0x0400, KEY_F9 },
	{ 0x0401, KEY_F10 },
	{ 0x0416, KEY_F11 },
	{ 0x0449, KEY_BACKSPACE },
	{ 0x0406, KEY_PROPS },
	{ 0x0414, KEY_UNDO },
	{ 0x0444, KEY_UP },
	{ 0x041d, KEY_DOWN },
	{ 0x041c, KEY_LEFT },
	{ 0x0448, KEY_RIGHT },
	{ 0x0453, KEY_LEFTMETA },
	{ 0x0445, KEY_PAGEUP },
	{ 0x0419, KEY_PAGEDOWN },
	{ 0x0452, KEY_PAUSE },
	{ 0x0405, KEY_HANGEUL },
	{ 0x0459, KEY_HANJA },
	{ 0x041b, KEY_SCALE },
	{ 0x0404, KEY_KPCOMMA },
	{ 0x041a, KEY_POWER },
	{ 0x040a, KEY_TAB },
	{ 0x040e, KEY_MUTE },
	{ 0x041f, KEY_HOME },
	{ 0x041e, KEY_FRONT },
	{ 0x0407, KEY_COPY },
	{ 0x0412, KEY_OPEN },
	{ 0x0454, KEY_PASTE },
	{ 0x0402, KEY_FIND },
	{ 0x044f, KEY_A },
	{ 0x0442, KEY_B },
	{ 0x045d, KEY_C },
	{ 0x044c, KEY_D },
	{ 0x0458, KEY_CUT },
	{ 0x0415, KEY_SEARCH },
	{ 0x0455, KEY_CALC },
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
