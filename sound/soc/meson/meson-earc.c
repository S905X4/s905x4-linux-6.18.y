// SPDX-License-Identifier: (GPL-2.0 OR MIT)
/*
 * Amlogic Meson eARC (Enhanced Audio Return Channel) Receiver Driver
 *
 * Copyright (c) 2019 Amlogic, Inc. All rights reserved.
 * Copyright (c) 2026 CoreELEC & LibreELEC developers
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>

#include "meson-earc.h"

#define DRV_NAME "meson-earc"

struct meson_earc_match_data {
	bool rx_dmac_sync_int;
	bool rterm_on;
	bool unstable_tick_sel;
	bool chnum_mult_mode;
};

struct meson_earc {
	struct device *dev;
	const struct meson_earc_match_data *data;

	struct regmap *cmdc_map;
	struct regmap *dmac_map;
	struct regmap *top_map;

	struct clk *clk_rx_cmdc;
	struct clk *clk_rx_dmac;
	struct clk *clk_rx_cmdc_srcpll;
	struct clk *clk_rx_dmac_srcpll;

	int irq;

	spinlock_t lock;
	bool dmac_clk_on;
	bool arc_connected;
	u8 latency;
	u8 cds[EARC_CDS_MAX_BYTES];
	u32 rx_status0;
	u32 rx_status1;
	int rx_cs_mute;

	enum meson_earc_cmdc_state state;
	enum meson_earc_attend_type attend_type;
};

static const struct regmap_config meson_earc_cmdc_regmap_cfg = {
	.name = "earc-cmdc",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x3fc,
};

static const struct regmap_config meson_earc_dmac_regmap_cfg = {
	.name = "earc-dmac",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x1fc,
};

static const struct regmap_config meson_earc_top_regmap_cfg = {
	.name = "earc-top",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x1fc,
};

/*
 * CMDC APB device register access
 */
static int meson_earc_cmdc_read_reg(struct meson_earc *priv, u8 dev_id, u8 offset,
				    u8 *buf, size_t len)
{
	unsigned int val;
	size_t i;

	regmap_update_bits(priv->cmdc_map, EARC_RX_CMDC_DEVICE_ID_CTRL,
			   EARC_RX_CMDC_DEV_ID_APB_WRITE |
			   EARC_RX_CMDC_DEV_ID_APB_READ |
			   EARC_RX_CMDC_DEV_ID_APB_DONE |
			   EARC_RX_CMDC_DEV_ID_MASK |
			   EARC_RX_CMDC_DEV_ID_ADDR_MASK,
			   EARC_RX_CMDC_DEV_ID_APB_READ |
			   FIELD_PREP(EARC_RX_CMDC_DEV_ID_MASK, dev_id) |
			   FIELD_PREP(EARC_RX_CMDC_DEV_ID_ADDR_MASK, offset));

	for (i = 0; i < len; i++) {
		regmap_read(priv->cmdc_map, EARC_RX_CMDC_DEVICE_RDATA, &val);
		buf[i] = val & 0xff;
	}

	regmap_update_bits(priv->cmdc_map, EARC_RX_CMDC_DEVICE_ID_CTRL,
			   EARC_RX_CMDC_DEV_ID_APB_DONE,
			   EARC_RX_CMDC_DEV_ID_APB_DONE);
	regmap_update_bits(priv->cmdc_map, EARC_RX_CMDC_DEVICE_ID_CTRL,
			   EARC_RX_CMDC_DEV_ID_APB_DONE, 0);

	return 0;
}

static int meson_earc_cmdc_write_reg(struct meson_earc *priv, u8 dev_id, u8 offset,
				     const u8 *buf, size_t len)
{
	size_t i;

	regmap_update_bits(priv->cmdc_map, EARC_RX_CMDC_DEVICE_ID_CTRL,
			   EARC_RX_CMDC_DEV_ID_APB_WRITE |
			   EARC_RX_CMDC_DEV_ID_APB_READ |
			   EARC_RX_CMDC_DEV_ID_APB_DONE |
			   EARC_RX_CMDC_DEV_ID_MASK |
			   EARC_RX_CMDC_DEV_ID_ADDR_MASK,
			   EARC_RX_CMDC_DEV_ID_APB_WRITE |
			   FIELD_PREP(EARC_RX_CMDC_DEV_ID_MASK, dev_id) |
			   FIELD_PREP(EARC_RX_CMDC_DEV_ID_ADDR_MASK, offset));

	for (i = 0; i < len; i++)
		regmap_write(priv->cmdc_map, EARC_RX_CMDC_DEVICE_WDATA, buf[i]);

	regmap_update_bits(priv->cmdc_map, EARC_RX_CMDC_DEVICE_ID_CTRL,
			   EARC_RX_CMDC_DEV_ID_APB_DONE,
			   EARC_RX_CMDC_DEV_ID_APB_DONE);
	regmap_update_bits(priv->cmdc_map, EARC_RX_CMDC_DEVICE_ID_CTRL,
			   EARC_RX_CMDC_DEV_ID_APB_DONE, 0);

	return 0;
}

static u8 meson_earc_get_rx_stat(struct meson_earc *priv)
{
	u8 stat = 0;

	meson_earc_cmdc_read_reg(priv, EARC_CMDC_STAT_CTRL_DEV_ID,
				 EARC_RX_STAT_REG, &stat, 1);
	return stat;
}

static void meson_earc_set_latency(struct meson_earc *priv, u8 latency)
{
	u8 stat = EARC_STAT_CHNG | meson_earc_get_rx_stat(priv);

	priv->latency = latency;
	meson_earc_cmdc_write_reg(priv, EARC_CMDC_STAT_CTRL_DEV_ID,
				  EARC_RX_LATENCY_REG, &latency, 1);
	meson_earc_cmdc_write_reg(priv, EARC_CMDC_STAT_CTRL_DEV_ID,
				  EARC_RX_STAT_REG, &stat, 1);
}

static void meson_earc_set_cds(struct meson_earc *priv, const u8 *cds)
{
	u8 stat = EARC_CAP_CHNG | EARC_HPD | meson_earc_get_rx_stat(priv);

	meson_earc_cmdc_write_reg(priv, EARC_CMDC_CAP_DEV_ID, 0,
				  cds, EARC_CDS_MAX_BYTES);
	meson_earc_cmdc_write_reg(priv, EARC_CMDC_STAT_CTRL_DEV_ID,
				  EARC_RX_STAT_REG, &stat, 1);
}

static enum meson_earc_cmdc_state meson_earc_get_state(struct meson_earc *priv)
{
	unsigned int val;

	regmap_read(priv->cmdc_map, EARC_RX_CMDC_STATUS0, &val);
	return (enum meson_earc_cmdc_state)FIELD_GET(EARC_RX_CMDC_STATUS0_STATE, val);
}

static enum meson_earc_attend_type meson_earc_get_attended_type(struct meson_earc *priv)
{
	unsigned int val;
	enum meson_earc_cmdc_state state;

	regmap_read(priv->cmdc_map, EARC_RX_CMDC_STATUS0, &val);
	state = (enum meson_earc_cmdc_state)FIELD_GET(EARC_RX_CMDC_STATUS0_STATE, val);

	if ((val & EARC_RX_CMDC_STATUS0_ARC_ATTENDED) && state == MESON_EARC_CMDC_ST_ARC)
		return MESON_EARC_ATTEND_ARC;
	if ((val & EARC_RX_CMDC_STATUS0_EARC_ATTENDED) && state == MESON_EARC_CMDC_ST_EARC)
		return MESON_EARC_ATTEND_EARC;

	return MESON_EARC_ATTEND_DISCONNECT;
}

static void meson_earc_pll_refresh(struct meson_earc *priv)
{
	regmap_update_bits(priv->top_map, EARCRX_PLL_CTRL3,
			   EARCRX_PLL_CTRL3_TDC_MODE,
			   EARCRX_PLL_CTRL3_TDC_MODE);
	regmap_update_bits(priv->top_map, EARCRX_PLL_CTRL0,
			   EARCRX_PLL_CTRL0_SELF_RST,
			   EARCRX_PLL_CTRL0_SELF_RST);
	regmap_update_bits(priv->top_map, EARCRX_PLL_CTRL0,
			   EARCRX_PLL_CTRL0_SELF_RST, 0);
	regmap_update_bits(priv->top_map, EARCRX_PLL_CTRL3,
			   EARCRX_PLL_CTRL3_TDC_MODE, 0);
}

static void meson_earc_cmdc_int_mask(struct meson_earc *priv)
{
	regmap_write(priv->top_map, EARCRX_CMDC_INT_MASK,
		     INT_EARCRX_CMDC_IDLE2 |
		     INT_EARCRX_CMDC_IDLE1 |
		     INT_EARCRX_CMDC_DISC2 |
		     INT_EARCRX_CMDC_DISC1 |
		     INT_EARCRX_CMDC_EARC |
		     INT_EARCRX_CMDC_HB_STATUS |
		     INT_EARCRX_CMDC_LOSTHB |
		     INT_EARCRX_CMDC_TIMEOUT |
		     INT_EARCRX_CMDC_STATUS_CH);
}

static void meson_earc_cmdc_arc_connect(struct meson_earc *priv, bool init)
{
	if (init) {
		regmap_update_bits(priv->cmdc_map, EARC_RX_CMDC_VSM_CTRL0,
				   0x7 << 25,
				   EARC_RX_CMDC_VSM_ARC_INITIATED |
				   EARC_RX_CMDC_VSM_ARC_ENABLE);
	} else {
		regmap_update_bits(priv->cmdc_map, EARC_RX_CMDC_VSM_CTRL0,
				   0x7 << 25,
				   EARC_RX_CMDC_VSM_ARC_TERMINATED);
	}
}

static void meson_earc_cmdc_hpd_detect(struct meson_earc *priv, bool st)
{
	if (st) {
		regmap_update_bits(priv->cmdc_map, EARC_RX_CMDC_VSM_CTRL0,
				   EARC_RX_CMDC_VSM_COMMA_CNT_RST,
				   EARC_RX_CMDC_VSM_COMMA_CNT_RST);
		regmap_update_bits(priv->cmdc_map, EARC_RX_CMDC_VSM_CTRL0,
				   EARC_RX_CMDC_VSM_COMMA_CNT_RST | 0xff,
				   0xa);
	} else {
		regmap_update_bits(priv->cmdc_map, EARC_RX_CMDC_TOP_CTRL1,
				   0xf << 1, 0xf << 1);
		regmap_update_bits(priv->cmdc_map, EARC_RX_CMDC_TOP_CTRL1,
				   0xf << 1, 0);
	}
}

static void meson_earc_cmdc_init(struct meson_earc *priv)
{
	if (priv->data->rterm_on) {
		regmap_write(priv->top_map, EARCRX_ANA_CTRL0,
			     (1U << 31) | (0x10 << 25) | (0x10 << 20) | (0x10 << 15) |
			     (0x4 << 12) | (0x10 << 7) | (0x1 << 4) | (0x4 << 0));
		regmap_write(priv->top_map, EARCRX_ANA_CTRL1,
			     (1 << 11) | (1 << 10) | (0x8 << 4) | (0x8 << 0));
	} else {
		regmap_write(priv->top_map, EARCRX_ANA_CTRL0,
			     (1U << 31) | (0x10 << 24) | (0x8 << 20) | (0x10 << 15) |
			     (0x4 << 12) | (0x10 << 7) | (0x1 << 4) | (0x4 << 0));
	}

	regmap_write(priv->top_map, EARCRX_PLL_CTRL3,
		     (0x2 << 20) | (0x4 << 16) | (1 << 13));
	regmap_write(priv->top_map, EARCRX_PLL_CTRL0,
		     (1U << 28) | (1 << 23) | (1 << 10));
	regmap_write(priv->top_map, EARCRX_PLL_CTRL1, 0x1410a8);
	regmap_update_bits(priv->top_map, EARCRX_PLL_CTRL2, 0x3, 0x2);
	regmap_write(priv->top_map, EARCRX_PLL_CTRL3, 0x20046000);
}

static void meson_earc_dmac_init(struct meson_earc *priv)
{
	regmap_update_bits(priv->top_map, EARCRX_DMAC_INT_MASK, 0x3ffff,
			   INT_EARCRX_ANA_RST_C_NEW_FORMAT_SET |
			   INT_EARCRX_ANA_RST_C_EARCRX_DIV2_HOLD |
			   INT_EARCRX_ERR_CORRECT_C_BCHERR_INT |
			   INT_ARCRX_BIPHASE_DECODE_C_FIND_PAPB |
			   INT_ARCRX_BIPHASE_DECODE_C_VALID_CHANGE |
			   INT_ARCRX_BIPHASE_DECODE_C_PCPD_CHANGE |
			   INT_ARCRX_BIPHASE_DECODE_C_CHST_CHANGE |
			   INT_ARCRX_BIPHASE_DECODE_I_MODE_CHANGE |
			   INT_ARCRX_BIPHASE_DECODE_R_PARITY_ERR);

	regmap_write(priv->dmac_map, EARCRX_DMAC_SYNC_CTRL0,
		     (1 << 16) | (3 << 12) | (7 << 8) | (7 << 4));
	regmap_write(priv->dmac_map, EARCRX_DMAC_UBIT_CTRL0,
		     (47 << 16) | (1 << 12) | 29);
	regmap_write(priv->dmac_map, EARCRX_ANA_RST_CTRL0,
		     (1U << 31) | 2000);

	if (priv->data->rx_dmac_sync_int)
		regmap_update_bits(priv->top_map, EARCRX_DMAC_INT_MASK,
				   INT_EARCRX_DMAC_VALID_AUTO_NEG_INT_SET,
				   INT_EARCRX_DMAC_VALID_AUTO_NEG_INT_SET);
}

static void meson_earc_arc_init(struct meson_earc *priv)
{
	unsigned int spdifin_clk = 500000000;
	unsigned int counter_32k  = spdifin_clk / (32000  * 64);
	unsigned int counter_44k  = spdifin_clk / (44100  * 64);
	unsigned int counter_48k  = spdifin_clk / (48000  * 64);
	unsigned int counter_88k  = spdifin_clk / (88200  * 64);
	unsigned int counter_96k  = spdifin_clk / (96000  * 64);
	unsigned int counter_176k = spdifin_clk / (176400 * 64);
	unsigned int counter_192k = spdifin_clk / (192000 * 64);

	unsigned int mode0_th = (3 * (counter_32k + counter_44k)) >> 1;
	unsigned int mode1_th = (3 * (counter_44k + counter_48k)) >> 1;
	unsigned int mode2_th = (3 * (counter_48k + counter_88k)) >> 1;
	unsigned int mode3_th = (3 * (counter_88k + counter_96k)) >> 1;
	unsigned int mode4_th = (3 * (counter_96k + counter_176k)) >> 1;
	unsigned int mode5_th = (3 * (counter_176k + counter_192k)) >> 1;

	unsigned int mode0_timer = counter_32k >> 1;
	unsigned int mode1_timer = counter_44k >> 1;
	unsigned int mode2_timer = counter_48k >> 1;
	unsigned int mode3_timer = counter_88k >> 1;
	unsigned int mode4_timer = counter_96k >> 1;
	unsigned int mode5_timer = counter_176k >> 1;
	unsigned int mode6_timer = counter_192k >> 1;

	regmap_write(priv->dmac_map, EARCRX_SPDIFIN_SAMPLE_CTRL0,
		     spdifin_clk / 10000);
	regmap_write(priv->dmac_map, EARCRX_SPDIFIN_SAMPLE_CTRL1,
		     (mode0_th << 20) | (mode1_th << 10) | mode2_th);
	regmap_write(priv->dmac_map, EARCRX_SPDIFIN_SAMPLE_CTRL2,
		     (mode3_th << 20) | (mode4_th << 10) | mode5_th);
	regmap_write(priv->dmac_map, EARCRX_SPDIFIN_SAMPLE_CTRL3,
		     (mode0_timer << 24) | (mode1_timer << 16) |
		     (mode2_timer << 8) | mode3_timer);
	regmap_write(priv->dmac_map, EARCRX_SPDIFIN_SAMPLE_CTRL4,
		     (mode4_timer << 24) | (mode5_timer << 16) |
		     (mode6_timer << 8));
	regmap_write(priv->dmac_map, EARCRX_SPDIFIN_CTRL0,
		     EARCRX_SPDIFIN_CTRL0_WORK_EN |
		     (1 << 25) | (1 << 24) | (0xfff << 12) | (1 << 2));
	regmap_write(priv->dmac_map, EARCRX_SPDIFIN_CTRL2,
		     (1 << 16) | (1 << 15) | (1 << 14) | (1 << 13) |
		     (1 << 12) | (19 << 4));
	regmap_write(priv->dmac_map, EARCRX_SPDIFIN_CTRL3,
		     (0xec37 << 16) | 0x5a5a);
}

static void meson_earc_enable(struct meson_earc *priv, bool enable)
{
	enum meson_earc_attend_type type = meson_earc_get_attended_type(priv);

	if (enable) {
		regmap_update_bits(priv->dmac_map, EARCRX_DMAC_SYNC_CTRL0,
				   EARCRX_DMAC_SYNC_CTRL0_RST_AFIFO_OUT_N |
				   EARCRX_DMAC_SYNC_CTRL0_RST_AFIFO_IN_N,
				   EARCRX_DMAC_SYNC_CTRL0_RST_AFIFO_OUT_N |
				   EARCRX_DMAC_SYNC_CTRL0_RST_AFIFO_IN_N);
		regmap_update_bits(priv->dmac_map, EARCRX_ERR_CORRECT_CTRL0,
				   EARCRX_ERR_CORRECT_CTRL0_RST_OUT_N |
				   EARCRX_ERR_CORRECT_CTRL0_RST_IN_N,
				   EARCRX_ERR_CORRECT_CTRL0_RST_OUT_N |
				   EARCRX_ERR_CORRECT_CTRL0_RST_IN_N);
	} else {
		regmap_update_bits(priv->dmac_map, EARCRX_DMAC_SYNC_CTRL0,
				   EARCRX_DMAC_SYNC_CTRL0_RST_AFIFO_OUT_N |
				   EARCRX_DMAC_SYNC_CTRL0_RST_AFIFO_IN_N,
				   0);
		regmap_update_bits(priv->dmac_map, EARCRX_ERR_CORRECT_CTRL0,
				   EARCRX_ERR_CORRECT_CTRL0_RST_OUT_N |
				   EARCRX_ERR_CORRECT_CTRL0_RST_IN_N,
				   0);
	}

	if (type == MESON_EARC_ATTEND_EARC) {
		regmap_update_bits(priv->dmac_map, EARCRX_SPDIFIN_CTRL1,
				   0x7fff << 9, (0x500 << 12) | (0x4 << 9));
		regmap_update_bits(priv->dmac_map, EARCRX_SPDIFIN_CTRL2,
				   0x7fff << 17,
				   IEC_CS_MUTE_OFFSET | (0x2 << 19) | (enable ? (1 << 17) : 0));
		regmap_update_bits(priv->dmac_map, EARCRX_DMAC_SYNC_CTRL0,
				   EARCRX_DMAC_SYNC_CTRL0_WORK_EN,
				   enable ? EARCRX_DMAC_SYNC_CTRL0_WORK_EN : 0);
	} else if (type == MESON_EARC_ATTEND_ARC) {
		regmap_update_bits(priv->dmac_map, EARCRX_SPDIFIN_SAMPLE_CTRL0,
				   EARCRX_SPDIFIN_SAMPLE_CTRL0_WORK_EN,
				   enable ? EARCRX_SPDIFIN_SAMPLE_CTRL0_WORK_EN : 0);
		if (!enable)
			regmap_write(priv->dmac_map, EARCRX_DMAC_SYNC_CTRL0, 0);
	}

	regmap_update_bits(priv->dmac_map, EARCRX_DMAC_UBIT_CTRL0,
			   EARCRX_DMAC_UBIT_CTRL0_WORK_EN,
			   enable ? EARCRX_DMAC_UBIT_CTRL0_WORK_EN : 0);
	regmap_update_bits(priv->dmac_map, EARCRX_ERR_CORRECT_CTRL0,
			   EARCRX_ERR_CORRECT_CTRL0_WORK_EN,
			   enable ? EARCRX_ERR_CORRECT_CTRL0_WORK_EN : 0);
	regmap_update_bits(priv->dmac_map, EARCRX_DMAC_TOP_CTRL0,
			   EARCRX_DMAC_TOP_CTRL0_WORK_EN,
			   enable ? EARCRX_DMAC_TOP_CTRL0_WORK_EN : 0);
}

static unsigned int meson_earc_get_cs_bits(struct meson_earc *priv,
					   int cs_offset, int mask)
{
	int reg_offset = cs_offset / 32;
	int bits_offset = cs_offset % 32;
	int stats_sel = reg_offset;
	unsigned int val = 0;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	regmap_update_bits(priv->dmac_map, EARCRX_SPDIFIN_CTRL0,
			   EARCRX_SPDIFIN_CTRL0_STATS_SEL_MASK,
			   stats_sel << 8);
	regmap_read(priv->dmac_map, EARCRX_SPDIFIN_STAT1, &val);
	spin_unlock_irqrestore(&priv->lock, flags);

	return (val >> bits_offset) & mask;
}

static int meson_earc_get_cs_mute(struct meson_earc *priv)
{
	return meson_earc_get_cs_bits(priv, IEC_CS_MUTE_OFFSET, IEC_CS_MUTE_MASK);
}

/*
 * Interrupt handling for SPI 87 (GIC_SPI 87 IRQ_TYPE_LEVEL_HIGH)
 */
static irqreturn_t meson_earc_irq_handler(int irq, void *data)
{
	struct meson_earc *priv = data;
	unsigned int dmac_mask = 0;

	regmap_read(priv->top_map, EARCRX_DMAC_INT_MASK, &dmac_mask);
	regmap_read(priv->top_map, EARCRX_CMDC_INT_PENDING, &priv->rx_status0);
	if (priv->rx_status0)
		regmap_write(priv->top_map, EARCRX_CMDC_INT_PENDING, priv->rx_status0);

	regmap_read(priv->top_map, EARCRX_DMAC_INT_PENDING, &priv->rx_status1);
	priv->rx_status1 &= dmac_mask;
	if (priv->rx_status1)
		regmap_write(priv->top_map, EARCRX_DMAC_INT_PENDING, priv->rx_status1 & 0xefffffff);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t meson_earc_irq_thread(int irq, void *data)
{
	struct meson_earc *priv = data;

	if (priv->rx_status0 & INT_EARCRX_CMDC_TIMEOUT) {
		dev_dbg(priv->dev, "CMDC timeout\n");
		priv->state = meson_earc_get_state(priv);
		priv->attend_type = meson_earc_get_attended_type(priv);
	}
	if (priv->rx_status0 & INT_EARCRX_CMDC_IDLE2) {
		dev_dbg(priv->dev, "CMDC IDLE2 (Discovery)\n");
		priv->state = meson_earc_get_state(priv);
	}
	if (priv->rx_status0 & INT_EARCRX_CMDC_IDLE1) {
		dev_dbg(priv->dev, "CMDC IDLE1 (Disconnected)\n");
		priv->state = meson_earc_get_state(priv);
		priv->attend_type = MESON_EARC_ATTEND_DISCONNECT;
	}
	if (priv->rx_status0 & INT_EARCRX_CMDC_DISC2)
		dev_dbg(priv->dev, "CMDC DISC2\n");
	if (priv->rx_status0 & INT_EARCRX_CMDC_DISC1)
		dev_dbg(priv->dev, "CMDC DISC1\n");
	if (priv->rx_status0 & INT_EARCRX_CMDC_EARC) {
		dev_info(priv->dev, "CMDC eARC connected - performing capability exchange\n");
		meson_earc_set_latency(priv, priv->latency);
		meson_earc_set_cds(priv, priv->cds);
		priv->state = MESON_EARC_CMDC_ST_EARC;
		priv->attend_type = MESON_EARC_ATTEND_EARC;
	}
	if (priv->rx_status0 & INT_EARCRX_CMDC_LOSTHB)
		dev_warn(priv->dev, "CMDC heartbeat lost\n");
	if (priv->rx_status0 & INT_EARCRX_CMDC_HB_STATUS)
		dev_dbg(priv->dev, "CMDC heartbeat status event\n");
	if (priv->rx_status0 & INT_EARCRX_CMDC_STATUS_CH)
		dev_dbg(priv->dev, "CMDC status change\n");

	if (priv->dmac_clk_on) {
		if (priv->rx_status1 & INT_EARCRX_ANA_RST_C_NEW_FORMAT_SET) {
			dev_dbg(priv->dev, "ANA reset: new format set, refreshing PLL\n");
			meson_earc_pll_refresh(priv);
		}
		if (priv->rx_status1 & INT_ARCRX_BIPHASE_DECODE_R_PARITY_ERR) {
			dev_dbg(priv->dev, "Biphase decode parity error, refreshing PLL\n");
			meson_earc_pll_refresh(priv);
		}
		if (priv->rx_status1 & INT_ARCRX_BIPHASE_DECODE_C_CHST_CHANGE) {
			int mute = meson_earc_get_cs_mute(priv);

			if (priv->rx_cs_mute != mute) {
				priv->rx_cs_mute = mute;
				meson_earc_pll_refresh(priv);
			}
		}
		if (priv->rx_status1 & INT_EARCRX_DMAC_VALID_AUTO_NEG_INT_SET) {
			regmap_write(priv->top_map, EARCRX_DMAC_INT_PENDING, BIT(28));
			dev_dbg(priv->dev, "DMAC auto neg valid event\n");
			meson_earc_pll_refresh(priv);
		}
	}

	return IRQ_HANDLED;
}

/*
 * DAI Operations
 */
static int meson_earc_dai_startup(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct meson_earc *priv = snd_soc_dai_get_drvdata(dai);
	unsigned long flags;
	int ret;

	if (substream->stream != SNDRV_PCM_STREAM_CAPTURE)
		return -EINVAL;

	if (priv->clk_rx_dmac_srcpll) {
		ret = clk_prepare_enable(priv->clk_rx_dmac_srcpll);
		if (ret) {
			dev_err(priv->dev, "failed to enable clk_rx_dmac_srcpll: %d\n", ret);
			return ret;
		}
	}

	if (priv->clk_rx_dmac) {
		clk_set_rate(priv->clk_rx_dmac, 500000000);
		ret = clk_prepare_enable(priv->clk_rx_dmac);
		if (ret) {
			dev_err(priv->dev, "failed to enable clk_rx_dmac: %d\n", ret);
			if (priv->clk_rx_dmac_srcpll)
				clk_disable_unprepare(priv->clk_rx_dmac_srcpll);
			return ret;
		}
	}

	spin_lock_irqsave(&priv->lock, flags);
	priv->dmac_clk_on = true;
	spin_unlock_irqrestore(&priv->lock, flags);

	meson_earc_pll_refresh(priv);

	return 0;
}

static void meson_earc_dai_shutdown(struct snd_pcm_substream *substream,
				    struct snd_soc_dai *dai)
{
	struct meson_earc *priv = snd_soc_dai_get_drvdata(dai);
	unsigned long flags;

	if (substream->stream != SNDRV_PCM_STREAM_CAPTURE)
		return;

	spin_lock_irqsave(&priv->lock, flags);
	priv->dmac_clk_on = false;
	spin_unlock_irqrestore(&priv->lock, flags);

	if (priv->clk_rx_dmac)
		clk_disable_unprepare(priv->clk_rx_dmac);
	if (priv->clk_rx_dmac_srcpll)
		clk_disable_unprepare(priv->clk_rx_dmac_srcpll);
}

static int meson_earc_dai_hw_params(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params,
				    struct snd_soc_dai *dai)
{
	return 0;
}

static int meson_earc_dai_prepare(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct meson_earc *priv = snd_soc_dai_get_drvdata(dai);
	enum meson_earc_attend_type type = meson_earc_get_attended_type(priv);

	meson_earc_dmac_init(priv);
	if (type == MESON_EARC_ATTEND_ARC)
		meson_earc_arc_init(priv);

	return 0;
}

static int meson_earc_dai_trigger(struct snd_pcm_substream *substream, int cmd,
				  struct snd_soc_dai *dai)
{
	struct meson_earc *priv = snd_soc_dai_get_drvdata(dai);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		meson_earc_enable(priv, true);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		meson_earc_enable(priv, false);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct snd_soc_dai_ops meson_earc_dai_ops = {
	.startup	= meson_earc_dai_startup,
	.shutdown	= meson_earc_dai_shutdown,
	.hw_params	= meson_earc_dai_hw_params,
	.prepare	= meson_earc_dai_prepare,
	.trigger	= meson_earc_dai_trigger,
};

static struct snd_soc_dai_driver meson_earc_dai_drv = {
	.name = "meson-earc-dai",
	.capture = {
		.stream_name = "eARC Capture",
		.channels_min = 1,
		.channels_max = 32,
		.rates = SNDRV_PCM_RATE_CONTINUOUS | SNDRV_PCM_RATE_8000_192000,
		.rate_min = 8000,
		.rate_max = 192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE |
			   SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_S32_LE |
			   SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_LE,
	},
	.ops = &meson_earc_dai_ops,
};

/*
 * ALSA Mixer Controls
 */
static const char * const meson_earc_attend_texts[] = {
	"Disconnected", "ARC", "eARC"
};
static SOC_ENUM_SINGLE_EXT_DECL(meson_earc_attend_enum, meson_earc_attend_texts);

static const char * const meson_earc_state_texts[] = {
	"Idle1", "Disc1", "Idle2", "Disc2", "eARC", "ARC"
};
static SOC_ENUM_SINGLE_EXT_DECL(meson_earc_state_enum, meson_earc_state_texts);

static int meson_earc_attended_type_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct meson_earc *priv = snd_soc_component_get_drvdata(component);

	priv->attend_type = meson_earc_get_attended_type(priv);
	ucontrol->value.enumerated.item[0] = priv->attend_type;
	return 0;
}

static int meson_earc_state_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct meson_earc *priv = snd_soc_component_get_drvdata(component);
	enum meson_earc_cmdc_state state;

	state = meson_earc_get_state(priv);
	if (state > MESON_EARC_CMDC_ST_ARC)
		state = MESON_EARC_CMDC_ST_IDLE1;
	ucontrol->value.enumerated.item[0] = state;
	return 0;
}

static int meson_earc_latency_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct meson_earc *priv = snd_soc_component_get_drvdata(component);
	u8 latency = 0;

	if (meson_earc_get_state(priv) == MESON_EARC_CMDC_ST_EARC)
		meson_earc_cmdc_read_reg(priv, EARC_CMDC_STAT_CTRL_DEV_ID,
					 EARC_RX_LATENCY_REG, &latency, 1);
	else
		latency = priv->latency;

	ucontrol->value.integer.value[0] = latency;
	return 0;
}

static int meson_earc_latency_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct meson_earc *priv = snd_soc_component_get_drvdata(component);
	u8 latency = ucontrol->value.integer.value[0];

	if (priv->latency == latency)
		return 0;

	priv->latency = latency;
	if (meson_earc_get_state(priv) == MESON_EARC_CMDC_ST_EARC)
		meson_earc_set_latency(priv, latency);

	return 1;
}

static int meson_earc_cds_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct meson_earc *priv = snd_soc_component_get_drvdata(component);

	memcpy(ucontrol->value.bytes.data, priv->cds, EARC_CDS_MAX_BYTES);
	return 0;
}

static int meson_earc_cds_put(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct meson_earc *priv = snd_soc_component_get_drvdata(component);

	if (!memcmp(priv->cds, ucontrol->value.bytes.data, EARC_CDS_MAX_BYTES))
		return 0;

	memcpy(priv->cds, ucontrol->value.bytes.data, EARC_CDS_MAX_BYTES);
	if (meson_earc_get_state(priv) == MESON_EARC_CMDC_ST_EARC)
		meson_earc_set_cds(priv, priv->cds);

	return 1;
}

static int meson_earc_mute_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct meson_earc *priv = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = meson_earc_get_cs_mute(priv);
	return 0;
}

static int meson_earc_arc_connect_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct meson_earc *priv = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = priv->arc_connected;
	return 0;
}

static int meson_earc_arc_connect_put(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct meson_earc *priv = snd_soc_component_get_drvdata(component);
	bool conn = !!ucontrol->value.integer.value[0];

	if (priv->arc_connected == conn)
		return 0;

	priv->arc_connected = conn;
	meson_earc_cmdc_arc_connect(priv, conn);
	return 1;
}

static const struct snd_kcontrol_new meson_earc_controls[] = {
	SOC_ENUM_EXT("eARC Attended Type", meson_earc_attend_enum,
		     meson_earc_attended_type_get, NULL),
	SOC_ENUM_EXT("eARC State", meson_earc_state_enum,
		     meson_earc_state_get, NULL),
	SOC_SINGLE_EXT("eARC Rx Latency", SND_SOC_NOPM, 0, 255, 0,
		       meson_earc_latency_get, meson_earc_latency_put),
	SND_SOC_BYTES_EXT("eARC Rx CDS", EARC_CDS_MAX_BYTES,
			  meson_earc_cds_get, meson_earc_cds_put),
	SOC_SINGLE_BOOL_EXT("eARC Rx Mute", 0,
			    meson_earc_mute_get, NULL),
	SOC_SINGLE_BOOL_EXT("eARC ARC Connect", 0,
			    meson_earc_arc_connect_get, meson_earc_arc_connect_put),
};

static const struct snd_soc_component_driver meson_earc_component_drv = {
	.name = "meson-earc",
	.controls = meson_earc_controls,
	.num_controls = ARRAY_SIZE(meson_earc_controls),
};

static int meson_earc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct meson_earc *priv;
	void __iomem *base;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	spin_lock_init(&priv->lock);
	priv->latency = EARC_DEFAULT_LATENCY;
	priv->arc_connected = true;

	priv->data = of_device_get_match_data(dev);
	if (!priv->data)
		return -ENODEV;

	platform_set_drvdata(pdev, priv);

	/* 1. Map CMDC, DMAC, and TOP registers */
	base = devm_platform_ioremap_resource_byname(pdev, "rx_cmdc");
	if (IS_ERR(base)) {
		base = devm_platform_ioremap_resource(pdev, 0);
		if (IS_ERR(base))
			return dev_err_probe(dev, PTR_ERR(base), "failed to ioremap rx_cmdc\n");
	}
	priv->cmdc_map = devm_regmap_init_mmio(dev, base, &meson_earc_cmdc_regmap_cfg);
	if (IS_ERR(priv->cmdc_map))
		return dev_err_probe(dev, PTR_ERR(priv->cmdc_map), "failed to init rx_cmdc regmap\n");

	base = devm_platform_ioremap_resource_byname(pdev, "rx_dmac");
	if (IS_ERR(base)) {
		base = devm_platform_ioremap_resource(pdev, 1);
		if (IS_ERR(base))
			return dev_err_probe(dev, PTR_ERR(base), "failed to ioremap rx_dmac\n");
	}
	priv->dmac_map = devm_regmap_init_mmio(dev, base, &meson_earc_dmac_regmap_cfg);
	if (IS_ERR(priv->dmac_map))
		return dev_err_probe(dev, PTR_ERR(priv->dmac_map), "failed to init rx_dmac regmap\n");

	base = devm_platform_ioremap_resource_byname(pdev, "rx_top");
	if (IS_ERR(base)) {
		base = devm_platform_ioremap_resource(pdev, 2);
		if (IS_ERR(base))
			return dev_err_probe(dev, PTR_ERR(base), "failed to ioremap rx_top\n");
	}
	priv->top_map = devm_regmap_init_mmio(dev, base, &meson_earc_top_regmap_cfg);
	if (IS_ERR(priv->top_map))
		return dev_err_probe(dev, PTR_ERR(priv->top_map), "failed to init rx_top regmap\n");

	/* 2. Clocks */
	priv->clk_rx_cmdc = devm_clk_get_optional(dev, "rx_cmdc");
	if (IS_ERR(priv->clk_rx_cmdc))
		return dev_err_probe(dev, PTR_ERR(priv->clk_rx_cmdc), "failed to get clk_rx_cmdc\n");

	priv->clk_rx_dmac = devm_clk_get_optional(dev, "rx_dmac");
	if (IS_ERR(priv->clk_rx_dmac))
		return dev_err_probe(dev, PTR_ERR(priv->clk_rx_dmac), "failed to get clk_rx_dmac\n");

	priv->clk_rx_cmdc_srcpll = devm_clk_get_optional(dev, "rx_cmdc_srcpll");
	if (IS_ERR(priv->clk_rx_cmdc_srcpll))
		return dev_err_probe(dev, PTR_ERR(priv->clk_rx_cmdc_srcpll), "failed to get clk_rx_cmdc_srcpll\n");

	priv->clk_rx_dmac_srcpll = devm_clk_get_optional(dev, "rx_dmac_srcpll");
	if (IS_ERR(priv->clk_rx_dmac_srcpll))
		return dev_err_probe(dev, PTR_ERR(priv->clk_rx_dmac_srcpll), "failed to get clk_rx_dmac_srcpll\n");

	if (priv->clk_rx_cmdc && priv->clk_rx_cmdc_srcpll) {
		ret = clk_set_parent(priv->clk_rx_cmdc, priv->clk_rx_cmdc_srcpll);
		if (ret)
			dev_warn(dev, "failed to set clk_rx_cmdc parent: %d\n", ret);
	}
	if (priv->clk_rx_dmac && priv->clk_rx_dmac_srcpll) {
		ret = clk_set_parent(priv->clk_rx_dmac, priv->clk_rx_dmac_srcpll);
		if (ret)
			dev_warn(dev, "failed to set clk_rx_dmac parent: %d\n", ret);
	}

	if (priv->clk_rx_cmdc) {
		clk_set_rate(priv->clk_rx_cmdc, 10000000);
		ret = clk_prepare_enable(priv->clk_rx_cmdc);
		if (ret)
			return dev_err_probe(dev, ret, "failed to enable clk_rx_cmdc\n");
	}

	/* 3. Interrupt */
	priv->irq = platform_get_irq_byname_optional(pdev, "earc_rx");
	if (priv->irq < 0) {
		priv->irq = platform_get_irq(pdev, 0);
		if (priv->irq < 0) {
			ret = priv->irq;
			goto err_clk_cmdc;
		}
	}

	ret = devm_request_threaded_irq(dev, priv->irq,
					meson_earc_irq_handler,
					meson_earc_irq_thread,
					IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
					"meson-earc", priv);
	if (ret) {
		dev_err_probe(dev, ret, "failed to request eARC IRQ %d\n", priv->irq);
		goto err_clk_cmdc;
	}

	/* 4. Hardware CMDC init */
	meson_earc_cmdc_init(priv);
	meson_earc_cmdc_int_mask(priv);
	meson_earc_cmdc_arc_connect(priv, true);
	meson_earc_cmdc_hpd_detect(priv, true);

	/* 5. Register ASoC Component */
	ret = devm_snd_soc_register_component(dev, &meson_earc_component_drv,
					       &meson_earc_dai_drv, 1);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register ASoC component\n");
		goto err_clk_cmdc;
	}

	dev_info(dev, "Amlogic eARC Receiver probed (irq %d)\n", priv->irq);
	return 0;

err_clk_cmdc:
	if (priv->clk_rx_cmdc)
		clk_disable_unprepare(priv->clk_rx_cmdc);
	return ret;
}

static void meson_earc_remove(struct platform_device *pdev)
{
	struct meson_earc *priv = platform_get_drvdata(pdev);

	meson_earc_enable(priv, false);
	meson_earc_cmdc_arc_connect(priv, false);
	meson_earc_cmdc_hpd_detect(priv, false);

	if (priv->clk_rx_cmdc)
		clk_disable_unprepare(priv->clk_rx_cmdc);
}

static const struct meson_earc_match_data sc2_earc_data = {
	.rx_dmac_sync_int = true,
	.rterm_on = false,
	.unstable_tick_sel = false,
	.chnum_mult_mode = false,
};

static const struct meson_earc_match_data tm2_revb_earc_data = {
	.rx_dmac_sync_int = true,
	.rterm_on = false,
	.unstable_tick_sel = false,
	.chnum_mult_mode = false,
};

static const struct meson_earc_match_data t7_earc_data = {
	.rx_dmac_sync_int = false,
	.rterm_on = true,
	.unstable_tick_sel = false,
	.chnum_mult_mode = false,
};

static const struct of_device_id meson_earc_of_match[] = {
	{ .compatible = "amlogic,meson-sc2-earc", .data = &sc2_earc_data },
	{ .compatible = "amlogic,tm2-revb-snd-earc", .data = &tm2_revb_earc_data },
	{ .compatible = "amlogic,t7-snd-earc", .data = &t7_earc_data },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, meson_earc_of_match);

static struct platform_driver meson_earc_pdrv = {
	.probe = meson_earc_probe,
	.remove = meson_earc_remove,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = meson_earc_of_match,
	},
};
module_platform_driver(meson_earc_pdrv);

MODULE_DESCRIPTION("Amlogic Meson eARC Receiver Driver");
MODULE_AUTHOR("CoreELEC & LibreELEC developers");
MODULE_LICENSE("GPL");
