// SPDX-License-Identifier: GPL-2.0-only
/* SC2 HEVC Request engine. Register protocol from Amlogic vh265.c/amvdec.c;
 * the native Request controls, not firmware history, own picture/reference state.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/firmware/meson/meson_sm.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/iommu.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/overflow.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/pm_runtime.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/soc/amlogic/meson-sc2-dos.h>
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/videodev2.h>

#include "meson_sc2_hw.h"
#include "hevc_request_syntax.h"
#include "../../../../staging/media/meson/vdec/hevc_regs.h"
#include "av1_request_syntax.h"

struct sc2_av1_state;
static void sc2_av1_reset(struct meson_sc2_hevc_context *ctx);
static void sc2_av1_release(struct meson_sc2_hevc_context *ctx);
struct sc2_vp9_state;
struct sc2_avs2_state;
static void sc2_vp9_reset(struct meson_sc2_hevc_context *ctx);
static void sc2_vp9_release(struct meson_sc2_hevc_context *ctx);
static void sc2_avs2_reset(struct meson_sc2_hevc_context *ctx);
static void sc2_avs2_release(struct meson_sc2_hevc_context *ctx);

#define SC2_MAX_WIDTH                  4096U
#define SC2_MAX_HEIGHT                 2304U
#define SC2_INPUT_SIZE                 SZ_16M
#define SC2_INPUT_PADDING              SZ_16K
#define SC2_GUARD_SIZE                 SZ_4K
#define SC2_GUARD_BYTE                 0xa5
#define SC2_TIMEOUT_US                 100000
#define SC2_JOB_TIMEOUT                msecs_to_jiffies(2000)
#define SC2_DOS_GCLK_EN3                0xfcd4
#define SC2_DOS_SW_RESET3               0xfcd0
#define SC2_DOS_MEM_PD_HEVC             0xfccc
#define SC2_MBOX_IRQ                    0xc1c0
#define SC2_MBOX_CLEAR                  0xc1c4
#define SC2_MBOX_MASK                   0xc1c8
#define SC2_LMEM_DMA_CTRL               0xcd40
#define SC2_WRRSP_LMEM                  0xcd4c
#define SC2_SAO_MMU_RESET               0xd904
#define SC2_SAO_CTRL9                   0xd8b4
#define SC2_SAO_MMU_STATUS              0xd8e4
#define SC2_MPRED_LT_REF                0xc828
#define SC2_DMC_PORTS                   (BIT(4) | BIT(8))
#define SC2_BRIDGE_PORTS                (BIT(1) | BIT(12) | BIT(13))
#define SC2_CORE_RESETS                 (BIT(3) | BIT(4) | BIT(8) | BIT(10) | \
	BIT(11) | BIT(12) | BIT(13) | BIT(14) | BIT(15) | BIT(17) | \
	BIT(18) | BIT(19) | BIT(24) | BIT(26))
#define SC2_FW_HEADER                  8
#define SC2_FW_DATA                    5
#define SC2_FW_DONE                    0x0a
#define SC2_FW_ACTION                  0xff
#define SC2_CPU_REQUEST                4

/* Disjoint, page-aligned capacities for the complete 4096x2304 envelope.
 * SAO tile scratch is multiplied by tile rows/columns, not picture count.
 */
#define W_IPP                          0x000000
#define W_SAO_ABOVE                    0x004000
#define W_SAO_VERTICAL                 0x104000
#define W_RPS                          0x204000
#define W_VPS                          0x205000
#define W_SPS                          0x206000
#define W_PPS                          0x207000
#define W_SAO_UP                       0x209000
#define W_SCALE                        0x20c000
#define W_DBLK_PARA                    0x214000
#define W_DBLK_DATA                    0x294000
#define W_DBLK_DATA2                   0x394000
#define W_MMU_VBH                      0x494000
#define W_MPRED_ABOVE                  0x499000
#define W_RPM                          0x4a1000
#define W_LMEM                         0x4a2000
#define W_MAP                          0x510000
#define W_AUX                          0x520000
#define SC2_WORK_SIZE                  0x530000
#define SC2_MAP_ENTRIES                (SZ_64K / sizeof(u32))

static const u16 sc2_parser_commands[] = {
	0x0401, 0x8401, 0x0800, 0x0402, 0x9002, 0x1423, 0x8cc3, 0x1423,
	0x8804, 0x9825, 0x0800, 0x04fe, 0x8406, 0x8411, 0x1800, 0x8408,
	0x8409, 0x8c2a, 0x9c2b, 0x1c00, 0x840f, 0x8407, 0x8000, 0x8408,
	0x2000, 0xa800, 0x8410, 0x04de, 0x840c, 0x840d, 0xac00, 0xa000,
	0x08c0, 0x08e0, 0xa40e, 0xfc00, 0x7c00,
};

struct sc2_surface_owner {
	struct list_head list;
	struct meson_sc2_surface surface;
	struct meson_sc2_hevc_context *context;
	struct dma_buf *capture_buffer;
	void *private_mv;
	/* Additional picture pins, protected by hw->lock. */
	unsigned int references;
	bool detached;
};

struct meson_sc2_hevc_context {
	struct list_head list;
	struct meson_sc2_hw *hw;
	void *work;
	dma_addr_t work_dma;
	void *input;
	dma_addr_t input_dma;
	void *dw8;
	dma_addr_t dw8_dma;
	u32 dw8_size;
	atomic_t aborted;
	struct sc2_av1_state *av1;
	struct sc2_vp9_state *vp9;
	struct sc2_avs2_state *avs2;
};

struct meson_sc2_hw {
	struct device *dev;
	struct meson_sm_firmware *firmware;
	void __iomem *dos;
	void __iomem *dmc;
	void __iomem *bridge;
	struct resource dos_resource;
	struct clk_bulk_data clocks[3];
	struct mutex lock;
	struct list_head contexts;
	struct list_head surfaces;
	wait_queue_head_t wait;
	unsigned int irq;
	u32 irq_sequence;
	u32 gates_before;
	u32 dmc_before;
	bool powered;
	bool irq_requested;
	bool fatal;
	struct dma_buf *output_buffer;
	struct dma_buf_attachment *output_attachment;
	struct sg_table *output_mapping;
	dma_addr_t output_dma;
};

static u32 sc2_read(struct meson_sc2_hw *hw, u32 reg)
{
	return readl(hw->dos + reg);
}

static void sc2_write(struct meson_sc2_hw *hw, u32 reg, u32 value)
{
	writel(value, hw->dos + reg);
}

static bool sc2_dma_range(dma_addr_t dma, size_t size)
{
	return size && dma <= U32_MAX && size - 1 <= U32_MAX - dma;
}

static irqreturn_t sc2_irq(int irq, void *data)
{
	struct meson_sc2_hw *hw = data;

	sc2_write(hw, SC2_MBOX_CLEAR, 1);
	/* Status remains latched until the worker acknowledges the command. */
	smp_store_release(&hw->irq_sequence, READ_ONCE(hw->irq_sequence) + 1);
	wake_up(&hw->wait);
	return IRQ_HANDLED;
}

static int sc2_disable_dmc(struct meson_sc2_hw *hw)
{
	u32 value;

	writel(readl(hw->dmc) & ~SC2_DMC_PORTS, hw->dmc);
	return readl_poll_timeout(hw->dmc + 0xf0, value,
		(value & SC2_DMC_PORTS) == SC2_DMC_PORTS, 10, SC2_TIMEOUT_US);
}

static int sc2_drain(struct meson_sc2_hw *hw, bool *recovered)
{
	u32 value;
	int ret, dma_ret;

	*recovered = false;
	sc2_write(hw, SC2_MBOX_MASK, 0);
	sc2_write(hw, HEVC_MPSR, 0);
	sc2_write(hw, HEVC_CPSR, 0);
	sc2_write(hw, HEVC_STREAM_CONTROL, 0);
	dma_ret = readl_poll_timeout(hw->dos + HEVC_IMEM_DMA_CTRL, value,
		!(value & BIT(15)), 10, SC2_TIMEOUT_US);
	ret = readl_poll_timeout(hw->dos + SC2_LMEM_DMA_CTRL, value,
		!(value & BIT(15)), 10, SC2_TIMEOUT_US);
	if (ret)
		dma_ret = ret;
	ret = sc2_disable_dmc(hw);
	if (ret || dma_ret) {
		*recovered = true;
		/* Bounded recovery: halt all sources, then require both port ACKs
		 * and independently recheck DMA; reset is never an idle proof.
		 */
		sc2_write(hw, HEVC_PARSER_CORE_CONTROL, 0);
		sc2_write(hw, HEVC_CABAC_CONTROL, 0);
		sc2_write(hw, SC2_DOS_SW_RESET3, SC2_CORE_RESETS);
		sc2_write(hw, SC2_DOS_SW_RESET3, 0);
		ret = sc2_disable_dmc(hw);
		if (ret)
			return ret;
		ret = readl_poll_timeout(hw->dos + HEVC_IMEM_DMA_CTRL, value,
			!(value & BIT(15)), 10, SC2_TIMEOUT_US);
		if (ret)
			return ret;
		ret = readl_poll_timeout(hw->dos + SC2_LMEM_DMA_CTRL, value,
			!(value & BIT(15)), 10, SC2_TIMEOUT_US);
		if (ret)
			return ret;
	}
	ret = readl_poll_timeout(hw->dos + HEVC_MPSR, value, !value,
		10, SC2_TIMEOUT_US);
	if (ret)
		return ret;
	ret = readl_poll_timeout(hw->dos + HEVC_CPSR, value, !value,
		10, SC2_TIMEOUT_US);
	if (ret)
		return ret;
	sc2_write(hw, HEVC_PARSER_CORE_CONTROL, 0);
	sc2_write(hw, HEVC_CABAC_CONTROL, 0);
	sc2_write(hw, SC2_DOS_SW_RESET3, SC2_CORE_RESETS);
	sc2_write(hw, SC2_DOS_SW_RESET3, 0);
	ret = readl_poll_timeout(hw->dos + SC2_WRRSP_LMEM, value,
		!(value & 0xfff), 10, SC2_TIMEOUT_US);
	if (ret)
		return ret;
	/* Recheck after the final reset; pre-reset ACKs do not retire it. */
	ret = sc2_disable_dmc(hw);
	if (ret)
		return ret;
	ret = readl_poll_timeout(hw->dos + HEVC_IMEM_DMA_CTRL, value,
		!(value & BIT(15)), 10, SC2_TIMEOUT_US);
	if (ret)
		return ret;
	ret = readl_poll_timeout(hw->dos + SC2_LMEM_DMA_CTRL, value,
		!(value & BIT(15)), 10, SC2_TIMEOUT_US);
	if (ret)
		return ret;
	sc2_write(hw, SC2_MBOX_CLEAR, 1);
	synchronize_irq(hw->irq);
	return ret;
}

static void sc2_quarantine(struct meson_sc2_hw *hw, int error)
{
	if (hw->fatal)
		return;
	hw->fatal = true;
	__module_get(THIS_MODULE);
	dev_crit(hw->dev,
		"HEVC DMA quiescence failed (%d): device, power, output dma-buf and all decoder allocations quarantined; controlled reboot required\n",
		error);
}

static int sc2_power_on(struct meson_sc2_hw *hw)
{
	int ret;

	ret = meson_sc2_dos_acquire(hw->dev, MESON_SC2_DOS_HEVC);
	if (ret)
		return ret;
	ret = pm_runtime_resume_and_get(hw->dev);
	if (ret < 0)
		goto release_dos;
	ret = clk_bulk_prepare_enable(ARRAY_SIZE(hw->clocks), hw->clocks);
	if (ret) {
		pm_runtime_put_sync_suspend(hw->dev);
		goto release_dos;
	}
	hw->powered = true;
	hw->gates_before = sc2_read(hw, SC2_DOS_GCLK_EN3);
	hw->dmc_before = readl(hw->dmc);
	sc2_write(hw, SC2_DOS_GCLK_EN3, U32_MAX);
	sc2_read(hw, SC2_DOS_GCLK_EN3);
	if (hw->irq_requested)
		enable_irq(hw->irq);
	return 0;

release_dos:
	meson_sc2_dos_release(hw->dev, MESON_SC2_DOS_HEVC);
	return ret;
}

static void sc2_power_off(struct meson_sc2_hw *hw)
{
	if (!hw->powered || hw->fatal)
		return;
	if (hw->irq_requested)
		disable_irq(hw->irq);
	writel((readl(hw->dmc) & ~SC2_DMC_PORTS) |
	       (hw->dmc_before & SC2_DMC_PORTS), hw->dmc);
	readl(hw->dmc);
	sc2_write(hw, SC2_DOS_GCLK_EN3, hw->gates_before);
	sc2_read(hw, SC2_DOS_GCLK_EN3);
	clk_bulk_disable_unprepare(ARRAY_SIZE(hw->clocks), hw->clocks);
	pm_runtime_put_sync_suspend(hw->dev);
	hw->powered = false;
	meson_sc2_dos_release(hw->dev, MESON_SC2_DOS_HEVC);
}

int meson_sc2_hw_create(struct device *dev, struct meson_sc2_hw **result)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct device_node *np;
	struct meson_sc2_hw *hw;
	struct resource *res;
	int ret;

	if (!result)
		return -EINVAL;
	*result = NULL;
	hw = kzalloc(sizeof(*hw), GFP_KERNEL);
	if (!hw)
		return -ENOMEM;
	hw->dev = get_device(dev);
	mutex_init(&hw->lock);
	INIT_LIST_HEAD(&hw->contexts);
	INIT_LIST_HEAD(&hw->surfaces);
	init_waitqueue_head(&hw->wait);
	np = of_find_compatible_node(NULL, NULL, "amlogic,meson-sc2-sm");
	if (!np) {
		ret = -ENODEV;
		goto free_hw;
	}
	hw->firmware = meson_sm_get(np);
	of_node_put(np);
	if (!hw->firmware) {
		ret = -EPROBE_DEFER;
		goto free_hw;
	}
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		goto free_hw;
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dos");
	if (!res || resource_size(res) < SZ_64K) {
		ret = -EINVAL;
		goto free_hw;
	}
	hw->dos_resource = *res;
	if (!request_mem_region(res->start, resource_size(res), dev_name(dev))) {
		ret = -EBUSY;
		goto free_hw;
	}
	hw->dos = ioremap(res->start, resource_size(res));
	if (!hw->dos) {
		ret = -ENOMEM;
		goto release_dos;
	}
	/* DMC and bridge contain registers belonging to other devices. */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dmc");
	if (!res || resource_size(res) < 0x100) {
		ret = -EINVAL;
		goto unmap;
	}
	hw->dmc = ioremap(res->start, resource_size(res));
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "bridge");
	if (!res || resource_size(res) < sizeof(u32)) {
		ret = -EINVAL;
		goto unmap;
	}
	hw->bridge = ioremap(res->start, resource_size(res));
	if (!hw->dmc || !hw->bridge) {
		ret = -ENOMEM;
		goto unmap;
	}
	hw->clocks[0].id = "dos";
	hw->clocks[1].id = "hevcf_mux";
	hw->clocks[2].id = "hevcb_mux";
	ret = clk_bulk_get(dev, ARRAY_SIZE(hw->clocks), hw->clocks);
	if (ret)
		goto unmap;
	ret = platform_get_irq(pdev, 0);
	if (ret < 0)
		goto put_clocks;
	hw->irq = ret;
	pm_runtime_enable(dev);
	ret = sc2_power_on(hw);
	if (ret)
		goto disable_pm;
	if (sc2_read(hw, HEVC_MPSR) || sc2_read(hw, HEVC_CPSR) ||
	    (sc2_read(hw, HEVC_IMEM_DMA_CTRL) & BIT(15)) ||
	    (sc2_read(hw, SC2_LMEM_DMA_CTRL) & BIT(15))) {
		ret = -EBUSY;
		goto power_off;
	}
	sc2_write(hw, SC2_MBOX_MASK, 0);
	sc2_write(hw, SC2_MBOX_CLEAR, 1);
	ret = request_irq(hw->irq, sc2_irq, 0, dev_name(dev), hw);
	if (ret)
		goto power_off;
	hw->irq_requested = true;
	sc2_power_off(hw);
	*result = hw;
	return 0;

power_off:
	sc2_power_off(hw);
disable_pm:
	pm_runtime_disable(dev);
put_clocks:
	clk_bulk_put(ARRAY_SIZE(hw->clocks), hw->clocks);
unmap:
	iounmap(hw->bridge);
	iounmap(hw->dmc);
	iounmap(hw->dos);
release_dos:
	release_mem_region(hw->dos_resource.start,
			   resource_size(&hw->dos_resource));
free_hw:
	put_device(hw->dev);
	kfree(hw);
	return ret;
}

int meson_sc2_hevc_context_create(struct meson_sc2_hw *hw,
				 struct meson_sc2_hevc_context **result)
{
	struct meson_sc2_hevc_context *ctx;
	int ret = -ENOMEM;

	if (!hw || !result)
		return -EINVAL;
	*result = NULL;
	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->hw = hw;
	atomic_set(&ctx->aborted, 0);
	ctx->work = dma_alloc_coherent(hw->dev, SC2_WORK_SIZE, &ctx->work_dma,
				       GFP_KERNEL);
	ctx->input = dma_alloc_coherent(hw->dev, SC2_INPUT_SIZE, &ctx->input_dma,
					GFP_KERNEL);
	if (!ctx->work || !ctx->input)
		goto free_dma;
	if (!sc2_dma_range(ctx->work_dma, SC2_WORK_SIZE) ||
	    !sc2_dma_range(ctx->input_dma, SC2_INPUT_SIZE + 1)) {
		ret = -EOVERFLOW;
		goto free_dma;
	}
	mutex_lock(&hw->lock);
	if (hw->fatal) {
		mutex_unlock(&hw->lock);
		ret = -EIO;
		goto free_dma;
	}
	list_add_tail(&ctx->list, &hw->contexts);
	mutex_unlock(&hw->lock);
	*result = ctx;
	return 0;

free_dma:
	if (ctx->input)
		dma_free_coherent(hw->dev, SC2_INPUT_SIZE, ctx->input,
				  ctx->input_dma);
	if (ctx->work)
		dma_free_coherent(hw->dev, SC2_WORK_SIZE, ctx->work, ctx->work_dma);
	kfree(ctx);
	return ret;
}

void meson_sc2_hevc_abort(struct meson_sc2_hevc_context *ctx)
{
	if (!ctx)
		return;
	atomic_set(&ctx->aborted, 1);
	wake_up(&ctx->hw->wait);
}

bool meson_sc2_hevc_context_fatal(struct meson_sc2_hevc_context *ctx)
{
	return !ctx || READ_ONCE(ctx->hw->fatal);
}

int meson_sc2_hevc_context_reset(struct meson_sc2_hevc_context *ctx)
{
	struct sc2_surface_owner *owner;
	int ret = 0;

	if (!ctx)
		return -EINVAL;
	mutex_lock(&ctx->hw->lock);
	if (ctx->hw->fatal)
		ret = -EIO;
	else {
		list_for_each_entry(owner, &ctx->hw->surfaces, list)
			if (owner->context == ctx)
				owner->surface.valid = false;
		atomic_set(&ctx->aborted, 0);
		sc2_av1_reset(ctx);
		sc2_vp9_reset(ctx);
		sc2_avs2_reset(ctx);
	}
	mutex_unlock(&ctx->hw->lock);
	return ret;
}

void meson_sc2_hevc_context_destroy(struct meson_sc2_hevc_context *ctx)
{
	struct meson_sc2_hw *hw;
	struct sc2_surface_owner *owner;

	if (!ctx)
		return;
	hw = ctx->hw;
	meson_sc2_hevc_abort(ctx);
	/* Decode holds this lock through the final DMA drain. */
	mutex_lock(&hw->lock);
	if (hw->fatal) {
		mutex_unlock(&hw->lock);
		return;
	}
	list_for_each_entry(owner, &hw->surfaces, list)
		if (owner->context == ctx) {
			owner->surface.valid = false;
			owner->context = NULL;
		}
	list_del(&ctx->list);
	sc2_av1_release(ctx);
	sc2_vp9_release(ctx);
	sc2_avs2_release(ctx);
	if (ctx->dw8)
		dma_free_coherent(hw->dev, ctx->dw8_size + SC2_GUARD_SIZE,
				  ctx->dw8, ctx->dw8_dma);
	dma_free_coherent(hw->dev, SC2_INPUT_SIZE, ctx->input, ctx->input_dma);
	dma_free_coherent(hw->dev, SC2_WORK_SIZE, ctx->work, ctx->work_dma);
	mutex_unlock(&hw->lock);
	kfree(ctx);
}

/*
 * Scatter headers contain one u32 address per 32x4 block, tiled in 64x64
 * groups (2 * 16 entries). The legacy 128x64/32-byte HEADER_LENGTH value
 * describes the decompressor register, not the MMU scatter-header storage.
 */
static u32 sc2_fbc_header_alloc_size(u32 width, u32 height)
{
	return ALIGN(DIV_ROUND_UP(width, 64) * DIV_ROUND_UP(height, 64) * 128,
		     SZ_4K);
}

static int sc2_surface_alloc(struct meson_sc2_hw *hw,
			    struct meson_sc2_surface *surface,
			    u32 width, u32 height, u8 depth, bool av1)
{
	struct sc2_surface_owner *owner;
	struct meson_sc2_surface s = {};
	int ret = 0;

	if (!hw || !surface || surface->cpu || !width || !height ||
	    width > SC2_MAX_WIDTH || height > SC2_MAX_HEIGHT || depth != 8)
		return depth == 10 ? -EOPNOTSUPP : -EINVAL;
	s.width = width;
	s.height = height;
	s.depth = depth;
	s.header_size = DIV_ROUND_UP(width, 128) * DIV_ROUND_UP(height, 64) * 32;
	s.header_alloc_size = sc2_fbc_header_alloc_size(width, height);
	s.body_size = DIV_ROUND_UP(width, 64) * DIV_ROUND_UP(height, 32) * 3200;
	s.body_offset = s.header_alloc_size + SC2_GUARD_SIZE;
	s.mv_offset = s.body_offset + ALIGN(s.body_size, SZ_4K) + SC2_GUARD_SIZE;
	/* Maximal CTU rounding also covers CTU16/32 MV layouts. */
	s.mv_size = ALIGN(DIV_ROUND_UP(width, 64) * DIV_ROUND_UP(height, 64) *
			  0x200, SZ_64K);
	if (av1)
		s.mv_size = max_t(u32, s.mv_size, SZ_1M);
	s.size = s.mv_offset + s.mv_size + SC2_GUARD_SIZE;
	if (DIV_ROUND_UP(s.body_size, SZ_4K) > SC2_MAP_ENTRIES)
		return -EOVERFLOW;
	owner = kzalloc(sizeof(*owner), GFP_KERNEL);
	if (!owner)
		return -ENOMEM;
	s.cpu = dma_alloc_coherent(hw->dev, s.size, &s.dma, GFP_KERNEL);
	if (!s.cpu) {
		kfree(owner);
		return -ENOMEM;
	}
	s.mv_dma = s.dma + s.mv_offset;
	if (!sc2_dma_range(s.dma, s.size) || !IS_ALIGNED(s.dma, SZ_4K)) {
		ret = -EOVERFLOW;
		goto free_surface;
	}
	memset(s.cpu, SC2_GUARD_BYTE, s.size);
	memset(s.cpu, 0, s.header_alloc_size);
	memset(s.cpu + s.body_offset, 0, ALIGN(s.body_size, SZ_4K));
	memset(s.cpu + s.mv_offset, 0, s.mv_size);
	owner->surface = s;
	mutex_lock(&hw->lock);
	if (hw->fatal) {
		ret = -EIO;
	} else {
		list_add_tail(&owner->list, &hw->surfaces);
		*surface = s;
	}
	mutex_unlock(&hw->lock);
	if (!ret)
		return 0;
free_surface:
	dma_free_coherent(hw->dev, s.size, s.cpu, s.dma);
	kfree(owner);
	return ret;
}

int meson_sc2_surface_alloc(struct meson_sc2_hw *hw,
			   struct meson_sc2_surface *surface,
			   u32 width, u32 height, u8 depth)
{
	return sc2_surface_alloc(hw, surface, width, height, depth, false);
}

int meson_sc2_av1_surface_alloc(struct meson_sc2_hw *hw,
			       struct meson_sc2_surface *surface,
			       u32 width, u32 height)
{
	return sc2_surface_alloc(hw, surface, width, height, 8, true);
}

/* AMF0/AMF8 keep all absolute scatter-page addresses inside the exported object. */
u32 meson_sc2_hevc_fbc_size(u32 width, u32 height)
{
	u32 header = sc2_fbc_header_alloc_size(width, height);
	u32 body = DIV_ROUND_UP(width, 64) * (ALIGN(height, 64) / 32) * SZ_4K;

	return header + SC2_GUARD_SIZE + body + SC2_GUARD_SIZE;
}

static int sc2_fbc_bind(struct meson_sc2_hevc_context *ctx,
			struct meson_sc2_surface *surface,
			u32 width, u32 height, u8 depth, u32 mv_unit, bool dw8, bool av1,
			void *cpu, dma_addr_t dma, size_t size,
			struct dma_buf *buffer)
{
	struct meson_sc2_surface s = {};
	struct sc2_surface_owner *owner;
	struct meson_sc2_hw *hw;
	u32 dw_size, columns = DIV_ROUND_UP(width, 64);
	u32 rows = DIV_ROUND_UP(height, 64);
	int ret = 0;

	if (!ctx || !surface || surface->cpu || !cpu || !buffer ||
	    !width || !height || width > SC2_MAX_WIDTH || height > SC2_MAX_HEIGHT ||
	    (depth != 8 && depth != 10) || (mv_unit != 0x200 && mv_unit != 0x240))
		return -EINVAL;
	hw = ctx->hw;
	/* Scatter headers encode physical PFNs, not relocatable IOVAs. */
	if (device_iommu_mapped(hw->dev))
		return -EOPNOTSUPP;
	s.size = meson_sc2_hevc_fbc_size(width, height);
	if (size < s.size || buffer->size < s.size ||
	    !IS_ALIGNED(dma, SZ_4K) || !sc2_dma_range(dma, s.size))
		return -EINVAL;
	s.cpu = cpu;
	s.dma = dma;
	s.width = width;
	s.height = height;
	s.depth = depth;
	s.capture_owned = true;
	s.header_size = DIV_ROUND_UP(width, 128) * DIV_ROUND_UP(height, 64) * 32;
	s.header_alloc_size = sc2_fbc_header_alloc_size(width, height);
	s.body_offset = s.header_alloc_size + SC2_GUARD_SIZE;
	s.body_size = DIV_ROUND_UP(width, 64) * (ALIGN(height, 64) / 32) * SZ_4K;
	/* VP9 cal_mv_buf_size adds a per-column tail on SC2, beyond 0x240
	 * per superblock. It is not safe to use the older staging MV size.
	 */
	s.mv_size = ALIGN(columns * rows * mv_unit +
			  (mv_unit == 0x240 ? columns * 16 * ((rows >> 3) + (rows & 7)) : 0),
			  SZ_64K);
	if (av1)
		s.mv_size = max_t(u32, s.mv_size, SZ_1M);
	if (s.body_size / SZ_4K > SC2_MAP_ENTRIES)
		return -EOVERFLOW;
	owner = kzalloc(sizeof(*owner), GFP_KERNEL);
	if (!owner)
		return -ENOMEM;
	owner->private_mv = dma_alloc_coherent(hw->dev, s.mv_size + SC2_GUARD_SIZE,
					     &s.mv_dma, GFP_KERNEL);
	if (!owner->private_mv) {
		ret = -ENOMEM;
		goto free_owner;
	}
	if (!sc2_dma_range(s.mv_dma, s.mv_size + SC2_GUARD_SIZE)) {
		ret = -EOVERFLOW;
		goto free_mv;
	}
	memset(owner->private_mv, 0, s.mv_size);
	memset(owner->private_mv + s.mv_size, SC2_GUARD_BYTE, SC2_GUARD_SIZE);
	dw_size = ALIGN(width, 64) * ALIGN(height, 64) * 3 / 2;
	mutex_lock(&hw->lock);
	if (hw->fatal) {
		ret = -EIO;
		goto unlock;
	}
	/* Reused per context, never exported or described as ten-bit pixels. */
	if (dw8 && ctx->dw8_size < dw_size) {
		void *dw;
		dma_addr_t dw_dma;

		dw = dma_alloc_coherent(hw->dev, dw_size + SC2_GUARD_SIZE,
					&dw_dma, GFP_KERNEL);
		if (!dw) {
			ret = -ENOMEM;
			goto unlock;
		}
		if (!sc2_dma_range(dw_dma, dw_size + SC2_GUARD_SIZE)) {
			dma_free_coherent(hw->dev, dw_size + SC2_GUARD_SIZE, dw, dw_dma);
			ret = -EOVERFLOW;
			goto unlock;
		}
		if (ctx->dw8)
			dma_free_coherent(hw->dev, ctx->dw8_size + SC2_GUARD_SIZE,
					  ctx->dw8, ctx->dw8_dma);
		ctx->dw8 = dw;
		ctx->dw8_dma = dw_dma;
		ctx->dw8_size = dw_size;
		memset(dw + dw_size, SC2_GUARD_BYTE, SC2_GUARD_SIZE);
	}
	memset(cpu, 0, s.header_alloc_size);
	memset(cpu + s.header_alloc_size, SC2_GUARD_BYTE, SC2_GUARD_SIZE);
	memset(cpu + s.body_offset + s.body_size, SC2_GUARD_BYTE, SC2_GUARD_SIZE);
	get_dma_buf(buffer);
	owner->capture_buffer = buffer;
	owner->surface = s;
	owner->context = ctx;
	list_add_tail(&owner->list, &hw->surfaces);
	*surface = s;
unlock:
	mutex_unlock(&hw->lock);
	if (!ret)
		return 0;
free_mv:
	dma_free_coherent(hw->dev, s.mv_size + SC2_GUARD_SIZE,
			  owner->private_mv, s.mv_dma);
free_owner:
	kfree(owner);
	return ret;
}

int meson_sc2_hevc_fbc_bind(struct meson_sc2_hevc_context *ctx,
			   struct meson_sc2_surface *surface,
			   u32 width, u32 height, u8 depth, void *cpu,
			   dma_addr_t dma, size_t size, struct dma_buf *buffer)
{
	return sc2_fbc_bind(ctx, surface, width, height, depth, 0x200, true, false,
			    cpu, dma, size, buffer);
}

int meson_sc2_modern_fbc_bind(struct meson_sc2_hevc_context *ctx,
			     struct meson_sc2_surface *surface,
			     u32 width, u32 height, u8 depth, u32 mv_unit,
			     void *cpu, dma_addr_t dma, size_t size,
			     struct dma_buf *buffer)
{
	return sc2_fbc_bind(ctx, surface, width, height, depth, mv_unit, false, false,
			    cpu, dma, size, buffer);
}

int meson_sc2_av1_fbc_bind(struct meson_sc2_hevc_context *ctx,
			  struct meson_sc2_surface *surface,
			  u32 width, u32 height, void *cpu, dma_addr_t dma,
			  size_t size, struct dma_buf *buffer)
{
	/* SC2 cal_mv_buf_size (19/76 vectors per SB64/SB128) fits in this
	 * conservative CTU64 allocation, with the existing 1 MiB AV1 minimum.
	 * Full-depth CAPTURE is the first write; no DW8 allocation or copy.
	 */
	return sc2_fbc_bind(ctx, surface, width, height, 10, 0x200, false, true,
			    cpu, dma, size, buffer);
}

static void sc2_surface_owner_free(struct meson_sc2_hw *hw,
				   struct sc2_surface_owner *owner)
{
	list_del(&owner->list);
	if (owner->capture_buffer) {
		dma_free_coherent(hw->dev,
			owner->surface.mv_size + SC2_GUARD_SIZE,
			owner->private_mv, owner->surface.mv_dma);
		dma_buf_put(owner->capture_buffer);
	} else {
		dma_free_coherent(hw->dev, owner->surface.size,
				  owner->surface.cpu, owner->surface.dma);
	}
	kfree(owner);
}

static void sc2_surface_owner_put(struct meson_sc2_hw *hw,
				  struct sc2_surface_owner *owner)
{
	if (WARN_ON(!owner->references))
		return;
	owner->references--;
	if (!owner->references && owner->detached && !hw->fatal)
		sc2_surface_owner_free(hw, owner);
}

void meson_sc2_surface_free(struct meson_sc2_hw *hw,
			   struct meson_sc2_surface *surface)
{
	struct sc2_surface_owner *owner;

	if (!hw || !surface || !surface->cpu)
		return;
	mutex_lock(&hw->lock);
	list_for_each_entry(owner, &hw->surfaces, list) {
		if (owner->surface.cpu != surface->cpu)
			continue;
		owner->detached = true;
		if (!owner->references && !hw->fatal)
			sc2_surface_owner_free(hw, owner);
		memset(surface, 0, sizeof(*surface));
		break;
	}
	mutex_unlock(&hw->lock);
}

void meson_sc2_hw_destroy(struct meson_sc2_hw *hw)
{
	if (!hw)
		return;
	mutex_lock(&hw->lock);
	if (hw->fatal) {
		mutex_unlock(&hw->lock);
		return;
	}
	/* Frontend must destroy contexts/surfaces after draining its worker. */
	if (WARN_ON(!list_empty(&hw->contexts) || !list_empty(&hw->surfaces))) {
		mutex_unlock(&hw->lock);
		return;
	}
	sc2_power_off(hw);
	free_irq(hw->irq, hw);
	pm_runtime_disable(hw->dev);
	clk_bulk_put(ARRAY_SIZE(hw->clocks), hw->clocks);
	iounmap(hw->bridge);
	iounmap(hw->dmc);
	iounmap(hw->dos);
	release_mem_region(hw->dos_resource.start,
			   resource_size(&hw->dos_resource));
	mutex_unlock(&hw->lock);
	put_device(hw->dev);
	kfree(hw);
}

static struct sc2_surface_owner *sc2_owner(struct meson_sc2_hw *hw,
					struct meson_sc2_surface *s)
{
	struct sc2_surface_owner *owner;

	if (!s)
		return NULL;
	list_for_each_entry(owner, &hw->surfaces, list)
		if (owner->surface.cpu == s->cpu &&
		    owner->surface.dma == s->dma && owner->surface.size == s->size &&
		    owner->surface.header_size == s->header_size &&
		    owner->surface.header_alloc_size == s->header_alloc_size &&
		    owner->surface.body_offset == s->body_offset &&
		    owner->surface.body_size == s->body_size &&
		    owner->surface.mv_offset == s->mv_offset &&
		    owner->surface.mv_size == s->mv_size &&
		    owner->surface.mv_dma == s->mv_dma &&
		    owner->surface.capture_owned == s->capture_owned)
			return owner;
	return NULL;
}

static int sc2_prepare(struct meson_sc2_hw *hw)
{
	u32 value;
	int ret;

	if (sc2_read(hw, SC2_DOS_MEM_PD_HEVC) ||
	    sc2_read(hw, HEVC_MPSR) || sc2_read(hw, HEVC_CPSR) ||
	    (sc2_read(hw, HEVC_IMEM_DMA_CTRL) & BIT(15)) ||
	    (sc2_read(hw, SC2_LMEM_DMA_CTRL) & BIT(15)) ||
	    (readl(hw->bridge) & SC2_BRIDGE_PORTS) != SC2_BRIDGE_PORTS)
		return -EBUSY;
	ret = sc2_disable_dmc(hw);
	if (ret)
		return ret;
	sc2_write(hw, SC2_SAO_MMU_RESET, sc2_read(hw, SC2_SAO_MMU_RESET) | 1);
	sc2_write(hw, SC2_DOS_SW_RESET3, SC2_CORE_RESETS);
	sc2_write(hw, SC2_DOS_SW_RESET3, 0);
	ret = readl_poll_timeout(hw->dos + SC2_WRRSP_LMEM, value,
				 !(value & 0xfff), 10, SC2_TIMEOUT_US);
	sc2_write(hw, SC2_SAO_MMU_RESET, sc2_read(hw, SC2_SAO_MMU_RESET) & ~1);
	if (ret)
		return ret;
	writel(readl(hw->bridge) & ~SC2_BRIDGE_PORTS, hw->bridge);
	readl(hw->bridge);
	writel(readl(hw->bridge) | SC2_BRIDGE_PORTS, hw->bridge);
	readl(hw->bridge);
	writel(readl(hw->dmc) | SC2_DMC_PORTS, hw->dmc);
	readl(hw->dmc);
	return 0;
}

/* sequence-probe-v2 register order, backed by isolated context storage. */
static void sc2_program_parser(struct meson_sc2_hevc_context *ctx, size_t length)
{
	struct meson_sc2_hw *hw = ctx->hw;
	u32 base = lower_32_bits(ctx->work_dma);
	u32 input = lower_32_bits(ctx->input_dma);
	unsigned int i;

	sc2_write(hw, HEVCD_IPP_LINEBUFF_BASE, base + W_IPP);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_1, base + W_RPM);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_2, base + W_RPS);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_3, base + W_VPS);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_4, base + W_SPS);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_5, base + W_PPS);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_6, base + W_SAO_UP);
	sc2_write(hw, HEVC_ASSIST_MMU_MAP_ADDR, base + W_MAP);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_D, base + W_SCALE);
	sc2_write(hw, HEVC_DBLK_CFG3, 0x4010);
	sc2_write(hw, HEVC_DBLK_CFG4, base + W_DBLK_PARA);
	sc2_write(hw, HEVC_DBLK_CFG5, base + W_DBLK_DATA);
	sc2_write(hw, HEVC_DBLK_CFGE, base + W_DBLK_DATA2);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_F, base + W_LMEM);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_L, base + W_AUX);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_M, 0);
	sc2_write(hw, HEVC_STREAM_CONTROL, 0x0eb14070);
	sc2_write(hw, HEVC_STREAM_START_ADDR, input);
	sc2_write(hw, HEVC_STREAM_END_ADDR, input + SC2_INPUT_SIZE);
	sc2_write(hw, HEVC_STREAM_RD_PTR, input);
	sc2_write(hw, HEVC_STREAM_WR_PTR,
		  input + round_down(length + SC2_INPUT_PADDING, 64));
	sc2_write(hw, SC2_DOS_SW_RESET3, BIT(14));
	sc2_write(hw, HEVC_CABAC_CONTROL, sc2_read(hw, HEVC_CABAC_CONTROL) & ~1);
	sc2_write(hw, HEVC_PARSER_CORE_CONTROL,
		  sc2_read(hw, HEVC_PARSER_CORE_CONTROL) & ~1);
	sc2_write(hw, HEVC_SHIFT_STARTCODE, 0x00000100);
	sc2_write(hw, HEVC_SHIFT_EMULATECODE, 0x00000300);
	sc2_write(hw, HEVC_PARSER_INT_CONTROL,
		  (sc2_read(hw, HEVC_PARSER_INT_CONTROL) & 0x03ffffff) |
		  (3 << 29) | (2 << 26) | BIT(24) | BIT(22) | BIT(7) | BIT(4) | BIT(0));
	sc2_write(hw, HEVC_SHIFT_STATUS, sc2_read(hw, HEVC_SHIFT_STATUS) | 3);
	sc2_write(hw, HEVC_SHIFT_CONTROL, (3 << 6) | (2 << 4) | (2 << 1) | 1);
	sc2_write(hw, HEVC_CABAC_CONTROL, 1);
	sc2_write(hw, HEVC_PARSER_CORE_CONTROL, 1);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_0, 0);
	sc2_write(hw, HEVC_IQIT_SCALELUT_WR_ADDR, 0);
	for (i = 0; i < 1024; i++)
		sc2_write(hw, HEVC_IQIT_SCALELUT_DATA, 0);
	sc2_write(hw, HEVC_PARSER_CMD_WRITE, BIT(16));
	for (i = 0; i < ARRAY_SIZE(sc2_parser_commands); i++)
		sc2_write(hw, HEVC_PARSER_CMD_WRITE, sc2_parser_commands[i]);
	sc2_write(hw, HEVC_PARSER_CMD_SKIP_0, 0x0000090b);
	sc2_write(hw, HEVC_PARSER_CMD_SKIP_1, 0x1b14140f);
	sc2_write(hw, HEVC_PARSER_CMD_SKIP_2, 0x001b1910);
	sc2_write(hw, HEVC_PARSER_IF_CONTROL, BIT(5) | BIT(2) | BIT(0));
	sc2_write(hw, HEVCD_IPP_TOP_CNTL, 1);
	sc2_write(hw, HEVCD_IPP_TOP_CNTL, 2);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_E, 1);
	sc2_write(hw, SC2_MBOX_CLEAR, 1);
	sc2_write(hw, SC2_MBOX_MASK, 1);
	sc2_write(hw, HEVC_PSCALE_CTRL, 0);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_G, 0);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_I, 4);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_K, 0);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_J, 0x80000001);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_H, 0);
	sc2_write(hw, HEVC_STREAM_CONTROL, sc2_read(hw, HEVC_STREAM_CONTROL) | 1);
	sc2_write(hw, HEVC_STREAM_FIFO_CTL, sc2_read(hw, HEVC_STREAM_FIFO_CTL) | BIT(29));
	sc2_write(hw, HEVC_ASSIST_SCRATCH_0, SC2_FW_ACTION);
	sc2_write(hw, HEVC_SHIFT_BYTE_COUNT, 0);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_N, length);
}

struct sc2_tile {
	u32 x, y, width, height;
	unsigned int column, row;
};

static int sc2_tile_at(const struct v4l2_ctrl_hevc_pps *pps, u32 x, u32 y,
		       u32 columns, u32 rows, struct sc2_tile *tile)
{
	u32 start, end, n, i;
	bool enabled = pps->flags & V4L2_HEVC_PPS_FLAG_TILES_ENABLED;
	bool uniform = pps->flags & V4L2_HEVC_PPS_FLAG_UNIFORM_SPACING;

	memset(tile, 0, sizeof(*tile));
	for (n = 0; n < 2; n++) {
		u32 total = n ? rows : columns;
		u32 pos = n ? y : x;
		u32 count = enabled ? (n ? pps->num_tile_rows_minus1 :
					 pps->num_tile_columns_minus1) + 1 : 1;
		const u8 *sizes = n ? pps->row_height_minus1 : pps->column_width_minus1;

		start = 0;
		for (i = 0; i < count; i++) {
			end = uniform ? (i + 1) * total / count :
				(i + 1 == count ? total : start + sizes[i] + 1);
			if (end <= start || end > total)
				return -EINVAL;
			if (pos >= start && pos < end) {
				if (n) {
					tile->y = start;
					tile->height = end - start;
					tile->row = i;
				} else {
					tile->x = start;
					tile->width = end - start;
					tile->column = i;
				}
				break;
			}
			start = end;
		}
		if (i == count)
			return -EINVAL;
	}
	return 0;
}

static void sc2_program_surfaces(struct meson_sc2_hevc_context *ctx,
				 const struct meson_sc2_hevc_job *job)
{
	struct meson_sc2_hw *hw = ctx->hw;
	const struct meson_sc2_surface *s = job->destination;
	u32 work = lower_32_bits(ctx->work_dma);
	u32 pages = DIV_ROUND_UP(s->body_size, SZ_4K);
	__le32 *map = ctx->work + W_MAP;
	unsigned int i;

	/* Unused page-map entries alias owned body storage, never an unowned PFN. */
	for (i = 0; i < SC2_MAP_ENTRIES; i++)
		map[i] = cpu_to_le32((s->dma + s->body_offset +
				     (i < pages ? i * SZ_4K : 0)) >> 12);
	dma_wmb();
	sc2_write(hw, HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR, BIT(1) | BIT(2));
	/* Slot zero is the destination; native DPB index i occupies slot i+1. */
	for (i = 0; i < 24; i++) {
		const struct meson_sc2_surface *ref = s;

		if (i && i <= job->syntax->decode->num_active_dpb_entries)
			ref = job->references[i - 1];
		sc2_write(hw, HEVCD_MPP_ANC2AXI_TBL_DATA, ref->dma >> 5);
	}
	sc2_write(hw, HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR, 1);
	sc2_write(hw, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, 1);
	for (i = 0; i < 32; i++)
		sc2_write(hw, HEVCD_MPP_ANC_CANVAS_DATA_ADDR, 0);
	sc2_write(hw, HEVCD_MPP_DECOMP_CTL1, BIT(4));
	sc2_write(hw, HEVCD_MPP_DECOMP_CTL2, 0);
	sc2_write(hw, HEVC_CM_BODY_LENGTH, s->body_size);
	sc2_write(hw, HEVC_CM_HEADER_OFFSET, s->body_size);
	sc2_write(hw, HEVC_CM_HEADER_LENGTH, s->header_size);
	sc2_write(hw, HEVC_SAO_MMU_VH0_ADDR, work + W_MMU_VBH);
	sc2_write(hw, HEVC_SAO_MMU_VH1_ADDR, work + W_MMU_VBH + 0x2800);
	sc2_write(hw, SC2_SAO_CTRL9, sc2_read(hw, SC2_SAO_CTRL9) | 1);
	sc2_write(hw, HEVC_SAO_CTRL5,
		  (sc2_read(hw, HEVC_SAO_CTRL5) & ~((0xff << 16) | BIT(9))) |
		  BIT(10) | (s->depth == 8 ? BIT(9) : 0));
	sc2_write(hw, HEVCD_MCRCC_CTL1, 2);
	sc2_write(hw, HEVCD_MCRCC_CTL1, 0);
}

static int sc2_program_slice(struct meson_sc2_hevc_context *ctx,
			     const struct meson_sc2_hevc_job *job,
			     unsigned int index, u32 *previous_tile,
			     u32 *independent_address)
{
	struct meson_sc2_hw *hw = ctx->hw;
	const struct meson_hevc_syntax_request *syntax = job->syntax;
	const struct v4l2_ctrl_hevc_sps *sps = syntax->sps;
	const struct v4l2_ctrl_hevc_pps *pps = syntax->pps;
	const struct v4l2_ctrl_hevc_decode_params *decode = syntax->decode;
	const struct v4l2_ctrl_hevc_slice_params *p = syntax->slices[index].params;
	const struct meson_sc2_surface *dst = job->destination, *col = dst;
	u32 log2 = sps->log2_min_luma_coding_block_size_minus3 + 3 +
		sps->log2_diff_max_min_luma_coding_block_size;
	u32 width = sps->pic_width_in_luma_samples, height = sps->pic_height_in_luma_samples;
	u32 columns = DIV_ROUND_UP(width, 1U << log2);
	u32 rows = DIV_ROUND_UP(height, 1U << log2);
	u32 work = lower_32_bits(ctx->work_dma);
	u32 mv = lower_32_bits(dst->mv_dma);
	dma_addr_t dw = dst->capture_owned ? ctx->dw8_dma : hw->output_dma;
	u32 unit = log2 == 6 ? 0x200 : log2 == 5 ? 0x80 : 0x20;
	u32 counts[2] = { p->slice_type == V4L2_HEVC_SLICE_TYPE_I ? 0 :
		p->num_ref_idx_l0_active_minus1 + 1,
		p->slice_type == V4L2_HEVC_SLICE_TYPE_B ? p->num_ref_idx_l1_active_minus1 + 1 : 0 };
	u32 lt = 0, value, filter, sao, tile_id, col_mv;
	u32 sao_unit = (log2 == 4 ? 9 : log2 == 5 ? 14 : 24) << 4;
	s32 pocs[2][16] = {};
	bool low_delay = p->slice_type != V4L2_HEVC_SLICE_TYPE_I;
	bool dependent = p->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_DEPENDENT_SLICE_SEGMENT;
	bool from_l0 = p->slice_type != V4L2_HEVC_SLICE_TYPE_B ||
		(p->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_COLLOCATED_FROM_L0);
	bool disabled = p->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_DEBLOCKING_FILTER_DISABLED;
	bool across = p->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED;
	bool col_long = false;
	struct sc2_tile tile;
	unsigned int list, i;
	int ret;

	ret = sc2_tile_at(pps, p->slice_segment_addr % columns,
			  p->slice_segment_addr / columns, columns, rows, &tile);
	if (ret)
		return ret;
	tile_id = tile.row * 12 + tile.column;
	if (!dependent)
		*independent_address = p->slice_segment_addr;
	for (list = 0; list < 2; list++) {
		const u8 *indices = list ? p->ref_idx_l1 : p->ref_idx_l0;

		sc2_write(hw, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, (list << 12) | 1);
		for (i = 0; i < counts[list]; i++) {
			u32 dpb = indices[i], slot = dpb + 1;

			if (dpb >= decode->num_active_dpb_entries)
				return -EINVAL;
			pocs[list][i] = decode->dpb[dpb].pic_order_cnt_val;
			if (pocs[list][i] > decode->pic_order_cnt_val)
				low_delay = false;
			if (decode->dpb[dpb].flags & V4L2_HEVC_DPB_ENTRY_LONG_TERM_REFERENCE)
				lt |= BIT(i + list * 16);
			sc2_write(hw, HEVCD_MPP_ANC_CANVAS_DATA_ADDR, slot * 0x010101);
			if (list == !from_l0 && i == p->collocated_ref_idx) {
				col = job->references[dpb];
				col_long = decode->dpb[dpb].flags & V4L2_HEVC_DPB_ENTRY_LONG_TERM_REFERENCE;
			}
		}
	}
	col_mv = lower_32_bits(col->mv_dma);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_9, sao_unit);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_A, work + W_SAO_ABOVE);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_B, (sao_unit + 32) * rows);
	sc2_write(hw, HEVC_ASSIST_SCRATCH_C, work + W_SAO_VERTICAL);
	sc2_write(hw, HEVC_MPRED_MV_WR_START_ADDR, mv);
	sc2_write(hw, HEVC_MPRED_MV_RD_START_ADDR, col_mv);
	sc2_write(hw, HEVC_MPRED_MV_WR_ROW_JUMP, (columns - tile.width) * unit);
	sc2_write(hw, HEVC_MPRED_MV_RD_ROW_JUMP, (columns - tile.width) * unit);
	value = p->slice_type | (!index << 2) | ((tile_id != *previous_tile) << 3) |
		(dependent << 4) |
		(!!(p->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_TEMPORAL_MVP_ENABLED) << 5) |
		(low_delay << 6) | (from_l0 << 7) | BIT(9) | BIT(10) |
		((p->slice_type != V4L2_HEVC_SLICE_TYPE_I) << 11) | BIT(13) |
		(col_long << 15) | (log2 << 16) | (3 << 20) |
		((pps->log2_parallel_merge_level_minus2 + 2) << 24);
	sc2_write(hw, HEVC_MPRED_CTRL0, value);
	sc2_write(hw, HEVC_MPRED_CTRL1, (5 - p->five_minus_max_num_merge_cand) |
		  (2 << 4) | (3 << 8) | (5 << 12) | (36 << 16));
	sc2_write(hw, HEVC_MPRED_PIC_SIZE, width | (height << 16));
	sc2_write(hw, HEVC_MPRED_PIC_SIZE_LCU, (columns - 1) | ((rows - 1) << 16));
	sc2_write(hw, HEVC_MPRED_TILE_START, tile.x | (tile.y << 16));
	sc2_write(hw, HEVC_MPRED_TILE_SIZE_LCU, tile.width | (tile.height << 16));
	sc2_write(hw, HEVC_MPRED_REF_NUM, counts[0] | (counts[1] << 8));
	sc2_write(hw, SC2_MPRED_LT_REF, lt);
	sc2_write(hw, HEVC_MPRED_REF_EN_L0, (1U << counts[0]) - 1);
	sc2_write(hw, HEVC_MPRED_REF_EN_L1, (1U << counts[1]) - 1);
	sc2_write(hw, HEVC_MPRED_CUR_POC, decode->pic_order_cnt_val);
	sc2_write(hw, HEVC_MPRED_COL_POC, col == dst ? decode->pic_order_cnt_val : col->poc);
	for (i = 0; i < 16; i++) {
		sc2_write(hw, HEVC_MPRED_L0_REF00_POC + i * 4, pocs[0][i]);
		sc2_write(hw, HEVC_MPRED_L1_REF00_POC + i * 4, pocs[1][i]);
	}
	if (!index) {
		sc2_write(hw, HEVC_MPRED_ABV_START_ADDR, work + W_MPRED_ABOVE);
		sc2_write(hw, HEVC_MPRED_MV_WPTR, mv);
		sc2_write(hw, HEVC_MPRED_MV_RPTR, col_mv);
	} else if (!dependent) {
		sc2_write(hw, HEVC_MPRED_MV_RPTR, col_mv + *independent_address * unit);
	}
	sc2_write(hw, HEVC_MPRED_MV_RD_END_ADDR, col_mv + col->mv_size);
	*previous_tile = tile_id;

	sc2_write(hw, HEVC_SAO_CTRL0, (sc2_read(hw, HEVC_SAO_CTRL0) & ~0xf) | log2);
	sc2_write(hw, HEVC_SAO_PIC_SIZE, width | (height << 16));
	sc2_write(hw, HEVC_SAO_PIC_SIZE_LCU, (columns - 1) | ((rows - 1) << 16));
	if (!index) {
		sc2_write(hw, HEVC_CM_BODY_START_ADDR, dst->dma + dst->body_offset);
		sc2_write(hw, HEVC_CM_HEADER_START_ADDR, dst->dma);
		sc2_write(hw, HEVC_SAO_Y_START_ADDR, dw);
		sc2_write(hw, HEVC_SAO_C_START_ADDR, dw + job->output_uv_offset);
		sc2_write(hw, HEVC_SAO_Y_LENGTH, job->output_uv_offset);
		sc2_write(hw, HEVC_SAO_C_LENGTH, job->output_uv_offset / 2);
		sc2_write(hw, HEVC_SAO_Y_WPTR, dw);
		sc2_write(hw, HEVC_SAO_C_WPTR, dw + job->output_uv_offset);
		sc2_write(hw, HEVC_DBLK_CFGB, (0x57 << 8) | (width >= 1280 ? BIT(4) : 0));
		sc2_write(hw, HEVC_DBLK_CFG2, width | (height << 16));
		value = (log2 == 6 ? 0 : log2 == 5 ? 1 : 2) |
			(width <= 64 ? BIT(20) : 0) |
			((pps->pps_cb_qp_offset & 0x1f) << 4) |
			((pps->pps_cr_qp_offset & 0x1f) << 9);
		if ((sps->flags & V4L2_HEVC_SPS_FLAG_PCM_ENABLED) &&
		    (sps->flags & V4L2_HEVC_SPS_FLAG_PCM_LOOP_FILTER_DISABLED))
			value |= BIT(3);
		sc2_write(hw, HEVC_DBLK_CFG1, value);
		/* Vendor word register 0x350c+4, not native byte offset+4. */
		sc2_write(hw, HEVC_DBLK_STS1 + 16, BIT(28));
	}
	value = sc2_read(hw, HEVC_SAO_CTRL1) & ~(0x3000 | 0xff0 | 3 | (3 << 14));
	/* SAO chroma byte-lane bit 8 selects UV (NV12) versus VU (NV21). */
	sc2_write(hw, HEVC_SAO_CTRL1, value |
		  (job->output_format == V4L2_PIX_FMT_NV21 ? 0xef0 : 0xff0) | (2 << 14));
	/* References stay compressed; do not apply CAPTURE's VU swap to IPP. */
	value = sc2_read(hw, HEVCD_IPP_AXIIF_CONFIG) & ~(0x30 | 0xf | BIT(12) | (3 << 8));
	sc2_write(hw, HEVCD_IPP_AXIIF_CONFIG, value | 0xf | (2 << 8));
	filter = disabled << 2;
	if (!disabled)
		filter |= ((p->slice_beta_offset_div2 & 0xf) << 3) |
			((p->slice_tc_offset_div2 & 0xf) << 7);
	sao = sc2_read(hw, HEVC_SAO_CTRL0) & ~0x300;
	if ((pps->flags & V4L2_HEVC_PPS_FLAG_TILES_ENABLED) &&
	    (pps->flags & V4L2_HEVC_PPS_FLAG_LOOP_FILTER_ACROSS_TILES_ENABLED)) {
		filter |= 1;
		sao |= BIT(8);
	}
	if (!(pps->flags & V4L2_HEVC_PPS_FLAG_PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED))
		across = false;
	else if (!(p->flags & (V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_SAO_LUMA |
			       V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_SAO_CHROMA)) && disabled)
		across = true;
	sc2_write(hw, HEVC_DBLK_CFG9, filter | (across << 1));
	sc2_write(hw, HEVC_SAO_CTRL0, sao | (across << 9));
	dma_wmb();
	sc2_write(hw, HEVC_ASSIST_SCRATCH_0, SC2_FW_DATA);
	sc2_write(hw, HEVC_MCPU_INTR_REQ, SC2_CPU_REQUEST);
	return 0;
}

static int sc2_validate_job(struct meson_sc2_hevc_context *ctx,
			    const struct meson_sc2_hevc_job *job)
{
	const struct meson_hevc_syntax_request *s;
	const struct v4l2_ctrl_hevc_sps *sps;
	struct sc2_surface_owner *owner;
	u32 width, height, log2, uv;
	unsigned int i;

	if (!job || !job->syntax || !job->destination || !job->output_buffer)
		return -EINVAL;
	s = job->syntax;
	if (!s->sps || !s->pps || !s->decode || !s->slices || !s->num_slices)
		return -ENODATA;
	sps = s->sps;
	width = sps->pic_width_in_luma_samples;
	height = sps->pic_height_in_luma_samples;
	log2 = sps->log2_min_luma_coding_block_size_minus3 + 3 +
		sps->log2_diff_max_min_luma_coding_block_size;
	if (!width || !height || width > SC2_MAX_WIDTH || height > SC2_MAX_HEIGHT ||
	    (sps->bit_depth_luma_minus8 != 0 && sps->bit_depth_luma_minus8 != 2) ||
	    sps->bit_depth_chroma_minus8 != sps->bit_depth_luma_minus8 ||
	    sps->chroma_format_idc != 1 || log2 < 4 || log2 > 6 ||
	    s->decode->num_active_dpb_entries > 16 ||
	    s->pps->log2_parallel_merge_level_minus2 > 5)
		return -EOPNOTSUPP;
	/* The authenticated firmware RPM has tile_width[12], tile_height[8]. */
	if ((s->pps->flags & V4L2_HEVC_PPS_FLAG_TILES_ENABLED) &&
	    (s->pps->num_tile_columns_minus1 >= 12 || s->pps->num_tile_rows_minus1 >= 8))
		return -EOPNOTSUPP;
	uv = job->output_stride * ALIGN(height, 64);
	if (job->output_stride != ALIGN(width, 64) || job->output_uv_offset != uv ||
	    job->output_buffer->size < job->output_size)
		return -EINVAL;
	if (job->output_format == V4L2_PIX_FMT_AMLOGIC_FBC8 ||
	    job->output_format == V4L2_PIX_FMT_AMLOGIC_FBC10) {
		if ((job->output_format == V4L2_PIX_FMT_AMLOGIC_FBC10) !=
		    (sps->bit_depth_luma_minus8 == 2) ||
		    !job->destination->capture_owned ||
		    job->output_size != meson_sc2_hevc_fbc_size(width, height) ||
		    !ctx->dw8 || ctx->dw8_size < uv + uv / 2)
			return -EINVAL;
	} else if (sps->bit_depth_luma_minus8 ||
		   (job->output_format != V4L2_PIX_FMT_NV12 &&
		    job->output_format != V4L2_PIX_FMT_NV21) ||
		   job->destination->capture_owned || job->output_size < uv + uv / 2) {
		return -EINVAL;
	}
	owner = sc2_owner(ctx->hw, job->destination);
	if (!owner || (owner->context && owner->context != ctx) ||
	    owner->surface.width != width || owner->surface.height != height ||
	    owner->surface.depth != sps->bit_depth_luma_minus8 + 8 ||
	    (owner->capture_buffer && owner->capture_buffer != job->output_buffer))
		return -EINVAL;
	for (i = 0; i < s->decode->num_active_dpb_entries; i++) {
		struct meson_sc2_surface *ref = job->references[i];
		struct sc2_surface_owner *ref_owner = sc2_owner(ctx->hw, ref);

		if (!ref_owner || ref_owner == owner || ref_owner->context != ctx ||
		    !ref_owner->surface.valid || !ref->valid ||
		    ref_owner->surface.poc != s->decode->dpb[i].pic_order_cnt_val ||
		    ref->poc != ref_owner->surface.poc ||
		    ref_owner->surface.width != width || ref_owner->surface.height != height ||
		    ref_owner->surface.depth != owner->surface.depth)
			return -EINVAL;
	}
	return 0;
}

static bool sc2_surface_guards(const struct meson_sc2_surface *s)
{
	u32 header_end = s->header_alloc_size;
	u32 body_end = s->body_offset + ALIGN(s->body_size, SZ_4K);
	u32 mv_end = s->mv_offset + s->mv_size;

	if (s->capture_owned)
		return !memchr_inv(s->cpu + header_end, SC2_GUARD_BYTE,
				   s->body_offset - header_end) &&
			!memchr_inv(s->cpu + body_end, SC2_GUARD_BYTE,
				    s->size - body_end);
	return !memchr_inv(s->cpu + header_end, SC2_GUARD_BYTE,
			   s->body_offset - header_end) &&
		!memchr_inv(s->cpu + body_end, SC2_GUARD_BYTE,
			    s->mv_offset - body_end) &&
		!memchr_inv(s->cpu + mv_end, SC2_GUARD_BYTE, s->size - mv_end);
}

/* Firmware RPM is checked only as a parser acknowledgment. Full POCs,
 * lists and all DMA addresses are installed from the native request above.
 */
static int sc2_check_header(struct meson_sc2_hevc_context *ctx,
			    const struct meson_sc2_hevc_job *job, unsigned int index)
{
	const struct v4l2_ctrl_hevc_slice_params *p = job->syntax->slices[index].params;
	const struct v4l2_ctrl_hevc_sps *sps = job->syntax->sps;
	const __le16 *raw = ctx->work + W_RPM;
	u16 rpm[76];
	unsigned int i;

	dma_rmb();
	for (i = 0; i < ARRAY_SIZE(rpm); i++)
		rpm[i] = le16_to_cpu(raw[(i & ~3) + 3 - (i & 3)]);
	if (rpm[23] != sps->pic_width_in_luma_samples ||
	    rpm[24] != sps->pic_height_in_luma_samples ||
	    rpm[72] != (sps->bit_depth_luma_minus8 | (sps->bit_depth_chroma_minus8 << 4)) ||
	    rpm[18] != p->slice_type || rpm[21] != p->slice_segment_addr ||
	    !!rpm[20] != !!(p->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_DEPENDENT_SLICE_SEGMENT) ||
	    !!rpm[65] != !index || rpm[67] != p->nal_unit_type ||
	    rpm[25] != sps->log2_min_luma_coding_block_size_minus3 ||
	    rpm[26] != sps->log2_diff_max_min_luma_coding_block_size)
		return -EPROTO;
	return 0;
}

static int sc2_run(struct meson_sc2_hevc_context *ctx,
		   const struct meson_sc2_hevc_job *job, size_t length)
{
	struct meson_sc2_hw *hw = ctx->hw;
	const struct v4l2_ctrl_hevc_sps *sps = job->syntax->sps;
	unsigned long deadline = jiffies + SC2_JOB_TIMEOUT, remaining;
	u32 log2 = sps->log2_min_luma_coding_block_size_minus3 + 3 +
		sps->log2_diff_max_min_luma_coding_block_size;
	u32 total = DIV_ROUND_UP(sps->pic_width_in_luma_samples, 1U << log2) *
		DIV_ROUND_UP(sps->pic_height_in_luma_samples, 1U << log2);
	u32 previous_tile = U32_MAX, independent_address = 0;
	u32 sequence, status;
	unsigned int headers = 0;
	int ret;

	sc2_program_parser(ctx, length);
	dma_wmb();
	sc2_write(hw, SC2_DOS_SW_RESET3, BIT(12) | BIT(11));
	sc2_write(hw, SC2_DOS_SW_RESET3, 0);
	sc2_read(hw, SC2_DOS_SW_RESET3);
	usleep_range(10, 20);
	sc2_write(hw, HEVC_MPSR, 1);
	for (;;) {
		if (atomic_read(&ctx->aborted))
			return -ECANCELED;
		if (time_after_eq(jiffies, deadline))
			return -ETIMEDOUT;
		sequence = smp_load_acquire(&hw->irq_sequence);
		status = sc2_read(hw, HEVC_ASSIST_SCRATCH_0);
		if (status == SC2_FW_HEADER) {
			if (headers >= job->syntax->num_slices)
				return -EPROTO;
			ret = sc2_check_header(ctx, job, headers);
			if (ret)
				return ret;
			if (!headers)
				sc2_program_surfaces(ctx, job);
			ret = sc2_program_slice(ctx, job, headers,
						&previous_tile, &independent_address);
			if (ret)
				return ret;
			headers++;
			continue;
		}
		if (status == SC2_FW_DONE) {
			if (headers != job->syntax->num_slices ||
			    (sc2_read(hw, HEVC_PARSER_LCU_START) & 0xffffff) != total - 1 ||
			    sc2_read(hw, HEVC_SHIFT_BYTE_COUNT) > length + SC2_INPUT_PADDING ||
			    (sc2_read(hw, SC2_SAO_MMU_STATUS) >> 16) >
				DIV_ROUND_UP(job->destination->body_size, SZ_4K))
				return -EIO;
			return 0;
		}
		if (status == 0x0b || (status >= 0x20 && status <= 0x24))
			return -EBADMSG;
		remaining = deadline - jiffies;
		if ((long)remaining <= 0)
			return -ETIMEDOUT;
		wait_event_timeout(hw->wait, atomic_read(&ctx->aborted) ||
			smp_load_acquire(&hw->irq_sequence) != sequence,
			remaining);
	}
}

static void sc2_unpin_output(struct meson_sc2_hw *hw)
{
	if (hw->output_mapping)
		dma_buf_unmap_attachment_unlocked(hw->output_attachment,
					hw->output_mapping, DMA_FROM_DEVICE);
	if (hw->output_attachment)
		dma_buf_detach(hw->output_buffer, hw->output_attachment);
	if (hw->output_buffer)
		dma_buf_put(hw->output_buffer);
	hw->output_mapping = NULL;
	hw->output_attachment = NULL;
	hw->output_buffer = NULL;
	hw->output_dma = 0;
}

static int sc2_pin_output(struct meson_sc2_hw *hw,
			  const struct meson_sc2_hevc_job *job)
{
	struct scatterlist *sg;
	dma_addr_t next;
	size_t mapped = 0;
	unsigned int i;
	int ret;

	get_dma_buf(job->output_buffer);
	hw->output_buffer = job->output_buffer;
	hw->output_attachment = dma_buf_attach(job->output_buffer, hw->dev);
	if (IS_ERR(hw->output_attachment)) {
		ret = PTR_ERR(hw->output_attachment);
		hw->output_attachment = NULL;
		goto fail;
	}
	hw->output_mapping = dma_buf_map_attachment_unlocked(hw->output_attachment,
								   DMA_FROM_DEVICE);
	if (IS_ERR(hw->output_mapping)) {
		ret = PTR_ERR(hw->output_mapping);
		hw->output_mapping = NULL;
		goto fail;
	}
	next = sg_dma_address(hw->output_mapping->sgl);
	hw->output_dma = next;
	ret = -EINVAL;
	if (!IS_ALIGNED(next, 64) || !sc2_dma_range(next, job->output_size))
		goto fail;
	if (job->destination->capture_owned) {
		if (next != job->destination->dma)
			goto fail;
		/* Meson scanout reads the physical PFNs embedded by DOS. */
		for_each_sgtable_sg(hw->output_mapping, sg, i) {
			if (sg_phys(sg) != next)
				goto fail;
			next += sg->length;
		}
		next = hw->output_dma;
	}
	for_each_sgtable_dma_sg(hw->output_mapping, sg, i) {
		if (sg_dma_address(sg) != next)
			goto fail;
		if (sg_dma_len(sg) >= job->output_size - mapped)
			return 0;
		mapped += sg_dma_len(sg);
		next += sg_dma_len(sg);
	}
fail:
	sc2_unpin_output(hw);
	return ret;
}

int meson_sc2_hevc_decode(struct meson_sc2_hevc_context *ctx,
			  const struct meson_sc2_hevc_job *job)
{
	struct meson_sc2_hw *hw;
	struct sc2_surface_owner *owner;
	struct meson_hevc_syntax_error error;
	size_t length = 0;
	s32 firmware_status;
	bool recovered;
	int ret, drain;

	if (!ctx)
		return -EINVAL;
	hw = ctx->hw;
	mutex_lock(&hw->lock);
	if (hw->fatal) {
		ret = -EIO;
		goto unlock;
	}
	if (atomic_read(&ctx->aborted)) {
		ret = -ECANCELED;
		goto unlock;
	}
	ret = sc2_validate_job(ctx, job);
	if (ret)
		goto unlock;
	owner = sc2_owner(hw, job->destination);
	owner->context = ctx;
	owner->surface.valid = false;
	job->destination->valid = false;
	ret = meson_hevc_request_syntax(job->syntax, ctx->input,
				SC2_INPUT_SIZE - SC2_INPUT_PADDING, &length, &error);
	if (ret) {
		dev_dbg(hw->dev, "HEVC syntax slice %zu: %s (%d)\n",
			error.slice, error.reason, ret);
		goto unlock;
	}
	memset(ctx->input + length, 0, SC2_INPUT_PADDING);
	memset(ctx->work, 0, SC2_WORK_SIZE);
	ret = sc2_pin_output(hw, job);
	if (ret)
		goto unlock;
	ret = sc2_power_on(hw);
	if (ret)
		goto unpin;
	ret = sc2_prepare(hw);
	/* Even a failed reset/load can have issued DMA; all exit paths below
	 * retain the external output and decoder-owned memory until idle proof. */
	if (!ret) {
		firmware_status = -EIO;
		ret = meson_sm_call(hw->firmware, SM_VIDEO_FW_LOAD,
				    &firmware_status, 0x11, 0, 0, 0, 0);
		if (!ret && firmware_status)
			ret = -EIO;
	}
	if (!ret && !atomic_read(&ctx->aborted))
		ret = sc2_run(ctx, job, length);
	else if (!ret)
		ret = -ECANCELED;
	drain = sc2_drain(hw, &recovered);
	if (drain) {
		sc2_quarantine(hw, drain);
		ret = -EIO;
		goto unlock;
	}
	/* Recovery proves DMA safety, not completion of the interrupted writes. */
	if (!ret && recovered)
		ret = -EIO;
	if (!ret && atomic_read(&ctx->aborted))
		ret = -ECANCELED;
	dma_rmb();
	if (!sc2_surface_guards(job->destination) ||
	    (owner->capture_buffer &&
	     (memchr_inv(owner->private_mv + owner->surface.mv_size,
			 SC2_GUARD_BYTE, SC2_GUARD_SIZE) ||
	      memchr_inv(ctx->dw8 + ctx->dw8_size, SC2_GUARD_BYTE, SC2_GUARD_SIZE)))) {
		dev_err(hw->dev, "HEVC compressed surface guard overwritten\n");
		ret = -EIO;
	}
	if (!ret) {
		job->destination->poc = job->syntax->decode->pic_order_cnt_val;
		job->destination->valid = true;
		owner->surface.poc = job->destination->poc;
		owner->surface.valid = true;
	}
	sc2_power_off(hw);
unpin:
	sc2_unpin_output(hw);
unlock:
	mutex_unlock(&hw->lock);
	return ret;
}

/* Shares the IRQ, MMIO mutex, DMA quarantine and power lifetime above. */
#include "meson_sc2_av1.h"
#include "meson_sc2_vp9.h"
#include "meson_sc2_avs2.h"
