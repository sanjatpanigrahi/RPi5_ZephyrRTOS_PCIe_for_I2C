/*
 * RP1 PCIe I2C (DW_apb_i2c) for Raspberry Pi 5 Zephyr rpi_5 (bcm2712)
 *
 * - Enumerates RP1 at BDF from DT (defaults 00:00.0)
 * - Uses BAR1 as RP1 peripheral window (auto-assigns if BAR1 is 0)
 * - Maps BAR1 with device_map()
 * - Selects I2C instance via DT property rp1-i2c-index (default = 1 for GPIO2/3 header)
 * - Configures RP1 GPIO2/3 pads + mux for I2C1 (FUNCSEL a3)
 *
 * NOTE: Polling-only bring-up (no interrupts yet).
 *
 * FIXES INCLUDED:
 * - Correct handling of repeated-start / multi-message transfers (i2c_write_read)
 *   by:
 *     * NOT waiting for STOP when msg->stop=0
 *     * Issuing RESTART on the first byte/command of the next message
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pcie/pcie.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <errno.h>

LOG_MODULE_REGISTER(rp1_pcie_i2c, LOG_LEVEL_INF);

#define DT_DRV_COMPAT brcm_rp1_pcie_i2c

/* --------------------------- RP1 address map --------------------------- */
#define RP1_PERIPH_BASE               0x40000000ULL
#define RP1_I2C_BASE(i)               (0x40070000ULL + ((uint64_t)(i) * 0x4000ULL))
#define RP1_I2C_OFFSET_IN_PERIPH(i)   (RP1_I2C_BASE(i) - RP1_PERIPH_BASE)

/* RP1 GPIO/pads blocks live inside BAR1 peripheral window */
#define RP1_IO_BANK0_BASE             0x400D0000ULL
#define RP1_PADS_BANK0_BASE           0x400F0000ULL
#define RP1_IO_BANK0_OFF              (RP1_IO_BANK0_BASE - RP1_PERIPH_BASE)
#define RP1_PADS_BANK0_OFF            (RP1_PADS_BANK0_BASE - RP1_PERIPH_BASE)

/* Header pins 3/5 are GPIO2/3 (SDA/SCL). On RP1 these are I2C1 on FUNCSEL=a3. */
#define RP1_HDR_SDA_GPIO              2
#define RP1_HDR_SCL_GPIO              3
#define RP1_FUNCSEL_A3                3U

/* IO_BANK0 GPIOx_CTRL: base + 0x004 + 8*x */
#define IO_BANK0_GPIO_CTRL_OFF(gpio)  (RP1_IO_BANK0_OFF + 0x004U + ((gpio) * 8U))

/* PADS_BANK0: VOLTAGE_SELECT at +0x000, GPIOx pad at +0x004 + 4*x */
#define PADS_VOLTAGE_SELECT_OFF       (RP1_PADS_BANK0_OFF + 0x000U)
#define PADS_GPIO_OFF(gpio)           (RP1_PADS_BANK0_OFF + 0x004U + ((gpio) * 4U))

/* PADS bits: OD[7], IE[6], DRIVE[5:4], PUE[3], PDE[2], SCHMITT[1], SLEWFAST[0] */
#define PADS_OD                       BIT(7)
#define PADS_IE                       BIT(6)
#define PADS_PUE                      BIT(3)
#define PADS_PDE                      BIT(2)
#define PADS_SCHMITT                  BIT(1)
#define PADS_SLEWFAST                 BIT(0)

/* -------------------------- PCI config offsets -------------------------- */
#define PCI_CFG_VENDOR_DEVICE         0x00U
#define PCI_CFG_CMD_STATUS            0x04U
#define PCI_CFG_BAR0                  0x10U

static inline uint32_t cfg_read32(pcie_bdf_t bdf, uint32_t byte_off)
{
	/* pcie_conf_read takes DWORD index */
	return pcie_conf_read(bdf, (unsigned int)(byte_off >> 2));
}

static inline void cfg_write32(pcie_bdf_t bdf, uint32_t byte_off, uint32_t val)
{
	/* pcie_conf_write returns void */
	pcie_conf_write(bdf, (unsigned int)(byte_off >> 2), val);
}

static inline uint32_t pci_bar_raw(pcie_bdf_t bdf, uint8_t bar_index)
{
	return cfg_read32(bdf, PCI_CFG_BAR0 + (bar_index * 4U));
}

static inline uint32_t pci_bar_bus_addr32(uint32_t bar_raw)
{
	return (bar_raw & ~0xFULL);
}

static inline uint16_t pci_cmd_read(pcie_bdf_t bdf)
{
	return (uint16_t)(cfg_read32(bdf, PCI_CFG_CMD_STATUS) & 0xFFFFU);
}

static inline void pci_cmd_write(pcie_bdf_t bdf, uint16_t cmd)
{
	uint32_t cs = cfg_read32(bdf, PCI_CFG_CMD_STATUS);
	cfg_write32(bdf, PCI_CFG_CMD_STATUS, (cs & 0xFFFF0000U) | cmd);
}

#define PCI_CMD_MEM                   BIT(1)

/* ------------------------- DW_apb_i2c registers ------------------------- */
#define IC_CON                        0x00
#define IC_TAR                        0x04
#define IC_DATA_CMD                   0x10
#define IC_SS_SCL_HCNT                0x14
#define IC_SS_SCL_LCNT                0x18
#define IC_FS_SCL_HCNT                0x1C
#define IC_FS_SCL_LCNT                0x20
#define IC_INTR_MASK                  0x30
#define IC_RAW_INTR_STAT              0x34
#define IC_RX_TL                      0x38
#define IC_TX_TL                      0x3C
#define IC_CLR_INTR                   0x40
#define IC_CLR_TX_ABRT                0x54
#define IC_CLR_STOP_DET               0x60
#define IC_ENABLE                     0x6C
#define IC_STATUS                     0x70
#define IC_TXFLR                      0x74
#define IC_RXFLR                      0x78
#define IC_TX_ABRT_SOURCE             0x80
#define IC_ENABLE_STATUS              0x9C

#define IC_CON_MASTER_MODE            BIT(0)
#define IC_CON_SPEED_SHIFT            1
#define IC_CON_SPEED_SS               (0x1U << IC_CON_SPEED_SHIFT)
#define IC_CON_SPEED_FS               (0x2U << IC_CON_SPEED_SHIFT)
#define IC_CON_RESTART_EN             BIT(5)
#define IC_CON_SLAVE_DISABLE          BIT(6)

/* IC_DATA_CMD bits */
#define IC_DATA_CMD_CMD_READ          BIT(8)
#define IC_DATA_CMD_STOP              BIT(9)
#define IC_DATA_CMD_RESTART           BIT(10)

#define INTR_TX_ABRT                  BIT(6)
#define INTR_STOP_DET                 BIT(9)

/* IC_STATUS bits (subset) */
#define IC_STATUS_TFE                 BIT(2)  /* TX FIFO empty */
#define IC_STATUS_ACTIVITY            BIT(0)  /* activity (not always reliable, but useful) */

/* ------------------------------ MMIO helpers ------------------------------ */
static inline uint32_t reg_read32(uintptr_t base, uint32_t off)
{
	return sys_read32(base + off);
}

static inline void reg_write32(uintptr_t base, uint32_t off, uint32_t val)
{
	sys_write32(val, base + off);
}

/* ------------------------------ Driver data ------------------------------ */
struct rp1_pcie_i2c_config {
	uint8_t bus;
	uint8_t dev;
	uint8_t fun;
	uint8_t bar;              /* expect 1 */
	uint8_t rp1_i2c_index;    /* default 1 for header GPIO2/3 */
	uint32_t bitrate;
};

struct rp1_pcie_i2c_data {
	mm_reg_t bar_virt_storage;
	uintptr_t bar_virt;
	uint64_t bar_phys;
	uint32_t bar_size;

	uintptr_t regs;           /* selected I2C regs */
	struct k_mutex lock;
};

/* -------------------- PCI BAR sizing (32-bit mem BAR) -------------------- */
static int pci_size_mem32_bar(pcie_bdf_t bdf, uint8_t bar, uint32_t orig_lo,
			      uint32_t *size_out, uint32_t *mask_out)
{
	/* IO BAR? */
	if (orig_lo & 0x1U) {
		return -ENOTSUP;
	}

	const uint32_t off = PCI_CFG_BAR0 + (bar * 4U);

	/* Probe by writing all-ones */
	cfg_write32(bdf, off, 0xFFFFFFFFu);
	uint32_t mask = cfg_read32(bdf, off);
	/* Restore */
	cfg_write32(bdf, off, orig_lo);

	uint32_t mask_addr = (mask & ~0xFULL);
	if (mask_addr == 0U || mask_addr == 0xFFFFFFFFu) {
		return -EINVAL;
	}

	uint32_t size = (~mask_addr) + 1U;
	if ((size & (size - 1U)) != 0U) {
		return -EINVAL;
	}

	*size_out = size;
	if (mask_out) {
		*mask_out = mask_addr;
	}
	return 0;
}

/* ---------------------- RP1 GPIO2/3 pin configuration --------------------- */
static void rp1_configure_header_i2c_pins(struct rp1_pcie_i2c_data *d)
{
	/* 3.3V select (bit0=0) */
	uint32_t vs = reg_read32(d->bar_virt, (uint32_t)PADS_VOLTAGE_SELECT_OFF);
	LOG_INF("PADS VOLTAGE_SELECT before=0x%08x", vs);
	vs &= ~BIT(0);
	reg_write32(d->bar_virt, (uint32_t)PADS_VOLTAGE_SELECT_OFF, vs);
	LOG_INF("PADS VOLTAGE_SELECT after =0x%08x",
		reg_read32(d->bar_virt, (uint32_t)PADS_VOLTAGE_SELECT_OFF));

	/* Pads: input enable, pull-up enable, schmitt enable, slew limited, OD off */
	const int gpios[2] = { RP1_HDR_SDA_GPIO, RP1_HDR_SCL_GPIO };
	for (int i = 0; i < 2; i++) {
		int gpio = gpios[i];
		uint32_t off = (uint32_t)PADS_GPIO_OFF(gpio);
		uint32_t p = reg_read32(d->bar_virt, off);
		LOG_INF("PADS GPIO%d before=0x%08x", gpio, p);

		p &= ~PADS_OD;        /* OD off (I2C peripheral drives open-drain) */
		p |= PADS_IE;
		p |= PADS_PUE;
		p &= ~PADS_PDE;
		p |= PADS_SCHMITT;
		p &= ~PADS_SLEWFAST;

		reg_write32(d->bar_virt, off, p);
		LOG_INF("PADS GPIO%d after =0x%08x", gpio, reg_read32(d->bar_virt, off));
	}

	/* IO_BANK0 mux: FUNCSEL=a3, overrides cleared to 0 */
	for (int i = 0; i < 2; i++) {
		int gpio = gpios[i];
		uint32_t off = (uint32_t)IO_BANK0_GPIO_CTRL_OFF(gpio);
		uint32_t c = reg_read32(d->bar_virt, off);
		LOG_INF("GPIO%d_CTRL before=0x%08x", gpio, c);

		/* FUNCSEL [4:0] */
		c &= ~0x1FU;
		/* OUTOVER [13:12], OEOVER [15:14], INOVER [17:16] -> 0 */
		c &= ~((3U << 12) | (3U << 14) | (3U << 16));

		c |= RP1_FUNCSEL_A3;

		reg_write32(d->bar_virt, off, c);
		LOG_INF("GPIO%d_CTRL after =0x%08x", gpio, reg_read32(d->bar_virt, off));
	}

	LOG_INF("Pinmux set: GPIO2/3 -> FUNCSEL a3 (I2C1 on header pins 3/5)");
}

/* ------------------------------ DW I2C init ------------------------------ */
static int rp1_dw_disable(uintptr_t regs)
{
	reg_write32(regs, IC_ENABLE, 0);

	int64_t start = k_uptime_get();
	while (reg_read32(regs, IC_ENABLE_STATUS) & 0x1U) {
		if (k_uptime_get() - start > 50) {
			LOG_WRN("HW disable timeout: ENABLE_STATUS=0x%08x",
				reg_read32(regs, IC_ENABLE_STATUS));
			return -ETIMEDOUT;
		}
		k_busy_wait(50);
	}
	return 0;
}

static void rp1_dw_enable(uintptr_t regs)
{
	reg_write32(regs, IC_ENABLE, 1);
	(void)reg_read32(regs, IC_ENABLE_STATUS);
}

static void rp1_dw_set_counts(uintptr_t regs, uint32_t speed_hz)
{
	/* conservative */
	if (speed_hz <= 100000U) {
		reg_write32(regs, IC_SS_SCL_HCNT, 0x01F0);
		reg_write32(regs, IC_SS_SCL_LCNT, 0x0220);
	} else {
		reg_write32(regs, IC_FS_SCL_HCNT, 0x0070);
		reg_write32(regs, IC_FS_SCL_LCNT, 0x00D0);
	}
}

static int rp1_dw_hw_init(uintptr_t regs, uint32_t bitrate_hz)
{
	(void)rp1_dw_disable(regs);

	reg_write32(regs, IC_INTR_MASK, 0);
	reg_write32(regs, IC_RX_TL, 0);
	reg_write32(regs, IC_TX_TL, 0);

	rp1_dw_set_counts(regs, bitrate_hz);

	uint32_t con = IC_CON_MASTER_MODE |
		       IC_CON_RESTART_EN |          /* MUST be set for repeated-start */
		       IC_CON_SLAVE_DISABLE |
		       ((bitrate_hz <= 100000U) ? IC_CON_SPEED_SS : IC_CON_SPEED_FS);
	reg_write32(regs, IC_CON, con);

	(void)reg_read32(regs, IC_CLR_INTR);

	rp1_dw_enable(regs);

	LOG_INF("HW init done: CON=0x%08x ENABLE=0x%08x EN_STAT=0x%08x STATUS=0x%08x",
		reg_read32(regs, IC_CON),
		reg_read32(regs, IC_ENABLE),
		reg_read32(regs, IC_ENABLE_STATUS),
		reg_read32(regs, IC_STATUS));
	return 0;
}

/* --------------------------- Wait helpers (polling) --------------------------- */

static int rp1_dw_wait_stop_or_abort(uintptr_t regs, int32_t timeout_ms)
{
	int64_t deadline = k_uptime_get() + timeout_ms;

	while (k_uptime_get() < deadline) {
		uint32_t raw = reg_read32(regs, IC_RAW_INTR_STAT);

		if (raw & INTR_TX_ABRT) {
			uint32_t src = reg_read32(regs, IC_TX_ABRT_SOURCE);
			(void)reg_read32(regs, IC_CLR_TX_ABRT);
			LOG_ERR("TX_ABRT (SRC=0x%08x) RAW=0x%08x STATUS=0x%08x",
				src, raw, reg_read32(regs, IC_STATUS));
			return -EIO;
		}

		if (raw & INTR_STOP_DET) {
			(void)reg_read32(regs, IC_CLR_STOP_DET);
			return 0;
		}

		k_busy_wait(50);
	}

	LOG_ERR("Timeout waiting STOP (RAW_INTR=0x%08x STATUS=0x%08x TXFLR=%u RXFLR=%u)",
		reg_read32(regs, IC_RAW_INTR_STAT),
		reg_read32(regs, IC_STATUS),
		reg_read32(regs, IC_TXFLR),
		reg_read32(regs, IC_RXFLR));
	return -ETIMEDOUT;
}

/* Used when stop=0: do NOT wait for STOP_DET. Just ensure TX FIFO empties and no abort. */
static int rp1_dw_wait_tx_empty_or_abort(uintptr_t regs, int32_t timeout_ms)
{
	int64_t deadline = k_uptime_get() + timeout_ms;

	while (k_uptime_get() < deadline) {
		uint32_t raw = reg_read32(regs, IC_RAW_INTR_STAT);
		if (raw & INTR_TX_ABRT) {
			uint32_t src = reg_read32(regs, IC_TX_ABRT_SOURCE);
			(void)reg_read32(regs, IC_CLR_TX_ABRT);
			LOG_ERR("TX_ABRT (SRC=0x%08x) RAW=0x%08x STATUS=0x%08x",
				src, raw, reg_read32(regs, IC_STATUS));
			return -EIO;
		}

		uint32_t st = reg_read32(regs, IC_STATUS);
		if (st & IC_STATUS_TFE) {
			return 0;
		}

		k_busy_wait(50);
	}

	LOG_ERR("Timeout waiting TX empty (RAW_INTR=0x%08x STATUS=0x%08x TXFLR=%u RXFLR=%u)",
		reg_read32(regs, IC_RAW_INTR_STAT),
		reg_read32(regs, IC_STATUS),
		reg_read32(regs, IC_TXFLR),
		reg_read32(regs, IC_RXFLR));
	return -ETIMEDOUT;
}

/* --------------------------- Transfer (polling) --------------------------- */
static int rp1_pcie_i2c_transfer(const struct device *dev,
				 struct i2c_msg *msgs,
				 uint8_t num_msgs,
				 uint16_t addr)
{
	struct rp1_pcie_i2c_data *d = dev->data;

	if (!msgs || num_msgs == 0) {
		return -EINVAL;
	}

	k_mutex_lock(&d->lock, K_FOREVER);

	LOG_INF("xfer: addr=0x%02x num_msgs=%u STATUS=0x%08x EN=0x%08x ENS=0x%08x",
		addr, num_msgs,
		reg_read32(d->regs, IC_STATUS),
		reg_read32(d->regs, IC_ENABLE),
		reg_read32(d->regs, IC_ENABLE_STATUS));

	/* Program target (disable around TAR) */
	(void)rp1_dw_disable(d->regs);
	reg_write32(d->regs, IC_TAR, (uint32_t)(addr & 0x7FU));
	(void)reg_read32(d->regs, IC_CLR_INTR);
	rp1_dw_enable(d->regs);

	for (uint8_t mi = 0; mi < num_msgs; mi++) {
		struct i2c_msg *m = &msgs[mi];

		bool is_read = (m->flags & I2C_MSG_READ) != 0U;

		/* STOP: use flag if present; but always STOP on final message for safety */
		bool stop = (m->flags & I2C_MSG_STOP) != 0U;
		if (mi == (num_msgs - 1U)) {
			stop = true;
		}

		/* RESTART: needed when previous message did not end with STOP.
		 * Zephyr i2c_write_read produces msg0 STOP=0 then msg1 with RESTART.
		 * We compute it from previous STOP and also honor I2C_MSG_RESTART if set.
		 */
		bool need_restart = false;
		if (mi > 0U) {
			bool prev_stop = ((msgs[mi - 1U].flags & I2C_MSG_STOP) != 0U);
			if (!prev_stop) {
				need_restart = true;
			}
		}
#ifdef I2C_MSG_RESTART
		if (m->flags & I2C_MSG_RESTART) {
			need_restart = true;
		}
#endif

		LOG_INF("  msg%u: %s len=%u stop=%u restart=%u",
			mi, is_read ? "READ" : "WRITE", m->len,
			stop ? 1 : 0, need_restart ? 1 : 0);

		if (m->len == 0U) {
			/* Nothing to send; still respect stop/restart sequencing.
			 * If stop requested, wait stop; else just continue.
			 */
			int rc = stop ? rp1_dw_wait_stop_or_abort(d->regs, 1000)
				      : rp1_dw_wait_tx_empty_or_abort(d->regs, 1000);
			if (rc) {
				k_mutex_unlock(&d->lock);
				return rc;
			}
			continue;
		}

		if (is_read) {
			/* Issue READ commands.
			 * RESTART is placed on the first read command if required.
			 * STOP is placed on the last read command if stop requested.
			 */
			for (uint32_t i = 0; i < m->len; i++) {
				uint32_t cmd = IC_DATA_CMD_CMD_READ;

				if (need_restart && (i == 0U)) {
					cmd |= IC_DATA_CMD_RESTART;
				}
				if (stop && (i == (m->len - 1U))) {
					cmd |= IC_DATA_CMD_STOP;
				}
				reg_write32(d->regs, IC_DATA_CMD, cmd);
			}

			/* Pull bytes */
			uint32_t got = 0;
			int64_t deadline = k_uptime_get() + 1000;
			while ((got < m->len) && (k_uptime_get() < deadline)) {
				uint32_t rxflr = reg_read32(d->regs, IC_RXFLR);
				while (rxflr && got < m->len) {
					m->buf[got++] = (uint8_t)reg_read32(d->regs, IC_DATA_CMD);
					rxflr--;
				}
				k_busy_wait(50);
			}
			if (got != m->len) {
				LOG_ERR("Timeout waiting RX data (got=%u/%u RXFLR=%u)",
					got, m->len, reg_read32(d->regs, IC_RXFLR));
				k_mutex_unlock(&d->lock);
				return -ETIMEDOUT;
			}

		} else {
			/* WRITE.
			 * RESTART is placed on the first byte if required.
			 * STOP is placed on the last byte if stop requested.
			 */
			for (uint32_t i = 0; i < m->len; i++) {
				uint32_t cmd = (uint32_t)m->buf[i];

				if (need_restart && (i == 0U)) {
					cmd |= IC_DATA_CMD_RESTART;
				}
				if (stop && (i == (m->len - 1U))) {
					cmd |= IC_DATA_CMD_STOP;
				}
				reg_write32(d->regs, IC_DATA_CMD, cmd);
			}
		}

		/* Wait end:
		 * - If STOP requested, wait STOP_DET.
		 * - If STOP not requested, DO NOT wait STOP_DET (would deadlock).
		 *   Just wait TX empty so the next message can proceed with RESTART.
		 */
		int rc = stop ? rp1_dw_wait_stop_or_abort(d->regs, 1000)
			      : rp1_dw_wait_tx_empty_or_abort(d->regs, 1000);
		if (rc) {
			k_mutex_unlock(&d->lock);
			return rc;
		}
	}

	k_mutex_unlock(&d->lock);
	return 0;
}

static int rp1_pcie_i2c_configure(const struct device *dev, uint32_t dev_config)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(dev_config);
	/* already configured in init; keep for API completeness */
	return 0;
}

static const struct i2c_driver_api rp1_pcie_i2c_api = {
	.configure = rp1_pcie_i2c_configure,
	.transfer  = rp1_pcie_i2c_transfer,
};

/* ----------------------------- PCIe init path ----------------------------- */

static int derive_bus_to_phys_bias(pcie_bdf_t bdf, uint64_t *bias_out)
{
	/* Use Zephyr helper for BAR0 phys+size */
	struct pcie_bar bar0;
	if (!pcie_get_mbar(bdf, 0, &bar0)) {
		LOG_ERR("pcie_get_mbar(BAR0) failed; cannot derive bias");
		return -EIO;
	}

	uint32_t bar0_raw = pci_bar_raw(bdf, 0);
	uint32_t bar0_bus = pci_bar_bus_addr32(bar0_raw);
	if (bar0_bus == 0) {
		LOG_ERR("BAR0 bus address is 0; cannot derive bias");
		return -EINVAL;
	}

	uint64_t bias = (uint64_t)bar0.phys_addr - (uint64_t)bar0_bus;

	LOG_INF("Derived bus->phys bias = 0x%llx (BAR0 phys=%p bus=0x%08x size=0x%x)",
		bias, (void *)bar0.phys_addr, bar0_bus, (uint32_t)bar0.size);

	*bias_out = bias;
	return 0;
}

static int assign_mem32_bar_if_zero(pcie_bdf_t bdf, uint8_t bar_index,
				   uint32_t bus_addr, uint32_t bar_size)
{
	uint32_t raw = pci_bar_raw(bdf, bar_index);
	uint32_t cur = pci_bar_bus_addr32(raw);

	if (cur != 0) {
		LOG_INF("BAR%u already assigned: raw=0x%08x bus=0x%08x", bar_index, raw, cur);
		return 0;
	}

	if ((bus_addr & (bar_size - 1U)) != 0U) {
		LOG_ERR("BAR%u assign addr 0x%08x not aligned to size 0x%x",
			bar_index, bus_addr, bar_size);
		return -EINVAL;
	}

	LOG_WRN("Assigning BAR%u: bus_addr=0x%08x size=0x%x (was raw=0x%08x)",
		bar_index, bus_addr, bar_size, raw);

	/* Enable MEM if needed */
	uint16_t cmd = pci_cmd_read(bdf);
	if (!(cmd & PCI_CMD_MEM)) {
		cmd |= PCI_CMD_MEM;
		pci_cmd_write(bdf, cmd);
		LOG_INF("Enabled PCI CMD.MEM (CMD=0x%04x)", cmd);
	}

	cfg_write32(bdf, PCI_CFG_BAR0 + (bar_index * 4U), bus_addr);

	uint32_t new_raw = pci_bar_raw(bdf, bar_index);
	uint32_t new_bus = pci_bar_bus_addr32(new_raw);

	LOG_INF("BAR%u after assign: raw=0x%08x bus=0x%08x", bar_index, new_raw, new_bus);
	return (new_bus == 0) ? -EIO : 0;
}

static int rp1_pcie_i2c_init(const struct device *dev)
{
	const struct rp1_pcie_i2c_config *cfg = dev->config;
	struct rp1_pcie_i2c_data *d = dev->data;

	k_mutex_init(&d->lock);

	pcie_bdf_t bdf = PCIE_BDF(cfg->bus, cfg->dev, cfg->fun);

	uint32_t vd = cfg_read32(bdf, PCI_CFG_VENDOR_DEVICE);
	uint16_t vendor = (uint16_t)(vd & 0xFFFFU);
	uint16_t device = (uint16_t)((vd >> 16) & 0xFFFFU);

	LOG_INF("INIT: BDF=%02x:%02x.%x vendor=%04x device=%04x bar=%u i2c_index=%u",
		cfg->bus, cfg->dev, cfg->fun, vendor, device, cfg->bar, cfg->rp1_i2c_index);

	LOG_INF("PCI CMDSTAT=0x%08x", cfg_read32(bdf, PCI_CFG_CMD_STATUS));

	for (int i = 0; i < 6; i++) {
		LOG_INF("PCI BAR%d raw=0x%08x", i, pci_bar_raw(bdf, i));
	}

	/* Probe BAR size */
	uint32_t bar_raw = pci_bar_raw(bdf, cfg->bar);
	uint32_t want_size = 0, mask = 0;

	LOG_INF("BAR%u orig_lo=0x%08x (mem, 32-bit)", cfg->bar, bar_raw);

	int rc = pci_size_mem32_bar(bdf, cfg->bar, bar_raw, &want_size, &mask);
	if (rc) {
		LOG_ERR("Failed to size BAR%u: %d", cfg->bar, rc);
		return rc;
	}

	LOG_INF("BAR%u sized (32b): addr=0x%x size=0x%x mask_lo=0x%08x",
		cfg->bar, pci_bar_bus_addr32(bar_raw), want_size, mask);

	/* Compute bus->phys bias using BAR0 */
	uint64_t bias = 0;
	rc = derive_bus_to_phys_bias(bdf, &bias);
	if (rc) {
		return rc;
	}

	/* Assign BAR if unassigned (bus addr 0) */
	uint32_t bar_bus = pci_bar_bus_addr32(bar_raw);
	if (bar_bus == 0) {
		/* Place BAR1 after BAR0 window, aligned to BAR size */
		uint32_t bar0_raw = pci_bar_raw(bdf, 0);
		uint32_t bar0_bus = pci_bar_bus_addr32(bar0_raw);
		uint32_t bar0_size = 0, bar0_mask = 0;

		rc = pci_size_mem32_bar(bdf, 0, bar0_raw, &bar0_size, &bar0_mask);
		if (rc) {
			LOG_ERR("Failed to size BAR0 for placement: %d", rc);
			return rc;
		}

		uint32_t candidate = ROUND_UP(bar0_bus + bar0_size, want_size);

		LOG_WRN("BAR%u is 0; assigning candidate bus addr 0x%08x (align=0x%x)",
			cfg->bar, candidate, want_size);

		rc = assign_mem32_bar_if_zero(bdf, cfg->bar, candidate, want_size);
		if (rc) {
			LOG_ERR("Failed to assign BAR%u: %d", cfg->bar, rc);
			return rc;
		}

		bar_raw = pci_bar_raw(bdf, cfg->bar);
		bar_bus = pci_bar_bus_addr32(bar_raw);
	}

	d->bar_phys = bias + (uint64_t)bar_bus;
	d->bar_size = want_size;

	LOG_INF("BAR%u final: bus=0x%08x phys=0x%llx size=0x%x",
		cfg->bar, bar_bus, d->bar_phys, d->bar_size);

	device_map(&d->bar_virt_storage, (uintptr_t)d->bar_phys, d->bar_size, K_MEM_CACHE_NONE);
	d->bar_virt = (uintptr_t)d->bar_virt_storage;

	LOG_INF("Mapped BAR%u phys=0x%llx size=0x%x -> virt=%p",
		cfg->bar, d->bar_phys, d->bar_size, (void *)d->bar_virt);

	/* Pick I2C instance and compute regs VA */
	uint64_t off = RP1_I2C_OFFSET_IN_PERIPH(cfg->rp1_i2c_index);
	if (off + 0x1000ULL > (uint64_t)d->bar_size) {
		LOG_ERR("I2C%u offset 0x%llx out of BAR%u range size=0x%x",
			cfg->rp1_i2c_index, off, cfg->bar, d->bar_size);
		return -ERANGE;
	}

	d->regs = d->bar_virt + (uintptr_t)off;

	LOG_INF("I2C%u regs @ BAR%u+0x%llx => %p",
		cfg->rp1_i2c_index, cfg->bar, off, (void *)d->regs);

	/* Configure header GPIO2/3 for I2C1 only if using I2C1 */
	if (cfg->rp1_i2c_index == 1) {
		rp1_configure_header_i2c_pins(d);
	} else {
		LOG_WRN("Not configuring GPIO2/3 mux because rp1-i2c-index=%u (expected 1 for header)",
			cfg->rp1_i2c_index);
	}

	LOG_INF("Regs pre-init: CON=0x%08x EN=0x%08x ENS=0x%08x STATUS=0x%08x",
		reg_read32(d->regs, IC_CON),
		reg_read32(d->regs, IC_ENABLE),
		reg_read32(d->regs, IC_ENABLE_STATUS),
		reg_read32(d->regs, IC_STATUS));

	rc = rp1_dw_hw_init(d->regs, cfg->bitrate);
	if (rc) {
		LOG_ERR("HW init failed: %d", rc);
		return rc;
	}

	LOG_INF("INIT OK: bitrate=%u", cfg->bitrate);
	return 0;
}

/* ------------------------------ DT glue ---------------------------------- */
/*
 * Optional DT props (all optional; defaults shown):
 *  - pcie-bus = <0>;
 *  - pcie-device = <0>;
 *  - pcie-function = <0>;
 *  - pcie-bar = <1>;
 *  - rp1-i2c-index = <1>;      // IMPORTANT for header GPIO2/3
 *  - clock-frequency = <400000>;
 */
#define RP1_PCIE_I2C_INIT(inst)                                                    \
	static struct rp1_pcie_i2c_data rp1_pcie_i2c_data_##inst;                   \
	static const struct rp1_pcie_i2c_config rp1_pcie_i2c_cfg_##inst = {         \
		.bus = (uint8_t)DT_INST_PROP_OR(inst, pcie_bus, 0),                 \
		.dev = (uint8_t)DT_INST_PROP_OR(inst, pcie_device, 0),              \
		.fun = (uint8_t)DT_INST_PROP_OR(inst, pcie_function, 0),            \
		.bar = (uint8_t)DT_INST_PROP_OR(inst, pcie_bar, 1),                 \
		.rp1_i2c_index = (uint8_t)DT_INST_PROP_OR(inst, rp1_i2c_index, 1),  \
		.bitrate = (uint32_t)DT_INST_PROP_OR(inst, clock_frequency, 100000),\
	};                                                                            \
	DEVICE_DT_INST_DEFINE(inst, rp1_pcie_i2c_init, NULL,                          \
			      &rp1_pcie_i2c_data_##inst,                           \
			      &rp1_pcie_i2c_cfg_##inst, POST_KERNEL,               \
			      CONFIG_I2C_INIT_PRIORITY, &rp1_pcie_i2c_api);

DT_INST_FOREACH_STATUS_OKAY(RP1_PCIE_I2C_INIT)
