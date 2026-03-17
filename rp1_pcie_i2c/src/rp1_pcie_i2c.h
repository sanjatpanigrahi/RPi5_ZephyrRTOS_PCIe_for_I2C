/* src/rp1_pcie_i2c.h */

#ifndef RP1_PCIE_I2C_H
#define RP1_PCIE_I2C_H

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pcie/pcie.h>
#include <zephyr/dt-bindings/pcie/pcie.h>

/* RP1 PCI IDs (as seen in your log) */
#define RP1_PCI_VENDOR_ID 0x1de4
#define RP1_PCI_DEVICE_ID 0x0001

/* On Zephyr RPi5, RP1 only exposes BAR0, 16 KB */
#define RP1_PCI_BAR_INDEX 0

/*
 * IMPORTANT NOTE:
 *
 * In Linux rp1.dtsi, I2C0 is at 0x40070000 within RP1's *internal* map.
 * Here, firmware / PCIe host have already turned that into a BAR window:
 *
 *   BAR0: phys=0x1f00410000 size=0x4000
 *
 * We don't know the exact internal offset behind this window yet, but
 * the only reasonable thing we can do (for now) is to treat BAR0 as
 * the I2C register block directly, i.e. offset 0 inside the BAR.
 *
 * So for RP1_I2C0, we use offset 0.
 */

#define RP1_I2C0_BASE_OFFSET  0x0000u

/* (We can add I2C1..I2C6 offsets later once we reverse-map the window.) */

const struct device *rp1_pcie_i2c0_get_device(void);

#endif /* RP1_PCIE_I2C_H */

