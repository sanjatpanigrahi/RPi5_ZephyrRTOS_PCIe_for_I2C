#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include "uart_cmd.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#define RP1_I2C_NODE DT_NODELABEL(rp1_i2c1)

#if !DT_NODE_HAS_STATUS(RP1_I2C_NODE, okay)
#error "rp1_i2c1 node is not okay in devicetree (overlay not applied?)"
#endif

static const struct device *const i2c_dev = DEVICE_DT_GET(RP1_I2C_NODE);

/* Console UART (same as Zephyr logs) */
static const struct device *const uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
static uart_cmd_ctx_t uart_ctx;

/* -------------------- TCS3408 basics -------------------- */
#define TCS3408_ADDR      0x39  /* 7-bit */

#define REG_PARTID        0x90
#define REG_REVID         0x91
#define REG_ID            0x92

#define CMD_BIT           0x80

#define REG_ENABLE        0x80
#define REG_ATIME         0x81
#define REG_CFG1          0x83

#define EN_PON            (1u << 0)
#define EN_AEN            (1u << 1)

/* Channel regs (16-bit LE) */
#define REG_CDATA_L       0x94
#define REG_RDATA_L       0x96
#define REG_GDATA_L       0x98
#define REG_BDATA_L       0x9A
#define REG_WDATA_L       0x9C

/* Flicker placeholder; adjust if you know exact address */
#define REG_FLICKER_L     0xA0

/* -------------------- Streaming + cached data -------------------- */
static bool streaming = false;
static bool read_once = false;
static uint32_t ch_mask = UART_CH_ALL;

static uint16_t last_flicker = 0;
static int last_flicker_valid = 0;

static uint16_t last_c = 0, last_r = 0, last_g = 0, last_b = 0, last_w = 0;

/* -------------------- Helpers: robust reg access -------------------- */

static int tcs_write8(const struct device *i2c, uint8_t addr, uint8_t reg, uint8_t val)
{
	uint8_t buf[2] = { reg, val };
	return i2c_write(i2c, buf, sizeof(buf), addr);
}

static int tcs_readn_rs(const struct device *i2c, uint8_t addr, uint8_t reg, uint8_t *buf, size_t n)
{
	/* repeated-start */
	return i2c_write_read(i2c, addr, &reg, 1, buf, n);
}

static int tcs_readn_auto(const struct device *i2c, uint8_t addr, uint8_t reg, uint8_t *buf, size_t n, bool *used_cmd_bit)
{
	int rc = tcs_readn_rs(i2c, addr, reg, buf, n);
	if (rc == 0) {
		if (used_cmd_bit) *used_cmd_bit = false;
		return 0;
	}

	rc = tcs_readn_rs(i2c, addr, (uint8_t)(CMD_BIT | reg), buf, n);
	if (rc == 0) {
		if (used_cmd_bit) *used_cmd_bit = true;
		return 0;
	}

	return rc;
}

static int tcs_write8_auto(const struct device *i2c, uint8_t addr, uint8_t reg, uint8_t val, bool use_cmd_bit)
{
	uint8_t r = use_cmd_bit ? (uint8_t)(CMD_BIT | reg) : reg;
	return tcs_write8(i2c, addr, r, val);
}

static int tcs_read8_auto(const struct device *i2c, uint8_t addr, uint8_t reg, uint8_t *val, bool *used_cmd_bit)
{
	return tcs_readn_auto(i2c, addr, reg, val, 1, used_cmd_bit);
}

static int tcs_read16_auto(const struct device *i2c, uint8_t addr, uint8_t reg_l, uint16_t *out, bool use_cmd_bit)
{
	uint8_t r = use_cmd_bit ? (uint8_t)(CMD_BIT | reg_l) : reg_l;
	uint8_t b[2] = {0};
	int rc = tcs_readn_rs(i2c, addr, r, b, 2);
	if (rc) return rc;
	*out = (uint16_t)b[0] | ((uint16_t)b[1] << 8);
	return 0;
}

/* -------------------- Sensor “validation” -------------------- */

static int tcs_validate_identity(bool *use_cmd_bit_out, uint8_t *partid, uint8_t *revid, uint8_t *id)
{
	bool use_cmd_bit = false;

	uint8_t v = 0;
	int rc = tcs_read8_auto(i2c_dev, TCS3408_ADDR, REG_PARTID, &v, &use_cmd_bit);
	if (rc) return rc;
	*partid = v;

	rc = tcs_read8_auto(i2c_dev, TCS3408_ADDR, REG_REVID, &v, NULL);
	if (rc) return rc;
	*revid = v;

	rc = tcs_read8_auto(i2c_dev, TCS3408_ADDR, REG_ID, &v, NULL);
	if (rc) return rc;
	*id = v;

	*use_cmd_bit_out = use_cmd_bit;

	if ((*partid == 0x00) || (*partid == 0xFF) || (*revid == 0x00) || (*revid == 0xFF)) {
		return -ENODEV;
	}

	return 0;
}

/* -------------------- Minimal start/config -------------------- */

static int tcs_start_measurement(bool use_cmd_bit)
{
	int rc = tcs_write8_auto(i2c_dev, TCS3408_ADDR, REG_ENABLE, EN_PON, use_cmd_bit);
	if (rc) return rc;

	k_msleep(5);

	rc = tcs_write8_auto(i2c_dev, TCS3408_ADDR, REG_ENABLE, EN_PON | EN_AEN, use_cmd_bit);
	if (rc) return rc;

	/* Optional ATIME tweak if you want
	 * rc = tcs_write8_auto(i2c_dev, TCS3408_ADDR, REG_ATIME, 0xDB, use_cmd_bit);
	 * if (rc) return rc;
	 */

	return 0;
}

/* -------------------- AGC (gain) -------------------- */
/* Assumption: REG_CFG1 AGAIN in [1:0], 0..3 => x1,x4,x16,x64 */
static const char *gain_names[] = { "x1", "x4", "x16", "x64" };

static int tcs_get_gain_code(bool use_cmd_bit, uint8_t *gain_code)
{
	uint8_t cfg1 = 0;
	int rc = tcs_read8_auto(i2c_dev, TCS3408_ADDR, REG_CFG1, &cfg1, NULL);
	if (rc) return rc;

	*gain_code = (cfg1 & 0x03u);
	return 0;
}

static int tcs_set_gain_code(bool use_cmd_bit, uint8_t gain_code)
{
	uint8_t cfg1 = 0;
	int rc = tcs_read8_auto(i2c_dev, TCS3408_ADDR, REG_CFG1, &cfg1, NULL);
	if (rc) return rc;

	cfg1 &= ~0x03u;
	cfg1 |= (gain_code & 0x03u);

	return tcs_write8_auto(i2c_dev, TCS3408_ADDR, REG_CFG1, cfg1, use_cmd_bit);
}

#define SAT_THRESH   0xFFF0u
#define LOW_THRESH   0x0100u

static int tcs_agc_step(bool use_cmd_bit,
			uint16_t c, uint16_t r, uint16_t g, uint16_t b, uint16_t w,
			bool *saturated_out, bool *low_out,
			uint8_t *gain_code_inout)
{
	bool sat = (c >= SAT_THRESH) || (r >= SAT_THRESH) || (g >= SAT_THRESH) || (b >= SAT_THRESH) || (w >= SAT_THRESH);
	bool low = (c <= LOW_THRESH) && (r <= LOW_THRESH) && (g <= LOW_THRESH) && (b <= LOW_THRESH) && (w <= LOW_THRESH);

	*saturated_out = sat;
	*low_out = low;

	uint8_t gc = *gain_code_inout;

	if (sat) {
		if (gc > 0) {
			gc--;
			int rc = tcs_set_gain_code(use_cmd_bit, gc);
			if (rc) return rc;
			LOG_WRN("AGC: saturation -> lowering gain to %s (code=%u)", gain_names[gc], gc);
		}
	} else if (low) {
		if (gc < 3) {
			gc++;
			int rc = tcs_set_gain_code(use_cmd_bit, gc);
			if (rc) return rc;
			LOG_INF("AGC: low signal -> raising gain to %s (code=%u)", gain_names[gc], gc);
		}
	}

	*gain_code_inout = gc;
	return 0;
}

/* -------------------- Read channels -------------------- */

static int tcs_read_channels(bool use_cmd_bit,
			     uint16_t *c, uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *w,
			     int *flicker_valid, uint16_t *flicker)
{
	int rc;

	rc = tcs_read16_auto(i2c_dev, TCS3408_ADDR, REG_CDATA_L, c, use_cmd_bit);
	if (rc) return rc;
	rc = tcs_read16_auto(i2c_dev, TCS3408_ADDR, REG_RDATA_L, r, use_cmd_bit);
	if (rc) return rc;
	rc = tcs_read16_auto(i2c_dev, TCS3408_ADDR, REG_GDATA_L, g, use_cmd_bit);
	if (rc) return rc;
	rc = tcs_read16_auto(i2c_dev, TCS3408_ADDR, REG_BDATA_L, b, use_cmd_bit);
	if (rc) return rc;

	rc = tcs_read16_auto(i2c_dev, TCS3408_ADDR, REG_WDATA_L, w, use_cmd_bit);
	if (rc) {
		*w = 0;
	}

	*flicker_valid = 0;
	*flicker = 0;
	rc = tcs_read16_auto(i2c_dev, TCS3408_ADDR, REG_FLICKER_L, flicker, use_cmd_bit);
	if (rc == 0) {
		*flicker_valid = 1;
	}

	return 0;
}

/* -------------------- UART command processing -------------------- */

static void handle_uart_cmd(const uart_cmd_t *cmd)
{
	switch (cmd->type) {
	case UART_CMD_HELP:
		uart_cmd_print_help(&uart_ctx);
		break;

	case UART_CMD_START:
		streaming = true;
		uart_cmd_printf(&uart_ctx, "OK: streaming started");
		break;

	case UART_CMD_STOP:
		streaming = false;
		uart_cmd_printf(&uart_ctx, "OK: streaming stopped");
		break;

	case UART_CMD_STATUS:
		uart_cmd_printf(&uart_ctx, "status: streaming=%d mask=0x%02x",
				streaming ? 1 : 0, ch_mask);
		break;

	case UART_CMD_SET_MASK:
		ch_mask = cmd->mask;
		uart_cmd_printf(&uart_ctx, "OK: mask set to 0x%02x", ch_mask);
		break;

	case UART_CMD_READ_ONCE:
		read_once = true;
		uart_cmd_printf(&uart_ctx, "OK: read once requested");
		break;

	case UART_CMD_PRINT_FLICKER:
		if (last_flicker_valid) {
			uart_cmd_printf(&uart_ctx, "flicker=%u", last_flicker);
		} else {
			uart_cmd_printf(&uart_ctx, "flicker=N/A (not read yet or reg not supported)");
		}
		break;

	default:
		/* ignore */
		break;
	}
}

static void uart_poll_commands(void)
{
	uart_cmd_t cmd;
	while (uart_cmd_get(&uart_ctx, &cmd, K_NO_WAIT) == 0) {
		handle_uart_cmd(&cmd);
	}
}

/* Print sample based on current ch_mask */
static void print_sample(bool sat, bool low, uint8_t gain_code,
			 uint16_t c, uint16_t r, uint16_t g, uint16_t b, uint16_t w,
			 int flicker_valid, uint16_t flicker)
{
	char line[256];
	int n = 0;

	n += snprintk(line + n, sizeof(line) - n, "gain=%s", gain_names[gain_code]);
	if (sat) n += snprintk(line + n, sizeof(line) - n, " [SAT]");
	if (low) n += snprintk(line + n, sizeof(line) - n, " [LOW]");

	if (ch_mask & UART_CH_CLEAR)   n += snprintk(line + n, sizeof(line) - n, " C=%u", c);
	if (ch_mask & UART_CH_RED)     n += snprintk(line + n, sizeof(line) - n, " R=%u", r);
	if (ch_mask & UART_CH_GREEN)   n += snprintk(line + n, sizeof(line) - n, " G=%u", g);
	if (ch_mask & UART_CH_BLUE)    n += snprintk(line + n, sizeof(line) - n, " B=%u", b);
	if (ch_mask & UART_CH_WIDE)    n += snprintk(line + n, sizeof(line) - n, " W=%u", w);

	if (ch_mask & UART_CH_FLICKER) {
		if (flicker_valid) {
			n += snprintk(line + n, sizeof(line) - n, " F=%u", flicker);
		} else {
			n += snprintk(line + n, sizeof(line) - n, " F=N/A");
		}
	}

	uart_cmd_printf(&uart_ctx, "%s", line);
}

/* -------------------- main -------------------- */

int main(void)
{
	LOG_INF("RP1 PCIe+I2C TCS3408 validate + AGC + UART CLI starting...");

	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("RP1 I2C device not ready");
		return 0;
	}
	LOG_INF("Got I2C device: %s", i2c_dev->name);

	if (!device_is_ready(uart_dev)) {
		LOG_ERR("Console UART not ready");
		return 0;
	}

	int rc = uart_cmd_init(&uart_ctx, uart_dev);
	if (rc) {
		LOG_ERR("uart_cmd_init failed: %d", rc);
		return 0;
	}
	uart_cmd_print_help(&uart_ctx);

	bool use_cmd_bit = false;
	uint8_t partid = 0, revid = 0, id = 0;

	rc = tcs_validate_identity(&use_cmd_bit, &partid, &revid, &id);
	if (rc == -ENODEV) {
		uart_cmd_printf(&uart_ctx,
				"TCS3408 validation failed: PARTID/REVID look invalid (PARTID=0x%02x REVID=0x%02x)",
				partid, revid);
		return 0;
	}
	if (rc) {
		uart_cmd_printf(&uart_ctx, "TCS3408 validation read failed: %d", rc);
		return 0;
	}

	uart_cmd_printf(&uart_ctx, "TCS3408 detected: PARTID=0x%02x REVID=0x%02x ID=0x%02x (cmd_bit=%s)",
			partid, revid, id, use_cmd_bit ? "YES" : "NO");

	rc = tcs_start_measurement(use_cmd_bit);
	if (rc) {
		uart_cmd_printf(&uart_ctx, "TCS3408 start failed: %d", rc);
		return 0;
	}

	/* Initialize gain */
	uint8_t gain_code = 1;
	(void)tcs_set_gain_code(use_cmd_bit, gain_code);
	(void)tcs_get_gain_code(use_cmd_bit, &gain_code);
	uart_cmd_printf(&uart_ctx, "Initial gain: %s (code=%u)", gain_names[gain_code], gain_code);

	/* Default: not streaming until 'start' */
	streaming = false;
	read_once = false;
	ch_mask = UART_CH_ALL;

	const int sample_period_ms = 200;

	while (1) {
		/* process UART commands */
		uart_poll_commands();

		/* Decide if we should take a sample */
		bool do_sample = streaming || read_once;

		if (do_sample) {
			uint16_t c = 0, rch = 0, gch = 0, bch = 0, w = 0;
			int flicker_valid = 0;
			uint16_t flicker = 0;

			rc = tcs_read_channels(use_cmd_bit, &c, &rch, &gch, &bch, &w, &flicker_valid, &flicker);
			if (rc) {
				uart_cmd_printf(&uart_ctx, "Read channels failed: %d", rc);
				k_msleep(50);
				continue;
			}

			/* cache last values */
			last_c = c; last_r = rch; last_g = gch; last_b = bch; last_w = w;
			if (flicker_valid) {
				last_flicker_valid = 1;
				last_flicker = flicker;
			}

			bool sat = false, low = false;
			rc = tcs_agc_step(use_cmd_bit, c, rch, gch, bch, w, &sat, &low, &gain_code);
			if (rc) {
				uart_cmd_printf(&uart_ctx, "AGC step failed: %d", rc);
			}

			print_sample(sat, low, gain_code, c, rch, gch, bch, w, flicker_valid, flicker);

			/* single-shot consumed */
			read_once = false;

			/* throttle */
			k_msleep(sample_period_ms);
		} else {
			/* idle */
			k_msleep(20);
		}
	}

	return 0;
}
