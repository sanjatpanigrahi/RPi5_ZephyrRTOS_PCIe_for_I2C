#include <uart_cmd.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <ctype.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>

LOG_MODULE_REGISTER(uart_cmd, LOG_LEVEL_INF);

/* ---------- Tunables ---------- */
#define UART_CMD_LINE_MAX   96
#define UART_CMD_Q_DEPTH    8
#define UART_RX_STACK_SIZE  2048
#define UART_RX_PRIORITY    5

/* ---------- Command queue storage ---------- */
K_MSGQ_DEFINE(g_cmd_q, sizeof(uart_cmd_t), UART_CMD_Q_DEPTH, 4);

/* ---------- RX thread ---------- */
K_THREAD_STACK_DEFINE(g_rx_stack, UART_RX_STACK_SIZE);
static struct k_thread g_rx_thread;

/* ---------- local helpers ---------- */

static void uart_putc_locked(uart_cmd_ctx_t *ctx, char c)
{
	uart_poll_out(ctx->uart, (unsigned char)c);
}

static void uart_puts_locked(uart_cmd_ctx_t *ctx, const char *s)
{
	while (*s) {
		uart_putc_locked(ctx, *s++);
	}
}

void uart_cmd_printf(uart_cmd_ctx_t *ctx, const char *fmt, ...)
{
	if (!ctx || !ctx->uart) {
		return;
	}

	char buf[256];

	va_list ap;
	va_start(ap, fmt);
	vsnprintk(buf, sizeof(buf), fmt, ap);
	va_end(ap);

	k_mutex_lock(&ctx->tx_lock, K_FOREVER);
	uart_puts_locked(ctx, buf);
	uart_puts_locked(ctx, "\r\n");
	k_mutex_unlock(&ctx->tx_lock);
}

void uart_cmd_print_help(uart_cmd_ctx_t *ctx)
{
	uart_cmd_printf(ctx, "Commands:");
	uart_cmd_printf(ctx, "  help | ?                 : show this help");
	uart_cmd_printf(ctx, "  start                    : start streaming");
	uart_cmd_printf(ctx, "  stop                     : stop streaming");
	uart_cmd_printf(ctx, "  read                     : read+print one sample now");
	uart_cmd_printf(ctx, "  status                   : show current mode/mask");
	uart_cmd_printf(ctx, "  ch <c|r|g|b|w|f>          : show only one channel (c=clear,w=wide,f=flicker)");
	uart_cmd_printf(ctx, "  ch <clear|red|green|blue|wide|flicker> : same as above");
	uart_cmd_printf(ctx, "  mask <list>              : set channels, comma-separated");
	uart_cmd_printf(ctx, "                             e.g. mask c,r,g,b   or mask clear,wide");
	uart_cmd_printf(ctx, "  all                      : same as mask all");
	uart_cmd_printf(ctx, "  flicker                  : print latest flicker value (from last sample)");
	uart_cmd_printf(ctx, "");
	uart_cmd_printf(ctx, "Notes:");
	uart_cmd_printf(ctx, "  - Streaming prints only the selected mask.");
	uart_cmd_printf(ctx, "  - 'read' prints one sample even if streaming is stopped.");
}

/* Trim leading/trailing whitespace in-place */
static char *trim(char *s)
{
	while (isspace((unsigned char)*s)) s++;
	if (*s == 0) return s;

	char *end = s + strlen(s) - 1;
	while (end > s && isspace((unsigned char)*end)) end--;
	end[1] = '\0';
	return s;
}

/* Lowercase in-place */
static void tolower_str(char *s)
{
	for (; *s; s++) {
		*s = (char)tolower((unsigned char)*s);
	}
}

/* Parse single token to channel bit */
static bool token_to_chbit(const char *t, uint32_t *bit_out)
{
	if (!t || !bit_out) return false;

	if (!strcmp(t, "c") || !strcmp(t, "clear"))   { *bit_out = UART_CH_CLEAR;   return true; }
	if (!strcmp(t, "r") || !strcmp(t, "red"))     { *bit_out = UART_CH_RED;     return true; }
	if (!strcmp(t, "g") || !strcmp(t, "green"))   { *bit_out = UART_CH_GREEN;   return true; }
	if (!strcmp(t, "b") || !strcmp(t, "blue"))    { *bit_out = UART_CH_BLUE;    return true; }
	if (!strcmp(t, "w") || !strcmp(t, "wide"))    { *bit_out = UART_CH_WIDE;    return true; }
	if (!strcmp(t, "f") || !strcmp(t, "flicker")) { *bit_out = UART_CH_FLICKER; return true; }
	if (!strcmp(t, "all"))                        { *bit_out = UART_CH_ALL;     return true; }

	return false;
}

static uint32_t parse_mask_list(char *list)
{
	/* list is like "c,r,g" or "clear,wide" */
	uint32_t mask = 0;

	char *save = NULL;
	for (char *tok = strtok_r(list, ",", &save); tok; tok = strtok_r(NULL, ",", &save)) {
		tok = trim(tok);
		uint32_t bit = 0;
		if (token_to_chbit(tok, &bit)) {
			if (bit == UART_CH_ALL) {
				return UART_CH_ALL;
			}
			mask |= bit;
		}
	}

	return mask;
}

static bool parse_line_to_cmd(char *line, uart_cmd_t *out)
{
	memset(out, 0, sizeof(*out));

	line = trim(line);
	if (!*line) {
		out->type = UART_CMD_NONE;
		return false;
	}

	tolower_str(line);

	/* single word commands */
	if (!strcmp(line, "help") || !strcmp(line, "?")) {
		out->type = UART_CMD_HELP;
		return true;
	}
	if (!strcmp(line, "start")) {
		out->type = UART_CMD_START;
		return true;
	}
	if (!strcmp(line, "stop")) {
		out->type = UART_CMD_STOP;
		return true;
	}
	if (!strcmp(line, "read")) {
		out->type = UART_CMD_READ_ONCE;
		return true;
	}
	if (!strcmp(line, "status")) {
		out->type = UART_CMD_STATUS;
		return true;
	}
	if (!strcmp(line, "all")) {
		out->type = UART_CMD_SET_MASK;
		out->mask = UART_CH_ALL;
		return true;
	}
	if (!strcmp(line, "flicker")) {
		out->type = UART_CMD_PRINT_FLICKER;
		return true;
	}

	/* commands with args */
	char *save = NULL;
	char *cmd = strtok_r(line, " ", &save);
	char *arg = strtok_r(NULL, "", &save); /* rest of string */

	if (!cmd) {
		out->type = UART_CMD_NONE;
		return false;
	}

	if (!strcmp(cmd, "ch")) {
		if (!arg) {
			out->type = UART_CMD_HELP;
			return true;
		}
		arg = trim(arg);
		uint32_t bit = 0;
		if (!token_to_chbit(arg, &bit)) {
			/* invalid channel -> help */
			out->type = UART_CMD_HELP;
			return true;
		}
		out->type = UART_CMD_SET_MASK;
		out->mask = (bit == UART_CH_ALL) ? UART_CH_ALL : bit;
		return true;
	}

	if (!strcmp(cmd, "mask")) {
		if (!arg) {
			out->type = UART_CMD_HELP;
			return true;
		}
		arg = trim(arg);
		uint32_t m = parse_mask_list(arg);
		if (m == 0) {
			out->type = UART_CMD_HELP;
			return true;
		}
		out->type = UART_CMD_SET_MASK;
		out->mask = m;
		return true;
	}

	/* unknown -> help */
	out->type = UART_CMD_HELP;
	return true;
}

static void push_cmd(uart_cmd_ctx_t *ctx, const uart_cmd_t *cmd)
{
	(void)k_msgq_put(ctx->cmd_q, cmd, K_NO_WAIT);
}

/* Basic line editor over uart_poll_in */
static void rx_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	uart_cmd_ctx_t *ctx = (uart_cmd_ctx_t *)p1;
	char line[UART_CMD_LINE_MAX];
	size_t n = 0;

	/* Prompt */
	uart_cmd_printf(ctx, "UART CLI ready. Type 'help'.");

	while (1) {
		unsigned char c;
		int rc = uart_poll_in(ctx->uart, &c);
		if (rc != 0) {
			k_msleep(5);
			continue;
		}

		/* handle CR/LF */
		if (c == '\r' || c == '\n') {
			/* echo newline */
			k_mutex_lock(&ctx->tx_lock, K_FOREVER);
			uart_puts_locked(ctx, "\r\n");
			k_mutex_unlock(&ctx->tx_lock);

			line[n] = '\0';

			uart_cmd_t cmd;
			if (parse_line_to_cmd(line, &cmd) && cmd.type != UART_CMD_NONE) {
				push_cmd(ctx, &cmd);
			}

			n = 0;
			continue;
		}

		/* backspace */
		if (c == 0x08 || c == 0x7F) {
			if (n > 0) {
				n--;
				/* erase from terminal: "\b \b" */
				k_mutex_lock(&ctx->tx_lock, K_FOREVER);
				uart_puts_locked(ctx, "\b \b");
				k_mutex_unlock(&ctx->tx_lock);
			}
			continue;
		}

		/* printable */
		if (isprint(c)) {
			if (n < (sizeof(line) - 1)) {
				line[n++] = (char)c;
				/* echo */
				k_mutex_lock(&ctx->tx_lock, K_FOREVER);
				uart_putc_locked(ctx, (char)c);
				k_mutex_unlock(&ctx->tx_lock);
			} else {
				/* line overflow -> reset */
				n = 0;
				uart_cmd_printf(ctx, "Line too long. Cleared.");
			}
		}
	}
}

int uart_cmd_init(uart_cmd_ctx_t *ctx, const struct device *uart_dev)
{
	if (!ctx || !uart_dev) return -EINVAL;
	if (!device_is_ready(uart_dev)) return -ENODEV;

	memset(ctx, 0, sizeof(*ctx));
	ctx->uart = uart_dev;
	ctx->cmd_q = &g_cmd_q;
	k_mutex_init(&ctx->tx_lock);

	k_thread_create(&g_rx_thread, g_rx_stack, K_THREAD_STACK_SIZEOF(g_rx_stack),
			rx_thread_fn, ctx, NULL, NULL,
			UART_RX_PRIORITY, 0, K_NO_WAIT);

	k_thread_name_set(&g_rx_thread, "uart_cmd_rx");
	return 0;
}

int uart_cmd_get(uart_cmd_ctx_t *ctx, uart_cmd_t *out, k_timeout_t timeout)
{
	if (!ctx || !out) return -EINVAL;

	int rc = k_msgq_get(ctx->cmd_q, out, timeout);
	if (rc == 0) return 0;

	/* If non-blocking and empty, Zephyr returns -EAGAIN typically */
	return rc;
}

