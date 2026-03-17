#pragma once

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Channel mask bits (use whatever you already print) */
#define UART_CH_CLEAR   (1u << 0)
#define UART_CH_RED     (1u << 1)
#define UART_CH_GREEN   (1u << 2)
#define UART_CH_BLUE    (1u << 3)
#define UART_CH_WIDE    (1u << 4)
#define UART_CH_FLICKER (1u << 5)

#define UART_CH_ALL (UART_CH_CLEAR | UART_CH_RED | UART_CH_GREEN | UART_CH_BLUE | UART_CH_WIDE | UART_CH_FLICKER)

typedef enum {
	UART_CMD_NONE = 0,
	UART_CMD_HELP,
	UART_CMD_START,
	UART_CMD_STOP,
	UART_CMD_STATUS,
	UART_CMD_SET_MASK,     /* cmd.mask valid */
	UART_CMD_READ_ONCE,    /* request a single sample print */
	UART_CMD_PRINT_FLICKER /* print last flicker value */
} uart_cmd_type_t;

typedef struct {
	uart_cmd_type_t type;
	uint32_t mask;   /* valid for UART_CMD_SET_MASK */
} uart_cmd_t;

typedef struct {
	const struct device *uart;

	/* Shared TX mutex (so main + RX thread don’t interleave bytes too badly) */
	struct k_mutex tx_lock;

	/* Command queue */
	struct k_msgq *cmd_q;
} uart_cmd_ctx_t;

/**
 * Initialize UART command module and start RX thread.
 *
 * @param ctx      module context
 * @param uart_dev UART device to use (typically DT_CHOSEN(zephyr_console))
 * @return 0 on success
 */
int uart_cmd_init(uart_cmd_ctx_t *ctx, const struct device *uart_dev);

/**
 * Get next parsed command (non-blocking or blocking depending on timeout).
 *
 * @param ctx
 * @param out
 * @param timeout  e.g. K_NO_WAIT, K_MSEC(100), K_FOREVER
 * @return 0 if a command was returned, -EAGAIN if none (when non-blocking), other <0 on error
 */
int uart_cmd_get(uart_cmd_ctx_t *ctx, uart_cmd_t *out, k_timeout_t timeout);

/**
 * Print to UART (thread-safe-ish).
 */
void uart_cmd_printf(uart_cmd_ctx_t *ctx, const char *fmt, ...);

/**
 * Print help menu.
 */
void uart_cmd_print_help(uart_cmd_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

