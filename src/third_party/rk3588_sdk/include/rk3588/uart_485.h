#ifndef RK3588_UART_485_H
#define RK3588_UART_485_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int rk3588_uart_init(const char *dev, int baudrate);
int rk3588_uart_send(int fd, const uint8_t *buf, int len);
int rk3588_uart_recv(int fd, uint8_t *buf, int max_len, int timeout_ms);
void rk3588_uart_deinit(int fd);

#ifdef __cplusplus
}
#endif

#endif