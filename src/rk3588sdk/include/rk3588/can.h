#ifndef RK3588_CAN_H
#define RK3588_CAN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t id;
    uint8_t data[8];
    uint8_t len;
} can_msg_t;

int rk3588_can_init(const char *dev);
int rk3588_can_send(int fd, const can_msg_t *msg);
int rk3588_can_recv(int fd, can_msg_t *msg);
void rk3588_can_deinit(int fd);

#ifdef __cplusplus
}
#endif

#endif