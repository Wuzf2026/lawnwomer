#include "rk3588/uart_485.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

static int set_speed(int fd, int speed)
{
    int speeds[] = {9600, 115200, 460800, 921600};
    int cfg[] = {B9600, B115200, B460800, B921600};
    struct termios opt;
    tcgetattr(fd, &opt);

    for (int i = 0; i < 4; i++) {
        if (speed == speeds[i]) {
            cfsetispeed(&opt, cfg[i]);
            cfsetospeed(&opt, cfg[i]);
            break;
        }
    }

    opt.c_cflag |= CLOCAL | CREAD;
    opt.c_cflag &= ~CSIZE;
    opt.c_cflag |= CS8;
    opt.c_cflag &= ~PARENB;
    opt.c_cflag &= ~CSTOPB;
    tcsetattr(fd, TCSANOW, &opt);
    return 0;
}

int rk3588_uart_init(const char *dev, int baudrate)
{
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) return -1;
    set_speed(fd, baudrate);
    return fd;
}

int rk3588_uart_send(int fd, const uint8_t *buf, int len)
{
    return write(fd, buf, len);
}

int rk3588_uart_recv(int fd, uint8_t *buf, int max_len, int timeout_ms)
{
    fd_set fds;
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    int ret = select(fd + 1, &fds, NULL, NULL, &tv);
    if (ret <= 0) return 0;
    return read(fd, buf, max_len);
}

void rk3588_uart_deinit(int fd)
{
    close(fd);
}