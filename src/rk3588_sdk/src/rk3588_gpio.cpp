#include "rk3588_gpio.h"

RK3588_GPIO::RK3588_GPIO(int gpio_num) {
    gpio_num_ = gpio_num;
    snprintf(gpio_path_, sizeof(gpio_path_), "%s/gpio%d", RK3588_GPIO_BASE, gpio_num);
    
    // 导出GPIO（未导出则执行）
    if (!isExported()) {
        int fd = open(RK3588_GPIO_BASE "/export", O_WRONLY);
        if (fd < 0) {
            perror("Failed to export GPIO");
            return;
        }
        char buf[16];
        snprintf(buf, sizeof(buf), "%d", gpio_num);
        write(fd, buf, strlen(buf));
        close(fd);
        // 等待sysfs节点创建
        usleep(10000);
    }
}

RK3588_GPIO::~RK3588_GPIO() {
    // 释放GPIO
    int fd = open(RK3588_GPIO_BASE "/unexport", O_WRONLY);
    if (fd < 0) {
        perror("Failed to unexport GPIO");
        return;
    }
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", gpio_num_);
    write(fd, buf, strlen(buf));
    close(fd);
}

int RK3588_GPIO::isExported() {
    struct stat st;
    return (stat(gpio_path_, &st) == 0);
}

int RK3588_GPIO::setDirection(const char* dir) {
    char path[256];
    snprintf(path, sizeof(path), "%s/direction", gpio_path_);
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("Failed to open direction file");
        return -1;
    }
    write(fd, dir, strlen(dir));
    close(fd);
    return 0;
}

int RK3588_GPIO::setValue(const char* value) {
    char path[256];
    snprintf(path, sizeof(path), "%s/value", gpio_path_);
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("Failed to open value file");
        return -1;
    }
    write(fd, value, strlen(value));
    close(fd);
    return 0;
}

int RK3588_GPIO::getValue(char* value) {
    char path[256];
    snprintf(path, sizeof(path), "%s/value", gpio_path_);
    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        perror("Failed to open value file");
        return -1;
    }
    read(fd, value, 1);
    close(fd);
    return 0;
}
