#ifndef RK3588_GPIO_H
#define RK3588_GPIO_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

// RK3588 GPIO编号映射（Ubuntu20.04 sysfs路径）
#define RK3588_GPIO_BASE "/sys/class/gpio"

class RK3588_GPIO {
public:
    // 构造函数：导出GPIO引脚
    RK3588_GPIO(int gpio_num);
    // 析构函数：释放GPIO引脚
    ~RK3588_GPIO();
    // 设置GPIO方向（in/out）
    int setDirection(const char* dir);
    // 设置GPIO输出电平（high/low）
    int setValue(const char* value);
    // 读取GPIO输入电平
    int getValue(char* value);

private:
    int gpio_num_;          // GPIO编号（RK3588物理引脚映射）
    char gpio_path_[256];   // GPIO sysfs路径
    // 检查GPIO是否已导出
    int isExported();
};

#endif // RK3588_GPIO_H
