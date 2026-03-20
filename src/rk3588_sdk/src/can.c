#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>  // 必须包含：ioctl函数声明
#include <net/if.h>     // 必须包含：SIOCGIFINDEX、ifreq结构体定义
#include <linux/can.h>
#include <linux/can/raw.h>

// 原有CAN驱动代码（示例，保留你的业务逻辑）
int rk3588_can_init(const char *ifname) {
    int s;
    struct ifreq ifr;
    struct sockaddr_can addr;

    // 创建CAN套接字
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("socket");
        return -1;
    }

    // 获取CAN接口索引
    strcpy(ifr.ifr_name, ifname);
    ioctl(s, SIOCGIFINDEX, &ifr);  // 现在能正确识别SIOCGIFINDEX

    // 绑定CAN接口
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(s);
        return -1;
    }

    return s;
}

// 其他CAN驱动函数（保留你的原有代码）
