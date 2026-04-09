nclude "rk3588/can.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>

// CAN初始化实现
bool RK3588_CAN::init(const std::string& interface_name) {
    // 打开CAN设备
    can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_fd < 0) {
        return false;
    }

    struct ifreq ifr;
    strcpy(ifr.ifr_name, interface_name.c_str());
    ioctl(can_fd, SIOCGIFINDEX, &ifr);

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(can_fd);
        return false;
    }

    // 设置非阻塞模式（可选）
    int flags = fcntl(can_fd, F_GETFL, 0);
    fcntl(can_fd, F_SETFL, flags | O_NONBLOCK);

    return true;
}

// CAN发送实现
int RK3588_CAN::send(uint32_t id, uint8_t* data, uint8_t len) {
    if (can_fd < 0 || len > 8) return -1;

    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = len;
    memcpy(frame.data, data, len);

    return write(can_fd, &frame, sizeof(frame));
}

// CAN接收实现
int RK3588_CAN::recv(uint32_t& id, uint8_t* data, uint8_t& len) {
    if (can_fd < 0) return -1;

    struct can_frame frame;
    int nbytes = read(can_fd, &frame, sizeof(frame));
    if (nbytes <= 0) return -1;

    id = frame.can_id;
    len = frame.can_dlc;
    memcpy(data, frame.data, len);

    return nbytes;
}

// 关闭CAN
void RK3588_CAN::close() {
    if (can_fd >= 0) {
        ::close(can_fd);
        can_fd = -1;
    }
}
