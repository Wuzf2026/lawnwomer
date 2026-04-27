#ifndef HANDSFREE_RTK_UM982_USB_SDK_H
#define HANDSFREE_RTK_UM982_USB_SDK_H

// 空框架：适配um982_gps_driver.cpp的基础调用
namespace handsfree_rtk {
    class UM982USBSDK {
    public:
        UM982USBSDK() = default;
        ~UM982USBSDK() = default;
        // 基础接口
        bool openUSBPort(const std::string& port = "/dev/ttyUSB0");
        void readGPSData(double& lat, double& lng, double& alt);
    };
}

#endif // HANDSFREE_RTK_UM982_USB_SDK_H