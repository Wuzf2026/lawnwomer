#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "rk3588_sdk/gpio_control.h"
#include "rk3588_sdk/uart_driver.h"

using namespace rk3588_sdk;

class RK3588SDKNode {
public:
    RK3588SDKNode() : gpio_(18), uart_("/dev/ttyS2", 115200) {
        ros::NodeHandle nh_private("~");
        
        // 读取参数
        nh_private.param<int>("gpio_pin", gpio_pin_, 18);
        nh_private.param<std::string>("uart_dev", uart_dev_, "/dev/ttyS2");
        nh_private.param<int>("uart_baud", uart_baud_, 115200);

        // 初始化GPIO
        if (gpio_.exportGPIO()) {
            gpio_.setDirection("out");
            ROS_INFO("GPIO %d initialized", gpio_pin_);
        } else {
            ROS_ERROR("Failed to initialize GPIO %d", gpio_pin_);
        }

        // 初始化UART
        if (uart_.openPort()) {
            ROS_INFO("UART %s opened (baud: %d)", uart_dev_.c_str(), uart_baud_);
        } else {
            ROS_ERROR("Failed to open UART %s", uart_dev_.c_str());
        }

        // 发布/订阅
        gpio_pub_ = nh_.advertise<std_msgs::Int32>("gpio_value", 10);
        uart_pub_ = nh_.advertise<std_msgs::String>("uart_data", 10);
        gpio_sub_ = nh_.subscribe("set_gpio", 10, &RK3588SDKNode::gpioCallback, this);

        // 定时器
        timer_ = nh_.createTimer(ros::Duration(0.1), &RK3588SDKNode::timerCallback, this);
    }

    ~RK3588SDKNode() {
        gpio_.unexportGPIO();
        uart_.closePort();
    }

    void gpioCallback(const std_msgs::Bool::ConstPtr& msg) {
        gpio_.setValue(msg->data ? 1 : 0);
        ROS_INFO("Set GPIO %d to %d", gpio_pin_, msg->data ? 1 : 0);
    }

    void timerCallback(const ros::TimerEvent& e) {
        // 发布GPIO值
        std_msgs::Int32 gpio_msg;
        gpio_msg.data = gpio_.getValue();
        gpio_pub_.publish(gpio_msg);

        // 读取UART数据
        uint8_t buffer[128];
        int len = uart_.readData(buffer, 128);
        if (len > 0) {
            std_msgs::String uart_msg;
            uart_msg.data = std::string((char*)buffer, len);
            uart_pub_.publish(uart_msg);
        }
    }

private:
    ros::NodeHandle nh_;
    GPIOControl gpio_;
    UARTDriver uart_;
    int gpio_pin_;
    std::string uart_dev_;
    int uart_baud_;

    ros::Publisher gpio_pub_;
    ros::Publisher uart_pub_;
    ros::Subscriber gpio_sub_;
    ros::Timer timer_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rk3588_sdk_node");
    RK3588SDKNode node;
    ros::spin();
    return 0;
}