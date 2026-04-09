#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/WheelState.h>
#include <rk3588/uart_driver.h>
// 电机控制协议定义
// 帧格式：[0xAA][0x55][0x01][L][R][C][XOR][0x55][0xAA]
// 其中：L-左轮速度(4字节float)，R-右轮速度(4字节float)，C-刀盘速度(4字节float)
class MotorControl {
public:
    MotorControl(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
        // 读取参数
        pnh_.param<std::string>("serial_port", serial_port_, "/dev/ttyS2");
        pnh_.param<int>("baudrate", baudrate_, 115200);
        pnh_.param<double>("wheel_separation", wheel_separation_, 0.5);
        pnh_.param<double>("wheel_radius", wheel_radius_, 0.15);
        
        // 初始化串口
        uart_ = new rk3588_sdk::UARTDriver(serial_port_, baudrate_);
        if (!uart_->openPort()) {
            ROS_ERROR("Failed to open serial port. Exiting...");
            exit(1);
        }
        
        // 初始化订阅者
        cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>(
            "/cmd_vel", 10, &MotorControl::cmd_vel_callback, this);
        
        joy_sub_ = nh_.subscribe<sensor_msgs/Joy>(
            "/joy", 10, &MotorControl::joy_callback, this);
        
        // 初始化发布者
        wheel_state_pub_ = nh_.advertise<sensor_msgs::WheelState>(
            "/wheel_speed", 10);
        
        // 启动轮速读取线程
        wheel_speed_thread_ = std::thread(&MotorControl::read_wheel_speed, this);
    }
    
    ~MotorControl() {
        if (uart_) {
            uart_->closePort();
            delete uart_;
        }
        if (wheel_speed_thread_.joinable()) {
            wheel_speed_thread_.join();
        }
    }
    
private:
    void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg) {
        // 运动学转换：将Twist转换为轮速
        double linear_x = msg->linear.x;
        double angular_z = msg->angular.z;
        
        // 差速驱动运动学
        double v_left = linear_x - angular_z * (wheel_separation_ / 2.0);
        double v_right = linear_x + angular_z * (wheel_separation_ / 2.0);
        
        // 转换为电机速度（假设单位一致）
        float motor_left = static_cast<float>(v_left);
        float motor_right = static_cast<float>(v_right);
        float motor_cutter = 0.0;  // 刀盘速度
        
        // 创建控制包
        uint8_t packet[18] = {0xAA, 0x55, 0x01};
        
        // 转换为字节流
        memcpy(packet + 3, &motor_left, 4);
        memcpy(packet + 7, &motor_right, 4);
        memcpy(packet + 11, &motor_cutter, 4);
        
        // 计算校验和（前15字节异或）
        uint8_t xor_check = 0;
        for (int i = 0; i < 15; i++) {
            xor_check ^= packet[i];
        }
        packet[15] = xor_check;
        
        // 添加帧尾
        packet[16] = 0x55;
        packet[17] = 0xAA;
        
        // 发送数据
        uart_->sendData(packet, 18);
    }
    
    void joy_callback(const sensor_msgs::Joy::ConstPtr& msg) {
        // 手柄控制（示例）
        if (msg->buttons.size() > 0) {
            // 可以实现手柄控制逻辑
        }
    }
    
    void read_wheel_speed() {
        // 持续读取轮速数据
        while (ros::ok()) {
            // 读取数据（根据协议实现）
            uint8_t buffer[4];  // 假设轮速数据为4字节
            
            int bytes_read = uart_->readData(buffer, 4, 100);  // 100ms超时
            
            if (bytes_read == 4) {
                // 解析轮速（示例：左前轮速度）
                float wheel_speed;
                memcpy(&wheel_speed, buffer, 4);
                
                // 创建并发布轮速消息
                sensor_msgs::WheelState wheel_state;
                wheel_state.header.stamp = ros::Time::now();
                wheel_state.speed = wheel_speed;
                wheel_state.header.frame_id = "front_left_wheel";
                
                wheel_state_pub_.publish(wheel_state);
            }
        }
    }
    
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber joy_sub_;
    ros::Publisher wheel_state_pub_;
    
    rk3588_sdk::UARTDriver* uart_ = nullptr;
    std::thread wheel_speed_thread_;
    
    std::string serial_port_;
    int baudrate_;
    double wheel_separation_;
    double wheel_radius_;
};
int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_control_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    MotorControl motor_control(nh, pnh);
    
    ros::spin();
    
    return 0;
}