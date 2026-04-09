#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <rk3588/serial_port.h>
// 串口配置
const std::string SERIAL_PORT = "/dev/ttyS2";  // RK3588的串口
const int BAUD_RATE = 115200;
// 协议定义
const uint8_t START_BYTE1 = 0xAA;
const uint8_t START_BYTE2 = 0x55;
const uint8_t END_BYTE1 = 0x55;
const uint8_t END_BYTE2 = 0xAA;
const int FRAME_LENGTH = 18;
class MotorControl {
public:
    MotorControl(ros::NodeHandle& nh) : nh_(nh), serial_port_(new rk3588::SerialPort) {
        // 订阅/cmd_vel话题
        cmd_vel_sub_ = nh.subscribe<geometry_msgs/Twist>(
            "/cmd_vel", 10, &MotorControl::cmd_vel_callback, this);
        
        // 发布轮速信息
        wheel_speed_pub_ = nh.advertise<std_msgs::Float32MultiArray>(
            "/wheel_speed", 10);
        
        // 初始化串口
        if (!serial_port_->open(SERIAL_PORT, BAUD_RATE)) {
            ROS_FATAL("Failed to open serial port %s", SERIAL_PORT.c_str());
            ros::shutdown();
        }
        
        // 启动读取轮速线程
        running_ = true;
        read_thread_ = std::thread(&MotorControl::read_wheel_speed, this);
    }
    ~MotorControl() {
        running_ = false;
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
        if (serial_port_->is_open()) {
            serial_port_->close();
        }
    }
private:
    ros::NodeHandle nh_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher wheel_speed_pub_;
    std::unique_ptr<rk3588::SerialPort> serial_port_;
    std::thread read_thread_;
    bool running_;
    
    // 电机控制参数
    float left_wheel_speed_ = 0.0;
    float right_wheel_speed_ = 0.0;
    float cutter_speed_ = 0.0;
    
    void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg) {
        // 速度控制（简化的差速控制）
        float linear_x = msg->linear.x;
        float angular_z = msg->angular.z;
        
        // 计算左右轮速度（单位：m/s）
        left_wheel_speed_ = linear_x - angular_z * 0.3;  // 假设轮距0.6m
        right_wheel_speed_ = linear_x + angular_z * 0.3;
        
        // 割草刀速度（示例值）
        cutter_speed_ = 3000.0;  // RPM
        
        // 发送控制指令
        send_control_command();
    }
    void send_control_command() {
        try {
            // 创建控制帧
            std::vector<uint8_t> frame(FRAME_LENGTH, 0);
            
            // 帧头
            frame[0] = START_BYTE1;
            frame[1] = START_BYTE2;
            
            // 数据（左轮、右轮、刀盘速度）
            float_to_bytes(left_wheel_speed_, &frame[2]);
            float_to_bytes(right_wheel_speed_, &frame[6]);
            float_to_bytes(cutter_speed_, &frame[10]);
            
            // 计算校验和
            uint8_t xor_check = 0;
            for (int i = 0; i < 15; ++i) {
                xor_check ^= frame[i];
            }
            frame[15] = xor_check;
            
            // 帧尾
            frame[16] = END_BYTE1;
            frame[17] = END_BYTE2;
            
            // 发送数据
            serial_port_->write(frame);
            
            // 打印调试信息
            ROS_DEBUG("Sent control command: L=%.2f m/s, R=%.2f m/s, C=%.0f RPM",
                      left_wheel_speed_, right_wheel_speed_, cutter_speed_);
        } catch (const std::exception& e) {