#motor_control:
最终串口协议（纯 UART，下位机直接对接）
帧长度：18 字节
[0xAA][0x55][0x01][L][R][C][XOR][0x55][0xAA]
  2B    1B    4*3=12B   1B      2B
左轮 L：4 字节 float
右轮 R：4 字节 float
刀盘 C：4 字节 float
XOR 校验：前 15 字节异或
下位机按此格式发回轮速，RK3588 自动解析并发布 /wheel_speed。

编译与运行步骤（RK3588 Ubuntu20.04 Noetic）
1. 安装依赖
bash
运行
sudo apt install ros-noetic-serial
sudo usermod -aG dialout $USER
注销重登录
2. 编译
bash
运行
cd ~/catkin_ws
catkin_make
source devel/setup.bash
3. 运行
bash
运行
roslaunch motor_control motor_control.launch
4. 测试控制
bash
运行
rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.3}, angular: {z: 0.2, y: 3.0}}' -r 10
5. 查看轮速回传
bash
运行
rostopic echo /wheel_speed

