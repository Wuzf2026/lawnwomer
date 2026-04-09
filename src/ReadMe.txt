1、对外开放API接口与参数说明
    1.1 ROS服务API（核心控制接口）
        服务名称	                            服务类型	             核心功能	                    入参说明	                                                出参说明
        /lawnmower/api/get_sensor_status	GetSensorStatus	    查询传感器连接与运行状态	    sensor_type：传感器类型（orbbec/hesai/um982）	                is_connected：是否连接；status_detail：详细状态；last_update：最后更新时间戳
        /lawnmower/api/set_sensor_param	    SetSensorParam	    运行时修改传感器参数	       sensor_type：传感器类型；param_key：参数名；param_value：参数值	 success：是否成功；message：执行结果
        /lawnmower/api/calibrate_sensor	    CalibrateSensor	    触发传感器校准	               sensor_type：传感器类型；calib_type：校准类型	                success：是否成功；calib_error：校准误差；message：校准结果
    1.2 ROS话题API（数据输出接口）
        话题名称	                        消息类型	        输出内容	            发布频率
        /lawnmower/orbbec/stereo_image	StereoImage	    双目左右目图像+相机基线	        30Hz
        /lawnmower/hesai/lidar_data	    LidarData	    激光点云三维坐标+强度	        10Hz
        /lawnmower/um982/rtk_data	    RTKData	        经纬度、海拔、航向、定位状态	10Hz
    1.3 可配置参数全表
        Orbbec Gemini335可配置参数
        参数名	            类型	    默认值	        取值范围	            说明
        frame_rate	        int	        30	        1~30	                图像帧率
        auto_exposure	    bool	    true	    true/false	            自动曝光开关
        exposure_value	    int	        100	        1~1000	                手动曝光值
        resolution	        string	1280x720	640x480/1280x720/1920x1080	图像分辨率
        Hesai JT128 可配置参数
        参数名	            类型	    默认值	        取值范围	            说明
        scan_frequency	    int	        10	        5~20	                扫描频率(Hz)
        return_mode	        string	    dual	    single/dual	            回波模式
        filter_distance	    float	    0.5	        0.1~100	                最小测距过滤阈值(m)
        UM982 RTK 可配置参数
        参数名	            类型	    默认值	        取值范围	            说明
        update_rate	        int	        10	         1~20	                定位更新频率(Hz)
        rtk_mode	        string	    rtcm3	     rtcm2/rtcm3	        RTK数据协议
        base_station_ip	    string	[192.168.2.10](192.168.2.10)合法IP地址	 基站IP
        base_station_port	int	        8080	    1~65535	                基站通信端口

2、工程编译与运行步骤
    2.1 环境依赖安装
        # ROS1 Noetic 基础依赖
        sudo apt install ros-noetic-roscpp ros-noetic-rospy ros-noetic-std-msgs ros-noetic-sensor-msgs ros-noetic-cv-bridge ros-noetic-serial
        # 系统基础依赖
        sudo apt install libusb-1.0-0-dev libyaml-cpp-dev libopencv-dev python3-rospy python3-yaml
    2.2 工程编译
        # 进入工作空间
        cd ~/lawnwomer_ws
        # 清理旧编译文件
        catkin clean
        # Release模式编译（适配RK3588平台）
        catkin_make -DCMAKE_BUILD_TYPE=Release
        # 配置环境变量
        source devel/setup.bash
        echo "source ~/lawnwomer_ws/devel/setup.bash" >> ~/.bashrc
    2.3 运行指令
        # 1. 全功能启动（传感器+核心逻辑+API服务+RViz可视化）
        roslaunch launch lawnmower_full.launch
        # 2. 仅启动传感器驱动（调试用）
        roslaunch launch sensor_only.launch
        # 3. 启动API调试模式
        roslaunch launch api_debug.launch
        # 4. 运行Python调试工具
        rosrun tools sensor_debugger.py --debug
        rosrun tools param_configurator.py
        rosrun tools api_client.py

3、RK3588平台驱动适配说明
    3.1 USB通信适配（双目相机+RTK模块）
        1.底层调用rk35588_sdk原厂USB驱动，代码中通过libusb-1.0完成对接，无需额外修改
        2.设备端口默认映射：双目相机/dev/ttyUSB0，RTK模块/dev/ttyUSB1
        3.权限配置：需将当前用户加入dialout组，避免串口权限不足
        Bash
        sudo usermod -aG dialout $USER
    3.2 ETH通信适配（激光雷达）
        1.调用rk35588_sdk原厂ETH网卡驱动，通过UDP协议直连雷达
        2.需提前配置RK3588主机静态IP：192.168.1.100，与雷达IP192.168.1.200保持同网段
        3.防火墙配置：需开放UDP端口9347，避免雷达数据被拦截
        sudo ufw allow 9347/udp
    3.3 原厂SDK集成
        1.将Orbbec、Hesai、UM982的官方SDK动态库（.so文件）放入RK3588系统的/usr/lib/目录
        2.将SDK头文件放入/usr/include/目录，确保编译时可正常链接
        3.若SDK为RK3588专属版本，需在CMakeLists.txt中补充对应架构的链接路径