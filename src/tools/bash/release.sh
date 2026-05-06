#!/bin/bash
set -euo pipefail

# ============================ 常量定义（中文注释）============================
# 脚本所在目录
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
# 工程根目录：lawnmower_ws
PROJECT_ROOT="${SCRIPT_DIR}/../../.."
# 发布输出目录
OUTPUT_DIR="${PROJECT_ROOT}/release"
# 生成的可执行命令工具
OUT_FILE="${OUTPUT_DIR}/lawnmower_release.out"
# 纯英文日志文件
LOG_FILE="${OUTPUT_DIR}/release.log"

# ============================ 日志函数（纯英文输出）============================
# 初始化日志目录（先创建，避免报错）
init_log() {
    mkdir -p "${OUTPUT_DIR}"
    > "${LOG_FILE}"
}

# 信息日志
info_log() {
    local msg="[INFO] $(date '+%Y-%m-%d %H:%M:%S'): $1"
    echo -e "${msg}" | tee -a "${LOG_FILE}"
}

# 错误日志
error_log() {
    local msg="[ERROR] $(date '+%Y-%m-%d %H:%M:%S'): $1"
    echo -e "${msg}" | tee -a "${LOG_FILE}"
    exit 1
}

# ============================ 生成全英文分级交互式工具 ============================
generate_command_tool() {
    info_log "Generating English interactive command tool..."
    cat > "${OUT_FILE}" << 'EOF'
#!/bin/bash
# ==============================================================
# Lawnmower ROS1 Noetic Command Tool
# Full compliant with Cmdlist.md | No Source Code | Relative Path
# ==============================================================

# Auto switch to project root directory (fix path error)
PROJECT_ROOT=$(cd "$(dirname "$0")/.." &>/dev/null && pwd)
cd "${PROJECT_ROOT}" || exit 1

# Auto source ROS environment (fix ROS package error)
source devel/setup.bash --extend 2>/dev/null

# Clear screen
clear

# ============================ Main Menu (100% match Cmdlist.md) ============================
show_main_menu() {
    clear
    echo "====================================================="
    echo "        Lawnmower ROS1 Noetic Command Tool"
    echo "====================================================="
    echo "1. Basic Environment Operations"
    echo "2. Sensor Node Launch"
    echo "3. Data Validation & Monitor"
    echo "4. Rosbag Record & Playback"
    echo "5. Python Debug Tools"
    echo "0. Exit"
    echo "====================================================="
    read -p "Select Menu [0-5]: " MAIN_CHOICE
}

# ============================ Sub Menu 1: Basic Environment Operations ============================
menu_env() {
    while true; do
        clear
        echo "====================================================="
        echo "1. Basic Environment Operations"
        echo "====================================================="
        echo "1.1 ROS Core Management"
        echo "  1) roscore"
        echo "  2) ./run.sh start_core"
        echo "1.2 Project Build"
        echo "  3) ./run.sh build"
        echo "  4) ./run.sh build -p rk3588_sdk"
        echo "1.3 Project Clean"
        echo "  5) ./run.sh clean"
        echo "  6) ./run.sh clean_source"
        echo "1.4 Project Release"
        echo "  7) ./run.sh release -v v1.0.0"
        echo "  8) ./run.sh release -v v1.0.0 --self-extractor"
        echo "0. Back to Main Menu"
        echo "====================================================="
        read -p "Select Command [0-8]: " CHOICE

        case $CHOICE in
            1) roscore ;;
            2) ./run.sh start_core ;;
            3) ./run.sh build ;;
            4) ./run.sh build -p rk3588_sdk ;;
            5) ./run.sh clean ;;
            6) ./run.sh clean_source ;;
            7) ./run.sh release -v v1.0.0 ;;
            8) ./run.sh release -v v1.0.0 --self-extractor ;;
            0) break ;;
            *) echo "Invalid Input!" ;;
        esac
        echo -e "\nPress Enter to continue..."
        read
    done
}

# ============================ Sub Menu 2: Sensor Node Launch ============================
menu_sensor() {
    while true; do
        clear
        echo "====================================================="
        echo "2. Sensor Node Launch"
        echo "====================================================="
        echo "2.1 Camera (Orbbec Gemini 335)"
        echo "  1) rosrun orbbec_camera list_devices_node"
        echo "  2) ./run.sh run_cmdlist roslaunch orbbec_camera gemini345.launch"
        echo "2.2 LiDAR (Hesai JT128)"
        echo "  3) ./run.sh run_cmdlist roslaunch hesai_lidar start.launch"
        echo "  4) ./run.sh view_topic_hz /hesai_jt128/pointcloud_raw"
        echo "2.3 RTK (UM982)"
        echo "  5) ./run.sh run_cmdlist roslaunch handsfree_rtk handsfree_rtk.launch"
        echo "  6) ./run.sh view_topic /um982/nmea_raw -n 5"
        echo "2.4 Combined Launch"
        echo "  7) roslaunch sensor_data sensors.launch"
        echo "  8) roslaunch motor_control motor_control.launch"
        echo "  9) roslaunch launch all_devices.launch"
        echo "0. Back to Main Menu"
        echo "====================================================="
        read -p "Select Command [0-9]: " CHOICE

        case $CHOICE in
            1) rosrun orbbec_camera list_devices_node ;;
            2) ./run.sh run_cmdlist roslaunch orbbec_camera gemini345.launch ;;
            3) ./run.sh run_cmdlist roslaunch hesai_lidar start.launch ;;
            4) ./run.sh view_topic_hz /hesai_jt128/pointcloud_raw ;;
            5) ./run.sh run_cmdlist roslaunch handsfree_rtk handsfree_rtk.launch ;;
            6) ./run.sh view_topic /um982/nmea_raw -n 5 ;;
            7) roslaunch sensor_data sensors.launch ;;
            8) roslaunch motor_control motor_control.launch ;;
            9) roslaunch launch all_devices.launch ;;
            0) break ;;
            *) echo "Invalid Input!" ;;
        esac
        echo -e "\nPress Enter to continue..."
        read
    done
}

# ============================ Sub Menu 3: Data Validation & Monitor ============================
menu_monitor() {
    while true; do
        clear
        echo "====================================================="
        echo "3. Data Validation & Monitor"
        echo "====================================================="
        echo "3.1 Topic List Validation"
        echo "  1) rostopic list | grep lawnmower"
        echo "3.2 Topic Data Monitor"
        echo "  2) ./run.sh view_topic_hz /lawnmower/pointcloud"
        echo "  3) ./run.sh view_topic /lawnmower/imu"
        echo "  4) ./run.sh view_topic /lawnmower/gps/fix -n 10"
        echo "  5) ./run.sh view_topic /lawnmower/odom"
        echo "3.3 System Status Check"
        echo "  6) ./run.sh view_topics"
        echo "  7) ./run.sh view_nodes"
        echo "  8) ./run.sh view_services"
        echo "  9) ./run.sh view_params"
        echo "0. Back to Main Menu"
        echo "====================================================="
        read -p "Select Command [0-9]: " CHOICE

        case $CHOICE in
            1) rostopic list | grep lawnmower ;;
            2) ./run.sh view_topic_hz /lawnmower/pointcloud ;;
            3) ./run.sh view_topic /lawnmower/imu ;;
            4) ./run.sh view_topic /lawnmower/gps/fix -n 10 ;;
            5) ./run.sh view_topic /lawnmower/odom ;;
            6) ./run.sh view_topics ;;
            7) ./run.sh view_nodes ;;
            8) ./run.sh view_services ;;
            9) ./run.sh view_params ;;
            0) break ;;
            *) echo "Invalid Input!" ;;
        esac
        echo -e "\nPress Enter to continue..."
        read
    done
}

# ============================ Sub Menu 4: Rosbag Record & Playback ============================
menu_bag() {
    while true; do
        clear
        echo "====================================================="
        echo "4. Rosbag Record & Playback"
        echo "====================================================="
        echo "  1) ./run.sh record_bag -a -O all_devices_raw.bag --duration=60"
        echo "  2) ./run.sh record_bag /lawnmower/pointcloud /lawnmower/imu -O sensor_data.bag"
        echo "  3) ./run.sh play_bag all_devices_raw.bag -r 2 --loop"
        echo "0. Back to Main Menu"
        echo "====================================================="
        read -p "Select Command [0-3]: " CHOICE

        case $CHOICE in
            1) ./run.sh record_bag -a -O all_devices_raw.bag --duration=60 ;;
            2) ./run.sh record_bag /lawnmower/pointcloud /lawnmower/imu -O sensor_data.bag ;;
            3) ./run.sh play_bag all_devices_raw.bag -r 2 --loop ;;
            0) break ;;
            *) echo "Invalid Input!" ;;
        esac
        echo -e "\nPress Enter to continue..."
        read
    done
}

# ============================ Sub Menu 5: Python Debug Tools ============================
menu_python() {
    while true; do
        clear
        echo "====================================================="
        echo "5. Python Debug Tools (src/tools/scripts/)"
        echo "====================================================="
        echo "5.1 Independent Tools"
        echo "  1) python3 src/tools/scripts/camera_tool.py --help"
        echo "  2) python3 src/tools/scripts/lidar_tool.py --help"
        echo "  3) python3 src/tools/scripts/rtk_tool.py --help"
        echo "5.2 Unified Debug Tool"
        echo "  4) python3 src/tools/scripts/unified_tool.py --help"
        echo "  5) python3 src/tools/scripts/unified_tool.py system start_all"
        echo "  6) python3 src/tools/scripts/unified_tool.py system stop_all"
        echo "  7) python3 src/tools/scripts/unified_tool.py system status"
        echo "  8) python3 src/tools/scripts/unified_tool.py camera get_status"
        echo "  9) python3 src/tools/scripts/unified_tool.py lidar record_pcl -d 10 -o lidar_data.bag"
        echo " 10) python3 src/tools/scripts/unified_tool.py rtk config_ntrip"
        echo "0. Back to Main Menu"
        echo "====================================================="
        read -p "Select Command [0-10]: " CHOICE

        case $CHOICE in
            1) python3 src/tools/scripts/camera_tool.py --help ;;
            2) python3 src/tools/scripts/lidar_tool.py --help ;;
            3) python3 src/tools/scripts/rtk_tool.py --help ;;
            4) python3 src/tools/scripts/unified_tool.py --help ;;
            5) python3 src/tools/scripts/unified_tool.py system start_all ;;
            6) python3 src/tools/scripts/unified_tool.py system stop_all ;;
            7) python3 src/tools/scripts/unified_tool.py system status ;;
            8) python3 src/tools/scripts/unified_tool.py camera get_status ;;
            9) python3 src/tools/scripts/unified_tool.py lidar record_pcl -d 10 -o lidar_data.bag ;;
            10) python3 src/tools/scripts/unified_tool.py rtk config_ntrip ;;
            0) break ;;
            *) echo "Invalid Input!" ;;
        esac
        echo -e "\nPress Enter to continue..."
        read
    done
}

# ============================ Main Loop ============================
while true; do
    show_main_menu
    case $MAIN_CHOICE in
        1) menu_env ;;
        2) menu_sensor ;;
        3) menu_monitor ;;
        4) menu_bag ;;
        5) menu_python ;;
        0) echo "Exiting tool..."; exit 0 ;;
        *) echo "Invalid Input!" ;;
    esac
done
EOF

    # 赋予执行权限
    chmod +x "${OUT_FILE}"
    info_log "Command tool generated successfully: ${OUT_FILE}"
}

# ============================ Main Process ============================
main() {
    init_log
    generate_command_tool
    info_log "Release process completed successfully!"
}

main "$@"