#!/bin/bash
set -uo pipefail

# ===================== 全局配置（纯相对路径，自解压环境适配） ======================
# 脚本所在目录（兼容原始目录/自解压临时目录）
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
# 工作空间根路径
CATKIN_WS_PATH="${SCRIPT_DIR}/../../.."
ROS_SETUP_FILE="${CATKIN_WS_PATH}/devel/setup.bash"
CMDLIST_PATH="${SCRIPT_DIR}/Cmdlist.md"
DEFAULT_ROS_DISTRO="noetic"
BAG_DIR="${CATKIN_WS_PATH}/bags"
RELEASE_DIR="${CATKIN_WS_PATH}/release"
DEFAULT_VERSION="v1.0.0"
SCRIPTS_DIR="${SCRIPT_DIR}/../scripts"
RELEASE_SCRIPT="${SCRIPT_DIR}/release.sh"

# 颜色输出（统一风格）
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# ===================== 工具函数（增强鲁棒性） ======================
# 信息日志（英文输出）
log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
# 成功日志
log_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
# 警告日志
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
# 错误日志（退出）
log_error() { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

# 检查ROS环境（中文注释）
# 功能：检查ROS安装状态、加载发行版、验证工作空间编译产物
check_ros_env() {
    if ! command -v roscore &>/dev/null; then
        log_error "ROS ${DEFAULT_ROS_DISTRO} environment not detected! Please install first: http://wiki.ros.org/noetic/Installation"
    fi

    if [ -z "${ROS_DISTRO:-}" ]; then
        log_warn "ROS_DISTRO not set, auto load ${DEFAULT_ROS_DISTRO}"
        source "/opt/ros/${DEFAULT_ROS_DISTRO}/setup.bash" || log_error "Failed to load ROS environment"
    fi

    if [ ! -f "${ROS_SETUP_FILE}" ]; then
        log_error "ROS workspace build products not detected! Please run first: ./run.sh build"
    fi

    source "${ROS_SETUP_FILE}" || log_error "Failed to load workspace environment"
    log_success "ROS environment loaded | Distro: ${ROS_DISTRO} | Workspace: ${CATKIN_WS_PATH}"
}

# 检查ROS工具完整性（中文注释）
check_ros_tools() {
    local tools=(
        "rostopic" "rosnode" "rosservice" "rosparam"
        "rosbag" "roscore" "roslaunch" "rosrun"
        "catkin_make" "rosmsg" "rossrv" "roswtf"
    )
    local missing_tools=()
    for tool in "${tools[@]}"; do
        if ! command -v "${tool}" &>/dev/null; then
            missing_tools+=("${tool}")
        fi
    done
    if [ ${#missing_tools[@]} -gt 0 ]; then
        log_warn "The following ROS tools are not installed, some functions are limited: ${missing_tools[*]}"
    else
        log_success "All ROS core tools are installed"
    fi
}

# 验证Cmdlist.md完整性（中文注释）
check_cmdlist() {
    if [ ! -f "${CMDLIST_PATH}" ]; then
        log_error "Debug command list missing: ${CMDLIST_PATH}"
    fi
    local cmd_count=$(grep "^- Command:" "${CMDLIST_PATH}" | wc -l)
    if [ "${cmd_count}" -eq 0 ]; then
        log_warn "No debug commands detected in Cmdlist.md"
    else
        log_success "Cmdlist.md verified | Contains ${cmd_count} debug commands"
    fi
}

# 解析Cmdlist.md中的调试命令（中文注释）
parse_cmdlist() {
    local cmd_name="$1"
    if [ -z "${cmd_name}" ]; then
        log_error "Please specify debug command name! Example: ./run.sh debug run_core_node"
    fi
    local cmd_content=$(grep -A1 "^- Command: ${cmd_name}" "${CMDLIST_PATH}" | grep "^  CommandContent:" | sed 's/  CommandContent: //')
    if [ -z "${cmd_content}" ]; then
        log_error "Debug command '${cmd_name}' not found in Cmdlist.md"
    fi
    echo "${cmd_content}"
}

# ===================== 1. Build Commands ======================
build_all() {
    log_info "Start full build of workspace (RK3588 single thread optimized)"
    cd "${CATKIN_WS_PATH}" || log_error "Failed to enter workspace"
    catkin_make clean >/dev/null 2>&1
    catkin_make -DCMAKE_BUILD_TYPE=Release -j1 || log_error "Full build failed"
    log_success "Full build completed"
}

build_pkg() {
    local pkg_name="$1"
    if [ -z "${pkg_name}" ]; then
        log_error "Please specify the package name! Example: ./run.sh build-pkg lawnmower_core"
    fi
    log_info "Start building single package: ${pkg_name}"
    cd "${CATKIN_WS_PATH}" || log_error "Failed to enter workspace"
    catkin_make -j1 --only-pkg-with-deps "${pkg_name}" || log_error "Failed to build ${pkg_name}"
    log_success "Package ${pkg_name} build completed"
}

build_incremental() {
    log_info "Start incremental build of workspace"
    cd "${CATKIN_WS_PATH}" || log_error "Failed to enter workspace"
    catkin_make -DCMAKE_BUILD_TYPE=Release -j1 || log_error "Incremental build failed"
    log_success "Incremental build completed"
}

clean_build() {
    log_info "Cleaning workspace build products"
    cd "${CATKIN_WS_PATH}" || log_error "Failed to enter workspace"
    catkin_make clean || log_error "Clean failed"
    rm -rf build devel install 2>/dev/null
    log_success "Build products cleaned up"
}

# ===================== 2. ROS Core Commands ======================
start_roscore() {
    check_ros_env
    if pgrep -x "roscore" &>/dev/null; then
        log_warn "roscore is already running (PID: $(pgrep -x roscore))"
        return 0
    fi
    log_info "Starting roscore (Log: /tmp/roscore.log)"
    roscore > /tmp/roscore.log 2>&1 &
    sleep 2
    if pgrep -x "roscore" &>/dev/null; then
        log_success "roscore started successfully (PID: $(pgrep -x roscore))"
    else
        log_error "roscore failed to start! Check log: /tmp/roscore.log"
    fi
}

stop_roscore() {
    if ! pgrep -x "roscore" &>/dev/null; then
        log_warn "roscore is not running"
        return 0
    fi
    log_info "Stopping roscore (PID: $(pgrep -x roscore))"
    pkill -x roscore || log_error "Failed to stop roscore"
    log_success "roscore stopped"
}

restart_roscore() {
    log_info "Restarting roscore..."
    stop_roscore
    sleep 1
    start_roscore
    log_success "roscore restarted successfully"
}

ros_env_info() {
    check_ros_env
    log_info "===== ROS Environment Info ====="
    echo -e "ROS_DISTRO: ${ROS_DISTRO}"
    echo -e "ROS_ROOT: ${ROS_ROOT}"
    echo -e "ROS_PACKAGE_PATH: ${ROS_PACKAGE_PATH}"
    echo -e "CATKIN_WS: ${CATKIN_WS_PATH}"
    echo -e "ROS_MASTER_URI: ${ROS_MASTER_URI:-unset}"
    echo -e "================================"
}

ros_check_health() {
    check_ros_env
    log_info "Checking ROS system health with roswtf..."
    roswtf || log_warn "roswtf detected potential issues (non-critical)"
    log_success "ROS health check completed"
}

# ===================== 3. Topic Commands ======================
list_topics() {
    check_ros_env
    log_info "List all ROS Topics (with type info)"
    rostopic list -v || log_error "Failed to get Topic list"
}

topic_hz() {
    check_ros_env
    local topic="$1"
    if [ -z "${topic}" ]; then
        log_error "Please specify the Topic name! Example: ./run.sh topic-hz /cmd_vel"
    fi
    log_info "Monitoring Topic frequency: ${topic}"
    rostopic hz "${topic}" || log_error "Failed to monitor Topic frequency"
}

topic_echo() {
    check_ros_env
    local topic="$1"
    if [ -z "${topic}" ]; then
        log_error "Please specify the Topic name! Example: ./run.sh topic-echo /odom"
    fi
    log_info "Printing Topic content: ${topic}"
    rostopic echo "${topic}" || log_error "Failed to print Topic content"
}

topic_pub() {
    check_ros_env
    local topic="$1" type="$2" msg="$3"
    if [ -z "${topic}" ] || [ -z "${type}" ] || [ -z "${msg}" ]; then
        log_error "Missing parameters! Example: ./run.sh topic-pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'"
    fi
    log_info "Publish message to Topic ${topic}: ${msg}"
    rostopic pub -1 "${topic}" "${type}" "${msg}" || log_error "Failed to publish Topic"
}

topic_type() {
    check_ros_env
    local topic="$1"
    if [ -z "${topic}" ]; then
        log_error "Please specify the Topic name! Example: ./run.sh topic-type /cmd_vel"
    fi
    log_info "Getting type of Topic: ${topic}"
    rostopic type "${topic}" || log_error "Failed to get Topic type"
}

topic_record() {
    check_ros_env
    local topic="$1" bag_name="topic_record_$(date +%Y%m%d_%H%M%S).bag"
    if [ -z "${topic}" ]; then
        log_error "Please specify the Topic name! Example: ./run.sh topic-record /odom"
    fi
    mkdir -p "${BAG_DIR}"
    log_info "Recording Topic ${topic} to ${BAG_DIR}/${bag_name}"
    rosbag record -O "${BAG_DIR}/${bag_name}" "${topic}" || log_error "Failed to record Topic"
}

# ===================== 4. Node Commands ======================
list_nodes() {
    check_ros_env
    log_info "List all ROS Nodes"
    rosnode list || log_error "Failed to get Node list"
}

node_info() {
    check_ros_env
    local node="$1"
    if [ -z "${node}" ]; then
        log_error "Please specify the Node name! Example: ./run.sh node-info /lawnmower_node"
    fi
    log_info "View Node details: ${node}"
    rosnode info "${node}" || log_error "Failed to get Node details"
}

kill_node() {
    check_ros_env
    local node="$1"
    if [ -z "${node}" ]; then
        log_error "Please specify the Node name! Example: ./run.sh node-kill /lawnmower_node"
    fi
    log_info "Stopping Node: ${node}"
    rosnode kill "${node}" || log_error "Failed to stop Node"
    log_success "Node ${node} stopped"
}

node_ping() {
    check_ros_env
    local node="$1"
    if [ -z "${node}" ]; then
        log_error "Please specify the Node name! Example: ./run.sh node-ping /lawnmower_node"
    fi
    log_info "Pinging Node: ${node}"
    rosnode ping "${node}" || log_error "Node ${node} is offline"
}

node_log() {
    check_ros_env
    local node="$1"
    if [ -z "${node}" ]; then
        log_error "Please specify the Node name! Example: ./run.sh node-log /lawnmower_node"
    fi
    log_info "Viewing log for Node: ${node}"
    rosnode log "${node}" || log_error "Failed to get Node log"
}

# ===================== 5. Service Commands ======================
list_services() {
    check_ros_env
    log_info "List all ROS Services"
    rosservice list || log_error "Failed to get Service list"
}

call_service() {
    check_ros_env
    local service="$1" args="${2:-}"
    if [ -z "${service}" ]; then
        log_error "Please specify the Service name! Example: ./run.sh call-service /reset_odom"
    fi
    log_info "Call Service: ${service} (Args: ${args})"
    rosservice call "${service}" "${args}" || log_error "Failed to call Service"
}

service_type() {
    check_ros_env
    local service="$1"
    if [ -z "${service}" ]; then
        log_error "Please specify the Service name! Example: ./run.sh service-type /reset_odom"
    fi
    log_info "Getting type of Service: ${service}"
    rosservice type "${service}" || log_error "Failed to get Service type"
}

service_args() {
    check_ros_env
    local service="$1"
    if [ -z "${service}" ]; then
        log_error "Please specify the Service name! Example: ./run.sh service-args /reset_odom"
    fi
    log_info "Getting arguments of Service: ${service}"
    rosservice args "${service}" || log_error "Failed to get Service arguments"
}

# ===================== 6. Parameter Commands ======================
list_params() {
    check_ros_env
    log_info "List all ROS Parameters"
    rosparam list || log_error "Failed to get Parameter list"
}

param_get() {
    check_ros_env
    local param="$1"
    if [ -z "${param}" ]; then
        log_error "Please specify the Parameter name! Example: ./run.sh param-get /lawnmower/speed_limit"
    fi
    log_info "Getting value of Parameter: ${param}"
    rosparam get "${param}" || log_error "Failed to get Parameter value"
}

param_set() {
    check_ros_env
    local param="$1" value="$2"
    if [ -z "${param}" ] || [ -z "${value}" ]; then
        log_error "Missing parameters! Example: ./run.sh param-set /lawnmower/speed_limit 0.5"
    fi
    log_info "Setting Parameter ${param} to ${value}"
    rosparam set "${param}" "${value}" || log_error "Failed to set Parameter value"
}

param_delete() {
    check_ros_env
    local param="$1"
    if [ -z "${param}" ]; then
        log_error "Please specify the Parameter name! Example: ./run.sh param-delete /lawnmower/speed_limit"
    fi
    log_info "Deleting Parameter: ${param}"
    rosparam delete "${param}" || log_error "Failed to delete Parameter"
}

# ===================== 7. Msg/Srv Commands ======================
list_msgs() {
    check_ros_env
    log_info "List all ROS Message types"
    rosmsg list || log_error "Failed to get Message list"
}

msg_info() {
    check_ros_env
    local msg="$1"
    if [ -z "${msg}" ]; then
        log_error "Please specify the Message type! Example: ./run.sh msg-info geometry_msgs/Twist"
    fi
    log_info "Viewing details of Message: ${msg}"
    rosmsg show "${msg}" || log_error "Failed to get Message details"
}

list_srvs() {
    check_ros_env
    log_info "List all ROS Service types"
    rossrv list || log_error "Failed to get Service type list"
}

srv_info() {
    check_ros_env
    local srv="$1"
    if [ -z "${srv}" ]; then
        log_error "Please specify the Service type! Example: ./run.sh srv-info std_srvs/Empty"
    fi
    log_info "Viewing details of Service: ${srv}"
    rossrv show "${srv}" || log_error "Failed to get Service details"
}

# ===================== 8. Rosbag Commands ======================
list_bags() {
    mkdir -p "${BAG_DIR}"
    log_info "List all Rosbag files (Dir: ${BAG_DIR})"
    local bag_count=$(ls -1 "${BAG_DIR}"/*.bag 2>/dev/null | wc -l)
    if [ "${bag_count}" -eq 0 ]; then
        log_warn "No Rosbag files detected"
        return 0
    fi
    ls -lh "${BAG_DIR}"/*.bag || log_error "Failed to list Bag files"
    log_success "Detected ${bag_count} Bag files"
}

play_bag() {
    check_ros_env
    local bag_name="$1" rate=1 loop=false topics=()
    shift
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -r|--rate) rate="$2"; shift 2 ;;
            --loop) loop=true; shift ;;
            -t|--topics) topics=("$2"); shift 2 ;;
            *) log_error "Unknown parameter: $1! Example: ./run.sh play-bag test.bag -r 0.5 --loop" ;;
        esac
    done

    local bag_path="${BAG_DIR}/${bag_name}"
    if [ ! -f "${bag_path}" ]; then
        log_error "Bag file does not exist: ${bag_path}"
    fi

    log_info "Play Bag file: ${bag_name} (Rate: ${rate} | Loop: ${loop})"
    local play_cmd="rosbag play ${bag_path} -r ${rate}"
    [ "${loop}" = true ] && play_cmd+=" --loop"
    [ ${#topics[@]} -gt 0 ] && play_cmd+=" --topics ${topics[*]}"
    eval "${play_cmd}" || log_error "Failed to play Bag"
}

record_bag() {
    check_ros_env
    mkdir -p "${BAG_DIR}"
    local bag_name="record_$(date +%Y%m%d_%H%M%S).bag"
    local bag_path="${BAG_DIR}/${bag_name}"
    local duration="" topics=()

    shift
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -d|--duration) duration="$2"; shift 2 ;;
            -t|--topics) topics=("$2"); shift 2 ;;
            -o|--output) bag_name="$2"; bag_path="${BAG_DIR}/${bag_name}"; shift 2 ;;
            *) log_error "Unknown parameter: $1! Example: ./run.sh record-bag -d 10 -t /cmd_vel /odom -o my_record.bag" ;;
        esac
    done

    if [ ${#topics[@]} -eq 0 ]; then
        log_error "Please specify topics to record! Example: ./run.sh record-bag -t /cmd_vel /odom"
    fi

    log_info "Recording Bag file: ${bag_name} (Duration: ${duration:-unlimited} | Topics: ${topics[*]})"
    local record_cmd="rosbag record -O ${bag_path}"
    [ -n "${duration}" ] && record_cmd+=" -d ${duration}"
    record_cmd+=" ${topics[*]}"
    eval "${record_cmd}" || log_error "Failed to record Bag"
}

bag_info() {
    local bag_name="$1"
    if [ -z "${bag_name}" ]; then
        log_error "Please specify the Bag file name! Example: ./run.sh bag-info test.bag"
    fi
    local bag_path="${BAG_DIR}/${bag_name}"
    if [ ! -f "${bag_path}" ]; then
        log_error "Bag file does not exist: ${bag_path}"
    fi
    log_info "Viewing info of Bag file: ${bag_name}"
    rosbag info "${bag_path}" || log_error "Failed to get Bag info"
}

bag_extract() {
    local bag_name="$1" extract_dir="${BAG_DIR}/extracted_${bag_name%.bag}"
    if [ -z "${bag_name}" ]; then
        log_error "Please specify the Bag file name! Example: ./run.sh bag-extract test.bag"
    fi
    local bag_path="${BAG_DIR}/${bag_name}"
    if [ ! -f "${bag_path}" ]; then
        log_error "Bag file does not exist: ${bag_path}"
    fi
    log_info "Extracting Bag file ${bag_name} to ${extract_dir}"
    mkdir -p "${extract_dir}"
    rosbag decompress "${bag_path}" -d "${extract_dir}" || log_error "Failed to extract Bag"
}

# ===================== 9. Debug Commands ======================
list_debug_cmds() {
    check_cmdlist
    log_info "===== All Debug Commands in Cmdlist.md ====="
    grep "^- Command:" "${CMDLIST_PATH}" | sed 's/- Command: //' | while read -r cmd; do
        echo "  - ${cmd}"
    done
    echo "============================================"
}

run_debug_cmd() {
    check_cmdlist
    local cmd_name="$1"
    local cmd_content=$(parse_cmdlist "${cmd_name}")
    log_info "Executing debug command: ${cmd_name} (Content: ${cmd_content})"
    eval "${cmd_content}" || log_error "Failed to execute debug command '${cmd_name}'"
    log_success "Debug command '${cmd_name}' executed successfully"
}

# ===================== 10. System Commands ======================
check_disk_space() {
    log_info "Checking disk space for ${BAG_DIR}"
    df -h "${BAG_DIR}" || log_error "Failed to check disk space"
}

check_ros_processes() {
    log_info "Listing ROS related processes"
    ps aux | grep -E "ros|roscore|roslaunch|rosrun" | grep -v grep || log_warn "No ROS processes found"
}

check_ros_deps() {
    check_ros_env
    log_info "Checking ROS package dependencies"
    rospack depends lawnmower_core || log_warn "Failed to check dependencies for lawnmower_core"
}

# ===================== 11. Release Command (新增) ======================
run_release() {
    if [ ! -f "${RELEASE_SCRIPT}" ]; then
        log_error "Release script not found: ${RELEASE_SCRIPT}"
    fi
    chmod +x "${RELEASE_SCRIPT}"
    log_info "Starting release packaging with parameters: $*"
    "${RELEASE_SCRIPT}" "$@"
}

# ===================== Help Command ======================
show_help() {
    cat << EOF
Usage: ./run.sh <command> [options]

===================== Build Commands =====================
  build              Full build of workspace
  build-pkg <pkg>    Build single ROS package
  build-inc          Incremental build
  clean              Clean build products

===================== ROS Core Commands =====================
  roscore-start      Start roscore
  roscore-stop       Stop roscore
  roscore-restart    Restart roscore
  ros-env            Show ROS environment info
  ros-health         Check ROS system health

===================== Topic Commands =====================
  topic-list         List all topics
  topic-hz <topic>   Monitor topic frequency
  topic-echo <topic> Print topic content
  topic-pub <t> <type> <msg> Publish topic message
  topic-type <topic> Get topic type
  topic-record <t>   Record topic to bag

===================== Node Commands =====================
  node-list          List all nodes
  node-info <node>   View node details
  node-kill <node>   Stop specified node
  node-ping <node>   Check if node is online
  node-log <node>    View node log

===================== Service Commands =====================
  service-list       List all services
  service-call <s> [args] Call service
  service-type <s>   Get service type
  service-args <s>   Get service arguments

===================== Parameter Commands =====================
  param-list         List all parameters
  param-get <p>      Get parameter value
  param-set <p> <v>  Set parameter value
  param-delete <p>   Delete parameter

===================== Msg/Srv Commands =====================
  msg-list           List all message types
  msg-info <msg>     View message details
  srv-list           List all service types
  srv-info <srv>     View service details

===================== Rosbag Commands =====================
  bag-list           List all bag files
  bag-play <bag> [opts] Play bag file
                     -r/--rate <num>  Play rate
                     --loop           Loop play
                     -t/--topics <t>  Specify topics
  bag-record [opts]  Record bag file
                     -d/--duration <s> Recording duration
                     -t/--topics <t>   Topics to record
                     -o/--output <name> Output bag name
  bag-info <bag>     View bag file info
  bag-extract <bag>  Extract bag file

===================== Debug Commands =====================
  debug-list         List all debug commands in Cmdlist.md
  debug <cmd>        Execute debug command from Cmdlist.md

===================== Release Command =====================
  release [-v version]  Build release package (e.g. ./run.sh release -v v1.1)

===================== System Checks =====================
  disk-space         Check disk space for bags directory
  ros-processes      List ROS related processes
  ros-deps           Check ROS package dependencies

===================== Utility Commands =====================
  check-tools        Check ROS core tools
  check-cmdlist      Verify Cmdlist.md
  help               Show this help message
EOF
}

# ===================== 主命令分发 ======================
main() {
    if [ $# -eq 0 ]; then
        show_help
        exit 0
    fi

    case "$1" in
        # Build
        build) build_all ;;
        build-pkg) shift; build_pkg "$1" ;;
        build-inc) build_incremental ;;
        clean) clean_build ;;

        # ROS Core
        roscore-start) start_roscore ;;
        roscore-stop) stop_roscore ;;
        roscore-restart) restart_roscore ;;
        ros-env) ros_env_info ;;
        ros-health) ros_check_health ;;

        # Topic
        topic-list) list_topics ;;
        topic-hz) shift; topic_hz "$1" ;;
        topic-echo) shift; topic_echo "$1" ;;
        topic-pub) shift; topic_pub "$1" "$2" "$3" ;;
        topic-type) shift; topic_type "$1" ;;
        topic-record) shift; topic_record "$1" ;;

        # Node
        node-list) list_nodes ;;
        node-info) shift; node_info "$1" ;;
        node-kill) shift; kill_node "$1" ;;
        node-ping) shift; node_ping "$1" ;;
        node-log) shift; node_log "$1" ;;

        # Service
        service-list) list_services ;;
        service-call) shift; call_service "$1" "$2" ;;
        service-type) shift; service_type "$1" ;;
        service-args) shift; service_args "$1" ;;

        # Param
        param-list) list_params ;;
        param-get) shift; param_get "$1" ;;
        param-set) shift; param_set "$1" "$2" ;;
        param-delete) shift; param_delete "$1" ;;

        # Msg/Srv
        msg-list) list_msgs ;;
        msg-info) shift; msg_info "$1" ;;
        srv-list) list_srvs ;;
        srv-info) shift; srv_info "$1" ;;

        # Bag
        bag-list) list_bags ;;
        bag-play) shift; play_bag "$@" ;;
        bag-record) shift; record_bag "$@" ;;
        bag-info) shift; bag_info "$1" ;;
        bag-extract) shift; bag_extract "$1" ;;

        # Debug
        debug-list) list_debug_cmds ;;
        debug) shift; run_debug_cmd "$1" ;;

        # Release (新增)
        release) shift; run_release "$@" ;;

        # System
        disk-space) check_disk_space ;;
        ros-processes) check_ros_processes ;;
        ros-deps) check_ros_deps ;;

        # Utils
        check-tools) check_ros_tools ;;
        check-cmdlist) check_cmdlist ;;
        help) show_help ;;

        *) log_error "Unknown command: $1! Use './run.sh help' to see all commands" ;;
    esac
}

main "$@"