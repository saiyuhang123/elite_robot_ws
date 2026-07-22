#!/bin/bash
# 视觉抓取一键启动：驱动 -> 相机 -> 抓取脚本
# 抓取脚本走控制器原生 movej（/script_sender/script_command），不需要 MoveIt。
# 每个命令间隔 2 秒；抓取脚本的按键窗口会正常弹出。
# 日志输出到 biaoding/logs/ 下；按 Ctrl+C 停止所有进程。
# 注意：不要用 set -u，ROS 的 setup.bash 会引用未定义变量导致报错。

WS=~/Documents/elite_robot_ws
LOG_DIR="$WS/biaoding/logs"
mkdir -p "$LOG_DIR"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

PIDS=()
cleanup() {
    echo ""
    echo "正在停止所有进程..."
    kill "${PIDS[@]}" 2>/dev/null
    wait 2>/dev/null
    echo "已全部停止。"
}
trap cleanup INT TERM

echo "[1/3] 启动机械臂驱动 (无头模式)..."
ros2 launch my_elite_robot_cell_control start_robot.launch.py headless_mode:=true launch_rviz:=false \
    > "$LOG_DIR/1_driver.log" 2>&1 &
PIDS+=($!)
sleep 2

echo "[2/3] 启动 RealSense 相机..."
ros2 launch realsense2_camera rs_launch.py \
    camera_namespace:=camera \
    enable_color:=true \
    enable_depth:=true \
    rgb_camera.color_profile:=1280x720x30 \
    depth_module.depth_profile:=640x480x30 \
    align_depth.enable:=true \
    > "$LOG_DIR/2_camera.log" 2>&1 &
PIDS+=($!)
sleep 2

echo "[3/3] 启动抓取脚本 (按键: G=抓取 C=闭爪 O=开爪 H=回零 Q=退出)..."
cd "$WS/biaoding"
python3 visual_grasp_test.py 2>&1 | tee "$LOG_DIR/3_visual_grasp.log" &
PIDS+=($!)

# 自检：机器人侧脚本可能启动即死（上电未就绪竞态），没连上 50001 就补发一次
sleep 5
if ! ss -tn 2>/dev/null | grep -q ":50001 .*192.168.1.212"; then
    echo "检测到机器人侧脚本未连接，补发 external control script..."
    ros2 service call /io_and_status_controller/resend_external_script std_srvs/srv/Trigger || true
fi

echo ""
echo "全部启动完成。日志目录: $LOG_DIR"
echo "抓取脚本画面窗口弹出后即可操作；按 Ctrl+C 停止所有进程。"

wait
