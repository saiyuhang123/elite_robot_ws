# YOLO 视觉抓取启动命令速查

## 环境准备

所有终端执行前先 source：

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/elite_robot_ws/install/setup.bash
source ~/Documents/linker_hand_ros2_sdk/install/setup.bash
```

CAN 端口（机械手，每次插拔/重启后执行一次）：

```bash
sudo /usr/sbin/ip link set can2 up type can bitrate 1000000
ip link show can2   # 应显示 <UP,LOWER_UP>
```

---

## 方式一：Launch 文件一键启动（推荐）

一个终端启动全部后台节点（驱动 + 相机 + 机械手 + YOLO 感知）：

```bash
ros2 launch ~/Documents/elite_robot_ws/biaoding/yolo_grasp.launch.py
```

第二个终端启动抓取主程序：

```bash
cd ~/Documents/elite_robot_ws/biaoding
python3 yolo_grasp.py
```

可选参数：

```bash
ros2 launch ~/Documents/elite_robot_ws/biaoding/yolo_grasp.launch.py \
    run_camera:=false \        # 不启动相机（相机已在别处运行）
    run_linker_hand:=false \   # 不启动机械手 SDK
    run_perception:=false      # 不启动 YOLO 感知节点
```

---

## 方式二：分终端手动启动（调试用）

**终端0 — CAN 端口**

```bash
sudo /usr/sbin/ip link set can2 up type can bitrate 1000000
ip link show can2
```

**终端1 — 机械臂驱动**

```bash
ros2 launch my_elite_robot_cell_control start_robot.launch.py \
    headless_mode:=true launch_rviz:=false
```

**终端2 — RealSense 相机**

```bash
ros2 launch realsense2_camera rs_launch.py \
    camera_namespace:=camera \
    enable_color:=true \
    enable_depth:=true \
    rgb_camera.color_profile:=1280x720x30 \
    depth_module.depth_profile:=640x480x30 \
    align_depth.enable:=true
```

验证：`ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw` 有数据。

**终端3 — LinkerHand SDK**

```bash
cd /home/nvidia/Documents/linker_hand_ros2_sdk
source ./install/setup.bash
ros2 launch linker_hand_ros2_sdk linker_hand.launch.py
```

**终端4 — YOLO 感知节点**

```bash
cd ~/Documents/elite_robot_ws/YOLO
python3 yolo_grasp_perception.py
```

正常标志：打印 `识别到 [apple] ... 基座系: [...]`

**终端5 — 抓取主程序**

```bash
cd ~/Documents/elite_robot_ws/biaoding
python3 yolo_grasp.py
```

---

## 键盘操作（yolo_grasp.py 终端内）

| 按键 | 作用 |
|---|---|
| `g` | 触发抓取流程（预抓取→前伸→闭合→退回） |
| `o` | 张开机械手 |
| `c` | 闭合机械手 |
| `p` | 打印当前目标点坐标 |
| `h` | 回零位 |
| `t apple` | 切换目标类别为 apple |
| `t cup` | 切换目标类别为 cup |
| `t apple,cup` | 同时检测多个类别 |
| `t all` | 检测所有类别 |
| `q` | 退出 |

---

## 远程控制（ROS 服务/话题，无需终端交互）

抓取主程序运行期间，可在任意终端通过 ROS 接口控制：

### 切换检测目标

```bash
# 单个类别
ros2 topic pub -1 /yolo/target_class std_msgs/msg/String "data: 'apple'"

# 多个类别
ros2 topic pub -1 /yolo/target_class std_msgs/msg/String "data: 'apple,cup'"

# 所有类别
ros2 topic pub -1 /yolo/target_class std_msgs/msg/String "data: 'all'"
```

### 抓取控制

```bash
# 触发一次完整抓取
ros2 service call /yolo_grasp/grasp std_srvs/srv/Trigger

# 张开机械手
ros2 service call /yolo_grasp/open std_srvs/srv/Trigger

# 闭合机械手
ros2 service call /yolo_grasp/close std_srvs/srv/Trigger

# 回零位
ros2 service call /yolo_grasp/home std_srvs/srv/Trigger

# 查看当前目标状态
ros2 service call /yolo_grasp/status std_srvs/srv/Trigger
```

### 典型自动化流程

```bash
# 1. 设目标为 cup
ros2 topic pub -1 /yolo/target_class std_msgs/msg/String "data: 'cup'"

# 2. 确认检测到目标
ros2 service call /yolo_grasp/status std_srvs/srv/Trigger

# 3. 抓取
ros2 service call /yolo_grasp/grasp std_srvs/srv/Trigger

# 4. 回零
ros2 service call /yolo_grasp/home std_srvs/srv/Trigger
```

---

## 调试工具

### 查看 YOLO 标注图像

```bash
cd ~/Documents/elite_robot_ws/YOLO
python3 view_annotated.py
```

### 查看目标点话题

```bash
ros2 topic echo /target_object_pose
```

### 查看机械手控制话题

```bash
ros2 topic echo /cb_right_hand_control_cmd
```

### 恢复外部控制脚本

机械臂指令被接受但不动时执行：

```bash
ros2 service call /io_and_status_controller/resend_external_script std_srvs/srv/Trigger
```

---

## 关键配置文件

| 文件 | 作用 |
|---|---|
| `biaoding/hand_eye_result.json` | 手眼标定结果（相机→法兰变换矩阵） |
| `biaoding/yolo_grasp.py` | 抓取主程序（含抓取偏移、速度等参数） |
| `YOLO/yolo_grasp_perception.py` | YOLO 感知节点（模型路径、目标类别、置信度阈值） |
| `YOLO/yolov8s.pt` | YOLOv8 模型权重 |

---

## 常见问题

| 现象 | 处理 |
|---|---|
| 感知节点不检测目标 | 确认相机三话题都有数据；`ros2 topic pub /yolo/target_class ...` 切换类别试试 |
| 抓取偏差大 | `hand_eye_result.json` 可能过期，重新标定后跑 `update_hand_eye_json.py` |
| 机械臂不动 | 确认远程模式 + `resend_external_script` |
| YOLO 推理卡死 | 确认用的是 `.pt` 模型而非跨机器导出的 `.engine` |
| TF 报错 `two unconnected trees` | 确认机械臂驱动已启动（TF 前缀 `cs66_`） |
