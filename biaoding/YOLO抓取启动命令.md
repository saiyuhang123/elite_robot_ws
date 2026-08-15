# YOLO 视觉抓取启动命令速查

## 环境准备

所有终端执行前先 source：

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/elite_robot_ws/install/setup.bash
```

CAN 端口（机械手，每次插拔/重启后执行一次）：

```bash
sudo /usr/sbin/ip link set can2 up type can bitrate 1000000
ip link show can2   # 应显示 <UP,LOWER_UP>
```

---
# 切到单个类别（以 cup 为例）
ros2 topic pub --once /yolo/target_class std_msgs/msg/String "data: 'cup'"

# 同时检测多个类别（取深度最近的一个）
ros2 topic pub --once /yolo/target_class std_msgs/msg/String "data: 'apple,cup'"

# 恢复检测所有类别
ros2 topic pub --once /yolo/target_class std_msgs/msg/String "data: 'all'"

## 方式一：Launch 文件一键启动（推荐）

一个终端启动全部后台节点（驱动 + 相机 + 机械手 + YOLO 感知）：

```bash
# 灵巧手版（LinkerHand O6）
ros2 launch ~/Documents/elite_robot_ws/biaoding/yolo_grasp.launch.py

# 二指版（Inspire 4B4C，含夹爪节点 + 自动带 --gripper two_finger 启动主程序）
ros2 launch ~/Documents/elite_robot_ws/biaoding/yolo_grasp_two_finger.launch.py

# 全程调度（导航中断抓瓶）建议加 headless，只留 ROS 服务
ros2 launch ~/Documents/elite_robot_ws/biaoding/yolo_grasp_two_finger.launch.py \
    grasp_headless:=true
```

如果用一键启动，抓取主程序会自动在独立终端窗口启动；手动启动时（根据夹爪选择）：

```bash
cd ~/Documents/elite_robot_ws/biaoding
python3 yolo_grasp.py                          # 默认 LinkerHand O6 灵巧手
python3 yolo_grasp.py --gripper two_finger     # 二指夹爪（Inspire 4B4C）
python3 yolo_grasp.py --gripper soft_touch     # 柔触三指气动夹爪
```

不同夹爪需要提前启动对应的控制节点：

| 夹爪 | 前置节点 |
|---|---|
| linkerhand | `source ~/Documents/elite_robot_ws/install/setup.bash && ros2 launch linker_hand_ros2_sdk linker_hand.launch.py` |
| two_finger | `ros2 run inspire_gripper Gripper_control_node` |
| soft_touch | `ros2 run gripper_control gripper_server` |

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

**终端2 — Percipio 相机**（彩色+深度 1280×960，registration 默认开，深度已对齐到彩色坐标系）

```bash
ros2 launch percipio_camera percipio_camera.launch.py \
    color_resolution:=1280x960 \
    depth_resolution:=1280x960
```

验证：`timeout 8 ros2 topic hz /camera/color/image_raw` 有数据；对齐状态用 `python3 ~/Documents/elite_robot_ws/check_camera_info.py` 复核（depth 与 color 内参应一致）。

**终端3 — LinkerHand SDK**

```bash
cd ~/Documents/elite_robot_ws
source ./install/setup.bash
ros2 launch linker_hand_ros2_sdk linker_hand.launch.py
```

**终端4 — YOLO 感知节点**

```bash
cd ~/Documents/elite_robot_ws/YOLO
python3 yolo_grasp_perception.py
```

正常标志：按 `e`（或在 yolo_grasp.py 里按 g 触发抓取）后打印 `识别到 [apple] ... 基座系: [...]`。

**注意：识别默认是关闭的（按需识别）** —— 感知节点只缓存图像不推理，
省算力也防止导航途中的旧检测污染抓取。抓取流程会在预备位姿停稳后
自动临时开启（`/yolo_perception/set_enabled`），锁存目标后立即关闭。
调试看标注图像/目标点时，需先在 yolo_grasp.py 按 `e` 或：
`ros2 service call /yolo_perception/set_enabled std_srvs/srv/SetBool "data: true"`

**终端5 — 抓取主程序**

```bash
cd ~/Documents/elite_robot_ws/biaoding
python3 yolo_grasp.py
```

---

柔触
source /opt/ros/humble/setup.bash
source ~/Documents/elite_robot_ws/install/setup.bash

ros2 launch ~/Documents/elite_robot_ws/biaoding/yolo_grasp_soft_touch.launch.py

## 键盘操作（yolo_grasp.py 终端内）

| 按键 | 作用 |
|---|---|
| `g` | 触发抓取流程（预抓取→前伸→闭合→退回） |
| `o` | 张开机械手 |
| `c` | 闭合机械手 |
| `p` | 打印当前目标点坐标 |
| `h` | 回零位 |
| `2` | 回 Home2 位姿（-2.2, 19.7, -154.8, -86.3, 94.1, 84.2°） |
| `r` | 移动到抓取预备位姿 |
| `j` | 示教放置位姿（记录当前关节角到 place_pose.json） |
| `l` | 执行放置（到放置位姿→张手→退回 Home2） |
| `e` | 开关持续识别（调试用；识别默认关闭，抓取时自动临时开启） |
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

# 多个类别（同时检测，取深度最近的一个）
ros2 topic pub -1 /yolo/target_class std_msgs/msg/String "data: 'apple,cup'"

# 单个类别
ros2 topic pub -1 /yolo/target_class std_msgs/msg/String "data: 'apple'"

# 所有类别
ros2 topic pub -1 /yolo/target_class std_msgs/msg/String "data: 'all'"
```

### 抓取控制

```bash
# 触发一次完整抓取


# 抓取并保持夹持（不自动松手，等 /yolo_grasp/place 放下；全程调度用这个）
ros2 service call /yolo_grasp/grasp_hold std_srvs/srv/Trigger

# 张开机械手
ros2 service call /yolo_grasp/open std_srvs/srv/Trigger

# 闭合机械手
ros2 service call /yolo_grasp/close std_srvs/srv/Trigger

# 回零位
ros2 service call /yolo_grasp/home std_srvs/srv/Trigger

# 回 Home2 位姿（收拢姿态，同步执行，返回时已到位；底盘导航前必调）
ros2 service call /yolo_grasp/home2 std_srvs/srv/Trigger

# 到抓取预备位姿
ros2 service call /yolo_grasp/ready std_srvs/srv/Trigger

# 放置（movej 到示教放置位姿 → 张手 → 退回 Home2，需先按 j 示教）
ros2 service call /yolo_grasp/place std_srvs/srv/Trigger

# 查看当前目标状态
ros2 service call /yolo_grasp/status std_srvs/srv/Trigger
```

### 典型自动化流程

```bash
# 1. 设目标为 cup
ros2 topic pub -1 /yolo/target_class std_msgs/msg/String "data: 'cup'"

# 2. 确认检测到目标
ros2 service call /yolo_grasp/status std_srvs/srv/Trigger

# 3. 抓取（全程/需放置时用 grasp_hold，单机测试用 grasp 也行）
ros2 service call /yolo_grasp/grasp std_srvs/srv/Trigger

# 4. 放置（需先按 j 示教 place_pose.json）
ros2 service call /yolo_grasp/place std_srvs/srv/Trigger
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
| `biaoding/yolo_grasp.py` | 抓取主程序（支持多夹爪，`--gripper` 选择） |
| `biaoding/grippers/` | 夹爪抽象层（base / linkerhand / two_finger / soft_touch），**抓取姿态由各夹爪的 `grasp_rotation()` 定义** |
| `biaoding/place_pose.json` | 放置位姿示教文件（按 `j` 示教） |
| `YOLO/yolo_grasp_perception.py` | YOLO 感知节点（模型路径、目标类别、置信度阈值） |
| `YOLO/yolov8s.pt` | YOLOv8 模型权重 |

注：`grasp_orientation_*.json` 示教姿态文件已废弃不再使用。抓取姿态现在是代码构造的固定姿态：
灵巧手 = 法兰面朝世界 X+、手水平伸出、手心朝下（`linkerhand.py`）；
二指/柔触 = 法兰 Z 朝世界正下方竖直抓（`base.py` 默认）。

### 夹爪参数速查

| 参数 | linkerhand | two_finger | soft_touch |
|---|---|---|---|
| 控制方式 | topic (JointState) | 服务 (serial→ROS) | 服务 (Modbus TCP) |
| IK 模式 | 6dof | 6dof | 5dof（旋转对称） |
| 抓取姿态 | 法兰面朝世界X+，手心朝下 | 法兰Z朝下 | 法兰Z朝下 |
| tool_length | 0.13 | 0.12 | 0.15 |
| grasp_offset Z | 0.06（压物调大/抓空调小） | 0.0（对准中心） | 0.02（略高） |
| close_delay | 1.5s | 1.0s | 2.0s |

---

## 常见问题

| 现象 | 处理 |
|---|---|
| 感知节点不检测目标 | 确认相机三话题都有数据；`ros2 topic pub /yolo/target_class ...` 切换类别试试 |
| 抓取偏差大 | `hand_eye_result.json` 可能过期，重新标定后跑 `update_hand_eye_json.py` |
| 机械臂不动 | 确认远程模式 + `resend_external_script` |
| 服务调用卡住没反应 | 旧版 `yolo_grasp.py` 的 bug：业务代码 `spin_once` 了已被后台执行器管理的节点。用最新代码（`spin()` 已改为纯 sleep），重启主程序 |
| 抓取时压住物体 | 调 `grippers/linkerhand.py` 的 `grasp_offset_world` Z 值（压物调大、抓空调小） |
| YOLO 推理卡死 | 确认用的是 `.pt` 模型而非跨机器导出的 `.engine` |
| TF 报错 `two unconnected trees` | 确认机械臂驱动已启动（TF 前缀 `cs66_`） |
