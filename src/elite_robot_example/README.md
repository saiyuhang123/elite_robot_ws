# Elite Robot Example - 运动控制与状态反馈示例

## 功能说明

这个包提供了两个示例节点：

1. **robot_controller** - 完整的运动控制示例
   - 订阅关节状态和末端位姿
   - 执行关节运动（绝对位置、单关节控制）
   - 控制夹爪（数字 IO）
   - 机器人管理（上电、刹车释放）

2. **status_monitor** - 状态监控工具
   - 实时显示关节位置（角度）
   - 实时显示末端位姿
   - 终端可视化

## 前置条件

1. 已安装 `elite-cs-series-sdk`
2. 已编译 ROS2 工作空间
3. 机械臂已连接并配置好 IP（192.168.1.212）

## 使用方法

### 1. 启动机械臂驱动

```bash
# 终端 1：启动驱动（根据你的机械臂型号修改 cs_type）
cd ~/Documents/elite_robot_ws
source install/setup.bash
ros2 launch eli_cs_robot_driver elite_control.launch.py \
    robot_ip:=192.168.1.212 \
    cs_type:=cs66 \
    local_ip:=<你的电脑 IP>
```

### 2. 运行状态监控

```bash
# 终端 2：实时查看状态
cd ~/Documents/elite_robot_ws
source install/setup.bash
ros2 run elite_robot_example status_monitor
```

### 3. 运行运动控制示例

```bash
# 终端 2：运行控制示例
cd ~/Documents/elite_robot_ws
source install/setup.bash
ros2 run elite_robot_example robot_controller
```

## 代码说明

### robot_controller.py 核心接口

```python
from elite_robot_example.robot_controller import EliteRobotController

# 创建控制器
node = EliteRobotController()

# 等待状态
node.wait_for_state(timeout=10.0)

# 获取状态
joint_pos = node.get_joint_positions()  # 弧度
joint_deg = node.get_joint_degrees()    # 角度
tcp_pose = node.get_tcp_pose()          # (position, orientation)

# 运动控制
node.move_to_home(duration=5.0)                    # 移动到零位
node.move_to_ready(duration=5.0)                   # 移动到准备位置
node.move_to_joint_positions([0, -1.2, 0.8, ...])  # 移动到指定位置
node.move_joint_by_index(0, 45.0, duration=3.0)    # 移动单个关节

# 夹爪控制
node.open_gripper(pin=0)   # 打开夹爪
node.close_gripper(pin=0)  # 关闭夹爪

# 机器人管理
node.power_on()      # 上电
node.brake_release() # 释放刹车
```

### 关节名称顺序

```python
JOINT_NAMES = [
    "shoulder_pan_joint",    # J1 底座旋转
    "shoulder_lift_joint",   # J2 肩部
    "elbow_joint",           # J3 肘部
    "wrist_1_joint",         # J4 腕部 1
    "wrist_2_joint",         # J5 腕部 2
    "wrist_3_joint",         # J6 腕部 3
]
```

### 预定义位置

```python
HOME_POSITION = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]      # 零位
READY_POSITION = [0.0, -1.2, 0.8, -1.57, -1.57, 0.0]     # 准备位置
```

## 常见问题

### 1. 无法连接机械臂

检查：
- 机械臂是否已上电
- IP 地址是否正确（192.168.1.212）
- 网线是否连接
- 防火墙是否放行

### 2. 运动被拒绝

可能原因：
- 目标位置超出关节限制
- 速度设置过快
- 安全模式触发

### 3. IO 控制无效

检查：
- 夹爪是否正确连接到数字输出
- 引脚号是否正确（默认 pin=0）
- 电压设置是否正确

## 扩展开发

基于这个示例，你可以：

1. **集成 YOLO 检测**：订阅相机 Topic，获取目标位置
2. **手眼标定**：将相机坐标转换为机械臂坐标
3. **自动抓取流程**：检测 → 移动 → 抓取 → 放置
4. **轨迹规划**：使用 MoveIt 进行路径规划
