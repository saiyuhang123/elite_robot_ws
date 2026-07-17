# 视觉引导抓取测试指南

## 概述

本指南介绍如何使用 Elite Robot SDK 进行视觉引导抓取测试。

## 文件说明

1. `calculate_my_matrix.py` - 手眼标定计算脚本
2. `visual_grasp_test.py` - 原始视觉抓取测试脚本（需要修改）
3. `visual_grasp_with_sdk.py` - 使用 Elite Robot SDK 的视觉抓取测试脚本
4. `hand_eye_result.json` - 手眼标定结果文件（运行 calculate_my_matrix.py 生成）

## 前提条件

1. 已安装 ROS2 和 Elite Robot 驱动
2. 已完成手眼标定（运行 calculate_my_matrix.py）
3. 机械臂已连接并启动

## 使用步骤

### 1. 启动机械臂驱动

```bash
# 终端 1：启动机械臂驱动
ros2 launch eli_cs_robot_driver elite_control.launch.py robot_ip:=192.168.1.212 cs_type:=cs66
```

### 2. 运行视觉抓取测试

```bash
# 终端 2：运行视觉抓取测试
cd /home/nvidia/Documents/elite_robot_ws/biaoding
python3 visual_grasp_with_sdk.py
```

### 3. 操作说明

1. 让相机对准标定板（ID 为 2 的 ArUco 标记）
2. 在画面窗口按下键盘 'G' 键，机械臂将尝试移动到标定板上方
3. 按 'Q' 键退出程序

## 代码说明

### RobotCartesianControl 类

这是 Elite Robot 的笛卡尔空间控制类，提供以下功能：

- `get_tcp_pose()` - 获取当前末端位姿
- `move_to_cartesian_pose(target_position, target_orientation)` - 移动到目标笛卡尔空间位姿
- `move_to_joint_positions(target_positions)` - 移动到目标关节位置
- `open_gripper()` / `close_gripper()` - 夹爪控制

### 坐标系转换

视觉引导的核心是坐标系转换：

1. **相机坐标系** → **末端坐标系**：使用手眼标定结果 (R_cam2tool, t_cam2tool)
2. **末端坐标系** → **基座坐标系**：使用当前末端位姿 (R_tool2base, t_tool2base)

转换公式：
```
P_base = R_tool2base * (R_cam2tool * P_cam + t_cam2tool) + t_tool2base
```

## 注意事项

1. **安全性**：第一次测试时，建议将机械臂倍率速度设在 5% 以内，防止撞机
2. **标定精度**：确保手眼标定结果准确，否则抓取位置会有偏差
3. **相机参数**：确保相机内参正确，否则检测到的目标位置会有误差

## 故障排除

### 1. 无法连接机械臂

检查：
- 机械臂驱动是否启动
- IP 地址是否正确 (192.168.1.212)
- 网络连接是否正常

### 2. 无法检测到标定板

检查：
- 相机是否对准标定板
- 标定板 ID 是否为 2
- 相机内参是否正确

### 3. 机械臂运动位置不准确

检查：
- 手眼标定结果是否准确
- 相机内参是否正确
- 标定板尺寸是否正确

## 扩展功能

### 1. 使用 MoveIt 进行逆运动学

如果需要更精确的逆运动学求解，可以集成 MoveIt：

```python
from moveit_msgs.srv import GetPositionIK
```

### 2. 添加视觉伺服

可以添加视觉伺服功能，实时调整机械臂位置：

```python
# 在循环中实时计算误差并调整
error = target_position - current_position
correction = Kp * error
robot_node.move_to_cartesian_pose(current_position + correction, current_orientation)
```

### 3. 添加夹爪控制

在抓取完成后，可以添加夹爪控制：

```python
# 移动到目标位置
robot_node.move_to_cartesian_pose(target_position, target_orientation)

# 关闭夹爪
robot_node.close_gripper(pin=0)

# 移动到放置位置
robot_node.move_to_cartesian_pose(place_position, place_orientation)

# 打开夹爪
robot_node.open_gripper(pin=0)
```
