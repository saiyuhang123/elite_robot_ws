# Gripper Control

## 启动节点

```bash
ros2 run gripper_control gripper_server
```

修改参数：

```bash
ros2 run gripper_control gripper_server --ros-args \
  -p device_ip:=192.168.2.100 \
  -p device_port:=502 \
  -p poll_rate_hz:=10.0
```

## 服务调用 — 写操作

### 1. 启动正压 (线圈 0x0100 = ON)

```bash
ros2 service call /gripper_command gripper_control/srv/GripperCommand \
  "{command: 0, value: 0, slave_id: 1}"
```

### 2. 正压松气 (线圈 0x0100 = OFF)

```bash
ros2 service call /gripper_command gripper_control/srv/GripperCommand \
  "{command: 1, value: 0, slave_id: 1}"
```

### 3. 启动负压 (线圈 0x0101 = ON)

```bash
ros2 service call /gripper_command gripper_control/srv/GripperCommand \
  "{command: 2, value: 0, slave_id: 1}"
```

### 4. 负压松气 (线圈 0x0101 = OFF)

```bash
ros2 service call /gripper_command gripper_control/srv/GripperCommand \
  "{command: 3, value: 0, slave_id: 1}"
```

### 5. 设置气压上限 (寄存器 0x0306)

```bash
ros2 service call /gripper_command gripper_control/srv/GripperCommand \
  "{command: 4, value: 150, slave_id: 1}"
```

### 6. 设置负压值 (寄存器 0x0307)

```bash
ros2 service call /gripper_command gripper_control/srv/GripperCommand \
  "{command: 5, value: -50, slave_id: 1}"
```

## 服务调用 — 查询操作

### 7. 查询正压反馈 (离散输入 0x0200, 返回 0/1)

```bash
ros2 service call /gripper_command gripper_control/srv/GripperCommand \
  "{command: 6, value: 0, slave_id: 1}"
```

### 8. 查询负压反馈 (离散输入 0x0201, 返回 0/1)

```bash
ros2 service call /gripper_command gripper_control/srv/GripperCommand \
  "{command: 7, value: 0, slave_id: 1}"
```

## 气压话题监听

```bash
# 实时气压数值 (std_msgs/Float32, 单位 kPa)
ros2 topic echo /gripper_pressure

# 详细文本信息 (std_msgs/String)
ros2 topic echo /gripper_pressure_info

# 查看发布频率
ros2 topic hz /gripper_pressure
```
