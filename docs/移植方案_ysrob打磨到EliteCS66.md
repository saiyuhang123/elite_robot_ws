# 移植方案：ysrob 打磨系统 → Elite CS66

## 概述

将 `ysrob_ws` 中的力控打磨功能移植到 `elite_robot_ws`，使 Elite CS66 机械臂能够执行相同的力控曲线抛光任务。

---

## 一、为什么不需要重写 IK

这是整个移植方案中最关键的设计决策。理解这一点，就理解了为什么移植工作量远小于重写。

### 1.1 两种控制架构的对比

两个系统虽然底层不同，但上层控制模式完全一致：**都是"发关节角度目标 → 底层驱动做插补 → 发给控制器执行"**。

```
┌─────────────────────────────────────────────────────────────────┐
│                    ysrob (UR5)                                   │
│                                                                 │
│   状态机 (C++)                    自定义 UR 驱动                   │
│   ┌──────────────┐    Topic      ┌──────────────────┐           │
│   │ TRAC-IK 逆解 │ ──────────→  │ YsURDriver       │           │
│   │ 算出关节角    │ JointTraj    │ 内部做三次插补     │           │
│   │              │              │ 调 servoj 发给UR5 │           │
│   └──────────────┘              └──────────────────┘           │
│                                                                 │
│   数据流: 笛卡尔位姿 → IK求解 → 关节角 → 发Topic → 驱动 → 机器人  │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                    Elite CS66                                    │
│                                                                 │
│   状态机 (C++)                    ros2_control 标准栈              │
│   ┌──────────────┐    Topic      ┌──────────────────────────┐   │
│   │ TRAC-IK 逆解 │ ──────────→  │ scaled_joint_trajectory   │   │
│   │ 算出关节角    │ JointTraj    │ _controller               │   │
│   │              │  [桥接转换]   │ (action server)           │   │
│   │              │              │ 标准插补 → writeServoj    │   │
│   └──────────────┘              └──────────────────────────┘   │
│                                                                 │
│   数据流: 笛卡尔位姿 → IK求解 → 关节角 → 发Topic → 桥接 → 机器人 │
└─────────────────────────────────────────────────────────────────┘
```

### 1.2 核心洞察：IK 在状态机内部就已经做完了

看 ysrob 状态机的一个典型运动步骤（`polish_goPolishBase`，约第 756 行）：

```cpp
// 步骤1: 用 TRAC-IK 把笛卡尔目标解成关节角
KDL::Frame upFrame = startPos * moveFrame;       // 笛卡尔目标位姿
KDL::JntArray resultJnt(joint_size_);
int rc = ys_tcp_tracik_solver_->CartToJnt(       // ← IK 在这里完成
    ys_polishBase_q_, upFrame, resultJnt);        //    输入: 笛卡尔位姿 + 种子关节角
                                                   //    输出: 6个关节角

// 步骤2: 把关节角打包成 JointTrajectory 消息
trajectory_msgs::msg::JointTrajectory traj_goal;
JointTrajectoryPoint point;
for (i = 0; i < 6; i++)
    point.positions.push_back(resultJnt(i));      // ← 这里已经是关节角了
traj_goal.points.push_back(point);

// 步骤3: 发出去
ys_traj_publisher_->publish(traj_goal);           // ← 发给驱动执行
```

**关键事实**：状态机发给驱动的永远是**关节角度**，不是笛卡尔坐标。IK 在 `publish` 之前就完成了。

### 1.3 为什么这样做就能独立于机器人

```
┌──────────────────────────────────────────────────────────┐
│                                                          │
│   状态机的职责边界                                        │
│                                                          │
│   ┌─────────────────────────┐                            │
│   │ 笛卡尔路径 → IK → 关节角 │  ← 这部分和机器人型号相关   │
│   │ (TRAC-IK + URDF)        │    但只需要替换 URDF        │
│   └───────────┬─────────────┘                            │
│               │                                          │
│               │ JointTrajectory (关节角)                  │
│               ▼                                          │
│   ┌─────────────────────────┐                            │
│   │ 发送给驱动执行            │  ← 这部分和机器人型号无关   │
│   │ (Topic / Action 都一样)  │    只是通信协议不同         │
│   └─────────────────────────┘                            │
│                                                          │
└──────────────────────────────────────────────────────────┘
```

状态机下层只需要提供两个东西：

| 需要的东西 | ysrob (UR5) | Elite (CS66) |
|---|---|---|
| 接收关节角并执行 | `/YsUR_driver/joint_trajectory` (Topic) | `/scaled_joint_trajectory_controller/follow_joint_trajectory` (Action) |
| 提供机器人模型 (URDF) 用于 IK | `ys_ur5_ftsensor_polish.xacro` | `kybot_elite_robot_cell` URDF |

### 1.4 为什么不能直接用 Elite 的 Python IK

Elite 项目确实有自己的一套 Python IK（`robot_cartesian_control.py` 中的 `cs66_inverse_kinematics_5dof()`），但如果在 C++ 状态机中调用它：

```
方案 A: 把 Python IK 翻译成 C++
  → 工作量大，没必要重复造轮子

方案 B: C++ 通过 ROS2 Service 调 Python IK
  → 每次逆解需要网络往返 (1-5ms)
  → 状态机每 4ms 一个 tick，延迟不可接受

方案 C: 用 TRAC-IK（C++库）+ CS66 URDF
  → 零额外延迟，和原来完全一样的用法
  → 只需改两行 chain 名字
```

**选 C**。TRAC-IK 是通用求解器，和机器人型号无关。只要给它正确的 URDF 和运动学链，它就能解出正确的关节角。

### 1.5 TRAC-IK 的适配：只改两个字符串

```cpp
// ysrob 原版 (第 89-93 行):
KDL::Tree kdl_tree;
kdl_parser::treeFromUrdfModel(robot_model, kdl_tree);
std::string ys_base = ys_prefix_ + "base";          // "ys_base"
std::string ys_tip_link = ys_prefix_ + "tool0";     // "ys_tool0"
kdl_tree.getChain(ys_base, ys_tip_link, chain);     // chain: base → tool0

// 改为 Elite:
std::string base = cs66_prefix_ + "base_link";      // "cs66_base_link"
std::string tip  = cs66_prefix_ + "tool0";          // "cs66_tool0"
kdl_tree.getChain(base, tip, chain);                // chain: base_link → tool0
```

其余代码（`CartToJnt()` 调用、路径生成、力控调整）完全不变。

---

## 二、接口适配：桥接节点

### 2.1 问题

| | ysrob | Elite |
|---|---|---|
| 接口类型 | **Topic** `trajectory_msgs/JointTrajectory` | **Action** `control_msgs/FollowJointTrajectory` |
| 地址 | `/YsUR_driver/joint_trajectory` | `/scaled_joint_trajectory_controller/follow_joint_trajectory` |

### 2.2 方案：写一个 50 行的桥接节点

```
ysURForceAppControl (不变)
        │
        │ publish(JointTrajectory)    ← Topic
        ▼
 elite_joint_trajectory_bridge        ← ★ 唯一新增代码
        │
        │ async_send_goal()           ← Action
        ▼
/scaled_joint_trajectory_controller
```

**ysrob 状态机里所有的 `ys_traj_publisher_->publish(traj_goal)` 一行都不需要改。**

### 2.3 桥接节点的伪代码

```cpp
class EliteJointTrajectoryBridge : public rclcpp::Node {
    // 1. 订阅和 ysrob 驱动完全一样的 topic
    sub_ = create_subscription<JointTrajectory>(
        "/YsUR_driver/joint_trajectory", 1,
        [this](JointTrajectory::SharedPtr msg) {
            FollowJT::Goal goal;
            goal.trajectory = *msg;
            action_client_->async_send_goal(goal);
        });

    // 2. Action client 连到 Elite 的 controller
    action_client_ = ActionClient<FollowJT>::create(
        this, "/scaled_joint_trajectory_controller/follow_joint_trajectory");
};
```

---

## 三、完整移植步骤

### 前置验证（30 分钟，不写代码）

```bash
# 0. 激活轨迹控制器（每次重启驱动后需要执行）
ros2 control set_controller_state scaled_joint_trajectory_controller activate

# 1. 确认 Elite 的 JointTrajectory Action 可用
ros2 action send_goal /scaled_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{...}"

# 2. 确认力传感器数据
ros2 topic echo /force_torque_sensor_broadcaster/wrench --once
# 输出为 geometry_msgs/WrenchStamped，frame_id: cs66_tool0

# 3. 确认 RealSense 点云
# 启动相机时需要显式开启点云:
ros2 launch realsense2_camera rs_launch.py \
  pointcloud.enable:=true \
  camera_namespace:=camera
# 验证:
ros2 topic echo /camera/camera/depth/color/points --once
```

---

### 第一步：复制 TRAC-IK 到 elite_ws

```bash
cp -r ~/Documents/ysrob_ws/src/trac_ik ~/Documents/elite_robot_ws/src/

cd ~/Documents/elite_robot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select trac_ik_lib
```

编译通过即完成。不需要改 TRAC-IK 源码。

---

### 第二步：创建 elite_polish_app 包 + 桥接节点

```
elite_robot_ws/src/elite_polish_app/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── elite_joint_trajectory_bridge.hpp
└── src/
    └── elite_joint_trajectory_bridge.cpp
```

从 ysrob 的 CMakeLists.txt 中摘出依赖项，替换为 Elite 环境下的包名。

验证：
```bash
# 发一个测试 JointTrajectory
ros2 topic pub /YsUR_driver/joint_trajectory trajectory_msgs/msg/JointTrajectory "{...}"
# 机器人应执行
```

---

### 第三步：复制打磨核心代码并全局替换

#### 3.1 从 ysrob 复制的文件

```
从: ysrob_ws/src/ys_app_polish/ys_force_app/
到: elite_robot_ws/src/elite_polish_app/

include/
  ysURForceAppControl.hpp
  ysCamera3DSolver.hpp
  pclCalcTransform.hpp          ← 视觉算法，和机器人无关
  pclTemplateAlign.hpp          ← 备用

src/
  ysURForceAppControl.cpp       ← 核心状态机
  ysURForceAppNode.cpp          ← 入口
  ysCamera3DSolver.cpp          ← 视觉解算
  ysAppCommandNode.cpp          ← 命令发布
  pclTemplateAlign.cpp          ← 独立工具
```

#### 3.2 全局字符串替换

| 搜索 | 替换为 | 说明 |
|---|---|---|
| `ys_shoulder_pan_joint` | `cs66_shoulder_pan_joint` | TF prefix |
| `ys_shoulder_lift_joint` | `cs66_shoulder_lift_joint` | |
| `ys_elbow_joint` | `cs66_elbow_joint` | |
| `ys_wrist_1_joint` | `cs66_wrist_1_joint` | |
| `ys_wrist_2_joint` | `cs66_wrist_2_joint` | |
| `ys_wrist_3_joint` | `cs66_wrist_3_joint` | |
| `"ys_base"` | `"cs66_base_link"` | IK chain root |
| `"ys_tool0"` | `"cs66_tool0"` | IK chain tip |
| `ys_prefix_ = "ys_"` | `cs66_prefix_ = "cs66_"` | 变量赋值 |
| `namespace ys_ur_robot` | `namespace elite_robot` | 命名空间 |
| `/ysrob_kw75/ysrob_fts_broadcaster/wrench` | `/force_torque_sensor_broadcaster/wrench` | 力传感器 topic |
| `/ys_forceapp_cmd` | `/elite_forceapp_cmd` | 命令 topic |
| `/ys_vision_job_cmd` | `/elite_vision_job_cmd` | 视觉命令 topic |
| `ys_vision_pose_broadcaster/pose` | `elite_vision_pose_broadcaster/pose` | 视觉结果 topic |
| `/camera/depth_registered/points` | `/camera/camera/depth/color/points` | 点云 topic |

#### 3.3 需要重新示教/标定的硬编码值

**关节角（需在 Elite 上逐点示教）**：

```cpp
// 原 UR5 值:
ys_home_q_ = {88.96, -122.02, 98.92,  -66.39, -88.56, 135.91}°;
ys_cameraCapture_q_ = {42.35, -131.42, 102.08, -94.75, -57.66, 98.55}°;
ys_polishBase_q_ = {86.34, -105.52, 120.04, -162.39, -86.85, 135.04}°;
// ↑↑↑ 这三个必须替换为 Elite CS66 对应姿态的关节角
```

**力传感器偏置（需重新标定）**：

```cpp
ys_bias_wrench_.force = KDL::Vector(-1.02, -3.25, 0.98);
// ↑ 空载状态下跑 200 个采样周期取平均
```

**重力补偿参数（需重新测量）**：

```cpp
ys_gravity_wrench_.force = KDL::Vector(-0.05, 0.19, -24.29);
ys_gravity_center_ = KDL::Vector(0.025, -0.034, 0.046);
// ↑ 打磨工具不同，质量/质心完全不同
```

---

### 第四步：适配视觉系统

`pclCalcTransform.hpp` 中的所有裁剪盒坐标是 UR5 坐标系下的。如果机器人基座和工件的相对位置变了，需要重新标定。

需重新标定的值：

```cpp
// getTargetCloud(): 工件区域裁剪盒
boxFilter.setMin(Eigen::Vector4f(-0.6, -1.4, -0.3, 1.0));  // ← 重标
boxFilter.setMax(Eigen::Vector4f( 0.4, -0.6,  0.4, 1.0));  // ← 重标

// getPlaneFrame(): 平面三个测点盒
minO.x = -0.115;  minO.z = -0.18;   // ← 重标
minX.x = -0.015;  minX.z = -0.18;   // ← 重标
minY.x = -0.115;  minY.z = -0.08;   // ← 重标

// getCurveCloud(): 圆弧区域裁剪盒
boxFilter.setMin(Eigen::Vector4f(-0.25, 0.24, -0.095, 1.0));  // ← 重标
boxFilter.setMax(Eigen::Vector4f( 0.25, 0.54, -0.045, 1.0));  // ← 重标

// getCurveFrame(): 圆弧参数
trans(2) = min.z + 0.9306 - 1.89539;  // 0.9306/1.89539: 同一工件不变
```

标定方法：工件放抛光位 → 用 RViz 看点云 → 在 `cs66_base_link` 下量出工件区域坐标。

---

### 第五步：清理 AGV 依赖

ysrob 的 `DO_AGV_GO_HOME` / `DO_AGV_GO_POLISH` 状态在 Elite 中不存在。

**最小改动方案**：在状态机中把 AGV 相关 case 改为直接跳转：

```cpp
// 原:
case AppCommand::DO_AGV_GO_POLISH:
    doAgvGoPolish();     // 发 WebSocket 命令给 AGV
    break;

// 改为: 直接跳过，进入下一步（视觉或抛光）
case AppCommand::DO_AGV_GO_POLISH:
    // Elite 无 AGV，直接标记完成
    app_cmd_ = AppCommand::DO_CAMERA_VISION_JOB;
    sub_step_ = app_cmd_ * 100;
    break;
```

抛光完成后的自动 AGV 回待命也去掉：
```cpp
// 原 polish_waitBackHome() 结尾:
app_cmd_ = AppCommand::DO_AGV_GO_HOME;  // ← 删掉
// 改为:
app_cmd_ = AppCommand::NOTHING;         // 抛光结束，等待下一条指令
```

---

### 第六步：编写 launch 文件

```python
# elite_robot_ws/src/elite_polish_app/launch/elite_polish.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='elite_polish_app',
            executable='elite_joint_trajectory_bridge',
            name='elite_joint_trajectory_bridge',
        ),
        Node(
            package='elite_polish_app',
            executable='eliteURForceAppControl',
            name='eliteURForceAppControl',
        ),
        Node(
            package='elite_polish_app',
            executable='eliteCamera3DSolver',
            name='eliteCamera3DSolver',
            parameters=[{'template_file': 'polish_feature0.pcd'}],
        ),
        Node(
            package='elite_polish_app',
            executable='eliteAppCommandNode',
            name='eliteAppCommandNode',
        ),
    ])
```

---

### 第七步：编译调试

```bash
cd ~/elite_robot_ws
source /opt/ros/humble/setup.bash

# 编译顺序：先 TRAC-IK，再打磨包
colcon build --packages-select trac_ik_lib
colcon build --packages-select elite_polish_app
source install/setup.bash
```

分节点验证：

```bash
# 1. 桥接节点
ros2 run elite_polish_app elite_joint_trajectory_bridge
# 发测试 JointTrajectory → 确认机器人动作

# 2. 状态机 + 桥接
ros2 run elite_polish_app eliteURForceAppControl
ros2 topic pub /elite_forceapp_cmd std_msgs/msg/Int32 "data: 0"  # GO_HOME
# 确认能走到 Home 位

# 3. 视觉解算
ros2 run elite_polish_app eliteCamera3DSolver
ros2 topic pub /elite_vision_job_cmd std_msgs/msg/Int32 "data: 1"
# 确认输出工件位姿
```

---

### 第八步：联调

```bash
# 1. 启动 Elite 驱动
ros2 launch kybot_elite_robot_cell_control elite_control.launch.py

# 2. 启动相机
ros2 launch realsense2_camera rs_launch.py

# 3. 启动打磨系统
ros2 launch elite_polish_app elite_polish.launch.py

# 4. 发送命令序列
ros2 topic pub /elite_forceapp_cmd std_msgs/msg/Int32 "data: 0"  # Home
ros2 topic pub /elite_forceapp_cmd std_msgs/msg/Int32 "data: 3"  # 视觉定位
ros2 topic pub /elite_forceapp_cmd std_msgs/msg/Int32 "data: 4"  # 力控抛光
```

---

## 四、文件对照总表

| 从 ysrob_ws | 到 elite_robot_ws | 改动程度 |
|---|---|---|
| `src/trac_ik/` (整个目录) | `src/trac_ik/` | **不改**（编译即可） |
| `ys_force_app/include/pclCalcTransform.hpp` | `elite_polish_app/include/pclCalcTransform.hpp` | 裁剪盒坐标重标定 |
| `ys_force_app/include/pclTemplateAlign.hpp` | `elite_polish_app/include/pclTemplateAlign.hpp` | **不改** |
| `ys_force_app/include/ysURForceAppControl.hpp` | `elite_polish_app/include/eliteURForceAppControl.hpp` | 全局替换 TF prefix、topic 名 |
| `ys_force_app/include/ysCamera3DSolver.hpp` | `elite_polish_app/include/eliteCamera3DSolver.hpp` | 改相机 topic |
| `ys_force_app/src/ysURForceAppControl.cpp` | `elite_polish_app/src/eliteURForceAppControl.cpp` | 全局替换 + 重新示教关节角 + 去 AGV |
| `ys_force_app/src/ysURForceAppNode.cpp` | `elite_polish_app/src/eliteURForceAppNode.cpp` | 改类名 |
| `ys_force_app/src/ysCamera3DSolver.cpp` | `elite_polish_app/src/eliteCamera3DSolver.cpp` | 改相机 topic、TF prefix |
| `ys_force_app/src/ysAppCommandNode.cpp` | `elite_polish_app/src/eliteAppCommandNode.cpp` | 改 topic 名 |
| `ys_force_app/src/pclTemplateAlign.cpp` | `elite_polish_app/src/pclTemplateAlign.cpp` | **不改** |
| *(新建)* | `elite_polish_app/src/elite_joint_trajectory_bridge.cpp` | **新写** (~50 行) |
| *(新建)* | `elite_polish_app/launch/elite_polish.launch.py` | **新写** |
| `etc/polish_feature0.pcd` | `elite_polish_app/share/polish_feature0.pcd` | **不改**（同一工件可复用） |
| **不复制** | `kwr_ftsensor_ros2/` | Elite 内置 F/T |
| **不复制** | `percipioxyz_3dcamera_ros2/` | Elite 用 RealSense |
| **不复制** | `hand_eye_ros2/` | Elite 已有 |
| **不复制** | `ysrob_agv_task_client/` | Elite 无 AGV |
| **不复制** | `ys_ur5_polish_robot/` | Elite 有自己的 URDF + 驱动 |

---

## 五、风险清单

| 风险 | 影响 | 缓解 |
|---|---|---|
| 力传感器坐标系不同 | 重力补偿、偏置标定需重做 | 先不做重力补偿跑抛光，看力控行为 |
| 视觉裁剪盒坐标不一致 | 工件定位失败 | 在 RViz 中看点云，逐步调整盒子 |
| Elite FT 数据量程/精度与 KWR75 不同 | 力控参数 (target_fz_, adjust_dz_) 需重调 | 先保守参数（小 adjust_dz_，大 deadband） |
| 关节角示教位不准确 | 机械臂碰撞或无法到达工件 | 用 RViz 先模拟，再低速实际运动 |
| TRAC-IK 对 CS66 求解不稳定 | 部分姿态逆解失败 | 备选：通过 ROS2 service 调用 Elite Python IK |

---

## 六、工作量估算

| 步骤 | 难度 | 时间 |
|---|---|---|
| 前置验证 | 低 | 0.5h |
| 复制 TRAC-IK | 低 | 0.5h |
| 桥接节点 | 低 | 1h |
| 核心代码适配（全局替换） | 低 | 1h |
| 关节角示教 | 中 | 2h |
| 力传感器适配 | 中 | 2h |
| 视觉裁剪盒标定 | 中 | 2h |
| 去 AGV | 低 | 0.5h |
| launch 文件 | 低 | 0.5h |
| 编译 + 单元调试 | 中 | 2h |
| 联调 | 高 | 4h+ |
| **总计** | | **约 2-3 天** |

---

## 七、核心设计决策总结

```
为什么不重写 IK？
  → 因为状态机发的永远是关节角，IK 在 publish 前已完成
  → TRAC-IK 是通用求解器，换 URDF 就能适配不同机器人
  → 下层只需要"收关节角并执行"

为什么不改状态机逻辑？
  → 加桥接节点屏蔽 Topic/Action 接口差异
  → 其余全是字符串替换（TF prefix、topic 名、关节名）

真正需要"重做"的只有：
  1. 关节角示教（三个姿态点）
  2. 力传感器参数标定（偏置、重力补偿）
  3. 视觉裁剪盒坐标（如果工件位置变了）
```
