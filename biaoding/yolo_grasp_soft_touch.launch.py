#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO 视觉抓取一体化 Launch 文件（柔触三指气动夹爪版）。

一键启动：机械臂驱动 + Percipio 相机 + 柔触夹爪 gripper_server + YOLO 感知节点。

启动前确保：
  1. 柔触夹爪控制器网络可达（Modbus TCP，默认 192.168.1.194:502）：
     ping 192.168.1.194
  2. gripper_control 包已编译（colcon build --packages-select gripper_control）
  3. 手眼标定已完成且 biaoding/hand_eye_result.json 是最新的
  4. 环境变量 ROS_DOMAIN_ID=42、RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 已设（.bashrc）

用法：
  source /opt/ros/humble/setup.bash
  source ~/Documents/elite_robot_ws/install/setup.bash
  ros2 launch ~/Documents/elite_robot_ws/biaoding/yolo_grasp_soft_touch.launch.py

  YOLO 感知启动后会自动在新终端窗口中启动抓取主程序（交互式）：
  cd ~/Documents/elite_robot_ws/biaoding && python3 yolo_grasp.py --gripper soft_touch

  全程调度（导航中断抓瓶）建议 headless：
  ros2 launch ~/Documents/elite_robot_ws/biaoding/yolo_grasp_soft_touch.launch.py \
      grasp_headless:=true

  夹爪控制器地址不同时：
  ros2 launch ~/Documents/elite_robot_ws/biaoding/yolo_grasp_soft_touch.launch.py \
      gripper_device_ip:=192.168.1.194 gripper_device_port:=502

远程控制（无需终端交互，服务与二指版完全一致）：
  # 触发抓取（单机调试：抓完自动松开）
  ros2 service call /yolo_grasp/grasp std_srvs/srv/Trigger

  # 触发抓取并保持夹持（全程调度用，等 place 才松手）
  ros2 service call /yolo_grasp/grasp_hold std_srvs/srv/Trigger

  # 张开 / 闭合夹爪
  ros2 service call /yolo_grasp/open std_srvs/srv/Trigger
  ros2 service call /yolo_grasp/close std_srvs/srv/Trigger

  # 放置（需先按 j 示教 place_pose.json）
  ros2 service call /yolo_grasp/place std_srvs/srv/Trigger

  # 回零位 / 回 Home2 收拢位
  ros2 service call /yolo_grasp/home std_srvs/srv/Trigger
  ros2 service call /yolo_grasp/home2 std_srvs/srv/Trigger

  # 查看当前目标状态
  ros2 service call /yolo_grasp/status std_srvs/srv/Trigger

参数：
  headless_mode    机械臂无头模式（默认 true）
  launch_rviz      启动 RViz（默认 false）
  run_perception   启动 YOLO 感知节点（默认 true）
  run_gripper      启动柔触夹爪 gripper_server 节点（默认 true）
  run_camera       启动 Percipio 相机（默认 true）
  run_grasp_main   自动启动抓取主程序 yolo_grasp.py --gripper soft_touch（默认 true）
  grasp_headless   抓取主程序无交互（默认 false；全程调度建议 true）
  gripper_device_ip   柔触夹爪控制器 Modbus TCP 地址（默认 192.168.1.194）
  gripper_device_port 柔触夹爪控制器 Modbus TCP 端口（默认 502）
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ============================================================
    # 声明 launch 参数
    # ============================================================
    declared_arguments = [
        DeclareLaunchArgument(
            "headless_mode",
            default_value="true",
            description="机械臂无头模式",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            description="是否启动 RViz",
        ),
        DeclareLaunchArgument(
            "run_perception",
            default_value="true",
            description="是否启动 YOLO 感知节点",
        ),
        DeclareLaunchArgument(
            "run_gripper",
            default_value="true",
            description="是否启动柔触夹爪 gripper_server 节点",
        ),
        DeclareLaunchArgument(
            "run_camera",
            default_value="true",
            description="是否启动 Percipio 相机",
        ),
        DeclareLaunchArgument(
            "run_grasp_main",
            default_value="true",
            description="是否自动启动抓取主程序 yolo_grasp.py --gripper soft_touch",
        ),
        DeclareLaunchArgument(
            "grasp_headless",
            default_value="false",
            description="抓取主程序无交互（全程调度建议 true）",
        ),
        DeclareLaunchArgument(
            "gripper_device_ip",
            default_value="192.168.1.194",
            description="柔触夹爪控制器 Modbus TCP 地址",
        ),
        DeclareLaunchArgument(
            "gripper_device_port",
            default_value="502",
            description="柔触夹爪控制器 Modbus TCP 端口",
        ),
    ]

    headless_mode = LaunchConfiguration("headless_mode")
    launch_rviz = LaunchConfiguration("launch_rviz")
    run_perception = LaunchConfiguration("run_perception")
    run_gripper = LaunchConfiguration("run_gripper")
    run_camera = LaunchConfiguration("run_camera")
    run_grasp_main = LaunchConfiguration("run_grasp_main")
    grasp_headless = LaunchConfiguration("grasp_headless")
    gripper_device_ip = LaunchConfiguration("gripper_device_ip")
    gripper_device_port = LaunchConfiguration("gripper_device_port")

    # ============================================================
    # 1. 机械臂驱动（无头模式）
    # ============================================================
    robot_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("my_elite_robot_cell_control"), "launch"]
                ),
                "/start_robot.launch.py",
            ]
        ),
        launch_arguments={
            "headless_mode": headless_mode,
            "launch_rviz": launch_rviz,
        }.items(),
    )

    # ============================================================
    # 2. Percipio 相机（彩色 1280x960 + 深度 1280x960，registration 默认开，
    #    深度图已对齐到彩色坐标系，与手眼标定分辨率一致）
    # ============================================================
    percipio_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("percipio_camera"), "launch"]
                ),
                "/percipio_camera.launch.py",
            ]
        ),
        launch_arguments={
            # 必须显式保持为空，让驱动自动发现相机。不能与夹爪 IP 共用参数名。
            "device_ip": "",
            "color_resolution": "1280x960",
            "depth_resolution": "1280x960",
        }.items(),
        condition=IfCondition(run_camera),
    )

    # ============================================================
    # 3. 柔触三指气动夹爪控制节点（gripper_server，Modbus TCP）
    #    gripper_control 包在本工作区，服务名 /gripper_command。
    #    独立终端窗口启动：10Hz 气压轮询日志量大，避免刷爆主终端
    # ============================================================
    soft_touch_gripper = ExecuteProcess(
        cmd=[
            "gnome-terminal",
            "--",
            "bash", "-c",
            [TextSubstitution(text="ros2 launch gripper_control "
                                   "soft_touch.launch.py device_ip:="),
             gripper_device_ip,
             TextSubstitution(text=" device_port:="),
             gripper_device_port,
             TextSubstitution(text="; exec bash")],
        ],
        output="screen",
        name="soft_touch_gripper",
        condition=IfCondition(run_gripper),
    )

    # ============================================================
    # 4. YOLO 感知节点（检测 + 3D 定位 → /target_object_pose）
    #    独立脚本，用 ExecuteProcess 在 YOLO 目录下运行。
    #    注：感知节点的 --mode 只影响深度处理策略（没有 soft_touch 档）；
    #    two_finger 模式（鲁棒深度 + 单帧发布）同样适用于柔触夹爪，
    #    与二指版完全一致，无需改动感知代码。
    # ============================================================
    yolo_dir = os.path.expanduser("~/Documents/elite_robot_ws/YOLO")
    yolo_perception_script = os.path.join(yolo_dir, "yolo_grasp_perception.py")

    yolo_perception = ExecuteProcess(
        cmd=["python3", yolo_perception_script,
             "--target-class", "bottle",
             "--mode", "two_finger"],
        cwd=yolo_dir,
        output="screen",
        name="yolo_grasp_perception",
        condition=IfCondition(run_perception),
    )

    # ============================================================
    # 5. 抓取主程序（--gripper soft_touch）
    #    抓取位姿与二指版走同一套代码：法兰 Z 竖直朝下、指尖对准目标点；
    #    仅夹爪自身参数不同（工具长度/偏移/IK 模式，见 grippers/soft_touch.py）
    #    交互模式：新终端窗口；headless 模式：直接后台运行
    # ============================================================
    grasp_main_dir = os.path.expanduser("~/Documents/elite_robot_ws/biaoding")
    grasp_main_script = os.path.join(grasp_main_dir, "yolo_grasp.py")

    grasp_cmd = (
        f"cd {grasp_main_dir} && python3 {grasp_main_script} "
        f"--gripper soft_touch --target-class bottle"
    )
    if grasp_headless == "true":
        grasp_cmd += " --headless"

    grasp_main = ExecuteProcess(
        cmd=[
            "gnome-terminal",
            "--",
            "bash", "-c",
            f"{grasp_cmd}; exec bash",
        ],
        output="screen",
        name="yolo_grasp_main",
        condition=IfCondition(run_grasp_main),
    )

    # ============================================================
    # 组装 LaunchDescription（按依赖顺序，带延时启动）
    #
    # 启动顺序：
    #   T+0s:  机械臂驱动 + 柔触夹爪 gripper_server（并行，互不依赖）
    #   T+3s:  Percipio 相机（需要在驱动之后以确保 TF 树完整）
    #   T+6s:  YOLO 感知节点（需要相机话题 + 驱动 TF 都就绪）
    #   T+9s:  抓取主程序（新终端窗口，等感知节点就绪）
    # ============================================================

    actions = [
        LogInfo(msg="=" * 60),
        LogInfo(msg="YOLO 抓取 Launch（柔触三指版）启动中..."),
        LogInfo(msg="  机械臂驱动 + 相机 + 柔触夹爪 gripper_server + YOLO 感知"),
        LogInfo(msg="  gripper_server 和抓取主程序各自在独立终端窗口启动"),
        LogInfo(msg="=" * 60),
        # 机械臂驱动 + 柔触夹爪并行启动
        robot_driver,
        soft_touch_gripper,
        # 相机在驱动后 3s 启动
        TimerAction(period=3.0, actions=[percipio_camera]),
        # YOLO 感知在相机后 3s 启动（总延时 6s）
        TimerAction(period=6.0, actions=[yolo_perception]),
        # 抓取主程序在感知后 3s 启动（总延时 9s，新终端窗口）
        TimerAction(period=9.0, actions=[grasp_main]),
        # 启动完成提示
        TimerAction(
            period=10.0,
            actions=[
                LogInfo(msg=""),
                LogInfo(msg="=" * 60),
                LogInfo(msg="全部节点已启动！"),
                LogInfo(msg="  抓取主程序已在新终端窗口自动启动"),
                LogInfo(msg="  （如需手动启动: python3 yolo_grasp.py --gripper soft_touch）"),
                LogInfo(msg=""),
                LogInfo(msg="  按键: g=抓取  o=张开  c=闭合  p=打印目标  h=回零"),
                LogInfo(msg="        j=示教放置位姿  l=放置  2=Home2  q=退出"),
                LogInfo(msg="=" * 60),
            ],
        ),
    ]

    return LaunchDescription(declared_arguments + actions)
