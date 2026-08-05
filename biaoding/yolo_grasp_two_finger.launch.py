#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO 视觉抓取一体化 Launch 文件（Inspire 4B4C 二指夹爪版）。

一键启动：机械臂驱动 + Percipio 相机 + Inspire 二指夹爪 + YOLO 感知节点。

启动前确保：
  1. CAN 端口已开启（每次插拔/重启后执行一次）：
     sudo /usr/sbin/ip link set can2 up type can bitrate 1000000
     ip link show can2   # 应显示 <UP,LOWER_UP>
  2. 手眼标定已完成且 biaoding/hand_eye_result.json 是最新的
  3. 环境变量 ROS_DOMAIN_ID=42、RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 已设（.bashrc）

用法：
  source /opt/ros/humble/setup.bash
  source ~/Documents/elite_robot_ws/install/setup.bash
  ros2 launch ~/Documents/elite_robot_ws/biaoding/yolo_grasp_two_finger.launch.py

  YOLO 感知启动后会自动在新终端窗口中启动抓取主程序（交互式）：
  cd ~/Documents/elite_robot_ws/biaoding && python3 yolo_grasp.py --gripper two_finger

  全程调度（导航中断抓瓶）建议 headless：
  ros2 launch ~/Documents/elite_robot_ws/biaoding/yolo_grasp_two_finger.launch.py \
      grasp_headless:=true

远程控制（无需终端交互）：
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
  run_gripper      启动 Inspire 二指夹爪节点（默认 true）
  run_camera       启动 Percipio 相机（默认 true）
  run_grasp_main   自动启动抓取主程序 yolo_grasp.py --gripper two_finger（默认 true）
  grasp_headless   抓取主程序无交互（默认 false；全程调度建议 true）
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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
            description="是否启动 Inspire 二指夹爪节点",
        ),
        DeclareLaunchArgument(
            "run_camera",
            default_value="true",
            description="是否启动 Percipio 相机",
        ),
        DeclareLaunchArgument(
            "run_grasp_main",
            default_value="true",
            description="是否自动启动抓取主程序 yolo_grasp.py --gripper two_finger",
        ),
        DeclareLaunchArgument(
            "grasp_headless",
            default_value="false",
            description="抓取主程序无交互（全程调度建议 true）",
        ),
    ]

    headless_mode = LaunchConfiguration("headless_mode")
    launch_rviz = LaunchConfiguration("launch_rviz")
    run_perception = LaunchConfiguration("run_perception")
    run_gripper = LaunchConfiguration("run_gripper")
    run_camera = LaunchConfiguration("run_camera")
    run_grasp_main = LaunchConfiguration("run_grasp_main")
    grasp_headless = LaunchConfiguration("grasp_headless")

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
            "color_resolution": "1280x960",
            "depth_resolution": "1280x960",
        }.items(),
        condition=IfCondition(run_camera),
    )

    # ============================================================
    # 3. Inspire 4B4C 二指夹爪控制节点
    #    与机械臂同工作区（elite_robot_ws），串口默认 /dev/ttyUSB0
    # ============================================================
    inspire_gripper = ExecuteProcess(
        cmd=["ros2", "run", "inspire_gripper", "Gripper_control_node"],
        output="screen",
        name="inspire_gripper",
        condition=IfCondition(run_gripper),
    )

    # ============================================================
    # 4. YOLO 感知节点（检测 + 3D 定位 → /target_object_pose）
    #    独立脚本，用 ExecuteProcess 在 YOLO 目录下运行
    # ============================================================
    yolo_dir = os.path.expanduser("~/Documents/elite_robot_ws/YOLO")
    yolo_perception_script = os.path.join(yolo_dir, "yolo_grasp_perception.py")

    yolo_perception = ExecuteProcess(
        cmd=["python3", yolo_perception_script,
             "--target-class", "bottle"],
        cwd=yolo_dir,
        output="screen",
        name="yolo_grasp_perception",
        condition=IfCondition(run_perception),
    )

    # ============================================================
    # 5. 抓取主程序（--gripper two_finger）
    #    交互模式：新终端窗口；headless 模式：直接后台运行
    # ============================================================
    grasp_main_dir = os.path.expanduser("~/Documents/elite_robot_ws/biaoding")
    grasp_main_script = os.path.join(grasp_main_dir, "yolo_grasp.py")

    grasp_cmd = (
        f"cd {grasp_main_dir} && python3 {grasp_main_script} "
        f"--gripper two_finger --target-class bottle"
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
    #   T+0s:  机械臂驱动 + Inspire 夹爪（并行，互不依赖）
    #   T+3s:  Percipio 相机（需要在驱动之后以确保 TF 树完整）
    #   T+6s:  YOLO 感知节点（需要相机话题 + 驱动 TF 都就绪）
    #   T+9s:  抓取主程序（新终端窗口，等感知节点就绪）
    # ============================================================

    actions = [
        LogInfo(msg="=" * 60),
        LogInfo(msg="YOLO 抓取 Launch（二指版）启动中..."),
        LogInfo(msg="  机械臂驱动 + 相机 + Inspire 二指夹爪 + YOLO 感知"),
        LogInfo(msg="  抓取主程序将在新终端窗口自动启动"),
        LogInfo(msg="=" * 60),
        # 机械臂驱动 + Inspire 夹爪并行启动
        robot_driver,
        inspire_gripper,
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
                LogInfo(msg="  （如需手动启动: python3 yolo_grasp.py --gripper two_finger）"),
                LogInfo(msg=""),
                LogInfo(msg="  按键: g=抓取  o=张开  c=闭合  p=打印目标  h=回零"),
                LogInfo(msg="        j=示教放置位姿  l=放置  2=Home2  q=退出"),
                LogInfo(msg="=" * 60),
            ],
        ),
    ]

    return LaunchDescription(declared_arguments + actions)
