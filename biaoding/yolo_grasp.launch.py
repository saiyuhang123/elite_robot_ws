#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO 视觉抓取一体化 Launch 文件（LinkerHand O6 机械手版）。

一键启动：机械臂驱动 + RealSense 相机 + LinkerHand SDK + YOLO 感知节点。

启动前确保：
  1. CAN 端口已开启（每次插拔/重启后执行一次）：
     sudo /usr/sbin/ip link set can2 up type can bitrate 1000000
     ip link show can2   # 应显示 <UP,LOWER_UP>
  2. 手眼标定已完成且 biaoding/hand_eye_result.json 是最新的
  3. 环境变量 ROS_DOMAIN_ID=42、RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 已设（.bashrc）

用法：
  source /opt/ros/humble/setup.bash
  source ~/Documents/elite_robot_ws/install/setup.bash
  source ~/Documents/linker_hand_ros2_sdk/install/setup.bash
  ros2 launch ~/Documents/elite_robot_ws/biaoding/yolo_grasp.launch.py

  然后在另一个终端手动启动抓取主程序（交互式）：
  cd ~/Documents/elite_robot_ws/biaoding
  python3 yolo_grasp.py

  可选：查看 YOLO 调试图像
  cd ~/Documents/elite_robot_ws/YOLO
  python3 view_annotated.py

	远程控制（无需终端交互）：
	  # 切换目标类别（如 cup, bottle, all 等）
	  ros2 topic pub /yolo/target_class std_msgs/msg/String "data: 'cup'" -1

	  # 触发抓取
	  ros2 service call /yolo_grasp/grasp std_srvs/srv/Trigger

	  # 张开 / 闭合机械手
	  ros2 service call /yolo_grasp/open std_srvs/srv/Trigger
	  ros2 service call /yolo_grasp/close std_srvs/srv/Trigger

	  # 回零位
	  ros2 service call /yolo_grasp/home std_srvs/srv/Trigger

	  # 到抓取预备位姿
	  ros2 service call /yolo_grasp/ready std_srvs/srv/Trigger

	  # 查看当前目标状态
	  ros2 service call /yolo_grasp/status std_srvs/srv/Trigger

参数：
  headless_mode    机械臂无头模式（默认 true）
  launch_rviz      启动 RViz（默认 false）
  run_perception   启动 YOLO 感知节点（默认 true）
  run_linker_hand  启动 LinkerHand SDK（默认 true）
  run_camera       启动 RealSense 相机（默认 true）
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
            "run_linker_hand",
            default_value="true",
            description="是否启动 LinkerHand SDK",
        ),
        DeclareLaunchArgument(
            "run_camera",
            default_value="true",
            description="是否启动 RealSense 相机",
        ),
    ]

    headless_mode = LaunchConfiguration("headless_mode")
    launch_rviz = LaunchConfiguration("launch_rviz")
    run_perception = LaunchConfiguration("run_perception")
    run_linker_hand = LaunchConfiguration("run_linker_hand")
    run_camera = LaunchConfiguration("run_camera")

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
    # 2. RealSense D435 相机（彩色 + 深度 + 对齐）
    # ============================================================
    realsense_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("realsense2_camera"), "launch"]
                ),
                "/rs_launch.py",
            ]
        ),
        launch_arguments={
            "camera_namespace": "camera",
            "enable_color": "true",
            "enable_depth": "true",
            "rgb_camera.color_profile": "1280x720x30",
            "depth_module.depth_profile": "640x480x30",
            "align_depth.enable": "true",
        }.items(),
        condition=IfCondition(run_camera),
    )

    # ============================================================
    # 3. LinkerHand O6 SDK（机械手控制）
    #    —— 位于独立工作区，用 bash -c 先 source 再 ros2 launch
    # ============================================================
    linker_hand_ws = os.path.expanduser("~/Documents/linker_hand_ros2_sdk")
    linker_hand = ExecuteProcess(
        cmd=[
            "bash", "-c",
            f"source {linker_hand_ws}/install/setup.bash && "
            "ros2 launch linker_hand_ros2_sdk linker_hand.launch.py"
        ],
        output="screen",
        condition=IfCondition(run_linker_hand),
    )

    # ============================================================
    # 4. YOLO 感知节点（检测 + 3D 定位 → /target_object_pose）
    #    独立脚本，用 ExecuteProcess 在 YOLO 目录下运行
    # ============================================================
    yolo_dir = os.path.expanduser("~/Documents/elite_robot_ws/YOLO")
    yolo_perception_script = os.path.join(yolo_dir, "yolo_grasp_perception.py")

    yolo_perception = ExecuteProcess(
        cmd=["python3", yolo_perception_script],
        cwd=yolo_dir,
        output="screen",
        name="yolo_grasp_perception",
        condition=IfCondition(run_perception),
    )

    # ============================================================
    # 组装 LaunchDescription（按依赖顺序，带延时启动）
    #
    # 启动顺序：
    #   T+0s:  机械臂驱动 + LinkerHand SDK（并行，互不依赖）
    #   T+3s:  RealSense 相机（需要在驱动之后以确保 TF 树完整）
    #   T+6s:  YOLO 感知节点（需要相机话题 + 驱动 TF 都就绪）
    # ============================================================

    actions = [
        LogInfo(msg="=" * 60),
        LogInfo(msg="YOLO 抓取 Launch 启动中..."),
        LogInfo(msg="  机械臂驱动 + 相机 + LinkerHand SDK + YOLO 感知"),
        LogInfo(msg="  抓取主程序请在另一个终端手动启动: python3 yolo_grasp.py"),
        LogInfo(msg="=" * 60),
        # 机械臂驱动 + LinkerHand 并行启动
        robot_driver,
        linker_hand,
        # 相机在驱动后 3s 启动
        TimerAction(period=3.0, actions=[realsense_camera]),
        # YOLO 感知在相机后 3s 启动（总延时 6s）
        TimerAction(period=6.0, actions=[yolo_perception]),
        # 启动完成提示
        TimerAction(
            period=8.0,
            actions=[
                LogInfo(msg=""),
                LogInfo(msg="=" * 60),
                LogInfo(msg="全部节点已启动！"),
                LogInfo(msg="  在另一个终端启动抓取主程序："),
                LogInfo(msg="  cd ~/Documents/elite_robot_ws/biaoding"),
                LogInfo(msg="  python3 yolo_grasp.py"),
                LogInfo(msg=""),
                LogInfo(msg="  按键: g=抓取  o=张开手  c=闭合手"),
                LogInfo(msg="        p=打印目标  h=回零  q=退出"),
                LogInfo(msg="=" * 60),
            ],
        ),
    ]

    return LaunchDescription(declared_arguments + actions)
