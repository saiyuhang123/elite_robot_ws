#!/usr/bin/env python3
"""Qwen 大模型感知 + YOLO 后端切换 Launch。

用法：
  # 只启动 Qwen 感知 + 切换器（YOLO 需另行启动）
  ros2 launch qwen_vision qwen_vision.launch.py backend:=qwen run_yolo:=false

  # 同时启动 YOLO 感知，默认用 Qwen，运行时随时切回 YOLO
  ros2 launch qwen_vision qwen_vision.launch.py backend:=qwen run_yolo:=true

运行前：
  - 机械臂驱动已启动（提供 cs66_base_link -> cs66_tool0 TF）
  - 图漾相机已启动（/camera/color/image_raw 等三话题）
  - biaoding/hand_eye_result.json 为最新标定
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('qwen_vision'),
        'config', 'qwen_vision.yaml')

    yolo_dir = os.path.expanduser('~/Documents/elite_robot_ws/YOLO')
    yolo_script = os.path.join(yolo_dir, 'yolo_grasp_perception.py')

    backend = LaunchConfiguration('backend')
    run_yolo = LaunchConfiguration('run_yolo')

    qwen_node = Node(
        package='qwen_vision',
        executable='qwen_perception_node',
        name='qwen_perception',
        output='screen',
        parameters=[params_file],
    )

    switch_node = Node(
        package='qwen_vision',
        executable='vision_backend_switch',
        name='vision_backend_switch',
        output='screen',
        parameters=[{'default_backend': backend}],
    )

    yolo_perception = ExecuteProcess(
        cmd=['python3', yolo_script],
        cwd=yolo_dir,
        output='screen',
        name='yolo_grasp_perception',
        condition=IfCondition(run_yolo),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'backend', default_value='qwen',
            description='初始后端: qwen 或 yolo'),
        DeclareLaunchArgument(
            'run_yolo', default_value='true',
            description='是否同时启动现有 YOLO 感知节点'),

        LogInfo(msg='Qwen 大模型感知启动中...'),
        LogInfo(msg='切换: ros2 topic pub /vision_backend std_msgs/msg/String '
                    '"data: yolo|qwen" --once'),
        qwen_node,
        switch_node,
        # YOLO 模型加载慢，稍后启动，避免抢占 Qwen 初始化
        TimerAction(period=2.0, actions=[yolo_perception]),
    ])
