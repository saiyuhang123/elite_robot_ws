#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device_ip = DeclareLaunchArgument(
        "device_ip",
        default_value="192.168.1.194",
        description="柔触夹爪控制器的 Modbus TCP 地址",
    )
    device_port = DeclareLaunchArgument(
        "device_port",
        default_value="502",
        description="柔触夹爪控制器的 Modbus TCP 端口",
    )
    poll_rate_hz = DeclareLaunchArgument(
        "poll_rate_hz",
        default_value="10.0",
        description="气压反馈轮询频率",
    )

    node = Node(
        package="gripper_control",
        executable="gripper_server",
        name="gripper_server",
        output="screen",
        parameters=[{
            "device_ip": LaunchConfiguration("device_ip"),
            "device_port": LaunchConfiguration("device_port"),
            "poll_rate_hz": LaunchConfiguration("poll_rate_hz"),
        }],
    )

    return LaunchDescription([
        device_ip,
        device_port,
        poll_rate_hz,
        node,
    ])
