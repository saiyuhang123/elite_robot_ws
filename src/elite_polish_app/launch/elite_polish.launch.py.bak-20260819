"""
Launch file for Elite CS66 force-controlled polishing system.
Ported from ysrob_ws.

Usage:
  ros2 launch elite_polish_app elite_polish.launch.py

Before launching, make sure:
  1. Elite robot driver is running (my_elite_robot_cell_control start_robot.launch.py)
  2. The trajectory controller is active:
     ros2 control set_controller_state scaled_joint_trajectory_controller active
  3. RealSense camera is running (if using vision):
     ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true camera_namespace:=camera

All tunable parameters (crop boxes, hand-eye, force control, taught poses, debug switches)
live in: src/elite_polish_app/config/polish_params.yaml
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # 统一参数文件（两个节点各自读取同名 section）
    params_file = PathJoinSubstitution([
        FindPackageShare('elite_polish_app'), 'config', 'polish_params.yaml'
    ])

    return LaunchDescription([
        # Bridge: converts Topic (ysrob-style) to Action (Elite-style)
        Node(
            package='elite_polish_app',
            executable='elite_joint_trajectory_bridge',
            name='elite_joint_trajectory_bridge',
            output='screen',
        ),

        # Main state machine for force-controlled polishing
        Node(
            package='elite_polish_app',
            executable='ysURForceAppControl',
            name='ysURForceAppControl',
            output='screen',
            parameters=[params_file],
        ),

        # Vision solver (required for command 3 / vision job; missing it blocks sub_step 302/303)
        # 2026-07-31: 相机已从 RealSense D435 换成 Camport(Percipio)，
        # 点云话题重映射 /camera/depth/points；换回 RealSense 时删掉 remappings 即可。
        Node(
            package='elite_polish_app',
            executable='ysCamera3DSolver',
            name='ysCamera3DSolver',
            output='screen',
            parameters=[params_file],
            remappings=[('/camera/camera/depth/color/points', '/camera/depth_registered/points')],
        ),

        # Interactive command line interface
        Node(
            package='elite_polish_app',
            executable='ysAppCommand',
            name='ysAppCommand',
            output='screen',
            prefix='x-terminal-emulator -e',  # Opens in new terminal for stdin
        ),
    ])
