"""
Launch file for Elite CS66 force-controlled polishing system.
Ported from ysrob_ws.

Usage:
  ros2 launch elite_polish_app elite_polish.launch.py

Before launching, make sure:
  1. Elite robot driver is running (elite_control.launch.py)
  2. The trajectory controller is activated:
     ros2 control set_controller_state scaled_joint_trajectory_controller activate
  3. RealSense camera is running (if using vision):
     ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true camera_namespace:=camera
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Bridge: converts Topic (ysrob-style) to Action (Elite-style)
        Node(
            package='elite_polish_app',
            executable='elite_joint_trajectory_bridge',
            name='elite_joint_trajectory_bridge',
            output='screen',
        ),

        # Main state machine for force-controlled polishing
        # debug_skip_force_contact: true=空跑模式(402免接触、404力控旁路)，验证动作用；
        #                           真打磨必须改回 false！
        # contact_fz_threshold: 402 接触判定阈值(N,负值)，软工件/纸箱可调小(如 -6.0)
        Node(
            package='elite_polish_app',
            executable='ysURForceAppControl',
            name='ysURForceAppControl',
            output='screen',
            parameters=[{
                'debug_skip_force_contact': True,
                'contact_fz_threshold': -16.0,
            }],
        ),

        # Vision solver (required for command 3 / vision job; missing it blocks sub_step 302/303)
        # template_file 当前仅作调试点云（cameracapture.pcd/base.pcd）的保存路径前缀，
        # 实际不加载模板文件（加载代码在 ysCamera3DSolver.cpp 中被注释），故指向 /tmp/。
        Node(
            package='elite_polish_app',
            executable='ysCamera3DSolver',
            name='ysCamera3DSolver',
            output='screen',
            parameters=[{'template_file': '/tmp/'}],
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
