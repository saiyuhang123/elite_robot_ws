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
        Node(
            package='elite_polish_app',
            executable='ysURForceAppControl',
            name='ysURForceAppControl',
            output='screen',
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
