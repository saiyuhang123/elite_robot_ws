# ===== 终端1：启动机械臂驱动 =====
cd ~/Documents/elite_robot_ws
source install/setup.bash
ros2 launch my_elite_robot_cell_control start_robot.launch.py headless_mode:=true launch_rviz:=false

# ===== 终端2：启动 MoveIt move_group =====
cd ~/Documents/elite_robot_ws
source install/setup.bash
ros2 launch my_elite_robot_cell_moveit_config move_group.launch.py

# ===== 终端3：启动运动服务（MoveJ/MoveL） =====
cd ~/Documents/elite_robot_ws
source install/setup.bash
ros2 run hello_moveit grasp_move_server

# ===== 终端4：手动输入坐标点测试 =====
cd ~/Documents/elite_robot_ws
source install/setup.bash
ros2 run hello_moveit manual_move_client.py