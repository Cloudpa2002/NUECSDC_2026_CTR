#!/bin/bash

# -------------------------------
# 配置区：按需修改路径和启动命令
# -------------------------------
# 加载工作空间（按优先级从低到高）
source /opt/ros/humble/setup.bash
source /home/elf/catkin_ws_livox_driver2/src/ws_livox/install/setup.bash
source /home/elf/catkin_ws_fastlio/install/setup.bash
source /home/elf/catkin_ws_ego_planner/install/setup.bash
source /home/elf/mnt_ws/vision_ws/install/setup.bash
source /home/elf/mnt_ws/UAVMain_ws/install/setup.bash
source /home/elf/mnt_ws/serial_ws/install/setup.bash
source /home/elf/mnt_ws/deepseek_ws/install/setup.bash

# 启动 deepseek
gnome-terminal --title="deepseek" -- bash -c \
  "source ~/mnt_ws/deepseek_ws/install/setup.bash && ros2 run deepseek_llm_ros deepseek_ros2; exec bash"
echo "[INFO] deepseek已启动"
sleep 1

# 启动 Lidar（显示终端）
gnome-terminal --title="Lidar Processing" -- bash -c \
  "ros2 launch livox_ros_driver2 msg_MID360_launch.py; exec bash"
echo "[INFO] 雷达节点已启动"
sleep 2

# 启动 fastlio（显示终端）
gnome-terminal --title="fastlio Processing" -- bash -c \
  "ros2 launch fast_lio mapping.launch.py; exec bash"
echo "[INFO] fastlio节点已启动"
sleep 3

# 启动 Mavros（显示终端）
gnome-terminal --title="Mavros" -- bash -c \
  "ros2 launch mavros px4.launch fcu_url:=serial:///dev/ttyS9:921600; exec bash"
echo "[INFO] mavros已启动"
sleep 3

# 启动 Odometry_trans（显示终端）
gnome-terminal --title="Odometry_trans" -- bash -c \
  "ros2 run joe_pkg mid_360; exec bash"
echo "[INFO] Odometry_trans已启动"
sleep 1

# 启动 ego_planner（显示终端）
gnome-terminal --title="ego_planner" -- bash -c \
  "ros2 launch ego_planner airdetect_single_run_in_sim.launch.py; exec bash"
echo "[INFO] ego_planner已启动"
sleep 1

# 启动 ego_planner RVIZ（显示终端）
gnome-terminal --title="ego_plannerRVIZ" -- bash -c \
  "ros2 launch ego_planner rviz.launch.py; exec bash"
echo "[INFO] ego_plannerRVIZ已启动"
sleep 3

