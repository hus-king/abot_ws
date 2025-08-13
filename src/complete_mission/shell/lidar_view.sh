#!/bin/bash

# 创建会话和第一个窗口
tmux new-session -d -s view_session -n lidar_view

# Pane 0: roscore
tmux send-keys -t view_session:0 'rosrun complete_mission laser_to_worldframe' C-m

# Pane 1: beginning.launch
tmux split-window -h -t view_session:0
tmux send-keys -t view_session:0.1 'sleep 3; roslaunch complete_mission ego_planner_mid360.launch' C-m

# Pane 1: rosbag record
tmux split-window -h -t view_session:0.1
tmux send-keys -t view_session:0.2 'cd ~/rosbag ; rosbag record /ego_planner_node/grid_map/occupancy_inflate /cloud /Laser_map /Odometry /path /ego_planner_node/goal_point /ego_planner_node/optimal_list /clicked_point /ego_planner/goal /mavros/local_position/odom /mavros/local_position/pose' C-m

# 自动整理布局为平铺
tmux select-layout -t view_session:0 tiled

# 附加到会话
tmux attach-session -t view_session
