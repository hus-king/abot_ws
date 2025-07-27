#!/bin/bash

# 创建会话和第一个窗口
tmux new-session -d -s view_session -n lidar_view

# Pane 0: roscore
tmux send-keys -t view_session:0 'rosrun complete_mission laser_to_worldframe' C-m

# Pane 1: beginning.launch
tmux split-window -h -t view_session:0
tmux send-keys -t view_session:0.1 'sleep 3; roslaunch complete_mission ego_planner_mid360.launch' C-m

# 自动整理布局为平铺
tmux select-layout -t view_session:0 tiled

# 附加到会话
tmux attach-session -t view_session
