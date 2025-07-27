#!/bin/bash

# 创建会话和第一个窗口
tmux new-session -d -s ros_session -n main_nodes

# Pane 0: roscore
tmux send-keys -t ros_session:0 'roscore' C-m

# Pane 1: beginning.launch
tmux split-window -h -t ros_session:0
tmux send-keys -t ros_session:0.1 'sleep 3; roslaunch abot_bringup location.launch' C-m

# Pane 2: /mavros/local_position/pose 监控
tmux split-window -v -t ros_session:0.1
tmux send-keys -t ros_session:0.2 'sleep 4; rostopic echo /mavros/local_position/pose' C-m

# Pane 3: complete_mission hover.launch
tmux split-window -v -t ros_session:0.2
tmux send-keys -t ros_session:0.3 'sleep 6; roslaunch complete_mission hover.launch' C-m

# 自动整理布局为平铺
tmux select-layout -t ros_session:0 tiled

# 附加到会话
tmux attach-session -t ros_session
