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

# Pane 3: simple_camera_driver.launch
tmux split-window -h -t ros_session:0.2
tmux send-keys -t ros_session:0.3 'sleep 4; roslaunch tutorial_vision simple_camera_driver.launch' C-m

# Pane 4: yolo_v8.launch
tmux split-window -v -t ros_session:0.3
tmux send-keys -t ros_session:0.4 'sleep 5; roslaunch yolov8_ros ros_predict.launch' C-m

# Pane 5: rostopic echo /object_position
tmux split-window -h -t ros_session:0.4
tmux send-keys -t ros_session:0.5 'sleep 6; rostopic echo /object_position' C-m

# Pane 6: complete_mission check_tank.launch
tmux split-window -v -t ros_session:0.5
tmux send-keys -t ros_session:0.6 'sleep 7; roslaunch complete_mission check_tank.launch' C-m

# 自动整理布局为平铺
tmux select-layout -t ros_session:0 tiled

# 附加到会话
tmux attach-session -t ros_session
