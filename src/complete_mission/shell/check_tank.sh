#!/bin/bash

# 创建会话和第一个窗口
tmux new-session -d -s ros_session -n main_nodes

# 第一个窗口 - 主要节点
# Pane 0: roscore
tmux send-keys -t ros_session:0 'roscore' C-m

# Pane 1: beginning.launch
tmux split-window -h -t ros_session:0
tmux send-keys -t ros_session:0.1 'sleep 4; roslaunch abot_bringup location.launch' C-m

# Pane 3: simple_camera_driver.launch
tmux split-window -v -t ros_session:0.1
tmux send-keys -t ros_session:0.2 'sleep 4; roslaunch tutorial_vision simple_camera_driver.launch' C-m

# Pane 4: yolo_v8.launch
tmux split-window -h -t ros_session:0.2
tmux send-keys -t ros_session:0.3 'sleep 4; roslaunch yolov8_ros ros_predict.launch' C-m

# 自动整理第一个窗口布局为平铺
tmux select-layout -t ros_session:0 tiled

# 创建第二个窗口 - 监控和调试
tmux new-window -t ros_session -n monitor_debug

# Pane 2: /mavros/local_position/pose 监控
tmux send-keys -t ros_session:1 'sleep 4; rostopic echo /mavros/local_position/pose' C-m

# Pane 5: rostopic echo /object_position
tmux split-window -h -t ros_session:1
tmux send-keys -t ros_session:1.1 'sleep 4; rostopic echo /object_position' C-m

# Pane 6: web_viewer
tmux split-window -v -t ros_session:1.1
tmux send-keys -t ros_session:1.2 'sleep 5; rosrun web_video_server web_video_server http://192.168.123.183:8080/stream?topic=/yolov8/detection_image' C-m

# Pane 7: complete_mission check_tank.launch
tmux split-window -h -t ros_session:1.2
tmux send-keys -t ros_session:1.3 'sleep 4; roslaunch complete_mission check_tank.launch' C-m

# 自动整理第二个窗口布局为平铺
tmux select-layout -t ros_session:1 tiled

# 默认选择第一个窗口
tmux select-window -t ros_session:0

# 附加到会话
tmux attach-session -t ros_session
