#!/bin/bash

# 创建会话和第一个窗口
tmux new-session -d -s ros_session -n main_nodes

# Pane 0: roscore
tmux send-keys -t ros_session:0 'roscore' C-m

# Pane 1: location.launch
tmux split-window -h -t ros_session:0
tmux send-keys -t ros_session:0.1 'sleep 3; roslaunch abot_bringup location.launch' C-m

# Pane 2: simple_camera_driver.launch
tmux split-window -v -t ros_session:0.1
tmux send-keys -t ros_session:0.2 'sleep 4; roslaunch tutorial_vision simple_camera_driver.launch' C-m

# Pane 3: ros_predict.launch
tmux split-window -v -t ros_session:0.2
tmux send-keys -t ros_session:0.3 'sleep 5; roslaunch yolov8_ros ros_predict.launch' C-m

# Pane 4: web_viewer
tmux split-window -v -t ros_session:0.3
tmux send-keys -t ros_session:0.4 'sleep 5; rosrun web_video_server web_video_server http://192.168.123.183:8080/stream?topic=/yolov8/detection_image' C-m

tmux select-layout -t ros_session:0 tiled

# --------------------
# 第二窗口（监控和任务）
# --------------------
tmux new-window -t ros_session:1 -n monitors_mission

# Pane 0: /mavros/local_position/pose
tmux send-keys -t ros_session:1 'sleep 6; rostopic echo /mavros/local_position/pose' C-m

# Pane 1: /object_position
tmux split-window -h -t ros_session:1
tmux send-keys -t ros_session:1.1 'sleep 6; rostopic echo /object_position' C-m

# Pane 2: check_camera.launch
tmux split-window -v -t ros_session:1.1
tmux send-keys -t ros_session:1.2 'sleep 7; roslaunch complete_mission check_camera.launch' C-m

# 整理第二个窗口布局
tmux select-layout -t ros_session:1 tiled

# 附加到会话并显示第一个窗口
tmux select-window -t ros_session:0
tmux attach-session -t ros_session
