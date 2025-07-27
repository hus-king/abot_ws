#!/bin/bash

# 创建会话和第一个窗口
tmux new-session -d -s ros_session -n main_nodes

# Pane 0: roscore
tmux send-keys -t ros_session:0 'roscore' C-m

# Pane 1: beginning.launch
tmux split-window -h -t ros_session:0
tmux send-keys -t ros_session:0.1 'sleep 3; roslaunch abot_bringup location.launch' C-m


# Pane 3: simple_camera_driver.launch
tmux split-window -h -t ros_session:0.1
tmux send-keys -t ros_session:0.2 'sleep 4; roslaunch tutorial_vision simple_camera_driver.launch' C-m

# Pane 4: rs_camera.launch
tmux split-window -h -t ros_session:0.2
tmux send-keys -t ros_session:0.3 'sleep 4; roslaunch realsense2_camera rs_camera.launch' C-m


tmux select-layout -t ros_session:0 tiled

tmux new-window -t ros_session:1 -n main_nods2

# Pane 5: yolo_v8.launch
tmux send-keys -t ros_session:1 'sleep 4; roslaunch yolov8_ros yolo_v8.launch' C-m

# Pane 6: qr_detection
tmux split-window -h -t ros_session:1
tmux send-keys -t ros_session:1.1 'sleep 4; roslaunch tutorial_vision qr_detction_craic.launch' C-m

# Pane 7: circle_detector.launch
tmux split-window -h -t ros_session:1.1
tmux send-keys -t ros_session:1.2 'sleep 4; roslaunch circle_detector circle_detector.launch' C-m

# 自动整理布局
tmux select-layout -t ros_session:1 tiled

# --------------------
# 第二窗口（监控）四宫格
# --------------------
tmux new-window -t ros_session:2 -n monitors

# Pane 0: /mavros/local_position/pose
tmux send-keys -t ros_session:2 'sleep 4; rostopic echo /mavros/local_position/pose' C-m

# Pane 1: /circle_detector/position
tmux split-window -h -t ros_session:2
tmux send-keys -t ros_session:2.1 'sleep 4; rostopic echo /circle_detector/position' C-m

# Pane 2: /qr_code_detection/qrcode_data
tmux split-window -v -t ros_session:2.0
tmux send-keys -t ros_session:2.1 'sleep 4; rostopic echo /qrcode_data' C-m

# Pane 3: complete_mission.launch
tmux split-window -v -t ros_session:2.1
tmux send-keys -t ros_session:2.2 'sleep 4; roslaunch complete_mission complete_mission_ego.launch' C-m

# 整理布局
tmux select-layout -t ros_session:2 tiled

# 附加到会话并显示第一个窗口
tmux select-window -t ros_session:0
tmux attach-session -t ros_session
