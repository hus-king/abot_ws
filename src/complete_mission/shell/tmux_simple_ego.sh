#!/usr/bin/env zsh
# 创建 tmux 会话和第一个窗口：主 ROS 节点
tmux new-session -d -s ros_session -n main_nodes

# Pane 0: roscore
tmux send-keys -t ros_session:0 'roscore' C-m

# Pane 1: location.launch
tmux split-window -h -t ros_session:0
tmux send-keys -t ros_session:0.1 'sleep 3; roslaunch fly_demo utils.launch' C-m

# Pane 2: simple_camera_driver.launch
tmux split-window -v -t ros_session:0.1
tmux send-keys -t ros_session:0.2 'sleep 4; rostopic echo /mavros/local_position/pose' C-m

# Pane 3: yolov8_ros ros_predict.launch
tmux split-window -v -t ros_session:0.2
tmux send-keys -t ros_session:0.3 'sleep 7;source /home/phoenixtech/abot_ws/devel/setup.zsh; roslaunch complete_mission simple_ego.launch' C-m

# 整理布局
tmux select-layout -t ros_session:0 tiled


# 进入会话
tmux select-window -t ros_session:0
tmux attach-session -t ros_session
