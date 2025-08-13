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
tmux send-keys -t ros_session:0.2 'sleep 4; source /home/phoenixtech/abot_ws/devel/setup.zsh; roslaunch complete_mission simple_camera_driver.launch' C-m

# Pane 3: yolov8_ros ros_predict.launch
tmux split-window -v -t ros_session:0.2
tmux send-keys -t ros_session:0.3 'sleep 5;source /home/phoenixtech/abot_ws/devel/setup.zsh; roslaunch yolov8_ros ros_predict.launch' C-m

# 整理布局
tmux select-layout -t ros_session:0 tiled


# --------------------
# 第二个窗口：相机与门检测
# --------------------
tmux new-window -t ros_session:1 -n camera_detection

tmux send-keys -t ros_session:1 'sleep 5;source /home/phoenixtech/abot_ws/devel/setup.zsh; rosrun web_video_server web_video_server' C-m

tmux split-window -h -t ros_session:1
tmux send-keys -t ros_session:1.1 'sleep 4; source ~/yzk_ws/devel/setup.bash; roslaunch door_detector door_detector.launch' C-m

tmux select-layout -t ros_session:1 tiled


# --------------------
# 第三个窗口：监控与任务
# --------------------
tmux new-window -t ros_session:2 -n monitors_mission

tmux send-keys -t ros_session:2 'sleep 6; rostopic echo /mavros/local_position/pose' C-m

tmux split-window -h -t ros_session:2
tmux send-keys -t ros_session:2.1 'sleep 6; rostopic echo /object_position' C-m

tmux split-window -h -t ros_session:2.1
tmux send-keys -t ros_session:2.2 'sleep 6; rostopic echo /door_center' C-m

tmux split-window -v -t ros_session:2.2
tmux send-keys -t ros_session:2.3 'sleep 7;source /home/phoenixtech/abot_ws/devel/setup.zsh; roslaunch complete_mission gjs.launch' C-m

tmux select-layout -t ros_session:2 tiled


# 进入会话
tmux select-window -t ros_session:0
tmux attach-session -t ros_session
