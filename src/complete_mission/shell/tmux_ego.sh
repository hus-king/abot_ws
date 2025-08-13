tmux new-session -d -s ros_session
tmux send-keys -t ros_session 'roscore' C-m
tmux send-keys -t ros_session 'exec bash' C-m

# 创建第一个水平分割窗格
tmux split-window -h -t ros_session
tmux send-keys -t ros_session.1 'sleep 3' C-m
tmux send-keys -t ros_session.1 'exec bash' C-m

# 创建第二个垂直分割窗格
tmux split-window -h -t ros_session.1
tmux send-keys -t ros_session.2 'sleep 4; rosrun complete_mission laser_to_worldframe' C-m
tmux send-keys -t ros_session.2 'exec bash' C-m

# 创建第三个垂直分割窗格
tmux split-window -h -t ros_session.2
tmux send-keys -t ros_session.3 'sleep 4; roslaunch complete_mission ego_planner_mid360.launch' C-m
tmux send-keys -t ros_session.3 'exec bash' C-m


tmux setw pane-base-index 0
tmux select-layout even-horizontal


tmux attach-session -t ros_session