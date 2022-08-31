#!/bin/bash

# Create a new tmux session
session_name="vint_rosys_$(date +%s)"
tmux new-session -d -s $session_name

# Split the window into four panes
tmux selectp -t 0    # select the first (0) pane
tmux splitw -h -p 50 # split it into two halves
tmux selectp -t 0    # select the first (0) pane
tmux splitw -v -p 50 # split it into two halves

tmux selectp -t 2    # select the new, second (2) pane
tmux splitw -v -p 50 # split it into two halves

tmux selectp -t 1    # select the  pane
tmux splitw -v -p 50 # split it into two halves



tmux select-pane -t 0
tmux send-keys "ssh rosys@192.168.0.100" Enter
tmux send-keys "roslaunch usb_cam usb_cam-test.launch" Enter


tmux select-pane -t 1
tmux send-keys "ssh rosys@192.168.0.100" Enter
tmux send-keys "rosrun topic_tools throttle messages /usb_cam/image_raw 4.0 /image_throttle" Enter

tmux select-pane -t 2
tmux send-keys "conda activate vint_deployment_11" Enter
tmux send-keys "cd ~/vint_ws/src/visualnav-transformer/deployment/src" Enter
tmux send-keys "python navigate.py $@" Enter

tmux select-pane -t 3
tmux send-keys "conda activate vint_deployment_11" Enter
tmux send-keys "cd ~/vint_ws/src/visualnav-transformer/deployment/src" Enter
tmux send-keys "python pd_controller.py"

tmux select-pane -t 4
tmux send-keys "conda activate vint_deployment_11" Enter
tmux send-keys "cd ~/vint_ws/src/visualnav-transformer/deployment/src" Enter
tmux send-keys "python visualize_path.py" Enter


# Attach to the tmux session
tmux -2 attach-session -t $session_name