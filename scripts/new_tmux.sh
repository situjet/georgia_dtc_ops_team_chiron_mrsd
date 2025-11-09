#!/bin/bash

# Configuration
SESSION_NAME="chiron_ops_panes"
REMOTE_HOST="10.3.1.32"
REMOTE_USER="dtc"
REMOTE_PASS="passme24"
REMOTE_SCRIPT_PATH="/home/dtc/restart.sh"


# Kill existing processes to prevent duplicates
echo "Cleaning up existing processes..."

# Kill foxglove bridge
echo "  - Stopping foxglove_bridge..."
pkill -f "foxglove_bridge" 2>/dev/null

# Kill QGroundControl
echo "  - Stopping QGroundControl..."
pkill -f "QGroundControl.AppImage" 2>/dev/null

# Kill GStreamer processes
echo "  - Stopping gst-launch..."
pkill -f "gst-launch-1.0" 2>/dev/null

# Kill VLM inference node
echo "  - Stopping vision_inference_node_refactored..."
pkill -f "vision_inference_node_refactored.py" 2>/dev/null

echo "Process cleanup complete."
sleep 2

# Kill existing session if it exists
echo "Clear existing tmux session..."
tmux kill-session -t $SESSION_NAME 2>/dev/null
sleep 1

# Create new tmux session with first pane
tmux new-session -d -s $SESSION_NAME

# Split the window to create 5 panes
# First, split horizontally to create left and right halves
tmux split-window -h

# Split the left pane vertically
tmux select-pane -t 0
tmux split-window -v

# Split the right pane vertically
tmux select-pane -t 2
tmux split-window -v

# Split one of the right panes to create the 5th pane
tmux select-pane -t 3
tmux split-window -v

# Now we have 5 panes, let's assign commands to each
# Pane 0: Foxglove (with clientPublish capability enabled)
tmux select-pane -t 0
tmux send-keys "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=100 && source /home/triage/georgia_dtc_ops_team_chiron_mrsd/airstack/ros_ws/install/setup.bash && ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765 -p capabilities:='[clientPublish]'" Enter

# Pane 1: QGroundControl
tmux select-pane -t 1
tmux send-keys "./Downloads/QGroundControl.AppImage" Enter

# Pane 2: RTSP Stream (Remote) - Send to TWO ports
tmux select-pane -t 2
tmux send-keys "bash dtc.sh" Enter
sleep 5
# Hardware decode test + UDP forward (no re-encoding, low CPU):
tmux send-keys "gst-launch-1.0 -v rtspsrc location=rtsp://10.3.1.124:8556/ghadron protocols=tcp latency=1500 ! rtph265depay ! h265parse ! tee name=t t. ! queue ! nvv4l2decoder ! fakesink t. ! queue ! rtph265pay config-interval=1 pt=96 ! tee name=udp_tee udp_tee. ! queue ! udpsink host=10.3.1.10 port=5004 sync=false udp_tee. ! queue ! udpsink host=10.3.1.10 port=5005 sync=false" Enter

# Pane 3: Local Receiver with Recording
tmux select-pane -t 3
sleep 5  # Wait for remote stream to start
tmux send-keys "gst-launch-1.0 -e -v udpsrc port=5004 caps='application/x-rtp,media=video,clock-rate=90000,encoding-name=H265,payload=96' ! rtpjitterbuffer latency=500 drop-on-latency=true ! rtph265depay ! h265parse config-interval=-1 ! tee name=t t. ! queue max-size-buffers=0 max-size-time=0 max-size-bytes=0 ! avdec_h265 ! videoconvert ! autovideosink sync=false t. ! queue max-size-buffers=0 max-size-time=0 max-size-bytes=0 ! matroskamux writing-app=GStreamer ! filesink location=output_\$(date +%Y%m%d_%H%M%S).mkv" Enter

# Pane 4: VLM Geolocator (Refactored) - Domain 100, publishes to /casualty_geolocated
tmux select-pane -t 4
sleep 5
tmux send-keys "cd /home/triage/vlm_geolocator" Enter
sleep 1
tmux send-keys "export ROS_DOMAIN_ID=100 && source /opt/ros/humble/setup.bash && source /home/triage/georgia_dtc_ops_team_chiron_mrsd/airstack/ros_ws/install/setup.bash && python3 src/vision_inference_node_refactored.py" Enter

# Add pane titles (requires tmux 3.0+)
tmux select-pane -t 0 -T "Foxglove"
tmux select-pane -t 1 -T "QGroundControl"
tmux select-pane -t 2 -T "RTSP Stream"
tmux select-pane -t 3 -T "Local Receiver"
tmux select-pane -t 4 -T "VLM (D100→/casualty_geolocated)"

# Enable pane titles display
tmux set -g pane-border-status top
tmux set -g pane-border-format "#{pane_index}: #{pane_title}"

# Go back to first pane
tmux select-pane -t 0

# Attach to the session
tmux attach-session -t $SESSION_NAME

echo "Tmux session '$SESSION_NAME' started with all processes in split panes."
echo "Use 'Ctrl+b' then arrow keys to switch between panes"
echo "Use 'Ctrl+b' then 'd' to detach from session"
echo "Use 'tmux attach -t $SESSION_NAME' to reattach later"

