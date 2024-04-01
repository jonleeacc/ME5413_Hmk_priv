#!/bin/bash

# Set umask to ensure files are created with 777 permissions
umask 000

# Define log directory
LOG_DIR="/home/rosuser/docker_volume/logs"
mkdir -p $LOG_DIR

# Create a timestamp to uniquely identify this session's log files
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")

# File names using the timestamp
POSITION_LOG="$LOG_DIR/rms_position_error_log_$TIMESTAMP.txt"
HEADING_LOG="$LOG_DIR/rms_heading_error_log_$TIMESTAMP.txt"
SPEED_LOG="$LOG_DIR/rms_speed_error_log_$TIMESTAMP.txt"

# Start logging each RMSE topic to its respective file
rostopic echo /me5413_world/planning/rms_position_error > "$POSITION_LOG" &
echo $! > "/tmp/rmse_position_error_pid"
rostopic echo /me5413_world/planning/rms_heading_error > "$HEADING_LOG" &
echo $! > "/tmp/rmse_heading_error_pid"
rostopic echo /me5413_world/planning/rms_speed_error > "$SPEED_LOG" &
echo $! > "/tmp/rmse_speed_error_pid"

# Save the timestamp for the Python script to use later
echo $TIMESTAMP > "/tmp/rmse_logging_timestamp"
