#!/bin/bash

sleep 60

LOGFILE="/home/pi/Downloads/Dry_run/log.txt"
SCRIPT_LOGFILE="/home/pi/Downloads/Dry_run/script_output_$(date +'%Y%m%d_%H%M%S').log"

echo "Script started at $(date)" >> $LOGFILE

echo "Starting tmux session" >> $LOGFILE
/usr/bin/tmux new-session -d -s sensor_logging "python3 /home/pi/Downloads/Dry_run/collect_flight_data.py >> $SCRIPT_LOGFILE 2>&1" 2>> $LOGFILE

if [ $? -eq 0 ]; then
    echo "tmux session started at $(date)" >> $LOGFILE
else
    echo "Failed to start tmux session at $(date)" >> $LOGFILE
fi

