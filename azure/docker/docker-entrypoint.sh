#!/bin/bash
AIRSIM_EXECUTABLE=/home/airsim_user/Blocks/Blocks.sh

echo Starting AirSim binary...
$AIRSIM_EXECUTABLE &

echo Waiting 10 seconds before starting app...
sleep 10

echo Starting Python app
python3.6 /home/airsim_user/app/multirotor.py