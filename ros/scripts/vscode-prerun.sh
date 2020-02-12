#!/bin/bash
# when debugging nodes with breakpoints in vscode on linux in a docker, you may need to set the envFile in launch.json for it to work in external console
# generate the env file via prerun
echo "exporting ROS environment"

env > /tmp/ros.env