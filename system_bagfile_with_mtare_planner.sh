#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd $SCRIPT_DIR
source ./source_workspace_setup.bash
ros2 launch vehicle_simulator system_bagfile_with_mtare_planner.launch.py &
sleep 1
ros2 run rviz2 rviz2 -d src/mtare_planner/tare_planner/rviz/tare_planner_ground.rviz
