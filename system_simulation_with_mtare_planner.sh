#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd $SCRIPT_DIR
# Set or-tools library path
export LD_LIBRARY_PATH=$SCRIPT_DIR/src/mtare_planner/tare_planner/or-tools/lib:$LD_LIBRARY_PATH
source ./install/setup.bash
./src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64 &
sleep 3 
ros2 launch vehicle_simulator system_simulation_with_mtare_planner.launch &
sleep 1
ros2 run rviz2 rviz2 -d src/mtare_planner/tare_planner/rviz/tare_planner_ground.rviz
