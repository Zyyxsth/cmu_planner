#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd $SCRIPT_DIR
source ./install/setup.bash

# Set or-tools library path
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$SCRIPT_DIR/src/mtare_planner/tare_planner/or-tools/lib

# Option 1: Single Unity with multiple robots (shared environment)
# Note: This means both robots see each other in the same simulation
./src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64 &
sleep 3 

# Option 2: Multiple Unity instances (uncomment for isolated environments)
# Each Unity instance needs a different port configured
# ./src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64 &
# sleep 3
# ./src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64 &
# sleep 3

# Launch multi-robot ROS2 system
ros2 launch vehicle_simulator system_simulation_with_mtare_planner_multi_robot.launch.py robot_num:=2 &
sleep 1

# Start RViz with multi-robot config
ros2 run rviz2 rviz2 -d src/mtare_planner/tare_planner/rviz/tare_planner_multi_robot.rviz
