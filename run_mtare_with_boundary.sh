#!/bin/bash
# Run MTARE planner with exploration boundary
# This script launches multi-robot exploration constrained to an indoor area

set -e

# Default values
ROBOT_NUM=2
BOUNDARY_FILE=""
COORDINATION="true"
AUTO_START="true"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --robot-num)
            ROBOT_NUM="$2"
            shift 2
            ;;
        --boundary-file)
            BOUNDARY_FILE="$2"
            shift 2
            ;;
        --no-boundary)
            USE_BOUNDARY="false"
            shift
            ;;
        --no-coordination)
            COORDINATION="false"
            shift
            ;;
        --manual-start)
            AUTO_START="false"
            shift
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --robot-num N          Number of robots (default: 2)"
            echo "  --boundary-file PATH   Path to boundary PLY file"
            echo "  --no-boundary          Disable boundary constraint"
            echo "  --no-coordination      Disable multi-robot coordination"
            echo "  --manual-start         Don't auto-start exploration"
            echo "  --help                 Show this help message"
            echo ""
            echo "Examples:"
            echo "  # Default: 2 robots with boundary"
            echo "  $0"
            echo ""
            echo "  # 3 robots with custom boundary"
            echo "  $0 --robot-num 3 --boundary-file /path/to/my_boundary.ply"
            echo ""
            echo "  # Create and use custom rectangular boundary"
            echo "  python3 scripts/create_boundary_ply.py --rectangle -20 -20 50 40 -o my_boundary.ply"
            echo "  $0 --boundary-file ./my_boundary.ply"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Source workspace
source install/setup.bash

# Build launch arguments
LAUNCH_ARGS="robot_num:=$ROBOT_NUM"
LAUNCH_ARGS="$LAUNCH_ARGS coordination:=$COORDINATION"
LAUNCH_ARGS="$LAUNCH_ARGS kAutoStart:=$AUTO_START"

# Handle boundary settings
if [[ -n "$USE_BOUNDARY" && "$USE_BOUNDARY" == "false" ]]; then
    LAUNCH_ARGS="$LAUNCH_ARGS use_boundary:=false"
    echo "Launching WITHOUT boundary constraint"
elif [[ -n "$BOUNDARY_FILE" ]]; then
    # Get absolute path
    if [[ "$BOUNDARY_FILE" = /* ]]; then
        ABS_BOUNDARY="$BOUNDARY_FILE"
    else
        ABS_BOUNDARY="$(pwd)/$BOUNDARY_FILE"
    fi
    
    if [[ ! -f "$ABS_BOUNDARY" ]]; then
        echo "Error: Boundary file not found: $BOUNDARY_FILE"
        exit 1
    fi
    
    LAUNCH_ARGS="$LAUNCH_ARGS use_boundary:=true"
    LAUNCH_ARGS="$LAUNCH_ARGS boundary_file:=$ABS_BOUNDARY"
    echo "Launching with boundary: $ABS_BOUNDARY"
else
    # Use default boundary
    LAUNCH_ARGS="$LAUNCH_ARGS use_boundary:=true"
    echo "Launching with default boundary"
fi

echo ""
echo "================================================"
echo "Launching MTARE Multi-Robot Exploration"
echo "================================================"
echo "Robot count: $ROBOT_NUM"
echo "Coordination: $COORDINATION"
echo "Auto-start: $AUTO_START"
echo "================================================"
echo ""

# Launch
ros2 launch vehicle_simulator system_simulation_with_mtare_planner_multi_robot.launch.py $LAUNCH_ARGS
