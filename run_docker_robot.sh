#!/bin/bash
# 使用完整镜像运行机器人

ROBOT_ID=${1:-0}
ROBOT_NUM=${2:-2}

echo "Starting Robot $ROBOT_ID of $ROBOT_NUM..."

# 检查镜像是否存在
if ! sudo docker images | grep -q "mtare-complete"; then
    echo "Error: Docker image 'mtare-complete' not found!"
    echo "Please build first: ./build_complete_docker.sh"
    exit 1
fi

# 运行容器
sudo docker run -it --rm \
    --name mtare_robot_${ROBOT_ID} \
    --network host \
    -e DISPLAY=$DISPLAY \
    -e ROS_DOMAIN_ID=42 \
    -e ROBOT_ID=$ROBOT_ID \
    -e ROBOT_NUM=$ROBOT_NUM \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    mtare-complete:latest \
    bash -c "
        source /opt/ros/humble/setup.bash
        source /ros2_ws/install/setup.bash
        
        echo '[Robot '\$ROBOT_ID'] Starting Unity...'
        /ros2_ws/src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64 &
        sleep 5
        
        echo '[Robot '\$ROBOT_ID'] Starting planner...'
        ros2 launch vehicle_simulator system_simulation_with_mtare_planner.launch.py \\
            robot_id:=\$ROBOT_ID \\
            kRobotNum:=\$ROBOT_NUM \\
            kAutoStart:=true \\
            coordination:=true
    "
