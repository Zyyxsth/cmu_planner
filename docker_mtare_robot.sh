#!/bin/bash
# Docker 单机器人启动脚本 - 在容器内编译并运行

ROBOT_ID=${1:-0}
ROBOT_NUM=${2:-2}
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# 检查 sudo
if ! sudo -n docker ps > /dev/null 2>&1; then
    echo "需要 sudo 权限，请输入密码："
    sudo -v
fi

echo "Starting Robot $ROBOT_ID of $ROBOT_NUM in Docker..."

# 创建入口脚本
ENTRY_SCRIPT="/tmp/mtare_entry_${ROBOT_ID}.sh"

cat > $ENTRY_SCRIPT << 'EOFSCRIPT'
#!/bin/bash
ROBOT_ID=$1
ROBOT_NUM=$2

# 设置环境
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH=/workspace/src/mtare_planner/tare_planner/or-tools/lib:$LD_LIBRARY_PATH

cd /workspace

# 检查是否已经编译过
if [ ! -f "/workspace/install/setup.bash" ]; then
    echo "[Robot $ROBOT_ID] First run - building packages..."
    
    # 编译依赖的消息包
    colcon build --packages-select motion_msgs ception_msgs mtare_msgs
    source source_workspace_setup.bash
    
    # 编译主要包
    colcon build --packages-select \
        ros_tcp_endpoint \
        vehicle_simulator \
        local_planner \
        terrain_analysis \
        terrain_analysis_ext \
        sensor_scan_generation \
        visualization_tools \
        mtare_planner
    
    echo "[Robot $ROBOT_ID] Build complete!"
fi

# Source 编译结果
source /workspace/install/setup.bash

echo "[Robot $ROBOT_ID] Starting Unity..."
/workspace/src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64 &
UNITY_PID=$!
sleep 5

echo "[Robot $ROBOT_ID] Starting planner..."
ros2 launch vehicle_simulator system_simulation_with_mtare_planner.launch.py \
    robot_id:=$ROBOT_ID \
    kRobotNum:=$ROBOT_NUM \
    kAutoStart:=true \
    coordination:=true

kill $UNITY_PID 2>/dev/null || true
EOFSCRIPT

chmod +x $ENTRY_SCRIPT

# 运行 Docker
sudo docker run -it --rm \
    --name mtare_robot_${ROBOT_ID} \
    --network host \
    -e DISPLAY=$DISPLAY \
    -e ROS_DOMAIN_ID=42 \
    -e XDG_RUNTIME_DIR=/tmp/runtime \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $SCRIPT_DIR:/workspace:rw \
    -v $ENTRY_SCRIPT:/tmp/entry.sh:ro \
    mtare-planner:latest \
    bash /tmp/entry.sh $ROBOT_ID $ROBOT_NUM
