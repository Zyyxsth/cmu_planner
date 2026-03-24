# 多机器人仿真配置指南

## 方案对比

### 方案1：独立 Unity 实例（推荐 ✅）
每个机器人有独立的 Unity 实例，完全隔离。

**优点：**
- 实现简单，无需修改代码
- 完全隔离，无干扰
- 支持任意数量的机器人

**缺点：**
- 资源消耗大（每个 Unity 实例需要独立 GPU/CPU 资源）
- 启动时间较长

**使用方法：**
```bash
# Terminal 1: Robot 0
ros2 launch vehicle_simulator system_simulation_with_mtare_planner.launch.py robot_id:=0

# Terminal 2: Robot 1 (需要第二个 Unity 实例，使用不同端口)
# 复制 Unity 目录并修改端口配置（见下方说明）
```

### 方案2：共享 Unity 实例（需修改代码 ⚠️）
两个机器人共享同一个 Unity 环境。

**需要修改：**
1. **Unity 端**：修改 ROS-TCP-Connector 支持多端口或多话题 namespace
2. **ROS 端**：使用单个 TCP Endpoint，但区分不同机器人的消息

**当前限制：**
- Unity 硬编码端口 10000
- vehicleSimulator 设计为单机器人

### 方案3：混合方案（折中）
使用单个 Unity 实例显示环境，但机器人状态由外部计算。

---

## 实施方案1：独立 Unity 实例

### 步骤1：创建多实例 Unity 配置

由于 Unity 端口硬编码，需要创建多个实例目录：

```bash
cd src/base_autonomy/vehicle_simulator/mesh/unity/

# 创建 robot_0 的 Unity 实例（端口 10000）
cp -r environment unity_robot_0
cd unity_robot_0/Model_Data/Managed/
# 修改 DLL 中的端口（需要 .NET 反编译/编译工具）

# 创建 robot_1 的 Unity 实例（端口 10001）
cd ../../..
cp -r environment unity_robot_1
cd unity_robot_1/Model_Data/Managed/
# 修改 DLL 中的端口为 10001
```

**注意：** 修改编译后的 Unity DLL 需要专业工具（如 dnSpy 或 ILSpy），不推荐。

### 替代方案：使用容器隔离

使用 Docker/Podman 容器隔离每个 Unity 实例的网络：

```bash
# 启动容器 0（Unity 端口 10000，映射到主机 10000）
docker run --rm -it --network=host \
  -v $(pwd)/environment:/unity \
  --name unity_robot_0 \
  ubuntu:22.04 /unity/Model.x86_64

# 启动容器 1（Unity 端口 10000，映射到主机 10001）
# 使用端口映射而非 host 网络
docker run --rm -it -p 10001:10000 \
  -v $(pwd)/environment:/unity \
  --name unity_robot_1 \
  ubuntu:22.04 /unity/Model.x86_64
```

### 步骤2：启动多机器人系统

```bash
# 启动第一个机器人（使用 unity_robot_0）
export UNITY_PATH=src/base_autonomy/vehicle_simulator/mesh/unity/unity_robot_0/Model.x86_64
ros2 launch vehicle_simulator system_simulation_with_mtare_planner.launch.py robot_id:=0

# 启动第二个机器人（使用 unity_robot_1）
export UNITY_PATH=src/base_autonomy/vehicle_simulator/mesh/unity/unity_robot_1/Model.x86_64
ros2 launch vehicle_simulator system_simulation_with_mtare_planner.launch.py robot_id:=1
```

---

## 当前 Launch 文件说明

`system_simulation_with_mtare_planner_multi_robot.launch.py` 配置：

- **Robot 0**: TCP Endpoint 端口 10000, 位置 (0, 0)
- **Robot 1**: TCP Endpoint 端口 10001, 位置 (10, 0)
- **通讯话题**: `/wheeled0/exploration_info`, `/wheeled1/exploration_info`（全局，用于协调）
- **数据话题**: `/robot_0/...`, `/robot_1/...`（带 namespace，隔离）

---

## 推荐的快速测试方案

### 单机器人测试（先验证功能）
```bash
# 使用单机器人 launch 文件
./system_simulation_with_mtare_planner.sh
```

### 多机器人仿真（使用 Gazebo 替代 Unity）
如果需要真正的多机器人仿真且无需修改 Unity，建议使用 Gazebo：

```bash
# 修改 launch 文件使用 Gazebo 而非 Unity
ros2 launch vehicle_simulator system_simulation_with_mtare_planner_multi_robot.launch.py world_name:=gazebo
```

### 多机器人实机测试
在实际机器人上测试（无需 Unity）：
```bash
ros2 launch vehicle_simulator system_real_robot_with_mtare_planner.launch.py
```

---

## 技术细节

### 话题架构

**全局话题（通讯，用于多机器人协调）：**
- `/wheeled0/exploration_info` - Robot 0 的探索信息
- `/wheeled1/exploration_info` - Robot 1 的探索信息

**Namespaced 话题（数据，每个机器人独立）：**
- `/robot_0/registered_scan` - Robot 0 的点云
- `/robot_1/registered_scan` - Robot 1 的点云
- `/robot_0/state_estimation` - Robot 0 的状态
- `/robot_1/state_estimation` - Robot 1 的状态
- `/robot_0/vehicleCommand` - Robot 0 的控制指令
- `/robot_1/vehicleCommand` - Robot 1 的控制指令

### ROS TCP Endpoint 配置

每个机器人有独立的 TCP Endpoint：

| 机器人 | Namespace | TCP 端口 |
|--------|-----------|----------|
| Robot 0 | /robot_0 | 10000 |
| Robot 1 | /robot_1 | 10001 |

Unity 实例需要连接到对应端口。

---

## 常见问题

### Q: Unity 连接失败
**A:** 检查防火墙设置，确保端口 10000-10001 开放：
```bash
sudo ufw allow 10000:10001/tcp
```

### Q: 机器人之间无法通信
**A:** 检查全局话题是否存在：
```bash
ros2 topic list | grep exploration_info
```

### Q: RViz 无法显示两个机器人
**A:** 使用多机器人 RViz 配置：
```bash
ros2 run rviz2 rviz2 -d src/mtare_planner/tare_planner/rviz/tare_planner_multi_robot.rviz
```

---

## 后续改进

要实现真正的共享 Unity（单实例多机器人），需要：

1. **修改 Unity 工程**：
   - 修改 ROS-TCP-Endpoint 配置，支持动态端口或单端口多路复用
   - 在 Unity 场景中实例化多个机器人模型
   - 每个机器人模型发布/订阅不同 namespace 的话题

2. **修改 vehicleSimulator**：
   - 添加 `robot_id` 参数
   - 根据 robot_id 发布到不同的话题

3. **修改 launch 文件**：
   - 单 TCP Endpoint 模式
   - 多个 vehicleSimulator 节点，每个控制一个机器人

这需要 Unity 工程源码，当前仓库只包含构建后的可执行文件。
