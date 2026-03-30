# 多机器人仿真配置指南

## 方案对比

### 方案1：独立 Unity 实例（推荐 ✅）
每个机器人有独立的 Unity 实例，完全隔离。现在仓库里已经提供了基于 Docker 的隔离启动脚本，不需要再去硬改 Unity DLL。

**优点：**
- 实现简单，无需修改代码
- 完全隔离，无干扰
- 支持任意数量的机器人

**缺点：**
- 资源消耗大（每个 Unity 实例需要独立 GPU/CPU 资源）
- 启动时间较长

**使用方法：**
```bash
# 一条命令起 2 机 ROS + 2 个 Docker Unity
./system_simulation_with_mtare_planner_multi_robot.sh 2 docker
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

### 步骤1：准备 Docker 镜像

Docker 镜像里已经补了 `socat` 和 Unity 常见运行库。首次使用前建议重建：

```bash
docker build -t mtare-planner:latest .
```

### 步骤2：启动独立 Unity 容器

仓库根目录提供了辅助脚本，会在容器内监听 `127.0.0.1:10000`，再转发到主机上的 `10000/10001/...`：

```bash
./run_unity_instance_docker.sh start 0 10000
./run_unity_instance_docker.sh start 1 10001
```

停止容器：

```bash
./run_unity_instance_docker.sh stop 0
./run_unity_instance_docker.sh stop 1
```

### 步骤3：启动多机器人系统

```bash
# 推荐：脚本统一起 ROS + Docker Unity + RViz
./system_simulation_with_mtare_planner_multi_robot.sh 2 docker

# 只起 ROS 多机，不起 Unity
./system_simulation_with_mtare_planner_multi_robot.sh 2 nounity
```

---

## 当前 Launch 文件说明

`system_simulation_with_mtare_planner_multi_robot.launch.py` 当前行为：

- **Robot i**: TCP Endpoint 端口 `10000 + i`
- **Robot i**: ROS namespace `/robot_i`
- **通讯话题**: `/wheeled0/exploration_info`, `/wheeled1/exploration_info`（全局，用于协调）
- **数据话题**: `/robot_0/...`, `/robot_1/...`（带 namespace，隔离）

---

## 推荐的快速测试方案

### 单机器人测试（先验证功能）
```bash
# 使用单机器人 launch 文件
./system_simulation_with_mtare_planner.sh
```

### 多机器人算法联调（先不带 Unity）
如果先只看协调算法和命名空间是否正常：

```bash
./system_simulation_with_mtare_planner_multi_robot_nounity.sh
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
- `/robot_0/cmd_vel` - Robot 0 的控制指令
- `/robot_1/cmd_vel` - Robot 1 的控制指令

### ROS TCP Endpoint 配置

每个机器人有独立的 TCP Endpoint：

| 机器人 | Namespace | TCP 端口 |
|--------|-----------|----------|
| Robot 0 | /robot_0 | 10000 |
| Robot 1 | /robot_1 | 10001 |

Docker Unity 容器里的 `127.0.0.1:10000` 会被转发到主机对应端口，所以不需要修改 Unity 可执行文件。

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
