# D1H + ODIN + CMU Planner 实际运行链路说明

这份文档只说明当前这台机器上已经确认过的真实运行方式。

目标是回答 4 个问题：

1. 现在到底是哪个仓库/哪套系统在干活
2. 哪些 ROS2 功能包真正参与了控制和导航
3. 电脑命令是怎么接到 D1H 上的
4. `cmu_planner` 现在应该怎么启动，哪些东西不要再手动起

---

## 启停原则

现在这套脚本已经改成了强制清理模式：

- 启动前，先自动清掉上一轮残留的上层导航进程
- 正常退出、`Ctrl+C` 退出、脚本异常退出时，也会再次自动清理
- 你如果只想手动停干净，也可以单独执行停止脚本

对应入口：

- 导航启动：
  - [system_real_robot_with_route_planner_d1.sh](/home/robot/cmu_planner/system_real_robot_with_route_planner_d1.sh)
- 探索启动：
  - [system_real_robot_with_exploration_planner_d1.sh](/home/robot/cmu_planner/system_real_robot_with_exploration_planner_d1.sh)
- MTARE 启动：
  - [system_real_robot_with_mtare_planner_d1.sh](/home/robot/cmu_planner/system_real_robot_with_mtare_planner_d1.sh)
- 手动停止上层导航栈：
  - [stop_d1_nav_stack.sh](/home/robot/cmu_planner/scripts/stop_d1_nav_stack.sh)
- 底层实际清理逻辑：
  - [cleanup_d1_nav_stack.sh](/home/robot/cmu_planner/scripts/cleanup_d1_nav_stack.sh)

如果你只是想确认“现在切回 SDK 会不会再被旧路径接管”，先执行：

```bash
cd /home/robot/cmu_planner
./scripts/stop_d1_nav_stack.sh
```

这条命令只清理 `cmu_planner` 上层导航，不会去动 `/opt/d1_ros2` 里的 vendor 底层控制服务。

---

## 1. 一句话结论

当前真实运行链路是：

- **底层控制**：不是 `cmu_planner` 自己起的，也不是我们后面试的临时桥
- **真正干活的底层**：机器上已有的 vendor service `d1_bringup.service`
- **感知/里程计**：`cmu_planner` 里的 `odin_ros_driver + odin_autonomy_bridge`
- **规划/导航**：`cmu_planner` 里的 `far_planner + local_planner + terrain_*`
- **导航速度输出**：`local_planner/pathFollower`
- **最终控制接口**：vendor 官方 ROS2 话题 `/d15020108/command/cmd_twist`

所以现在不要再理解成“`cmu_planner` 自己控制底盘”。

更准确地说：

- `cmu_planner` 负责感知桥接、规划、路径跟踪
- vendor 原生服务负责真正的 D1H 控制器
- 两边通过官方 ROS2 话题对接

---

## 2. 现在涉及的几个仓库/目录分别干什么

### A. `/opt/d1_ros2`

这是 **vendor 安装好的运行环境**，当前机器真正在线运行的 D1H 控制链在这里。

里面实际在跑的关键东西有：

- `ddt_bringup`
- `rl_controller`
- `teleop_command`
- `controller_manager`
- `robot_state_publisher`

其中最关键的是 systemd 服务：

- `d1_bringup.service`

它会在后台把 D1H 原生控制链启动起来。

你不需要手动再去 `ros2 launch rl_controller hw.launch.py ...`，因为这样会重新初始化控制器，可能导致机器人自己动、坐下、重置状态。

### B. `/home/robot/cmu_planner`

这是你现在在改、在用的导航仓库。

这里真正参与当前 D1H 导航链的包主要有：

- `vehicle_simulator`
  - 这里虽然名字叫 simulator，但里面包含真机 launch
- `local_planner`
  - 里面的 `pathFollower` 最终输出速度命令
- `terrain_analysis`
- `terrain_analysis_ext`
- `sensor_scan_generation`
- `far_planner`
  - 如果你跑的是导航，不是探索，就是这个
- `odin_ros_driver`
- `odin_autonomy_bridge`
- `d1_compat_bridge`
  - 这里只保留状态兼容和按键兼容，不再负责重启底层控制

### C. `/home/robot/ddt_ros2_control`

这是你后面 clone 下来的 vendor 源码仓库。

它的作用是：

- 帮我们确认官方控制链内部的源码实现
- 帮我们确认 `/command/cmd_twist`、`/command/cmd_pose` 到底是谁在订阅
- 帮我们确认 `SDK mode` 的逻辑

它**不是**当前机器实际在跑的那套 launch 源。

当前实际运行还是 `/opt/d1_ros2` 里的已安装版本，不是这个源码目录。

所以这个仓库现在的定位应该理解成：

- **参考源码**
- 不是“当前现场主运行入口”

---

## 3. 当前 D1H 原生控制链到底是什么

机器上后台已经在跑：

- `d1_bringup.service`

这条服务会启动 vendor 原生 D1H 控制系统。

我们已经确认到它的环境是：

- `ROS_DOMAIN_ID=42`
- `ROS_LOCALHOST_ONLY=1`
- 机器人 namespace：`/d15020108`

这 3 个点非常重要，因为它们决定了你看到的话题名为什么不是裸的 `command/cmd_twist`，而是：

- `/d15020108/command/cmd_twist`
- `/d15020108/command/cmd_pose`
- `/d15020108/command/cmd_key`

如果你的终端环境不对，你会误以为“没有订阅者”“没有话题”“命令发了没效果”。

---

## 4. 已经确认的官方命令接口

### 4.1 速度命令

真正生效的官方速度口是：

- `/d15020108/command/cmd_twist`

消息类型是：

- `geometry_msgs/msg/TwistStamped`

这个点已经实测验证过。

之前机器人不动，不是因为底层没接到，而是因为最开始错发成了：

- `geometry_msgs/msg/Twist`

正确的是：

- `geometry_msgs/msg/TwistStamped`

### 4.2 姿态/高度命令

官方姿态口是：

- `/d15020108/command/cmd_pose`

消息类型是：

- `geometry_msgs/msg/PoseStamped`

但是当前这台 D1H 的官方配置里，**当前 locomotion 配置并不会实际使用 `pose.z` 去调高度**。

也就是说：

- 这个话题不是坏的
- 也不是没人订阅
- 只是当前 D1H 配置只吃速度命令，不吃 base height

所以你现在已经确认能走通、也真正有意义继续对接的是：

- `cmd_twist`

不是高度控制。

### 4.3 SDK mode

遥控器切到 `SDK mode` 的含义，不是“切到底层另一套控制器”。

更准确地说是：

- 遥控器不再继续占着 ROS command 话题发速度/姿态命令
- 这样电脑就可以往同样的官方 `/command/*` 话题发命令接管

所以流程是：

1. 机器人原生上电、自检
2. 遥控器站稳
3. 切 `SDK mode`
4. 电脑往官方 `/d15020108/command/cmd_twist` 发命令

---

## 5. `cmu_planner` 这边现在怎么接入 D1H

当前已经改成：

- **不再默认起本地 D1 底层控制器**
- **复用机器上已经在跑的 vendor 原生控制服务**

### 5.1 哪个包在发速度

`local_planner` 里的 `pathFollower.cpp` 会发布：

- `geometry_msgs::msg::TwistStamped`

它本来就有 `cmdVelTopic` 参数。

现在 D1 真机 launch 已经把这个 `cmdVelTopic` 默认改到：

- `/d15020108/command/cmd_twist`

所以现在导航速度是：

- `pathFollower`
  -> 官方 `TwistStamped`
  -> vendor 原生 `d1h_rl_controller`

### 5.2 `d1_compat_bridge` 现在干什么

`d1_compat_bridge` 现在不再负责把 `MotionCtrl` 转成底层速度。

现在它主要只做：

- 从 vendor 原生状态读：
  - `joint_states`
  - `imu_sensor_broadcaster/imu`
  - `rl_controller/fsm`
  - `system_status_broadcaster/motors_status`
- 兼容输出老 Diablo 栈还在用的话题：
  - `diablo/sensor/Body_state`
  - `diablo/sensor/Motors`
  - `/diablo/sensor/ImuEuler`
- 处理 `MotionCtrl.mode_mark` 这种模式按键兼容，必要时转成 `/command/cmd_key`

所以它现在更像：

- **状态兼容桥**
- 不是底盘驱动

---

## 6. 现在真机导航到底会起哪些包

以导航这条为例：

- [system_real_robot_with_route_planner_d1.launch.py](/home/robot/cmu_planner/src/base_autonomy/vehicle_simulator/launch/system_real_robot_with_route_planner_d1.launch.py)

它现在默认做的是：

### 6.1 不再做的事

- 不再默认起 `d1_traditional_hw.launch.py`
- 也就是不再自己重启 D1 底层控制器

默认现在是：

- `start_d1_traditional_hw:=false`

### 6.2 会起的上层内容

- `host_sdk_sample`
- `odin_autonomy_bridge`
- `local_planner`
- `terrain_analysis`
- `terrain_analysis_ext`
- `sensor_scan_generation`
- `visualization_tools`
- `far_planner`
- `d1_compat_bridge`

也就是说，这条 launch 现在默认只接：

- ODIN 感知链
- planner
- 路径跟踪
- D1 状态兼容

真正的 D1H 控制器本身，依赖的是机器后台已经在跑的 vendor service。

---

## 7. 现在应该怎么启动

### 7.1 不要再做的事

不要再手动起这些作为主控制链：

```bash
ros2 launch rl_controller hw.launch.py robot:=d1h
```

```bash
ros2 launch ddt_bringup d1_robots.launch.py
```

```bash
ros2 launch d1_compat_bridge d1_traditional_hw.launch.py ...
```

原因：

- 它们都会重新初始化控制器
- 会和当前已经运行的 `d1_bringup.service` 冲突或重复接管
- 这就是之前“机器人自己动”“坐下”“发抖”的根源之一

### 7.2 现在推荐启动导航

在 `cmu_planner` 仓库里：

```bash
cd /home/robot/cmu_planner
source ./source_workspace_setup.bash
ros2 launch vehicle_simulator system_real_robot_with_route_planner_d1.launch.py
```

这条 launch 会自动把：

- `ROS_DOMAIN_ID` 设成 `42`
- `ROS_LOCALHOST_ONLY` 设成 `1`

并默认把 D1 话题指到：

- `/d15020108/...`

另外，当前真机 launch 已经改成了更安全的启动行为：

- 启动后默认 `autonomy=false`
- 不会一上来就拿 `(0, 0)` 当目标自己跑
- 只有真正收到 `/goal_point`，`far_planner -> /way_point -> localPlanner/pathFollower` 这条链才会切进自动导航

### 7.3 如果你只是想手工测官方速度口

在机器人本机终端执行：

```bash
source /opt/ros/humble/setup.bash
source /opt/d1_ros2/local_setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
source /opt/d1_ros2/namespace.sh

ros2 topic pub -r 20 /d15020108/command/cmd_twist geometry_msgs/msg/TwistStamped "
header:
  frame_id: vehicle
twist:
  linear:
    x: 0.1
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0
"
```

前提还是：

1. 遥控器站稳
2. 切 `SDK mode`

---

## 8. 为什么远端电脑有时看不到这些话题

因为当前 vendor 原生服务跑的是：

- `ROS_LOCALHOST_ONLY=1`

这意味着：

- D1H 原生控制链默认只接受本机 DDS
- 远端电脑不能直接看到这些控制话题

所以现在你要分两类话题看：

### 8.1 本机原生 D1H 控制话题

例如：

- `/d15020108/command/cmd_twist`
- `/d15020108/command/cmd_pose`
- `/d15020108/command/cmd_key`

这类话题默认只适合在机器人本机测。

### 8.2 `cmu_planner` 上层导航/感知话题

例如：

- `/state_estimation`
- `/registered_scan`
- `/path`

这些要不要远端可见，要看你怎么起上层链和你是否改 DDS/网络配置。

---

## 9. 你现在可以怎么理解整条系统

把它拆成三层最容易：

### 第 1 层：vendor 原生底层

位置：

- `/opt/d1_ros2`

职责：

- 真正控制 D1H
- 接受官方 `/command/*` 话题

### 第 2 层：CMU 感知和规划

位置：

- `/home/robot/cmu_planner`

职责：

- ODIN 驱动和桥接
- route planner / exploration planner
- local planner / path follower

### 第 3 层：兼容层

位置：

- `/home/robot/cmu_planner/src/utilities/d1_compat_bridge`

职责：

- 把 vendor 状态兼容成老 Diablo 栈还在用的话题
- 必要时转发模式按键

---

## 10. 目前已经确认正确的点

- vendor 原生控制链已经在后台运行
- 真正的机器人 namespace 是 `/d15020108`
- 官方速度口是 `/d15020108/command/cmd_twist`
- 正确消息类型是 `geometry_msgs/msg/TwistStamped`
- `cmd_pose` 在当前 D1H 配置下不会用来调高度
- `cmu_planner` 现在已经改成默认复用原生控制链，而不是自己重启底层

---

## 11. 目前仍然不要混淆的点

- `vehicle_simulator` 只是包名历史遗留，不代表这里一定是仿真
- `ddt_ros2_control` 是参考源码，不是当前机器现场主运行入口
- `MotionCtrl` 是 CMU/Diablo 兼容接口，不是 D1 官方接口
- 现在真正应该对接的是官方 `/d15020108/command/cmd_twist`

---

## 12. 推荐你接下来怎么用

### 只想验证 D1H 官方控制

1. 原生上电
2. 遥控器站稳
3. 切 `SDK mode`
4. 本机发 `/d15020108/command/cmd_twist`

### 想跑导航

1. 保持原生服务在后台运行
2. 起：

```bash
cd /home/robot/cmu_planner
source ./source_workspace_setup.bash
ros2 launch vehicle_simulator system_real_robot_with_route_planner_d1.launch.py
```

3. 然后检查：
- `/state_estimation`
- `/registered_scan`
- `/goal_point`
- `/way_point`

### 手工给导航发目标点

注意，这一步如果你是在另外一个终端里执行，必须先把 ROS 环境补齐，否则你会看到“`/goal_point` 没订阅者”。

推荐直接用下面这组命令：

```bash
source /opt/ros/humble/setup.bash
source /opt/d1_ros2/local_setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/d1_ros2/namespace.sh

cd /home/robot/cmu_planner
source ./source_workspace_setup.bash

ros2 topic pub --once /goal_point geometry_msgs/msg/PointStamped \
  "{header: {frame_id: map}, point: {x: 3.0, y: 0.0, z: 0.0}}"
```

发布成功后，当前已经确认过的正确链路是：

- `far_planner` 订阅 `/goal_point`
- `far_planner` 发布 `/way_point`
- `localPlanner` 收到 `/way_point` 后切到 `autonomy=true`
- `pathFollower` 收到 `/way_point` 后切到 `autonomy=true`
- `pathFollower` 向 `/d15020108/command/cmd_twist` 发 `TwistStamped`
- `/path`
- `/d15020108/command/cmd_twist`

### 远端看 RViz

如果你想在另一台电脑上看 RViz，不要直接改 `/opt/d1_ros2`。

只需要把 `cmu_planner` 这一层起成“允许局域网访问”即可，因为：

- vendor 底层控制链仍然留在本机
- `cmu_planner` 会在本机读取这些底层话题
- 然后把 `/state_estimation`、`/registered_scan`、`/path`、`/way_point` 这些上层可视化话题对外广播

机器人这一侧这样启动：

```bash
cd /home/robot/cmu_planner
source ./source_workspace_setup.bash

export D1_RUN_RVIZ=0
export D1_ROS_LOCALHOST_ONLY=0
export D1_ROS_DOMAIN_ID=42

./system_real_robot_with_route_planner_d1.sh
```

这样做的效果是：

- 机器人本机不再起 RViz
- 上层导航/感知话题允许远端机器发现

远端电脑这一侧至少要满足：

- 和机器人在同一个局域网
- `ROS_DOMAIN_ID=42`
- `ROS_LOCALHOST_ONLY=0`
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`

远端电脑可以这样准备环境：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

然后先确认能看到这些话题：

```bash
ros2 topic list | grep -E "state_estimation|registered_scan|path|way_point"
```

如果能看到，再起 RViz：

```bash
rviz2 -d /home/robot/cmu_planner/src/route_planner/far_planner/rviz/default.rviz
```

注意：

- 远端如果是 Ubuntu/Linux，直接跑 `rviz2` 最稳
- 远端如果是 macOS，原生跑 RViz2 一般不太稳，最好还是用一台 Linux 机器看

---

## 13. 如果后面你只记一件事

**不要再自己手动重启 D1 底层控制器。**

现在正确的思路是：

- vendor 原生服务已经在控 D1H
- `cmu_planner` 只要把导航速度送到官方 `TwistStamped` 话题即可

这才是当前机器上已经验证通的链路。
