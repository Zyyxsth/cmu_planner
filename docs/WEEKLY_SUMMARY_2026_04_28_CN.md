# 本周工作总结：Gazebo 2.5D 楼梯仿真与双向跨层导航

日期：2026-04-28

分支：`codex/gazebo-two-floor-whitebox`

## 1. 本周完成了什么

本周主要完成了从 Unity 仿真接口迁移到 Gazebo 白盒仿真，并在此基础上搭建了 2.5D 楼梯跨层导航验证链路。当前实现仍然保持原有导航栈接口不变：`vehicleSimulator`、FAR、`localPlanner`、`pathFollower`、`/cmd_vel`、`/state_estimation` 和 `/registered_scan` 的基本合同没有改成 Gazebo 专属模型。

### Gazebo 仿真接口

- 新增 Gazebo route simulation backend，用 Gazebo 提供可视化和 lidar 点云。
- 保留原来的 `vehicleSimulator` 作为运动学仿真主体。
- Gazebo 机器人模型通过 `/unity_sim/set_model_state` 同步显示。
- Gazebo `/lidar/points` 经过 `registeredScanFromOdom` 转成 planner 使用的 `/registered_scan`。
- 解决了前期出现的机器人高度、传感器位置、点云跟随、Gazebo/RViz 显示不同步等接口问题。

### 白盒 2.5D 场景

- 生成固定白盒地形场景，不再依赖随机障碍物。
- 场景包含：
  - 大平地
  - 多个缓坡
  - 多个单台阶
  - 两段式真实尺度楼梯
  - 中间平台
  - 二楼平台
- 楼梯高度按真实尺度建模：每级约 `0.15m`，总高度约 `3.0m`。

### RViz 目标高度表达

- 增加 RViz Goalpoint 的目标高度模式。
- 支持 `floor1` 和 `floor2`，用于在同一个 XY 投影下区分一楼目标和二楼目标。
- Gazebo 白盒多楼层模式下保留 `/goal_point.z`，避免目标被错误压回当前地面。

### 白盒楼梯 router

- 新增 `whitebox_stair_goal_router.py`。
- router 根据当前 `/state_estimation.z` 判断机器人当前楼层，根据 `/goal_point.z` 判断目标楼层。
- 当前逻辑：
  - `1F -> 1F`：直接透传给 FAR。
  - `2F -> 2F`：直接透传给 FAR。
  - `1F -> 2F`：FAR 到楼梯入口，楼梯 connector 上楼，FAR 接管二楼平台段。
  - `2F -> 1F`：FAR 到二楼楼梯入口，楼梯 connector 下楼，FAR 接管一楼目标段。
- 楼梯段使用 `/whitebox_vehicle_pose_override` 模拟未来 RL / stair low-level controller 的位置。
- router 在楼梯段发布 `/stop=2` 和零 `/cmd_vel`，避免 pathFollower 用旧 FAR waypoint 造成抖动。

### 双向跨层验证

- 新增 `two_floor_round_trip` 自动 probe。
- probe 会先发布二楼目标，再发布一楼目标，验证上楼和下楼都能通过同一条 connector。
- 已验证结果：
  - 上楼段 `status=reached`，最终 `final_odom_z=3.75m`。
  - 下楼段 `status=reached`，最终 `final_odom_z=0.75m`。
  - 结果文件：`logs/whitebox_terrain_probe/20260428_163140_round_trip/summary.json`

### 楼梯 connector metadata

- 场景 metadata 新增 `connectors` 字段。
- `realistic` profile 会生成 `south_split_stair_connector`。
- connector 包含：
  - `lower_floor`
  - `upper_floor`
  - `lower_entry_goal`
  - `upper_entry_goal`
  - `up_route`
  - `down_route`
  - `allowed_directions`
  - `controller: stair_policy_placeholder`
- router 优先读取 metadata connector；旧 metadata 没有该字段时才回退到 object 名字推导。
- 这一步把楼梯从 router 私有硬编码提升成后续 planner / RL policy 都可以读取的 2.5D 拓扑数据。

## 2. 已验证内容

- `system_simulation_with_route_planner.sh --gazebo` 可以启动 Gazebo 白盒仿真链路。
- Gazebo lidar 点云可以进入 `/registered_scan`、`/terrain_map`、`/terrain_map_ext`。
- RViz 中能通过目标高度区分一楼目标和二楼目标。
- 一楼到二楼可以通过楼梯 connector 到达。
- 二楼到一楼可以通过反向楼梯 connector 返回。
- 同楼层目标不会错误重新走楼梯。
- `two_floor_round_trip --require-goal-z` 已通过。
- 相关 Python 脚本通过 `py_compile`。
- git diff 检查通过。

## 3. 当前仍然没有完成的部分

当前实现证明了“接口替换为 Gazebo 后，原导航栈仍能跑，并且跨层拓扑可以通过显式 connector 打通”。但它还不是最终的 2.5D 楼梯规划能力。

还没完成的关键点：

- `terrainAnalysis` 还没有正式输出地形类别。
- `localPlanner` 还没有基于地形类别区分缓坡、楼梯边缘和障碍物。
- FAR 还没有原生把楼梯 connector 作为图搜索边使用。
- 楼梯段仍然是仿真 pose override，不是真实轮足/强化学习控制。
- TARE 探索层还没有适配多高度目标、坡道 viewpoint 连通性和跨层 coverage。
- 实机 shadow mode 和受控楼梯测试还没有开始。

## 4. 下一步建议

下一步不要继续堆 router 规则，应该开始让 planner 真正理解局部地形。

优先级建议：

1. 在 `terrainAnalysis` / `terrainAnalysisExt` 增加局部坡度、连续性和高度突变统计。
2. 增加调试输出，区分 `flat_ground`、`traversable_slope`、`step_or_stair_edge`、`vertical_obstacle`。
3. 用 Gazebo 白盒 probes 验证缓坡和楼梯边缘在分类上可区分。
4. 修改 `localPlanner`，先实现“缓坡可走，楼梯边缘默认不可直接通过”。
5. 再考虑把 metadata `connectors` 接入 FAR 图搜索，替代当前 router 分段。
6. 最后再接入 RL stair policy 作为楼梯 connector 的底层执行器。

## 5. 相关提交

- `f7d1abc Add Gazebo route simulation backend`
- `89d552a Add Gazebo two-floor whitebox simulation`
- `d383c0a Add Gazebo whitebox two-floor stair routing`
- `fbfbf36 Keep upstairs goals on second floor`
- `5fe6ad7 Route whitebox goals by current floor`
- `1565448 Add whitebox round-trip stair probe`
- `e943369 Expose whitebox stair connector metadata`
