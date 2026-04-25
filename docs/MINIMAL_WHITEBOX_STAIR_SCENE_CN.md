# Gazebo 2.5D 白盒地形测试场景

这个文档对应脚本：

- [scripts/generate_whitebox_stair_test_scene.py](/home/yy/cmu_planner/scripts/generate_whitebox_stair_test_scene.py)

它会生成一个固定 2.5D 白盒地形场景，用来验证：

- Gazebo 可以替代 Unity 作为仿真感知源
- 原来的 ROS `vehicleSimulator` 仍然是唯一运动源
- FAR / local planner 在换仿真平台后仍能吃 `/state_estimation` 和 `/registered_scan`
- 坡道、单台阶、楼梯在 `/terrain_map` / `/terrain_map_ext` 中如何被表达
- RViz 和 Gazebo 中的机器人位姿能够同步

## 场景组成

脚本生成的几何固定为：

- 默认 `realistic` 场景为 `36m x 36m` 平地
- `4` 段固定缓坡，包含不同坡长和升高
- `4` 个固定单台阶，高度包含 `0.08m`、`0.10m`、`0.15m`
- `1` 个更真实的两跑楼梯，中间有休息平台
- 楼梯总共 `20` 级，每级高 `0.15m`、进深 `0.28m`、宽 `1.2m`
- 中间平台高度 `1.50m`
- `1` 个二楼平台，高度 `3.00m`

布局上，机器人默认从场地中心附近出发，坡道、台阶和楼梯分布在不同象限，避免一开始就穿模或压在地形上，也方便从多个方向观察 `/registered_scan`、`/terrain_map` 和 `/terrain_map_ext`。南侧楼梯额外连接一个二楼平台，用来测试“一楼目标”和“二楼目标”是否能通过同一套 `/goal_point` 接口连通。

## 生成命令

在仓库根目录执行：

```bash
python3 scripts/generate_whitebox_stair_test_scene.py --profile realistic
```

如果需要回到旧的 `0.60m` 快速回归场景，可以用：

```bash
python3 scripts/generate_whitebox_stair_test_scene.py --profile compact
```

默认会生成：

- `src/base_autonomy/vehicle_simulator/mesh/whitebox_stair_test/whitebox_stair_test.obj`
- `src/base_autonomy/vehicle_simulator/mesh/whitebox_stair_test/whitebox_stair_test.json`
- `src/base_autonomy/vehicle_simulator/mesh/whitebox_stair_test/map.ply`

这些生成物位于 `mesh/` 下，默认被 `.gitignore` 忽略。正常启动 Gazebo 时不需要手动提交这些文件，启动脚本会自动重新生成。

## Gazebo 启动方式

```bash
./system_simulation_with_route_planner.sh --gazebo
```

如果只想无 GUI 检查：

```bash
./system_simulation_with_route_planner.sh --gazebo --no-rviz
```

## RViz 中如何区分一楼目标和二楼目标

同一个 XY 位置可能同时对应一楼和二楼，所以只靠鼠标点击的 `x/y` 不够。现在 `Goalpoint` 工具增加了 `Goal Z Mode` 属性：

- `current`：旧行为，发布当前机器人 `/state_estimation.z`
- `floor1`：发布 `z=0.75`，表示一楼车体/传感器高度
- `floor2`：发布 `z=3.75`，表示 `3.0m` 二楼平台加 `vehicleHeight`
- `custom`：使用 `Custom Goal Z`
- 也可以直接在 `Goal Z Mode` 里填一个数字，例如 `1.50`

因此，如果你想点同一个 XY 的一楼目标，把 `Goal Z Mode` 设成 `floor1`；如果想点同一个 XY 的二楼目标，把它设成 `floor2`。Gazebo 白盒 launch 默认使用 `whitebox_multilevel` FAR 配置，会保留 `/goal_point.z`，不会再把目标高度自动压回当前地面。

## 当前 2.5D 表达方式

这里的“2.5D”不是完整 3D voxel map。当前栈的实际表达更接近：

- `/registered_scan` 保留 3D 点云，点有 `x/y/z`
- `terrainAnalysis` 将点云按 XY 平面栅格聚合
- 每个 XY 栅格估计一个局部地面高度
- 输出 `/terrain_map` 时，点的 `z` 是空间位置，`intensity` 表示相对局部地面的高度差
- `localPlanner` 主要根据 `intensity` 判断低成本地形、障碍物、不可通行区域
- FAR planner 仍主要在 XY 拓扑上做全局绕障，但会用地形高度调整目标点高度

所以当前不是纯 2D，也不是完整 3D。它是“3D 点云输入 + XY 栅格高度/障碍表达 + 2D 路径搜索”的中间形态。

这也是为什么要先做这个白盒场景：我们需要先看坡、台阶、楼梯进入 `/terrain_map` 后到底变成什么，再决定怎么改地形分类和规划策略。

## 当前接口对齐方式

当前 Gazebo 版本故意对齐原来的 Unity 架构：

- `vehicleSimulator` 继续订阅 `/cmd_vel`
- `vehicleSimulator` 继续发布 `/state_estimation`
- `vehicleSimulator` 继续发布 `/unity_sim/set_model_state`
- Gazebo 订阅 `/unity_sim/set_model_state` 并同步显示机器人模型
- Gazebo lidar 发布 `/lidar/points`
- `registeredScanFromOdom` 将 `/lidar/points` 按 `/state_estimation` 转成 `/registered_scan`
- 白盒多楼层模式额外启动 `whitebox_stair_goal_router.py`：一楼普通目标继续走 `/routed_goal_point -> FAR`，二楼目标会被拆成分段路线。

二楼目标当前分段为：

- `TO_ENTRY`：先把楼梯入口前目标发给 FAR，由原来的 FAR / localPlanner / pathFollower 走过去。
- `STAIR_EXEC`：只在楼梯 connector 段使用仿真专用 `/whitebox_vehicle_pose_override` 推进 `vehicleSimulator`。这个段对应以后要替换成 RL/楼梯底层控制器的位置。
- `TO_FINAL`：到达楼梯顶并进入 `floor2_entry` 后，把二楼平台内的短距离子目标依次发回 FAR。
- `FLOOR2_EXEC_FALLBACK`：如果 FAR 超时没有接上当前二楼子目标，router 会明确打印 warning，然后用临时 fallback 把剩余二楼平台段走完，避免白盒 demo 假死。正常验证路径不应该进入这个状态。

如果机器人当前 `/state_estimation.z` 已经在二楼高度附近，再次点击二楼 `Goalpoint` 时不会重新走 `TO_ENTRY -> STAIR_EXEC`，而是直接把新的二楼目标透传给 FAR。这样在二楼连续点多个目标时，不会回到一楼楼梯入口。

所以现在不是“两个 vehicleSimulator”，也不是两个运动模型；仍然只有一个 `vehicleSimulator`。差别只是不同路线段选择不同控制输入：普通平地段用原始 `/cmd_vel`，楼梯/失败 fallback 段用 pose override。为了避免旧 FAR 目标残留造成抖动，router 在 `STAIR_EXEC` 和 `FLOOR2_EXEC_FALLBACK` 期间会持续发布 `/stop=2` 和零 `/cmd_vel`，等下一个 FAR 控制段开始时再发布 `/stop=0`。

## 建议验证顺序

先只测 route planner：

1. 平地目标点
2. 缓坡顶部目标点
3. 单台阶后方目标点
4. 楼梯入口前目标点
5. 楼梯顶部目标点

当前基础预期是确认：

- 换成 Gazebo 后原来的规划接口没有被改坏
- RViz 中规划和点云稳定
- Gazebo 中机器人模型跟随 `/state_estimation` 同步移动
- 坡道、台阶、楼梯在地形图中有可解释的高度表达
- 二楼目标可以通过南侧楼梯路径触发 `/state_estimation.z` 上升，而不是只在 XY 上假到达

## 2.5D 地形统计脚本

先离线检查生成出来的全局地形几何：

```bash
python3 scripts/analyze_whitebox_terrain_map.py \
  --ply src/base_autonomy/vehicle_simulator/mesh/whitebox_stair_test/map.ply
```

这个模式不需要 ROS，只验证生成的 `map.ply` 里是否真的有坡道、台阶、楼梯，以及它们的 `z` 高度范围是否合理。

启动 Gazebo 仿真后，可以另开终端检查机器人当前实际看到的局部 2.5D 地形：

```bash
python3 scripts/analyze_whitebox_terrain_map.py --topic /terrain_map
```

它会读取白盒场景 metadata，并按每个坡道、台阶、楼梯区域统计：

- 点数
- `z` 的 min / p50 / p90 / max / mean
- `intensity` 的 min / p50 / p90 / max / mean
- `terrain_class` 和 `class_reason`
- `traversal_axis`、`entry_side`、`approach_alignment`、`approach_class`

这里的 `intensity` 是当前地形分析输出的相对高度信号。我们后续判断“缓坡是否被连续表达，台阶/楼梯是否呈现明显突变”，优先看这个统计结果，而不是只靠 RViz 里肉眼观察。

注意：`/terrain_map` 是局部感知结果，不是全局地图。如果机器人还在中心平地，远处的坡道、楼梯区域会显示 `outside_current_cloud_xy`，这是正常的。需要先在 RViz 给目标点，让机器人走到对应地形附近，再检查该区域的 `/terrain_map` 统计。

当前调试分类标签包括：

- `flat`
- `ramp_like_traversable`
- `step_like`
- `stair_like`
- `obstacle_like`
- `rough_or_uncertain`

在这个白盒场景里，`terrain_class` 会使用已知区域 metadata 作为验证先验，再结合实测高度和 `intensity` 统计输出分类。它现在只是调试标签，用来验证 2.5D 数据链路，不是最终 planner 策略。

坡道和楼梯还会输出方向信息：

- `traversal_axis`：从低到高的通行方向
- `entry_side`：应该从哪一侧开始上坡/上楼梯
- `approach_alignment`：机器人当前朝向和通行方向的点积
- `approach_class`：当前接近方向分类
- `stair_traversal_decision`：楼梯通行调试决策

`approach_class` 的含义：

- `front_ascent_aligned`：接近方向基本对齐上升方向，可以认为是正面入口
- `reverse_or_descent_aligned`：方向相反，更像下坡/下楼梯方向
- `side_view_blocking`：侧面看到了楼梯，应当先按障碍边界处理
- `oblique_uncertain`：斜向观察，置信度不足

当前规则是：只有 `approach_alignment >= 0.80` 时，楼梯区域才输出 `stair_traversal_decision=front_entry_candidate`。这大约等价于机器人朝向和楼梯上升方向夹角小于 `37` 度。侧面接近会输出 `side_blocked`，不能当作可上楼梯路径。

## 自动 probe 测试脚本

启动 Gazebo 后，可以运行：

```bash
python3 scripts/probe_whitebox_terrain_topics.py
```

这个脚本会：

- 向 `/goal_point` 发布和 RViz goalpoint 等价的目标点
- 向 `/joy` 发布自动模式输入，打开 autonomy 并给一个非零前进速度
- 等待 `/state_estimation` 接近每个固定测试点
- 分别采样 `/registered_scan`、`/terrain_map`、`/terrain_map_ext`
- 把结果写到 `logs/whitebox_terrain_probe/<timestamp>/`

输出文件包括：

- `summary.json`：每个 probe 的目标点、是否到达、最近距离、最终 `/state_estimation` 位置
- `topic_overview.csv`：每个 topic 的点数、边界范围、整体 intensity 统计
- `region_stats.csv`：每个坡道、台阶、楼梯区域在各 topic 里的点数、`z`、`intensity`、`terrain_class` 和方向统计

如果只想跑某几个点，可以用：

```bash
python3 scripts/probe_whitebox_terrain_topics.py \
  --probe flat_center,ramp_south_2m_18cm
```

如果只想测试从一楼经楼梯到二楼平台：

```bash
python3 scripts/probe_whitebox_terrain_topics.py \
  --probe two_floor_goal \
  --require-goal-z
```

这条测试只发布正常 `/goal_point`，不会直接注入 `/way_point`。加上 `--require-goal-z` 后，判断不会只看 XY 距离，还会要求 `/state_estimation.z` 接近 `goal.z + vehicleHeight`。真实二楼平台目标附近应当从一楼高度上升到约 `3.75m`，对应二楼平台高度 `3.00m` 加 `vehicleHeight`。

注意：真实多楼层白盒场景里，一楼和二楼可能在 XY 上重叠。Gazebo lidar 和 planner-facing `/terrain_map` 仍然来自真实点云；但 `vehicleSimulator` 的高度跟随会单独订阅 `/whitebox_vehicle_terrain_map`。这个 topic 由 `scripts/publish_whitebox_vehicle_terrain_map.py` 根据生成的 metadata 发布，只用于 kinematic simulator 的当前楼层高度，避免 `vehicleSimulator` 在二楼 XY 下误选一楼高度。

二楼目标现在由 `whitebox_stair_goal_router.py` 处理。它订阅正常 `/goal_point`，识别二楼高度目标后，按南侧楼梯的正面上楼方向生成 connector 序列，并在 connector 段通过 `/whitebox_vehicle_pose_override` 连续推进 `vehicleSimulator`。这不是轮足真实接触动力学，也不是最终 RL 策略；它的作用是先证明“二楼目标可以通过一个显式 2.5D 楼梯连接器连通”，并让 Gazebo/RViz/点云接口继续保持可观测。

当前已验证的一次二楼 probe：

```bash
python3 scripts/probe_whitebox_terrain_topics.py \
  --probe two_floor_goal \
  --goal-timeout 180 \
  --settle-time 1.0 \
  --reach-radius 0.90 \
  --require-goal-z
```

最新验证结果写入：

```text
logs/whitebox_terrain_probe/20260425_135214_segmented_floor2_stop_gate_goal/summary.json
```

该次结果为 `status=reached`，`final_odom_z=3.75m`，`expected_odom_z=3.75m`，`min_z_error=0.01m`。同时日志明确显示 `TO_FINAL` 阶段 FAR 对第一个二楼平台子目标 `floor2_entry` 报 `goal_traversable=false`，随后 router 切到 `FLOOR2_EXEC_FALLBACK`。这说明当前“接口和拓扑连通”已经通过，而且楼梯/兜底段不会再被旧 `/cmd_vel` 抖动干扰；下一步要做的是让 FAR 的 2.5D 图能原生接上二楼平台入口，最终去掉 fallback。

如果不想发布合成 `/joy`，可以加 `--no-publish-joy`，但这通常只适合你已经手动打开 autonomy 的情况。

## 后续建议

如果这套 2.5D 白盒场景跑通，再做下一步：

- 对比 Unity 深度点云和 Gazebo lidar 点云差异
- 对比 `/registered_scan`、`/terrain_map`、`/terrain_map_ext`
- 再决定是否修改 `terrainAnalysis` / `terrainAnalysisExt` / `localPlanner`

现在的严格二楼测试结果是：`two_floor_goal --require-goal-z` 可以到达，最终 `/state_estimation.z` 约为 `3.75m`，说明二楼目标不再只是 XY 投影假到达。下一步要把这个白盒 connector 从“已知场景脚本”提升为 planner 可用的数据结构：楼梯入口、通行方向、楼层 ID、连接边和切换到 RL stair policy 的触发条件。
