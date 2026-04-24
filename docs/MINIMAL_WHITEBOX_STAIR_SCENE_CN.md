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

- `24m x 24m` 平地
- `4` 段固定缓坡，包含不同坡长和升高
- `4` 个固定单台阶，高度包含 `0.08m`、`0.10m`、`0.15m`
- `3` 段固定楼梯
- 每段楼梯 `6` 级，每级高 `0.10m`、进深 `0.28m`、宽 `1.2m`
- `1` 个二楼平台，高度 `0.60m`，接在 `stair_south_right` 楼梯顶部

布局上，机器人默认从场地中心附近出发，坡道、台阶和楼梯分布在不同象限，避免一开始就穿模或压在地形上，也方便从多个方向观察 `/registered_scan`、`/terrain_map` 和 `/terrain_map_ext`。南侧楼梯额外连接一个二楼平台，用来测试“一楼目标”和“二楼目标”是否能通过同一套 `/goal_point` 接口连通。

## 生成命令

在仓库根目录执行：

```bash
python3 scripts/generate_whitebox_stair_test_scene.py
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
  --probe two_floor_stair_preentry,two_floor_stair_lower,two_floor_stair_top,two_floor_goal
```

这条测试只发布正常 `/goal_point`，不会直接注入 `/way_point`。判断时不要只看 `status=reached`，还要看 `summary.json` 里的 `final_odom.z`：二楼平台目标附近应当从一楼高度上升到约 `1.30m`，对应二楼平台高度约 `0.55-0.60m` 加 `vehicleHeight`。

如果不想发布合成 `/joy`，可以加 `--no-publish-joy`，但这通常只适合你已经手动打开 autonomy 的情况。

## 后续建议

如果这套 2.5D 白盒场景跑通，再做下一步：

- 对比 Unity 深度点云和 Gazebo lidar 点云差异
- 对比 `/registered_scan`、`/terrain_map`、`/terrain_map_ext`
- 再决定是否修改 `terrainAnalysis` / `terrainAnalysisExt` / `localPlanner`

现在的二楼测试验证的是 `vehicleSimulator` 运动模型下的 2.5D 高度跟随和接口连通，不等价于 Gazebo 真实轮足接触动力学。后续如果要做真实上楼梯策略，还需要把 `stair_traversal_decision` 这类调试标签接入 planner / costmap，而不是只停留在统计脚本里。
