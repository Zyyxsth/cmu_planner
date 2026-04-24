# Gazebo 固定障碍物白盒测试场景

这个文档对应脚本：

- [scripts/generate_whitebox_stair_test_scene.py](/home/yy/cmu_planner/scripts/generate_whitebox_stair_test_scene.py)

它会生成一个固定白盒障碍物场景，用来验证：

- Gazebo 可以替代 Unity 作为仿真感知源
- 原来的 ROS `vehicleSimulator` 仍然是唯一运动源
- FAR / local planner 在换仿真平台后仍能吃 `/state_estimation` 和 `/registered_scan`
- RViz 和 Gazebo 中的机器人位姿能够同步

## 场景组成

脚本生成的几何固定为：

- `32m x 32m` 平地
- `24` 个固定方块障碍物
- 中心出生区域保留为空地
- 当前版本先不包含楼梯和坡道

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
2. 障碍物之间的短距离目标点
3. 跨越多个障碍区的远距离目标点

第一版的预期不是验证楼梯策略，而是确认：

- 换成 Gazebo 后原来的规划接口没有被改坏
- RViz 中规划和点云稳定
- Gazebo 中机器人模型跟随 `/state_estimation` 同步移动

## 后续建议

如果这套固定障碍物场景跑通，再做下一步：

- 单独恢复坡道 / 台阶 / 楼梯几何
- 对比 Unity 深度点云和 Gazebo lidar 点云差异
- 再决定是否修改 `terrainAnalysis` / `terrainAnalysisExt` / `localPlanner`

不要在 Gazebo 接口稳定前先改楼梯通行策略。
