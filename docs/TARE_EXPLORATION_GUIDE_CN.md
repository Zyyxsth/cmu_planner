# TARE 探索逻辑说明（中文）

这份文档专门解释当前仓库里 **TARE 探索器** 的核心逻辑，重点回答下面这些问题：

1. 什么叫“未探索区域”
2. 什么叫“已探索区域”
3. frontier 是怎么来的
4. viewpoint 候选点是怎么生成和筛选的
5. local path / global path 是怎么来的
6. 为什么有时会很快进入 `return home`

配套参数说明请看：

- [D1H_ODIN_PARAMETER_GUIDE_CN.md](/home/robot/cmu_planner/docs/D1H_ODIN_PARAMETER_GUIDE_CN.md)

## 1. 先说最重要的结论

这套探索器不是用一张“黑白栅格图”简单地表示已探索/未探索。  
它更像是把流程拆成了 4 层：

1. `PlanningEnv`
   - 从注册点云、地形图、frontier 提取“可供探索器理解”的点
2. `ViewPointManager`
   - 在局部 horizon 里生成 viewpoint 网格，并判断每个 viewpoint 能看到哪些未覆盖点
3. `LocalCoveragePlanner`
   - 从候选 viewpoint 里挑一小组当前值得去的点，并排出局部访问顺序
4. `GridWorld`
   - 维护全局 cell 状态，判断哪些子空间还在 exploring、哪些 covered、是否该 return home

所以：

- `/uncovered_cloud`
  不是一张地图，而是一批“当前还没覆盖到的点”
- `/viewpoint_vis_cloud`
  不是轨迹，而是“当前局部 horizon 里的 viewpoint 候选”
- `/local_path`
  才是这轮局部决策之后真正形成的路径

## 2. “未探索区域”到底是什么

### 2.1 `/uncovered_cloud`

这个话题在代码里就是直接这样生成的：

- [planning_env.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/planning_env/planning_env.cpp#L355)

流程是：

1. 先更新覆盖情况：
   - [planning_env.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/planning_env/planning_env.cpp#L266)
2. 再把当前还没被覆盖的点收集出来：
   - [planning_env.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/planning_env/planning_env.cpp#L365)
   - [planning_env.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/planning_env/planning_env.cpp#L395)

所以“未探索区域”在这套里更准确地说是：

- 当前 planning cloud 里
- 还没有被已有视点/访问轨迹覆盖到的那些点

也就是说，它不是：

- 一整块连续面片
- 或一个固定大小的网格块

而是：

- 一批 still-uncovered 的点云

### 2.2 `/uncovered_frontier_cloud`

这个是 frontier 版的未探索点：

- [planning_env.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/planning_env/planning_env.cpp#L400)
- [planning_env.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/planning_env/planning_env.cpp#L425)

它表达的是：

- 还没覆盖到
- 同时又属于 frontier 的点

它通常比 `/uncovered_cloud` 更能解释“下一步为什么往那个方向走”。

## 3. 什么叫“已探索区域”

这套里“已探索”不是单独发一张地图，而是通过“covered”状态来隐含表示。

对应代码：

- [planning_env.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/planning_env/planning_env.cpp#L274)
- [planning_env.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/planning_env/planning_env.cpp#L301)
- [planning_env.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/planning_env/planning_env.cpp#L316)

当前逻辑是：

1. 如果某个点被当前观测覆盖
   - 记为 covered
2. 如果某个点被已访问过的 viewpoint 覆盖
   - 也记为 covered
3. 然后再做一次 dilation
   - 把已覆盖区域向周边扩一圈

所以“已探索”可以理解成：

- 被当前或历史 viewpoint 观测到
- 并经过一定膨胀处理
- 因此不再属于 `uncovered_cloud`

## 4. frontier 是怎么来的

frontier 是在 `PlanningEnv` 里根据点云边界和可见性条件提取的。

相关参数在：

- [indoor_d1h_20x5_reference.yaml](/home/robot/cmu_planner/src/exploration_planner/tare_planner/config/indoor_d1h_20x5_reference.yaml#L42)

当前值：

- `kUseFrontier = true`
- `kFrontierClusterTolerance = 1.0`
- `kFrontierClusterMinSize = 10`
- `kUseCoverageBoundaryOnFrontier = true`

对应代码：

- [planning_env.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/planning_env/planning_env.cpp#L169)
- [planning_env.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/planning_env/planning_env.cpp#L190)
- [planning_env.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/planning_env/planning_env.cpp#L201)

含义：

- frontier 会先做聚类
- 只有点数达到 `kFrontierClusterMinSize`
  才算有效 frontier
- 如果开了 `kUseCoverageBoundaryOnFrontier`
  那么边界外的 frontier 会被裁掉

所以如果你发现：

- 场地里肉眼还有很多没看过的地方
- 但探索器就是不觉得那里值得去

优先怀疑的是：

- frontier 被边界裁掉了
- frontier 聚类太小，被过滤掉了

## 5. viewpoint 候选点是怎么来的

### 5.1 先在局部 horizon 里铺一张 viewpoint 网格

相关参数：

- [indoor_d1h_20x5_reference.yaml](/home/robot/cmu_planner/src/exploration_planner/tare_planner/config/indoor_d1h_20x5_reference.yaml#L72)

当前值：

- `number_x = 25`
- `number_y = 10`
- `resolution_x = 0.4`
- `resolution_y = 0.4`

也就是局部 horizon 约：

- `10m x 4m`

### 5.2 viewpoint 本身并不是随便一个平面点

还要经过这些约束：

- 是否在边界内
- 是否与地形高度匹配
- 是否与邻居 viewpoint 高差过大
- 是否离障碍太近
- 是否在传感器有效 FOV 内

相关参数：

- `kConnectivityHeightDiffThr`
- `kViewPointCollisionMargin`
- `kViewPointCollisionMarginZPlus`
- `kViewPointCollisionMarginZMinus`
- `kViewPointHeightFromTerrain`
- `kViewPointHeightFromTerrainChangeThreshold`
- `kSensorRange`
- `kNeighborRange`
- `kCoverageOcclusionThr`
- `kCoverageDilationRadius`

对应代码：

- [viewpoint_manager.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/viewpoint_manager/viewpoint_manager.cpp#L17)
- [viewpoint_manager.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/viewpoint_manager/viewpoint_manager.cpp#L42)
- [viewpoint_manager.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/viewpoint_manager/viewpoint_manager.cpp#L738)
- [viewpoint_manager.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/viewpoint_manager/viewpoint_manager.cpp#L819)
- [viewpoint_manager.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/viewpoint_manager/viewpoint_manager.cpp#L914)

### 5.3 你在 RViz 里看到的候选点是什么

真正有用的是这几个：

- `/viewpoint_vis_cloud`
- `/selected_viewpoint_vis_cloud`
- `/lookahead_point_cloud`

它们是当前局部规划实时发布的：

- [sensor_coverage_planner_ground.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp#L979)
- [sensor_coverage_planner_ground.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp#L982)
- [sensor_coverage_planner_ground.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp#L994)

注意：

- `/tare_visualizer/viewpoints`
- `/tare_visualizer/viewpoint_candidates`

这些名字看起来像对，但当前并不是最可靠的实时可视化来源。

## 6. 探索器是怎么挑“值得去的 viewpoint”的

这一步主要在 `LocalCoveragePlanner`。

它会看每个 viewpoint 能覆盖多少：

- 未覆盖普通点
- 未覆盖 frontier 点

相关代码：

- [local_coverage_planner.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/local_coverage_planner/local_coverage_planner.cpp#L124)
- [local_coverage_planner.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/local_coverage_planner/local_coverage_planner.cpp#L132)
- [local_coverage_planner.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/local_coverage_planner/local_coverage_planner.cpp#L595)
- [local_coverage_planner.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/local_coverage_planner/local_coverage_planner.cpp#L604)

当前阈值：

- `kMinAddPointNumSmall = 30`
- `kMinAddPointNumBig = 50`
- `kMinAddFrontierPointNum = 10`

含义：

- 如果一个 viewpoint 覆盖到的未覆盖点太少
  - 它就不会进入候选
- frontier 覆盖数太少
  - 也不会进入 frontier 候选

所以你看到“明明有 viewpoint grid，但真正被选出来的很少”，通常不是 bug，而是：

- 大部分 viewpoint 的收益没过阈值

## 7. `local_path` / `global_path` 是怎么来的

这一层是在选出 viewpoint 之后，由局部覆盖规划器排顺序，再生成路径。

相关位置：

- [local_coverage_planner.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/local_coverage_planner/local_coverage_planner.cpp#L671)
- [local_coverage_planner.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/local_coverage_planner/local_coverage_planner.cpp#L712)

可以粗略理解成：

1. 先选出一批值得去的 viewpoint
2. 再做顺序优化（这里用 TSP 思路）
3. 得到局部访问顺序
4. 发布成 `local_path`
5. 再结合全局信息形成 `global_path`

所以：

- 有 `path`
- 不代表你一定能稳定看到所有“viewpoint 调试点”

因为路径和调试点不是同一个可视化层。

## 8. 什么情况下会很快 `return home`

这是你之前最困惑的一个点。

真正的完成判定在这里：

- [sensor_coverage_planner_ground.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp#L1553)

它要求同时满足：

1. `grid_world_->IsReturningHome()`
2. `local_coverage_planner_->IsLocalCoverageComplete()`
3. 运行超过 5 秒

### 8.1 `IsReturningHome()` 什么时候成立

核心逻辑在：

- [grid_world.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/grid_world/grid_world.cpp#L802)

如果当前没有任何 `exploring_cell_indices`：

- 就会进入 `return_home_ = true`

也就是：

- 全局层觉得“没有还值得继续探索的 cell 了”

### 8.2 `IsLocalCoverageComplete()` 什么时候成立

对应逻辑在：

- [local_coverage_planner.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/local_coverage_planner/local_coverage_planner.cpp#L696)

当一轮局部挑选下来：

- `selected_viewpoint_indices_itr.empty()`

也就是：

- 一个新的有效 viewpoint 都没选出来

那局部层就认为：

- 这轮 local coverage complete

### 8.3 所以“没怎么动就 return home”通常意味着什么

不是它真的把整个场地都探索完了，而更像是：

- 当前边界里
- 当前参数下
- 没有足够收益的 exploring cell
- 局部也挑不出新的 viewpoint

于是系统认为：

- “探索结束，回家”

## 9. 哪些参数最影响“什么算未探索/值得探索”

如果你后面要重点调探索行为，最值得先盯的是下面这些：

### A. 未探索 / frontier 相关

- `kUseFrontier`
- `kFrontierClusterTolerance`
- `kFrontierClusterMinSize`
- `kUseCoverageBoundaryOnFrontier`
- `kUseCoverageBoundaryOnObjectSurface`

### B. viewpoint 生成相关

- `viewpoint_manager/number_x`
- `viewpoint_manager/number_y`
- `viewpoint_manager/resolution_x`
- `viewpoint_manager/resolution_y`
- `kSensorRange`
- `kNeighborRange`
- `kViewPointCollisionMargin`
- `kViewPointHeightFromTerrain`
- `kConnectivityHeightDiffThr`

### C. “值不值得去”相关

- `kMinAddPointNumSmall`
- `kMinAddPointNumBig`
- `kMinAddFrontierPointNum`

### D. cell 状态切换相关

- `kCellExploringToCoveredThr`
- `kCellCoveredToExploringThr`
- `kCellExploringToAlmostCoveredThr`
- `kCellAlmostCoveredToExploringThr`
- `kCellUnknownToExploringThr`

### E. 完成 / 回家相关

- `kNoExplorationReturnHome`
- `kRushHome`
- `kRushHomeDist`
- `kAtHomeDistThreshold`

## 10. 如果你后面要调探索，我建议的顺序

不要一口气全改。最稳的顺序是：

1. 先固定场地边界
2. 先固定 local horizon
3. 先看 `/uncovered_cloud` 和 `/uncovered_frontier_cloud` 是否真的有内容
4. 再看 `/viewpoint_vis_cloud` 和 `/selected_viewpoint_vis_cloud`
5. 最后再调：
   - frontier 阈值
   - viewpoint 收益阈值
   - return home 相关逻辑

这样你最容易知道：

- 是“未探索点就没了”
- 还是“未探索点有，但 viewpoint 没挑出来”
- 还是“viewpoint 挑出来了，但全局 cell 已经不想继续探索”

