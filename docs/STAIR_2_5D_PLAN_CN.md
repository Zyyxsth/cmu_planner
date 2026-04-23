# 2.5D 上坡 / 上楼梯规划改造计划（中文）

这份文档的目标不是直接给出最终代码，而是先把我们后面要做的事情拆成可以执行的阶段。

当前问题不是单纯“调一个参数”：

- 现有系统能处理一定程度的高度起伏
- 但它更像 **2.5D 高度场导航 / 探索**
- 还不能稳定回答下面两个更强的问题：
  1. 机器人能不能稳定通过缓坡，从一楼走到二楼
  2. 系统能不能把“楼梯”区分为一种特殊的可通行结构，而不是普通障碍物

所以这里我们先列清楚目标、分阶段路线、每阶段的验收标准，再按计划推进。

## 1. 我们真正想实现什么

先把目标分成 3 个层级，不混在一起：

### Level 1：缓坡可通行

系统能够：

- 把连续坡面识别成可通行地形
- 在坡面上连续规划路径
- 允许机器人从低层沿坡到高层

这是最现实、也最该先做好的目标。

### Level 2：楼梯可识别

系统能够区分：

- 缓坡
- 普通竖直障碍
- 楼梯 / 台阶结构

注意这里先是“识别”，不等于立刻“可爬”。

### Level 3：楼梯可规划通过

系统不仅识别这是楼梯，还能：

- 判断这段楼梯对当前机器人是否可通行
- 生成上楼梯的局部 / 全局路径
- 在上下层切换时保持位姿与目标表达合理

这是最终目标，也是改动最大的一层。

## 2. 先说现实判断

当前这套系统最适合先推进的是：

1. **先把 Level 1 做扎实**
2. **再做 Level 2**
3. **最后评估 Level 3 是否值得做**

原因很简单：

- 当前系统本质仍是高度场 / 2.5D 表达
- 直接去做“稳定上楼梯”会牵涉：
  - 地形表示
  - 可通行性定义
  - 局部规划碰撞模型
  - 目标点高度解释
  - 上下层地图关系
- 如果不分层推进，很容易代码改了一堆，但不知道哪一层真的起作用

## 3. 为什么顺序要先放在“表达层”

我们重新对了一遍现有代码依赖，结论是：

- `terrainAnalysis` / `terrainAnalysisExt`
  先把原始点云变成带高度含义的地形点
- `ViewPointManager`
  会直接根据 terrain height 给 viewpoint 定高、判连通
- `localPlanner`
  直接根据 `terrain_map` 的高度/障碍解释做可通行判定
- FAR / goal 逻辑
  会对目标点做高度重投影

所以如果我们还没先定义：

- 2.5D 地图如何表达
- 一楼和二楼如何区分
- goal 的 `z` 到底是什么语义

那后面的：

- 地形分类
- 局部规划
- 探索

都会建立在不稳的空间语义上。

这也是为什么，这份计划里要把：

- **2.5D / 分层表达**
- **goal / pose 的高度语义**

排在前面。

## 4. 总体路线

建议按下面 7 个阶段推进。

### Phase 0：先把问题定义清楚

目标：

- 明确“缓坡可通行”和“楼梯可通行”分别是什么意思
- 明确仿真验收标准和实机验收标准

输出：

- 1 套测试场景定义
- 1 套评价指标

建议指标：

- 是否生成连续路径
- 是否把目标点保持在正确高度层
- 是否出现 `localPlanner failed to find path`
- 是否出现错误 return-home / 提前结束
- 是否撞上楼梯前沿 / 台阶边

### Phase 1：先定义 2.5D 地图表达

目标：

先回答“系统到底怎样表示一楼和二楼”。

必须先定下来的问题：

- 同一个 `(x, y)` 是否允许对应多个候选高度层
- 当前系统是继续使用“单高度场”，还是扩展成“局部分层高度场”
- 坡道连接上下层时，系统如何表达这是同一条连续可通行面
- 楼梯在地图表示里是：
  - 竖直边缘集合
  - 一系列可攀爬台阶
  - 还是特殊连通结构

建议先输出一份简短设计说明，至少明确：

- 当前版本支持什么
- 当前版本不支持什么
- 后续如果要支持楼梯可通行，表达层需要增加什么

建议先对比 2 个候选方案：

#### 方案 A：单高度场增强版

特点：

- 每个 `(x, y)` 仍只保留一个主地面高度
- 优先解决缓坡、轻微高差
- 楼梯先视作特殊障碍或边缘

适合：

- 快速实现“缓坡可走”

#### 方案 B：局部分层高度场

特点：

- 同一个 `(x, y)` 允许保留多个局部高度候选
- 需要让 goal / viewpoint / path 都具备层级语义

适合：

- 真正做“楼梯可通行 / 一楼二楼区分”

当前更合理的策略是：

- **Phase 1 先把 A 和 B 的边界写清楚**
- **Phase 2~5 先按 A 落地**
- 如果 A 不够，再升级到 B

验收标准：

- 我们能明确说明：
  - 当前系统怎么表示一楼和二楼
  - 同投影不同 `z` 时怎么处理
  - 哪些场景当前明确不支持

### Phase 2：先定义 goal / pose 的高度语义

目标：

明确这些量的 `z` 到底是什么含义：

- `/goal_point`
- `/state_estimation`
- `/way_point`
- `local_path` / `global_path`
- viewpoint 高度

必须回答的问题：

- goal 的 `z` 是硬约束还是提示值
- goal 何时会被重投影到地形高度
- 机器人 pose 的 `z` 是底盘中心、传感器高度，还是地形参考高度
- 当坡道从一楼连接到二楼时，路径高度应该如何连续变化

当前最值得对的代码：

- [graph_planner.cpp](/home/robot/cmu_planner/src/route_planner/far_planner/src/graph_planner.cpp)
- [viewpoint_manager.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/viewpoint_manager/viewpoint_manager.cpp)
- [terrainAnalysis.cpp](/home/robot/cmu_planner/src/base_autonomy/terrain_analysis/src/terrainAnalysis.cpp)

验收标准：

- 能明确列出：
  - 哪些模块会改写 goal 高度
  - 哪些模块假设“地图只有一层主地面”
- 至少能在设计上保证：
  - 坡道上的高层目标不会被错误压回低层

### Phase 3：搭建仿真测试场景

目标：

在仿真中准备 3 类标准场景：

1. **缓坡场景**
   - 一楼到二楼用连续坡道连接
2. **单台阶场景**
   - 只有一个或几个明显高度突变
3. **标准楼梯场景**
   - 多级台阶

输出：

- 一组固定地图或仿真场景
- 每个场景至少 1 个起点 + 1 个终点

验收标准：

- 可以稳定重复启动
- 可以重复发同一组目标点
- 每次测试结果具有可比性

### Phase 4：改地形表示，不再只靠“高度阈值”

目标：

把当前“只看高度差阈值”的地形理解，升级成“高度 + 局部坡度 + 连续性”的判断。

当前最值得改的地方：

- [terrainAnalysis.cpp](/home/robot/cmu_planner/src/base_autonomy/terrain_analysis/src/terrainAnalysis.cpp)
- [terrainAnalysisExt.cpp](/home/robot/cmu_planner/src/base_autonomy/terrain_analysis_ext/src/terrainAnalysisExt.cpp)

建议新增的地形分类至少包括：

- `flat_ground`
- `traversable_slope`
- `step_or_stair_edge`
- `vertical_obstacle`

核心思路：

- 连续高度缓变 -> 候选可通行坡面
- 明显突变但高度低 -> 台阶/边缘
- 明显竖直且高度大 -> 障碍

验收标准：

- 在仿真里能把缓坡和楼梯边缘区分开
- `/terrain_map` 或新调试输出里能明显看出分类差异

### Phase 5：改局部规划可通行性判定

目标：

让 `localPlanner` 不再只靠一个统一的 `obstacleHeightThre` 判定一切。

当前最值得改的地方：

- [localPlanner.cpp](/home/robot/cmu_planner/src/base_autonomy/local_planner/src/localPlanner.cpp)

这一步建议做两层能力：

1. **先支持“坡道可走，楼梯/台阶边不可走”**
2. 再决定要不要做“楼梯可走”

建议新增逻辑：

- 对坡面使用坡度和连续性约束
- 对台阶边 / 楼梯边增加更明确的碰撞限制
- 如果后面要支持楼梯，再引入“可攀爬台阶”判定

验收标准：

- 缓坡场景下局部规划能持续出路
- 楼梯场景下不会把楼梯直接误当平地冲上去
- 若开启 stair-climb 模式，再单独验证可爬性

### Phase 6：探索层适配 2.5D

目标：

让 TARE 的 viewpoint / coverage / frontier 逻辑在坡道场景里更合理。

当前最值得看的地方：

- [viewpoint_manager.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/viewpoint_manager/viewpoint_manager.cpp)
- [planning_env.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/planning_env/planning_env.cpp)
- [local_coverage_planner.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/local_coverage_planner/local_coverage_planner.cpp)

这里先不要一上来就改成真正 3D 多层搜索。先做：

- 坡道区域 viewpoint 连通性更合理
- 不因为局部高度变化就过早把 cell 判成 covered
- 让坡道两端 frontier 不要太早消失

验收标准：

- 坡道场景中探索不会因为“没走到上面”就提前判完成

### Phase 7：实机影子验证，再做受控实机测试

目标：

先在实机上“只看，不放行”，再逐步开放。

建议分两步：

#### 6.1 Shadow mode

- 实机正常跑感知和规划
- 记录：
  - `/terrain_map`
  - `/terrain_map_ext`
  - `/path`
  - `/way_point`
  - 以及楼梯/坡道附近的可视化
- 但先不真正执行危险动作

#### 6.2 Controlled execution

只在这类场景逐步放开：

1. 轻缓坡
2. 矮台阶
3. 楼梯入口前几级

验收标准：

- 不撞击
- 不误判楼梯为普通可通行平地
- 如果支持 stair mode，再验证上楼稳定性

## 5. 建议的开发顺序

为了降低风险，推荐下面这个最小可行顺序：

1. **先定义 2.5D 地图表达**
2. **先定义 goal / pose 的高度语义**
3. **再完成仿真场景**
4. **再改地形分类**
5. **再改 localPlanner 的可通行判定**
6. **验证缓坡成功**
7. **再研究楼梯识别**
8. **最后再决定是否做“楼梯可通行”**

也就是说：

- 第一阶段先做：
  - **2.5D 表达与高度语义统一**
  - **缓坡可通行**
  - **楼梯视为特殊障碍**
- 第二阶段再做：
  - **楼梯识别**
- 第三阶段再决定：
  - **楼梯是否真的允许通过**

## 6. 当前 TODO 清单

### Milestone A：表达层定义

- [ ] 明确当前版本采用“单高度场增强版”还是“局部分层高度场”
- [ ] 明确一楼/二楼在当前系统中的表达方式
- [ ] 明确 `/goal_point` 的 `z` 在各模块中的语义
- [ ] 明确 `/state_estimation` / viewpoint / path 的高度参考

### Milestone B：仿真基线

- [ ] 搭建缓坡仿真场景
- [ ] 搭建单台阶仿真场景
- [ ] 搭建标准楼梯仿真场景
- [ ] 固定起点 / 终点 / 评价指标

### Milestone C：地形分类

- [ ] 在 `terrainAnalysis` 中增加局部坡度 / 连续性特征
- [ ] 区分：
  - [ ] 平地
  - [ ] 缓坡
  - [ ] 台阶/楼梯边缘
  - [ ] 竖直障碍
- [ ] 为分类结果增加调试输出

### Milestone D：导航规划

- [ ] 让 `localPlanner` 使用新的地形分类
- [ ] 缓坡场景下生成连续路径
- [ ] 楼梯场景下先稳定判成不可直接通过
- [ ] 落实 goal `z` 的保留与重投影规则

### Milestone E：探索适配

- [ ] 调整 viewpoint 连通性在坡道场景下的行为
- [ ] 调整 frontier / uncovered 逻辑，避免坡道导致过早完成
- [ ] 验证从低层探索到高层坡道终点时不会太早 return-home

### Milestone F：实机验证

- [ ] 先做 shadow mode 记录
- [ ] 先测缓坡
- [ ] 再测低矮台阶
- [ ] 最后才测楼梯

## 7. 我们下一步最适合从哪里开始

如果按当前仓库和当前风险来排，我建议下一步先做：

1. **把表达层和高度语义先写清楚**
2. **把仿真坡道 / 楼梯场景搭起来**
3. **先改 `terrainAnalysis` 的地形分类**

不要一开始就改探索器，也不要一开始就改楼梯通行策略。

原因是：

- 如果 2.5D / 高度语义没有先定义清楚，goal 的 `z`、path 的 `z`、viewpoint 的 `z` 会互相打架
- 如果地形表达本身没有区分“坡”和“楼梯边”，后面规划层永远是在错误输入上做决策
- 先把输入改对，后面的局部规划和探索才有意义

## 8. 一句话总计划

**先把 2.5D / 高度语义定义清楚，再在仿真里做“缓坡可走、楼梯先当特殊障碍”的地形分类与规划；验证稳定后，再决定是否把楼梯升级成真正可通行结构，并最后迁移到实机。**
