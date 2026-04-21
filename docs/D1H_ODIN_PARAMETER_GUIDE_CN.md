# D1H + ODIN 参数说明（中文）

这份文档专门整理当前仓库里和 **机器人本体、场景大小、ODIN 设备、地形/障碍、探索规划** 相关的关键参数，方便你后面按自己的机器人和测试场地去调。

说明基准：

- 当前真机探索主入口：
  [system_real_robot_with_exploration_planner_d1.launch.py](/home/robot/cmu_planner/src/base_autonomy/vehicle_simulator/launch/system_real_robot_with_exploration_planner_d1.launch.py)
- 当前探索参考配置：
  [indoor_d1h_20x5_reference.yaml](/home/robot/cmu_planner/src/exploration_planner/tare_planner/config/indoor_d1h_20x5_reference.yaml)
- 当前 ODIN 驱动配置：
  [control_command.yaml](/home/robot/cmu_planner/src/odin_ros_driver/config/control_command.yaml)

这份文档主要回答 4 个问题：

1. 这个参数是干什么的
2. 当前默认值是多少
3. 值变大/变小通常会带来什么影响
4. 什么时候你应该考虑改它

## 1. 坐标系和高度的基本理解

### `map -> odom`

当前真机 launch 里，`map -> odom` 是一个静态恒等变换：

- [system_real_robot_with_exploration_planner_d1.launch.py](/home/robot/cmu_planner/src/base_autonomy/vehicle_simulator/launch/system_real_robot_with_exploration_planner_d1.launch.py#L147)

当前值：

- 平移：`0 0 0`
- 旋转：`0 0 0`

含义：

- 当前这套里，`map` 可以近似理解成“这次启动时建立的全局参考系”
- 机器人初始位置通常在 `map` 原点附近
- 但很多障碍/地形判断并不是直接拿世界坐标 `z` 来判

### `/terrain_map` 里的 `intensity`

这个非常重要。

在当前链路里，`localPlanner` 和探索器大量依赖 `/terrain_map` / `/terrain_map_ext` 的 `PointXYZI` 点云。这里的 `intensity` 不是普通亮度，而更像：

- 点相对局部地面的高度差

对应代码：

- [terrainAnalysis.cpp](/home/robot/cmu_planner/src/base_autonomy/terrain_analysis/src/terrainAnalysis.cpp#L615)
- [terrainAnalysis.cpp](/home/robot/cmu_planner/src/base_autonomy/terrain_analysis/src/terrainAnalysis.cpp#L618)
- [terrainAnalysisExt.cpp](/home/robot/cmu_planner/src/base_autonomy/terrain_analysis_ext/src/terrainAnalysisExt.cpp#L551)
- [terrainAnalysisExt.cpp](/home/robot/cmu_planner/src/base_autonomy/terrain_analysis_ext/src/terrainAnalysisExt.cpp#L558)

所以：

- `obstacleHeightThre = 0.08`
- `groundHeightThre = 0.05`

它们说的不是“世界坐标里 z 是多少”，而是：

- 某个点比它所在局部地面高多少

## 2. 机器人本体尺寸与安装相关参数

### 2.1 `vehicleLength` / `vehicleWidth`

位置：

- [local_planner.launch](/home/robot/cmu_planner/src/base_autonomy/local_planner/launch/local_planner.launch#L35)

当前值：

- `vehicleLength = 0.6`
- `vehicleWidth = 0.6`

作用：

- 给 `localPlanner` 用来做路径碰撞和通行宽度判断
- 这两个值偏小，会让机器人“觉得自己更瘦”，更容易从窄缝里走
- 这两个值偏大，会更保守，更容易绕开障碍，但也更容易卡住

什么时候该改：

- 真实车体包络明显不是 `0.6 x 0.6` 时
- 你发现它总想穿很窄的缝时

建议：

- 按“实际最大占用宽度”来设，不要只按机身裸尺寸
- 如果轮足展开后更宽，要按展开状态算

### 2.2 `sensorOffsetX` / `sensorOffsetY`

位置：

- [local_planner.launch](/home/robot/cmu_planner/src/base_autonomy/local_planner/launch/local_planner.launch#L3)
- [system_real_robot_with_exploration_planner_d1.launch.py](/home/robot/cmu_planner/src/base_autonomy/vehicle_simulator/launch/system_real_robot_with_exploration_planner_d1.launch.py#L57)

当前值：

- `sensorOffsetX = 0.15`
- `sensorOffsetY = 0.0`

作用：

- 表示规划所用 `sensor` 坐标系相对机器人参考位置的平移
- 同时影响 `vehicleTransPublisher`
- 正方向约定是：
  - `sensorOffsetX > 0` 表示传感器在机器人参考点前方
  - `sensorOffsetY > 0` 表示传感器在机器人参考点左侧

这一点也能从代码里看出来：

- [localPlanner.cpp](/home/robot/cmu_planner/src/base_autonomy/local_planner/src/localPlanner.cpp#L158)
- [pathFollower.cpp](/home/robot/cmu_planner/src/base_autonomy/local_planner/src/pathFollower.cpp#L147)

系统会用 odom 里的传感器位置，反推出机器人本体位置：

- `vehicleX = sensorX - cos(yaw) * sensorOffsetX + ...`

所以如果你的传感器在质心正前方 `15 cm`，那设：

- `sensorOffsetX = 0.15`

是对的。

什么时候该改：

- 如果你的 ODIN 不是装在机器人几何中心附近
- 或者你明确知道“传感器前移/后移/偏左偏右”

### 2.3 `cameraOffsetZ`

位置：

- [local_planner.launch](/home/robot/cmu_planner/src/base_autonomy/local_planner/launch/local_planner.launch#L5)
- [system_real_robot_with_exploration_planner_d1.launch.py](/home/robot/cmu_planner/src/base_autonomy/vehicle_simulator/launch/system_real_robot_with_exploration_planner_d1.launch.py#L58)

当前值：

- `cameraOffsetZ = 0.16`

作用：

- 主要用于 `/sensor -> /camera` 这个静态 TF
- 对当前地面导航主逻辑影响不大，但会影响相机相关显示和外参链

### 2.4 `vehicleHeight`

位置：

- [terrain_analysis.launch](/home/robot/cmu_planner/src/base_autonomy/terrain_analysis/launch/terrain_analysis.launch#L29)
- [terrain_analysis_ext.launch](/home/robot/cmu_planner/src/base_autonomy/terrain_analysis_ext/launch/terrain_analysis_ext.launch#L17)

当前值：

- `vehicleHeight = 1.5`

作用：

- 不是“雷达离地高度”
- 更像“机器人在垂直方向上会占用/关心的高度范围”
- 地形分析里只保留 `disZ < vehicleHeight` 的点作为相关障碍

对应代码：

- [terrainAnalysis.cpp](/home/robot/cmu_planner/src/base_autonomy/terrain_analysis/src/terrainAnalysis.cpp#L615)
- [terrainAnalysisExt.cpp](/home/robot/cmu_planner/src/base_autonomy/terrain_analysis_ext/src/terrainAnalysisExt.cpp#L552)

什么时候该改：

- 你的真实机器人高度明显不是 `1.5m`

当前对你这台车的建议：

- 如果真实有效车高大约 `0.8m`
- 可以考虑试 `0.8 ~ 0.9`

风险：

- 设太大：会把一些本来不影响车体通过的高处点也算进来
- 设太小：会漏掉本该考虑的上沿障碍

## 3. 低矮障碍 / 地形判定相关参数

这一组是你现在最需要盯的。

### 3.1 `obstacleHeightThre`

位置：

- [local_planner.launch](/home/robot/cmu_planner/src/base_autonomy/local_planner/launch/local_planner.launch#L21)

当前值：

- `obstacleHeightThre = 0.08`

作用：

- 在 `localPlanner` 里，超过这个高度差的点会被更像“硬障碍”处理

对应代码：

- [localPlanner.cpp](/home/robot/cmu_planner/src/base_autonomy/local_planner/src/localPlanner.cpp#L213)
- [localPlanner.cpp](/home/robot/cmu_planner/src/base_autonomy/local_planner/src/localPlanner.cpp#L902)

可以粗略理解成：

- 高于 `8 cm`：更像真正挡路障碍
- 低于这个值：未必强制绕开

什么时候该改：

- 低矮障碍明显被看到，但执行轨迹仍然愿意直穿

### 3.2 `groundHeightThre`

位置：

- [local_planner.launch](/home/robot/cmu_planner/src/base_autonomy/local_planner/launch/local_planner.launch#L22)

当前值：

- `groundHeightThre = 0.05`

作用：

- 定义“多高开始不算纯地面”
- 在 `groundHeightThre < h <= obstacleHeightThre` 这段里，点更像是“代价地形”，不是硬障碍

对应代码：

- [localPlanner.cpp](/home/robot/cmu_planner/src/base_autonomy/local_planner/src/localPlanner.cpp#L905)

可以粗略理解成：

- `<= 5 cm`：更像地面起伏
- `5 cm ~ 8 cm`：会增加 penalty
- `> 8 cm`：更像硬障碍

### 3.3 `checkRotObstacle`

位置：

- [local_planner.launch](/home/robot/cmu_planner/src/base_autonomy/local_planner/launch/local_planner.launch#L20)

当前值：

- `checkRotObstacle = false`

作用：

- 是否在原地转向/转身时也严格检查障碍

对应代码：

- [localPlanner.cpp](/home/robot/cmu_planner/src/base_autonomy/local_planner/src/localPlanner.cpp#L915)

影响：

- `false`：转向时更宽松
- `true`：近身障碍时更保守，但也更容易“不敢动”

### 3.4 `useTerrainAnalysis`

位置：

- [local_planner.launch](/home/robot/cmu_planner/src/base_autonomy/local_planner/launch/local_planner.launch#L17)

当前值：

- `useTerrainAnalysis = true`

作用：

- `true`：局部规划用 `/terrain_map`
- `false`：更接近直接用原始点云思路

当前建议：

- 真机先保持 `true`
- 因为之前直接吃 `/registered_scan` 的实验已经验证过，会让当前配置在硬件上过于保守

### 3.5 `kTerrainCollisionThreshold`

位置：

- [indoor_d1h_20x5_reference.yaml](/home/robot/cmu_planner/src/exploration_planner/tare_planner/config/indoor_d1h_20x5_reference.yaml#L34)

当前值：

- `kTerrainCollisionThreshold = 0.5`

作用：

- 探索器 TARE 从 `/terrain_map` / `/terrain_map_ext` 里提取碰撞点时用的阈值
- 只有 `intensity > 0.5` 的地形点，才会进入 exploration 侧的 collision cloud

对应代码：

- [sensor_coverage_planner_ground.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp#L557)
- [sensor_coverage_planner_ground.cpp](/home/robot/cmu_planner/src/exploration_planner/tare_planner/src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp#L575)

影响：

- 这个值偏高时，低矮障碍更容易“看到了但没被提升成探索碰撞”
- 这就是你现在有时看到“检测到了但没绕”的原因之一

## 4. 地形分析相关参数

### 4.1 `minRelZ` / `maxRelZ`

位置：

- [terrain_analysis.launch](/home/robot/cmu_planner/src/base_autonomy/terrain_analysis/launch/terrain_analysis.launch#L31)
- [local_planner.launch](/home/robot/cmu_planner/src/base_autonomy/local_planner/launch/local_planner.launch#L46)

当前值：

- terrain analysis:
  - `minRelZ = -1.5`
  - `maxRelZ = 0.3`
- local planner:
  - `minRelZ = -0.4`
  - `maxRelZ = 0.3`

作用：

- 限定在机器人相对高度窗口内哪些点参与分析
- 和你的轮足高度、ODIN 安装高度、地面起伏都有关系

什么时候该改：

- 传感器装得更高/更低
- 场地有明显坡度、坎、台阶

### 4.2 `scanVoxelSize`

位置：

- [terrain_analysis.launch](/home/robot/cmu_planner/src/base_autonomy/terrain_analysis/launch/terrain_analysis.launch#L8)
- [terrain_analysis_ext.launch](/home/robot/cmu_planner/src/base_autonomy/terrain_analysis_ext/launch/terrain_analysis_ext.launch#L10)

当前值：

- terrainAnalysis: `0.05`
- terrainAnalysisExt: `0.1`

作用：

- 点云体素化分辨率
- 值小：更细、更重
- 值大：更粗、更轻

## 5. 探索场景与局部规划范围

### 5.1 `viewpoint_manager/number_x`, `number_y`, `resolution_x`, `resolution_y`

位置：

- [indoor_d1h_20x5_reference.yaml](/home/robot/cmu_planner/src/exploration_planner/tare_planner/config/indoor_d1h_20x5_reference.yaml#L72)

当前值：

- `number_x = 25`
- `number_y = 10`
- `resolution_x = 0.4`
- `resolution_y = 0.4`

作用：

- 共同决定 local planning horizon 的大小

当前等效范围：

- `25 * 0.4 = 10m`
- `10 * 0.4 = 4m`

也就是：

- 当前局部探索窗口大约是 `10m x 4m`

什么时候该改：

- 场地比这个更小很多，绿色 horizon 框明显伸出边界太多
- 场地更大，局部窗口太短导致探索碎

### 5.2 `kSensorRange`

位置：

- [indoor_d1h_20x5_reference.yaml](/home/robot/cmu_planner/src/exploration_planner/tare_planner/config/indoor_d1h_20x5_reference.yaml#L93)

当前值：

- `kSensorRange = 3.5`

作用：

- 探索器认为传感器有效观察范围大约多远

什么时候该改：

- 实际 ODIN 在当前环境下可用距离明显不是 `3.5m`

### 5.3 `kViewPointCollisionMargin`

位置：

- [indoor_d1h_20x5_reference.yaml](/home/robot/cmu_planner/src/exploration_planner/tare_planner/config/indoor_d1h_20x5_reference.yaml#L84)

当前值：

- `kViewPointCollisionMargin = 0.6`

作用：

- exploration viewpoint 离障碍的安全边界

影响：

- 值大：更保守，离障碍更远
- 值小：更激进，更可能贴边

### 5.4 `kViewPointHeightFromTerrain`

位置：

- [indoor_d1h_20x5_reference.yaml](/home/robot/cmu_planner/src/exploration_planner/tare_planner/config/indoor_d1h_20x5_reference.yaml#L90)

当前值：

- `kViewPointHeightFromTerrain = 0.75`

作用：

- 探索器内部 viewpoint 相对地面的高度
- 这会影响你在 RViz/Foxglove 里看到的候选点高度

注意：

- 它不是车身真实高度
- 更像探索器在地面上方放置“可行视点”的参考高度

## 6. 速度与跟踪相关参数

### 6.1 `autonomySpeed`

位置：

- [system_real_robot_with_exploration_planner_d1.launch.py](/home/robot/cmu_planner/src/base_autonomy/vehicle_simulator/launch/system_real_robot_with_exploration_planner_d1.launch.py#L89)

当前值：

- `autonomySpeed = 0.3`

作用：

- 上层给路径跟踪器的自动速度参考

### 6.2 `lookAheadDis`

位置：

- [local_planner.launch](/home/robot/cmu_planner/src/base_autonomy/local_planner/launch/local_planner.launch#L106)

当前值：

- `lookAheadDis = 0.5`

作用：

- `pathFollower` 跟踪路径时向前看的距离

影响：

- 值小：更贴轨迹，但可能更抖
- 值大：更平滑，但可能转弯切角

## 7. ODIN 设备侧关键参数

位置：

- [control_command.yaml](/home/robot/cmu_planner/src/odin_ros_driver/config/control_command.yaml)

### 7.1 `sendodom`

当前值：

- `sendodom = 1`

作用：

- ODIN 发布 odometry

### 7.2 `send_odom_baselink_tf`

当前值：

- `send_odom_baselink_tf = 1`

作用：

- 发布 `odom -> base_link` TF
- 对点云和 RViz/Foxglove 显示都很重要

### 7.3 `senddtof`

当前值：

- `senddtof = 1`

作用：

- 打开原始 dtof 数据

### 7.4 `dtof_fps`

当前值：

- `dtof_fps = 100`

含义：

- 约 `10 fps`

可选值：

- `100` = 10 fps
- `145` = 14.5 fps

### 7.5 `sendcloudslam`

当前值：

- `sendcloudslam = 1`

作用：

- 开启融合后的 slam cloud

### 7.6 `custom_map_mode`

当前值：

- `custom_map_mode = 0`

含义：

- `0` = odometry mode
- `1` = slam mode
- `2` = relocalization mode

当前你这套真实导航主要就是：

- 先用当前 odom / slam 输出做上层规划

## 8. 针对你现在这台车的实际建议

按我们现在已经知道的情况，你最值得优先对照的不是全部参数，而是下面这几项：

### 优先级 A：和低矮障碍最相关

- `obstacleHeightThre = 0.08`
- `groundHeightThre = 0.05`
- `kTerrainCollisionThreshold = 0.5`

如果你看到：

- 低矮障碍被看到
- 但执行轨迹仍然不稳定绕开

优先就查这三个。

### 优先级 B：和你车高不匹配

- `vehicleHeight = 1.5`

如果你车高实际只有 `0.8m` 左右，这个值偏大，后面很值得单独做一轮对照。

### 优先级 C：和场地大小最相关

- `viewpoint_manager/number_x = 25`
- `viewpoint_manager/number_y = 10`
- `resolution_x = resolution_y = 0.4`

如果你实际测试场地就是 `20m x 5m` 左右，当前这版 `10m x 4m` 的 horizon 是一个偏稳妥的中间值。

## 9. 最后给你一个最实用的调参顺序

如果后面你要按自己的机器人重新调，我建议按这个顺序，不要一下子全改：

1. 先固定轮足高度和 ODIN 安装姿态
2. 先确认机器人包络尺寸：
   - `vehicleLength`
   - `vehicleWidth`
   - `vehicleHeight`
3. 再调低矮障碍判定：
   - `groundHeightThre`
   - `obstacleHeightThre`
   - `kTerrainCollisionThreshold`
4. 最后再调探索尺度：
   - `viewpoint_manager/number_x/y`
   - `resolution_x/y`
   - `kSensorRange`

这样最不容易把问题混在一起。

## 10. 按你现在这台机器人的尺寸，建议先怎么设

你给出的机械尺寸是：

- 站立尺寸：`375/750 × 493 × 643 mm`
- 匍匐尺寸：`470/845 × 580 × 250 mm`

这里斜杠通常意味着一个可变范围，所以我先按“保守包络”来理解：

- 长度最大按：`0.845 m`
- 宽度最大按：`0.580 m`
- 高度最大按：`0.643 m`

这是一种更稳妥的用法：

- 只要机器人在测试过程中有可能展开到这个范围
- 那碰撞/通行参数就应该至少覆盖这个最大包络

### 10.1 当前参数和你这台车的对比

当前 `localPlanner` 用的是：

- `vehicleLength = 0.6`
- `vehicleWidth = 0.6`

和你的机器人相比：

- `vehicleWidth = 0.6` 已经比较接近你的最大宽度 `0.58`
- 但 `vehicleLength = 0.6` 明显偏短，低于你给出的最大长度 `0.75 ~ 0.845`

这意味着当前系统更容易出现：

- 宽度方向还算保守
- 但前后方向把自己看得偏短
- 所以前向穿缝、贴障碍、低矮物“擦过去”的风险会更大

### 10.2 我建议你先试的机器人尺寸参数

如果你想先按“保守安全”来设，建议第一版直接这样：

- `vehicleLength = 0.85`
- `vehicleWidth = 0.58`

如果你觉得这太保守，也可以先试一个稍微折中的版本：

- `vehicleLength = 0.80`
- `vehicleWidth = 0.56`

我的建议是：

- 真机第一轮：先用保守版
- 如果发现过于不敢走，再慢慢往回收

### 10.3 `vehicleHeight` 按你的车应该怎么理解

你这台车给出的最高机体高度大约是：

- `0.643 m`

所以前面说的 `vehicleHeight = 1.5` 对你来说确实偏大。

更合适的第一版建议是：

- `vehicleHeight = 0.8`

为什么不是直接设成 `0.643`：

- 因为通常会给一点余量
- 这样可以覆盖机身上沿、姿态波动、传感器安装高度误差

如果后面你想更贴合实际，也可以试：

- `vehicleHeight = 0.75`

### 10.4 如果轮足高度会变，该按哪个尺寸设

这是最关键的实际问题。

建议原则：

- 如果你测试时会固定一种姿态
  - 就按那一种姿态的包络设
- 如果测试时机器人会在不同高度/不同展开状态切换
  - 就按“整个测试过程中可能出现的最大包络”设

也就是说：

- 只要你探索/导航时可能进入匍匐展开状态
  - `vehicleLength` 最好不要小于 `0.845`
- 只要你探索/导航时机身可能抬到站立高度
  - `vehicleHeight` 最好不要低于 `0.64` 附近

### 10.5 按你这台车，建议优先改哪 3 个参数

如果你下一步只想先做最关键的一轮对照，我建议先改这三个：

- `vehicleLength: 0.85`
- `vehicleWidth: 0.58`
- `vehicleHeight: 0.8`

然后再观察：

- 低矮障碍是否更稳定被绕开
- 前向穿障碍/贴障碍现象是否减少
- 局部轨迹是否变得更保守

### 10.6 一个很实用的经验

对你这类轮足可变形机器人，调参时最容易踩的坑就是：

- 机器人真实外形在变
- 但规划器一直以一个固定的小包络在算

这样就会出现：

- 视觉上已经“明显会碰”
- 但规划器还是认为“我能过去”

所以如果你只想先把系统调稳，优先做法不是先把所有阈值都改复杂，而是：

1. 先固定一种常用姿态
2. 先把 `vehicleLength / vehicleWidth / vehicleHeight` 设到接近真实包络
3. 再去调低矮障碍阈值
