# 地图对齐结构化环境：使用与实现原理

建图起点朝向任意，保存的地图往往相对建筑结构（墙体）旋转一个任意角度：
栅格边缘锯齿、导航坐标不直观、沿墙规划路径别扭。本功能从**全局线特征**
估计一个旋转角，把地图坐标系旋转到与主正交结构方向对齐。

提供三个入口，覆盖「建图时」与「已存地图」两类场景：

| 入口 | 作用对象 | 触发方式 |
|---|---|---|
| 在线建图节点 `cartographer_node` | 内存中位姿图（保存前） | 建图结束保存时自动 |
| 离线建图节点 `cartographer_offline_node` | 同上 | bag 回放建图保存时自动 |
| 离线工具 `align_map_tool` | 已保存的地图文件夹 | 命令行手动 |

> 本文路径约定（与 [reset_map_origin_tool.md](reset_map_origin_tool.md) 等文档一致）：
> `cartographer_ros/` 指本 ROS 包根目录 `mobile_slam/dexe_cartographer_ros/cartographer_ros/`，
> `cartographer/` 指 `mobile_slam/dexe_cartographer/cartographer/`。

---

## 一、使用

### 1.1 建图时自动对齐（在线 / 离线建图）

**配置**（`cartographer_ros/configuration_files/dexe_slam_2d.lua`，默认已开启）：

```lua
reset_map_origin_to_align_structure = true,      -- 总开关
reset_map_origin_min_line_length = 0.3,          -- 结构线段最小长度（米）
reset_map_origin_max_candidate_lines = 20,       -- 按长度取前 N 条候选
reset_map_origin_orthogonality_tolerance_deg = 10.0,  -- 正交容差（度）
reset_map_origin_min_secondary_support_ratio = 0.1,   -- 次/主方向支撑量最小比值
```

**在线建图**（服务流）：

```bash
ros2 launch cartographer_ros dexe_slam_2d.launch.py map_name:=<名称> [use_sim_time:=true]
ros2 service call /mapping_node/set_map_name nav_interfaces/srv/SetMapName "{map_name: <名称>}"
ros2 service call /mapping_node/start_mapping ...
# 实机移动或 ros2 bag play <bag> --clock
ros2 service call /mapping_node/finish_mapping ...   # 保存前自动应用对齐重置
```

**离线建图**（bag 回放）：`ros2 launch cartographer_ros dexe_slam_2d_offline.launch.py`，
保存前同样自动应用（重置逻辑在 `cartographer_ros::Node` 内，两条路径共享）。

**行为约定**：

- 重置发生在 `RunFinalOptimization()` 之后、`SerializeState()`/`SaveMap()` 之前，
  因此 pbstream、png、yaml、pcd 全部产物天然处于对齐后的坐标系；
- 估计不出可靠正交结构（开阔无墙环境等）时**静默跳过**，按原样保存，不报错；
- 幂等：同一次建图保存重试不会二次施加（`map_origin_reset_done_` 标志）；
- 确认是否生效：建图日志搜 `EstimateAlignmentYaw:`，会打印主/次方向、正交误差、
  RMS 残差与最终角度；保存后可用 `cartographer_pbstream check <map.pbstream>`
  看 `ALIGN-01`（PASS = 已对齐，残差 ≤ 1°）。

### 1.2 离线工具 align_map_tool（后处理已保存的地图）

对**历史地图**做同样的旋转对齐，就地更新全部地图文件。

```bash
colcon build --packages-select cartographer_ros

# 先只看估计角度，不改任何文件
ros2 run cartographer_ros align_map_tool --map_dir <地图文件夹> --dry_run

# 正式执行
ros2 run cartographer_ros align_map_tool --map_dir <地图文件夹>
```

**路径解析**：

- `--map_dir /home/dex/.dexe/data/maps/map_dynamic`（推荐）：自动探测
  `<map_dir>/slam/map.pbstream`（分层布局），否则 `<map_dir>/map.pbstream`；
- `--map_name <名称>`：相对 `~/.dexe/data/maps`（`--maps_root` 可改）解析，同样自动补 `slam/`。

**工具执行步骤**：

1. 读 pbstream，从 `global_static_submap.line_map_2d_list`（保存时已聚合 + 剔除的
   全局线特征）估计对齐角；
2. **备份**：原地图文件整体拷贝到 `<slam>/backups/backup_<YYYYMMDD_HHMMSS>/`，
   目录内子文件保持原名（`map.pbstream`、`map.png`、`map.yaml`、`map.pcd` …），
   可整目录直接恢复；
3. 对 pose graph 全部 submap/node/landmark 位姿、`trajectory_data`、
   `global_static_submap` 的 local_pose/点云/线特征施加绕原点纯旋转；
4. 重建栅格并重渲染 `map.png` + `map.yaml`（阈值沿用原 yaml）；
5. 变换 `map.pcd` / `map_highlight.pcd`（若存在）；
6. 原子写回 pbstream（先 `.tmp` 再 rename），并对变换后线特征**复检残余角**（应 ≈ 0）。

**常用 flag**：

| flag | 默认 | 说明 |
|---|---|---|
| `--dry_run` | false | 只打印估计角，不修改文件 |
| `--rebuild_grid_from_nodes` | **true** | 栅格从 trajectory node 点云重建干净栅格；`--norebuild_grid_from_nodes` 改为逐 cell 重栅格化原 grid（保留 free space，观感最接近保存时） |
| `--min_line_length` / `--max_candidate_lines` / `--orthogonality_tolerance_deg` / `--min_secondary_support_ratio` | 0.3 / 20 / 10.0 / 0.1 | 结构对齐阈值，与 lua 参数同义 |
| `--resolution` / `--occ_thresh` / `--free_thresh` | 0 | 0 = 沿用原 map.yaml 的值 |

**失败行为**：pbstream 无 `global_static_submap`、无线特征、或估计不出可靠正交
结构时，打印明确错误并退出，**不修改任何文件**（备份在估角之后才做）。

**实测示例**（`map_dynamic`，起点与墙体成 ~45°）：

```
=== Estimated alignment yaw: 0.779986 rad (44.6899 deg) ===
Backup directory: .../slam/backups/backup_20260910_211743
BuildFreshGridFromNodes: 100 nodes inserted ..., grid 1379x1369 @ res=0.05m
=== Post-transform residual yaw: 2.22045e-16 rad ... ===
```

执行后 `cartographer_pbstream check` 的 `ALIGN-01` 由
`WARN ... residual rotation 44.69 deg` 变为 `PASS ... 0.00 deg`。

**注意**：

- 工具依赖本分支的 pbstream 格式（`global_static_submap` + 线特征）；旧格式或
  建图时关了线特征的地图无法处理（报错信息会指向 `pbstream_check LINE-01` 排查方向）；
- 备份目录命名 `backup_<时间戳>/`（下划线）与建图节点自己的归档 `backup.<时间戳>/`
  （点）不同：节点的 `AtomicDirSwap` 备份轮换只识别后者，两者共存互不干扰。

---

## 二、实现原理

### 2.1 算法：EstimateAlignmentYaw

核心在 `cartographer/mapping/2d/structure_alignment.{h,cc}`，输入聚合后的全局
线地图 `LineMap2DList`，输出新原点朝向 `new_yaw ∈ [-45°, 45°]`（找不到可靠
正交结构返回 nullopt）。六步：

1. **长度门槛**：只保留 `length ≥ min_line_length` 的线段（墙通常较长；观测次数 /
   RANSAC 内点的基础剔除上游 `LineDataCulling` 已完成）；
2. **Top-N**：按长度降序取前 `max_candidate_lines` 条，防止某方向大量中等长度
   线段在投票时压过真正的长墙；
3. **粗找峰**：1° 长度加权角度直方图（定义域 [0, π)，线段方向无正负）。直方图
   **只用于定种子**，最终角度不用分箱值；
4. **主方向精化**：从峰值 bin 出发迭代收窄窗口（±10° → ±2°），每轮取窗口内线段的
   **长度加权倍角圆均值** `atan2(Σw·sin2θ, Σw·cos2θ)`。渐缩窗口能剔除漂移墙、
   斜向家具线等不对称尾部，得到主方向 θ1 与支撑量 w1（窗口内长度和）；
5. **次方向 + 正交校验**：屏蔽主峰 ±容差 的 bin 后找次峰，同法精化得 θ2、w2；
   校验 `|θ2 − θ1 − 90°| ≤ orthogonality_tolerance_deg` 且
   `w2 ≥ min_secondary_support_ratio · w1`，任一不满足 → nullopt（无可靠正交结构）；
6. **加权融合 + 折叠**：两族墙不严格正交时（δ = θ2 − θ1 − 90° ≠ 0，多由位姿图漂移
   引起），取支撑加权最小二乘折中

   ```
   J(r) = w1·(θ1 − r)² + w2·(θ2 − r − 90°)²   ⇒   r = θ1 + δ·w2/(w1+w2)
   ```

   支撑相等则两族各担 δ/2，次方向弱时自然退化为只用主方向；最后利用矩形结构的
   四重对称性把 r 折叠到 `new_yaw ≡ r (mod 90°)`、`|new_yaw| ≤ 45°`。

施加的变换是**绕地图原点的纯旋转**：`T_new_origin = R(new_yaw)`，所有全局数据乘
`T_inv = T_new_origin⁻¹`（原点位置不动，只改朝向）。位姿图约束是相对量，在全局
刚体变换下不变，因此**无需重优化**。

线段角度本身的质量前提：全局线段重拟合使用正交回归（TLS）而非 OLS，消除角度被
系统性拉向地图系水平方向的偏差（见提交 `db20adc`）。

### 2.2 程序结构：建图路径

```
FinishAllTrajectories → RunFinalOptimization
  → Node::ResetMapOriginToAlignStructure
      cartographer_ros/cartographer_ros/mapping/node.cc
      ├─ MapBuilderBridge::EstimateStructureAlignmentYaw
      │    └─ MapBuilder::EstimateStructureAlignmentYaw
      │         cartographer/mapping/map_builder.cc
      │         遍历位姿图节点：局部线特征 × global_pose → TrackLineFromMap 关联
      │         + 重拟合聚合为全局 LineMap2DList → LineDataCulling
      │         → EstimateAlignmentYaw(line_map, opts)
      │              cartographer/mapping/2d/structure_alignment.cc
      ├─ 构造 T_new_origin（绕原点纯旋转）
      └─ MapBuilder::ApplyOriginTransform(T⁻¹)
           └─ PoseGraph2D::ApplyTransformToAllPoses
                cartographer/mapping/internal/2d/pose_graph_2d.cc
                变换全部 node / submap / landmark 全局位姿；失效 global_static_submap 缓存
  → SerializeState    按新系从零重建 global_static_submap（栅格+点云+线特征）写入 pbstream
  → SaveMap           从 global_static_submap 渲染 png/yaml、导出 pcd
```

开关与阈值经 `MappingNodeOptions`（lua）→ `CreateStructureAlignmentOptions()` 传入；
`map_origin_reset_done_` 保证幂等。

### 2.3 程序结构：离线工具路径

`cartographer_ros/cartographer_ros/tools/align_map_main.cc`（共享 helper 在
`cartographer_ros/cartographer_ros/tools/map_transform_common.{h,cc}`，
`reset_map_origin_tool` 同用）：

```
LoadPbstream（header + pose_graph + traj_options + 全部 SerializedData 入内存）
  → 定位 global_static_submap，LineMap2DListFromProto 轻量重建运行时线地图
    （只取主线段、不重放观测历史 —— 与 pbstream_check ALIGN-01 同源）
  → EstimateAlignmentYaw → nullopt/无线特征即 FATAL 退出（不碰文件）；--dry_run 到此为止
  → BackupMapFiles：backups/backup_<ts>/ 整目录备份原文件
  → T_inv 施加到：pose_graph 位姿 / trajectory_data.fixed_frame_origin_in_map
    / TransformStaticSubmapNonGrid（local_pose + cloud + highlight_cloud + line）
  → 栅格：BuildFreshGridFromNodes（默认：node 过滤点云重插干净 ProbabilityGrid）
          或 RerasterizeGridProto（--norebuild...：旧 grid 逐 cell 旋转重写，保留概率）
  → RenderSingleStaticSubmap → WriteNavigationMap(png) + WriteYaml（与 node 端 SaveMap 一致）
  → TransformPcdFile（PCL 读写，点云在全局系）
  → WritePbstream（.tmp + rename 原子替换）
  → 变换后线特征重跑 EstimateAlignmentYaw 打印残余角（自检，期望 < 1°）
```

**与建图路径的关键差异**：

| | 建图路径 | 工具路径 |
|---|---|---|
| 线特征来源 | 从全部 node 重新聚合 + 跟踪 + culling | 直接读 pbstream 内已聚合好的 `line_map_2d_list` |
| 变换对象 | 内存位姿图对象 | proto 级遍历（pbstream 已落盘） |
| 产物生成 | 旋转后从零重建/渲染 | 变换已有数据：grid 重建或重栅格、png/yaml 重渲染、pcd 读写 |
| 失败语义 | 静默跳过、原样保存 | 报错退出、不碰文件（历史数据不可盲改） |
| 安全网 | 幂等标志 | 整目录备份 + 原子写 + 残余角自检 |

工具栅格默认走 node 点云重建而非重栅格化，是因为重栅格化在旋转后会有 cell 冲突
（first-wins + 取较高概率）与轴对齐近似误差；点云重建与建图路径
`GenerateFinalSubmap` 的产物口径一致。代价是 pbstream 不存原始 range_data，
misses 为空，free space 信息丢失——需要保留原观感时用 `--norebuild_grid_from_nodes`。

### 2.4 诊断工具

`cartographer_pbstream check <map.pbstream>`（`cartographer/io/internal/pbstream_check.cc`）
输出地图体检报告，其中与本功能直接相关：

- `LINE-01/02/03`：线特征数量、长度分布、加载后 culling 预测（无线特征 ⇒ 地图
  建自关闭线特征的旧二进制，见 LINE-01 提示）；
- `ALIGN-01`：用同一 `EstimateAlignmentYaw` 判断地图是否已对齐（残差 ≤ 1° 为 PASS），
  未对齐时提示用本功能重存。

---

## 三、源码位置

| 内容 | 路径 |
|---|---|
| 估角算法 | `cartographer/mapping/2d/structure_alignment.{h,cc}` |
| 建图路径接入 | `cartographer_ros/cartographer_ros/mapping/node.{h,cc}`、`map_builder_bridge.{h,cc}`、`cartographer/mapping/map_builder.cc`、`cartographer/mapping/internal/2d/pose_graph_2d.{h,cc}` |
| 开关与阈值 | `cartographer_ros/cartographer_ros/options/mapping_node_options.{h,cc}`、`cartographer_ros/configuration_files/dexe_slam_2d.lua` |
| 离线工具 | `cartographer_ros/cartographer_ros/tools/align_map_main.cc` |
| 共享 helper | `cartographer_ros/cartographer_ros/tools/map_transform_common.{h,cc}` |
| 诊断检查 | `cartographer/io/internal/pbstream_check.{h,cc}` |
| 算法单测 | `cartographer/mapping/2d/structure_alignment_test.cc` |

相关文档：[reset_map_origin_tool.md](reset_map_origin_tool.md)（同类离线地图变换工具）、
[../interfaces/reset_map_origin.md](../interfaces/reset_map_origin.md)（在线重置原点的服务接口）。
