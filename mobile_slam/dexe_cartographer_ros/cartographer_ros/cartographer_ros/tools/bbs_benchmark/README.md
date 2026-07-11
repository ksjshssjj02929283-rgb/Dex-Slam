# bbs_benchmark_tool — 2D 全局初始定位（BBS）离线基准测试

测量 `FastCorrelativeScanMatcher2D::MatchFullSubmap`（全局初始定位的分支定界搜索）
的各阶段耗时与搜索树规模。用于建立优化前基线，以及优化前后 A/B 对照。

**默认即线上对齐模式 + W1 双雷达**，无需配置参数：
- 自动从本包 ament share 目录加载 `dexe_localization_2d.lua` —— min_score(0.3)、
  branch_and_bound_depth(9)、预处理参数全部取自 lua（与 flag 无关）
- 默认 `--scan_topics /laser_scan/front,/laser_scan/rear`、
  `--tracking_frame base_footprint`

## 原理

- **地图**：读 `map.pbstream` 的 `global_static_submap` 字段，与线上
  `localization_updater` 加载路径完全一致。
- **算法**：`instrumented_fast_correlative_scan_matcher_2d.{h,cc}` 是生产
  matcher 的逐行复刻副本（仅引用公开头文件，生产库零改动），在各阶段插
  `steady_clock` 打点，并批量累加候选数统计（不进逐点内循环，扰动可忽略）。
- **预处理 = 线上同一份代码**：bag 模式把双雷达 LaserScan 组装成
  TimedPointCloudData（外参从 bag /tf_static 自动解析），喂给线上
  `DataPreprocessor::ProcessMultipleSensorData`（即 pose_estimator_2d →
  pose_initializer 的真实链路），取 merged_filtered_cloud 作为匹配输入。
  离线约束：强制 use_odometry=false（不去畸变）、enable_feature_detection=false
  （line 特征只服务 ceres 阶段，不影响 BBS 输入）。
- **scan 来源**：
  - `synth`：地图自由空间随机撒真值位姿 + ray-cast 合成 scan（过线上同款
    AdaptiveVoxelFilter），批量统计成功率与位姿误差。
  - `bag`：双雷达实机数据回放；`--bag_random_samples N` 从整个时间轴无放回
    随机抽 N 帧（seed 可复现），rear 与 front 按时间最近帧配对（容差 0.1s）。

## 用法

```bash
source /opt/ros/humble/setup.bash && source <工作区>/install/setup.bash

# 合成模式（带真值）
ros2 run cartographer_ros bbs_benchmark_tool \
  --map_name <地图名> --mode synth --num_poses 15

# bag 模式（W1 双雷达实机数据）
ros2 run cartographer_ros bbs_benchmark_tool \
  --map_name <地图名> --mode bag \
  --bag_filename <bag目录> --bag_random_samples 15

# 全部 flag
ros2 run cartographer_ros bbs_benchmark_tool --help
```

bag 没录 `/tf_static` 时手动指定外参（米/度，按话题顺序）：
`--scan_extrinsics "0.3,0,0,0,0,0;-0.3,0,0,0,0,180"`

## 输出

每个模式一个报告文件，写到 `~/.data/localization/bbs_baseline/`
（`--output_dir` 可覆盖）：

```
<地图名>_<时间戳>_synth.txt
<地图名>_<时间戳>_bag.txt
```

文件结构：头部（地图/参数/金字塔构建耗时）→ 总结（各阶段耗时占比、各深度层
候选数、匹配率/成功率/误差）→ 末尾附每次测试的明细 CSV（25 列）。
stdout 同步打印头部与总结。

## A/B 对照约定

优化版 matcher 接入本工具时：同地图 + 同 `--seed`（位姿/抽帧可复现）+ 同参数；
耗时看 mean 与 p95 双口径；成功率/误差必须不退化。
