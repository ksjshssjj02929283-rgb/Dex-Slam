# glass_detector

2D 激光玻璃检测 + scan 修正（一期 A 路径）

> 项目 [[17. 2D玻璃检测]] Phase 2 一期实现。详见知识库 `90. Projects/robot_engineer/17. 2D玻璃检测（20260514）/3. 玻璃检测一期方案_A单路双雷达独立输出.md`。

## 目标一句话

把"穿透玻璃的激光射线"修正为"打到玻璃前沿"，让 SLAM (Cartographer) 把玻璃当墙建到 `/map`。**走 perception 接口，不动 SLAM 内核**（DEC-001）。

## 数据流

```
/laser_scan/front  ──→ [glass_detector / front instance] ──→ /laser_scan/front_corrected ──→ Cartographer
/laser_scan/rear   ──→ [glass_detector / rear  instance] ──→ /laser_scan/rear_corrected  ──→ Cartographer
```

双雷达完全独立处理（独立 callback / 独立参数 namespace / 独立 publisher）。

## 算法

| 算法 | 来源 | 输入 | 优势 | 默认开启 |
|------|------|------|------|--------|
| **E1 几何不连续** | Tibebu 2021 风格两步滤波 | ranges | 不依赖 intensity，对所有 2D LiDAR 适用 | ✅ |
| **E2 强度异常** | Wang 2017 三参数 | intensities | 补 E1 闭合玻璃门盲区 | ✅ |

两种算法 OR-merge 到 `is_glass[i]`，可独立开关（参数 `enable_e1` / `enable_e2`）。

## 关键参数（双雷达独立 namespace）

详见 `config/glass_detector_params.yaml`，每路雷达可独立调：

```yaml
front:
  e1_std_threshold:     0.05   # m
  e1_range_jump:        0.30   # m
  e2_trigger_intensity: 8000   # ⚠️ 占位值，BlueSea 需现场标定
  e2_intensity_delta:   4000
  e2_profile_width:     5

rear: { ... 同上，可独立 override }
```

## 编译

```bash
cd ~/dexforce/0_slam/add_perception_2d
colcon build --packages-select glass_detector --symlink-install
source install/setup.bash
```

## 测试（明天上机前）

### 1. 单元测试

```bash
cd ~/dexforce/0_slam/add_perception_2d
colcon test --packages-select glass_detector
colcon test-result --verbose
```

### 2. ROS 节点起停

```bash
ros2 launch glass_detector glass_detector.launch.py
# 另起一个终端
ros2 topic list | grep corrected
# 应看到 /laser_scan/front_corrected 和 /laser_scan/rear_corrected
```

### 3. pass-through 验证（use_glass_detection:=false）

```bash
ros2 launch glass_detector glass_detector.launch.py use_glass_detection:=false
# 此时 corrected 话题 = 原 scan 透传，方便 A/B 对比
```

### 4. 实机 / bag 回放（验收门）

```bash
# 步骤 1：开雷达驱动（bluesea2_ros2_driver）
ros2 launch ... dual_udp_lidar.launch  # 现有 launch
# 步骤 2：开 glass_detector
ros2 launch glass_detector glass_detector.launch.py
# 步骤 3：开 Cartographer，订阅 /laser_scan/*_corrected
# 步骤 4：rviz2 查看 /map 玻璃位置是否成为墙
```

## Cartographer 集成

**只改 `.lua` 配置 + launch remap，不改 Cartographer C++ 源码**。

在 Cartographer launch 文件里把订阅话题改为：
- `/laser_scan/front` → `/laser_scan/front_corrected`
- `/laser_scan/rear`  → `/laser_scan/rear_corrected`

或保留原 remap 但启动一个 `topic_tools relay`。

## 阈值标定（明天的工作）

W1 PRO 用 BlueSea 双 2D 激光，intensity 量纲与 Wang 2017 原始 Hokuyo URG-04LX 不同。默认 trigger=8000 / delta=4000 是**占位值**，需要：

1. 录一段含玻璃的 bag（30-60s，走过若干玻璃门）
2. 写一个简单脚本读 bag 中 LaserScan 的 intensities → 算直方图
3. 找"墙的强度分布"和"玻璃前沿的强度峰"之间的合适阈值
4. 把数值更新到 `config/glass_detector_params.yaml`
5. 调试结果记录到知识库项目目录的 `4. 强度阈值标定记录.md`（Phase 2 一期 Step 7）

## 验收（来自 DEC-004 一期成功标准）

| # | 验收项 | 必过 |
|---|--------|------|
| 1 | bag 回放：`use_glass_detection:=false` 时 /map 玻璃位置呈灰带；切到 `true` 后呈墙（p_occ > 0.7） | ✅ |
| 2 | 单独关 `front.enable_e1` / `front.enable_e2`，后雷达不受影响 | ✅ |
| 3 | Cartographer trajectory 误差对比：ATE/RPE 差值 < 5%（修正不打坏 scan-matching） | ✅ |

## 二期范围（不在本包做）

- 内部维护 `glass_probability_grid` (log-odds 概率层)
- 发布 `/glass_probability_grid` (nav_msgs/OccupancyGrid)
- Nav2 `glass_layer` plugin (~300 行 CostmapLayer)
- E4 时序闪烁 / E5 穿透到稳定远墙
- 合并进 `perception_2d_ros2`

## 相关文档

- `90. Projects/robot_engineer/17. 2D玻璃检测（20260514）/1. 玻璃概率层详解.md` — 数学模型
- `90. Projects/robot_engineer/17. 2D玻璃检测（20260514）/2. 玻璃检测输出形式与 SLAM 集成方案.md` — 接口设计
- `90. Projects/robot_engineer/17. 2D玻璃检测（20260514）/3. 玻璃检测一期方案_A单路双雷达独立输出.md` — 本包实施蓝图
- `decisions.md` — DEC-001 / DEC-002 / DEC-003 / DEC-004
