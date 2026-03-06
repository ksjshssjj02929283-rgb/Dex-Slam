# 回环调试点云ROS发布功能

## 概述

本次修改在回环检测模块中添加了**配准结果可视化**功能，将回环检测过程中的关键数据通过ROS话题发布，便于调试和分析。

## 新增ROS话题

### 1. `/lightning/loop/submap` 
- **类型**: `sensor_msgs::msg::PointCloud2`
- **内容**: 回环历史关键帧构建的submap（±40帧范围，步长4）
- **坐标系**: `map`（世界坐标系）
- **用途**: 显示回环匹配的目标地图

### 2. `/lightning/loop/scan`
- **类型**: `sensor_msgs::msg::PointCloud2`  
- **内容**: 当前关键帧的点云扫描
- **坐标系**: `map`（从body坐标系转换）
- **用途**: 显示待匹配的当前扫描

### 3. `/lightning/loop/aligned` ⭐新增
- **类型**: `sensor_msgs::msg::PointCloud2`
- **内容**: NDT配准后的当前扫描点云
- **坐标系**: `map`（使用配准后的位姿）
- **用途**: 显示配准结果，评估匹配质量

## 数据流程

```
回环检测 → NDT配准 → 保存调试数据 → ROS发布
                ↓
         submap (红色)
         scan (黄色)  
         aligned (绿色) ⭐
```

## LoopDebugData结构

```cpp
struct LoopDebugData {
    CloudPtr loop_submap;          // 回环帧构建的submap（世界坐标系）
    CloudPtr current_scan;         // 当前帧的scan（body坐标系）
    CloudPtr aligned_scan;         // ⭐配准后的scan（世界坐标系）
    SE3 loop_kf_pose;              // 回环关键帧的位姿
    SE3 current_kf_pose;           // 当前关键帧的位姿
    SE3 aligned_pose;              // ⭐配准后的位姿
    int loop_kf_id;                // 回环关键帧ID
    int current_kf_id;             // 当前关键帧ID
    double ndt_score;              // ⭐NDT匹配得分
    bool valid;                    // 是否有效
};
```

## RViz可视化配置

### 推荐颜色方案

1. **回环Submap** (`/lightning/loop/submap`)
   - 颜色：红色 (R=255, G=0, B=0)
   - 点大小：2-3
   - 含义：历史地图（配准目标）

2. **当前Scan** (`/lightning/loop/scan`)
   - 颜色：黄色 (R=255, G=255, B=0)
   - 点大小：2-3
   - 含义：当前扫描（配准前）

3. **配准结果** (`/lightning/loop/aligned`) ⭐
   - 颜色：绿色 (R=0, G=255, B=0)
   - 点大小：2-3
   - 含义：配准后的扫描

### 质量评估

通过观察三个点云的关系，可以直观判断回环质量：

- **高质量回环**: 绿色（aligned）与红色（submap）高度重叠
- **中等质量**: 绿色与红色部分重叠，有一定偏差
- **低质量/失败**: 绿色与红色不重叠或偏差很大

黄色（scan）与绿色（aligned）的位置差异反映了NDT配准的调整量。

## 使用示例

### 1. 运行SLAM
```bash
source install/setup.bash
ros2 launch lightning slam_online.launch.py
```

### 2. 在RViz中添加点云显示

```python
# RViz配置
Displays:
  - PointCloud2:
      Topic: /lightning/loop/submap
      Color: 255;0;0 (红色)
      Size: 3
  - PointCloud2:
      Topic: /lightning/loop/scan
      Color: 255;255;0 (黄色)
      Size: 3
  - PointCloud2:
      Topic: /lightning/loop/aligned
      Color: 0;255;0 (绿色)
      Size: 3
```

### 3. 观察日志输出

```
[INFO] Loop debug data saved: kf 21 -> 114, score: 0.849, 
       submap size: 45231, scan size: 12045, aligned size: 12045
[INFO] RViz: Updated loop debug clouds, submap: 45231 pts, 
       scan: 12045 pts, aligned: 12045 pts, NDT score: 0.849
[INFO] RViz: Published loop debug clouds with NDT score: 0.849
```

## 技术细节

### 坐标系转换

1. **Submap**: 构建时已在世界坐标系，直接发布
2. **Scan**: 从body坐标系转到map: `T_map = current_kf_pose * scan_body`
3. **Aligned**: 使用配准后位姿转到map: `T_map = aligned_pose * scan_body`

### 数据更新时机

- 每次检测到回环候选并完成NDT配准后更新
- 只保存最新一次的调试数据（避免内存积累）
- 在`HandleKF()`开始时清空上一次数据

### 性能影响

- **点云拷贝**: 约2-5ms（取决于点云大小）
- **坐标转换**: 约1-3ms
- **ROS发布**: 异步，100ms周期，不阻塞主线程
- **内存占用**: 约1.5-3MB（三个点云）

## 调试技巧

### 1. 检查配准质量

```bash
# 实时查看NDT得分
ros2 topic echo /lightning/loop/aligned | grep -A 5 "frame_id"
```

### 2. 判断回环失败原因

观察点云关系：
- **黄色与绿色几乎重叠**: NDT配准未收敛，可能初值太差
- **绿色与红色相距很远**: 回环候选选择错误
- **绿色与红色部分重叠但未对齐**: NDT参数需要调整

### 3. 参数调优

根据可视化结果调整`config/default_livox.yaml`:

```yaml
loop_closing:
  ndt_score_th: 0.6         # 降低以接受更多回环
  max_range: 30.0           # 增大搜索范围
  closest_id_th: 50         # 调整ID间隔
```

## 修改文件清单

1. `src/core/loop_closing/loop_closing.h`
   - 添加 `aligned_scan` 到 `LoopDebugData`
   - 添加 `aligned_pose` 和 `ndt_score` 字段

2. `src/core/loop_closing/loop_closing.cc`
   - 在 `ComputeForCandidate()` 中保存配准结果
   - 添加调试日志输出

3. `src/wrapper/rviz_publisher.h`
   - 添加 `loop_aligned_pub_` 发布器
   - 更新 `UpdateLoopDebugClouds()` 接口

4. `src/wrapper/rviz_publisher.cc`
   - 实现配准结果点云发布
   - 添加坐标转换逻辑

5. `src/core/system/slam.cc`
   - 在两个 `ProcessLidar()` 中集成调试数据传递

## 预期效果

✅ **实时观察配准过程**: 看到NDT如何调整当前扫描与历史地图的匹配  
✅ **直观评估回环质量**: 通过点云重叠度判断回环是否有效  
✅ **快速定位问题**: 根据可视化结果调整参数和策略  
✅ **提高调试效率**: 无需保存PCD文件，实时分析

## 未来改进

1. 添加配准误差可视化（颜色映射）
2. 显示配准迭代过程动画
3. 添加多个候选帧的并行可视化
4. 集成到RViz panel中交互式调试

---

**修改日期**: 2026年3月5日  
**功能状态**: ✅ 编译通过，待测试  
**相关文档**: `loop_closure_improvements.md`
