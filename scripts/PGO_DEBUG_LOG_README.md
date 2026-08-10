# PGO 调试日志工具使用说明

## 📋 功能概述

本工具用于记录 PGO（Pose Graph Optimization）融合过程中的所有中间位姿，包括：

1. **DR输入** - IMU航位推算位姿（200Hz）
2. **LO输入** - 激光里程计位姿（10Hz）
3. **LidarLoc输入** - 激光定位位姿（5Hz）
4. **内插后位姿** - 给到优化器的DR/LO/LidarLoc值
5. **PGO优化后** - 因子图优化后的位姿
6. **外推后** - 用DR/LO外推到最新时刻的位姿
7. **平滑输出** - 最终输出的平滑位姿（200Hz）

## 🚀 使用方法

### 1. 编译

```bash
cd /home/dex/projects/dexe_mobile_slam3d
colcon build --packages-select lightning
source install/setup.bash
```

### 2. 运行定位程序

调试日志默认启用，输出到 `/tmp/pgo_debug/` 目录：

```bash
# 默认输出目录
ros2 launch lightning demo_loc_online.launch.py

# 自定义输出目录（通过环境变量）
PGO_DEBUG_LOG_DIR=/path/to/your/dir ros2 launch lightning demo_loc_online.launch.py
```

### 3. 查看日志文件

日志文件为 CSV 格式，带时间戳：

```bash
ls -lh /tmp/pgo_debug/
# 输出示例:
# pgo_debug_1738901234.csv
```

### 4. 可视化分析

```bash
# 使用最新的CSV文件生成PDF报告
python3 scripts/visualize_pgo_debug_relative.py

# 指定CSV文件
python3 scripts/visualize_pgo_debug_relative.py /tmp/pgo_debug/pgo_debug_1738901234.csv

# 指定输出PDF路径
python3 scripts/visualize_pgo_debug_relative.py -o my_analysis.pdf
```

## 📊 CSV 文件格式

```csv
# PGO Debug Log - Intermediate Poses
# Format: CSV
#
# Columns:
#   timestamp       - 时间戳 (秒)
#   stage           - 融合阶段
#   tx,ty,tz        - 平移 (米)
#   qx,qy,qz,qw     - 四元数
#   confidence      - LidarLoc置信度
#   smooth_factor   - 平滑因子
#   dr_tx,dr_ty,... - 内插后DR位姿
#   lo_tx,lo_ty,... - 内插后LO位姿
#   loc_tx,loc_ty,...- 内插后LidarLoc位姿
#   dt_extrap       - 外推时间差 (秒)
#
# Stages:
#   DR_INPUT        - DR输入位姿
#   LO_INPUT        - LO输入位姿
#   LIDARLOC_INPUT  - LidarLoc输入位姿
#   INTERPOLATED    - 内插后给优化器的位姿
#   OPTIMIZED       - PGO优化后位姿
#   EXTRAPOLATED    - 外推后位姿
#   SMOOTHED_OUTPUT - 平滑后最终输出

timestamp,stage,tx,ty,tz,qx,qy,qz,qw,confidence,smooth_factor,...
1738901234.567890,DR_INPUT,1.234,2.345,0.123,0.0,0.0,0.0,1.0,,,,
1738901234.567890,LO_INPUT,1.235,2.346,0.124,0.0,0.0,0.0,1.0,,,,
1738901234.567890,LIDARLOC_INPUT,1.236,2.347,0.125,0.0,0.0,0.0,1.0,0.85,,,
...
```

## 📈 可视化报告内容

生成的 PDF 报告包含以下图表：

### 1. 轨迹对比图
- **XY平面轨迹对比** - 各阶段位姿在XY平面的轨迹
- **Z轴高度对比** - 各阶段Z轴高度随时间变化
- **输入数据源XY对比** - DR/LO/LidarLoc输入对比
- **自适应平滑因子变化** - 平滑因子α的动态调整

### 2. 误差分析图
- **各阶段与平滑输出的位置误差** - 评估融合效果
- **LidarLoc定位置信度** - 置信度随时间变化
- **外推时间差分布** - 外推时间差统计

### 3. 统计信息图
- **各阶段数据量统计** - 柱状图
- **各阶段时间间隔分布** - 直方图
- **各阶段位置分布** - 箱线图
- **LidarLoc置信度分布** - 直方图

## 🔍 数据分析示例

### 使用 Python 直接读取

```python
import pandas as pd
import numpy as np

# 读取CSV
df = pd.read_csv('/tmp/pgo_debug/pgo_debug_1738901234.csv', comment='#')

# 查看各阶段数据量
print(df['stage'].value_counts())

# 提取特定阶段
lidarloc = df[df['stage'] == 'LIDARLOC_INPUT']
smoothed = df[df['stage'] == 'SMOOTHED_OUTPUT']

# 计算LidarLoc与平滑输出的误差
# (需要时间对齐)
```

### 使用命令行工具

```bash
# 查看文件头
head -50 /tmp/pgo_debug/pgo_debug_*.csv

# 统计各阶段数据量
grep -v '^#' /tmp/pgo_debug/pgo_debug_*.csv | cut -d',' -f2 | sort | uniq -c

# 查看低置信度记录
grep 'LIDARLOC_INPUT' /tmp/pgo_debug/pgo_debug_*.csv | awk -F',' '$10 < 0.5'
```

## ⚙️ 配置选项

### 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `PGO_DEBUG_LOG_DIR` | `/tmp/pgo_debug` | 日志输出目录 |

### 代码中修改

在 `pgo.cc` 中修改：

```cpp
// PGO::PGO() 构造函数中
std::string log_dir = "/your/custom/path";
debug_log_enabled_ = PGODebugLogger::Instance().Init(log_dir);
```

## 🎯 典型应用场景

### 1. 定位漂移分析
```bash
# 查看LidarLoc置信度低的时段
python3 scripts/visualize_pgo_debug.py
# 在PDF中查看"LidarLoc定位置信度"图表
```

### 2. 外推效果评估
```bash
# 查看外推时间差分布
# 在PDF中查看"外推时间差分布"图表
# 外推时间差越大，说明DR/LO滞后越严重
```

### 3. 平滑效果分析
```bash
# 查看平滑因子变化
# 在PDF中查看"自适应平滑因子变化"图表
# 平滑因子突然增大说明定位出现跳变
```

### 4. 多源数据对比
```bash
# 对比DR/LO/LidarLoc的差异
# 在PDF中查看"输入数据源XY对比"图表
# 差异大说明某个数据源可能异常
```

## 🐛 故障排查

### 问题1: 未生成日志文件

**原因**: 编译未包含新文件

**解决**:
```bash
cd /home/dex/projects/dexe_mobile_slam3d
colcon build --packages-select lightning --cmake-clean-cache
source install/setup.bash
```

### 问题2: 日志文件为空

**原因**: 定位未触发PGO流程

**解决**:
- 检查是否收到LidarLoc数据
- 检查DR/LO队列是否为空
- 查看ROS日志: `grep "PGO debug logging" ~/.ros/log/latest/*.log`

### 问题3: 可视化脚本报错

**原因**: 缺少Python依赖

**解决**:
```bash
pip3 install pandas numpy matplotlib
```

## 📝 日志记录位置

| 阶段 | 代码位置 | 函数 |
|------|----------|------|
| DR输入 | [pgo.cc:L177](file:///home/dex/projects/dexe_mobile_slam3d/src/core/localization/pose_graph/pgo.cc#L177) | `ProcessDR()` |
| LO输入 | [pgo.cc:L215](file:///home/dex/projects/dexe_mobile_slam3d/src/core/localization/pose_graph/pgo.cc#L215) | `ProcessLidarOdom()` |
| LidarLoc输入 | [pgo.cc:L317](file:///home/dex/projects/dexe_mobile_slam3d/src/core/localization/pose_graph/pgo.cc#L317) | `ProcessLidarLoc()` |
| 内插后位姿 | [pgo_impl.cc:L103](file:///home/dex/projects/dexe_mobile_slam3d/src/core/localization/pose_graph/pgo_impl.cc#L103) | `AddPGOFrame()` |
| 优化后位姿 | [pgo_impl.cc:L130](file:///home/dex/projects/dexe_mobile_slam3d/src/core/localization/pose_graph/pgo_impl.cc#L130) | `AddPGOFrame()` |
| 外推后位姿 | [pgo.cc:L65](file:///home/dex/projects/dexe_mobile_slam3d/src/core/localization/pose_graph/pgo.cc#L65) | `PubResult()` |
| 平滑输出 | [pgo.cc:L127](file:///home/dex/projects/dexe_mobile_slam3d/src/core/localization/pose_graph/pgo.cc#L127) | `PubResult()` |

## 📚 相关文件

- `src/core/localization/pose_graph/pgo_debug_logger.h` - 日志记录器头文件
- `src/core/localization/pose_graph/pgo_debug_logger.cc` - 日志记录器实现
- `src/core/localization/pose_graph/pgo.cc` - PGO融合逻辑（已集成日志）
- `src/core/localization/pose_graph/pgo_impl.cc` - PGO实现（已集成日志）
- `src/core/localization/pose_graph/smoother.h` - 平滑器（添加GetSmoothFactor）
- `scripts/visualize_pgo_debug.py` - 可视化脚本